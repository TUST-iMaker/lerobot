"""Workflow orchestration for SO-101 calibration, teleop, teaching, data collection, and cameras."""

from __future__ import annotations

import asyncio
import json
import math
import re
import shutil
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from app.backend.config import StudioConfig
from app.backend.hardware import CAMERA_IO_LOCK, HardwareManager
from app.backend.process_manager import ProcessRegistry
from app.backend.realtime_bridge import SO101RealBridge, SyncDirection
from app.backend.simulation import (
    JOINT_NAMES,
    Arm101Simulator,
    compute_fk_details,
    default_joints,
    deg_to_encoder,
    encoder_to_deg,
    solve_ik_target,
    solve_ik_target_with_status,
    validate_joint_dict,
)


WorkflowName = Literal["idle", "calibrate", "teleop", "teaching_replay", "collect", "demo_routine"]


class WorkflowConflictError(Exception):
    """Raised when an operation conflicts with the active workflow or empty state."""

    def __init__(self, detail: str, code: str = "WORKFLOW_CONFLICT") -> None:
        super().__init__(detail)
        self.detail = detail
        self.code = code


class _PersistentCameraHub:
    """Maintains persistent low-latency OpenCV VideoCapture handles for real-time MJPEG streaming.

    - Opens each physical camera once (MJPG + CAP_PROP_BUFFERSIZE=1) instead of per-frame open/close.
    - Auto-releases idle handles after `idle_timeout_s` or immediately on `release_all()`.
    """

    def __init__(self, idle_timeout_s: float = 3.0) -> None:
        self._idle_timeout_s = idle_timeout_s
        self._lock = threading.Lock()
        self._workers: dict[str, dict[str, Any]] = {}

    def get_frame(self, index_or_path: int | str) -> bytes | None:
        key = str(index_or_path)
        now = time.monotonic()
        with self._lock:
            worker = self._workers.get(key)
            if worker is None or not worker["thread"].is_alive():
                stop_ev = threading.Event()
                ready_ev = threading.Event()
                worker = {
                    "key": key,
                    "index_or_path": index_or_path,
                    "stop_ev": stop_ev,
                    "ready_ev": ready_ev,
                    "latest_jpeg": None,
                    "latest_rgb": None,
                    "last_access": now,
                    "failed": False,
                }
                t = threading.Thread(
                    target=self._capture_loop,
                    args=(worker,),
                    name=f"cam-stream-{key}",
                    daemon=True,
                )
                worker["thread"] = t
                self._workers[key] = worker
                t.start()
            else:
                worker["last_access"] = now

        if worker["latest_jpeg"] is None and not worker["failed"]:
            worker["ready_ev"].wait(timeout=1.5)
        return worker["latest_jpeg"]

    def get_latest_rgb(self, index_or_path: int | str) -> Any:
        self.get_frame(index_or_path)
        with self._lock:
            worker = self._workers.get(str(index_or_path))
            return worker.get("latest_rgb") if worker else None

    def _capture_loop(self, worker: dict[str, Any]) -> None:
        cap = None
        try:
            import cv2

            backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
            with CAMERA_IO_LOCK:
                cap = cv2.VideoCapture(worker["index_or_path"], backend)
                if not cap.isOpened():
                    worker["failed"] = True
                    worker["ready_ev"].set()
                    return
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                except Exception:
                    pass
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass

            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 78]
            while not worker["stop_ev"].is_set():
                if time.monotonic() - worker["last_access"] > self._idle_timeout_s:
                    break
                ok, frame = cap.read()
                if ok and frame is not None:
                    try:
                        if frame.shape[0] != 480 or frame.shape[1] != 640:
                            frame_std = cv2.resize(frame, (640, 480))
                        else:
                            frame_std = frame
                        worker["latest_rgb"] = cv2.cvtColor(frame_std, cv2.COLOR_BGR2RGB)
                    except Exception:
                        pass
                    enc_ok, buf = cv2.imencode(".jpg", frame, encode_params)
                    if enc_ok:
                        worker["latest_jpeg"] = bytes(buf)
                        worker["ready_ev"].set()
                else:
                    time.sleep(0.03)
                time.sleep(0.033)  # ~25-30 FPS
        except Exception:
            worker["failed"] = True
            worker["ready_ev"].set()
        finally:
            worker["ready_ev"].set()
            if cap is not None:
                with CAMERA_IO_LOCK:
                    try:
                        cap.release()
                    except Exception:
                        pass
            with self._lock:
                if self._workers.get(worker["key"]) is worker:
                    self._workers.pop(worker["key"], None)

    def release_all(self) -> None:
        with self._lock:
            workers = list(self._workers.values())
            self._workers.clear()
        for w in workers:
            w["stop_ev"].set()
        for w in workers:
            t = w.get("thread")
            if t is not None and t.is_alive():
                t.join(timeout=1.5)


class WorkflowManager:
    """Coordinates real-mode lerobot CLI processes and dry-run 6-DOF workflows."""

    def __init__(
        self,
        config: StudioConfig,
        hardware: HardwareManager,
        simulator: Arm101Simulator,
        process_registry: ProcessRegistry,
    ) -> None:
        self.config = config
        self.hardware = hardware
        self.simulator = simulator
        self.process_registry = process_registry
        self.bridge = SO101RealBridge(config, simulator, on_motion=self._notify_motion)
        self._camera_hub = _PersistentCameraHub(idle_timeout_s=15.0)

        self.active_workflow: WorkflowName = "idle"
        self.state: str = "idle"
        self.current_episode: int = 0
        self.target_episodes: int = 0
        self.current_step: int = 0
        self.last_error: str | None = None
        self.logs: deque[str] = deque(maxlen=250)

        self._task: asyncio.Task[None] | None = None
        self._active_proc_id: str | None = None
        self._collect_ctx: dict[str, Any] | None = None
        self._linear_teach_ctx: dict[str, Any] | None = None
        self._collect_stage: str = "idle"
        self._collect_stage_text: str = ""
        self._collect_action_ev: asyncio.Event | None = None
        self._collect_action_cmd: str | None = None
        self._dry_cal_queue: asyncio.Queue[str] | None = None
        self._cal_state: dict[str, Any] = self._empty_cal_state()
        self._waypoints: list[dict[str, Any]] = []
        self.vla_chunk: list[dict[str, Any]] = []
        self.overlay_badge: str = ""
        self.overlay_seq: int = 1
        self.last_motion_ts: float = 0.0
        self.motion_event: asyncio.Event = asyncio.Event()
        self._load_waypoints()

    @staticmethod
    def _empty_cal_state() -> dict[str, Any]:
        return {
            "active": False,
            "target": "follower",
            "port": "",
            "arm_id": "",
            "mode": "dry_run",
            "stage": "idle",
            "prompt": "",
            "waiting_for_input": False,
            "allow_recalibrate": False,
            "live_table": {},
            "saved_path": None,
        }

    def get_calibration_snapshot(self) -> dict[str, Any]:
        table_rows = [
            self._cal_state["live_table"][name]
            for name in JOINT_NAMES
            if name in self._cal_state["live_table"]
        ]
        return {
            **self._cal_state,
            "active": self.active_workflow == "calibrate",
            "live_table": table_rows,
        }

    @property
    def waypoints_path(self) -> Path:
        return self.config.dry_run_dir / "waypoints.json"

    def log(self, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        self.logs.append(f"[{ts}] {message}")

    def _load_waypoints(self) -> None:
        p = self.waypoints_path
        if p.exists():
            try:
                data = json.loads(p.read_text(encoding="utf-8"))
                if isinstance(data, list):
                    self._waypoints = data
            except Exception:
                self._waypoints = []

    def _save_waypoints(self) -> None:
        self.config.dry_run_dir.mkdir(parents=True, exist_ok=True)
        self.waypoints_path.write_text(
            json.dumps(self._waypoints, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    async def _ensure_idle(self) -> None:
        if self.active_workflow != "idle":
            prev = self.active_workflow
            self._stop_active_process()
            await self._cancel_background_task()
            if prev == "collect":
                self._flush_dry_run_episode()
                self._collect_ctx = None
            self.active_workflow = "idle"
            self.state = "idle"
            self.log(f"Preempted active workflow '{prev}' to start new workflow.")

    def _sync_process_state(self) -> None:
        if self._active_proc_id is not None and not self.process_registry.is_running(
            self._active_proc_id
        ):
            managed = self.process_registry.get(self._active_proc_id)
            rc = managed.returncode if managed is not None else None
            if rc not in (None, 0):
                self.last_error = f"Process '{self._active_proc_id}' exited with code {rc}."
                self.log(self.last_error)
            self._active_proc_id = None
            if self.active_workflow != "idle":
                if self.active_workflow == "calibrate":
                    if rc == 0:
                        self._cal_state["stage"] = "completed"
                        self._cal_state["waiting_for_input"] = False
                        self._cal_state["prompt"] = "标定已完成并写入电机与本地配置文件。"
                    elif rc is not None:
                        self._cal_state["stage"] = "error"
                        self._cal_state["waiting_for_input"] = False
                        self._cal_state["prompt"] = self.last_error or f"标定异常退出 (code={rc})。"
                self.active_workflow = "idle"
                self.state = "idle"

    def get_status(self) -> dict[str, Any]:
        self._sync_process_state()
        return {
            "mode": self.hardware.active_mode,
            "active_workflow": self.active_workflow,
            "state": self.state,
            "teleop_active": self.active_workflow == "teleop",
            "calibrate_active": self.active_workflow == "calibrate",
            "teaching_replaying": self.active_workflow == "teaching_replay",
            "collect_active": self.active_workflow == "collect",
            "collect_stage": self._collect_stage,
            "collect_stage_text": self._collect_stage_text,
            "current_episode": self.current_episode,
            "target_episodes": self.target_episodes,
            "current_step": self.current_step,
            "waypoint_count": len(self._waypoints),
            "calibration": self.get_calibration_snapshot(),
            "last_error": self.last_error,
            "logs": list(self.logs),
        }

    def _notify_motion(self) -> None:
        self.last_motion_ts = time.monotonic()
        try:
            self.motion_event.set()
        except Exception:
            pass

    def get_overlay_frame(self) -> dict[str, Any]:
        return {
            "type": "overlay_update",
            "overlay_seq": self.overlay_seq,
            "waypoints": list(self._waypoints),
            "vla_chunk": list(self.vla_chunk),
            "overlay_badge": self.overlay_badge,
        }

    def set_vla_chunk(self, chunk: list[dict[str, Any]], badge: str = "") -> None:
        self.vla_chunk = list(chunk)
        self.overlay_badge = badge
        self.overlay_seq += 1
        self._notify_motion()

    def _arbitrate_manual_write(self, *, allow_dry_calibrate: bool = False) -> None:
        self._sync_process_state()
        if self.active_workflow == "calibrate":
            if allow_dry_calibrate and self.hardware.active_mode == "dry_run":
                return
            raise WorkflowConflictError(
                "当前正在执行标定任务（关节空间模式），已锁定笛卡尔/IK 控制，请使用标定滑块或先停止标定。",
                code="WORKFLOW_BUSY",
            )
        if self.active_workflow == "collect":
            raise WorkflowConflictError(
                "当前正在执行 'collect' 数据采集任务，手动控臂已锁定，请先停止该任务。",
                code="WORKFLOW_BUSY",
            )
        if self.active_workflow in ("teaching_replay", "demo_routine"):
            if self._task is not None and not self._task.done():
                self._task.cancel()
            self._task = None
            self.active_workflow = "idle"
            self.state = "idle"
            self.overlay_badge = ""

    def _record_dry_cal_live_table(self, joints: dict[str, float]) -> None:
        """Update min/pos/max encoder table during dry_run calibration Step 2 (recording_range)."""
        if (
            self.active_workflow != "calibrate"
            or self.hardware.active_mode != "dry_run"
            or self._cal_state.get("stage") != "recording_range"
        ):
            return
        for jname in JOINT_NAMES:
            pos = int(deg_to_encoder(jname, float(joints.get(jname, 0.0))))
            prev = self._cal_state["live_table"].get(jname)
            vmin = min(int(prev["min"]), pos) if prev else pos
            vmax = max(int(prev["max"]), pos) if prev else pos
            self._cal_state["live_table"][jname] = {
                "name": jname,
                "min": str(vmin),
                "pos": str(pos),
                "max": str(vmax),
            }

    def get_telemetry_frame(self) -> dict[str, Any]:
        self._sync_process_state()
        leader, follower, sim_steps = self.simulator.snapshot()
        leader_fk = compute_fk_details(leader)
        follower_fk = compute_fk_details(follower)
        bridge_st = self.bridge.status()
        hw_ready = bool(self.hardware.last_probe.get("hardware_ready", False))
        return {
            "type": "telemetry",
            "overlay_seq": self.overlay_seq,
            "overlay_badge": self.overlay_badge,
            "timestamp": round(time.time(), 3),
            "mode": self.hardware.active_mode,
            "active_workflow": self.active_workflow,
            "leader_joints": leader,
            "follower_joints": follower,
            "leader_fk": leader_fk,
            "follower_fk": follower_fk,
            "linked_arms": self.simulator.linked_arms,
            "safety": {
                "estop_latched": self.simulator.estop_latched,
                "torque_enabled": self.simulator.torque_enabled,
                "max_step_deg": self.simulator.max_step_deg,
                "sync_direction": bridge_st["direction"],
            },
            "connection_status": {
                "hardware_ready": hw_ready,
                "follower_port": self.config.follower_port,
                "leader_port": self.config.leader_port,
                "follower_connected": bridge_st["connected_follower"] or (self.hardware.active_mode == "real" and hw_ready),
                "leader_connected": bridge_st["connected_leader"] or (self.hardware.active_mode == "real" and hw_ready),
            },
            "calibration": self.get_calibration_snapshot(),
            "workflow_status": {
                "state": self.state,
                "current_episode": self.current_episode,
                "target_episodes": self.target_episodes,
                "current_step": max(self.current_step, sim_steps),
                "waypoint_count": len(self._waypoints),
            },
            "log_tail": list(self.logs)[-25:],
        }

    async def _cancel_background_task(self) -> None:
        if self._task is not None and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        self._task = None

    def _stop_active_process(self) -> None:
        if self._active_proc_id is not None:
            self.process_registry.terminate(self._active_proc_id)
            self._active_proc_id = None

    # ------------------------------------------------------------------
    # Command builders for real-mode lerobot v0.7.1 CLIs
    # ------------------------------------------------------------------
    def build_cameras_dict(self) -> dict[str, dict[str, Any]]:
        cams: dict[str, dict[str, Any]] = {}
        for name, raw in (
            ("front", self.config.front_camera_id),
            ("wrist", self.config.wrist_camera_id),
        ):
            cleaned = (raw or "").strip()
            if not cleaned or cleaned.lower() == "none":
                continue
            idx_or_path: int | str = int(cleaned) if cleaned.isdigit() else cleaned
            cams[name] = {
                "type": "opencv",
                "index_or_path": idx_or_path,
                "width": 640,
                "height": 480,
                "fps": 30,
            }
        return cams

    def build_calibrate_cmd(
        self,
        *,
        target: Literal["follower", "leader"],
        port: str | None = None,
        arm_id: str | None = None,
        calibration_dir: str | None = None,
    ) -> list[str]:
        """Build interactive calibrate CLI args (strictly ONE of --robot.* or --teleop.*)."""
        if target == "follower":
            resolved_port = port or self.config.follower_port
            resolved_id = arm_id or self.config.follower_id
            resolved_dir = calibration_dir or str(
                self.config.calibration_dir / "robots" / "so_follower"
            )
            return [
                sys.executable,
                "-u",
                "-m",
                "app.backend.calibrate_runner",
                "--robot.type=so101_follower",
                f"--robot.port={resolved_port}",
                f"--robot.id={resolved_id}",
                f"--robot.calibration_dir={resolved_dir}",
            ]
        resolved_port = port or self.config.leader_port
        resolved_id = arm_id or self.config.leader_id
        resolved_dir = calibration_dir or str(
            self.config.calibration_dir / "teleoperators" / "so_leader"
        )
        return [
            sys.executable,
            "-u",
            "-m",
            "app.backend.calibrate_runner",
            "--teleop.type=so101_leader",
            f"--teleop.port={resolved_port}",
            f"--teleop.id={resolved_id}",
            f"--teleop.calibration_dir={resolved_dir}",
        ]

    def build_teleop_cmd(
        self,
        *,
        fps: int = 60,
        follower_port: str | None = None,
        leader_port: str | None = None,
        display_data: bool = False,
        use_cameras: bool = False,
    ) -> list[str]:
        f_port = follower_port or self.config.follower_port
        l_port = leader_port or self.config.leader_port
        f_cal = str(self.config.calibration_dir / "robots" / "so_follower")
        l_cal = str(self.config.calibration_dir / "teleoperators" / "so_leader")
        cmd = [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_teleoperate",
            "--robot.type=so101_follower",
            f"--robot.port={f_port}",
            f"--robot.id={self.config.follower_id}",
            f"--robot.calibration_dir={f_cal}",
            "--robot.use_degrees=true",
            "--teleop.type=so101_leader",
            f"--teleop.port={l_port}",
            f"--teleop.id={self.config.leader_id}",
            f"--teleop.calibration_dir={l_cal}",
            "--teleop.use_degrees=true",
            f"--fps={fps}",
            f"--display_data={'true' if display_data else 'false'}",
            "--display_mode=rerun",
        ]
        if use_cameras or display_data:
            cams = self.build_cameras_dict()
            if cams:
                cmd.append(f"--robot.cameras={json.dumps(cams, separators=(',', ':'))}")
        return cmd

    def build_record_cmd(
        self,
        *,
        dataset_name: str,
        single_task: str,
        num_episodes: int,
        episode_time_s: float,
        reset_time_s: float,
        fps: int,
        push_to_hub: bool = False,
    ) -> list[str]:
        f_cal = str(self.config.calibration_dir / "robots" / "so_follower")
        l_cal = str(self.config.calibration_dir / "teleoperators" / "so_leader")
        ds_root = self.config.datasets_dir / dataset_name
        cmd = [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_record",
            "--robot.type=so101_follower",
            f"--robot.port={self.config.follower_port}",
            f"--robot.id={self.config.follower_id}",
            f"--robot.calibration_dir={f_cal}",
            "--teleop.type=so101_leader",
            f"--teleop.port={self.config.leader_port}",
            f"--teleop.id={self.config.leader_id}",
            f"--teleop.calibration_dir={l_cal}",
            f"--dataset.repo_id=local/{dataset_name}",
            f"--dataset.root={ds_root}",
            f"--dataset.single_task={single_task}",
            f"--dataset.num_episodes={num_episodes}",
            f"--dataset.episode_time_s={episode_time_s}",
            f"--dataset.reset_time_s={reset_time_s}",
            f"--dataset.fps={fps}",
            f"--dataset.push_to_hub={'true' if push_to_hub else 'false'}",
            "--dataset.streaming_encoding=true",
            "--play_sounds=false",
            "--display_data=false",
        ]
        if ds_root.exists():
            cmd.append("--resume=true")
        cams = self.build_cameras_dict()
        if cams:
            cmd.append(f"--robot.cameras={json.dumps(cams, separators=(',', ':'))}")
        return cmd

    def build_replay_cmd(
        self,
        *,
        dataset_name: str,
        episode: int = 0,
        fps: int = 30,
    ) -> list[str]:
        f_cal = str(self.config.calibration_dir / "robots" / "so_follower")
        ds_root = self.config.datasets_dir / dataset_name
        return [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_replay",
            "--robot.type=so101_follower",
            f"--robot.port={self.config.follower_port}",
            f"--robot.id={self.config.follower_id}",
            f"--robot.calibration_dir={f_cal}",
            f"--dataset.repo_id=local/{dataset_name}",
            f"--dataset.root={ds_root}",
            f"--dataset.episode={episode}",
            f"--dataset.fps={fps}",
            "--play_sounds=false",
        ]

    # ------------------------------------------------------------------
    # Calibrate
    # ------------------------------------------------------------------
    _CAL_TABLE_ROW_RE = re.compile(
        r"^\s*(shoulder_pan|shoulder_lift|elbow_flex|wrist_flex|wrist_roll|gripper)\s*\|\s*"
        r"(-?\d+(?:\.\d+)?)\s*\|\s*(-?\d+(?:\.\d+)?)\s*\|\s*(-?\d+(?:\.\d+)?)\s*$"
    )

    def _on_calibrate_line(self, line: str) -> None:
        m = self._CAL_TABLE_ROW_RE.match(line)
        if m:
            motor, vmin, vpos, vmax = m.groups()
            self._cal_state["live_table"][motor] = {
                "name": motor,
                "min": vmin,
                "pos": vpos,
                "max": vmax,
            }
            try:
                deg_val = encoder_to_deg(motor, float(vpos))
                target = self._cal_state.get("target", "follower")
                if target == "leader":
                    self.simulator.set_leader_joints({motor: deg_val}, step_immediately=False)
                else:
                    self.simulator.set_follower_joints({motor: deg_val})
            except Exception:
                pass
            return

        self.log(line)
        low_line = line.lower()
        if "[CAL_STAGE]" in line:
            stage_info = line.split("[CAL_STAGE]", 1)[1].strip()
            if stage_info.startswith("connecting_bus"):
                self._cal_state["prompt"] = (
                    f"正在握手并配置 {self._cal_state.get('target', 'arm')} "
                    f"({self._cal_state.get('port', '')}) 总线，请稍候..."
                )
            elif stage_info.startswith("torque_disabled"):
                self._cal_state["prompt"] = "已连接总线并安全释放舵机扭矩（机械臂已松弛），正在进入标定引导..."
        elif "[CAL_PROMPT]" in line:
            raw_prompt = line.split("[CAL_PROMPT]", 1)[1].strip()
            low = raw_prompt.lower()
            self._cal_state["waiting_for_input"] = True
            if "calibration file" in low or "type 'c'" in low:
                self._cal_state["stage"] = "existing_file_prompt"
                self._cal_state["allow_recalibrate"] = True
                self._cal_state["prompt"] = (
                    "检测到已有标定文件：点击【确认 / 下一步 (Enter)】直接写入电机，"
                    "或点击【重新标定 (c)】重新采集。"
                )
            elif "middle" in low:
                self._cal_state["stage"] = "waiting_middle"
                self._cal_state["allow_recalibrate"] = False
                self._cal_state["prompt"] = (
                    "第 1 步（中位归零）：请用手将机械臂摆成官方中位——底座居中、大臂垂直向上、小臂与夹爪水平向前平直（与灰虚线一致），"
                    "然后点击【确认 / 下一步 (Enter)】。"
                )
            else:
                self._cal_state["stage"] = "waiting_input"
                self._cal_state["allow_recalibrate"] = False
                self._cal_state["prompt"] = raw_prompt
        elif "move the joints to the middle" in low_line or "middle of their range" in low_line:
            self._cal_state["stage"] = "waiting_middle"
            self._cal_state["waiting_for_input"] = True
            self._cal_state["allow_recalibrate"] = False
            self._cal_state["prompt"] = (
                "第 1 步（中位归零）：请用手将机械臂所有关节摆到活动范围正中间，"
                "然后点击【确认 / 下一步 (Enter)】。"
            )
        elif "press enter to use provided calibration file" in low_line:
            self._cal_state["stage"] = "existing_file_prompt"
            self._cal_state["waiting_for_input"] = True
            self._cal_state["allow_recalibrate"] = True
            self._cal_state["prompt"] = (
                "检测到已有标定文件：点击【确认 / 下一步 (Enter)】直接写入电机，"
                "或点击【重新标定 (c)】重新采集。"
            )
        elif "Recording positions" in line or "Press ENTER to stop" in line:
            self._cal_state["stage"] = "recording_range"
            self._cal_state["waiting_for_input"] = True
            self._cal_state["allow_recalibrate"] = False
            self._cal_state["prompt"] = (
                "第 2 步（行程录制）：请逐个掰动所有关节到最小与最大极限位置，"
                "确认下方各关节 MIN/MAX 均有变化后，点击【确认 / 下一步 (Enter)】完成保存。"
            )
        elif "Calibration saved to" in line:
            self._cal_state["stage"] = "completed"
            self._cal_state["waiting_for_input"] = False
            self._cal_state["saved_path"] = line.split("Calibration saved to", 1)[1].strip()
            self._cal_state["prompt"] = f"标定成功！已保存到 {self._cal_state['saved_path']}"
        elif any(
            err_tok in line
            for err_tok in ("ConnectionError:", "OSError:", "TimeoutError:", "RuntimeError:", "ValueError:")
        ):
            self.last_error = line
            if self._cal_state.get("stage") == "connecting":
                self._cal_state["prompt"] = f"连接或配置失败：{line}"

    async def start_calibrate(
        self,
        *,
        target: Literal["follower", "leader"] = "follower",
        port: str | None = None,
        arm_id: str | None = None,
        calibration_dir: str | None = None,
    ) -> dict[str, Any]:
        self._sync_process_state()
        await self._ensure_idle()
        self.last_error = None
        resolved_port = port or (
            self.config.follower_port if target == "follower" else self.config.leader_port
        )
        resolved_id = arm_id or (
            self.config.follower_id if target == "follower" else self.config.leader_id
        )
        cmd = self.build_calibrate_cmd(
            target=target,
            port=resolved_port,
            arm_id=resolved_id,
            calibration_dir=calibration_dir,
        )
        self._cal_state = {
            "active": True,
            "target": target,
            "port": resolved_port,
            "arm_id": resolved_id,
            "mode": self.hardware.active_mode,
            "stage": "connecting",
            "prompt": f"正在连接 {target} ({resolved_port})，请稍候...",
            "waiting_for_input": False,
            "allow_recalibrate": False,
            "live_table": {},
            "saved_path": None,
        }
        self._linear_teach_ctx: dict[str, Any] | None = None

        if self.hardware.active_mode == "real":
            self.bridge.disconnect(return_to_rest=False, keep_torque=False)
            proc_id = "workflow_calibrate"
            self.process_registry.spawn(
                proc_id,
                "lerobot_calibrate",
                cmd,
                cwd=self.config.workspace_root,
                on_line=self._on_calibrate_line,
                stdin_pipe=True,
            )
            self._active_proc_id = proc_id
            self.active_workflow = "calibrate"
            self.state = "running"
            self.log(f"Started real calibration for {target} on {resolved_port}.")
            return {
                "status": "started",
                "workflow": "calibrate",
                "mode": "real",
                "target": target,
                "port": resolved_port,
                "command": cmd,
            }

        self.active_workflow = "calibrate"
        self.state = "running"
        self.current_step = 1
        self._cal_state["stage"] = "waiting_middle"
        self._cal_state["waiting_for_input"] = True
        self._cal_state["prompt"] = (
            "【模拟标定 1/2】请通过下方关节滑块（或一键对齐中位）将蓝臂对齐灰虚线中位，然后点击【确认 / 下一步】。"
        )
        self._dry_cal_queue = asyncio.Queue()
        cal_queue = self._dry_cal_queue
        self.log(f"Started dry_run calibration for {target} ({resolved_id}).")

        async def _sim_calibrate() -> None:
            try:
                # Step 1 (waiting_middle): show an unaligned initial pose against the translucent URDF homing ghost
                unaligned_pose = {
                    "shoulder_pan": -16.0,
                    "shoulder_lift": 22.0,
                    "elbow_flex": -25.0,
                    "wrist_flex": 18.0,
                    "wrist_roll": 0.0,
                    "gripper": 35.0,
                }
                if target == "leader":
                    self.simulator.set_leader_joints(unaligned_pose, step_immediately=False)
                else:
                    self.simulator.set_follower_joints(unaligned_pose)
                self._notify_motion()

                await cal_queue.get()

                # User confirmed middle position -> enter recording_range; allow manual slider sweep OR optional 'sweep' demo
                self._cal_state["stage"] = "recording_range"
                self._cal_state["waiting_for_input"] = True
                self._cal_state["prompt"] = (
                    "【模拟标定 2/2】请拖动下方滑块活动各关节（或点【自动演示扫限位】），完成后点击【确认保存】。"
                )
                self.current_step = 2
                cur_l, cur_f, _ = self.simulator.snapshot()
                cur_target = cur_l if target == "leader" else cur_f
                self._record_dry_cal_live_table(cur_target)
                self._notify_motion()

                while True:
                    cmd_val = await cal_queue.get()
                    if str(cmd_val).strip().lower() == "sweep":
                        for tick in range(18):
                            sweep_joints: dict[str, float] = {}
                            for idx, jname in enumerate(JOINT_NAMES):
                                pos = int(2048 + math.sin(tick * 0.45 + idx * 0.7) * 620)
                                sweep_joints[jname] = encoder_to_deg(jname, float(pos))
                            if target == "leader":
                                self.simulator.set_leader_joints(sweep_joints, step_immediately=False)
                            else:
                                self.simulator.set_follower_joints(sweep_joints)
                            self._record_dry_cal_live_table(sweep_joints)
                            self._notify_motion()
                            await asyncio.sleep(0.06)
                        continue
                    break

                self.current_step = 3
                # Hard isolation: dry_run calibration NEVER writes to self.config.calibration_dir (real hardware config)
                cal_dir = self.config.dry_run_dir / "calibration"
                if cal_dir.resolve() == self.config.calibration_dir.resolve():
                    cal_dir = self.config.dry_run_dir / "sim_calibration"
                cal_dir.mkdir(parents=True, exist_ok=True)
                cal_file = cal_dir / f"{target}_{resolved_id}.json"
                joints_cal = {}
                for j in JOINT_NAMES:
                    row = self._cal_state["live_table"].get(j)
                    rmin = int(row["min"]) if row else 0
                    rmax = int(row["max"]) if row else 4095
                    joints_cal[j] = {
                        "homing_offset": 0,
                        "range_min": min(rmin, rmax),
                        "range_max": max(rmin, rmax),
                    }
                cal_payload = {
                    "simulated": True,
                    "target": target,
                    "arm_id": resolved_id,
                    "port": resolved_port,
                    "mode": "dry_run",
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "joints": joints_cal,
                }
                cal_file.write_text(
                    json.dumps(cal_payload, indent=2, ensure_ascii=False), encoding="utf-8"
                )
                self.simulator.reset()
                self._notify_motion()
                self._cal_state["stage"] = "completed"
                self._cal_state["waiting_for_input"] = False
                self._cal_state["saved_path"] = str(cal_file)
                self._cal_state["prompt"] = f"模拟标定完成（已隔离保存至模拟目录 -> {cal_file.name}）"
                self.log(f"[dry_run] Isolated sim calibration complete -> {cal_file.name}")
            except asyncio.CancelledError:
                self.log("[dry_run] Calibration cancelled.")
                raise
            finally:
                self._dry_cal_queue = None
                if self.active_workflow == "calibrate":
                    self.active_workflow = "idle"
                    self.state = "idle"

        self._task = asyncio.create_task(_sim_calibrate())
        return {
            "status": "started",
            "workflow": "calibrate",
            "mode": "dry_run",
            "target": target,
            "port": resolved_port,
            "command": cmd,
        }

    async def send_calibrate_input(self, value: str = "") -> dict[str, Any]:
        self._sync_process_state()
        if self.active_workflow != "calibrate":
            raise WorkflowConflictError(
                "No calibration workflow is currently running.",
                code="CALIBRATE_NOT_ACTIVE",
            )
        cleaned = (value or "").strip()
        if self.hardware.active_mode == "real":
            if self._active_proc_id is None or not self.process_registry.write_stdin(
                self._active_proc_id, cleaned
            ):
                raise WorkflowConflictError(
                    "Calibration process is not accepting input.",
                    code="CALIBRATE_NOT_ACCEPTING_INPUT",
                )
            self._cal_state["waiting_for_input"] = False
            self.log(f"Calibration input sent: {'<ENTER>' if not cleaned else cleaned}")
            return {"status": "sent", "value": cleaned}

        if self._dry_cal_queue is not None:
            await self._dry_cal_queue.put(cleaned)
            self.log(f"[dry_run] Calibration input sent: {'<ENTER>' if not cleaned else cleaned}")
            return {"status": "sent", "value": cleaned}
        raise WorkflowConflictError("Dry-run calibration queue unavailable.", code="CALIBRATE_NOT_ACTIVE")

    async def resume_real_monitor_if_idle(self) -> None:
        """In real mode while idle, keep 30Hz passive real_to_sim monitor alive (torque OFF, 3D live)."""
        if self.hardware.active_mode != "real" or self.active_workflow != "idle":
            return
        if self.bridge.direction == "real_to_sim":
            return
        try:
            await asyncio.to_thread(
                self.bridge.set_sync_mode,
                "real_to_sim",
                is_real_mode=True,
                safety_confirmed=False,
            )
            self.simulator.torque_enabled = False
            self._notify_motion()
        except Exception as exc:
            self.log(f"[BRIDGE] Auto-resume real_to_sim monitor skipped: {exc}")

    async def stop_calibrate(self) -> dict[str, Any]:
        if self.active_workflow == "calibrate":
            # Close stdin pipe first so calibrate_runner catches EOFError and runs device.disconnect()
            # (cleanly disabling servo torque instead of leaving the follower arm stiff/locked).
            if self._active_proc_id is not None:
                managed = self.process_registry.get(self._active_proc_id)
                if managed is not None and managed.proc.stdin is not None:
                    try:
                        managed.proc.stdin.close()
                    except Exception:
                        pass
                    for _ in range(8):
                        if managed.proc.poll() is not None:
                            break
                        await asyncio.sleep(0.1)
            self._stop_active_process()
            await self._cancel_background_task()
            self._dry_cal_queue = None
            self.active_workflow = "idle"
            self.state = "idle"
            self._cal_state["active"] = False
            self._cal_state["waiting_for_input"] = False
            self._cal_state["stage"] = "stopped"
            self._cal_state["prompt"] = "标定已停止（已释放舵机扭矩）。"
            await self.resume_real_monitor_if_idle()
            self.log("Calibration stopped.")
        return {"status": "stopped", "workflow": "calibrate"}

    # ------------------------------------------------------------------
    # Teleop
    # ------------------------------------------------------------------
    async def start_teleop(
        self,
        *,
        fps: int = 60,
        follower_port: str | None = None,
        leader_port: str | None = None,
        display_data: bool = False,
        use_cameras: bool = False,
    ) -> dict[str, Any]:
        if self.simulator.estop_latched:
            raise WorkflowConflictError("急停已锁定 (E-STOP)，请先解除急停再启动遥操作。", code="ESTOP_LATCHED")
        self._sync_process_state()
        await self._ensure_idle()
        self.last_error = None
        cmd = self.build_teleop_cmd(
            fps=fps,
            follower_port=follower_port,
            leader_port=leader_port,
            display_data=display_data,
            use_cameras=use_cameras,
        )

        if self.hardware.active_mode == "real":
            dual_arm = bool(self.hardware.last_probe.get("dual_arm_ready", False))
            self.simulator.set_linked_arms(True)
            bridge_st = await asyncio.to_thread(
                self.bridge.start_bidirectional_teleop,
                fps=fps,
                connect_leader=dual_arm,
            )
            self.active_workflow = "teleop"
            self.state = "running"
            self._notify_motion()
            self.log(
                f"Started real bidirectional teleop at {fps} FPS "
                f"(follower={bridge_st['connected_follower']}, leader={bridge_st['connected_leader']})."
            )
            return {
                "status": "started",
                "workflow": "teleop",
                "mode": "real",
                "fps": fps,
                "bridge": bridge_st,
                "command": cmd,
            }

        self.simulator.set_linked_arms(True)
        self.active_workflow = "teleop"
        self.state = "running"
        self.current_step = 0
        self.log(f"Started dry_run teleoperation loop ({fps} FPS).")

        async def _sim_teleop() -> None:
            interval = max(0.02, min(0.1, 1.0 / max(1, fps)))
            try:
                while True:
                    self.simulator.step(interval)
                    self.current_step += 1
                    await asyncio.sleep(interval)
            except asyncio.CancelledError:
                self.log("[dry_run] Teleoperation stopped.")
                raise

        self._task = asyncio.create_task(_sim_teleop())
        return {
            "status": "started",
            "workflow": "teleop",
            "mode": "dry_run",
            "fps": fps,
        }

    async def stop_teleop(self) -> dict[str, Any]:
        if self.active_workflow == "teleop":
            self._stop_active_process()
            await self._cancel_background_task()
            self.active_workflow = "idle"
            self.state = "idle"
            if self.hardware.active_mode == "real":
                await self.resume_real_monitor_if_idle()
            self.log("Teleoperation stopped (servo torque released, 3D monitor active).")
        return {"status": "stopped", "workflow": "teleop"}

    def update_teleop_joints(self, raw_joints: dict[str, Any]) -> dict[str, Any]:
        self._arbitrate_manual_write()
        validated = validate_joint_dict(raw_joints, require_non_empty=True)
        # Tab 2 direct-push sliders always control the actuated Follower arm.
        # Never overwrite physical Leader's 3D pose when real Leader is actively streaming (prevents twitching).
        if self.simulator.linked_arms and not self.simulator.real_leader_active:
            self.simulator.set_leader_joints(validated, step_immediately=True)
        else:
            self.simulator.set_follower_joints(validated)
        self._notify_motion()
        leader, follower, _ = self.simulator.snapshot()
        real_dispatched = self.bridge.send_follower_target_deg(follower)
        return {
            "status": "ok",
            "leader_joints": leader,
            "follower_joints": follower,
            "linked_arms": self.simulator.linked_arms,
            "real_dispatched": real_dispatched,
        }

    # ------------------------------------------------------------------
    # Demonstration Teaching & Waypoint Replay
    # ------------------------------------------------------------------
    def record_waypoint(
        self, *, label: str | None = None, joints: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        _, follower, _ = self.simulator.snapshot()
        if joints is not None:
            validated = validate_joint_dict(joints, require_non_empty=True)
            snap = {**follower, **validated}
            self.simulator.set_leader_joints(validated, step_immediately=True)
        else:
            snap = dict(follower)

        idx = len(self._waypoints)
        wp_id = f"wp_{idx + 1:04d}"
        wp = {
            "index": idx,
            "id": wp_id,
            "label": (label or "").strip() or wp_id,
            "joints": snap,
            "mode": self.hardware.active_mode,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._waypoints.append(wp)
        self._save_waypoints()
        self.overlay_seq += 1
        self._notify_motion()
        self.log(f"Recorded waypoint '{wp['label']}' (#{idx + 1}).")
        return {
            "status": "recorded",
            "waypoint": wp,
            "total_waypoints": len(self._waypoints),
        }

    def list_waypoints(self) -> dict[str, Any]:
        try:
            rel_path = str(self.waypoints_path.relative_to(self.config.workspace_root))
        except ValueError:
            rel_path = str(self.waypoints_path)
        return {
            "waypoints": list(self._waypoints),
            "count": len(self._waypoints),
            "storage_path": rel_path.replace("\\", "/"),
        }

    async def _ensure_real_drive_bridge(self) -> None:
        """In real mode, ensure the follower bridge is connected in drive mode before replay/preset/routine."""
        if self.hardware.active_mode != "real":
            return
        if self.bridge.direction in ("sim_to_real", "teleop_bidirectional"):
            return
        await asyncio.to_thread(
            self.bridge.set_sync_mode,
            "sim_to_real",
            is_real_mode=True,
            safety_confirmed=True,
        )

    async def replay_waypoints(self, *, speed: float = 1.0, loop: bool = False) -> dict[str, Any]:
        self._sync_process_state()
        if len(self._waypoints) == 0:
            raise WorkflowConflictError(
                "No waypoints recorded to replay.", code="NO_WAYPOINTS"
            )
        await self._ensure_idle()
        await self._ensure_real_drive_bridge()
        self.last_error = None
        self.active_workflow = "teaching_replay"
        self.state = "running"
        self.current_step = 0
        waypoints_snapshot = list(self._waypoints)
        self.log(f"Replaying {len(waypoints_snapshot)} waypoints (speed={speed}x).")

        async def _run_replay() -> None:
            substeps = max(6, int(12 / max(0.1, speed)))
            try:
                while True:
                    for wp in waypoints_snapshot:
                        _, start_j, _ = self.simulator.snapshot()
                        target_j = wp["joints"]
                        for s in range(1, substeps + 1):
                            t = 0.5 * (1.0 - math.cos(math.pi * (s / substeps)))
                            interp = {
                                k: round(
                                    start_j[k] + (float(target_j[k]) - start_j[k]) * t, 3
                                )
                                for k in JOINT_NAMES
                            }
                            use_real_fb = self.hardware.active_mode == "real" and self.bridge.connected_follower
                            if use_real_fb:
                                self.bridge.send_follower_target_deg(interp)
                            else:
                                if not self.simulator.real_leader_active:
                                    self.simulator.set_leader_joints(interp, step_immediately=False)
                                self.simulator.set_follower_joints(interp)
                                self._notify_motion()
                            self.current_step += 1
                            await asyncio.sleep(0.03)
                    if not loop:
                        break
                self.log("Waypoint replay finished.")
            except asyncio.CancelledError:
                self.log("Waypoint replay stopped.")
                raise
            finally:
                if self.active_workflow == "teaching_replay":
                    self.active_workflow = "idle"
                    self.state = "idle"

        self._task = asyncio.create_task(_run_replay())
        return {
            "status": "replaying",
            "workflow": "teaching_replay",
            "waypoint_count": len(waypoints_snapshot),
            "speed": speed,
            "loop": loop,
        }

    async def clear_waypoints(self) -> dict[str, Any]:
        if self.active_workflow == "teaching_replay":
            await self._cancel_background_task()
            self.active_workflow = "idle"
            self.state = "idle"
        self._waypoints = []
        self._save_waypoints()
        self.overlay_seq += 1
        self._notify_motion()
        self.log("Cleared all teaching waypoints.")
        return {"status": "cleared", "count": 0}

    # ------------------------------------------------------------------
    # Continuous Trajectory Teaching & Linear Replay (Live Teleop Bridge + Official lerobot-replay)
    # ------------------------------------------------------------------
    def _linear_store_dir(self, *, is_real: bool) -> Path:
        base = self.config.datasets_dir if is_real else self.config.dry_run_dir
        p = base / "linear_teach"
        p.mkdir(parents=True, exist_ok=True)
        return p

    def list_linear_trajectories(self) -> dict[str, Any]:
        items: list[dict[str, Any]] = []
        # 1. Real live-recorded linear trajectories under data/datasets/linear_teach/
        real_lin_dir = self._linear_store_dir(is_real=True)
        for f in sorted(real_lin_dir.glob("*.json")):
            try:
                payload = json.loads(f.read_text(encoding="utf-8"))
                name = str(payload.get("name") or f.stem)
                ep = int(payload.get("episode", 0))
                fps = int(payload.get("fps", 30))
                frames = len(payload.get("frames", []))
                items.append(
                    {
                        "name": name,
                        "episode": ep,
                        "mode": "real",
                        "fps": fps,
                        "frames": frames,
                        "path": str(f),
                        "replay_cmd": " ".join(
                            self.build_replay_cmd(dataset_name=name, episode=ep, fps=fps)
                        ),
                    }
                )
            except Exception:
                continue

        # 2. Official LeRobot datasets under data/datasets/<name>/ (with meta/info.json)
        if self.config.datasets_dir.exists():
            for ds_dir in sorted(self.config.datasets_dir.iterdir()):
                if not ds_dir.is_dir() or ds_dir.name == "linear_teach":
                    continue
                info_p = ds_dir / "meta" / "info.json"
                if not info_p.exists():
                    continue
                total_eps = 1
                fps = 30
                total_frames = 0
                try:
                    info = json.loads(info_p.read_text(encoding="utf-8"))
                    total_eps = max(1, int(info.get("total_episodes", 1)))
                    fps = int(info.get("fps", 30))
                    total_frames = int(info.get("total_frames", 0))
                except Exception:
                    pass
                for ep in range(total_eps):
                    cmd = self.build_replay_cmd(dataset_name=ds_dir.name, episode=ep, fps=fps)
                    items.append(
                        {
                            "name": ds_dir.name,
                            "episode": ep,
                            "mode": "real",
                            "fps": fps,
                            "frames": total_frames,
                            "path": str(ds_dir),
                            "replay_cmd": " ".join(cmd),
                        }
                    )

        # 3. Dry-run continuous trajectories under data/dry_run/linear_teach/
        dry_dir = self._linear_store_dir(is_real=False)
        for f in sorted(dry_dir.glob("*.json")):
            try:
                payload = json.loads(f.read_text(encoding="utf-8"))
                name = str(payload.get("name") or f.stem)
                ep = int(payload.get("episode", 0))
                fps = int(payload.get("fps", 20))
                frames = len(payload.get("frames", []))
                items.append(
                    {
                        "name": name,
                        "episode": ep,
                        "mode": "dry_run",
                        "fps": fps,
                        "frames": frames,
                        "path": str(f),
                        "replay_cmd": " ".join(
                            self.build_replay_cmd(dataset_name=name, episode=ep, fps=fps)
                        ),
                    }
                )
            except Exception:
                continue

        return {
            "mode": self.hardware.active_mode,
            "active_workflow": self.active_workflow,
            "trajectories": items,
            "count": len(items),
        }

    def _flush_linear_trajectory(self) -> dict[str, Any] | None:
        ctx = self._linear_teach_ctx
        if not ctx:
            return None
        frames = ctx.get("frames", [])
        if not frames:
            self._linear_teach_ctx = None
            return None
        name = str(ctx["name"])
        ep = int(ctx.get("episode", 0))
        fps = int(ctx.get("fps", 30))
        is_real = bool(ctx.get("is_real", False))
        out_file = self._linear_store_dir(is_real=is_real) / f"{name}_ep{ep:04d}.json"
        payload = {
            "name": name,
            "episode": ep,
            "mode": "real" if is_real else "dry_run",
            "fps": fps,
            "num_frames": len(frames),
            "created_at": datetime.now(timezone.utc).isoformat(),
            "frames": frames,
        }
        out_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        self._linear_teach_ctx = None
        result = {
            "name": name,
            "episode": ep,
            "mode": "real" if is_real else "dry_run",
            "fps": fps,
            "frames": len(frames),
            "path": str(out_file),
        }
        self._last_linear_saved = result
        return result

    async def start_linear_teach(
        self,
        *,
        name: str = "demo_linear_01",
        duration_s: float = 30.0,
        fps: int = 30,
    ) -> dict[str, Any]:
        if self.simulator.estop_latched:
            raise WorkflowConflictError("急停已锁定 (E-STOP)，请先解除急停再开始连续示教。", code="ESTOP_LATCHED")
        self._sync_process_state()
        safe_name = re.sub(r"[^a-zA-Z0-9_\-]", "_", name.strip() or "demo_linear_01")
        is_real = self.hardware.active_mode == "real"

        # Keep existing live teleop running seamlessly! Only stop non-teleop conflicts.
        if self.active_workflow not in ("idle", "teleop"):
            await self._ensure_idle()
        self.last_error = None

        if is_real:
            # Ensure live Leader->Follower teleop is running (do NOT disconnect teleop!)
            self.simulator.set_linked_arms(True)
            if self.bridge.direction != "teleop_bidirectional":
                await asyncio.to_thread(
                    self.bridge.start_bidirectional_teleop,
                    fps=min(60, max(20, int(fps))),
                )
            # Clear UI latch so physical Leader immediately drives Follower
            with self.bridge._lock:
                self.bridge._ui_control_latched = False
            self.simulator.clear_follower_cmd_target()

        rec_fps = min(30, max(10, int(fps))) if is_real else min(20, max(10, int(fps)))
        self.active_workflow = "linear_teach"
        self.state = "running"
        self.current_step = 0
        self._linear_teach_ctx = {
            "name": safe_name,
            "episode": 0,
            "is_real": is_real,
            "fps": rec_fps,
            "frames": [],
        }
        self.log(
            f"Started {'real teleop' if is_real else 'dry_run'} continuous linear teaching "
            f"'{safe_name}' ({duration_s}s @ {rec_fps}Hz — Leader guides Follower live)."
        )

        async def _linear_record_loop() -> None:
            dt = 1.0 / rec_fps
            max_frames = max(10, int(duration_s * rec_fps))
            try:
                for i in range(max_frames):
                    _, f_j, _ = self.simulator.snapshot()
                    if self._linear_teach_ctx is not None:
                        self._linear_teach_ctx["frames"].append(dict(f_j))
                    self.current_step = i + 1
                    await asyncio.sleep(dt)
                saved = self._flush_linear_trajectory()
                if saved:
                    self.log(f"Saved linear trajectory '{saved['name']}' ({saved['frames']} frames).")
            except asyncio.CancelledError:
                saved = self._flush_linear_trajectory()
                if saved:
                    self.log(f"Stopped & saved linear trajectory '{saved['name']}' ({saved['frames']} frames).")
                raise
            finally:
                if self.active_workflow == "linear_teach":
                    # If real teleop bridge is still live, return cleanly to teleop state instead of dropping it
                    if is_real and self.bridge.direction == "teleop_bidirectional":
                        self.active_workflow = "teleop"
                        self.state = "running"
                    else:
                        self.active_workflow = "idle"
                        self.state = "idle"

        self._task = asyncio.create_task(_linear_record_loop())
        return {
            "status": "recording",
            "mode": "real" if is_real else "dry_run",
            "workflow": "linear_teach",
            "name": safe_name,
            "duration_s": duration_s,
            "fps": rec_fps,
            "teleop_kept_alive": is_real,
        }

    async def stop_linear_teach(self) -> dict[str, Any]:
        saved: dict[str, Any] | None = None
        if self._active_proc_id == "linear_teach":
            self.process_registry.terminate("linear_teach")
            self._active_proc_id = None
        if self.active_workflow == "linear_teach":
            self._last_linear_saved = None
            await self._cancel_background_task()
            saved = self._flush_linear_trajectory() or getattr(self, "_last_linear_saved", None)
            if self.hardware.active_mode == "real" and self.bridge.direction == "teleop_bidirectional":
                self.active_workflow = "teleop"
                self.state = "running"
            else:
                self.active_workflow = "idle"
                self.state = "idle"
        self.log("Stopped linear teaching.")
        return {"status": "stopped", "workflow": self.active_workflow, "saved": saved}

    async def replay_linear_teach(
        self,
        *,
        name: str,
        episode: int = 0,
        fps: int = 30,
    ) -> dict[str, Any]:
        self._sync_process_state()
        safe_name = re.sub(r"[^a-zA-Z0-9_\-]", "_", name.strip())
        if not safe_name:
            raise WorkflowConflictError("请先选择要线性回放的轨迹名称。", code="NO_TRAJECTORY")

        is_real = self.hardware.active_mode == "real"
        # Look for live-recorded JSON trajectory first (real dir -> dry_run dir)
        traj_file = self._linear_store_dir(is_real=is_real) / f"{safe_name}_ep{episode:04d}.json"
        if not traj_file.exists():
            alt_file = self._linear_store_dir(is_real=not is_real) / f"{safe_name}_ep{episode:04d}.json"
            if alt_file.exists():
                traj_file = alt_file

        if traj_file.exists():
            payload = json.loads(traj_file.read_text(encoding="utf-8"))
            frames = payload.get("frames", [])
            rec_fps = max(5, int(payload.get("fps", fps)))
            if not frames:
                raise WorkflowConflictError("该轨迹文件帧数为 0，无法回放。", code="EMPTY_TRAJECTORY")

            if self.active_workflow not in ("idle", "teleop"):
                await self._ensure_idle()
            self.last_error = None

            if is_real:
                # Keep live bridge if already connected; otherwise engage Follower drive bridge
                if self.bridge.direction not in ("teleop_bidirectional", "sim_to_real"):
                    await self._ensure_real_drive_bridge()

            self.active_workflow = "linear_replay"
            self.state = "running"
            self.current_step = 0
            self.log(
                f"Replaying {'real Follower' if is_real else 'dry_run'} linear trajectory "
                f"'{safe_name}' ({len(frames)} frames @ {rec_fps}Hz)."
            )

            async def _linear_replay_loop() -> None:
                dt = 1.0 / rec_fps
                try:
                    # Smooth 0.6s anti-jump ramp from current Follower pose to frame 0
                    _, cur_f, _ = self.simulator.snapshot()
                    first_f = frames[0]
                    ramp_steps = 18
                    use_real_fb = is_real and self.bridge.connected_follower
                    for r in range(1, ramp_steps + 1):
                        t = 0.5 * (1.0 - math.cos(math.pi * (r / ramp_steps)))
                        interp = {
                            k: round(cur_f[k] + (float(first_f[k]) - cur_f[k]) * t, 3)
                            for k in JOINT_NAMES
                        }
                        if use_real_fb:
                            # Single source of truth: send to real arm; 30Hz bridge feedback animates 3D cleanly (zero jitter)
                            self.bridge.send_follower_target_deg(interp)
                        else:
                            self.simulator.set_follower_joints(interp)
                            self._notify_motion()
                        await asyncio.sleep(0.03)

                    # Frame-by-frame continuous linear replay on Follower only
                    for i, frame in enumerate(frames):
                        if use_real_fb:
                            self.bridge.send_follower_target_deg(frame)
                        else:
                            self.simulator.set_follower_joints(frame)
                            self._notify_motion()
                        self.current_step = i + 1
                        await asyncio.sleep(dt)
                    self.log(f"Finished linear replay '{safe_name}'.")
                except asyncio.CancelledError:
                    self.log("Stopped linear replay.")
                    raise
                finally:
                    if self.active_workflow == "linear_replay":
                        if is_real and self.bridge.direction == "teleop_bidirectional":
                            self.active_workflow = "teleop"
                            self.state = "running"
                        else:
                            self.active_workflow = "idle"
                            self.state = "idle"

            self._task = asyncio.create_task(_linear_replay_loop())
            return {
                "status": "replaying",
                "mode": "real" if is_real else "dry_run",
                "workflow": "linear_replay",
                "name": safe_name,
                "episode": episode,
                "frames": len(frames),
                "fps": rec_fps,
            }

        # Fallback: official LeRobotDataset folder replay via lerobot-replay CLI (real mode)
        if is_real:
            ds_dir = self.config.datasets_dir / safe_name
            if not ds_dir.exists():
                raise WorkflowConflictError(
                    f"未找到连续轨迹 '{safe_name}'，请先点击「开始连续示教」录制一段轨迹。",
                    code="TRAJECTORY_NOT_FOUND",
                )
            await self._ensure_idle()
            self._camera_hub.release_all()
            self.bridge.disconnect(return_to_rest=False, keep_torque=False)
            cmd = self.build_replay_cmd(dataset_name=safe_name, episode=episode, fps=fps)
            env = {
                "PYTHONPATH": str(self.config.repos_dir / "lerobot" / "src"),
                "PYTHONUNBUFFERED": "1",
            }
            managed = self.process_registry.spawn(
                "linear_replay",
                "lerobot-replay",
                cmd,
                cwd=self.config.workspace_root,
                env_overrides=env,
                on_line=self.log,
            )
            self._active_proc_id = "linear_replay"
            self.active_workflow = "linear_replay"
            self.state = "running"
            self.log(f"Started official lerobot-replay '{safe_name}' ep={episode} (pid={managed.proc.pid}).")
            return {
                "status": "replaying",
                "mode": "real",
                "workflow": "linear_replay",
                "name": safe_name,
                "episode": episode,
                "command": cmd,
                "pid": managed.proc.pid,
            }

        raise WorkflowConflictError(
            f"未找到模拟连续轨迹 '{safe_name}'，请先点击「开始连续示教」录制一段轨迹。",
            code="TRAJECTORY_NOT_FOUND",
        )

    async def stop_linear_replay(self) -> dict[str, Any]:
        if self._active_proc_id == "linear_replay":
            self.process_registry.terminate("linear_replay")
            self._active_proc_id = None
        if self.active_workflow == "linear_replay":
            await self._cancel_background_task()
            if self.hardware.active_mode == "real" and self.bridge.direction == "teleop_bidirectional":
                self.active_workflow = "teleop"
                self.state = "running"
            else:
                self.active_workflow = "idle"
                self.state = "idle"
        self.log("Stopped linear replay.")
        return {"status": "stopped", "workflow": self.active_workflow}

    def delete_linear_trajectory(self, *, name: str, episode: int = 0) -> dict[str, Any]:
        safe_name = re.sub(r"[^a-zA-Z0-9_\-]", "_", name.strip())
        deleted = False
        for is_real in (True, False):
            p = self._linear_store_dir(is_real=is_real) / f"{safe_name}_ep{episode:04d}.json"
            if p.exists():
                p.unlink()
                deleted = True
        return {"status": "deleted" if deleted else "not_found", "name": safe_name, "episode": episode}

    # ------------------------------------------------------------------
    # Data Collection
    # ------------------------------------------------------------------
    def _flush_dry_run_episode(self) -> int:
        ctx = self._collect_ctx
        if ctx is None:
            return 0
        frames = ctx.get("frames", [])
        if not frames:
            return int(ctx.get("saved_episodes", 0))

        ep_idx = int(ctx["episode_index"])
        ep_dir: Path = ctx["episodes_dir"]
        ep_dir.mkdir(parents=True, exist_ok=True)
        ep_file = ep_dir / f"episode_{ep_idx:04d}.json"
        ep_payload = {
            "dataset": ctx["dataset_name"],
            "episode_index": ep_idx,
            "task": ctx["single_task"],
            "fps": ctx["fps"],
            "num_frames": len(frames),
            "created_at": datetime.now(timezone.utc).isoformat(),
            "frames": frames,
        }
        ep_file.write_text(
            json.dumps(ep_payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        ctx["frames"] = []
        ctx["episode_index"] = ep_idx + 1
        ctx["saved_episodes"] = int(ctx.get("saved_episodes", 0)) + 1
        return int(ctx["saved_episodes"])

    def _resolve_active_cameras_for_collect(self) -> list[tuple[str, int | str]]:
        """Return configured & available (logical_name, index_or_path) cameras for in-process collection."""
        out: list[tuple[str, int | str]] = []
        probe_cams = self.hardware.last_probe.get("cameras", [])
        for logical, configured_raw in (
            ("front", self.config.front_camera_id),
            ("wrist", self.config.wrist_camera_id),
        ):
            if configured_raw is not None and configured_raw.lower() == "none":
                continue
            for cam in probe_cams:
                if not cam.get("available"):
                    continue
                cam_raw = str(cam.get("raw_id", cam.get("index_or_path", "")))
                matched = (
                    cam_raw == configured_raw
                    if configured_raw is not None
                    else str(cam.get("id")) == logical
                )
                if matched:
                    out.append((logical, cam["index_or_path"]))
                    break
        return out

    def collect_episode_action(
        self, action: Literal["next", "rerecord", "stop"]
    ) -> dict[str, Any]:
        if self.active_workflow != "collect":
            raise WorkflowConflictError("当前未在执行数据采集。", code="NOT_COLLECTING")
        self._collect_action_cmd = action
        if self._collect_action_ev is not None:
            self._collect_action_ev.set()
        self.log(f"[COLLECT] Episode action triggered: {action}")
        return {"status": "ok", "action": action, "collect_stage": self._collect_stage}

    async def start_collect(
        self,
        *,
        dataset_name: str,
        single_task: str,
        num_episodes: int = 2,
        episode_time_s: float = 30.0,
        reset_time_s: float = 3.0,
        fps: int = 30,
        push_to_hub: bool = False,
    ) -> dict[str, Any]:
        if self.simulator.estop_latched:
            raise WorkflowConflictError("急停已锁定 (E-STOP)，请先解除急停再开始数据采集。", code="ESTOP_LATCHED")
        self._sync_process_state()
        await self._ensure_idle()
        self.last_error = None
        self._collect_action_ev = asyncio.Event()
        self._collect_action_cmd = None

        if self.hardware.active_mode == "real":
            # In-process official LeRobotDataset v3.0 recorder on top of live 30Hz SO101RealBridge + CameraHub.
            # Instant (~0.2s) start, never disconnects 3D viewport or USB cameras, 100% official Parquet + MP4 format.
            safe_ds = re.sub(r"[^a-zA-Z0-9_\-]", "_", dataset_name.strip()) or "so101_dataset"
            out_dir = self.config.datasets_dir / safe_ds
            self.config.datasets_dir.mkdir(parents=True, exist_ok=True)
            cmd = self.build_record_cmd(
                dataset_name=safe_ds,
                single_task=single_task,
                num_episodes=num_episodes,
                episode_time_s=episode_time_s,
                reset_time_s=reset_time_s,
                fps=fps,
                push_to_hub=push_to_hub,
            )

            self.active_workflow = "collect"
            self.state = "running"
            self.current_episode = 0
            self.target_episodes = num_episodes
            self.current_step = 0
            self._collect_stage = "preparing"
            self._collect_stage_text = "正在开启双臂遥操作并初始化 LeRobotDataset v3.0..."
            self._notify_motion()

            try:
                await asyncio.to_thread(self.bridge.start_bidirectional_teleop, fps=fps)
                with self.bridge._lock:
                    self.bridge._ui_control_latched = False
                self.simulator.clear_follower_cmd_target()
            except Exception as exc:
                self.active_workflow = "idle"
                self.state = "idle"
                self._collect_stage = "idle"
                self._collect_stage_text = ""
                self.last_error = f"开启双臂遥操作失败: {exc}"
                raise WorkflowConflictError(self.last_error, code="BRIDGE_ERROR") from exc

            active_cams = self._resolve_active_cameras_for_collect()
            for _, cam_idx in active_cams:
                self._camera_hub.get_frame(cam_idx)

            def _init_dataset() -> Any:
                from lerobot.datasets.lerobot_dataset import LeRobotDataset

                repo_id = f"local/{safe_ds}"
                info_json = out_dir / "meta" / "info.json"
                if out_dir.exists() and not info_json.is_file():
                    shutil.rmtree(out_dir, ignore_errors=True)
                if info_json.is_file():
                    return LeRobotDataset.resume(
                        repo_id=repo_id,
                        root=out_dir,
                        streaming_encoding=True,
                    )
                joint_feat_names = [f"{j}.pos" for j in JOINT_NAMES]
                features: dict[str, dict[str, Any]] = {
                    "observation.state": {
                        "dtype": "float32",
                        "shape": (6,),
                        "names": joint_feat_names,
                    },
                    "action": {
                        "dtype": "float32",
                        "shape": (6,),
                        "names": joint_feat_names,
                    },
                }
                for cam_name, _ in active_cams:
                    features[f"observation.images.{cam_name}"] = {
                        "dtype": "video",
                        "shape": (480, 640, 3),
                        "names": ["height", "width", "channels"],
                    }
                return LeRobotDataset.create(
                    repo_id=repo_id,
                    fps=fps,
                    features=features,
                    root=out_dir,
                    robot_type="so101_follower",
                    use_videos=bool(active_cams),
                    streaming_encoding=True,
                )

            try:
                dataset = await asyncio.to_thread(_init_dataset)
            except Exception as exc:
                self.active_workflow = "idle"
                self.state = "idle"
                self._collect_stage = "idle"
                self._collect_stage_text = ""
                self.last_error = f"初始化 LeRobotDataset v3.0 失败: {exc}"
                self.log(self.last_error)
                raise WorkflowConflictError(self.last_error, code="DATASET_INIT_ERROR") from exc

            self._collect_ctx = {
                "mode": "real",
                "dataset_name": safe_ds,
                "single_task": single_task,
                "fps": fps,
                "saved_episodes": 0,
            }
            self.log(
                f"Started in-process LeRobotDataset v3.0 collection -> {out_dir} "
                f"(cams={[c[0] for c in active_cams]}, {num_episodes} eps @ {fps}Hz)"
            )

            async def _real_collect() -> None:
                import numpy as np

                period = 1.0 / max(1, fps)
                stop_all = False
                try:
                    for ep_num in range(1, num_episodes + 1):
                        if stop_all:
                            break
                        while True:
                            self.current_episode = ep_num
                            self._collect_stage = "recording_ep"
                            self._collect_stage_text = (
                                f"正在录制第 {ep_num}/{num_episodes} 条 · 摆动主臂即可录制"
                                f"（点「完成本条」或按 → 提前结束）"
                            )
                            self._collect_action_cmd = None
                            if self._collect_action_ev is not None:
                                self._collect_action_ev.clear()

                            ep_frames = 0
                            t_ep_start = time.monotonic()
                            unlimited = episode_time_s <= 0
                            while unlimited or (time.monotonic() - t_ep_start) < episode_time_s:
                                if self._collect_action_ev is not None and self._collect_action_ev.is_set():
                                    break
                                t_tick = time.monotonic()
                                elapsed_ep = t_tick - t_ep_start
                                if ep_frames % max(1, fps // 3) == 0:
                                    if unlimited:
                                        self._collect_stage_text = (
                                            f"录制第 {ep_num}/{num_episodes} 条 · 已录 {elapsed_ep:.1f}s（手动模式，按 → 完成本条）"
                                        )
                                    else:
                                        rem_ep = max(0.0, episode_time_s - elapsed_ep)
                                        self._collect_stage_text = (
                                            f"录制第 {ep_num}/{num_episodes} 条 · 剩余 {rem_ep:.1f}s/{episode_time_s:.0f}s（按 → 提前完成）"
                                        )
                                leader, follower, _ = self.simulator.snapshot()
                                frame: dict[str, Any] = {
                                    "observation.state": np.array(
                                        [float(follower[j]) for j in JOINT_NAMES],
                                        dtype=np.float32,
                                    ),
                                    "action": np.array(
                                        [float(leader[j]) for j in JOINT_NAMES],
                                        dtype=np.float32,
                                    ),
                                    "task": single_task,
                                }
                                for cam_name, cam_idx in active_cams:
                                    rgb = self._camera_hub.get_latest_rgb(cam_idx)
                                    if rgb is None:
                                        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
                                    frame[f"observation.images.{cam_name}"] = rgb
                                await asyncio.to_thread(dataset.add_frame, frame)
                                ep_frames += 1
                                self.current_step += 1
                                rem = period - (time.monotonic() - t_tick)
                                if rem > 0.002:
                                    await asyncio.sleep(rem)
                                else:
                                    await asyncio.sleep(0)

                            cmd_act = self._collect_action_cmd
                            self._collect_action_cmd = None
                            if self._collect_action_ev is not None:
                                self._collect_action_ev.clear()

                            if cmd_act == "rerecord":
                                self._collect_stage = "resetting"
                                self._collect_stage_text = f"已丢弃第 {ep_num} 条，准备重录..."
                                await asyncio.to_thread(
                                    dataset.clear_episode_buffer, delete_images=True
                                )
                                self.log(f"[COLLECT] Discarded ep {ep_num}, re-recording...")
                                await asyncio.sleep(0.25)
                                continue

                            if ep_frames > 0:
                                self._collect_stage = "saving"
                                self._collect_stage_text = (
                                    f"正在保存第 {ep_num}/{num_episodes} 条 ({ep_frames} 帧 Parquet+MP4)..."
                                )
                                await asyncio.to_thread(dataset.save_episode)
                                if self._collect_ctx is not None:
                                    self._collect_ctx["saved_episodes"] = (
                                        int(self._collect_ctx.get("saved_episodes", 0)) + 1
                                    )
                                self.log(
                                    f"[COLLECT] Saved episode {ep_num}/{num_episodes} "
                                    f"({ep_frames} frames) -> {safe_ds}"
                                )
                            else:
                                await asyncio.to_thread(
                                    dataset.clear_episode_buffer, delete_images=True
                                )

                            if cmd_act == "stop":
                                stop_all = True
                            break

                        if not stop_all and ep_num < num_episodes and reset_time_s > 0:
                            self._collect_stage = "resetting"
                            t_rst = time.monotonic()
                            while (time.monotonic() - t_rst) < reset_time_s:
                                rem_s = max(0.0, reset_time_s - (time.monotonic() - t_rst))
                                self._collect_stage_text = (
                                    f"复位间隔 {rem_s:.1f}s（准备第 {ep_num + 1}/{num_episodes} 条，按 → 跳过）"
                                )
                                if self._collect_action_ev is not None and self._collect_action_ev.is_set():
                                    if self._collect_action_cmd == "stop":
                                        stop_all = True
                                    self._collect_action_cmd = None
                                    self._collect_action_ev.clear()
                                    break
                                await asyncio.sleep(0.1)

                    self._collect_stage = "completed"
                    self._collect_stage_text = f"采集完成：{safe_ds}（已落盘标准 LeRobotDataset v3.0）"
                except asyncio.CancelledError:
                    try:
                        if getattr(dataset, "episode_buffer", None) and dataset.episode_buffer.get("size", 0) > 0:
                            await asyncio.to_thread(dataset.save_episode)
                    except Exception:
                        pass
                    self.log("[COLLECT] Real collection cancelled & flushed.")
                    raise
                except Exception as exc:
                    self.last_error = f"实时采集异常: {exc}"
                    self._collect_stage = "idle"
                    self._collect_stage_text = self.last_error
                    self.log(self.last_error)
                finally:
                    try:
                        await asyncio.to_thread(dataset.finalize)
                    except Exception as fin_exc:
                        self.log(f"[COLLECT] finalize note: {fin_exc}")
                    if self.active_workflow == "collect":
                        self.active_workflow = "idle"
                        self.state = "idle"
                        if self._collect_stage != "completed":
                            self._collect_stage = "idle"
                    await self.resume_real_monitor_if_idle()

            self._task = asyncio.create_task(_real_collect())
            return {
                "status": "started",
                "workflow": "collect",
                "mode": "real",
                "dataset_name": safe_ds,
                "output_dir": str(out_dir),
                "command": cmd,
                "in_process": True,
            }

        ds_dir = self.config.dry_run_dir / "datasets" / dataset_name
        meta_dir = ds_dir / "meta"
        episodes_dir = ds_dir / "episodes"
        meta_dir.mkdir(parents=True, exist_ok=True)
        episodes_dir.mkdir(parents=True, exist_ok=True)

        existing_eps = sorted(episodes_dir.glob("episode_*.json"))
        next_idx = len(existing_eps)

        info_file = meta_dir / "info.json"
        info_payload = {
            "dataset_name": dataset_name,
            "robot_type": "so101_follower",
            "mode": "dry_run",
            "fps": fps,
            "single_task": single_task,
            "joint_names": list(JOINT_NAMES),
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        info_file.write_text(
            json.dumps(info_payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )

        self._collect_ctx = {
            "dataset_name": dataset_name,
            "single_task": single_task,
            "fps": fps,
            "episodes_dir": episodes_dir,
            "episode_index": next_idx,
            "saved_episodes": 0,
            "frames": [],
        }

        self.active_workflow = "collect"
        self.state = "running"
        self.current_episode = 0
        self.target_episodes = num_episodes
        self.current_step = 0
        self._collect_stage = "recording_ep"
        self._collect_stage_text = f"[DRY-RUN] 正在录制 {dataset_name} (0/{num_episodes})"
        self.log(
            f"Started dry_run collection '{dataset_name}' ({num_episodes} episodes @ {fps} FPS)."
        )

        async def _sim_collect() -> None:
            tick = max(0.03, min(0.1, 1.0 / max(1, fps)))
            unlimited = episode_time_s <= 0
            frames_per_ep = 100000 if unlimited else max(6, min(60, int(episode_time_s * 5)))
            self.simulator.set_linked_arms(True)
            stop_all = False
            try:
                for ep_num in range(1, num_episodes + 1):
                    if stop_all:
                        break
                    while True:
                        self.current_episode = ep_num
                        self._collect_stage = "recording_ep"
                        self._collect_stage_text = (
                            f"[DRY-RUN] 正在录制第 {ep_num}/{num_episodes} 条"
                            + ("（手动不限时，按 → 完成本条）" if unlimited else "...")
                        )
                        self._collect_action_cmd = None
                        if self._collect_action_ev is not None:
                            self._collect_action_ev.clear()
                        for frame_i in range(frames_per_ep):
                            if self._collect_action_ev is not None and self._collect_action_ev.is_set():
                                break
                            cycle_len = 60 if unlimited else max(1, frames_per_ep - 1)
                            progress = (frame_i % cycle_len) / cycle_len
                            wave = math.sin(progress * math.pi)
                            pan = round(-24.0 + 52.0 * progress, 2)
                            lift = round(30.0 * wave, 2)
                            elbow = round(26.0 * wave, 2)
                            wrist = round(-44.0 * wave, 2)
                            grip = 72.0 if progress < 0.35 or progress > 0.82 else 14.0
                            target_pose = {
                                "shoulder_pan": pan,
                                "shoulder_lift": lift,
                                "elbow_flex": elbow,
                                "wrist_flex": wrist,
                                "wrist_roll": round(15.0 * math.sin(progress * math.pi * 2), 2),
                                "gripper": grip,
                            }
                            self.simulator.set_leader_joints(target_pose, step_immediately=False)
                            self.simulator.step(max(0.08, tick))
                            leader, follower, _ = self.simulator.snapshot()
                            if self._collect_ctx is not None:
                                self._collect_ctx["frames"].append(
                                    {
                                        "frame_index": frame_i,
                                        "timestamp": round(frame_i / max(1, fps), 4),
                                        "observation.state": [follower[j] for j in JOINT_NAMES],
                                        "action": [leader[j] for j in JOINT_NAMES],
                                    }
                                )
                            self.current_step += 1
                            await asyncio.sleep(tick)
                        cmd_act = self._collect_action_cmd
                        self._collect_action_cmd = None
                        if self._collect_action_ev is not None:
                            self._collect_action_ev.clear()
                        if cmd_act == "rerecord":
                            if self._collect_ctx is not None:
                                self._collect_ctx["frames"] = []
                            continue
                        saved = self._flush_dry_run_episode()
                        self.log(
                            f"[dry_run] Saved episode {ep_num}/{num_episodes} to {dataset_name} (total={saved})."
                        )
                        if cmd_act == "stop":
                            stop_all = True
                        break
                    if not stop_all and ep_num < num_episodes:
                        self._collect_stage = "resetting"
                        await asyncio.sleep(min(0.3, reset_time_s))
                self._collect_stage = "completed"
                self._collect_stage_text = f"[DRY-RUN] 采集完成：{dataset_name}"
            except asyncio.CancelledError:
                self._flush_dry_run_episode()
                self.log("[dry_run] Collection stopped and flushed.")
                raise
            finally:
                if self.active_workflow == "collect":
                    self.active_workflow = "idle"
                    self.state = "idle"
                    if self._collect_stage != "completed":
                        self._collect_stage = "idle"

        self._task = asyncio.create_task(_sim_collect())
        try:
            rel_out = str(ds_dir.relative_to(self.config.workspace_root)).replace("\\", "/")
        except ValueError:
            rel_out = str(ds_dir)
        return {
            "status": "started",
            "workflow": "collect",
            "mode": "dry_run",
            "dataset_name": dataset_name,
            "output_dir": rel_out,
        }

    async def stop_collect(self) -> dict[str, Any]:
        saved_episodes = 0
        if self.active_workflow == "collect":
            ctx = self._collect_ctx
            self._stop_active_process()
            await self._cancel_background_task()
            if ctx is not None and ctx.get("mode") == "real":
                saved_episodes = int(ctx.get("saved_episodes", 0))
            else:
                saved_episodes = self._flush_dry_run_episode()
            self._collect_ctx = None
            self.active_workflow = "idle"
            self.state = "idle"
            self._collect_stage = "idle"
            self._collect_stage_text = ""
            await self.resume_real_monitor_if_idle()
            self.log(f"Data collection stopped (saved={saved_episodes}).")
        return {
            "status": "stopped",
            "workflow": "collect",
            "saved_episodes": saved_episodes,
        }

    def list_episodes(self) -> dict[str, Any]:
        episodes: list[dict[str, Any]] = []
        # 1. Real official LeRobotDataset v3.0 folders under data/datasets/<name>/
        if self.config.datasets_dir.exists():
            for ds_dir in sorted(self.config.datasets_dir.iterdir()):
                if not ds_dir.is_dir() or ds_dir.name == "linear_teach":
                    continue
                info_p = ds_dir / "meta" / "info.json"
                if not info_p.is_file():
                    continue
                try:
                    info = json.loads(info_p.read_text(encoding="utf-8"))
                    fps = int(info.get("fps", 30))
                    total_eps = int(info.get("total_episodes", 0))
                    total_frames = int(info.get("total_frames", 0))
                    features = info.get("features") or {}
                    video_cams = [
                        k.removeprefix("observation.images.")
                        for k, v in features.items()
                        if k.startswith("observation.images.") and isinstance(v, dict)
                    ]
                    ep_lengths: dict[int, int] = {}
                    ep_tasks: dict[int, str] = {}
                    ep_meta_files = sorted((ds_dir / "meta" / "episodes").glob("chunk-*/file-*.parquet"))
                    if ep_meta_files:
                        try:
                            import pyarrow.parquet as pq

                            for mf in ep_meta_files:
                                tbl = pq.read_table(mf)
                                for row in tbl.to_pylist():
                                    eidx = int(row.get("episode_index", 0))
                                    ep_lengths[eidx] = int(row.get("length", 0))
                                    tval = row.get("tasks")
                                    if isinstance(tval, list) and tval:
                                        ep_tasks[eidx] = str(tval[0])
                                    elif isinstance(tval, str):
                                        ep_tasks[eidx] = tval
                        except Exception:
                            pass
                    try:
                        rel_ds = str(ds_dir.relative_to(self.config.workspace_root)).replace("\\", "/")
                    except ValueError:
                        rel_ds = str(ds_dir)
                    created_ts = datetime.fromtimestamp(
                        info_p.stat().st_mtime, tz=timezone.utc
                    ).isoformat()
                    for ep_idx in range(total_eps):
                        n_frames = ep_lengths.get(
                            ep_idx, total_frames // max(1, total_eps)
                        )
                        episodes.append(
                            {
                                "dataset": ds_dir.name,
                                "episode_index": ep_idx,
                                "task": ep_tasks.get(ep_idx, ""),
                                "num_frames": n_frames,
                                "fps": fps,
                                "created_at": created_ts,
                                "path": rel_ds,
                                "source": "real",
                                "has_video": bool(video_cams),
                                "video_cams": video_cams,
                            }
                        )
                except Exception:
                    continue

        # 2. Dry-run episodes under data/dry_run/datasets/<name>/episodes/
        dry_datasets = self.config.dry_run_dir / "datasets"
        if dry_datasets.exists():
            for ep_file in sorted(dry_datasets.glob("*/episodes/episode_*.json")):
                try:
                    data = json.loads(ep_file.read_text(encoding="utf-8"))
                    try:
                        rel = str(ep_file.relative_to(self.config.workspace_root)).replace(
                            "\\", "/"
                        )
                    except ValueError:
                        rel = str(ep_file)
                    episodes.append(
                        {
                            "dataset": data.get("dataset", ep_file.parents[1].name),
                            "episode_index": data.get("episode_index", 0),
                            "task": data.get("task", ""),
                            "num_frames": data.get("num_frames", len(data.get("frames", []))),
                            "fps": data.get("fps", 30),
                            "created_at": data.get("created_at", ""),
                            "path": rel,
                            "source": "dry_run",
                            "has_video": False,
                            "video_cams": [],
                        }
                    )
                except Exception:
                    continue
        try:
            root_str = str(self.config.datasets_dir.relative_to(self.config.workspace_root)).replace("\\", "/")
        except ValueError:
            root_str = str(self.config.datasets_dir)
        return {
            "mode": self.hardware.active_mode,
            "root": root_str,
            "episodes": episodes,
            "count": len(episodes),
        }

    # ------------------------------------------------------------------
    # Camera JPEG Frames
    # ------------------------------------------------------------------
    def _valid_camera_ids(self) -> set[str]:
        valid = {"front", "wrist", "0", "1"}
        for cam in self.hardware.last_probe.get("cameras", []):
            cid = cam.get("id")
            if cid:
                valid.add(str(cid))
        return valid

    def release_cameras(self) -> None:
        self._camera_hub.release_all()

    def get_camera_jpeg(self, cam_id: str, theme: str = "light") -> bytes:
        if cam_id not in self._valid_camera_ids():
            raise KeyError(f"Camera '{cam_id}' not found.")

        alias_map = {"0": "front", "1": "wrist"}
        logical_id = alias_map.get(cam_id, cam_id)

        configured_raw: str | None = None
        if logical_id == "front":
            configured_raw = self.config.front_camera_id
        elif logical_id == "wrist":
            configured_raw = self.config.wrist_camera_id

        if (
            self.hardware.active_mode == "real"
            and self.active_workflow in ("idle", "calibrate", "teleop", "collect", "linear_teach")
            and (configured_raw is None or configured_raw.lower() != "none")
        ):
            for cam in self.hardware.last_probe.get("cameras", []):
                cam_raw = str(cam.get("raw_id", cam.get("index_or_path", "")))
                matched = (
                    cam_raw == configured_raw
                    if configured_raw is not None
                    else str(cam.get("id")) in (cam_id, logical_id)
                )
                if matched and cam.get("available"):
                    frame_bytes = self._camera_hub.get_frame(cam["index_or_path"])
                    if frame_bytes:
                        return frame_bytes

        return self.simulator.render_camera_jpeg(
            logical_id,
            mode=self.hardware.active_mode,
            active_workflow=self.active_workflow,
            theme=theme,
        )

    async def shutdown(self) -> None:
        self._camera_hub.release_all()
        self.bridge.disconnect(return_to_rest=False, keep_torque=False)
        self._stop_active_process()
        await self._cancel_background_task()
        self._flush_dry_run_episode()
        self.active_workflow = "idle"
        self.state = "idle"

    # ------------------------------------------------------------------
    # Safety Controls, Independent Dual-Arm Sim, IK, Presets & Demo Routines
    # ------------------------------------------------------------------
    async def trigger_estop(self, reason: str = "Web Studio E-Stop") -> dict[str, Any]:
        self._stop_active_process()
        await self._cancel_background_task()
        self.bridge.trigger_estop(reason=reason)
        self.simulator.estop_latched = True
        self.simulator.torque_enabled = False
        self.active_workflow = "idle"
        self.state = "estop"
        self.log(f"[SAFETY] E-STOP triggered: {reason}")
        return {
            "status": "estop_latched",
            "estop_latched": True,
            "torque_enabled": False,
            "reason": reason,
        }

    async def reset_estop(self) -> dict[str, Any]:
        self.bridge.reset_estop()
        self.simulator.estop_latched = False
        self.state = "idle"
        self.log("[SAFETY] E-STOP reset cleared.")
        return {
            "status": "ok",
            "estop_latched": False,
            "torque_enabled": self.simulator.torque_enabled,
        }

    async def set_torque(self, enabled: bool) -> dict[str, Any]:
        if enabled and self.simulator.estop_latched:
            raise WorkflowConflictError("急停已锁定，请先解除急停后再使能扭矩。", code="ESTOP_LATCHED")
        is_real = self.hardware.active_mode == "real"
        if is_real and self._active_proc_id is not None and self.process_registry.is_running(self._active_proc_id):
            raise WorkflowConflictError(
                f"当前正在运行 '{self.active_workflow}'（串口被占用），请先停止该任务再切换真机扭矩。",
                code="WORKFLOW_BUSY",
            )
        # Controller mode switch aligned with ros2_control architecture:
        # - Torque OFF while idle -> FREEDRIVE (real_to_sim: torque disabled + always-on 30Hz state monitor)
        # - Torque ON  while idle -> POSITION CONTROL (sim_to_real: zero-jump torque enable + UI drive + 30Hz monitor)
        if is_real and self.active_workflow == "idle":
            target_dir = "sim_to_real" if enabled else "real_to_sim"
            if self.bridge.direction != target_dir and self.bridge.direction != "teleop_bidirectional":
                try:
                    await asyncio.to_thread(
                        self.bridge.set_sync_mode,
                        target_dir,
                        is_real_mode=True,
                        safety_confirmed=bool(enabled),
                    )
                    hw_ok = True
                    hw_msg = (
                        "已使能从臂扭矩并切入「电脑控从臂」驱动模式（零跳变锁位 + 30Hz 监听）"
                        if enabled
                        else "已释放扭矩并切入「自由示教」监听模式（手掰从臂 → 3D 实时跟随）"
                    )
                except Exception as exc:
                    hw_ok, hw_msg = await asyncio.to_thread(
                        self.bridge.set_torque, enabled, is_real_mode=is_real
                    )
                    hw_msg = f"{hw_msg}（自动切换同步模式失败: {exc}）"
            else:
                hw_ok, hw_msg = await asyncio.to_thread(
                    self.bridge.set_torque, enabled, is_real_mode=is_real
                )
        else:
            hw_ok, hw_msg = await asyncio.to_thread(
                self.bridge.set_torque, enabled, is_real_mode=is_real
            )
        self.simulator.torque_enabled = bool(enabled)
        self._notify_motion()
        self.log(f"[SAFETY] Torque {'enabled' if enabled else 'disabled'}: {hw_msg}")
        return {
            "status": "ok",
            "torque_enabled": self.simulator.torque_enabled,
            "estop_latched": self.simulator.estop_latched,
            "hardware_applied": hw_ok,
            "message": hw_msg,
        }

    def set_linked_arms(self, linked: bool) -> dict[str, Any]:
        val = self.simulator.set_linked_arms(linked)
        if val:
            self.bridge.disable_leader_torque()
        self._notify_motion()
        self.log(f"Leader-Follower link {'enabled' if val else 'disabled'}.")
        return {
            "status": "ok",
            "linked_arms": val,
        }

    def set_sim_joints(
        self,
        *,
        arm: Literal["leader", "follower", "both"],
        raw_joints: dict[str, Any],
        linked: bool | None = None,
    ) -> dict[str, Any]:
        self._arbitrate_manual_write(allow_dry_calibrate=True)
        if self.simulator.estop_latched and arm in ("follower", "both"):
            raise WorkflowConflictError("急停已锁定 (E-STOP)，无法驱动从臂。", code="ESTOP_LATCHED")
        if linked is not None:
            self.simulator.set_linked_arms(linked)
        validated = validate_joint_dict(raw_joints, require_non_empty=True)
        in_dry_cal = self.active_workflow == "calibrate" and self.hardware.active_mode == "dry_run"
        leader, follower = self.simulator.set_arm_joints(
            arm,
            validated,
            step_immediately=not in_dry_cal,
        )
        if in_dry_cal:
            cal_target = self._cal_state.get("target", "follower")
            self._record_dry_cal_live_table(leader if cal_target == "leader" else follower)
        self._notify_motion()
        real_dispatched = False
        if not in_dry_cal:
            if arm in ("follower", "both") or (arm == "leader" and self.simulator.linked_arms):
                real_dispatched = self.bridge.send_follower_target_deg(follower)
            if arm == "leader" and not self.simulator.linked_arms:
                real_dispatched = self.bridge.send_leader_target_deg(validated)
            elif arm == "both" and not self.simulator.linked_arms:
                real_dispatched = self.bridge.send_leader_target_deg(validated) or real_dispatched
        return {
            "status": "ok",
            "arm": arm,
            "linked_arms": self.simulator.linked_arms,
            "leader_joints": leader,
            "follower_joints": follower,
            "leader_fk": compute_fk_details(leader),
            "follower_fk": compute_fk_details(follower),
            "real_dispatched": real_dispatched,
        }

    def apply_ik_target(
        self,
        *,
        arm: Literal["leader", "follower", "both"],
        x_mm: float,
        y_mm: float,
        z_mm: float,
        pitch_deg: float | None = None,
        gripper: float | None = None,
        clamp_to_workspace: bool = False,
    ) -> dict[str, Any]:
        self._arbitrate_manual_write(allow_dry_calibrate=False)
        if self.simulator.estop_latched and (
            arm in ("follower", "both") or (arm == "leader" and self.simulator.linked_arms)
        ):
            raise WorkflowConflictError("急停已锁定 (E-STOP)，无法执行 IK 驱动。", code="ESTOP_LATCHED")
        leader_cur, follower_cur, _ = self.simulator.snapshot()
        base_joints = leader_cur if arm == "leader" else follower_cur
        solved, clamped, boundary_msg = solve_ik_target_with_status(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            current_joints=base_joints,
            pitch_deg=pitch_deg,
            gripper=gripper,
            clamp_to_workspace=clamp_to_workspace,
        )
        effective_arm: Literal["leader", "follower", "both"] = (
            "both" if (arm == "leader" and self.simulator.linked_arms) else arm
        )
        leader, follower = self.simulator.set_arm_joints(effective_arm, solved, step_immediately=True)
        self._notify_motion()
        real_dispatched = False
        if effective_arm in ("follower", "both"):
            real_dispatched = self.bridge.send_follower_target_deg(follower)
        if effective_arm == "leader" and not self.simulator.linked_arms:
            real_dispatched = self.bridge.send_leader_target_deg(leader)
        return {
            "status": "ok",
            "arm": arm,
            "clamped": clamped,
            "boundary_message": boundary_msg if clamped else "",
            "solved_joints": solved,
            "leader_joints": leader,
            "follower_joints": follower,
            "leader_fk": compute_fk_details(leader),
            "follower_fk": compute_fk_details(follower),
            "real_dispatched": real_dispatched,
        }

    def set_sync_mode(
        self,
        *,
        direction: SyncDirection,
        safety_confirmed: bool = False,
    ) -> dict[str, Any]:
        res = self.bridge.set_sync_mode(
            direction,
            is_real_mode=self.hardware.active_mode == "real",
            safety_confirmed=safety_confirmed,
        )
        self.log(f"[SYNC] Bidirectional sync mode set to '{direction}'.")
        return res

    def send_control_key(self, key: str, step_mm: float = 8.0) -> dict[str, Any]:
        self._arbitrate_manual_write()
        res = self.simulator.step_cartesian_key(key, step_mm=step_mm)
        self._notify_motion()
        if res.get("action") == "estop":
            self.bridge.trigger_estop("Keyboard SPACE E-Stop")
            self.log("[SAFETY] Keyboard SPACE triggered E-STOP.")
        else:
            self.bridge.send_follower_target_deg(res["follower_joints"])
        return res

    def _load_preset_pose_deg(self, pose_name: str) -> dict[str, float]:
        try:
            from core.data_types import JointUnit as SwUnit
            from core.presets import get_preset_pose

            raw_deg = get_preset_pose(pose_name, unit=SwUnit.DEGREE)
            out = default_joints()
            for k in JOINT_NAMES[:5]:
                if k in raw_deg:
                    out[k] = round(max(-180.0, min(180.0, float(raw_deg[k]))), 2)
            if "gripper" in raw_deg:
                out["gripper"] = round(max(0.0, min(100.0, float(raw_deg["gripper"]))), 2)
            return out
        except Exception:
            fallback: dict[str, dict[str, float]] = {
                "home": default_joints(),
                "rest": {
                    "shoulder_pan": 0.0,
                    "shoulder_lift": -63.0,
                    "elbow_flex": 77.0,
                    "wrist_flex": 12.0,
                    "wrist_roll": 0.0,
                    "gripper": 5.0,
                },
                "inspect": {
                    "shoulder_pan": 0.0,
                    "shoulder_lift": 20.0,
                    "elbow_flex": 15.0,
                    "wrist_flex": -35.0,
                    "wrist_roll": 0.0,
                    "gripper": 20.0,
                },
                "pick_hover": {
                    "shoulder_pan": 0.0,
                    "shoulder_lift": 28.0,
                    "elbow_flex": 23.0,
                    "wrist_flex": -52.0,
                    "wrist_roll": 0.0,
                    "gripper": 75.0,
                },
                "wave": {
                    "shoulder_pan": 0.0,
                    "shoulder_lift": 37.0,
                    "elbow_flex": -17.0,
                    "wrist_flex": 6.0,
                    "wrist_roll": 40.0,
                    "gripper": 30.0,
                },
            }
            if pose_name not in fallback:
                raise ValueError(f"Unknown preset pose '{pose_name}'.")
            return fallback[pose_name]

    async def run_preset_pose(self, pose_name: str, duration_s: float = 1.2) -> dict[str, Any]:
        if self.simulator.estop_latched:
            raise WorkflowConflictError("急停已锁定，请先解除急停。", code="ESTOP_LATCHED")
        await self._ensure_idle()
        await self._ensure_real_drive_bridge()
        target = self._load_preset_pose_deg(pose_name)
        self.active_workflow = "demo_routine"
        self.state = "running"
        self.log(f"[PRESET] Moving to '{pose_name}' (S-curve, {duration_s:.1f}s)...")

        async def _animate() -> None:
            steps = max(8, int(duration_s * 20))
            _, start_j, _ = self.simulator.snapshot()
            try:
                for i in range(1, steps + 1):
                    alpha = 0.5 * (1.0 - math.cos(math.pi * (i / steps)))
                    interp = {
                        k: round(start_j[k] + (target[k] - start_j[k]) * alpha, 2)
                        for k in JOINT_NAMES
                    }
                    self.simulator.set_follower_joints(interp)
                    if self.simulator.linked_arms:
                        self.simulator.set_leader_joints(interp, step_immediately=False)
                    self.bridge.send_follower_target_deg(interp)
                    await asyncio.sleep(max(0.02, duration_s / steps))
                if pose_name == "rest":
                    self.simulator.torque_enabled = False
                    self.bridge.set_torque(False)
                    self.log("[PRESET] Reached rest pose & disabled torque.")
                else:
                    self.log(f"[PRESET] Reached '{pose_name}'.")
            except asyncio.CancelledError:
                raise
            finally:
                if self.active_workflow == "demo_routine":
                    self.active_workflow = "idle"
                    self.state = "idle"

        self._task = asyncio.create_task(_animate())
        return {"status": "started", "pose": pose_name, "target_joints": target}

    async def run_demo_routine(self, routine: Literal["wave", "pick"]) -> dict[str, Any]:
        if self.simulator.estop_latched:
            raise WorkflowConflictError("急停已锁定，请先解除急停。", code="ESTOP_LATCHED")
        await self._ensure_idle()
        await self._ensure_real_drive_bridge()
        self.active_workflow = "demo_routine"
        self.state = "running"
        self.log(f"[DEMO] Starting routine '{routine}'...")

        if routine == "wave":
            w_base = self._load_preset_pose_deg("wave")
            w_left = {**w_base, "wrist_roll": -45.0, "shoulder_pan": -15.0}
            w_right = {**w_base, "wrist_roll": 45.0, "shoulder_pan": 15.0}
            seq = [w_base, w_left, w_right, w_left, w_right, self._load_preset_pose_deg("rest")]
        else:
            hover = self._load_preset_pose_deg("pick_hover")
            grasp = {**hover, "shoulder_lift": hover["shoulder_lift"] + 12.0, "gripper": 12.0}
            lift = {**hover, "gripper": 12.0}
            place = {**hover, "shoulder_pan": 35.0, "gripper": 75.0}
            seq = [hover, grasp, lift, place, self._load_preset_pose_deg("rest")]

        async def _run_seq() -> None:
            try:
                for tgt in seq:
                    _, start_j, _ = self.simulator.snapshot()
                    for i in range(1, 9):
                        alpha = 0.5 * (1.0 - math.cos(math.pi * (i / 8)))
                        interp = {
                            k: round(start_j[k] + (tgt[k] - start_j[k]) * alpha, 2)
                            for k in JOINT_NAMES
                        }
                        self.simulator.set_follower_joints(interp)
                        if self.simulator.linked_arms:
                            self.simulator.set_leader_joints(interp, step_immediately=False)
                        self.bridge.send_follower_target_deg(interp)
                        await asyncio.sleep(0.03)
                self.simulator.torque_enabled = False
                self.bridge.set_torque(False)
                self.log(f"[DEMO] Routine '{routine}' finished (parked at rest, torque disabled).")
            except asyncio.CancelledError:
                raise
            finally:
                if self.active_workflow == "demo_routine":
                    self.active_workflow = "idle"
                    self.state = "idle"

        self._task = asyncio.create_task(_run_seq())
        return {"status": "started", "routine": routine, "steps": len(seq)}

    def save_custom_pose(self, target: Literal["home", "rest"]) -> dict[str, Any]:
        _, follower, _ = self.simulator.snapshot()
        filename = "upright_pose.json" if target == "home" else "rest_pose.json"
        if self.hardware.active_mode == "real":
            out_path = self.config.workspace_root / "software" / filename
            rel_path = f"software/{filename}"
        else:
            out_path = self.config.dry_run_dir / filename
            rel_path = f"data/dry_run/{filename}"
        payload = {
            "unit": "degree",
            "gripper_unit": "range_0_100",
            "joints": {f"{k}.pos": follower[k] for k in JOINT_NAMES},
        }
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        self.log(f"[POSE] Saved current follower pose to {rel_path}.")
        return {"status": "saved", "target": target, "path": rel_path, "joints": follower}

    def get_episode_frames(self, dataset_name: str, episode_index: int) -> dict[str, Any]:
        if ".." in dataset_name or "/" in dataset_name or "\\" in dataset_name:
            raise ValueError("Invalid dataset_name.")

        # 1. Check real official LeRobotDataset v3.0 folder under data/datasets/<dataset_name>/
        real_ds = self.config.datasets_dir / dataset_name
        info_p = real_ds / "meta" / "info.json"
        if info_p.is_file():
            info = json.loads(info_p.read_text(encoding="utf-8"))
            fps = int(info.get("fps", 30))
            frames: list[dict[str, Any]] = []
            task_name = ""
            parquet_files = sorted((real_ds / "data").glob("chunk-*/file-*.parquet"))
            if parquet_files:
                import pyarrow.parquet as pq

                for pf in parquet_files:
                    tbl = pq.read_table(pf)
                    for row in tbl.to_pylist():
                        if int(row.get("episode_index", -1)) != episode_index:
                            continue
                        obs = row.get("observation.state") or [0.0] * 6
                        act = row.get("action") or [0.0] * 6
                        frames.append(
                            {
                                "frame_index": int(row.get("frame_index", len(frames))),
                                "timestamp": round(float(row.get("timestamp", len(frames) / max(1, fps))), 4),
                                "observation.state": [round(float(v), 3) for v in obs],
                                "action": [round(float(v), 3) for v in act],
                            }
                        )
            # Extract task description from meta/episodes if available
            for mf in sorted((real_ds / "meta" / "episodes").glob("chunk-*/file-*.parquet")):
                try:
                    import pyarrow.parquet as pq

                    for row in pq.read_table(mf).to_pylist():
                        if int(row.get("episode_index", -1)) == episode_index:
                            tval = row.get("tasks")
                            if isinstance(tval, list) and tval:
                                task_name = str(tval[0])
                            elif isinstance(tval, str):
                                task_name = tval
                            break
                except Exception:
                    pass
            # Discover video URLs
            video_urls: dict[str, str] = {}
            vid_root = real_ds / "videos"
            if vid_root.is_dir():
                for cam_dir in sorted(vid_root.iterdir()):
                    if not cam_dir.is_dir() or not cam_dir.name.startswith("observation.images."):
                        continue
                    cam_key = cam_dir.name.removeprefix("observation.images.")
                    mp4s = sorted(cam_dir.glob("chunk-*/file-*.mp4"))
                    if mp4s:
                        video_urls[cam_key] = (
                            f"/api/workflows/collect/video/{dataset_name}/{cam_key}/{episode_index}"
                        )
            return {
                "dataset": dataset_name,
                "episode_index": episode_index,
                "task": task_name,
                "fps": fps,
                "num_frames": len(frames),
                "source": "real",
                "video_urls": video_urls,
                "frames": frames,
            }

        # 2. Fallback to dry_run episode JSON
        ep_file = (
            self.config.dry_run_dir
            / "datasets"
            / dataset_name
            / "episodes"
            / f"episode_{episode_index:04d}.json"
        )
        if not ep_file.is_file():
            raise FileNotFoundError(f"Episode {episode_index} not found in dataset '{dataset_name}'.")
        data = json.loads(ep_file.read_text(encoding="utf-8"))
        data.setdefault("source", "dry_run")
        data.setdefault("video_urls", {})
        return data

    def get_episode_video_path(
        self, dataset_name: str, cam_key: str, episode_index: int
    ) -> Path:
        for seg in (dataset_name, cam_key):
            if ".." in seg or "/" in seg or "\\" in seg:
                raise ValueError("Invalid dataset or camera name.")
        cam_dir = (
            self.config.datasets_dir
            / dataset_name
            / "videos"
            / f"observation.images.{cam_key}"
        )
        if not cam_dir.is_dir():
            raise FileNotFoundError(f"No videos found for camera '{cam_key}'.")
        mp4s = sorted(cam_dir.glob("chunk-*/file-*.mp4"))
        if not mp4s:
            raise FileNotFoundError(f"No MP4 files for '{dataset_name}/{cam_key}'.")
        # In LeRobot v3.0 multiple episodes may share file-000.mp4; pick indexed or first available
        return mp4s[episode_index] if episode_index < len(mp4s) else mp4s[0]

    def delete_episode(self, dataset_name: str, episode_index: int) -> dict[str, Any]:
        if ".." in dataset_name or "/" in dataset_name or "\\" in dataset_name:
            raise ValueError("Invalid dataset_name.")
        dry_ep = (
            self.config.dry_run_dir
            / "datasets"
            / dataset_name
            / "episodes"
            / f"episode_{episode_index:04d}.json"
        )
        if dry_ep.is_file():
            dry_ep.unlink()
            self.log(f"[COLLECT] Deleted dry_run episode {dataset_name} #{episode_index}")
            return {"status": "deleted", "dataset": dataset_name, "episode_index": episode_index}
        real_ds = self.config.datasets_dir / dataset_name
        if real_ds.is_dir():
            # For LeRobot v3.0 Parquet chunks, removing the dataset folder cleanly avoids corrupting episode indices
            shutil.rmtree(real_ds, ignore_errors=True)
            self.log(f"[COLLECT] Deleted real dataset folder {dataset_name}")
            return {
                "status": "deleted_dataset",
                "dataset": dataset_name,
                "episode_index": episode_index,
            }
        raise FileNotFoundError(f"Episode {episode_index} not found in '{dataset_name}'.")
