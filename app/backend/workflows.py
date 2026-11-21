"""Workflow orchestration for SO-101 calibration, teleop, teaching, data collection, and cameras."""

from __future__ import annotations

import asyncio
import json
import math
import re
import sys
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
    encoder_to_deg,
    solve_ik_target,
    validate_joint_dict,
)


WorkflowName = Literal["idle", "calibrate", "teleop", "teaching_replay", "collect", "demo_routine"]


class WorkflowConflictError(Exception):
    """Raised when an operation conflicts with the active workflow or empty state."""

    def __init__(self, detail: str, code: str = "WORKFLOW_CONFLICT") -> None:
        super().__init__(detail)
        self.detail = detail
        self.code = code


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
        self.bridge = SO101RealBridge(config, simulator)

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
        self._dry_cal_queue: asyncio.Queue[str] | None = None
        self._cal_state: dict[str, Any] = self._empty_cal_state()
        self._waypoints: list[dict[str, Any]] = []
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
            "current_episode": self.current_episode,
            "target_episodes": self.target_episodes,
            "current_step": self.current_step,
            "waypoint_count": len(self._waypoints),
            "calibration": self.get_calibration_snapshot(),
            "last_error": self.last_error,
            "logs": list(self.logs),
        }

    def get_telemetry_frame(self) -> dict[str, Any]:
        self._sync_process_state()
        leader, follower, sim_steps = self.simulator.snapshot()
        leader_fk = compute_fk_details(leader)
        follower_fk = compute_fk_details(follower)
        bridge_st = self.bridge.status()
        hw_ready = bool(self.hardware.last_probe.get("hardware_ready", False))
        return {
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
    ) -> list[str]:
        f_cal = str(self.config.calibration_dir / "robots" / "so_follower")
        l_cal = str(self.config.calibration_dir / "teleoperators" / "so_leader")
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
            f"--dataset.root={self.config.datasets_dir / dataset_name}",
            f"--dataset.single_task={single_task}",
            f"--dataset.num_episodes={num_episodes}",
            f"--dataset.episode_time_s={episode_time_s}",
            f"--dataset.reset_time_s={reset_time_s}",
            f"--dataset.fps={fps}",
            "--dataset.push_to_hub=false",
            "--play_sounds=false",
            "--display_data=false",
        ]
        cams = self.build_cameras_dict()
        if cams:
            cmd.append(f"--robot.cameras={json.dumps(cams, separators=(',', ':'))}")
        return cmd

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
        if "[CAL_PROMPT]" in line:
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
                    "第 1 步（中位归零）：请用手将机械臂所有关节摆到活动范围正中间，"
                    "然后点击【确认 / 下一步 (Enter)】。"
                )
            else:
                self._cal_state["stage"] = "waiting_input"
                self._cal_state["allow_recalibrate"] = False
                self._cal_state["prompt"] = raw_prompt
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
            "【模拟标定 1/2】请摆好中位，然后点击【确认 / 下一步 (Enter)】。"
        )
        self._dry_cal_queue = asyncio.Queue()
        cal_queue = self._dry_cal_queue
        self.log(f"Started dry_run calibration for {target} ({resolved_id}).")

        async def _sim_calibrate() -> None:
            try:
                await cal_queue.get()

                self._cal_state["stage"] = "recording_range"
                self._cal_state["waiting_for_input"] = True
                self._cal_state["prompt"] = (
                    "【模拟标定 2/2】正在模拟活动各关节限位，点击【确认 / 下一步 (Enter)】保存。"
                )
                self.current_step = 2
                tick = 0
                while True:
                    for idx, jname in enumerate(JOINT_NAMES):
                        pos = int(2048 + math.sin(tick * 0.35 + idx) * 600)
                        prev = self._cal_state["live_table"].get(jname)
                        vmin = min(int(prev["min"]), pos) if prev else pos
                        vmax = max(int(prev["max"]), pos) if prev else pos
                        self._cal_state["live_table"][jname] = {
                            "name": jname,
                            "min": str(vmin),
                            "pos": str(pos),
                            "max": str(vmax),
                        }
                    tick += 1
                    try:
                        await asyncio.wait_for(cal_queue.get(), timeout=0.15)
                        break
                    except asyncio.TimeoutError:
                        continue

                self.current_step = 3
                cal_dir = self.config.dry_run_dir / "calibration"
                cal_dir.mkdir(parents=True, exist_ok=True)
                cal_file = cal_dir / f"{target}_{resolved_id}.json"
                cal_payload = {
                    "target": target,
                    "arm_id": resolved_id,
                    "port": resolved_port,
                    "mode": "dry_run",
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "joints": {
                        j: {"homing_offset": 0, "range_min": 0, "range_max": 4095}
                        for j in JOINT_NAMES
                    },
                }
                cal_file.write_text(
                    json.dumps(cal_payload, indent=2, ensure_ascii=False), encoding="utf-8"
                )
                self.simulator.reset()
                self._cal_state["stage"] = "completed"
                self._cal_state["waiting_for_input"] = False
                self._cal_state["saved_path"] = str(cal_file)
                self._cal_state["prompt"] = f"模拟标定完成 -> {cal_file.name}"
                self.log(f"[dry_run] Calibration complete -> {cal_file.name}")
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

    async def stop_calibrate(self) -> dict[str, Any]:
        if self.active_workflow == "calibrate":
            self._stop_active_process()
            await self._cancel_background_task()
            self._dry_cal_queue = None
            self.active_workflow = "idle"
            self.state = "idle"
            self._cal_state["active"] = False
            self._cal_state["waiting_for_input"] = False
            self._cal_state["stage"] = "stopped"
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
            self.bridge.disconnect(return_to_rest=False, keep_torque=False)
            proc_id = "workflow_teleop"
            self.process_registry.spawn(
                proc_id, "lerobot_teleoperate", cmd, cwd=self.config.workspace_root, on_line=self.log
            )
            self._active_proc_id = proc_id
            self.active_workflow = "teleop"
            self.state = "running"
            self.log(f"Started real teleoperation at {fps} FPS.")
            return {
                "status": "started",
                "workflow": "teleop",
                "mode": "real",
                "fps": fps,
                "command": cmd,
            }

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
            self.log("Teleoperation stopped.")
        return {"status": "stopped", "workflow": "teleop"}

    def update_teleop_joints(self, raw_joints: dict[str, Any]) -> dict[str, Any]:
        validated = validate_joint_dict(raw_joints, require_non_empty=True)
        self.simulator.set_leader_joints(validated, step_immediately=True)
        leader, follower, _ = self.simulator.snapshot()
        return {
            "status": "ok",
            "leader_joints": leader,
            "follower_joints": follower,
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

    async def replay_waypoints(self, *, speed: float = 1.0, loop: bool = False) -> dict[str, Any]:
        self._sync_process_state()
        if len(self._waypoints) == 0:
            raise WorkflowConflictError(
                "No waypoints recorded to replay.", code="NO_WAYPOINTS"
            )
        await self._ensure_idle()
        self.last_error = None
        self.active_workflow = "teaching_replay"
        self.state = "running"
        self.current_step = 0
        waypoints_snapshot = list(self._waypoints)
        self.log(f"Replaying {len(waypoints_snapshot)} waypoints (speed={speed}x).")

        async def _run_replay() -> None:
            substeps = max(4, int(8 / max(0.1, speed)))
            try:
                while True:
                    for wp in waypoints_snapshot:
                        _, start_j, _ = self.simulator.snapshot()
                        target_j = wp["joints"]
                        for s in range(1, substeps + 1):
                            t = s / substeps
                            interp = {
                                k: round(
                                    start_j[k] + (float(target_j[k]) - start_j[k]) * t, 3
                                )
                                for k in JOINT_NAMES
                            }
                            self.simulator.set_leader_joints(interp, step_immediately=False)
                            self.simulator.set_follower_joints(interp)
                            self.current_step += 1
                            await asyncio.sleep(0.02)
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
        self.log("Cleared all teaching waypoints.")
        return {"status": "cleared", "count": 0}

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

    async def start_collect(
        self,
        *,
        dataset_name: str,
        single_task: str,
        num_episodes: int = 2,
        episode_time_s: float = 5.0,
        reset_time_s: float = 1.0,
        fps: int = 30,
    ) -> dict[str, Any]:
        self._sync_process_state()
        await self._ensure_idle()
        self.last_error = None

        if self.hardware.active_mode == "real":
            self.bridge.disconnect(return_to_rest=False, keep_torque=False)
            cmd = self.build_record_cmd(
                dataset_name=dataset_name,
                single_task=single_task,
                num_episodes=num_episodes,
                episode_time_s=episode_time_s,
                reset_time_s=reset_time_s,
                fps=fps,
            )
            proc_id = "workflow_collect"
            self.process_registry.spawn(
                proc_id, "lerobot_record", cmd, cwd=self.config.workspace_root, on_line=self.log
            )
            self._active_proc_id = proc_id
            self.active_workflow = "collect"
            self.state = "running"
            self.current_episode = 0
            self.target_episodes = num_episodes
            out_dir = self.config.datasets_dir / dataset_name
            self.log(f"Started real data collection -> {out_dir}")
            return {
                "status": "started",
                "workflow": "collect",
                "mode": "real",
                "dataset_name": dataset_name,
                "output_dir": str(out_dir),
                "command": cmd,
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
        self.log(
            f"Started dry_run collection '{dataset_name}' ({num_episodes} episodes @ {fps} FPS)."
        )

        async def _sim_collect() -> None:
            tick = max(0.03, min(0.1, 1.0 / max(1, fps)))
            frames_per_ep = max(5, min(60, int(episode_time_s * 5)))
            try:
                for ep_num in range(1, num_episodes + 1):
                    self.current_episode = ep_num
                    for frame_i in range(frames_per_ep):
                        self.simulator.step(tick)
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
                    saved = self._flush_dry_run_episode()
                    self.log(
                        f"[dry_run] Saved episode {ep_num}/{num_episodes} to {dataset_name} (total={saved})."
                    )
                    if ep_num < num_episodes:
                        await asyncio.sleep(min(0.3, reset_time_s))
            except asyncio.CancelledError:
                self._flush_dry_run_episode()
                self.log("[dry_run] Collection stopped and flushed.")
                raise
            finally:
                if self.active_workflow == "collect":
                    self.active_workflow = "idle"
                    self.state = "idle"

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
            self._stop_active_process()
            await self._cancel_background_task()
            saved_episodes = self._flush_dry_run_episode()
            self._collect_ctx = None
            self.active_workflow = "idle"
            self.state = "idle"
            self.log(f"Data collection stopped (saved={saved_episodes}).")
        return {
            "status": "stopped",
            "workflow": "collect",
            "saved_episodes": saved_episodes,
        }

    def list_episodes(self) -> dict[str, Any]:
        episodes: list[dict[str, Any]] = []
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
                        }
                    )
                except Exception:
                    continue
        try:
            root_str = str(dry_datasets.relative_to(self.config.workspace_root)).replace("\\", "/")
        except ValueError:
            root_str = str(dry_datasets)
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

    def get_camera_jpeg(self, cam_id: str) -> bytes:
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
            and self.active_workflow in ("idle", "calibrate")
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
                    try:
                        import cv2

                        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
                        with CAMERA_IO_LOCK:
                            cap = cv2.VideoCapture(cam["index_or_path"], backend)
                            try:
                                if cap.isOpened():
                                    ok, frame = cap.read()
                                    if ok and frame is not None:
                                        enc_ok, buf = cv2.imencode(".jpg", frame)
                                        if enc_ok:
                                            return bytes(buf)
                            finally:
                                cap.release()
                    except Exception:
                        pass

        return self.simulator.render_camera_jpeg(
            logical_id,
            mode=self.hardware.active_mode,
            active_workflow=self.active_workflow,
        )

    async def shutdown(self) -> None:
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
        self.bridge.set_torque(enabled)
        self.simulator.torque_enabled = bool(enabled)
        self.log(f"[SAFETY] Torque {'enabled' if enabled else 'disabled (sleep)'}.")
        return {
            "status": "ok",
            "torque_enabled": self.simulator.torque_enabled,
            "estop_latched": self.simulator.estop_latched,
        }

    def set_sim_joints(
        self,
        *,
        arm: Literal["leader", "follower", "both"],
        raw_joints: dict[str, Any],
        linked: bool | None = None,
    ) -> dict[str, Any]:
        if self.simulator.estop_latched and arm in ("follower", "both"):
            raise WorkflowConflictError("急停已锁定 (E-STOP)，无法驱动从臂。", code="ESTOP_LATCHED")
        if linked is not None:
            self.simulator.set_linked_arms(linked)
        validated = validate_joint_dict(raw_joints, require_non_empty=True)
        leader, follower = self.simulator.set_arm_joints(arm, validated, step_immediately=True)
        real_dispatched = False
        if arm in ("follower", "both") or (arm == "leader" and self.simulator.linked_arms):
            real_dispatched = self.bridge.send_follower_target_deg(follower)
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
    ) -> dict[str, Any]:
        if self.simulator.estop_latched and arm in ("follower", "both"):
            raise WorkflowConflictError("急停已锁定 (E-STOP)，无法执行 IK 驱动。", code="ESTOP_LATCHED")
        leader_cur, follower_cur, _ = self.simulator.snapshot()
        base_joints = leader_cur if arm == "leader" else follower_cur
        solved = solve_ik_target(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            current_joints=base_joints,
            pitch_deg=pitch_deg,
            gripper=gripper,
        )
        leader, follower = self.simulator.set_arm_joints(arm, solved, step_immediately=True)
        real_dispatched = False
        if arm in ("follower", "both"):
            real_dispatched = self.bridge.send_follower_target_deg(follower)
        return {
            "status": "ok",
            "arm": arm,
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
        res = self.simulator.step_cartesian_key(key, step_mm=step_mm)
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
        out_path = self.config.workspace_root / "software" / filename
        payload = {
            "unit": "degree",
            "joints": {f"{k}.pos": follower[k] for k in JOINT_NAMES},
        }
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        self.log(f"[POSE] Saved current follower pose to software/{filename}.")
        return {"status": "saved", "target": target, "path": f"software/{filename}", "joints": follower}

    def get_episode_frames(self, dataset_name: str, episode_index: int) -> dict[str, Any]:
        if ".." in dataset_name or "/" in dataset_name or "\\" in dataset_name:
            raise ValueError("Invalid dataset_name.")
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
        return data
