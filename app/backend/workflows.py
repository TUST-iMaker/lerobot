"""Workflow orchestration for SO-101 calibration, teleop, teaching, data collection, and cameras."""

from __future__ import annotations

import asyncio
import json
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from app.backend.config import StudioConfig
from app.backend.hardware import HardwareManager
from app.backend.process_manager import ProcessRegistry
from app.backend.simulation import (
    JOINT_NAMES,
    Arm101Simulator,
    default_joints,
    validate_joint_dict,
)


WorkflowName = Literal["idle", "calibrate", "teleop", "teaching_replay", "collect"]


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
        self._waypoints: list[dict[str, Any]] = []
        self._load_waypoints()

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
            "last_error": self.last_error,
            "logs": list(self.logs),
        }

    def get_telemetry_frame(self) -> dict[str, Any]:
        self._sync_process_state()
        leader, follower, sim_steps = self.simulator.snapshot()
        return {
            "timestamp": round(time.time(), 3),
            "mode": self.hardware.active_mode,
            "active_workflow": self.active_workflow,
            "leader_joints": leader,
            "follower_joints": follower,
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
    def build_calibrate_cmd(
        self,
        *,
        target: Literal["follower", "leader"],
        port: str | None = None,
        arm_id: str | None = None,
        calibration_dir: str | None = None,
    ) -> list[str]:
        """Build lerobot-calibrate CLI args (strictly ONE of --robot.* or --teleop.*)."""
        if target == "follower":
            resolved_port = port or self.config.follower_port
            resolved_id = arm_id or self.config.follower_id
            resolved_dir = calibration_dir or str(
                self.config.calibration_dir / "robots" / "so_follower"
            )
            return [
                sys.executable,
                "-m",
                "lerobot.scripts.lerobot_calibrate",
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
            "lerobot.scripts.lerobot_calibrate",
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
    ) -> list[str]:
        f_port = follower_port or self.config.follower_port
        l_port = leader_port or self.config.leader_port
        f_cal = str(self.config.calibration_dir / "robots" / "so_follower")
        l_cal = str(self.config.calibration_dir / "teleoperators" / "so_leader")
        return [
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
        return [
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

    # ------------------------------------------------------------------
    # Calibrate
    # ------------------------------------------------------------------
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
        cmd = self.build_calibrate_cmd(
            target=target, port=port, arm_id=arm_id, calibration_dir=calibration_dir
        )

        if self.hardware.active_mode == "real":
            proc_id = "workflow_calibrate"
            self.process_registry.spawn(
                proc_id, "lerobot_calibrate", cmd, cwd=self.config.workspace_root, on_line=self.log
            )
            self._active_proc_id = proc_id
            self.active_workflow = "calibrate"
            self.state = "running"
            self.log(f"Started real calibration for {target}.")
            return {
                "status": "started",
                "workflow": "calibrate",
                "mode": "real",
                "target": target,
                "command": cmd,
            }

        self.active_workflow = "calibrate"
        self.state = "running"
        self.current_step = 0
        resolved_id = arm_id or (
            self.config.follower_id if target == "follower" else self.config.leader_id
        )
        self.log(f"Started dry_run calibration for {target} ({resolved_id}).")

        async def _sim_calibrate() -> None:
            try:
                for step_idx in range(1, 4):
                    await asyncio.sleep(0.25)
                    self.current_step = step_idx
                    self.log(f"[dry_run] Calibrating {target} step {step_idx}/3...")
                cal_dir = self.config.dry_run_dir / "calibration"
                cal_dir.mkdir(parents=True, exist_ok=True)
                cal_file = cal_dir / f"{target}_{resolved_id}.json"
                cal_payload = {
                    "target": target,
                    "arm_id": resolved_id,
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
                self.log(f"[dry_run] Calibration complete -> {cal_file.name}")
            except asyncio.CancelledError:
                self.log("[dry_run] Calibration cancelled.")
                raise
            finally:
                if self.active_workflow == "calibrate":
                    self.active_workflow = "idle"
                    self.state = "idle"

        self._task = asyncio.create_task(_sim_calibrate())
        return {
            "status": "started",
            "workflow": "calibrate",
            "mode": "dry_run",
            "target": target,
            "command": cmd,
        }

    async def stop_calibrate(self) -> dict[str, Any]:
        if self.active_workflow == "calibrate":
            self._stop_active_process()
            await self._cancel_background_task()
            self.active_workflow = "idle"
            self.state = "idle"
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
    ) -> dict[str, Any]:
        self._sync_process_state()
        await self._ensure_idle()
        self.last_error = None
        cmd = self.build_teleop_cmd(
            fps=fps,
            follower_port=follower_port,
            leader_port=leader_port,
            display_data=display_data,
        )

        if self.hardware.active_mode == "real":
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

        if self.hardware.active_mode == "real":
            for cam in self.hardware.last_probe.get("cameras", []):
                if str(cam.get("id")) in (cam_id, logical_id) and cam.get("available"):
                    try:
                        import cv2

                        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
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
        self._stop_active_process()
        await self._cancel_background_task()
        self._flush_dry_run_episode()
        self.active_workflow = "idle"
        self.state = "idle"
