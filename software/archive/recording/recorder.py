"""SO-ARM101 具身机械臂示教轨迹记录器 (EpisodeRecorder).

职责:
1. 严格使用 core.data_types 中的统一数据契约 (JointState, Pose3D, RobotAction, RobotObservation, ControlSource);
2. 逐帧记录包含机械臂观测 (关节角度、扭矩状态、TCP位姿) 与动作意图 (目标、来源、单位) 的标准时序数据;
3. 按照标准 JSONL 格式持久化落盘至 recording/data/<task_name>/episode_XXXXXX.jsonl，便于后续直接转换为 LeRobotDataset.
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from core.data_types import (
    ControlSource,
    JointState,
    Pose3D,
    RobotAction,
    RobotObservation,
)

logger = logging.getLogger(__name__)


class EpisodeRecorder:
    """机械臂示教轨迹记录器."""

    def __init__(
        self,
        base_dir: Path | str | None = None,
        is_sim: bool = False,
    ) -> None:
        if base_dir is None:
            root_dir = (Path(__file__).resolve().parent / "data").resolve()
        else:
            root_dir = Path(base_dir).resolve()

        self._is_sim = bool(is_sim)
        self.env_type = "sim" if self._is_sim else "real"

        # 若传入的目录末尾未区分 sim 或 real，自动按环境隔离至子目录
        if root_dir.name in ("sim", "real"):
            self.base_dir = root_dir
            self._is_sim = (root_dir.name == "sim")
            self.env_type = root_dir.name
        else:
            self.base_dir = (root_dir / self.env_type).resolve()

        self.base_dir.mkdir(parents=True, exist_ok=True)

        self._is_recording: bool = False
        self._current_file: Path | None = None
        self._file_handle: Any = None
        self._task_name: str = "default_task"
        self._episode_id: int = 0
        self._frame_count: int = 0
        self._start_time_monotonic: float = 0.0
        self._start_time_wall: float = 0.0

    @property
    def is_recording(self) -> bool:
        """是否处于正在录制状态."""
        return self._is_recording

    @property
    def is_sim(self) -> bool:
        """是否处于仿真示教模式 (False 代表真实物理机械臂)."""
        return self._is_sim

    @property
    def current_frame_count(self) -> int:
        """当前 episode 已录制帧数."""
        return self._frame_count

    @property
    def current_duration_s(self) -> float:
        """当前 episode 录制持续时长 (秒)."""
        if not self._is_recording:
            return 0.0
        return max(0.0, time.monotonic() - self._start_time_monotonic)

    @property
    def current_file_path(self) -> Path | None:
        """当前录制文件的绝对路径."""
        return self._current_file

    @property
    def current_task_name(self) -> str:
        """当前录制任务名称."""
        return self._task_name

    def _next_episode_id(self, task_dir: Path) -> int:
        """扫描任务目录并计算自增 episode_id."""
        existing = list(task_dir.glob("episode_*.jsonl"))
        if not existing:
            return 0
        ids = []
        for p in existing:
            stem = p.stem  # episode_000003
            parts = stem.split("_")
            if len(parts) == 2 and parts[1].isdigit():
                ids.append(int(parts[1]))
        return max(ids) + 1 if ids else 0

    def start_episode(
        self,
        task_name: str = "teleop_demo",
        metadata: dict[str, Any] | None = None,
    ) -> Path:
        """开始新的示教 Episode 录制."""
        if self._is_recording:
            logger.warning("录制已处于活动状态，先自动关闭当前 Episode")
            self.stop_episode()

        self._task_name = task_name.strip() or "teleop_demo"
        task_dir = self.base_dir / self._task_name
        task_dir.mkdir(parents=True, exist_ok=True)

        self._episode_id = self._next_episode_id(task_dir)
        filename = f"episode_{self._episode_id:06d}.jsonl"
        self._current_file = task_dir / filename

        self._file_handle = open(self._current_file, "w", encoding="utf-8")
        self._frame_count = 0
        self._start_time_monotonic = time.monotonic()
        self._start_time_wall = time.time()
        self._is_recording = True

        # 写入 Episode 头元数据行
        header = {
            "type": "episode_metadata",
            "task_name": self._task_name,
            "episode_id": self._episode_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "start_time_wall": self._start_time_wall,
            "robot_type": "so_arm101",
            "env_type": self.env_type,
            "is_sim": self._is_sim,
            "metadata": metadata or {},
        }
        self._file_handle.write(json.dumps(header, ensure_ascii=False) + "\n")
        self._file_handle.flush()

        logger.info(
            "[*] 开启示教轨迹录制 [%s]: Task='%s', Episode=%06d, 目标文件: %s",
            self.env_type.upper(),
            self._task_name,
            self._episode_id,
            self._current_file,
        )
        return self._current_file

    def record_step(
        self,
        joint_state: JointState,
        action: RobotAction | None = None,
        tcp_pose: Pose3D | None = None,
        control_source: ControlSource | None = None,
        camera_frames: dict[str, Any] | None = None,
        custom_data: dict[str, Any] | None = None,
    ) -> None:
        """录制单帧标准时序数据 (严格基于统一数据类型)."""
        if not self._is_recording or self._file_handle is None:
            return

        now_mono = time.monotonic()
        rel_time_s = round(now_mono - self._start_time_monotonic, 4)

        # 组装统一数据观测契约 RobotObservation (集成多相机引用)
        observation = RobotObservation(
            timestamp=joint_state.timestamp,
            joint_state=joint_state,
            tcp_state=tcp_pose,
            camera_frames=camera_frames or {},
        )

        source_val = (
            control_source.value
            if control_source
            else (action.source.value if action else ControlSource.UI.value)
        )

        frame_payload: dict[str, Any] = {
            "type": "frame",
            "frame_index": self._frame_count,
            "relative_time_s": rel_time_s,
            "timestamp": time.time(),
            "env_type": self.env_type,
            "is_sim": self._is_sim,
            "observation": observation.to_dict(),
            "action": action.to_dict() if action else None,
            "control_source": source_val,
        }
        if custom_data:
            frame_payload["custom"] = custom_data

        self._file_handle.write(json.dumps(frame_payload, ensure_ascii=False) + "\n")
        self._frame_count += 1

        # 定期刷新缓冲区 (每 15 帧 flush 一次以保证安全性)
        if self._frame_count % 15 == 0:
            self._file_handle.flush()

    def stop_episode(self) -> dict[str, Any]:
        """结束当前 Episode 录制并保存落盘."""
        if not self._is_recording:
            return {"success": False, "message": "当前未在录制"}

        duration = self.current_duration_s
        fps = round(self._frame_count / duration, 1) if duration > 0.05 else 0.0

        if self._file_handle:
            # 写入末尾摘要行
            summary_record = {
                "type": "episode_summary",
                "task_name": self._task_name,
                "episode_id": self._episode_id,
                "env_type": self.env_type,
                "is_sim": self._is_sim,
                "total_frames": self._frame_count,
                "duration_seconds": round(duration, 3),
                "average_fps": fps,
                "ended_at": datetime.now(timezone.utc).isoformat(),
            }
            self._file_handle.write(json.dumps(summary_record, ensure_ascii=False) + "\n")
            self._file_handle.flush()
            self._file_handle.close()
            self._file_handle = None

        saved_file = self._current_file
        file_size_kb = round(saved_file.stat().st_size / 1024.0, 1) if saved_file and saved_file.is_file() else 0.0

        self._is_recording = False
        logger.info(
            "[*] 示教轨迹录制完成 [%s]: %d 帧, 时长 %.2fs, 均值 %.1f FPS, 大小 %.1f KB -> %s",
            self.env_type.upper(),
            self._frame_count,
            duration,
            fps,
            file_size_kb,
            saved_file,
        )

        return {
            "success": True,
            "task_name": self._task_name,
            "episode_id": self._episode_id,
            "env_type": self.env_type,
            "is_sim": self._is_sim,
            "file_path": str(saved_file),
            "total_frames": self._frame_count,
            "duration_s": round(duration, 3),
            "average_fps": fps,
            "file_size_kb": file_size_kb,
        }


def scan_recorded_episodes(base_dir: Path | str | None = None) -> list[dict[str, Any]]:
    """扫描指定目录下所有已记录的 JSONL 示教文件并提取元数据摘要."""
    if base_dir is None:
        target_dir = (Path(__file__).resolve().parent / "data").resolve()
    else:
        target_dir = Path(base_dir).resolve()

    results: list[dict[str, Any]] = []
    if not target_dir.is_dir():
        return results

    for fpath in target_dir.glob("**/*.jsonl"):
        if not fpath.is_file():
            continue
        try:
            st = fpath.stat()
            mtime = st.st_mtime
            size_kb = round(st.st_size / 1024.0, 1)

            meta: dict[str, Any] = {}
            summary: dict[str, Any] = {}
            frame_cnt = 0

            with open(fpath, "r", encoding="utf-8") as f:
                first_line = f.readline().strip()
                if first_line:
                    try:
                        meta = json.loads(first_line)
                    except Exception:
                        pass
                last_line = ""
                for line in f:
                    if line.strip():
                        last_line = line.strip()
                        frame_cnt += 1

                if last_line:
                    try:
                        parsed = json.loads(last_line)
                        if parsed.get("type") == "episode_summary":
                            summary = parsed
                    except Exception:
                        pass

            task_name = meta.get("task_name") or fpath.parent.name
            is_sim = "sim" in fpath.parts or meta.get("is_sim", False)
            duration_s = summary.get("duration_s") or meta.get("duration_s", 0.0)
            total_frames = summary.get("total_frames") or meta.get("total_frames", frame_cnt)

            results.append({
                "file_path": str(fpath),
                "file_name": fpath.name,
                "task_name": task_name,
                "is_sim": is_sim,
                "is_cleaned": "_cleaned" in fpath.stem,
                "file_size_kb": size_kb,
                "total_frames": total_frames,
                "duration_s": duration_s,
                "modified_time": mtime,
                "created_at": meta.get("created_at", ""),
            })
        except Exception:
            continue

    results.sort(key=lambda x: x["modified_time"], reverse=True)
    return results

