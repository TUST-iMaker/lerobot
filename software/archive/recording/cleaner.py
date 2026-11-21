"""SO-ARM101 离线轨迹数据解析、统计分析与清洗工具 (EpisodeCleaner).

核心特性:
1. 完整解析标准 JSONL 示教文件 (包含 Header 元数据行、Frame 序列与 Summary 摘要行);
2. 运动学统计与异常诊断:
   - 计算各舵机关节瞬时角速度与角加速度;
   - 自动检测前置/后置静止冗余区间 (Idle Deadzones);
   - 探测突变跳变帧 (Spike / Outlier Detection);
   - 解算末端笛卡尔工作空间极值范围 (Bounding Box);
3. 轨迹精修与清洗操作:
   - 智能裁剪首尾静止死区 (Auto-Trim Idle);
   - 指定区间精确裁剪 (Range Trim);
   - 轨迹多项式/样条滤波平滑 (Trajectory Smoothing);
   - 重整时间戳与自增帧序号，输出符合 LeRobot 标准的高质量清洗数据集.
"""

from __future__ import annotations

import copy
import json
import logging
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from core.data_types import (
    SO101_JOINT_NAMES,
    JointState,
    JointUnit,
    Pose3D,
    RobotAction,
    rad_to_deg,
)
from core.kinematics import KinematicsConfig, SO101Kinematics

logger = logging.getLogger(__name__)


@dataclass
class TrajectoryStats:
    """单个 Episode 轨迹数据的统计评估指标."""
    total_frames: int
    duration_s: float
    average_fps: float
    idle_start_frames: int
    idle_end_frames: int
    effective_frames: int
    effective_duration_s: float
    max_joint_velocities: dict[str, float]  # rad/s
    tcp_bounds: dict[str, float]            # min/max x, y, z in meters
    anomalies_count: int
    anomalies_detail: list[dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "total_frames": self.total_frames,
            "duration_s": round(self.duration_s, 3),
            "average_fps": round(self.average_fps, 1),
            "idle_start_frames": self.idle_start_frames,
            "idle_end_frames": self.idle_end_frames,
            "effective_frames": self.effective_frames,
            "effective_duration_s": round(self.effective_duration_s, 3),
            "max_joint_velocities": {k: round(v, 3) for k, v in self.max_joint_velocities.items()},
            "tcp_bounds": {k: round(v, 4) for k, v in self.tcp_bounds.items()},
            "anomalies_count": self.anomalies_count,
            "anomalies_detail": self.anomalies_detail,
        }


class EpisodeData:
    """示教 Episode 轨迹内存数据容器."""

    def __init__(
        self,
        metadata: dict[str, Any],
        frames: list[dict[str, Any]],
        summary: dict[str, Any] | None = None,
        filepath: Path | str | None = None,
    ) -> None:
        self.metadata = dict(metadata)
        self.frames = list(frames)
        self.summary = dict(summary) if summary else {}
        self.filepath = Path(filepath).resolve() if filepath else None
        self._kin = SO101Kinematics()

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def task_name(self) -> str:
        return self.metadata.get("task_name", "unknown_task")

    @property
    def episode_id(self) -> int:
        return int(self.metadata.get("episode_id", 0))

    @property
    def is_sim(self) -> bool:
        return bool(self.metadata.get("is_sim", False))

    @property
    def env_type(self) -> str:
        return self.metadata.get("env_type", "sim" if self.is_sim else "real")

    @property
    def duration_s(self) -> float:
        if not self.frames:
            return 0.0
        t0 = self.frames[0].get("relative_time_s", 0.0)
        t1 = self.frames[-1].get("relative_time_s", 0.0)
        return max(0.0, t1 - t0)

    @property
    def average_fps(self) -> float:
        d = self.duration_s
        if d <= 0.01:
            return 0.0
        return self.frame_count / d

    @classmethod
    def from_file(cls, filepath: Path | str) -> EpisodeData:
        """从标准 JSONL 文件中完整反序列化 Episode 数据."""
        fpath = Path(filepath).resolve()
        if not fpath.is_file():
            raise FileNotFoundError(f"轨迹文件不存在: {fpath}")

        meta: dict[str, Any] = {}
        frames: list[dict[str, Any]] = []
        summary: dict[str, Any] = {}

        with open(fpath, "r", encoding="utf-8") as f:
            for line_idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                try:
                    record = json.loads(line)
                except Exception as exc:
                    logger.warning("[%s] 第 %d 行 JSON 解析失败: %s", fpath.name, line_idx + 1, exc)
                    continue

                rtype = record.get("type")
                if rtype == "episode_metadata":
                    meta = record
                elif rtype == "frame":
                    frames.append(record)
                elif rtype == "episode_summary":
                    summary = record

        if not meta:
            meta = {
                "type": "episode_metadata",
                "task_name": fpath.parent.name,
                "episode_id": 0,
                "robot_type": "so_arm101",
                "created_at": datetime.now(timezone.utc).isoformat(),
            }

        return cls(metadata=meta, frames=frames, summary=summary, filepath=fpath)

    def get_joint_positions(self, frame_idx: int) -> dict[str, float]:
        """提取指定帧的各关节弧度字典."""
        if frame_idx < 0 or frame_idx >= len(self.frames):
            raise IndexError(f"帧序号超限: {frame_idx} (总帧数: {len(self.frames)})")
        frame = self.frames[frame_idx]
        obs = frame.get("observation", {})
        js = obs.get("joint_state", {})
        pos = js.get("positions", {})
        return {k: float(v) for k, v in pos.items()}

    def get_tcp_position(self, frame_idx: int) -> tuple[float, float, float]:
        """提取或正运动学解算指定帧的 TCP 空间坐标 (x, y, z 米)."""
        frame = self.frames[frame_idx]
        obs = frame.get("observation", {})
        tcp = obs.get("tcp_state", {})
        if tcp and "position_m" in tcp and len(tcp["position_m"]) == 3:
            p = tcp["position_m"]
            return (float(p[0]), float(p[1]), float(p[2]))

        # 若未记录则自动正运动学求解
        q = self.get_joint_positions(frame_idx)
        (x, y, z), _, _ = self._kin.forward_kinematics(q)
        return (round(x, 4), round(y, 4), round(z, 4))

    def get_trajectory_points(self) -> list[tuple[float, float, float]]:
        """获取整个示教过程的所有末端 TCP 空间点序列."""
        return [self.get_tcp_position(i) for i in range(len(self.frames))]

    def analyze(
        self,
        idle_velocity_threshold: float = 0.025,
        anomaly_jump_threshold: float = 0.35,
    ) -> TrajectoryStats:
        """执行全方位轨迹数据健康诊断与统计计算."""
        n = len(self.frames)
        if n == 0:
            return TrajectoryStats(
                total_frames=0,
                duration_s=0.0,
                average_fps=0.0,
                idle_start_frames=0,
                idle_end_frames=0,
                effective_frames=0,
                effective_duration_s=0.0,
                max_joint_velocities={j: 0.0 for j in SO101_JOINT_NAMES},
                tcp_bounds={"min_x": 0.0, "max_x": 0.0, "min_y": 0.0, "max_y": 0.0, "min_z": 0.0, "max_z": 0.0},
                anomalies_count=0,
            )

        # 1. 计算时序差分速度与异常突变
        max_vels: dict[str, float] = {j: 0.0 for j in SO101_JOINT_NAMES}
        frame_speeds: list[float] = [0.0] * n
        anomalies: list[dict[str, Any]] = []

        tcp_pts = self.get_trajectory_points()
        min_x = min(p[0] for p in tcp_pts)
        max_x = max(p[0] for p in tcp_pts)
        min_y = min(p[1] for p in tcp_pts)
        max_y = max(p[1] for p in tcp_pts)
        min_z = min(p[2] for p in tcp_pts)
        max_z = max(p[2] for p in tcp_pts)

        for i in range(n - 1):
            q_curr = self.get_joint_positions(i)
            q_next = self.get_joint_positions(i + 1)
            t_curr = self.frames[i].get("relative_time_s", i * 0.033)
            t_next = self.frames[i + 1].get("relative_time_s", (i + 1) * 0.033)
            dt = max(0.001, t_next - t_curr)

            max_speed_this_frame = 0.0
            for j in SO101_JOINT_NAMES:
                val0 = q_curr.get(j, 0.0)
                val1 = q_next.get(j, 0.0)
                diff = abs(val1 - val0)
                v = diff / dt

                if v > max_vels[j]:
                    max_vels[j] = v
                if v > max_speed_this_frame:
                    max_speed_this_frame = v

                # 突变检测 (单步跳跃过大)
                if diff > anomaly_jump_threshold:
                    anomalies.append({
                        "frame": i + 1,
                        "joint": j,
                        "jump_rad": round(diff, 4),
                        "jump_deg": round(rad_to_deg(diff), 2),
                        "speed_rad_s": round(v, 2),
                    })

            frame_speeds[i] = max_speed_this_frame

        # 最后一帧速度近似等于倒数第二帧
        if n > 1:
            frame_speeds[-1] = frame_speeds[-2]

        # 2. 静止死区检测 (Idle Detection)
        # 前向搜索动作开始帧
        start_idx = 0
        for i in range(n):
            if frame_speeds[i] > idle_velocity_threshold:
                start_idx = max(0, i - 1)  # 保留 1 帧启动余量
                break

        # 后向搜索动作结束帧
        end_idx = n - 1
        for i in range(n - 1, -1, -1):
            if frame_speeds[i] > idle_velocity_threshold:
                end_idx = min(n - 1, i + 1)  # 保留 1 帧收尾余量
                break

        if start_idx > end_idx:
            start_idx, end_idx = 0, n - 1

        idle_start = start_idx
        idle_end = (n - 1) - end_idx
        effective_frames = max(1, end_idx - start_idx + 1)

        t_start = self.frames[start_idx].get("relative_time_s", 0.0)
        t_end = self.frames[end_idx].get("relative_time_s", self.duration_s)
        effective_duration = max(0.0, t_end - t_start)

        return TrajectoryStats(
            total_frames=n,
            duration_s=self.duration_s,
            average_fps=self.average_fps,
            idle_start_frames=idle_start,
            idle_end_frames=idle_end,
            effective_frames=effective_frames,
            effective_duration_s=effective_duration,
            max_joint_velocities=max_vels,
            tcp_bounds={
                "min_x": min_x,
                "max_x": max_x,
                "min_y": min_y,
                "max_y": max_y,
                "min_z": min_z,
                "max_z": max_z,
            },
            anomalies_count=len(anomalies),
            anomalies_detail=anomalies[:20],  # 最多返回前 20 条异常
        )


class EpisodeCleaner:
    """示教轨迹数据精修与清洗器."""

    @staticmethod
    def trim_range(episode: EpisodeData, start_frame: int, end_frame: int) -> EpisodeData:
        """裁剪指定闭区间 [start_frame, end_frame] 的轨迹数据，并重新对齐时间基准."""
        n = len(episode.frames)
        if n == 0:
            return copy.deepcopy(episode)

        start_frame = max(0, min(n - 1, int(start_frame)))
        end_frame = max(start_frame, min(n - 1, int(end_frame)))

        trimmed_frames_raw = episode.frames[start_frame : end_frame + 1]
        if not trimmed_frames_raw:
            return copy.deepcopy(episode)

        # 时间基准归零重整
        base_time = trimmed_frames_raw[0].get("relative_time_s", 0.0)
        base_wall = trimmed_frames_raw[0].get("timestamp", 0.0)

        cleaned_frames: list[dict[str, Any]] = []
        for new_idx, f in enumerate(trimmed_frames_raw):
            f_copy = copy.deepcopy(f)
            orig_rel_t = f_copy.get("relative_time_s", 0.0)
            f_copy["frame_index"] = new_idx
            f_copy["relative_time_s"] = round(max(0.0, orig_rel_t - base_time), 4)
            cleaned_frames.append(f_copy)

        # 组装元数据与更新摘要
        new_meta = copy.deepcopy(episode.metadata)
        new_meta["cleaned_at"] = datetime.now(timezone.utc).isoformat()
        new_meta["trim_source"] = {
            "original_file": str(episode.filepath) if episode.filepath else None,
            "original_total_frames": n,
            "cut_start_frame": start_frame,
            "cut_end_frame": end_frame,
        }

        dur_s = cleaned_frames[-1]["relative_time_s"]
        fps = round(len(cleaned_frames) / dur_s, 1) if dur_s > 0.01 else episode.average_fps

        new_summary = {
            "type": "episode_summary",
            "task_name": episode.task_name,
            "episode_id": episode.episode_id,
            "env_type": episode.env_type,
            "is_sim": episode.is_sim,
            "total_frames": len(cleaned_frames),
            "duration_seconds": dur_s,
            "average_fps": fps,
            "cleaned": True,
            "ended_at": datetime.now(timezone.utc).isoformat(),
        }

        return EpisodeData(
            metadata=new_meta,
            frames=cleaned_frames,
            summary=new_summary,
            filepath=None,
        )

    @classmethod
    def auto_trim_idle(
        cls,
        episode: EpisodeData,
        idle_velocity_threshold: float = 0.025,
    ) -> tuple[EpisodeData, int, int]:
        """智能自动检测并去除机械臂动作开始前与完成后的多余静止死区."""
        stats = episode.analyze(idle_velocity_threshold=idle_velocity_threshold)
        start_idx = stats.idle_start_frames
        end_idx = stats.total_frames - 1 - stats.idle_end_frames
        cleaned = cls.trim_range(episode, start_idx, end_idx)
        return cleaned, start_idx, end_idx

    @staticmethod
    def smooth_trajectory(
        episode: EpisodeData,
        window_size: int = 5,
    ) -> EpisodeData:
        """对关节角度序列进行平滑滤波 (滑动窗口)，消除示教轻微手抖与传感器噪声."""
        if len(episode.frames) <= window_size or window_size <= 1:
            return copy.deepcopy(episode)

        half_w = window_size // 2
        new_episode = copy.deepcopy(episode)

        n = len(new_episode.frames)
        for j in SO101_JOINT_NAMES:
            raw_vals = [episode.get_joint_positions(i).get(j, 0.0) for i in range(n)]
            smoothed_vals: list[float] = []

            for i in range(n):
                low = max(0, i - half_w)
                high = min(n, i + half_w + 1)
                subset = raw_vals[low:high]
                smoothed_vals.append(sum(subset) / len(subset))

            # 回填平滑后的角度至帧观测中
            for i in range(n):
                obs = new_episode.frames[i].setdefault("observation", {})
                js = obs.setdefault("joint_state", {})
                pos = js.setdefault("positions", {})
                pos[j] = round(smoothed_vals[i], 5)

        new_meta = new_episode.metadata
        new_meta["smoothed"] = True
        new_meta["smooth_window"] = window_size
        return new_episode

    @staticmethod
    def save_to_file(
        episode: EpisodeData,
        output_filepath: Path | str,
        overwrite: bool = True,
    ) -> Path:
        """将清洗后的 Episode 数据持久化写入标准 JSONL 规范文件."""
        out_path = Path(output_filepath).resolve()
        if out_path.exists() and not overwrite:
            raise FileExistsError(f"目标文件已存在且未开启覆盖: {out_path}")

        out_path.parent.mkdir(parents=True, exist_ok=True)

        with open(out_path, "w", encoding="utf-8") as f:
            # 1. 写入 Header
            f.write(json.dumps(episode.metadata, ensure_ascii=False) + "\n")
            # 2. 写入 Frames
            for frame in episode.frames:
                f.write(json.dumps(frame, ensure_ascii=False) + "\n")
            # 3. 写入 Summary
            if episode.summary:
                f.write(json.dumps(episode.summary, ensure_ascii=False) + "\n")

        episode.filepath = out_path
        logger.info(
            "[*] 清洗后轨迹保存成功: %d 帧, 时长 %.2fs -> %s",
            len(episode.frames),
            episode.duration_s,
            out_path,
        )
        return out_path

