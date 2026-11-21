"""SO-ARM101 离线轨迹回放控制器 (EpisodeReplayer).

核心职责:
1. 载入示教录制生成的 EpisodeData / JSONL 轨迹文件;
2. 提供毫秒级帧定位与姿态跳转 (Seek / Step), 支持鼠标时间轴无缝拖拽预览;
3. 具备高精度定时回放循环 (Play / Pause / Resume / Stop), 支持 0.25x ~ 3.0x 动态倍速与循环回放 (Loop);
4. 严格遵循控制权状态机: 以 ControlSource.REPLAY 来源提交 RobotAction，回放结束或暂停时安全释放控制权.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from core.action_api import ActionAPI
from core.data_types import (
    ActionType,
    ControlSource,
    JointUnit,
    RobotAction,
    SystemState,
)
from core.kinematics import SO101Kinematics
from recording.cleaner import EpisodeData

logger = logging.getLogger(__name__)


class EpisodeReplayer:
    """机械臂离线示教轨迹回放引擎."""

    def __init__(
        self,
        action_api: ActionAPI,
        kinematics: SO101Kinematics | None = None,
    ) -> None:
        self.api = action_api
        self.kin = kinematics or SO101Kinematics()

        self._lock = threading.Lock()
        self._episode: EpisodeData | None = None
        self._current_frame_idx: int = 0
        self._is_playing: bool = False
        self._is_paused: bool = False
        self._playback_speed: float = 1.0
        self._loop: bool = False

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()  # set 表示非暂停状态

        # 事件回调 (可在每帧回放或回放完成时触发前端通知)
        self.on_frame_callback: Callable[[int, dict[str, Any]], None] | None = None
        self.on_finished_callback: Callable[[], None] | None = None

    @property
    def is_loaded(self) -> bool:
        return self._episode is not None and self._episode.frame_count > 0

    @property
    def is_playing(self) -> bool:
        return self._is_playing and not self._is_paused

    @property
    def is_paused(self) -> bool:
        return self._is_playing and self._is_paused

    @property
    def current_frame(self) -> int:
        return self._current_frame_idx

    @property
    def total_frames(self) -> int:
        return self._episode.frame_count if self._episode else 0

    @property
    def episode(self) -> EpisodeData | None:
        return self._episode

    def load_episode(self, filepath_or_data: Path | str | EpisodeData) -> dict[str, Any]:
        """加载一个 Episode 轨迹以备回放."""
        self.stop()
        with self._lock:
            if isinstance(filepath_or_data, EpisodeData):
                self._episode = filepath_or_data
            else:
                self._episode = EpisodeData.from_file(filepath_or_data)

            self._current_frame_idx = 0
            stats = self._episode.analyze()

        logger.info(
            "[*] 已载入示教轨迹: %s (%d 帧, %.2fs, %.1f FPS)",
            self._episode.task_name,
            self._episode.frame_count,
            self._episode.duration_s,
            self._episode.average_fps,
        )

        return {
            "success": True,
            "task_name": self._episode.task_name,
            "episode_id": self._episode.episode_id,
            "env_type": self._episode.env_type,
            "is_sim": self._episode.is_sim,
            "file_path": str(self._episode.filepath) if self._episode.filepath else None,
            "file_name": self._episode.filepath.name if self._episode.filepath else "",
            "total_frames": self._episode.frame_count,
            "duration_s": round(self._episode.duration_s, 2),
            "average_fps": round(self._episode.average_fps, 1),
            "stats": stats.to_dict(),
        }

    def _ensure_replay_control(self) -> bool:
        """确保当前持有 REPLAY 控制权."""
        curr = self.api.state_machine.active_source
        if curr == ControlSource.REPLAY:
            return True

        if self.api.state_machine.current_state in (SystemState.ESTOP_LATCHED, SystemState.FAULT):
            logger.warning("系统处于急停或故障锁定，拒绝启动回放")
            return False

        if curr is not None:
            self.api.release_control(curr)

        ok = self.api.claim_control(ControlSource.REPLAY)
        if not ok:
            # 尝试由 IDLE 申请进入
            ok = self.api.claim_control(ControlSource.REPLAY)
        return ok

    def seek(self, frame_idx: int) -> dict[str, Any]:
        """将机械臂跳转至指定帧序号的姿态."""
        if not self.is_loaded or self._episode is None:
            return {"success": False, "message": "尚未加载任何轨迹"}

        n = self._episode.frame_count
        frame_idx = max(0, min(n - 1, int(frame_idx)))

        with self._lock:
            self._current_frame_idx = frame_idx
            q = self._episode.get_joint_positions(frame_idx)
            tcp = self._episode.get_tcp_position(frame_idx)

        # 下发目标关节角
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.REPLAY,
            timestamp=time.monotonic(),
            target_joint_positions=q,
            unit=JointUnit.RADIAN,
        )
        self._ensure_replay_control()
        ok, msg = self.api.submit_action(action)

        frame_data = self._episode.frames[frame_idx]
        return {
            "success": ok,
            "frame_index": frame_idx,
            "total_frames": n,
            "relative_time_s": frame_data.get("relative_time_s", 0.0),
            "joints_rad": q,
            "tcp_m": tcp,
            "message": msg,
        }

    def step(self, direction: int = 1) -> dict[str, Any]:
        """单步前进或后退 1 帧."""
        target_idx = self._current_frame_idx + direction
        return self.seek(target_idx)

    def play(
        self,
        speed: float = 1.0,
        loop: bool = False,
        start_frame: int | None = None,
    ) -> bool:
        """启动后台高精度轨迹时序回放线程."""
        if not self.is_loaded or self._episode is None:
            logger.warning("未载入轨迹，无法开始回放")
            return False

        if self._is_playing:
            if self._is_paused:
                self.resume()
                return True
            return True

        if not self._ensure_replay_control():
            logger.warning("无法获取 REPLAY 控制权")
            return False

        self._playback_speed = max(0.1, min(5.0, float(speed)))
        self._loop = bool(loop)
        if start_frame is not None:
            self._current_frame_idx = max(0, min(self.total_frames - 1, start_frame))

        self._stop_event.clear()
        self._pause_event.set()
        self._is_playing = True
        self._is_paused = False

        self._thread = threading.Thread(target=self._playback_loop, daemon=True)
        self._thread.start()
        logger.info("[*] 示教轨迹回放已启动 (倍速: %.2fx, 循环: %s)", self._playback_speed, self._loop)
        return True

    def pause(self) -> None:
        """暂停当前回放."""
        if self._is_playing and not self._is_paused:
            self._is_paused = True
            self._pause_event.clear()
            logger.info("[*] 示教轨迹回放已暂停 (停在第 %d 帧)", self._current_frame_idx)

    def resume(self) -> None:
        """从当前帧恢复回放."""
        if self._is_playing and self._is_paused:
            self._ensure_replay_control()
            self._is_paused = False
            self._pause_event.set()
            logger.info("[*] 示教轨迹回放已继续")

    def stop(self) -> None:
        """停止回放并退出线程."""
        self._stop_event.set()
        self._pause_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self._is_playing = False
        self._is_paused = False

        if self.api.state_machine.active_source == ControlSource.REPLAY:
            self.api.release_control(ControlSource.REPLAY)
        logger.info("[*] 示教轨迹回放已停止")

    def set_speed(self, speed: float) -> None:
        """动态调节回放倍速 (0.1x ~ 5.0x)."""
        self._playback_speed = max(0.1, min(5.0, float(speed)))

    def set_loop(self, loop: bool) -> None:
        """动态设置是否循环回放."""
        self._loop = bool(loop)

    def _playback_loop(self) -> None:
        """高精度时序回放主循环."""
        if not self._episode:
            return

        frames = self._episode.frames
        n = len(frames)
        if n == 0:
            return

        while not self._stop_event.is_set():
            if self._current_frame_idx >= n:
                if self._loop and not self._stop_event.is_set():
                    self._current_frame_idx = 0
                else:
                    break

            # 处理暂停状态挂起
            self._pause_event.wait()
            if self._stop_event.is_set():
                break

            idx = self._current_frame_idx
            frame = frames[idx]

            # 1. 计算下一帧的时间差
            dt = 0.033
            if idx < n - 1:
                t0 = frame.get("relative_time_s", idx * 0.033)
                t1 = frames[idx + 1].get("relative_time_s", (idx + 1) * 0.033)
                dt = max(0.005, (t1 - t0) / self._playback_speed)

            # 2. 提取当前帧目标并下发
            q = self._episode.get_joint_positions(idx)
            action = RobotAction(
                action_type=ActionType.JOINT_POSITION,
                source=ControlSource.REPLAY,
                timestamp=time.monotonic(),
                target_joint_positions=q,
                unit=JointUnit.RADIAN,
            )
            self.api.submit_action(action)

            # 3. 触发监听回调
            if self.on_frame_callback:
                try:
                    self.on_frame_callback(idx, frame)
                except Exception as e:
                    logger.debug("回放回调异常: %s", e)

            # 4. 精确休眠
            time.sleep(dt)
            self._current_frame_idx += 1

        # 播放结束收尾
        self._is_playing = False
        self._is_paused = False
        if self.api.state_machine.active_source == ControlSource.REPLAY:
            self.api.release_control(ControlSource.REPLAY)

        if self.on_finished_callback and not self._stop_event.is_set():
            try:
                self.on_finished_callback()
            except Exception:
                pass
        logger.info("[*] 示教轨迹回放流程完毕")

    def get_status(self) -> dict[str, Any]:
        """获取当前回放引擎的遥测状态字典."""
        total = self.total_frames
        curr = self._current_frame_idx
        pct = round((curr / total) * 100.0, 1) if total > 0 else 0.0

        filename = self._episode.filepath.name if self._episode and self._episode.filepath else ""
        rel_time = 0.0
        if self._episode and curr < total:
            rel_time = self._episode.frames[curr].get("relative_time_s", 0.0)

        return {
            "is_loaded": self.is_loaded,
            "task_name": self._episode.task_name if self._episode else "",
            "file_name": filename,
            "current_frame": curr,
            "total_frames": total,
            "current_time_s": round(rel_time, 2),
            "duration_s": round(self._episode.duration_s, 2) if self._episode else 0.0,
            "progress_percent": pct,
            "is_playing": self.is_playing,
            "is_paused": self.is_paused,
            "speed": self._playback_speed,
            "loop": self._loop,
        }

