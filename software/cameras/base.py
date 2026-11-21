"""SO-ARM101 相机抽象基类与标准帧数据结构.

职责:
1. 统一输出标准 CameraFrame 实体，解耦底层硬件驱动协议与上层视觉算法/录制管道;
2. 封装高吞吐、零掉帧的后台异步抓帧线程 (Async Reader Loop)，确保机械臂运动主频绝不被相机 I/O 阻塞;
3. 完全对齐 core.data_types.CameraFrameRef，无缝衔接多模态 RobotObservation.
"""

from __future__ import annotations

import abc
import io
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

from PIL import Image

from core.data_types import CameraFrameRef
from cameras.config import CameraConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class CameraFrame:
    """相机单帧数据实体.
    
    Attributes:
        camera_id: 相机逻辑标识符
        frame_id: 自增帧编号
        timestamp: 单调时钟时间戳 (time.monotonic())
        width: 宽度像素
        height: 高度像素
        color_format: 编码或通道格式 (如 'RGB', 'BGR', 'JPEG')
        raw_bytes: 原始图像字节流 (JPEG 压缩流或未经压缩的原始内存切片)
    """
    camera_id: str
    frame_id: int
    timestamp: float
    width: int
    height: int
    color_format: str
    raw_bytes: bytes

    def to_pil(self) -> Image.Image:
        """解析还原为标准 PIL Image 对象 (RGB 模式)."""
        if self.color_format == "JPEG" or self.raw_bytes[:2] == b"\xff\xd8":
            # 直接从 JPEG 字节流解码
            img = Image.open(io.BytesIO(self.raw_bytes))
            if img.mode != "RGB":
                img = img.convert("RGB")
            return img
        elif self.color_format == "RGB":
            return Image.frombytes("RGB", (self.width, self.height), self.raw_bytes)
        elif self.color_format == "BGR":
            b, g, r = Image.frombytes("RGB", (self.width, self.height), self.raw_bytes).split()
            return Image.merge("RGB", (r, g, b))
        elif self.color_format == "GRAY":
            return Image.frombytes("L", (self.width, self.height), self.raw_bytes).convert("RGB")
        else:
            # 尝试回退通过 PIL 自动侦测
            try:
                return Image.open(io.BytesIO(self.raw_bytes)).convert("RGB")
            except Exception as e:
                raise ValueError(f"无法解析格式为 {self.color_format} 的图像帧: {e}") from e

    def to_jpeg_bytes(self, quality: int = 85) -> bytes:
        """获取或转换为紧凑的 JPEG 压缩字节流 (适用于网络推流或落盘)."""
        if self.color_format == "JPEG" or self.raw_bytes[:2] == b"\xff\xd8":
            return self.raw_bytes
        pil_img = self.to_pil()
        out = io.BytesIO()
        pil_img.save(out, format="JPEG", quality=quality)
        return out.getvalue()

    def to_frame_ref(self, relative_path: str | None = None) -> CameraFrameRef:
        """转换为 core.data_types.CameraFrameRef，用于嵌入 RobotObservation 时序记录."""
        return CameraFrameRef(
            camera_id=self.camera_id,
            frame_id=self.frame_id,
            timestamp=self.timestamp,
            width=self.width,
            height=self.height,
            color_format=self.color_format,
            relative_path=relative_path,
        )


class BaseCamera(abc.ABC):
    """所有具体硬件/仿真相机实现的抽象基类."""

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._connected: bool = False
        self._frame_counter: int = 0
        self._lock = threading.Lock()

        # 异步后台抓帧相关
        self._latest_frame: CameraFrame | None = None
        self._stream_running: bool = False
        self._stream_thread: threading.Thread | None = None
        self._fps_tracker_time: float = time.monotonic()
        self._fps_tracker_count: int = 0
        self._actual_fps: float = 0.0

    @property
    def camera_id(self) -> str:
        return self.config.camera_id

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def actual_fps(self) -> float:
        """统计测得的瞬时抓帧实际 FPS."""
        return self._actual_fps

    def __enter__(self) -> BaseCamera:
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.disconnect()

    def connect(self) -> bool:
        """建立相机物理或逻辑连接."""
        with self._lock:
            if self._connected:
                logger.info("[%s] 相机已连接", self.camera_id)
                return True

            logger.info("[%s] 正在连接相机 (设备: %s, 后端: %s)...", self.camera_id, self.config.device, self.config.backend.value)
            success = self._open_device()
            self._connected = success

            if success and self.config.async_mode:
                self.start_stream()

            return success

    def disconnect(self) -> None:
        """安全释放底层硬件资源与抓帧线程."""
        self.stop_stream()
        with self._lock:
            if not self._connected:
                return
            logger.info("[%s] 正在断开相机连接...", self.camera_id)
            try:
                self._close_device()
            except Exception as e:
                logger.warning("[%s] 断开相机时发生异常: %e", self.camera_id, e)
            finally:
                self._connected = False
                self._latest_frame = None

    def read_frame(self) -> CameraFrame:
        """同步读取一帧 (若异步线程开启，返回最新缓存帧并等待首帧就绪)."""
        if not self._connected:
            raise RuntimeError(f"[{self.camera_id}] 相机未连接，无法读取帧")

        if self.config.async_mode and self._stream_running:
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                frame = self.get_latest_frame()
                if frame is not None:
                    return frame
                time.sleep(0.01)
            frame = self.get_latest_frame()
            if frame is not None:
                return frame
            raise TimeoutError(f"[{self.camera_id}] 等待异步视频首帧超时")

        # 同步非异步模式: 直接读取底层硬件单帧
        with self._lock:
            frame = self._capture_frame_raw()
            self._update_fps_stats()
            self._latest_frame = frame
            return frame

    def get_latest_frame(self) -> CameraFrame | None:
        """获取当前最新捕获的一帧图像 (零等待非阻塞，适合 30Hz 控制循环)."""
        with self._lock:
            return self._latest_frame

    def start_stream(self) -> None:
        """启动后台独立抓帧守护线程."""
        if self._stream_running:
            return
        self._stream_running = True
        self._stream_thread = threading.Thread(
            target=self._async_capture_loop,
            name=f"CameraAsyncReader-{self.camera_id}",
            daemon=True,
        )
        self._stream_thread.start()
        logger.info("[%s] 已启动异步抓帧线程 (目标帧率: %d FPS)", self.camera_id, self.config.fps)

    def stop_stream(self) -> None:
        """停止后台抓帧线程."""
        self._stream_running = False
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=1.5)
            self._stream_thread = None

    def _async_capture_loop(self) -> None:
        """异步抓帧线程主循环."""
        target_interval = 1.0 / max(1, self.config.fps)
        while self._stream_running and self._connected:
            t0 = time.monotonic()
            try:
                with self._lock:
                    if not self._connected:
                        break
                    frame = self._capture_frame_raw()
                    self._latest_frame = frame
                    self._update_fps_stats()
            except Exception as e:
                logger.warning("[%s] 异步抓帧异常: %s", self.camera_id, e)
                time.sleep(0.05)
                continue

            elapsed = time.monotonic() - t0
            sleep_time = max(0.001, target_interval - elapsed)
            time.sleep(sleep_time)

    def _update_fps_stats(self) -> None:
        """统计刷新实时帧率."""
        self._fps_tracker_count += 1
        now = time.monotonic()
        delta = now - self._fps_tracker_time
        if delta >= 1.0:
            self._actual_fps = round(self._fps_tracker_count / delta, 1)
            self._fps_tracker_count = 0
            self._fps_tracker_time = now

    @abc.abstractmethod
    def _open_device(self) -> bool:
        """硬件具体打开流程."""
        raise NotImplementedError

    @abc.abstractmethod
    def _close_device(self) -> None:
        """硬件具体关闭与释放流程."""
        raise NotImplementedError

    @abc.abstractmethod
    def _capture_frame_raw(self) -> CameraFrame:
        """硬件底层抓取单帧图像字节并组装 CameraFrame."""
        raise NotImplementedError
