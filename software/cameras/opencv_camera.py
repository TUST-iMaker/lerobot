"""基于 OpenCV (cv2.VideoCapture) 的标准相机驱动适配器.

用于跨平台摄像头交互 (当环境中已安装 opencv-python 时使用).
"""

from __future__ import annotations

import io
import logging
import time
from typing import Any

from cameras.base import BaseCamera, CameraFrame
from cameras.config import CameraConfig

logger = logging.getLogger(__name__)


class OpenCVCamera(BaseCamera):
    """基于 OpenCV cv2.VideoCapture 的通用摄像头实现."""

    def __init__(self, config: CameraConfig) -> None:
        super().__init__(config)
        self._cap: Any = None
        self._cv2: Any = None

    def _open_device(self) -> bool:
        try:
            import cv2
            self._cv2 = cv2
        except ImportError as e:
            raise ImportError(f"[{self.camera_id}] 当前 Python 环境未安装 opencv-python (cv2): {e}") from e

        dev = self.config.device
        if isinstance(dev, str) and dev.isdigit():
            dev = int(dev)

        self._cap = self._cv2.VideoCapture(dev)
        if not self._cap.isOpened():
            self._cap.release()
            self._cap = None
            raise RuntimeError(f"[{self.camera_id}] 无法通过 OpenCV 打开设备: {dev}")

        # 尝试配置分辨率与帧率
        self._cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self._cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self._cap.set(self._cv2.CAP_PROP_FPS, self.config.fps)

        actual_w = int(self._cap.get(self._cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info("[%s] OpenCV 相机连接成功 (分辨率: %dx%d)", self.camera_id, actual_w, actual_h)
        return True

    def _close_device(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            logger.info("[%s] OpenCV 相机设备已释放", self.camera_id)

    def _capture_frame_raw(self) -> CameraFrame:
        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(f"[{self.camera_id}] 相机未处于打开状态")

        ret, frame_bgr = self._cap.read()
        if not ret or frame_bgr is None:
            raise RuntimeError(f"[{self.camera_id}] OpenCV 读取视频帧失败")

        h, w = frame_bgr.shape[:2]
        self._frame_counter += 1
        now = time.monotonic()

        # 编码为高保真 JPEG 字节流
        encode_params = [int(self._cv2.IMWRITE_JPEG_QUALITY), 85]
        success, enc_buf = self._cv2.imencode(".jpg", frame_bgr, encode_params)
        if not success:
            raise RuntimeError(f"[{self.camera_id}] JPEG 压缩失败")

        return CameraFrame(
            camera_id=self.camera_id,
            frame_id=self._frame_counter,
            timestamp=now,
            width=w,
            height=h,
            color_format="JPEG",
            raw_bytes=enc_buf.tobytes(),
        )

