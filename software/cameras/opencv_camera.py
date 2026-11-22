"""基于 OpenCV (cv2.VideoCapture) 的标准相机驱动适配器 (对齐官方 OpenCVCamera).

特性:
1. Windows 下导入 cv2 前自动设置 OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS="0" 防止打开挂死;
2. Windows 下整数设备优先使用 cv2.CAP_DSHOW, 且严格在 WIDTH/HEIGHT/FPS 之后设置 FOURCC;
3. 强校验读回分辨率与帧率, 不匹配时释放句柄并抛出 RuntimeError;
4. 支持 color_mode (RGB/BGR/GRAY) 与 rotation (0/90/180/270/-90).
"""

from __future__ import annotations

import logging
import math
import os
import platform
import time
from typing import Any

if platform.system() == "Windows" and "OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS" not in os.environ:
    os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

from cameras.base import BaseCamera, CameraFrame
from cameras.config import CameraConfig

logger = logging.getLogger(__name__)


class OpenCVCamera(BaseCamera):
    """基于 OpenCV cv2.VideoCapture 的跨平台摄像头实现 (对齐 lerobot OpenCVCamera)."""

    def __init__(self, config: CameraConfig) -> None:
        super().__init__(config)
        self._cap: Any = None
        self._cv2: Any = None

    def _apply_fourcc(self) -> None:
        if not self.config.fourcc or self._cap is None or self._cv2 is None:
            return
        fourcc_str = self.config.fourcc
        if hasattr(self._cv2, "VideoWriter") and hasattr(self._cv2.VideoWriter, "fourcc"):
            fourcc_code = self._cv2.VideoWriter.fourcc(*fourcc_str)
        else:
            fourcc_code = self._cv2.VideoWriter_fourcc(*fourcc_str)
        ok = self._cap.set(self._cv2.CAP_PROP_FOURCC, fourcc_code)
        actual_raw = int(round(self._cap.get(self._cv2.CAP_PROP_FOURCC)))
        actual_str = "".join(chr((actual_raw >> (8 * i)) & 0xFF) for i in range(4))
        if not ok or actual_str.upper() != fourcc_str.upper():
            logger.warning(
                "[%s] 请求 FOURCC=%s, 实际读回=%r (ok=%s)",
                self.camera_id,
                fourcc_str,
                actual_str,
                ok,
            )

    def _open_device(self) -> bool:
        is_win = platform.system() == "Windows"
        if is_win and "OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS" not in os.environ:
            os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

        try:
            import cv2
            self._cv2 = cv2
        except ImportError as e:
            raise ImportError(f"[{self.camera_id}] 当前 Python 环境未安装 opencv-python (cv2): {e}") from e

        if hasattr(self._cv2, "setNumThreads"):
            self._cv2.setNumThreads(1)

        dev = self.config.device
        if isinstance(dev, str) and dev.isdigit():
            dev = int(dev)

        backend = self.config.cv2_backend
        if backend is None and is_win and isinstance(dev, int) and hasattr(self._cv2, "CAP_DSHOW"):
            backend = self._cv2.CAP_DSHOW

        if backend is not None:
            self._cap = self._cv2.VideoCapture(dev, backend)
        else:
            self._cap = self._cv2.VideoCapture(dev)

        if not self._cap.isOpened():
            self._cap.release()
            self._cap = None
            raise RuntimeError(f"[{self.camera_id}] 无法通过 OpenCV 打开设备: {dev}")

        capture_w, capture_h = self.config.width, self.config.height
        if self.config.rotation in (90, 270, -90):
            capture_w, capture_h = self.config.height, self.config.width

        try:
            if not is_win:
                self._apply_fourcc()

            w_ok = self._cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, capture_w)
            h_ok = self._cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, capture_h)
            actual_w = int(round(self._cap.get(self._cv2.CAP_PROP_FRAME_WIDTH)))
            actual_h = int(round(self._cap.get(self._cv2.CAP_PROP_FRAME_HEIGHT)))
            if not w_ok or actual_w != capture_w or not h_ok or actual_h != capture_h:
                raise RuntimeError(
                    f"[{self.camera_id}] 相机分辨率校验失败: 请求 {capture_w}x{capture_h}, 实得 {actual_w}x{actual_h}"
                )

            fps_ok = self._cap.set(self._cv2.CAP_PROP_FPS, float(self.config.fps))
            actual_fps = float(self._cap.get(self._cv2.CAP_PROP_FPS))
            if not fps_ok or not math.isclose(float(self.config.fps), actual_fps, rel_tol=1e-3):
                raise RuntimeError(
                    f"[{self.camera_id}] 相机帧率校验失败: 请求 {self.config.fps} FPS, 实得 {actual_fps} FPS"
                )

            if is_win:
                self._apply_fourcc()

            if hasattr(self._cv2, "CAP_PROP_BUFFERSIZE"):
                try:
                    self._cap.set(self._cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size)
                except Exception:
                    pass
        except Exception:
            if self._cap is not None:
                self._cap.release()
                self._cap = None
            raise

        logger.info(
            "[%s] OpenCV 相机连接成功 (分辨率: %dx%d @ %.1ffps, mode=%s, rot=%d)",
            self.camera_id,
            actual_w,
            actual_h,
            actual_fps,
            self.config.color_mode,
            self.config.rotation,
        )
        return True

    def _close_device(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            logger.info("[%s] OpenCV 相机设备已释放", self.camera_id)

    def _capture_frame_raw(self) -> CameraFrame:
        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(f"[{self.camera_id}] 相机未处于打开状态")

        ret, frame = self._cap.read()
        if not ret or frame is None:
            raise RuntimeError(f"[{self.camera_id}] OpenCV 读取视频帧失败")

        if getattr(frame, "ndim", 0) != 3 or frame.shape[2] != 3:
            raise RuntimeError(f"[{self.camera_id}] 非法视频帧维度: {getattr(frame, 'shape', None)}")

        rot = self.config.rotation
        if rot == 90:
            frame = self._cv2.rotate(frame, self._cv2.ROTATE_90_CLOCKWISE)
        elif rot == 180:
            frame = self._cv2.rotate(frame, self._cv2.ROTATE_180)
        elif rot in (270, -90):
            frame = self._cv2.rotate(frame, self._cv2.ROTATE_90_COUNTERCLOCKWISE)

        mode = self.config.color_mode.upper()
        if mode == "RGB":
            frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
            fmt = "RGB"
        elif mode == "GRAY":
            frame = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2GRAY)
            fmt = "GRAY"
        elif mode == "BGR":
            fmt = "BGR"
        else:
            raise ValueError(f"[{self.camera_id}] 不支持的色彩模式: {self.config.color_mode}")

        h, w = frame.shape[:2]
        self._frame_counter += 1
        now = time.monotonic()

        return CameraFrame(
            camera_id=self.camera_id,
            frame_id=self._frame_counter,
            timestamp=now,
            width=w,
            height=h,
            color_format=fmt,
            raw_bytes=frame.tobytes(),
        )
