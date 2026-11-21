"""SO-ARM101 仿真与测试合成相机 (MockCamera).

用于在无物理 USB 摄像头连接、CI/CD 自动化测试或纯仿真环境下提供可重现的合成视觉流:
1. 生成动态色彩网格与时钟跳变目标 (可测试运动检测与丢帧率);
2. 绘制相机名、时间戳、帧序号水印;
3. 产出与真机完全一致的标准 CameraFrame 与 JPEG 压缩字节流.
"""

from __future__ import annotations

import io
import math
import time
from typing import Any

from PIL import Image, ImageDraw

from cameras.base import BaseCamera, CameraFrame
from cameras.config import CameraConfig


class MockCamera(BaseCamera):
    """纯软件合成仿真相机实现."""

    def __init__(self, config: CameraConfig) -> None:
        super().__init__(config)
        self._start_time: float = time.monotonic()

    def _open_device(self) -> bool:
        self._start_time = time.monotonic()
        return True

    def _close_device(self) -> None:
        pass

    def _capture_frame_raw(self) -> CameraFrame:
        now = time.monotonic()
        self._frame_counter += 1
        elapsed = now - self._start_time

        w = self.config.width
        h = self.config.height

        # 1. 构建背景渐变底图 (深灰/蓝暗色调)
        img = Image.new("RGB", (w, h), color=(15, 23, 42))
        draw = ImageDraw.Draw(img)

        # 绘制网格参考线
        grid_step = 40
        for x in range(0, w, grid_step):
            draw.line([(x, 0), (x, h)], fill=(30, 41, 59), width=1)
        for y in range(0, h, grid_step):
            draw.line([(0, y), (w, y)], fill=(30, 41, 59), width=1)

        # 2. 绘制动态动态运动色球 (模拟桌面追踪工件，做李萨如曲线运动)
        cx = int((w / 2.0) + (w / 3.0) * math.cos(elapsed * 2.5))
        cy = int((h / 2.0) + (h / 3.0) * math.sin(elapsed * 3.5))
        radius = 22
        draw.ellipse(
            [cx - radius, cy - radius, cx + radius, cy + radius],
            fill=(245, 158, 11),  # Amber 发光球
            outline=(255, 255, 255),
            width=2,
        )

        # 3. 绘制水印信息文本
        info_lines = [
            f"MOCK CAMERA: {self.camera_id}",
            f"Frame #{self._frame_counter:06d} | {self.config.fps} FPS",
            f"Time: {elapsed:07.3f}s",
            f"Res: {w}x{h}",
        ]
        text_y = 10
        for line in info_lines:
            draw.text((12, text_y), line, fill=(56, 189, 248))
            text_y += 16

        # 4. 压缩为标准 JPEG 字节流
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        raw_bytes = buf.getvalue()

        return CameraFrame(
            camera_id=self.camera_id,
            frame_id=self._frame_counter,
            timestamp=now,
            width=w,
            height=h,
            color_format="JPEG",
            raw_bytes=raw_bytes,
        )

