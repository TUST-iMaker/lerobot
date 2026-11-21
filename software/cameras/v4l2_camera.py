"""Linux 原生 Video4Linux2 (V4L2) USB 摄像头驱动适配器.

核心特性:
1. 零额外第三方依赖: 基于标准库 fcntl / mmap 与 Linux 内核驱动高效交互;
2. 硬件 MMAP 零拷贝缓冲区: 避免在内核空间与用户空间之间进行冗余内存拷贝;
3. 优选硬件 MJPEG 解码: 优先使用 UVC 摄像机自带的 JPEG 压缩流 (低 USB 带宽、高帧率), 并支持回退到 YUYV 等原始像素格式;
4. 鲁棒异常保护: 具备设备断开保护、非阻塞超时轮询与完整的 MMAP 内存生命周期管理.
"""

from __future__ import annotations

try:
    import fcntl
except ImportError:  # Windows fallback
    fcntl = None  # type: ignore[assignment]
import logging
import mmap
import os
import select
import struct
import time
from typing import Any

from cameras.base import BaseCamera, CameraFrame
from cameras.config import CameraConfig

logger = logging.getLogger(__name__)

# V4L2 ioctl 命令常量
VIDIOC_QUERYCAP = 0x80685600
VIDIOC_G_FMT = 0xc0d05604
VIDIOC_S_FMT = 0xc0d05605
VIDIOC_REQBUFS = 0xc0145608
VIDIOC_QUERYBUF = 0xc0585609
VIDIOC_QBUF = 0xc058560f
VIDIOC_DQBUF = 0xc0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_ANY = 0
V4L2_FIELD_NONE = 1


class V4L2Camera(BaseCamera):
    """基于 Linux V4L2 内核接口的标准 USB 摄像头实现."""

    def __init__(self, config: CameraConfig) -> None:
        super().__init__(config)
        self._fd: int | None = None
        self._buffers: list[tuple[mmap.mmap, int]] = []
        self._actual_width: int = config.width
        self._actual_height: int = config.height
        self._pixel_format: str = "MJPG"
        self._device_path: str = ""

    def _resolve_device_path(self) -> str:
        """根据配置解析真实 /dev/videoX 设备路径."""
        dev = self.config.device
        if isinstance(dev, int):
            return f"/dev/video{dev}"
        dev_str = str(dev).strip()
        if dev_str.isdigit():
            return f"/dev/video{dev_str}"
        return dev_str

    def _open_device(self) -> bool:
        """打开 USB 摄像头并初始化 MMAP 视频流."""
        self._device_path = self._resolve_device_path()
        if not os.path.exists(self._device_path):
            raise FileNotFoundError(f"[{self.camera_id}] 相机设备节点不存在: {self._device_path}")

        try:
            self._fd = os.open(self._device_path, os.O_RDWR | os.O_NONBLOCK)
        except Exception as e:
            raise PermissionError(f"[{self.camera_id}] 打开相机节点 {self._device_path} 失败: {e}") from e

        try:
            # 1. 验证设备捕获能力
            cap_buf = bytearray(104)
            fcntl.ioctl(self._fd, VIDIOC_QUERYCAP, cap_buf)
            caps, dev_caps = struct.unpack_from("<II", cap_buf, 84)
            eff_caps = dev_caps if dev_caps else caps
            if not (eff_caps & 0x00000001):  # V4L2_CAP_VIDEO_CAPTURE
                raise RuntimeError(f"[{self.camera_id}] 设备 {self._device_path} 不支持视频捕获")

            # 2. 配置格式与分辨率 (优先 MJPG, 备选 YUYV)
            # struct v4l2_format (type=1, width, height, pixelformat, field=1)
            fmt_buf = bytearray(208)
            fourcc_code = struct.unpack("<I", b"MJPG")[0]
            struct.pack_into("<IIIII", fmt_buf, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE, self.config.width, self.config.height, fourcc_code, V4L2_FIELD_NONE)
            
            try:
                fcntl.ioctl(self._fd, VIDIOC_S_FMT, fmt_buf)
            except OSError:
                # 尝试 YUYV 回退
                fourcc_code = struct.unpack("<I", b"YUYV")[0]
                struct.pack_into("<IIIII", fmt_buf, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE, self.config.width, self.config.height, fourcc_code, V4L2_FIELD_NONE)
                fcntl.ioctl(self._fd, VIDIOC_S_FMT, fmt_buf)

            # 读取实际协商后的分辨率与格式
            act_w, act_h, act_fmt = struct.unpack_from("<III", fmt_buf, 4)
            self._actual_width = act_w
            self._actual_height = act_h
            try:
                self._pixel_format = struct.pack("<I", act_fmt).decode("ascii", errors="ignore").strip()
            except Exception:
                self._pixel_format = "UNKNOWN"

            logger.info(
                "[%s] 成功协商分辨率: %dx%d, 像素编码: %s",
                self.camera_id,
                self._actual_width,
                self._actual_height,
                self._pixel_format,
            )

            # 3. 申请内核 MMAP 缓冲区
            req = bytearray(20)
            buf_count = max(2, self.config.buffer_size)
            struct.pack_into("<IIII", req, 0, buf_count, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, 0)
            fcntl.ioctl(self._fd, VIDIOC_REQBUFS, req)
            actual_count = struct.unpack_from("<I", req, 0)[0]

            self._buffers = []
            for i in range(actual_count):
                qbuf = bytearray(88)
                struct.pack_into("<II", qbuf, 0, i, V4L2_BUF_TYPE_VIDEO_CAPTURE)
                fcntl.ioctl(self._fd, VIDIOC_QUERYBUF, qbuf)
                offset, length = struct.unpack_from("<II", qbuf, 68)
                mm = mmap.mmap(self._fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
                self._buffers.append((mm, length))
                # 将缓冲区入队就绪
                fcntl.ioctl(self._fd, VIDIOC_QBUF, qbuf)

            # 4. 开启硬件视频流 (STREAMON)
            stream_type = struct.pack("<I", V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self._fd, VIDIOC_STREAMON, stream_type)
            logger.info("[%s] V4L2 硬件视频流已开启 (挂载 %d 个 MMAP 缓冲区)", self.camera_id, len(self._buffers))
            return True

        except Exception as e:
            self._close_device()
            raise RuntimeError(f"[{self.camera_id}] 初始化 V4L2 硬件失败: {e}") from e

    def _close_device(self) -> None:
        """关闭流并释放 MMAP 与文件描述符."""
        if self._fd is not None:
            try:
                stream_type = struct.pack("<I", V4L2_BUF_TYPE_VIDEO_CAPTURE)
                fcntl.ioctl(self._fd, VIDIOC_STREAMOFF, stream_type)
            except Exception:
                pass

            for mm, _ in self._buffers:
                try:
                    mm.close()
                except Exception:
                    pass
            self._buffers.clear()

            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = None
            logger.info("[%s] V4L2 设备已释放", self.camera_id)

    def _capture_frame_raw(self) -> CameraFrame:
        """从 MMAP 缓冲区出队抓取一帧并立即将缓冲重入队."""
        if self._fd is None or not self._buffers:
            raise RuntimeError(f"[{self.camera_id}] 相机未就绪")

        dqbuf = bytearray(88)
        struct.pack_into("<II", dqbuf, 0, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)

        # 使用 select 监听文件描述符可读事件 (低 CPU 消耗，响应精确)
        r, _, _ = select.select([self._fd], [], [], 2.0)
        if not r:
            raise TimeoutError(f"[{self.camera_id}] 等待 V4L2 视频帧超时 (2.0s)")

        try:
            fcntl.ioctl(self._fd, VIDIOC_DQBUF, dqbuf)
        except BlockingIOError as e:
            raise TimeoutError(f"[{self.camera_id}] 读取 V4L2 视频帧阻塞: {e}") from e

        idx, _, bytesused = struct.unpack_from("<III", dqbuf, 0)
        raw_payload = self._buffers[idx][0][:bytesused]

        # 立即重新入队 (以便硬件接收下一帧)
        fcntl.ioctl(self._fd, VIDIOC_QBUF, dqbuf)

        self._frame_counter += 1
        now = time.monotonic()

        # 判定图像格式
        fmt_label = "JPEG" if self._pixel_format in ("MJPG", "JPEG") or raw_payload[:2] == b"\xff\xd8" else self._pixel_format

        return CameraFrame(
            camera_id=self.camera_id,
            frame_id=self._frame_counter,
            timestamp=now,
            width=self._actual_width,
            height=self._actual_height,
            color_format=fmt_label,
            raw_bytes=bytes(raw_payload),
        )
