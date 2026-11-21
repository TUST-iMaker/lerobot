"""SO-ARM101 USB 相机自动发现与探测工具.

特性:
1. 扫描 Linux /dev/video* 节点及 /dev/v4l/by-id 稳定符号链接;
2. 通过 V4L2 VIDIOC_QUERYCAP 自动过滤无效的元数据流节点 (Metadata Nodes);
3. 枚举物理摄像头硬件名称、驱动类型、总线拓扑以及支持的像素编码格式 (MJPEG, YUYV 等).
"""

from __future__ import annotations

try:
    import fcntl
except ImportError:  # Windows does not provide fcntl / V4L2
    fcntl = None  # type: ignore[assignment]
import glob
import logging
import os
import platform
import struct
from pathlib import Path

from cameras.config import CameraDeviceInfo

logger = logging.getLogger(__name__)

# Linux V4L2 ioctl 常量
VIDIOC_QUERYCAP = 0x80685600
VIDIOC_ENUM_FMT = 0xc0405602

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_META_CAPTURE = 0x00800000
V4L2_CAP_DEVICE_CAPS = 0x80000000
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1


def discover_cameras() -> list[CameraDeviceInfo]:
    """探测系统上所有物理 USB 相机设备节点 (Linux V4L2 / Windows DirectShow).
    
    Returns:
        包含硬件名称、驱动属性、支持格式列表的 CameraDeviceInfo 列表 (按 index 排序).
    """
    discovered: list[CameraDeviceInfo] = []

    if fcntl is None or platform.system() == "Windows":
        try:
            import cv2
        except ImportError:
            return discovered
        for idx in range(4):
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            try:
                if cap.isOpened():
                    discovered.append(
                        CameraDeviceInfo(
                            index=idx,
                            device_path=str(idx),
                            card_name=f"Camera {idx} (DirectShow)",
                            driver_name="dshow",
                            bus_info=f"dshow:{idx}",
                            is_capture=True,
                            is_metadata=False,
                            supported_formats=["MJPG"],
                        )
                    )
            finally:
                cap.release()
        return discovered

    # 1. 建立 /dev/v4l/by-id 的真实物理路径映射 (如果存在)
    by_id_map: dict[str, str] = {}
    by_id_dir = Path("/dev/v4l/by-id")
    if by_id_dir.exists():
        for symlink in by_id_dir.iterdir():
            try:
                resolved = str(symlink.resolve())
                by_id_map[resolved] = symlink.name
            except Exception:
                pass

    # 2. 收集 /dev/video* 节点
    video_nodes = sorted(glob.glob("/dev/video*"), key=lambda p: int(p.replace("/dev/video", "")) if p.replace("/dev/video", "").isdigit() else 999)

    for dev_path in video_nodes:
        idx_str = dev_path.replace("/dev/video", "")
        if not idx_str.isdigit():
            continue
        idx = int(idx_str)

        try:
            fd = os.open(dev_path, os.O_RDWR | os.O_NONBLOCK)
        except (PermissionError, FileNotFoundError, OSError):
            continue

        try:
            # 查询设备能力
            buf = bytearray(104)
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
            driver = buf[0:16].decode("utf-8", errors="ignore").strip("\x00")
            card = buf[16:48].decode("utf-8", errors="ignore").strip("\x00")
            bus = buf[48:80].decode("utf-8", errors="ignore").strip("\x00")
            caps, dev_caps = struct.unpack_from("<II", buf, 84)

            effective_caps = dev_caps if (caps & V4L2_CAP_DEVICE_CAPS) and dev_caps else caps
            is_capture = bool(effective_caps & V4L2_CAP_VIDEO_CAPTURE)
            is_meta = bool(effective_caps & V4L2_CAP_META_CAPTURE)

            # 枚举该节点支持的编码格式
            supported_formats: list[str] = []
            fmt_index = 0
            while True:
                fmt_buf = bytearray(64)
                struct.pack_into("<II", fmt_buf, 0, fmt_index, V4L2_BUF_TYPE_VIDEO_CAPTURE)
                try:
                    fcntl.ioctl(fd, VIDIOC_ENUM_FMT, fmt_buf)
                    _, _, _, pixelformat = struct.unpack_from("<IIII", fmt_buf, 0)
                    fourcc = struct.pack("<I", pixelformat).decode("ascii", errors="ignore")
                    if fourcc and fourcc not in supported_formats:
                        supported_formats.append(fourcc)
                    fmt_index += 1
                except OSError:
                    break

            # 增强设备名称 (附带 by-id 别名)
            card_display = card
            if dev_path in by_id_map:
                card_display = f"{card} ({by_id_map[dev_path]})"

            info = CameraDeviceInfo(
                index=idx,
                device_path=dev_path,
                card_name=card_display,
                driver_name=driver,
                bus_info=bus,
                is_capture=is_capture,
                is_metadata=is_meta,
                supported_formats=supported_formats,
            )
            discovered.append(info)

        except Exception as e:
            logger.debug("探测设备节点 %s 异常: %s", dev_path, e)
        finally:
            os.close(fd)

    return discovered


def list_capture_cameras() -> list[CameraDeviceInfo]:
    """仅列出具备真实图像捕获能力的有效相机设备 (过滤纯元数据节点)."""
    return [cam for cam in discover_cameras() if cam.is_capture and not cam.is_metadata]

