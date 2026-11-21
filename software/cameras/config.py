"""SO-ARM101 相机模块配置与硬件描述定义.

支持标准 USB 摄像头 (UVC / Video4Linux2 / OpenCV) 与仿真 Mock 相机:
1. 统一强类型配置: 明确分辨率 (width, height)、帧率 (fps)、色彩空间与逻辑设备名;
2. 多后端适配: 支持原生 V4L2、OpenCV (cv2) 以及仿真合成 Mock 后端;
3. 自动化硬件探测: 提供 CameraDeviceInfo 结构体用于枚举物理 USB 节点与格式特性.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any


class CameraBackendType(str, Enum):
    """相机底层驱动后端类型."""
    AUTO = "auto"       # 自动探测: Linux 下优先 V4L2/OpenCV，无物理设备时回退至 Mock
    V4L2 = "v4l2"       # Linux 原生 Video4Linux2 (零第三方依赖，极致低延迟与零拷贝)
    OPENCV = "opencv"   # OpenCV VideoCapture (通用跨平台)
    MOCK = "mock"       # 仿真合成相机 (用于自动化测试、CI与无摄像头环境)


@dataclass(frozen=True)
class CameraDeviceInfo:
    """物理检测到的 USB 相机节点元数据."""
    index: int                                  # 逻辑序号 (如 0, 1)
    device_path: str                            # 节点路径 (如 /dev/video0)
    card_name: str                              # 硬件名称 (如 "Integrated Camera")
    driver_name: str                            # 驱动名 (如 "uvcvideo")
    bus_info: str                               # USB 总线标识 (如 "usb-0000:00:14.0-11")
    is_capture: bool                            # 是否为真实视频捕获设备 (过滤元数据节点)
    is_metadata: bool                           # 是否为元数据节点 (如 uvc 伴随节点)
    supported_formats: list[str] = field(default_factory=list)  # 支持的色彩格式 (如 ["MJPG", "YUYV"])

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "device_path": self.device_path,
            "card_name": self.card_name,
            "driver_name": self.driver_name,
            "bus_info": self.bus_info,
            "is_capture": self.is_capture,
            "is_metadata": self.is_metadata,
            "supported_formats": list(self.supported_formats),
        }


@dataclass(frozen=True)
class CameraConfig:
    """单个相机的实例化参数配置.
    
    Attributes:
        camera_id: 逻辑相机名称 (如 'wrist', 'overhead', 'front')
        device: 设备路径或索引 (如 0, '/dev/video0', '/dev/v4l/by-id/...')
        width: 采集宽度 (像素)
        height: 采集高度 (像素)
        fps: 采集帧率 (Hz)
        color_mode: 图像色彩空间 ('RGB', 'BGR', 'GRAY')
        backend: 驱动后端 (AUTO, V4L2, OPENCV, MOCK)
        rotation: 图像旋转角度 (0, 90, 180, 270)
        buffer_size: 环形缓冲队列深度 (默认 2，降低延迟)
        async_mode: 是否启用后台独立抓帧线程 (推荐 True，保证控制主频不受相机阻塞)
    """
    camera_id: str
    device: int | str = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    color_mode: str = "RGB"
    backend: CameraBackendType = CameraBackendType.AUTO
    rotation: int = 0
    buffer_size: int = 2
    async_mode: bool = True

    @classmethod
    def default_usb(
        cls,
        camera_id: str = "cam_top",
        device: int | str = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        async_mode: bool = True,
    ) -> CameraConfig:
        """创建标准 USB 摄像头配置."""
        return cls(
            camera_id=camera_id,
            device=device,
            width=width,
            height=height,
            fps=fps,
            color_mode="RGB",
            backend=CameraBackendType.AUTO,
            async_mode=async_mode,
        )

    @classmethod
    def mock_camera(
        cls,
        camera_id: str = "cam_mock",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> CameraConfig:
        """创建仿真/测试专用虚拟相机配置."""
        return cls(
            camera_id=camera_id,
            device="mock",
            width=width,
            height=height,
            fps=fps,
            color_mode="RGB",
            backend=CameraBackendType.MOCK,
            async_mode=True,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "device": self.device,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "color_mode": self.color_mode,
            "backend": self.backend.value,
            "rotation": self.rotation,
            "buffer_size": self.buffer_size,
            "async_mode": self.async_mode,
        }

