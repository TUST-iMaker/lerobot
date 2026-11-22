"""SO-ARM101 相机驱动与多视角视觉管理模块 (cameras).

提供标准 USB 摄像头 (UVC / Video4Linux2 / OpenCV) 与仿真合成相机的统一管理:
1. CameraConfig, CameraDeviceInfo, CameraBackendType: 强类型配置与设备元数据;
2. BaseCamera, CameraFrame: 统一相机抽象与高保真图像帧实体;
3. V4L2Camera: Linux 原生极低延迟 MMAP 零拷贝驱动;
4. OpenCVCamera: 基于 cv2 的通用摄像头驱动;
5. MockCamera: 仿真合成相机 (用于测试、CI与模拟环境);
6. CameraManager: 多相机管理中枢、图像持久化、RobotObservation 组装与 Web 实时推流;
7. discover_cameras, list_capture_cameras: USB 硬件设备自动探测.
"""

from __future__ import annotations

from cameras.base import BaseCamera, CameraFrame
from cameras.config import (
    CameraBackendType,
    CameraConfig,
    CameraDeviceInfo,
)
from cameras.discovery import discover_cameras, list_capture_cameras
from cameras.manager import CameraManager, create_camera
from cameras.mock_camera import MockCamera
from cameras.opencv_camera import OpenCVCamera
from cameras.v4l2_camera import V4L2Camera

__all__ = [
    "BaseCamera",
    "CameraFrame",
    "CameraConfig",
    "CameraDeviceInfo",
    "CameraBackendType",
    "CameraManager",
    "MockCamera",
    "OpenCVCamera",
    "V4L2Camera",
    "create_camera",
    "discover_cameras",
    "list_capture_cameras",
]

