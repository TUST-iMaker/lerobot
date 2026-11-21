"""SO-ARM101 多相机系统集成与综合管理中枢 (CameraManager).

核心职责:
1. 多路相机生命周期统一调度 (添加、连接、断开、健康监测);
2. 驱动后端自适应工厂 (自动适配 V4L2、OpenCV 与 Mock);
3. 观测帧对齐与时序元数据转换: 生成符合 core.data_types.RobotObservation 契约的字典;
4. 示教录制落盘支持: 高效持久化 JPEG 序列至 recording/data/<task>/images/<camera_id>/;
5. Web 实时推流发生器: 支持 HTTP multipart/x-mixed-replace (MJPEG) 供浏览器零插件实时监看.
"""

from __future__ import annotations

import logging
import os
import threading
from pathlib import Path
from typing import Any, Generator

from core.data_types import CameraFrameRef
from cameras.base import BaseCamera, CameraFrame
from cameras.config import CameraBackendType, CameraConfig
from cameras.discovery import list_capture_cameras
from cameras.mock_camera import MockCamera
from cameras.v4l2_camera import V4L2Camera

logger = logging.getLogger(__name__)


def create_camera(config: CameraConfig) -> BaseCamera:
    """相机驱动实例自适应创建工厂."""
    backend = config.backend

    # 1. 显式指定后端
    if backend == CameraBackendType.MOCK or str(config.device).lower() == "mock":
        return MockCamera(config)

    if backend == CameraBackendType.V4L2:
        return V4L2Camera(config)

    if backend == CameraBackendType.OPENCV:
        from cameras.opencv_camera import OpenCVCamera
        return OpenCVCamera(config)

    # 2. AUTO 自动协商模式
    # 如果运行在 Linux 上且设备为物理 video 节点，优先使用高效稳定的原生 V4L2
    if os.name == "posix":
        dev_str = str(config.device)
        if dev_str.startswith("/dev/video") or dev_str.isdigit():
            try:
                return V4L2Camera(config)
            except Exception as e:
                logger.warning("[%s] 初始化 V4L2 失败，尝试回退: %s", config.camera_id, e)

    # 尝试 OpenCV
    try:
        from cameras.opencv_camera import OpenCVCamera
        return OpenCVCamera(config)
    except Exception:
        pass

    # 无物理可用驱动时，回退到仿真 Mock 相机
    logger.warning("[%s] 未能加载真实摄像头驱动，安全回退到 MockCamera", config.camera_id)
    return MockCamera(config)


class CameraManager:
    """机械臂视觉多相机总管理器."""

    def __init__(
        self,
        base_images_dir: Path | str | None = None,
        is_sim: bool = False,
    ) -> None:
        if base_images_dir is None:
            root_dir = (Path(__file__).resolve().parent.parent / "recording" / "data").resolve()
        else:
            root_dir = Path(base_images_dir).resolve()

        self._is_sim = bool(is_sim)
        self.env_type = "sim" if self._is_sim else "real"

        if root_dir.name in ("sim", "real"):
            self.base_images_dir = root_dir
            self._is_sim = (root_dir.name == "sim")
            self.env_type = root_dir.name
        else:
            self.base_images_dir = (root_dir / self.env_type).resolve()

        self._cameras: dict[str, BaseCamera] = {}
        self._lock = threading.Lock()

    def __enter__(self) -> CameraManager:
        self.connect_all()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.disconnect_all()

    @property
    def camera_ids(self) -> list[str]:
        """已注册相机逻辑名称列表."""
        with self._lock:
            return list(self._cameras.keys())

    def get_camera(self, camera_id: str) -> BaseCamera | None:
        """获取指定相机的驱动实例."""
        with self._lock:
            return self._cameras.get(camera_id)

    def add_camera(self, config: CameraConfig) -> BaseCamera:
        """注册并实例化一个新相机."""
        with self._lock:
            if config.camera_id in self._cameras:
                logger.warning("相机 [%s] 已存在，先关闭旧实例", config.camera_id)
                self._cameras[config.camera_id].disconnect()

            cam = create_camera(config)
            self._cameras[config.camera_id] = cam
            logger.info("已注册相机: [%s] (设备: %s, 后端: %s)", config.camera_id, config.device, cam.__class__.__name__)
            return cam

    def auto_discover_and_register(self, max_cameras: int = 2) -> list[str]:
        """自动扫描物理 USB 摄像头并批量注册.
        
        具备智能硬件识别: 优先识别腕部微型相机 (如 icspring / endoscope / wrist)，
        将其映射为 cam_wrist 并排在首位，保障仿真与示教界面默认展示腕部视角.
        """
        devices = list_capture_cameras()
        registered: list[str] = []

        if not devices:
            logger.info("未检测到物理 USB 摄像头，自动添加一个默认仿真 Mock 相机")
            mock_cfg = CameraConfig.mock_camera(camera_id="cam_mock")
            self.add_camera(mock_cfg)
            return ["cam_mock"]

        # 智能匹配逻辑名称: 优先识别腕部相机与顶部/笔记本相机
        assigned: dict[str, CameraDeviceInfo] = {}
        unassigned: list[CameraDeviceInfo] = []

        for dev in devices[:max_cameras]:
            card_lower = dev.card_name.lower()
            if any(k in card_lower for k in ("icspring", "wrist", "arm", "endoscope")):
                assigned["cam_wrist"] = dev
            elif any(k in card_lower for k in ("integrated", "internal", "laptop", "built-in", "builtin")):
                assigned["cam_top"] = dev
            else:
                unassigned.append(dev)

        # 补全未自动命名的相机
        for dev in unassigned:
            if "cam_wrist" not in assigned:
                assigned["cam_wrist"] = dev
            elif "cam_top" not in assigned:
                assigned["cam_top"] = dev
            else:
                assigned[f"cam_{dev.index}"] = dev

        # 注册相机实例 (优先将 cam_wrist 排在首位)
        sort_order = ["cam_wrist", "cam_top", "cam_front", "cam_side"]
        sorted_keys = sorted(
            assigned.keys(),
            key=lambda k: sort_order.index(k) if k in sort_order else 99
        )

        for cid in sorted_keys:
            dev_info = assigned[cid]
            cfg = CameraConfig.default_usb(
                camera_id=cid,
                device=dev_info.device_path,
                width=640,
                height=480,
                fps=30,
            )
            self.add_camera(cfg)
            registered.append(cid)

        return registered

    def connect_all(self) -> dict[str, bool]:
        """连接所有已注册相机."""
        with self._lock:
            results: dict[str, bool] = {}
            for cid, cam in self._cameras.items():
                try:
                    ok = cam.connect()
                    results[cid] = ok
                except Exception as e:
                    logger.error("连接相机 [%s] 失败: %s", cid, e)
                    results[cid] = False
            return results

    def disconnect_all(self) -> None:
        """安全断开所有相机."""
        with self._lock:
            for cid, cam in self._cameras.items():
                try:
                    cam.disconnect()
                except Exception as e:
                    logger.warning("断开相机 [%s] 异常: %s", cid, e)

    def capture_all(self) -> dict[str, CameraFrame]:
        """同步读取所有相机的当前帧."""
        frames: dict[str, CameraFrame] = {}
        with self._lock:
            cams = list(self._cameras.items())

        for cid, cam in cams:
            if cam.is_connected:
                try:
                    frames[cid] = cam.read_frame()
                except Exception as e:
                    logger.warning("从相机 [%s] 读取帧异常: %s", cid, e)
        return frames

    def get_latest_frames(self) -> dict[str, CameraFrame]:
        """非阻塞获取所有相机的最新缓存帧 (无锁低延迟)."""
        frames: dict[str, CameraFrame] = {}
        with self._lock:
            cams = list(self._cameras.items())

        for cid, cam in cams:
            if cam.is_connected:
                f = cam.get_latest_frame()
                if f is not None:
                    frames[cid] = f
        return frames

    def get_observation_frame_refs(self) -> dict[str, CameraFrameRef]:
        """获取当前多相机快照元数据 (直接嵌入 RobotObservation)."""
        frames = self.get_latest_frames()
        return {cid: frame.to_frame_ref() for cid, frame in frames.items()}

    def save_episode_frames(
        self,
        task_name: str,
        episode_id: int,
        frame_idx: int,
        frames: dict[str, CameraFrame] | None = None,
    ) -> dict[str, CameraFrameRef]:
        """将各相机当前帧持久化保存为独立 JPEG 图片，并返回包含相对路径的 CameraFrameRef.
        
        目录规则: <base_dir>/<task_name>/images/<camera_id>/frame_XXXXXX.jpg
        """
        captured = frames if frames is not None else self.get_latest_frames()
        frame_refs: dict[str, CameraFrameRef] = {}

        for cid, frame in captured.items():
            cam_dir = self.base_images_dir / task_name / "images" / cid
            cam_dir.mkdir(parents=True, exist_ok=True)

            filename = f"frame_{frame_idx:06d}.jpg"
            full_path = cam_dir / filename
            rel_path = f"images/{cid}/{filename}"

            try:
                # 写入 JPEG 字节
                with open(full_path, "wb") as f:
                    f.write(frame.to_jpeg_bytes())
                frame_refs[cid] = frame.to_frame_ref(relative_path=rel_path)
            except Exception as e:
                logger.error("保存相机 [%s] 帧图像失败: %s", cid, e)
                frame_refs[cid] = frame.to_frame_ref(relative_path=None)

        return frame_refs

    def get_mjpeg_stream(self, camera_id: str, fps: int = 25) -> Generator[bytes, None, None]:
        """生成 HTTP multipart/x-mixed-replace 视频流供浏览器前端渲染."""
        cam = self.get_camera(camera_id)
        if cam is None:
            return

        interval = 1.0 / max(1, fps)
        import time
        while cam.is_connected:
            frame = cam.get_latest_frame()
            if frame is not None:
                jpeg_bytes = frame.to_jpeg_bytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg_bytes)).encode() + b"\r\n\r\n"
                    + jpeg_bytes
                    + b"\r\n"
                )
            time.sleep(interval)

