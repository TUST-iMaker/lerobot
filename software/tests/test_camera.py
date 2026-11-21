"""Unit and integration tests for cameras module."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from PIL import Image

from core.data_types import CameraFrameRef, JointState, RobotObservation
from cameras import (
    CameraBackendType,
    CameraConfig,
    CameraDeviceInfo,
    CameraFrame,
    CameraManager,
    MockCamera,
    V4L2Camera,
    create_camera,
    discover_cameras,
    list_capture_cameras,
)


class TestCameraModule(unittest.TestCase):
    """Test suite for cameras package."""

    def test_mock_camera_capture_and_pil(self) -> None:
        cfg = CameraConfig.mock_camera(camera_id="cam_test", width=320, height=240, fps=30)
        cam = MockCamera(cfg)

        self.assertFalse(cam.is_connected)
        self.assertTrue(cam.connect())
        self.assertTrue(cam.is_connected)

        frame = cam.read_frame()
        self.assertIsInstance(frame, CameraFrame)
        self.assertEqual(frame.camera_id, "cam_test")
        self.assertEqual(frame.width, 320)
        self.assertEqual(frame.height, 240)
        self.assertEqual(frame.frame_id, 1)
        self.assertGreater(len(frame.raw_bytes), 0)

        # 验证能够无损还原为 PIL Image
        pil_img = frame.to_pil()
        self.assertIsInstance(pil_img, Image.Image)
        self.assertEqual(pil_img.size, (320, 240))
        self.assertEqual(pil_img.mode, "RGB")

        # 验证转换为 JPEG 字节
        jpeg_bytes = frame.to_jpeg_bytes()
        self.assertEqual(jpeg_bytes[:2], b"\xff\xd8")  # JPEG SOI 标记

        cam.disconnect()
        self.assertFalse(cam.is_connected)

    def test_camera_frame_to_ref(self) -> None:
        cfg = CameraConfig.mock_camera(camera_id="cam_wrist")
        cam = MockCamera(cfg)
        cam.connect()
        frame = cam.read_frame()

        ref = frame.to_frame_ref(relative_path="images/cam_wrist/frame_000001.jpg")
        self.assertIsInstance(ref, CameraFrameRef)
        self.assertEqual(ref.camera_id, "cam_wrist")
        self.assertEqual(ref.frame_id, 1)
        self.assertEqual(ref.width, 640)
        self.assertEqual(ref.height, 480)
        self.assertEqual(ref.relative_path, "images/cam_wrist/frame_000001.jpg")

        # 验证嵌入 RobotObservation 并序列化
        obs = RobotObservation(
            timestamp=100.0,
            joint_state=JointState(timestamp=100.0, positions={"shoulder_pan": 0.0}),
            camera_frames={"cam_wrist": ref},
        )
        obs_dict = obs.to_dict()
        self.assertIn("cam_wrist", obs_dict["camera_frames"])
        self.assertEqual(obs_dict["camera_frames"]["cam_wrist"]["relative_path"], "images/cam_wrist/frame_000001.jpg")

        cam.disconnect()

    def test_camera_manager_lifecycle(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            cm = CameraManager(base_images_dir=tmpdir)
            cfg1 = CameraConfig.mock_camera(camera_id="cam1")
            cfg2 = CameraConfig.mock_camera(camera_id="cam2")

            cm.add_camera(cfg1)
            cm.add_camera(cfg2)
            self.assertEqual(set(cm.camera_ids), {"cam1", "cam2"})

            # 连接所有相机
            res = cm.connect_all()
            self.assertTrue(res["cam1"])
            self.assertTrue(res["cam2"])

            # 抓取各相机帧
            frames = cm.capture_all()
            self.assertEqual(len(frames), 2)
            self.assertIn("cam1", frames)
            self.assertIn("cam2", frames)

            # 获取最新缓存帧
            latest = cm.get_latest_frames()
            self.assertEqual(len(latest), 2)

            # 获取观测字典
            obs_refs = cm.get_observation_frame_refs()
            self.assertEqual(len(obs_refs), 2)
            self.assertEqual(obs_refs["cam1"].camera_id, "cam1")

            # 保存当帧落盘
            saved_refs = cm.save_episode_frames(task_name="test_task", episode_id=0, frame_idx=5, frames=frames)
            self.assertEqual(len(saved_refs), 2)

            f1 = cm.base_images_dir / "test_task" / saved_refs["cam1"].relative_path
            f2 = cm.base_images_dir / "test_task" / saved_refs["cam2"].relative_path
            self.assertTrue(f1.exists())
            self.assertTrue(f2.exists())
            self.assertGreater(f1.stat().st_size, 1000)
            self.assertGreater(f2.stat().st_size, 1000)

            # 测试 MJPEG 流生成器
            stream_gen = cm.get_mjpeg_stream("cam1")
            chunk = next(stream_gen)
            self.assertIn(b"--frame", chunk)
            self.assertIn(b"Content-Type: image/jpeg", chunk)

            cm.disconnect_all()
            for cid in cm.camera_ids:
                cam = cm.get_camera(cid)
                self.assertIsNotNone(cam)
                self.assertFalse(cam.is_connected)

    def test_camera_discovery(self) -> None:
        # 执行物理相机探测 (即使无物理摄像头也不会崩溃)
        devices = discover_cameras()
        self.assertIsInstance(devices, list)
        for dev in devices:
            self.assertIsInstance(dev, CameraDeviceInfo)
            self.assertTrue(dev.device_path.startswith("/dev/video"))

        capture_only = list_capture_cameras()
        self.assertIsInstance(capture_only, list)
        for dev in capture_only:
            self.assertTrue(dev.is_capture)
            self.assertFalse(dev.is_metadata)

    def test_factory_creation(self) -> None:
        mock_cfg = CameraConfig.mock_camera(camera_id="cam_factory_mock")
        cam = create_camera(mock_cfg)
        self.assertIsInstance(cam, MockCamera)

        v4l_cfg = CameraConfig(camera_id="cam_v4l", device="/dev/video_non_existent", backend=CameraBackendType.V4L2)
        cam2 = create_camera(v4l_cfg)
        self.assertIsInstance(cam2, V4L2Camera)

    def test_auto_discover_and_register_wrist_preference(self) -> None:
        """测试相机自动探测注册对腕部相机的优先识别机制."""
        with tempfile.TemporaryDirectory() as tmpdir:
            cm = CameraManager(base_images_dir=tmpdir, is_sim=True)
            reg = cm.auto_discover_and_register(max_cameras=2)
            self.assertIsInstance(reg, list)
            self.assertGreater(len(reg), 0)
            # 若系统存在 icspring 相机，应自动被识别为 cam_wrist 且置于首位
            cams = cm._cameras
            if any("icspring" in getattr(c.config, "device", "").lower() or "video2" in str(getattr(c.config, "device", "")) for c in cams.values()):
                self.assertIn("cam_wrist", reg)
                self.assertEqual(reg[0], "cam_wrist")


if __name__ == "__main__":
    unittest.main()

