"""Unit and mocked integration tests for cameras module (ZERO physical camera access)."""

from __future__ import annotations

import os
import sys
import tempfile
import time
import types
import unittest
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
from PIL import Image

from core.data_types import CameraFrameRef, JointState, RobotObservation
from cameras import (
    CameraBackendType,
    CameraConfig,
    CameraDeviceInfo,
    CameraFrame,
    CameraManager,
    MockCamera,
    OpenCVCamera,
    V4L2Camera,
    create_camera,
    discover_cameras,
    list_capture_cameras,
)


def _make_fake_cv2(
    actual_w: int = 640,
    actual_h: int = 480,
    actual_fps: float = 30.0,
    fourcc_str: str = "MJPG",
    opened: bool = True,
) -> tuple[types.SimpleNamespace, MagicMock, list[tuple[int, Any]]]:
    set_calls: list[tuple[int, Any]] = []
    cap = MagicMock()
    cap.isOpened.return_value = opened

    fourcc_val = sum(ord(c) << (8 * i) for i, c in enumerate(fourcc_str[:4]))

    def _cap_set(prop: int, val: Any) -> bool:
        set_calls.append((prop, val))
        return True

    def _cap_get(prop: int) -> float:
        if prop == 3:
            return float(actual_w)
        if prop == 4:
            return float(actual_h)
        if prop == 5:
            return float(actual_fps)
        if prop == 6:
            return float(fourcc_val)
        return 0.0

    cap.set.side_effect = _cap_set
    cap.get.side_effect = _cap_get
    sample_bgr = np.zeros((actual_h, actual_w, 3), dtype=np.uint8)
    sample_bgr[:, :, 0] = 255  # Pure blue in BGR -> (0, 0, 255) in RGB
    cap.read.return_value = (True, sample_bgr)
    cap.getBackendName.return_value = "DSHOW"

    fake_cv2 = types.SimpleNamespace(
        CAP_DSHOW=700,
        CAP_ANY=0,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        CAP_PROP_FOURCC=6,
        CAP_PROP_BUFFERSIZE=38,
        ROTATE_90_CLOCKWISE=0,
        ROTATE_180=1,
        ROTATE_90_COUNTERCLOCKWISE=2,
        COLOR_BGR2RGB=4,
        COLOR_BGR2GRAY=6,
        VideoCapture=MagicMock(return_value=cap),
        VideoWriter=types.SimpleNamespace(
            fourcc=lambda a, b, c, d: sum(ord(ch) << (8 * i) for i, ch in enumerate((a, b, c, d)))
        ),
        VideoWriter_fourcc=lambda a, b, c, d: sum(ord(ch) << (8 * i) for i, ch in enumerate((a, b, c, d))),
        setNumThreads=MagicMock(),
        rotate=lambda img, code: np.rot90(img, k=3 if code == 0 else (2 if code == 1 else 1)).copy(),
        cvtColor=lambda img, code: (
            img[:, :, ::-1].copy() if code == 4 else img[:, :, 0].copy()
        ),
    )
    return fake_cv2, cap, set_calls


class TestCameraModule(unittest.TestCase):
    """Test suite for cameras package (100% mocked, zero physical device probes)."""

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

        pil_img = frame.to_pil()
        self.assertIsInstance(pil_img, Image.Image)
        self.assertEqual(pil_img.size, (320, 240))
        self.assertEqual(pil_img.mode, "RGB")

        jpeg_bytes = frame.to_jpeg_bytes()
        self.assertEqual(jpeg_bytes[:2], b"\xff\xd8")

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

        obs = RobotObservation(
            timestamp=100.0,
            joint_state=JointState(timestamp=100.0, positions={"shoulder_pan": 0.0}),
            camera_frames={"cam_wrist": ref},
        )
        obs_dict = obs.to_dict()
        self.assertIn("cam_wrist", obs_dict["camera_frames"])
        self.assertEqual(
            obs_dict["camera_frames"]["cam_wrist"]["relative_path"],
            "images/cam_wrist/frame_000001.jpg",
        )
        cam.disconnect()

    def test_camera_manager_lifecycle(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            cm = CameraManager(base_images_dir=tmpdir)
            cfg1 = CameraConfig.mock_camera(camera_id="cam1")
            cfg2 = CameraConfig.mock_camera(camera_id="cam2")

            cm.add_camera(cfg1)
            cm.add_camera(cfg2)
            self.assertEqual(set(cm.camera_ids), {"cam1", "cam2"})

            res = cm.connect_all()
            self.assertTrue(res["cam1"])
            self.assertTrue(res["cam2"])

            frames = cm.capture_all()
            self.assertEqual(len(frames), 2)

            latest = cm.get_latest_frames()
            self.assertEqual(len(latest), 2)

            obs_refs = cm.get_observation_frame_refs()
            self.assertEqual(len(obs_refs), 2)

            saved_refs = cm.save_episode_frames(task_name="test_task", episode_id=0, frame_idx=5, frames=frames)
            self.assertEqual(len(saved_refs), 2)

            f1 = cm.base_images_dir / "test_task" / saved_refs["cam1"].relative_path
            f2 = cm.base_images_dir / "test_task" / saved_refs["cam2"].relative_path
            self.assertTrue(f1.exists())
            self.assertTrue(f2.exists())

            stream_gen = cm.get_mjpeg_stream("cam1")
            chunk = next(stream_gen)
            self.assertIn(b"--frame", chunk)
            self.assertIn(b"Content-Type: image/jpeg", chunk)

            cm.disconnect_all()
            for cid in cm.camera_ids:
                cam = cm.get_camera(cid)
                self.assertIsNotNone(cam)
                self.assertFalse(cam.is_connected)

    def test_camera_discovery_mocked(self) -> None:
        fake_devices = [
            CameraDeviceInfo(
                index=7,
                device_path="7",
                card_name="USB2.0_CAM1 (icspring)",
                driver_name="dshow",
                bus_info="dshow:7",
                is_capture=True,
                is_metadata=False,
                supported_formats=["MJPG"],
            ),
            CameraDeviceInfo(
                index=1,
                device_path="1",
                card_name="Metadata Node",
                driver_name="uvcvideo",
                bus_info="usb:1",
                is_capture=False,
                is_metadata=True,
                supported_formats=[],
            ),
        ]
        with patch("cameras.discovery.discover_cameras", return_value=fake_devices):
            capture_only = list_capture_cameras()
            self.assertEqual(len(capture_only), 1)
            self.assertEqual(capture_only[0].index, 7)

        # Direct discovery Windows 0..59 fallback scan with mocked cv2 (zero real hardware probes)
        fake_cv2, cap, _ = _make_fake_cv2()
        opened_indices: list[int] = []

        def _video_cap(idx: int, api: int = 0) -> MagicMock:
            opened_indices.append(idx)
            c = MagicMock()
            c.isOpened.return_value = (idx == 7)
            c.getBackendName.return_value = "DSHOW"
            c.get.return_value = float(sum(ord(ch) << (8 * i) for i, ch in enumerate("MJPG")))
            return c

        fake_cv2.VideoCapture.side_effect = _video_cap
        with (
            patch("cameras.discovery.platform.system", return_value="Windows"),
            patch.dict(sys.modules, {"cv2": fake_cv2}),
            patch.dict("sys.modules", {"lerobot.cameras.opencv": None}),
        ):
            cams = discover_cameras()
            self.assertEqual(len(opened_indices), 60)
            self.assertEqual(len(cams), 1)
            self.assertEqual(cams[0].index, 7)
            self.assertEqual(cams[0].supported_formats, ["MJPG"])

    def test_factory_creation(self) -> None:
        mock_cfg = CameraConfig.mock_camera(camera_id="cam_factory_mock")
        cam = create_camera(mock_cfg)
        self.assertIsInstance(cam, MockCamera)

        v4l_cfg = CameraConfig(camera_id="cam_v4l", device="/dev/video_non_existent", backend=CameraBackendType.V4L2)
        cam2 = create_camera(v4l_cfg)
        self.assertIsInstance(cam2, V4L2Camera)

    def test_auto_discover_and_register_wrist_preference_mocked(self) -> None:
        fake_capture_cams = [
            CameraDeviceInfo(
                index=0,
                device_path="0",
                card_name="Integrated Camera",
                driver_name="dshow",
                bus_info="dshow:0",
                is_capture=True,
                is_metadata=False,
                supported_formats=["MJPG"],
            ),
            CameraDeviceInfo(
                index=2,
                device_path="2",
                card_name="icspring wrist camera",
                driver_name="dshow",
                bus_info="dshow:2",
                is_capture=True,
                is_metadata=False,
                supported_formats=["MJPG"],
            ),
        ]
        with (
            patch("cameras.manager.list_capture_cameras", return_value=fake_capture_cams),
            tempfile.TemporaryDirectory() as tmpdir,
        ):
            cm = CameraManager(base_images_dir=tmpdir, is_sim=True)
            reg = cm.auto_discover_and_register(max_cameras=2)
            self.assertEqual(reg[0], "cam_wrist")
            self.assertIn("cam_top", reg)

    def test_opencv_camera_windows_dshow_fourcc_readback_color_and_rotation(self) -> None:
        fake_cv2, cap, set_calls = _make_fake_cv2(actual_w=640, actual_h=480, actual_fps=30.0)
        cfg = CameraConfig(
            camera_id="cam_cv",
            device=0,
            width=640,
            height=480,
            fps=30,
            color_mode="RGB",
            backend=CameraBackendType.OPENCV,
            rotation=0,
            async_mode=False,
            fourcc="MJPG",
        )
        cam = OpenCVCamera(cfg)
        with (
            patch("cameras.opencv_camera.platform.system", return_value="Windows"),
            patch.dict(sys.modules, {"cv2": fake_cv2}),
        ):
            self.assertTrue(cam.connect())
            self.assertEqual(os.environ.get("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"), "0")
            # Windows: WIDTH(3), HEIGHT(4), FPS(5) must be set BEFORE FOURCC(6)
            props = [p for p, _ in set_calls]
            self.assertLess(props.index(5), props.index(6))

            frame = cam.read_frame()
            self.assertEqual(frame.color_format, "RGB")
            self.assertEqual(frame.width, 640)
            self.assertEqual(frame.height, 480)
            # BGR (255, 0, 0) -> RGB (0, 0, 255)
            self.assertEqual(frame.raw_bytes[:3], bytes([0, 0, 255]))
            cam.disconnect()

    def test_opencv_camera_readback_mismatch_raises(self) -> None:
        fake_cv2, cap, _ = _make_fake_cv2(actual_w=320, actual_h=240, actual_fps=30.0)
        cfg = CameraConfig(
            camera_id="cam_bad",
            device=0,
            width=640,
            height=480,
            fps=30,
            backend=CameraBackendType.OPENCV,
            async_mode=False,
        )
        cam = OpenCVCamera(cfg)
        with (
            patch("cameras.opencv_camera.platform.system", return_value="Windows"),
            patch.dict(sys.modules, {"cv2": fake_cv2}),
        ):
            with self.assertRaises(RuntimeError):
                cam.connect()
            cap.release.assert_called()

    def test_base_camera_staleness_and_read_latest(self) -> None:
        cfg = CameraConfig.mock_camera(camera_id="cam_stale")
        cam = MockCamera(CameraConfig(
            camera_id=cfg.camera_id,
            device="mock",
            backend=CameraBackendType.MOCK,
            async_mode=False,
            max_age_ms=50,
        ))
        cam.connect()
        frame = cam.read_frame()
        self.assertIsNotNone(cam.get_latest_frame(max_age_ms=500))
        self.assertEqual(cam.read_latest(max_age_ms=500).frame_id, frame.frame_id)

        # Simulate stale frame
        cam._latest_timestamp = time.monotonic() - 1.0
        self.assertIsNone(cam.get_latest_frame(max_age_ms=50))
        with self.assertRaises(TimeoutError):
            cam.read_latest(max_age_ms=50)
        cam.disconnect()


if __name__ == "__main__":
    unittest.main()
