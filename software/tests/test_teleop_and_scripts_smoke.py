"""Windows/cross-OS smoke tests for teleop/*, root scripts, and hardware.doctor (ZERO hardware access)."""

from __future__ import annotations

import importlib
import json
import os
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from core.action_api import ActionAPI
from core.data_types import ControlSource, JointState, JointUnit, SO101_JOINT_NAMES
from hardware.config import RobotSystemConfig


class TestTeleopAndScriptsSmoke(unittest.TestCase):
    """Verify imports, official_wrapper cmd/env, DemoRoutineRunner, doctor, save_pose, web_ui, hardware_init."""

    def test_all_teleop_and_camera_modules_importable_on_windows(self) -> None:
        modules = [
            "teleop.leader_follower",
            "teleop.keyboard_joint",
            "teleop.official_wrapper",
            "teleop.keyboard_ee",
            "teleop.coordinate_tool",
            "teleop.demo_routine",
            "cameras.opencv_camera",
            "cameras.discovery",
            "cameras.teleop_camera",
        ]
        for mod in modules:
            imported = importlib.import_module(mod)
            self.assertIsNotNone(imported)

    def test_official_wrapper_cmd_and_env(self) -> None:
        from teleop import official_wrapper

        env = official_wrapper._build_subprocess_env()
        if "PYTHONPATH" in env:
            # Must not use ':' as path separator on Windows when multiple paths exist, and must not add bogus <root>/src
            self.assertNotIn(str(official_wrapper._lerobot_root / "src"), env["PYTHONPATH"].split(os.pathsep))

        cmd, f_port, f_id = official_wrapper.build_official_record_cmd(
            follower_port="COM11",
            leader_port="COM12",
            follower_id="arm101_follower_01",
            leader_id="arm101_leader_01",
            camera_id=0,
            camera_enabled=True,
            fps=None,
        )
        self.assertEqual(f_port, "COM11")
        self.assertEqual(f_id, "arm101_follower_01")
        self.assertTrue(any(a.startswith("--robot.calibration_dir=") and "so_follower" in a for a in cmd))
        self.assertTrue(any(a.startswith("--teleop.calibration_dir=") and "so_leader" in a for a in cmd))
        cam_arg = next(a for a in cmd if a.startswith("--robot.cameras="))
        cam_json = json.loads(cam_arg.split("=", 1)[1])
        self.assertIsInstance(cam_json["cam_robot"]["fps"], int)
        self.assertNotIn('"fps": null', cam_arg)

        # Verify safe_park_to_rest backward-compatible signature
        with patch("core.action_api.ActionAPI") as mock_api_cls:
            mock_api = mock_api_cls.return_value
            mock_api.connect.return_value = True
            mock_api.safe_return_to_rest.return_value = True
            with patch("teleop.official_wrapper.time.sleep"):
                ok = official_wrapper.safe_park_to_rest(port="COM11", duration_s=1.0)
            self.assertTrue(ok)

    def test_demo_routine_pick_on_mock_arm(self) -> None:
        from teleop.demo_routine import DemoRoutineRunner

        api = ActionAPI(config=RobotSystemConfig.mock_system())
        self.assertTrue(api.connect(connect_leader=False))
        api.claim_control(ControlSource.SCRIPT)
        runner = DemoRoutineRunner(api)
        with patch("teleop.demo_routine.time.sleep"), patch("core.action_api.time.sleep"):
            ok = runner.run_pick_routine()
        self.assertTrue(ok)
        st = api.get_joint_state()
        self.assertGreaterEqual(st.positions["gripper"], 0.0)
        self.assertLessEqual(st.positions["gripper"], 100.0)
        api.disconnect(keep_torque=False, return_to_rest=False)

    def test_doctor_serial_ports_mocked(self) -> None:
        from hardware import doctor

        fake_ports = [
            types.SimpleNamespace(device="COM12", description="USB Serial (COM12)"),
            types.SimpleNamespace(device="COM11", description="USB Serial (COM11)"),
        ]
        with (
            patch("hardware.doctor.platform.system", return_value="Windows"),
            patch("serial.tools.list_ports.comports", return_value=fake_ports),
        ):
            ports = doctor.check_serial_ports()
        self.assertEqual([p["port"] for p in ports], ["COM11", "COM12"])
        self.assertTrue(all(p["accessible"] for p in ports))

    def test_so_arm101_hardware_init_checks_rustypot_and_2047_sync_write(self) -> None:
        import so_arm101_hardware_init as hw_init

        self.assertFalse(hasattr(hw_init, "FIXED_CALIBRATION_FILE"))
        self.assertNotIn("scservo_sdk", hw_init.__file__)
        fake_bus = MagicMock()
        fake_bus.sync_read.return_value = {j: 2100 for j in SO101_JOINT_NAMES}
        fake_follower = MagicMock(bus=fake_bus)
        args = types.SimpleNamespace(middle_step=10.0, middle_tolerance=5.0, middle_timeout=2.0, yes=True)
        ranges = {j: (500, 3500) for j in SO101_JOINT_NAMES}

        # Step once (error 253 > tol 57) then converge to 2047
        seq = iter([
            {j: 2300 for j in SO101_JOINT_NAMES},
            {j: 2047 for j in SO101_JOINT_NAMES},
        ])
        fake_bus.sync_read.side_effect = lambda *a, **kw: next(seq)
        res = hw_init.move_all_to_middle(fake_follower, args, ranges)
        self.assertEqual(res["raw_targets"]["shoulder_pan"], 2047)
        self.assertNotIn("gripper", res["raw_targets"])
        fake_bus.sync_write.assert_called()

    def test_save_pose_leader_and_mock_cli(self) -> None:
        import save_pose

        fake_ports = [types.SimpleNamespace(device="COM11")]
        with (
            patch("save_pose.platform.system", return_value="Windows"),
            patch("serial.tools.list_ports.comports", return_value=fake_ports),
        ):
            self.assertEqual(save_pose.detect_serial_ports(), ["COM11"])

        with patch("sys.argv", ["save_pose.py", "--mock", "--output", str(Path.cwd() / "software" / "tests" / "_tmp_pose.json")]):
            tmp_out = Path.cwd() / "software" / "tests" / "_tmp_pose.json"
            try:
                rc = save_pose.main()
                self.assertEqual(rc, 0)
                data = json.loads(tmp_out.read_text(encoding="utf-8"))
                self.assertEqual(data["gripper_unit"], "range_0_100")
            finally:
                if tmp_out.exists():
                    tmp_out.unlink()

        # Verify capture_pose(arm_type="leader") uses SO101Leader and calls write_calibration after range expansion
        with patch.dict("os.environ", {"HF_LEROBOT_CALIBRATION": str(Path.cwd() / "software" / "tests" / "_tmp_cal")}, clear=False):
            tmp_cal_root = Path.cwd() / "software" / "tests" / "_tmp_cal"
            leader_cal_dir = tmp_cal_root / "teleoperators" / "so_leader"
            leader_cal_dir.mkdir(parents=True, exist_ok=True)
            cal_json = leader_cal_dir / "test_leader.json"
            cal_json.write_text(
                json.dumps({"shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 1000, "range_max": 3000}}),
                encoding="utf-8",
            )
            fake_bus = MagicMock()
            fake_bus.sync_read.return_value = {"shoulder_pan": 500, "gripper": 2000}

            class FakeLeader:
                def __init__(self, cfg) -> None:
                    self.cfg = cfg
                    self.bus = fake_bus
                    self.calibration = {"shoulder_pan": {"range_min": 1000, "range_max": 3000}}
                    self.reloaded = False

                def _load_calibration(self, p) -> None:
                    self.reloaded = True
                    self.calibration = json.loads(Path(p).read_text(encoding="utf-8"))

                def get_action(self) -> dict[str, float]:
                    return {"shoulder_pan.pos": -45.0, "gripper.pos": 55.0}

                def disconnect(self) -> None:
                    pass

            fake_mod = types.SimpleNamespace(SO101Leader=FakeLeader, SO101LeaderConfig=lambda **kw: kw)
            try:
                with patch.dict("sys.modules", {"lerobot.teleoperators.so_leader": fake_mod}), patch("save_pose.time.sleep"):
                    pos, raw = save_pose.capture_pose("COM12", robot_id="test_leader", arm_type="leader")
                self.assertEqual(pos["gripper"], 55.0)
                fake_bus.write_calibration.assert_called_once()
            finally:
                import shutil
                shutil.rmtree(tmp_cal_root, ignore_errors=True)

    def test_web_ui_estop_and_gripper_step(self) -> None:
        import web_ui

        mock_api = MagicMock()
        mock_api.state_machine.active_source = ControlSource.KEYBOARD
        mock_api.state_machine.is_estop = False
        mock_api.state_machine.current_state.value = "TELEOP"
        mock_api.get_joint_state.return_value = JointState(
            timestamp=1.0,
            positions={j: 0.0 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        mock_api.submit_action.return_value = (True, "ok")

        handler = web_ui.WebRequestHandler.__new__(web_ui.WebRequestHandler)
        sent_responses: list[dict] = []
        handler.send_json = lambda d, status=200: sent_responses.append(d)

        with patch.object(web_ui, "_GLOBAL_API", mock_api):
            handler.handle_post_estop()
            mock_api.request_estop.assert_called_once()

            handler.handle_post_control_key({"key": " "})
            self.assertEqual(mock_api.request_estop.call_count, 2)

            handler.handle_post_control_key({"key": "c"})
            submitted = mock_api.submit_action.call_args[0][0]
            self.assertAlmostEqual(submitted.target_joint_positions["gripper"], 4.0, places=4)

            # Verify /api/status keeps gripper unscaled in RANGE_0_100 while converting arm joints
            mock_api.get_joint_state.return_value = JointState(
                timestamp=2.0,
                positions={**{j: 0.0 for j in SO101_JOINT_NAMES}, "shoulder_pan": 1.0, "gripper": 62.5},
                unit=JointUnit.RADIAN,
            )
            mock_api.hardware.follower = None
            mock_api.hardware.config.use_mock = True
            sent_responses.clear()
            handler.handle_get_status()
            status_payload = sent_responses[-1]
            self.assertAlmostEqual(status_payload["joints"]["gripper"], 62.5, places=2)
            self.assertGreater(status_payload["joints"]["shoulder_pan"], 50.0)


if __name__ == "__main__":
    unittest.main()
