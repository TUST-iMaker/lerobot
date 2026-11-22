"""Mocked driver tests for SO101FollowerArm, SO101LeaderArm, and hardware config (ZERO COM access)."""

from __future__ import annotations

import json
import math
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from core.data_types import (
    SO101_ARM_JOINT_NAMES,
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointUnit,
    RobotAction,
    deg_to_rad,
)
from hardware.config import ArmConfig, ArmRole, resolve_calibration_dir
from hardware.follower_arm import SO101FollowerArm
from hardware.leader_arm import SO101LeaderArm


def _build_fake_bus() -> MagicMock:
    bus = MagicMock()
    present = {j: 2047 for j in SO101_JOINT_NAMES}
    bus.sync_read.side_effect = lambda reg, normalize=True: (
        {j: 1 for j in SO101_JOINT_NAMES} if reg == "Torque_Enable" else dict(present)
    )
    return bus


class TestHardwareDriversMocked(unittest.TestCase):
    """Test SO101FollowerArm and SO101LeaderArm with mocked SerialMotorsBus/SOFollower/SOLeader."""

    def test_calibration_dir_and_env_resolution(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            cal_root = Path(tmpdir) / "calibration"
            with patch.dict("os.environ", {"HF_LEROBOT_CALIBRATION": str(cal_root)}, clear=False):
                f_dir = resolve_calibration_dir(ArmRole.FOLLOWER)
                l_dir = resolve_calibration_dir(ArmRole.LEADER)
                self.assertEqual(f_dir, cal_root / "robots" / "so_follower")
                self.assertEqual(l_dir, cal_root / "teleoperators" / "so_leader")

                cfg = ArmConfig.default_follower(
                    port="COM99",
                    robot_id="arm101_follower_01",
                    calibration_dir=cal_root,
                )
                self.assertEqual(cfg.calibration_dir, cal_root / "robots" / "so_follower")
                self.assertEqual(cfg.calibration_fpath, cal_root / "robots" / "so_follower" / "arm101_follower_01.json")

                leaf_dir = Path(tmpdir) / "direct_leaf"
                cfg_leaf = ArmConfig.default_follower(
                    port="COM99",
                    robot_id="arm101_follower_01",
                    calibration_dir=leaf_dir,
                )
                self.assertEqual(cfg_leaf.calibration_dir, leaf_dir)

    def test_follower_connect_is_calibrated_max_relative_target_and_gripper(self) -> None:
        captured_cfg = {}
        call_order: list[str] = []
        fake_bus = _build_fake_bus()

        def _sync_write(reg: str, values: dict, normalize: bool = True) -> None:
            call_order.append(f"sync_write:{reg}")

        def _disable_torque() -> None:
            call_order.append("disable_torque")

        def _enable_torque() -> None:
            call_order.append("enable_torque")

        fake_bus.sync_write.side_effect = _sync_write
        fake_bus.disable_torque.side_effect = _disable_torque
        fake_bus.enable_torque.side_effect = _enable_torque

        class FakeFollowerConfig:
            def __init__(self, **kwargs) -> None:
                captured_cfg.update(kwargs)

        class FakeFollower:
            def __init__(self, cfg) -> None:
                self.config = cfg
                self.bus = fake_bus
                self.cameras = {}
                self.is_calibrated = True
                self.calibration_fpath = Path(cfg["calibration_dir"]) / f"{cfg['id']}.json" if isinstance(cfg, dict) else Path("cal.json")
                self.sent_payloads: list[dict] = []

            def configure(self) -> None:
                call_order.append("configure")

            def get_observation(self) -> dict[str, float]:
                return {
                    "shoulder_pan.pos": 45.0,
                    "shoulder_lift.pos": 0.0,
                    "elbow_flex.pos": 0.0,
                    "wrist_flex.pos": 0.0,
                    "wrist_roll.pos": -30.0,
                    "gripper.pos": 100.0,
                }

            def send_action(self, payload: dict[str, float]) -> dict[str, float]:
                self.sent_payloads.append(dict(payload))
                return payload

            def disconnect(self) -> None:
                call_order.append("disconnect")

        with tempfile.TemporaryDirectory() as tmpdir:
            cal_dir = Path(tmpdir) / "robots" / "so_follower"
            cal_dir.mkdir(parents=True)
            cal_file = cal_dir / "arm101_follower_01.json"
            cal_file.write_text(
                json.dumps({j: {"id": i + 1, "drive_mode": 0, "homing_offset": 0, "range_min": 500, "range_max": 3500} for i, j in enumerate(SO101_JOINT_NAMES)}),
                encoding="utf-8",
            )

            arm_cfg = ArmConfig.default_follower(
                port="COM99",
                robot_id="arm101_follower_01",
                calibration_dir=cal_dir,
                max_relative_target=12.0,
            )
            adapter = SO101FollowerArm(arm_cfg)

            fake_mod = types.SimpleNamespace(SO101Follower=FakeFollower, SO101FollowerConfig=FakeFollowerConfig)
            with patch.dict("sys.modules", {"lerobot.robots.so_follower": fake_mod}):
                adapter.connect()

            self.assertEqual(captured_cfg["calibration_dir"], cal_dir)
            self.assertEqual(captured_cfg["id"], "arm101_follower_01")
            self.assertEqual(captured_cfg["max_relative_target"], 12.0)
            self.assertTrue(adapter.is_calibrated)

            # Assert Goal_Position written BEFORE configure() AND before final enable_torque()
            idx_cfg = call_order.index("configure")
            sync_indices = [i for i, item in enumerate(call_order) if item == "sync_write:Goal_Position"]
            self.assertTrue(any(i < idx_cfg for i in sync_indices))
            self.assertTrue(any(i > idx_cfg for i in sync_indices))
            self.assertLess(max(sync_indices), call_order.index("enable_torque"))

            # Read state: arm joints -> rad, gripper -> 100.0 (unclipped RANGE_0_100!)
            st = adapter.read_state()
            self.assertEqual(st.unit, JointUnit.RADIAN)
            self.assertEqual(st.gripper_unit, JointUnit.RANGE_0_100)
            self.assertAlmostEqual(st.positions["shoulder_pan"], deg_to_rad(45.0), places=5)
            self.assertAlmostEqual(st.positions["gripper"], 100.0, places=5)

            # Send JOINT_POSITION: arm rad -> deg, gripper 100.0 -> 100.0
            act = RobotAction(
                action_type=ActionType.JOINT_POSITION,
                source=ControlSource.KEYBOARD,
                timestamp=time.monotonic(),
                target_joint_positions={**{j: 0.0 for j in SO101_ARM_JOINT_NAMES}, "shoulder_pan": deg_to_rad(30.0), "gripper": 100.0},
                unit=JointUnit.RADIAN,
                gripper_unit=JointUnit.RANGE_0_100,
            )
            self.assertTrue(adapter.send_action(act))
            payload = adapter._follower.sent_payloads[-1]
            self.assertAlmostEqual(payload["shoulder_pan.pos"], 30.0, places=4)
            self.assertAlmostEqual(payload["gripper.pos"], 100.0, places=4)

            # Send JOINT_DELTA: mixes cur_state(rad) + delta(rad) cleanly, gripper +delta in %
            act_delta = RobotAction(
                action_type=ActionType.JOINT_DELTA,
                source=ControlSource.KEYBOARD,
                timestamp=time.monotonic(),
                target_joint_deltas={"shoulder_pan": deg_to_rad(5.0), "gripper": -20.0},
                unit=JointUnit.RADIAN,
                gripper_unit=JointUnit.RANGE_0_100,
            )
            self.assertTrue(adapter.send_action(act_delta))
            payload_d = adapter._follower.sent_payloads[-1]
            self.assertAlmostEqual(payload_d["shoulder_pan.pos"], 50.0, places=4)
            self.assertAlmostEqual(payload_d["gripper.pos"], 80.0, places=4)

            # move_to_middle: moves arm joints toward 0.0, does NOT force gripper=100.0 to 0.0
            obs_seq = iter([
                {"shoulder_pan.pos": 2.0, "shoulder_lift.pos": 0.0, "elbow_flex.pos": 0.0, "wrist_flex.pos": 0.0, "wrist_roll.pos": 0.0, "gripper.pos": 80.0},
                {"shoulder_pan.pos": 0.0, "shoulder_lift.pos": 0.0, "elbow_flex.pos": 0.0, "wrist_flex.pos": 0.0, "wrist_roll.pos": 0.0, "gripper.pos": 80.0},
            ])
            adapter._follower.get_observation = lambda: next(obs_seq)
            self.assertTrue(adapter.move_to_middle(step_deg=5.0, tolerance_deg=1.0, timeout_s=2.0))
            payload_mid = adapter._follower.sent_payloads[-1]
            self.assertAlmostEqual(payload_mid["shoulder_pan.pos"], 0.0, places=4)
            self.assertAlmostEqual(payload_mid["gripper.pos"], 80.0, places=4)

            adapter.disconnect()

    def test_leader_connect_nonblocking_calibration_dir_and_gripper(self) -> None:
        captured_cfg = {}
        connect_kwargs = {}
        fake_bus = _build_fake_bus()

        class FakeLeaderConfig:
            def __init__(self, **kwargs) -> None:
                captured_cfg.update(kwargs)

        class FakeLeader:
            def __init__(self, cfg) -> None:
                self.config = cfg
                self.bus = fake_bus
                self.is_calibrated = True
                self.calibration_fpath = Path("leader.json")

            def connect(self, **kwargs) -> None:
                connect_kwargs.update(kwargs)

            def get_action(self) -> dict[str, float]:
                return {
                    "shoulder_pan.pos": 15.0,
                    "shoulder_lift.pos": 0.0,
                    "elbow_flex.pos": 0.0,
                    "wrist_flex.pos": 0.0,
                    "wrist_roll.pos": 0.0,
                    "gripper.pos": 65.0,
                }

            def disconnect(self) -> None:
                pass

        with tempfile.TemporaryDirectory() as tmpdir:
            cal_dir = Path(tmpdir) / "teleoperators" / "so_leader"
            cal_dir.mkdir(parents=True)
            arm_cfg = ArmConfig.default_leader(
                port="COM98",
                robot_id="arm101_leader_01",
                calibration_dir=cal_dir,
            )
            object.__setattr__(arm_cfg, "joint_offsets_deg", {"wrist_roll": 10.0, "gripper": 50.0})
            adapter = SO101LeaderArm(arm_cfg)

            fake_mod = types.SimpleNamespace(SO101Leader=FakeLeader, SO101LeaderConfig=FakeLeaderConfig)
            with patch.dict("sys.modules", {"lerobot.teleoperators.so_leader": fake_mod}):
                adapter.connect()

            self.assertEqual(captured_cfg["calibration_dir"], cal_dir)
            self.assertEqual(connect_kwargs.get("calibrate"), False)

            act = adapter.read_action()
            self.assertEqual(act.gripper_unit, JointUnit.RANGE_0_100)
            self.assertAlmostEqual(act.target_joint_positions["wrist_roll"], deg_to_rad(10.0), places=5)
            # Gripper offset skipped and percentage preserved
            self.assertAlmostEqual(act.target_joint_positions["gripper"], 65.0, places=5)
            adapter.disconnect()


if __name__ == "__main__":
    unittest.main()
