"""Unit tests for redesigned hardware package."""

from __future__ import annotations

import unittest
from pathlib import Path

from core.data_types import (
    ActionType,
    ControlSource,
    JointUnit,
    RobotAction,
    SO101_JOINT_NAMES,
)
from hardware import (
    ArmConfig,
    ArmRole,
    BaseArmAdapter,
    RobotSystemAdapter,
    RobotSystemConfig,
    SO101FollowerArm,
    SO101LeaderArm,
)
from hardware.config import DEFAULT_FOLLOWER_CALIBRATION


class TestHardwareAdapters(unittest.TestCase):
    def setUp(self) -> None:
        self.follower_cfg = ArmConfig.default_follower()
        self.leader_cfg = ArmConfig.default_leader()
        self.follower = SO101FollowerArm(self.follower_cfg)
        self.leader = SO101LeaderArm(self.leader_cfg)

    def test_config_roles(self) -> None:
        self.assertEqual(self.follower_cfg.role, ArmRole.FOLLOWER)
        self.assertEqual(self.leader_cfg.role, ArmRole.LEADER)
        self.assertEqual(self.follower_cfg.port, "/dev/ttyACM0")
        self.assertEqual(self.leader_cfg.port, "/dev/ttyACM1")

    def test_follower_unconnected_safety(self) -> None:
        self.assertIsInstance(self.follower, BaseArmAdapter)
        self.assertFalse(self.follower.is_connected)

        with self.assertRaises(RuntimeError):
            self.follower.read_state()

        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=1.0,
            target_joint_positions={j: 0.0 for j in SO101_JOINT_NAMES},
        )
        with self.assertRaises(RuntimeError):
            self.follower.send_action(action)

        with self.assertRaises(RuntimeError):
            self.follower.move_to_middle()

        # stop_motion 与 disconnect 在未连接时必须安全静默
        self.follower.stop_motion()
        self.follower.disconnect()

    def test_leader_unconnected_safety(self) -> None:
        self.assertIsInstance(self.leader, BaseArmAdapter)
        self.assertFalse(self.leader.is_connected)

        with self.assertRaises(RuntimeError):
            self.leader.read_state()

        with self.assertRaises(RuntimeError):
            self.leader.read_action()

        self.leader.disconnect()

    def test_calibration_parsing(self) -> None:
        if DEFAULT_FOLLOWER_CALIBRATION.is_file():
            ranges = self.follower.load_calibration()
            self.assertEqual(len(ranges), 6)
            for j in SO101_JOINT_NAMES:
                self.assertIn(j, ranges)
                self.assertLess(ranges[j][0], ranges[j][1])

    def test_system_adapter_integration(self) -> None:
        sys_cfg = RobotSystemConfig(
            follower=self.follower_cfg,
            leader=self.leader_cfg,
        )
        sys_adapter = RobotSystemAdapter(sys_cfg)
        self.assertFalse(sys_adapter.is_follower_connected)
        self.assertFalse(sys_adapter.is_leader_connected)

        health = sys_adapter.get_health_report()
        self.assertIn("follower", health)
        self.assertIn("leader", health)

    def test_real_system_config_factory(self) -> None:
        cfg = RobotSystemConfig.real_system(
            follower_port="/dev/ttyACM10",
            leader_port="/dev/ttyACM11",
            frequency_hz=50.0,
        )
        self.assertFalse(cfg.use_mock)
        self.assertEqual(cfg.follower.port, "/dev/ttyACM10")
        self.assertIsNotNone(cfg.leader)
        self.assertEqual(cfg.leader.port, "/dev/ttyACM11")
        self.assertEqual(cfg.frequency_hz, 50.0)

    def test_context_manager_mock_adapter(self) -> None:
        mock_cfg = RobotSystemConfig.mock_system(with_leader=True)
        with RobotSystemAdapter(mock_cfg) as adapter:
            self.assertTrue(adapter.is_follower_connected)
            self.assertTrue(adapter.is_leader_connected)
            state = adapter.read_follower_state()
            self.assertIsNotNone(state)
        self.assertFalse(adapter.is_follower_connected)
        self.assertFalse(adapter.is_leader_connected)

    def test_leader_joint_offsets(self) -> None:
        from core.data_types import deg_to_rad
        cfg = ArmConfig.default_leader(is_mock=True)
        # 赋予 90° 偏移
        object.__setattr__(cfg, "joint_offsets_deg", {"wrist_roll": 90.0})
        leader = SO101LeaderArm(cfg)
        self.assertEqual(leader.config.joint_offsets_deg.get("wrist_roll"), 90.0)


if __name__ == "__main__":
    unittest.main()

