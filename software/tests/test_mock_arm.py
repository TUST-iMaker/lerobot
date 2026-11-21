"""Unit tests for MockArmAdapter and mock RobotSystemAdapter."""

from __future__ import annotations

import time
import unittest

from core.data_types import (
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
)
from hardware.config import ArmConfig, ArmRole, RobotSystemConfig
from hardware.mock_arm import DEFAULT_MOCK_WORKING_POSE, MockArmAdapter
from hardware.system_adapter import RobotSystemAdapter


class TestMockArmAdapter(unittest.TestCase):
    """Test suite for MockArmAdapter."""

    def setUp(self) -> None:
        self.config = ArmConfig.mock_follower()
        self.adapter = MockArmAdapter(self.config, max_joint_speed_rad_s=5.0)

    def tearDown(self) -> None:
        if self.adapter.is_connected:
            self.adapter.disconnect()

    def test_connect_and_disconnect(self) -> None:
        self.assertFalse(self.adapter.is_connected)
        self.adapter.connect()
        self.assertTrue(self.adapter.is_connected)
        self.assertTrue(self.adapter.is_calibrated)
        self.assertEqual(len(self.adapter.calibration_ranges), 6)

        torques = self.adapter.read_torque_states()
        self.assertTrue(all(torques.values()))

        # Disconnect with torque release
        self.adapter.disconnect(keep_torque=False)
        self.assertFalse(self.adapter.is_connected)
        torques_off = self.adapter.read_torque_states()
        self.assertFalse(any(torques_off.values()))

    def test_read_state_and_dynamics(self) -> None:
        self.adapter.connect()
        state = self.adapter.read_state()
        self.assertEqual(state.unit, JointUnit.RADIAN)
        for j in SO101_JOINT_NAMES:
            self.assertIn(j, state.positions)
            self.assertAlmostEqual(state.positions[j], DEFAULT_MOCK_WORKING_POSE[j], places=4)

        # Send action to target position 0.5 rad
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=time.monotonic(),
            target_joint_positions={"shoulder_pan": 0.5},
            unit=JointUnit.RADIAN,
        )
        ok = self.adapter.send_action(action)
        self.assertTrue(ok)

        # Advance time by stepping read_state
        time.sleep(0.05)
        new_state = self.adapter.read_state()
        # Should be moving towards 0.5
        self.assertGreater(new_state.positions["shoulder_pan"], 0.0)

    def test_stop_motion(self) -> None:
        self.adapter.connect()
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=time.monotonic(),
            target_joint_positions={"shoulder_lift": 1.0},
            unit=JointUnit.RADIAN,
        )
        self.adapter.send_action(action)
        time.sleep(0.02)
        self.adapter.read_state()

        # Trigger emergency stop / hold position
        self.adapter.stop_motion()
        pos_when_stopped = self.adapter.read_positions()["shoulder_lift"]

        # Further steps should stay at stopped position
        time.sleep(0.05)
        pos_after_steps = self.adapter.read_positions()["shoulder_lift"]
        self.assertAlmostEqual(pos_when_stopped, pos_after_steps, places=4)

    def test_move_to_middle(self) -> None:
        self.adapter.connect()
        self.adapter.set_joint_positions({"shoulder_pan": 0.8, "elbow_flex": -1.2})
        pos = self.adapter.read_positions()
        self.assertAlmostEqual(pos["shoulder_pan"], 0.8)

        ok = self.adapter.move_to_middle()
        self.assertTrue(ok)
        mid_pos = self.adapter.read_positions()
        for j in SO101_JOINT_NAMES:
            self.assertAlmostEqual(mid_pos[j], DEFAULT_MOCK_WORKING_POSE[j], places=4)

    def test_fault_injection(self) -> None:
        self.adapter.inject_fault("connect_error")
        with self.assertRaises(RuntimeError):
            self.adapter.connect()

        self.adapter.clear_fault()
        self.adapter.connect()

        self.adapter.inject_fault("read_error")
        with self.assertRaises(TimeoutError):
            self.adapter.read_state()

        self.adapter.clear_fault()
        state = self.adapter.read_state()
        self.assertIsNotNone(state)

    def test_system_adapter_with_mock(self) -> None:
        sys_cfg = RobotSystemConfig.mock_system(with_leader=True)
        sys_adapter = RobotSystemAdapter(sys_cfg)
        self.assertTrue(isinstance(sys_adapter.follower, MockArmAdapter))
        self.assertTrue(isinstance(sys_adapter.leader, MockArmAdapter))

        sys_adapter.connect()
        self.assertTrue(sys_adapter.is_follower_connected)
        self.assertTrue(sys_adapter.is_leader_connected)

        f_state = sys_adapter.read_follower_state()
        l_state = sys_adapter.read_leader_state()
        self.assertIsNotNone(f_state)
        self.assertIsNotNone(l_state)

        sys_adapter.disconnect()
        self.assertFalse(sys_adapter.is_follower_connected)


if __name__ == "__main__":
    unittest.main()
