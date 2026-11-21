"""Unit tests for core/data_types.py."""

from __future__ import annotations

import math
import unittest

from core.data_types import (
    SO101_JOINT_NAMES,
    ActionType,
    CameraFrameRef,
    ControlSource,
    JointState,
    JointUnit,
    Pose3D,
    RobotAction,
    RobotObservation,
    SafetyReport,
    SystemState,
)


class TestJointState(unittest.TestCase):
    def test_valid_joint_state(self) -> None:
        positions = {j: 0.1 for j in SO101_JOINT_NAMES}
        state = JointState(
            timestamp=100.5,
            positions=positions,
            unit=JointUnit.RADIAN,
        )
        is_valid, err = state.validate()
        self.assertTrue(is_valid)
        self.assertIsNone(err)

    def test_missing_joint(self) -> None:
        positions = {"shoulder_pan": 0.0, "shoulder_lift": 0.0}
        state = JointState(
            timestamp=100.5,
            positions=positions,
            unit=JointUnit.RADIAN,
        )
        is_valid, err = state.validate(require_all_joints=True)
        self.assertFalse(is_valid)
        self.assertIn("缺少关键关节", str(err))

    def test_nan_joint(self) -> None:
        positions = {j: 0.1 for j in SO101_JOINT_NAMES}
        positions["elbow_flex"] = float("nan")
        state = JointState(
            timestamp=100.5,
            positions=positions,
            unit=JointUnit.RADIAN,
        )
        is_valid, err = state.validate()
        self.assertFalse(is_valid)
        self.assertIn("非有限数值", str(err))

    def test_unit_conversion(self) -> None:
        positions_deg = {j: 90.0 for j in SO101_JOINT_NAMES}
        state_deg = JointState(timestamp=1.0, positions=positions_deg, unit=JointUnit.DEGREE)
        state_rad = state_deg.to_radians()
        self.assertEqual(state_rad.unit, JointUnit.RADIAN)
        for val in state_rad.positions.values():
            self.assertAlmostEqual(val, math.pi / 2, places=5)

        state_back = state_rad.to_degrees()
        self.assertEqual(state_back.unit, JointUnit.DEGREE)
        for val in state_back.positions.values():
            self.assertAlmostEqual(val, 90.0, places=5)

    def test_serialization_roundtrip(self) -> None:
        positions = {j: 0.25 for j in SO101_JOINT_NAMES}
        state = JointState(
            timestamp=12.34,
            positions=positions,
            unit=JointUnit.RADIAN,
            torques_enabled={j: True for j in SO101_JOINT_NAMES},
        )
        d = state.to_dict()
        restored = JointState.from_dict(d)
        self.assertEqual(state.timestamp, restored.timestamp)
        self.assertEqual(state.unit, restored.unit)
        self.assertEqual(state.positions, restored.positions)
        self.assertEqual(state.torques_enabled, restored.torques_enabled)


class TestRobotAction(unittest.TestCase):
    def test_joint_position_action(self) -> None:
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=10.0,
            target_joint_positions={j: 0.0 for j in SO101_JOINT_NAMES},
        )
        is_valid, err = action.validate()
        self.assertTrue(is_valid)
        self.assertIsNone(err)

    def test_invalid_action(self) -> None:
        # 声明为 JOINT_POSITION 但没有传 target_joint_positions
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=10.0,
        )
        is_valid, err = action.validate()
        self.assertFalse(is_valid)

    def test_cartesian_delta_action(self) -> None:
        action = RobotAction(
            action_type=ActionType.CARTESIAN_DELTA,
            source=ControlSource.KEYBOARD,
            timestamp=10.0,
            target_cartesian_delta=(0.01, 0.0, -0.01),
        )
        is_valid, err = action.validate()
        self.assertTrue(is_valid)

        # 序列化恢复
        restored = RobotAction.from_dict(action.to_dict())
        self.assertEqual(restored.action_type, ActionType.CARTESIAN_DELTA)
        self.assertEqual(restored.target_cartesian_delta, (0.01, 0.0, -0.01))


class TestRobotObservation(unittest.TestCase):
    def test_observation_roundtrip(self) -> None:
        joint_state = JointState(
            timestamp=100.0,
            positions={j: 0.0 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        tcp_state = Pose3D(
            timestamp=100.0,
            position_m=(0.2, 0.0, 0.15),
            orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            valid=True,
            kinematics_version="v1.0",
        )
        cam = CameraFrameRef(
            camera_id="wrist",
            frame_id=42,
            timestamp=99.98,
            width=640,
            height=480,
            relative_path="images/wrist/000042.jpg",
        )
        obs = RobotObservation(
            timestamp=100.0,
            joint_state=joint_state,
            tcp_state=tcp_state,
            camera_frames={"wrist": cam},
        )
        d = obs.to_dict()
        restored = RobotObservation.from_dict(d)
        self.assertEqual(restored.timestamp, 100.0)
        self.assertEqual(restored.tcp_state.position_m, (0.2, 0.0, 0.15))
        self.assertEqual(restored.camera_frames["wrist"].frame_id, 42)


class TestSafetyReport(unittest.TestCase):
    def test_safety_check(self) -> None:
        safe_report = SafetyReport(
            timestamp=1.0,
            system_state=SystemState.TELEOP,
            estop_active=False,
            limit_triggered=False,
            active_source=ControlSource.KEYBOARD,
        )
        self.assertTrue(safe_report.is_safe_for_motion())

        estop_report = SafetyReport(
            timestamp=1.0,
            system_state=SystemState.ESTOP_LATCHED,
            estop_active=True,
        )
        self.assertFalse(estop_report.is_safe_for_motion())


if __name__ == "__main__":
    unittest.main()

