"""Unit tests for core/action_api.py."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock

from core.action_api import ActionAPI
from core.data_types import (
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    SystemState,
)
from core.safety_controller import SafetyController, SafetyLimitsConfig
from core.state_machine import RobotStateMachine
from hardware.config import ArmConfig, RobotSystemConfig
from hardware.system_adapter import RobotSystemAdapter


class TestActionAPI(unittest.TestCase):
    def setUp(self) -> None:
        # 构造 Mock 硬件系统
        self.mock_hw = MagicMock(spec=RobotSystemAdapter)
        self.mock_hw.is_follower_connected = True
        self.mock_hw.is_leader_connected = False
        self.mock_hw.leader = None

        # 模拟当前关节状态
        self.sample_state = JointState(
            timestamp=10.0,
            positions={j: 0.0 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        self.mock_hw.read_follower_state.return_value = self.sample_state
        self.mock_hw.send_action.return_value = True

        self.sm = RobotStateMachine(initial_state=SystemState.IDLE)
        self.safety = SafetyController(self.sm)

        self.api = ActionAPI(
            hardware=self.mock_hw,
            state_machine=self.sm,
            safety_controller=self.safety,
        )

    def test_initialization(self) -> None:
        report = self.api.get_status_report()
        self.assertEqual(report["system_state"], "IDLE")
        self.assertFalse(report["is_estop"])

    def test_claim_and_release_control(self) -> None:
        self.assertTrue(self.api.claim_control(ControlSource.KEYBOARD))
        self.assertEqual(self.sm.current_state, SystemState.TELEOP)
        self.assertEqual(self.sm.active_source, ControlSource.KEYBOARD)

        self.assertTrue(self.api.release_control(ControlSource.KEYBOARD))
        self.assertEqual(self.sm.current_state, SystemState.IDLE)
        self.assertIsNone(self.sm.active_source)

    def test_submit_action_pipeline(self) -> None:
        # 1. 未授权控制权直接下发动作 -> 拦截
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=10.0,
            target_joint_positions={j: 0.01 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        ok, msg = self.api.submit_action(action)
        self.assertFalse(ok)
        self.assertIn("安全拦截", msg)
        self.mock_hw.send_action.assert_not_called()

        # 2. 正常申请控制权后下发 -> 成功执行
        self.api.claim_control(ControlSource.KEYBOARD)
        ok, msg = self.api.submit_action(action)
        self.assertTrue(ok)
        self.assertEqual(msg, "动作执行成功")
        self.mock_hw.send_action.assert_called_once()

    def test_estop_and_hardware_braking_linkage(self) -> None:
        self.api.claim_control(ControlSource.KEYBOARD)

        # 触发急停
        self.api.request_estop("急停按钮测试")

        # 1. 验证状态机转入急停
        self.assertEqual(self.sm.current_state, SystemState.ESTOP_LATCHED)
        self.assertTrue(self.sm.is_estop)

        # 2. 验证硬件联动原位制动 stop_motion 被调用
        self.mock_hw.stop_motion.assert_called()

        # 3. 验证急停状态下动作被绝对拒绝
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=10.0,
            target_joint_positions={j: 0.01 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        ok, msg = self.api.submit_action(action)
        self.assertFalse(ok)
        self.assertIn("急停锁定", msg)

        # 4. 显式人工复位
        success, reset_msg = self.api.request_reset()
        self.assertTrue(success)
        self.assertEqual(self.sm.current_state, SystemState.IDLE)

    def test_safe_return_to_rest_execution(self) -> None:
        self.mock_hw.disable_torque.return_value = True
        ok = self.api.safe_return_to_rest(duration_s=0.1, release_torque=True)
        self.assertTrue(ok)
        self.mock_hw.send_action.assert_called()
        self.mock_hw.disable_torque.assert_called()

    def test_safe_return_to_rest_already_at_rest_releases_torque(self) -> None:
        from core.presets import get_preset_pose
        rest_target = get_preset_pose("rest", unit=JointUnit.RADIAN)
        self.mock_hw.read_follower_state.return_value = JointState(
            timestamp=20.0,
            positions=rest_target,
            unit=JointUnit.RADIAN,
        )
        self.mock_hw.disable_torque.return_value = True
        ok = self.api.safe_return_to_rest(duration_s=0.1, release_torque=True)
        self.assertTrue(ok)
        self.mock_hw.disable_torque.assert_called()


if __name__ == "__main__":
    unittest.main()

