"""Unit tests for core/safety_controller.py."""

from __future__ import annotations

import unittest

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


class TestSafetyController(unittest.TestCase):
    def setUp(self) -> None:
        self.sm = RobotStateMachine(initial_state=SystemState.IDLE)
        self.config = SafetyLimitsConfig(
            max_joint_delta_rad=0.08,
            max_tracking_error_rad=0.30,
            max_consecutive_tracking_errors=3,
            limit_action_mode="clamp",
        )
        self.controller = SafetyController(self.sm, self.config)

        # 构造正常位姿的当前状态
        self.cur_state = JointState(
            timestamp=100.0,
            positions={j: 0.0 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )

    def test_state_guard_blocking(self) -> None:
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=100.0,
            target_joint_positions={j: 0.01 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        # 1. 处于 IDLE 状态时下发动作 -> 被拒绝
        ok, safe_act, msg = self.controller.filter_and_verify(action, self.cur_state)
        self.assertFalse(ok)
        self.assertIn("不允许执行运动", msg)

        # 2. 授权给 KEYBOARD 进入 TELEOP
        self.sm.claim_control(ControlSource.KEYBOARD)
        ok, safe_act, msg = self.controller.filter_and_verify(action, self.cur_state)
        self.assertTrue(ok)
        self.assertIsNotNone(safe_act)

        # 3. 拍下急停 -> 被拦截
        self.sm.trigger_estop("急停测试")
        ok, safe_act, msg = self.controller.filter_and_verify(action, self.cur_state)
        self.assertFalse(ok)
        self.assertIn("急停锁定", msg)

    def test_source_mismatch_blocking(self) -> None:
        self.sm.claim_control(ControlSource.KEYBOARD)

        # 伪造一个来自 LEADER 的动作
        fake_action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.LEADER,
            timestamp=100.0,
            target_joint_positions={j: 0.01 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        ok, _, msg = self.controller.filter_and_verify(fake_action, self.cur_state)
        self.assertFalse(ok)
        self.assertIn("控制源不匹配", msg)

    def test_soft_limit_clamping(self) -> None:
        self.sm.claim_control(ControlSource.KEYBOARD)

        # shoulder_pan 限制为 [-1.8, 1.8] rad
        # 下发超出限位的 3.0 rad
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=100.0,
            target_joint_positions={"shoulder_pan": 3.0},
            unit=JointUnit.RADIAN,
        )

        # 设置当前位置就在 1.7 rad (只看限位)
        cur = JointState(
            timestamp=100.0,
            positions={"shoulder_pan": 1.7},
            unit=JointUnit.RADIAN,
        )
        ok, safe_act, msg = self.controller.filter_and_verify(action, cur)
        self.assertTrue(ok)
        self.assertIsNotNone(safe_act)
        # clamp 到了边界 1.8，同时受到单步增量 0.08 限制 -> 1.7 + 0.08 = 1.78
        self.assertLessEqual(safe_act.target_joint_positions["shoulder_pan"], 1.8)

    def test_delta_rate_limiting(self) -> None:
        self.sm.claim_control(ControlSource.KEYBOARD)

        # 当前在 0.0，尝试一步跳到 0.5 rad (突变冲击)
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=100.0,
            target_joint_positions={"elbow_flex": 0.5},
            unit=JointUnit.RADIAN,
        )
        ok, safe_act, msg = self.controller.filter_and_verify(action, self.cur_state)
        self.assertTrue(ok)
        # 被限制为 0.08 rad
        self.assertAlmostEqual(safe_act.target_joint_positions["elbow_flex"], 0.08, places=4)

    def test_tracking_error_fault_trigger(self) -> None:
        self.sm.claim_control(ControlSource.KEYBOARD)

        command = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=100.0,
            target_joint_positions={"shoulder_pan": 0.0},
            unit=JointUnit.RADIAN,
        )
        # 实际位置严重滞后/卡死在 0.8 rad (> 0.3 rad 阈值)
        stuck_state = JointState(
            timestamp=100.0,
            positions={"shoulder_pan": 0.8},
            unit=JointUnit.RADIAN,
        )

        # 模拟连续 3 帧超差
        for _ in range(self.config.max_consecutive_tracking_errors - 1):
            ok, _ = self.controller.check_tracking_error(command, stuck_state)
            self.assertTrue(ok)
            self.assertEqual(self.sm.current_state, SystemState.TELEOP)

        # 第 3 帧应触发 FAULT
        ok, fault_msg = self.controller.check_tracking_error(command, stuck_state)
        self.assertFalse(ok)
        self.assertEqual(self.sm.current_state, SystemState.FAULT)
        self.assertIn("卡死", str(self.sm.fault_reason))

    def test_dynamic_max_joint_delta_from_config(self) -> None:
        """测试默认 SafetyLimitsConfig 会联动 config.yaml 中的 max_step_deg."""
        from core.config_loader import get_config
        from core.data_types import deg_to_rad
        cfg = get_config()
        expected_rad = deg_to_rad(cfg.safety.max_step_deg)
        default_limits = SafetyLimitsConfig()
        self.assertAlmostEqual(default_limits.max_joint_delta_rad, expected_rad, places=4)

    def test_rest_pose_within_safety_limits(self) -> None:
        """测试 rest_pose.json 各关节目标角度都在默认软限位范围内."""
        from core.presets import get_preset_pose
        rest_pose = get_preset_pose("rest", unit=JointUnit.RADIAN)
        for joint, (min_lim, max_lim) in self.controller.config.joint_limits_rad.items():
            if joint in rest_pose:
                val = rest_pose[joint]
                self.assertGreaterEqual(val, min_lim, f"关节 {joint} 的蜷缩位角度 {val} 低于软限位下限 {min_lim}")
                self.assertLessEqual(val, max_lim, f"关节 {joint} 的蜷缩位角度 {val} 高于软限位上限 {max_lim}")


if __name__ == "__main__":
    unittest.main()

