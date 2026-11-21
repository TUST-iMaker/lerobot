"""Unit tests for core/state_machine.py."""

from __future__ import annotations

import unittest

from core.data_types import ControlSource, SystemState
from core.state_machine import RobotStateMachine


class TestRobotStateMachine(unittest.TestCase):
    def setUp(self) -> None:
        self.sm = RobotStateMachine(initial_state=SystemState.DISCONNECTED)

    def test_initial_state(self) -> None:
        self.assertEqual(self.sm.current_state, SystemState.DISCONNECTED)
        self.assertIsNone(self.sm.active_source)
        self.assertFalse(self.sm.is_estop)

    def test_valid_transitions(self) -> None:
        self.assertTrue(self.sm.transition_to(SystemState.IDLE, "连接成功"))
        self.assertEqual(self.sm.current_state, SystemState.IDLE)

        self.assertTrue(self.sm.transition_to(SystemState.TELEOP, "进入手动遥控"))
        self.assertEqual(self.sm.current_state, SystemState.TELEOP)

        self.assertTrue(self.sm.transition_to(SystemState.RECORDING, "开始示教录制"))
        self.assertEqual(self.sm.current_state, SystemState.RECORDING)

        self.assertTrue(self.sm.transition_to(SystemState.IDLE, "示教结束"))
        self.assertEqual(self.sm.current_state, SystemState.IDLE)

    def test_invalid_transitions_rejected(self) -> None:
        # 从未连接直接跳到 PLAYBACK 应该被拒绝
        self.assertFalse(self.sm.transition_to(SystemState.PLAYBACK, "非法跳转"))
        self.assertEqual(self.sm.current_state, SystemState.DISCONNECTED)

    def test_control_claiming_and_mutual_exclusion(self) -> None:
        self.sm.transition_to(SystemState.IDLE)

        # 键盘申请控制权
        ok = self.sm.claim_control(ControlSource.KEYBOARD)
        self.assertTrue(ok)
        self.assertEqual(self.sm.current_state, SystemState.TELEOP)
        self.assertEqual(self.sm.active_source, ControlSource.KEYBOARD)

        # 主臂在键盘使用时尝试抢占 -> 拒绝
        ok_leader = self.sm.claim_control(ControlSource.LEADER)
        self.assertFalse(ok_leader)
        self.assertEqual(self.sm.active_source, ControlSource.KEYBOARD)

        # 键盘释放控制权
        self.assertTrue(self.sm.release_control(ControlSource.KEYBOARD))
        self.assertEqual(self.sm.current_state, SystemState.IDLE)
        self.assertIsNone(self.sm.active_source)

        # 主臂再次申请 -> 成功
        self.assertTrue(self.sm.claim_control(ControlSource.LEADER))
        self.assertEqual(self.sm.active_source, ControlSource.LEADER)

    def test_estop_latching_and_reset(self) -> None:
        self.sm.transition_to(SystemState.IDLE)
        self.sm.claim_control(ControlSource.KEYBOARD)

        # 拍下急停
        self.sm.trigger_estop("拍下紧急停止按钮")
        self.assertEqual(self.sm.current_state, SystemState.ESTOP_LATCHED)
        self.assertTrue(self.sm.is_estop)
        self.assertIsNone(self.sm.active_source)

        # 急停时申请控制权必须失败
        self.assertFalse(self.sm.claim_control(ControlSource.KEYBOARD))

        # 急停时不能非法转入其他运动状态
        self.assertFalse(self.sm.transition_to(SystemState.TELEOP))

        # 人工显式复位
        success, msg = self.sm.request_reset()
        self.assertTrue(success)
        self.assertEqual(self.sm.current_state, SystemState.IDLE)
        self.assertFalse(self.sm.is_estop)

    def test_callback_invocations(self) -> None:
        history = []

        def on_transition(old, new, reason):
            history.append((old, new, reason))

        self.sm.register_callback(on_transition)
        self.sm.transition_to(SystemState.IDLE, "test_reason")
        self.assertEqual(len(history), 1)
        self.assertEqual(history[0], (SystemState.DISCONNECTED, SystemState.IDLE, "test_reason"))


if __name__ == "__main__":
    unittest.main()

