"""SO-ARM101 系统运行状态机与控制权仲裁器.

职责:
1. 维护机器人运行生命周期 (DISCONNECTED, IDLE, TELEOP, RECORDING, PLAYBACK, ESTOP_LATCHED, FAULT 等);
2. 动作控制权仲裁: 同一时刻仅允许单一控制源 (KEYBOARD, LEADER, REPLAY, POLICY) 拥有写控制权;
3. 严格的白名单状态转移规则, 杜绝非法跳变;
4. 瞬时急停闭锁 (Estop Latching) 与显式人工复位流程 (禁止自动续跑历史指令);
5. 状态变更事件回调通知 (发布/订阅模式).
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import Any

from core.data_types import ControlSource, SystemState

logger = logging.getLogger(__name__)

# 状态转移白名单: {当前状态: 允许转移的目标状态集合}
VALID_TRANSITIONS: dict[SystemState, set[SystemState]] = {
    SystemState.DISCONNECTED: {
        SystemState.IDLE,
        SystemState.FAULT,
    },
    SystemState.IDLE: {
        SystemState.DISCONNECTED,
        SystemState.TELEOP,
        SystemState.RECORDING,
        SystemState.PLAYBACK_READY,
        SystemState.PLAYBACK,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.TELEOP: {
        SystemState.IDLE,
        SystemState.RECORDING,
        SystemState.PLAYBACK_READY,
        SystemState.PLAYBACK,
        SystemState.STOPPING,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.RECORDING: {
        SystemState.RECORD_PAUSED,
        SystemState.IDLE,
        SystemState.STOPPING,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.RECORD_PAUSED: {
        SystemState.RECORDING,
        SystemState.IDLE,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.PLAYBACK_READY: {
        SystemState.PLAYBACK,
        SystemState.IDLE,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.PLAYBACK: {
        SystemState.PLAYBACK_PAUSED,
        SystemState.STOPPING,
        SystemState.IDLE,
        SystemState.TELEOP,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.PLAYBACK_PAUSED: {
        SystemState.PLAYBACK,
        SystemState.STOPPING,
        SystemState.IDLE,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.STOPPING: {
        SystemState.IDLE,
        SystemState.ESTOP_LATCHED,
        SystemState.FAULT,
    },
    SystemState.ESTOP_LATCHED: {
        SystemState.IDLE,  # 仅能通过显式人工复位回到 IDLE
        SystemState.DISCONNECTED,
        SystemState.FAULT,
    },
    SystemState.FAULT: {
        SystemState.IDLE,  # 仅能通过排查故障后人工复位
        SystemState.DISCONNECTED,
        SystemState.ESTOP_LATCHED,
    },
}

StateCallback = Callable[[SystemState, SystemState, str], None]


class RobotStateMachine:
    """机器人系统核心运行状态机."""

    def __init__(self, initial_state: SystemState = SystemState.DISCONNECTED) -> None:
        self._state = initial_state
        self._active_source: ControlSource | None = None
        self._state_enter_time = time.monotonic()
        self._estop_reason: str | None = None
        self._fault_reason: str | None = None
        self._callbacks: list[StateCallback] = []

    @property
    def current_state(self) -> SystemState:
        """当前系统状态."""
        return self._state

    @property
    def active_source(self) -> ControlSource | None:
        """当前拥有动作控制权的输入源."""
        return self._active_source

    @property
    def is_estop(self) -> bool:
        """是否处于急停闭锁."""
        return self._state == SystemState.ESTOP_LATCHED

    @property
    def estop_reason(self) -> str | None:
        """急停原因."""
        return self._estop_reason

    @property
    def fault_reason(self) -> str | None:
        """故障报警原因."""
        return self._fault_reason

    def register_callback(self, callback: StateCallback) -> None:
        """注册状态流转监听器 (old_state, new_state, reason)."""
        if callback not in self._callbacks:
            self._callbacks.append(callback)

    def transition_to(self, new_state: SystemState, reason: str = "") -> bool:
        """请求状态流转 (受白名单严格约束)."""
        if new_state == self._state:
            return True

        allowed = VALID_TRANSITIONS.get(self._state, set())
        if new_state not in allowed:
            logger.error(
                "非法状态转移被拒绝: %s -> %s (原因: %s)",
                self._state.value,
                new_state.value,
                reason,
            )
            return False

        old_state = self._state
        self._state = new_state
        self._state_enter_time = time.monotonic()

        # 状态切换时的控制权自动调整
        if new_state in (SystemState.IDLE, SystemState.DISCONNECTED, SystemState.STOPPING):
            self._active_source = None
        elif new_state in (SystemState.ESTOP_LATCHED, SystemState.FAULT):
            self._active_source = None

        logger.info(
            "状态机流转成功: [%s] -> [%s] (原因: %s, 授权来源: %s)",
            old_state.value,
            new_state.value,
            reason or "无",
            self._active_source.value if self._active_source else "无",
        )

        for cb in self._callbacks:
            try:
                cb(old_state, new_state, reason)
            except Exception as exc:
                logger.error("状态机监听回调执行异常: %s", exc)

        return True

    def claim_control(self, source: ControlSource) -> bool:
        """申请独占动作控制权."""
        if self.is_estop or self._state == SystemState.FAULT:
            logger.warning("处于急停或故障锁定状态，拒绝授权控制权给: %s", source.value)
            return False

        # 如果当前在 IDLE 状态，申请进入 TELEOP 或 PLAYBACK
        if self._state == SystemState.IDLE:
            target_state = SystemState.PLAYBACK if source == ControlSource.REPLAY else SystemState.TELEOP
            if self.transition_to(target_state, f"控制权授权给 {source.value}"):
                self._active_source = source
                return True
            return False

        # 如果已经处于 TELEOP 状态
        if self._state == SystemState.TELEOP:
            if self._active_source == source:
                return True
            logger.warning(
                "控制权冲突: 当前已被 %s 占用，拒绝 %s 的抢占请求",
                self._active_source.value if self._active_source else "未知",
                source.value,
            )
            return False

        # 录制状态下允许保持或切换当前来源
        if self._state in (SystemState.RECORDING, SystemState.RECORD_PAUSED):
            self._active_source = source
            return True

        # 回放状态下仅允许 REPLAY
        if self._state == SystemState.PLAYBACK:
            if source == ControlSource.REPLAY:
                self._active_source = source
                return True
            logger.warning("正在回放中，拒绝外部输入源 %s 抢占", source.value)
            return False

        logger.warning("当前状态 %s 不允许申请控制权", self._state.value)
        return False

    def release_control(self, source: ControlSource) -> bool:
        """释放控制权，若当前由该来源持有则退回 IDLE."""
        if self._active_source == source:
            self._active_source = None
            if self._state in (SystemState.TELEOP, SystemState.PLAYBACK, SystemState.PLAYBACK_PAUSED):
                return self.transition_to(SystemState.IDLE, f"来源 {source.value} 释放控制权")
            return True
        return False

    def trigger_estop(self, reason: str = "用户触发急停") -> None:
        """触发软件急停锁定 (最高优先级，瞬间执行)."""
        logger.critical("!!! 触发急停锁定 !!! 原因: %s", reason)
        self._estop_reason = reason
        self._active_source = None

        old_state = self._state
        self._state = SystemState.ESTOP_LATCHED
        self._state_enter_time = time.monotonic()

        for cb in self._callbacks:
            try:
                cb(old_state, SystemState.ESTOP_LATCHED, reason)
            except Exception as exc:
                logger.error("急停回调执行异常: %s", exc)

    def trigger_fault(self, reason: str) -> None:
        """报告系统故障，锁定状态机."""
        logger.error("系统故障报警: %s", reason)
        self._fault_reason = reason
        self._active_source = None

        old_state = self._state
        self._state = SystemState.FAULT
        self._state_enter_time = time.monotonic()

        for cb in self._callbacks:
            try:
                cb(old_state, SystemState.FAULT, reason)
            except Exception as exc:
                logger.error("故障回调执行异常: %s", exc)

    def request_reset(self) -> tuple[bool, str]:
        """请求解除急停或故障，执行显式人工复位.
        
        Returns:
            (是否复位成功, 说明文本)
        """
        if self._state not in (SystemState.ESTOP_LATCHED, SystemState.FAULT):
            return False, f"当前状态 {self._state.value} 无需复位"

        old_state = self._state
        self._estop_reason = None
        self._fault_reason = None
        self._active_source = None
        self._state = SystemState.IDLE
        self._state_enter_time = time.monotonic()

        logger.info("系统已完成显式人工复位，回到 [%s] 待机就绪状态", self._state.value)

        for cb in self._callbacks:
            try:
                cb(old_state, SystemState.IDLE, "人工确认复位完成")
            except Exception as exc:
                logger.error("复位回调执行异常: %s", exc)

        return True, "复位成功，系统已就绪待机 (所有历史积压指令已清空)"

