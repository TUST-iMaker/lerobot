"""机械臂整机系统统一适配器 (门面模式).

统一管理 Follower 从臂与可选的 Leader 主臂，提供一站式状态采集、动作执行、主从遥操作同步与整机健康诊断.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

from core.data_types import (
    ActionType,
    ControlSource,
    JointState,
    RobotAction,
)
from hardware.config import RobotSystemConfig
from hardware.follower_arm import SO101FollowerArm
from hardware.leader_arm import SO101LeaderArm
from hardware.mock_arm import MockArmAdapter

logger = logging.getLogger(__name__)


class RobotSystemAdapter:
    """机械臂整机系统硬件总适配器 (支持真机与仿真透明切换)."""

    def __init__(self, config: RobotSystemConfig) -> None:
        self.config = config
        self._bus_lock = threading.RLock()
        self._is_mock_follower = config.use_mock or config.follower.is_mock or config.follower.port == "mock"
        self._is_mock_leader = (
            config.leader is not None
            and (config.use_mock or config.leader.is_mock or config.leader.port == "mock")
        )

        if self._is_mock_follower:
            logger.info("初始化仿真 Follower 从臂适配器 (MockArmAdapter)")
            self.follower = MockArmAdapter(config.follower)
        else:
            self.follower = SO101FollowerArm(config.follower)

        if config.leader:
            if self._is_mock_leader:
                logger.info("初始化仿真 Leader 主臂适配器 (MockArmAdapter)")
                self.leader = MockArmAdapter(config.leader)
            else:
                self.leader = SO101LeaderArm(config.leader)
        else:
            self.leader = None

    def __enter__(self) -> RobotSystemAdapter:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: Any,
    ) -> None:
        self.disconnect(keep_torque=False)

    @property
    def is_follower_connected(self) -> bool:
        return self.follower.is_connected

    @property
    def is_leader_connected(self) -> bool:
        return self.leader.is_connected if self.leader else False

    def connect(self, connect_leader: bool = True) -> None:
        """连接机械臂硬件. 优先连接从臂，可选连接主臂."""
        logger.info("=== 启动机械臂硬件系统连接 ===")
        self.follower.connect()

        if self.leader and connect_leader:
            try:
                self.leader.connect()
            except Exception as exc:
                logger.warning("主臂连接失败 (可降级为仅键盘控制): %s", exc)

    def disconnect(self, keep_torque: bool = False) -> None:
        """安全断开所有机械臂."""
        logger.info("=== 正在关闭机械臂硬件系统 ===")
        if self.leader and self.leader.is_connected:
            self.leader.disconnect()
        if self.follower.is_connected:
            self.follower.disconnect(keep_torque=keep_torque)

    def disable_torque(self) -> bool:
        """主动释放从臂舵机扭矩 (进入松弛休眠状态)."""
        with self._bus_lock:
            if hasattr(self.follower, "disable_torque"):
                return self.follower.disable_torque()
            return False

    def enable_torque(self) -> bool:
        """主动恢复从臂舵机扭矩."""
        with self._bus_lock:
            if hasattr(self.follower, "enable_torque"):
                return self.follower.enable_torque()
            return False

    def read_follower_state(self) -> JointState:
        """读取执行从臂当前状态."""
        with self._bus_lock:
            return self.follower.read_state()

    def read_leader_state(self) -> JointState | None:
        """读取示教主臂当前状态 (若主臂未连接返回 None)."""
        if not self.leader or not self.leader.is_connected:
            return None
        with self._bus_lock:
            return self.leader.read_state()

    def send_action(self, action: RobotAction) -> bool:
        """向从臂下发动作指令."""
        with self._bus_lock:
            return self.follower.send_action(action)

    def stop_motion(self) -> None:
        """急停/原位保持: 锁定从臂当前位置并保持力矩."""
        self.follower.stop_motion()

    def step_teleop(self) -> tuple[JointState | None, RobotAction | None, bool]:
        """单步执行主从遥操作: 读取主臂位姿 -> 生成指令 -> 下发给从臂.
        
        Returns:
            (leader_state, action_sent, success_bool)
        """
        if not self.leader or not self.leader.is_connected:
            logger.warning("主臂未连接，无法执行主从遥操作")
            return None, None, False

        leader_state = self.leader.read_state()
        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.LEADER,
            timestamp=leader_state.timestamp,
            target_joint_positions=dict(leader_state.positions),
            unit=leader_state.unit,
        )
        ok = self.follower.send_action(action)
        return leader_state, action, ok

    def get_health_report(self) -> dict[str, Any]:
        """汇总系统整机健康遥测状态."""
        report: dict[str, Any] = {
            "follower": {
                "connected": self.follower.is_connected,
                "calibrated": self.follower.is_calibrated,
                "health": self.follower.health.__dict__,
                "diagnostics": self.follower.read_diagnostics(),
            }
        }
        if self.leader:
            report["leader"] = {
                "connected": self.leader.is_connected,
                "calibrated": self.leader.is_calibrated,
                "health": self.leader.health.__dict__,
                "diagnostics": self.leader.read_diagnostics(),
            }
        return report


def action_type_position():
    from core.data_types import ActionType
    return ActionType.JOINT_POSITION

