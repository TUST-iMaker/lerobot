"""SO-ARM101 虚拟机械臂适配器 (MockArmAdapter).

提供全纯软件轻量级仿真机械臂，完全履行 BaseArmAdapter 接口契约：
1. 内存中精确维护 6 个舵机关节状态，无须任何物理硬件、USB 串口或 LeRobot 底层依赖；
2. 内置一阶平滑动力学与舵机角速度限幅仿真 (max_joint_speed_rad_s)，杜绝位置瞬移，真实模拟电机运动过程；
3. 支持完整的生命周期：上电防跳变、力矩使能/释放、安全顺序平滑回中位、急停原位锁止；
4. 支持故障注入测试 (断开、超时、卡阻、动作拒绝)，为系统安全控制器与上层业务提供完备的测试替身.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from core.data_types import (
    MIDDLE_ORDER,
    SO101_ARM_JOINT_NAMES,
    SO101_GRIPPER_JOINT_NAME,
    SO101_JOINT_NAMES,
    ActionType,
    JointState,
    JointUnit,
    RobotAction,
    clamp_gripper_pct,
    convert_joint_positions,
    deg_to_rad,
    rad_to_deg,
)
from hardware.base import BaseArmAdapter
from hardware.config import ArmConfig, ArmRole

logger = logging.getLogger(__name__)

# 默认仿真物理 raw 校准范围 (每个舵机 0 ~ 4095)
DEFAULT_MOCK_CALIBRATION_RANGES = {
    "shoulder_pan": (0, 4095),
    "shoulder_lift": (0, 4095),
    "elbow_flex": (0, 4095),
    "wrist_flex": (0, 4095),
    "wrist_roll": (0, 4095),
    "gripper": (0, 4095),
}

# 默认仿真物理校准中位姿态 (所有关节处于几何与标定 0 位: 大臂垂直、小臂水平、夹爪水平向正前)
DEFAULT_MOCK_WORKING_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}


class MockArmAdapter(BaseArmAdapter):
    """SO-ARM101 纯软件仿真机械臂适配器."""

    def __init__(
        self,
        config: ArmConfig,
        initial_positions: dict[str, float] | None = None,
        max_joint_speed_rad_s: float = 3.0,
    ) -> None:
        super().__init__(config)
        self.max_joint_speed_rad_s = max_joint_speed_rad_s

        # 内部状态 (统一存储为弧度制 rad)
        init_pos = initial_positions or DEFAULT_MOCK_WORKING_POSE
        self._current_positions: dict[str, float] = dict(init_pos)
        self._target_positions: dict[str, float] = dict(self._current_positions)
        self._torques: dict[str, bool] = {j: False for j in SO101_JOINT_NAMES}
        self._last_step_time: float = time.monotonic()
        self._fault_type: str | None = None

    def connect(self) -> None:
        """建立虚拟连接，模拟加载校准与平稳使能."""
        if self._connected:
            logger.info("[%s] 仿真机械臂已处于连接状态", self.config.robot_id)
            return

        if self._fault_type == "connect_error":
            msg = f"[{self.config.robot_id}] 模拟连接失败: 设备未就绪"
            self._health.last_error_message = msg
            raise RuntimeError(msg)

        logger.info("[%s] 正在连接仿真机械臂 (Role=%s)...", self.config.robot_id, self.config.role)

        # 尝试读取真实校准文件，若不存在则回退至默认仿真校准范围
        real_ranges = self.load_calibration()
        if not real_ranges:
            self._calibration_ranges = dict(DEFAULT_MOCK_CALIBRATION_RANGES)

        # 目标对齐当前位置，防止旧目标跳变
        self._target_positions = dict(self._current_positions)

        # 从臂使能全部扭矩，主臂默认释放扭矩
        is_follower = self.config.role == ArmRole.FOLLOWER
        for j in SO101_JOINT_NAMES:
            self._torques[j] = is_follower

        self._connected = True
        self._last_step_time = time.monotonic()
        self._health.torque_enabled_count = sum(1 for v in self._torques.values() if v)
        logger.info(
            "[%s] 仿真机械臂连接就绪! (使能舵机: %d/%d)",
            self.config.robot_id,
            self._health.torque_enabled_count,
            len(SO101_JOINT_NAMES),
        )

    def disconnect(self, keep_torque: bool = False) -> None:
        """安全断开虚拟机械臂."""
        if not self._connected:
            return

        logger.info("[%s] 正在断开仿真机械臂连接 (keep_torque=%s)...", self.config.robot_id, keep_torque)
        if not keep_torque:
            for j in SO101_JOINT_NAMES:
                self._torques[j] = False

        self._connected = False
        self._health.torque_enabled_count = sum(1 for v in self._torques.values() if v)
        logger.info("[%s] 仿真机械臂已断开", self.config.robot_id)

    def _step_dynamics(self) -> None:
        """根据耗时 dt 平滑推演关节插值运动."""
        now = time.monotonic()
        dt = max(1e-4, min(0.2, now - self._last_step_time))
        self._last_step_time = now

        max_arm_step = self.max_joint_speed_rad_s * dt
        max_gripper_step = (self.max_joint_speed_rad_s * (100.0 / math.pi)) * dt

        for j in SO101_JOINT_NAMES:
            # 仅在使能扭矩时响应目标位置更新
            if not self._torques.get(j, False):
                continue

            max_step = max_gripper_step if j == SO101_GRIPPER_JOINT_NAME else max_arm_step
            target = self._target_positions[j]
            current = self._current_positions[j]
            diff = target - current

            if abs(diff) <= max_step:
                self._current_positions[j] = target
            else:
                step = math.copysign(max_step, diff)
                self._current_positions[j] = current + step

    def read_state(self) -> JointState:
        """读取仿真关节当前观测状态."""
        if not self._connected:
            raise RuntimeError(f"[{self.config.robot_id}] 仿真机械臂未连接，无法读取状态")

        if self._fault_type == "read_error":
            self._health.consecutive_read_errors += 1
            msg = f"[{self.config.robot_id}] 模拟串口读取超时"
            self._health.last_error_message = msg
            raise TimeoutError(msg)

        # 推进物理插值运动
        self._step_dynamics()

        # 构造观测数据 (根据 config.internal_unit 转换 5 个臂关节，保持 gripper RANGE_0_100)
        positions = dict(self._current_positions)
        if self.config.internal_unit == JointUnit.DEGREE:
            positions = convert_joint_positions(
                positions,
                JointUnit.RADIAN,
                JointUnit.DEGREE,
                gripper_unit=JointUnit.RANGE_0_100,
            )

        self._record_read_latency(0.45)  # 模拟 ~0.45ms 极低读取延迟

        state = JointState(
            timestamp=time.monotonic(),
            positions=positions,
            unit=self.config.internal_unit,
            torques_enabled=dict(self._torques),
            gripper_unit=JointUnit.RANGE_0_100,
        )
        return state

    def send_action(self, action: RobotAction) -> bool:
        """向仿真机械臂下发动作目标."""
        if not self._connected:
            raise RuntimeError(f"[{self.config.robot_id}] 仿真机械臂未连接，无法下发动作")

        if self._fault_type == "action_rejected":
            logger.warning("[%s] 模拟动作被拒绝", self.config.robot_id)
            return False

        is_valid, err = action.validate()
        if not is_valid:
            logger.error("[%s] 拒绝执行非法动作: %s", self.config.robot_id, err)
            return False

        # 臂关节统一转换为弧度制，gripper 保持 RANGE_0_100 [0.0, 100.0]
        if action.action_type == ActionType.JOINT_POSITION:
            if not action.target_joint_positions:
                return False
            for j, val in action.target_joint_positions.items():
                if j in self._target_positions:
                    if j == SO101_GRIPPER_JOINT_NAME:
                        self._target_positions[j] = clamp_gripper_pct(val)
                    else:
                        target_rad = deg_to_rad(val) if action.unit == JointUnit.DEGREE else float(val)
                        self._target_positions[j] = target_rad

        elif action.action_type == ActionType.JOINT_DELTA:
            if not action.target_joint_deltas:
                return False
            for j, delta in action.target_joint_deltas.items():
                if j in self._target_positions:
                    if j == SO101_GRIPPER_JOINT_NAME:
                        self._target_positions[j] = clamp_gripper_pct(self._current_positions[j] + float(delta))
                    else:
                        delta_rad = deg_to_rad(delta) if action.unit == JointUnit.DEGREE else float(delta)
                        self._target_positions[j] = self._current_positions[j] + delta_rad

        else:
            logger.error("[%s] 不支持的动作类型: %s", self.config.robot_id, action.action_type)
            return False

        return True

    def stop_motion(self) -> None:
        """急停/停止运动: 立即将目标位置锁定为当前位置."""
        if not self._connected:
            return
        self._target_positions = dict(self._current_positions)
        logger.info("[%s] 仿真机械臂已执行原位保持锁止", self.config.robot_id)

    def move_to_middle(
        self,
        step_deg: float = 3.0,
        tolerance_deg: float = 1.0,
        timeout_s: float = 10.0,
        gripper_target: float | None = None,
    ) -> bool:
        """按安全次序平稳回到工作区中位 (5 臂关节归 0.0, gripper 默认保持当前开合度)."""
        if not self._connected:
            raise RuntimeError(f"[{self.config.robot_id}] 仿真机械臂未连接")

        logger.info("[%s] 正在按 MIDDLE_ORDER 顺序分步回中位...", self.config.robot_id)
        for joint in MIDDLE_ORDER:
            if joint == SO101_GRIPPER_JOINT_NAME:
                if gripper_target is not None:
                    g_val = clamp_gripper_pct(gripper_target)
                    self._target_positions[joint] = g_val
                    self._current_positions[joint] = g_val
                continue
            target_val = DEFAULT_MOCK_WORKING_POSE.get(joint, 0.0)
            self._target_positions[joint] = target_val
            self._current_positions[joint] = target_val

        logger.info("[%s] 所有臂关节已回到仿真中位", self.config.robot_id)
        return True

    def read_torque_states(self) -> dict[str, bool]:
        """获取各关节扭矩使能状态."""
        return dict(self._torques)

    def read_diagnostics(self) -> dict[str, Any]:
        """仿真舵机健康诊断信息."""
        return {
            "temperatures": {j: 32.5 for j in SO101_JOINT_NAMES},
            "voltages": {j: 12.1 for j in SO101_JOINT_NAMES},
            "max_temp": 32.5,
            "min_voltage": 12.1,
        }

    def read_positions(self, use_degrees: bool = False) -> dict[str, float]:
        """获取各关节当前位置数值 (arm 转 deg/rad, gripper 保持 RANGE_0_100)."""
        self._step_dynamics()
        if use_degrees:
            return convert_joint_positions(
                self._current_positions,
                JointUnit.RADIAN,
                JointUnit.DEGREE,
                gripper_unit=JointUnit.RANGE_0_100,
            )
        return dict(self._current_positions)

    def set_joint_positions(self, positions: dict[str, float], unit: JointUnit = JointUnit.RADIAN) -> None:
        """强制瞬时重设关节位置 (仅供仿真器/测试用例初始化)."""
        for j, val in positions.items():
            if j in self._current_positions:
                if j == SO101_GRIPPER_JOINT_NAME:
                    g_val = clamp_gripper_pct(val)
                    self._current_positions[j] = g_val
                    self._target_positions[j] = g_val
                else:
                    rad_val = deg_to_rad(val) if unit == JointUnit.DEGREE else float(val)
                    self._current_positions[j] = rad_val
                    self._target_positions[j] = rad_val

    def set_torque(self, joint: str, enable: bool) -> None:
        """设置单个关节扭矩使能."""
        if joint in self._torques:
            self._torques[joint] = enable
            self._health.torque_enabled_count = sum(1 for v in self._torques.values() if v)

    def disable_torque(self) -> bool:
        """释放所有模拟关节扭矩."""
        for j in self._torques:
            self._torques[j] = False
        self._health.torque_enabled_count = 0
        return True

    def enable_torque(self) -> bool:
        """恢复所有模拟关节扭矩."""
        for j in self._torques:
            self._torques[j] = True
        self._health.torque_enabled_count = len(self._torques)
        return True

    def inject_fault(self, fault_type: str | None) -> None:
        """注入模拟故障 (connect_error, read_error, action_rejected 等)."""
        self._fault_type = fault_type
        logger.warning("[%s] 注入故障状态: %s", self.config.robot_id, fault_type)

    def clear_fault(self) -> None:
        """清除注入的故障."""
        self._fault_type = None
        self._health.consecutive_read_errors = 0
