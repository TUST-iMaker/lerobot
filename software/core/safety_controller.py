"""SO-ARM101 核心安全控制器.

职责:
1. 动作下发唯一强制门禁: 任何来源 (键盘、Leader、回放、模型) 均需在此进行安全审计;
2. 软限位保护 (Soft Limit): 拦截或平滑截断超出关节安全角度范围的目标;
3. 速度/增量平滑限幅 (Velocity/Delta Clamp): 严格限制单控制周期内最大角位移, 杜绝猛甩与齿轮冲击;
4. 跟随误差监控 (Tracking Error Watchdog): 连续跟踪误差过大即判定卡阻失步, 触发故障停机;
5. 状态机协同: 严格审查控制权所有权与急停状态.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any

from core.data_types import (
    SO101_ARM_JOINT_NAMES,
    SO101_GRIPPER_JOINT_NAME,
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    SafetyReport,
    SystemState,
    convert_joint_positions,
    deg_to_rad,
    rad_to_deg,
)
from core.state_machine import RobotStateMachine

logger = logging.getLogger(__name__)

# 默认臂关节软限位 (严格对齐 repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf)
DEFAULT_JOINT_LIMITS_RAD: dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
}
DEFAULT_GRIPPER_LIMITS_PCT: tuple[float, float] = (0.0, 100.0)


def get_default_max_joint_delta_rad() -> float:
    """动态读取 config.yaml 中的 safety.max_step_deg 并转换为弧度.
    
    默认联动配置文件 (如 max_step_deg=15.0° 时对应 ~0.26 rad/帧)，
    避免操作员快速拖拽示教臂时动作被过度截断导致发粘滞后。
    """
    try:
        from core.config_loader import get_config
        cfg = get_config()
        if hasattr(cfg, "safety") and hasattr(cfg.safety, "max_step_deg"):
            return deg_to_rad(float(cfg.safety.max_step_deg))
    except Exception:
        pass
    return deg_to_rad(15.0)


@dataclass
class SafetyLimitsConfig:
    """安全限制阈值配置."""
    # 各臂关节软限位 (min_rad, max_rad)
    joint_limits_rad: dict[str, tuple[float, float]] = field(
        default_factory=lambda: dict(DEFAULT_JOINT_LIMITS_RAD)
    )
    # 夹爪安全百分比软限位 (官方 RANGE_0_100: [0.0, 100.0])
    gripper_limits_pct: tuple[float, float] = DEFAULT_GRIPPER_LIMITS_PCT
    # 控制周期内单步最大允许角增量 (弧度)，默认与 config.yaml 中的 max_step_deg 联动
    max_joint_delta_rad: float = field(default_factory=get_default_max_joint_delta_rad)
    # 控制周期内夹爪单步最大允许增量 (百分比 %/step)
    max_gripper_delta_pct: float = 15.0
    # 跟随误差报警阈值 (当前目标与实际位置相差超过此值视为异常卡死)
    max_tracking_error_rad: float = 0.40
    # 夹爪跟随误差报警阈值 (百分比 %)
    max_tracking_error_gripper_pct: float = 25.0
    # 连续允许超差的最大帧数
    max_consecutive_tracking_errors: int = 5
    # 超出限位时的处理策略: "clamp" (自动截断到边界) 或 "reject" (拒绝本帧动作)
    limit_action_mode: str = "clamp"
    # 心跳超时时间 (秒): 超过此时间没有新命令，触发自动静止
    command_timeout_s: float = 0.5


class SafetyController:
    """机械臂全局动作安全守护控制器."""

    def __init__(
        self,
        state_machine: RobotStateMachine,
        config: SafetyLimitsConfig | None = None,
    ) -> None:
        self.state_machine = state_machine
        self.config = config or SafetyLimitsConfig()
        self._consecutive_tracking_errors: int = 0
        self._last_valid_command_time = time.monotonic()

    def filter_and_verify(
        self,
        action: RobotAction,
        current_state: JointState,
    ) -> tuple[bool, RobotAction | None, str]:
        """对下发的动作意图执行强制安全检查与平滑限幅.
        
        Args:
            action: 待下发的动作命令
            current_state: 当前机械臂实际物理关节反馈
            
        Returns:
            (是否放行, 安全过滤后的新动作对象, 说明信息)
        """
        # 1. 基础合法性检验
        is_valid, err = action.validate()
        if not is_valid:
            return False, None, f"动作自身参数校验失败: {err}"

        # 2. 状态机门禁
        current_sys_state = self.state_machine.current_state
        if current_sys_state == SystemState.ESTOP_LATCHED:
            return False, None, f"系统处于急停锁定状态 ({self.state_machine.estop_reason})，拒绝一切动作"

        if current_sys_state == SystemState.FAULT:
            return False, None, f"系统处于故障锁定状态 ({self.state_machine.fault_reason})，拒绝动作"

        if current_sys_state not in (SystemState.TELEOP, SystemState.RECORDING, SystemState.PLAYBACK):
            return False, None, f"当前系统状态 [{current_sys_state.value}] 不允许执行运动动作"

        # 3. 控制源所有权审查
        active_source = self.state_machine.active_source
        if action.source != active_source:
            return (
                False,
                None,
                f"控制源不匹配: 动作来源为 {action.source.value}，但当前授权来源为 {active_source.value if active_source else '无'}",
            )

        # 4. 单位统一规范化 (内部安全计算统一在 RADIAN 下执行)
        current_state_rad = current_state.to_radians() if current_state.unit != JointUnit.RADIAN else current_state

        # 处理 JOINT_POSITION 绝对模式
        if action.action_type == ActionType.JOINT_POSITION:
            if not action.target_joint_positions:
                return False, None, "缺少目标关节位置"

            # 臂关节转为弧度，夹爪保持 RANGE_0_100 [0.0, 100.0]
            raw_targets = action.target_joint_positions
            targets_rad = convert_joint_positions(
                raw_targets,
                action.unit,
                JointUnit.RADIAN,
                gripper_unit=action.gripper_unit,
            )

            safe_targets_rad: dict[str, float] = {}

            # 5. 遍历每个关节做限位与增量限速检查
            for joint_name, target_val in targets_rad.items():
                cur_val = current_state_rad.positions.get(joint_name, target_val)

                if joint_name == SO101_GRIPPER_JOINT_NAME:
                    min_lim, max_lim = self.config.gripper_limits_pct
                    if target_val < min_lim or target_val > max_lim:
                        if self.config.limit_action_mode == "reject":
                            return (
                                False,
                                None,
                                f"夹爪目标 {target_val:.1f}% 超出软限位 [{min_lim:.1f}%, {max_lim:.1f}%]",
                            )
                        target_val = max(min_lim, min(max_lim, target_val))

                    delta = target_val - cur_val
                    max_delta = self.config.max_gripper_delta_pct
                    if abs(delta) > max_delta:
                        target_val = cur_val + math.copysign(max_delta, delta)
                else:
                    # A. 臂关节软限位检查
                    if joint_name in self.config.joint_limits_rad:
                        min_lim, max_lim = self.config.joint_limits_rad[joint_name]
                        if target_val < min_lim or target_val > max_lim:
                            if self.config.limit_action_mode == "reject":
                                return (
                                    False,
                                    None,
                                    f"关节 {joint_name} 目标 {target_val:.3f} rad 超出软限位 [{min_lim:.3f}, {max_lim:.3f}]",
                                )
                            # clamp 模式：截断到边界
                            target_val = max(min_lim, min(max_lim, target_val))

                    # B. 单步最大角增量限幅 (防止猛甩冲击)
                    delta = target_val - cur_val
                    max_delta = self.config.max_joint_delta_rad
                    if abs(delta) > max_delta:
                        target_val = cur_val + math.copysign(max_delta, delta)

                safe_targets_rad[joint_name] = target_val

            self._last_valid_command_time = time.monotonic()

            # 将经过限位和限速后的安全值打包返回
            safe_action = RobotAction(
                action_type=ActionType.JOINT_POSITION,
                source=action.source,
                timestamp=action.timestamp,
                target_joint_positions=safe_targets_rad,
                unit=JointUnit.RADIAN,
                gripper_unit=JointUnit.RANGE_0_100,
            )
            return True, safe_action, "安全检查通过 (已应用限位与增量平滑限制)"

        # 处理 JOINT_DELTA 相对增量模式
        elif action.action_type == ActionType.JOINT_DELTA:
            if not action.target_joint_deltas:
                return False, None, "缺少目标关节增量"

            raw_deltas = action.target_joint_deltas
            deltas_rad = convert_joint_positions(
                raw_deltas,
                action.unit,
                JointUnit.RADIAN,
                gripper_unit=action.gripper_unit,
            )

            safe_deltas_rad: dict[str, float] = {}
            for joint_name, delta_val in deltas_rad.items():
                cur_val = current_state_rad.positions.get(joint_name, 0.0)
                if joint_name == SO101_GRIPPER_JOINT_NAME:
                    max_delta = self.config.max_gripper_delta_pct
                    if abs(delta_val) > max_delta:
                        delta_val = math.copysign(max_delta, delta_val)
                    expected_target = cur_val + delta_val
                    min_lim, max_lim = self.config.gripper_limits_pct
                    if expected_target < min_lim:
                        delta_val = max(0.0, min_lim - cur_val)
                    elif expected_target > max_lim:
                        delta_val = min(0.0, max_lim - cur_val)
                else:
                    max_delta = self.config.max_joint_delta_rad
                    # 速度限幅
                    if abs(delta_val) > max_delta:
                        delta_val = math.copysign(max_delta, delta_val)

                    # 叠加后做限位保护
                    expected_target = cur_val + delta_val
                    if joint_name in self.config.joint_limits_rad:
                        min_lim, max_lim = self.config.joint_limits_rad[joint_name]
                        if expected_target < min_lim:
                            delta_val = max(0.0, min_lim - cur_val)
                        elif expected_target > max_lim:
                            delta_val = min(0.0, max_lim - cur_val)

                safe_deltas_rad[joint_name] = delta_val

            self._last_valid_command_time = time.monotonic()

            safe_action = RobotAction(
                action_type=ActionType.JOINT_DELTA,
                source=action.source,
                timestamp=action.timestamp,
                target_joint_deltas=safe_deltas_rad,
                unit=JointUnit.RADIAN,
                gripper_unit=JointUnit.RANGE_0_100,
            )
            return True, safe_action, "增量安全检查通过"

        return False, None, f"暂不支持的动作类型: {action.action_type.value}"

    def check_tracking_error(
        self,
        commanded_action: RobotAction,
        current_state: JointState,
    ) -> tuple[bool, str | None]:
        """检查机械臂是否产生过大的跟随误差 (失步、卡阻或外力干涉监测).
        
        Returns:
            (是否正常, 异常描述)
        """
        if commanded_action.action_type != ActionType.JOINT_POSITION or not commanded_action.target_joint_positions:
            return True, None

        current_rad = current_state.to_radians() if current_state.unit != JointUnit.RADIAN else current_state
        targets = commanded_action.target_joint_positions
        targets_rad = convert_joint_positions(
            targets,
            commanded_action.unit,
            JointUnit.RADIAN,
            gripper_unit=commanded_action.gripper_unit,
        )

        exceeded_joints: list[str] = []
        for joint, target_val in targets_rad.items():
            if joint not in current_rad.positions:
                continue
            cur_val = current_rad.positions[joint]
            error = abs(target_val - cur_val)
            if joint == SO101_GRIPPER_JOINT_NAME:
                if error > self.config.max_tracking_error_gripper_pct:
                    exceeded_joints.append(
                        f"{joint}(误差={error:.1f}% > 阈值{self.config.max_tracking_error_gripper_pct:.1f}%)"
                    )
            else:
                if error > self.config.max_tracking_error_rad:
                    exceeded_joints.append(
                        f"{joint}(误差={error:.2f}rad > 阈值{self.config.max_tracking_error_rad:.2f}rad)"
                    )

        if exceeded_joints:
            self._consecutive_tracking_errors += 1
            msg = f"检测到舵机跟随误差过大: {', '.join(exceeded_joints)}"
            logger.warning("[%d/%d] %s", self._consecutive_tracking_errors, self.config.max_consecutive_tracking_errors, msg)

            if self._consecutive_tracking_errors >= self.config.max_consecutive_tracking_errors:
                fault_reason = f"连续 {self.config.max_consecutive_tracking_errors} 帧舵机跟踪误差超标，疑似物理卡死堵转!"
                self.state_machine.trigger_fault(fault_reason)
                return False, fault_reason
            return True, msg

        self._consecutive_tracking_errors = 0
        return True, None

    def create_safety_report(self) -> SafetyReport:
        """生成系统当前的安全健康综合报告."""
        sys_state = self.state_machine.current_state
        return SafetyReport(
            timestamp=time.monotonic(),
            system_state=sys_state,
            estop_active=(sys_state == SystemState.ESTOP_LATCHED),
            limit_triggered=False,
            tracking_error_exceeded=(self._consecutive_tracking_errors > 0),
            active_source=self.state_machine.active_source,
            message=self.state_machine.estop_reason or self.state_machine.fault_reason or "系统状态正常",
        )
