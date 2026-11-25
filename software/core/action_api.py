"""SO-ARM101 统一动作入口与控制中枢 (ActionAPI).

职责:
1. 系统统一总入口: 汇聚控制权仲裁、安全检查流水线与硬件执行;
2. 动作提交唯一出口: submit_action() 强制经过安全控制器多重审计后方可下发硬件;
3. 急停双重闭环: 触发急停时同步锁定状态机并指示硬件执行原位保持锁止;
4. 状态观测聚合: 统一提供从臂状态、主臂动作捕获、整机多模态观测;
5. 解耦前端与输入源: 键盘、遥操作、Web API、模型推理等模块只需与 ActionAPI 交互.
"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

from core.data_types import (
    SO101_ARM_JOINT_NAMES,
    SO101_GRIPPER_JOINT_NAME,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    RobotObservation,
    SafetyReport,
    SystemState,
    clamp_gripper_pct,
)
from core.safety_controller import SafetyController, SafetyLimitsConfig
from core.state_machine import RobotStateMachine

if TYPE_CHECKING:
    from hardware.config import ArmConfig, RobotSystemConfig
    from hardware.system_adapter import RobotSystemAdapter

logger = logging.getLogger(__name__)


class ActionAPI:
    """机械臂统一动作接口与控制中枢."""

    def __init__(
        self,
        hardware: RobotSystemAdapter | None = None,
        state_machine: RobotStateMachine | None = None,
        safety_controller: SafetyController | None = None,
        config: RobotSystemConfig | None = None,
        calibration_dir: Path | str | None = None,
    ) -> None:
        """初始化统一动作中枢.
        
        支持直接依赖注入, 或传入配置自动构建内部组件.
        """
        if hardware is not None:
            self.hardware = hardware
        else:
            from hardware.config import ArmConfig, RobotSystemConfig
            from hardware.system_adapter import RobotSystemAdapter

            if config is not None:
                sys_cfg = config
            else:
                sys_cfg = RobotSystemConfig(
                    follower=ArmConfig.default_follower(calibration_dir=calibration_dir)
                )
            self.hardware = RobotSystemAdapter(sys_cfg)

        self.state_machine = state_machine or RobotStateMachine(initial_state=SystemState.DISCONNECTED)
        self.safety_controller = safety_controller or SafetyController(self.state_machine)

        # 注册状态机回调，急停时自动同步触发硬件停止
        self.state_machine.register_callback(self._on_state_transition)

    def _on_state_transition(
        self,
        old_state: SystemState,
        new_state: SystemState,
        reason: str,
    ) -> None:
        """状态转移响应: 触发急停或故障时立即联动硬件刹车."""
        if new_state in (SystemState.ESTOP_LATCHED, SystemState.FAULT):
            if self.hardware.is_follower_connected:
                self.hardware.stop_motion()

    # =========================================================================
    # 1. 硬件连接与生命周期
    # =========================================================================

    def connect(self, connect_leader: bool = True) -> bool:
        """连接机器人硬件，成功后推动系统状态机进入 IDLE 待机就绪."""
        if self.state_machine.current_state != SystemState.DISCONNECTED:
            logger.info("系统已处于连接状态: %s", self.state_machine.current_state.value)
            return True

        try:
            self.hardware.connect(connect_leader=connect_leader)
            if self.hardware.is_follower_connected:
                self.state_machine.transition_to(SystemState.IDLE, "硬件连接完成，伺服就绪")
                return True
            return False
        except Exception as exc:
            logger.error("连接机械臂失败: %s", exc)
            self.state_machine.trigger_fault(f"连接异常: {exc}")
            return False

    def safe_return_to_rest(self, duration_s: float = 2.0, release_torque: bool = True) -> bool:
        """安全平滑收回蜷缩休眠姿态 (基于 rest_pose.json 或系统折叠预设)."""
        if not self.hardware.is_follower_connected:
            return False

        import json
        from core.data_types import (
            ActionType,
            SO101_JOINT_NAMES,
            deg_to_rad,
        )

        # 优先读取配置文件指定的 rest_pose.json
        try:
            from core.config_loader import get_config
            cfg = get_config()
            rest_path = cfg.safety.rest_pose_path
            if duration_s == 2.0:
                duration_s = cfg.safety.rest_duration_s
        except Exception:
            rest_path = Path(__file__).resolve().parent.parent / "rest_pose.json"

        target_rad: dict[str, float] | None = None
        is_from_file = False
        if rest_path.is_file():
            try:
                with open(rest_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                joints_data = data.get("joints", {})
                is_deg = data.get("unit") == "degree"
                parsed_rad: dict[str, float] = {}
                for k, v in joints_data.items():
                    name = k[:-4] if k.endswith(".pos") else k
                    if name == SO101_GRIPPER_JOINT_NAME:
                        parsed_rad[name] = clamp_gripper_pct(float(v))
                    else:
                        parsed_rad[name] = deg_to_rad(float(v)) if is_deg else float(v)
                if all(j in parsed_rad for j in SO101_JOINT_NAMES):
                    target_rad = parsed_rad
                    is_from_file = True
            except Exception as exc:
                logger.warning("加载蜷缩姿态文件 %s 失败: %s，将回退至系统预设", rest_path, exc)
                target_rad = None

        if not target_rad:
            from core.presets import get_preset_pose
            target_rad = get_preset_pose("rest", unit=JointUnit.RADIAN)

        try:
            cur_state = self.hardware.read_follower_state()
            cur_state_rad = cur_state.to_radians() if cur_state.unit != JointUnit.RADIAN else cur_state
            start_positions = {j: cur_state_rad.positions.get(j, 0.0) for j in SO101_JOINT_NAMES}

            # 若臂关节平均差值 < 0.04 rad (~2°) 且夹爪差值 < 2.5%，说明已在蜷缩位附近
            max_arm_diff = max(
                abs(start_positions[j] - target_rad.get(j, start_positions[j]))
                for j in SO101_ARM_JOINT_NAMES
            )
            gripper_diff = abs(
                start_positions.get(SO101_GRIPPER_JOINT_NAME, 0.0)
                - target_rad.get(SO101_GRIPPER_JOINT_NAME, start_positions.get(SO101_GRIPPER_JOINT_NAME, 0.0))
            )
            if max_arm_diff < 0.04 and gripper_diff < 2.5:
                if release_torque:
                    self.disable_torque()
                    curr_src = self.state_machine.active_source
                    if curr_src == ControlSource.SCRIPT:
                        self.release_control(curr_src)
                    print("[✓] 机械臂已处于蜷缩位附近，扭矩已释放进入休眠。")
                return True

            # 确保持有执行动作的 SCRIPT 控制权
            curr_src = self.state_machine.active_source
            if curr_src != ControlSource.SCRIPT:
                if curr_src is not None:
                    self.release_control(curr_src)
                if not self.claim_control(ControlSource.SCRIPT):
                    if self.state_machine.current_state in (SystemState.FAULT, SystemState.ESTOP_LATCHED):
                        logger.warning("系统处于锁定状态 (%s)，执行复位以允许收回蜷缩姿态", self.state_machine.current_state.value)
                        self.state_machine.request_reset()
                        self.claim_control(ControlSource.SCRIPT)

            # 自适应运动时长: 臂关节与夹爪分别按 rad 与 % 换算时长，避免混合单位导致百秒级停顿
            actual_duration = max(duration_s, max_arm_diff / 0.75, gripper_diff / 50.0)
            target_source_name = rest_path.name if is_from_file else "内置折叠预设"
            print(f"[*] 正在执行 S 曲线平滑过渡 (当前姿态 -> {target_source_name}, 角跨度 {math.degrees(max_arm_diff):.1f}°, 耗时 {actual_duration:.1f}s)...")

            rate_hz = 30.0
            total_steps = max(10, int(actual_duration * rate_hz))
            dt = 1.0 / rate_hz

            for step in range(1, total_steps + 1):
                alpha = step / float(total_steps)
                # 五次多项式 / 余弦 S 曲线平滑加速减速
                smooth_alpha = 0.5 * (1.0 - math.cos(alpha * math.pi))
                interp_pos = {}
                for j in SO101_JOINT_NAMES:
                    s_val = start_positions[j]
                    t_val = target_rad.get(j, s_val)
                    interp_pos[j] = s_val + (t_val - s_val) * smooth_alpha

                action = RobotAction(
                    action_type=ActionType.JOINT_POSITION,
                    source=self.state_machine.active_source or ControlSource.SCRIPT,
                    timestamp=time.monotonic(),
                    target_joint_positions=interp_pos,
                    unit=JointUnit.RADIAN,
                    gripper_unit=JointUnit.RANGE_0_100,
                )
                ok, msg = self.submit_action(action)
                if not ok:
                    logger.warning("回蜷缩位动作被拦截: %s", msg)
                time.sleep(dt)

            # 终点平稳保位缓冲 (Settle Buffer): 持续保持最终目标 12 帧 (~0.4s)，让机械臂依靠使能力矩彻底压实贴合
            hold_steps = 12
            for _ in range(hold_steps):
                hold_action = RobotAction(
                    action_type=ActionType.JOINT_POSITION,
                    source=self.state_machine.active_source or ControlSource.SCRIPT,
                    timestamp=time.monotonic(),
                    target_joint_positions=target_rad,
                    unit=JointUnit.RADIAN,
                    gripper_unit=JointUnit.RANGE_0_100,
                )
                self.submit_action(hold_action)
                time.sleep(dt)

            print("[✓] 从臂已平滑移动至蜷缩休眠位。")
            if release_torque:
                time.sleep(0.3)
                self.disable_torque()
                curr_src = self.state_machine.active_source
                if curr_src == ControlSource.SCRIPT:
                    self.release_control(curr_src)
                print("[✓] 从臂扭矩已安全释放进入休眠。")
            return True
        except Exception as exc:
            logger.warning("执行安全回蜷缩位异常: %s", exc)
            return False

    def disable_torque(self) -> bool:
        """主动释放从臂舵机扭矩 (进入松弛休眠状态)."""
        return self.hardware.disable_torque()

    def enable_torque(self) -> bool:
        """主动恢复从臂舵机扭矩."""
        return self.hardware.enable_torque()

    def disconnect(self, keep_torque: bool = False, return_to_rest: bool = True) -> None:
        """安全断开机械臂连接，系统回归 DISCONNECTED.
        
        如果启用了 return_to_rest (默认开启) 且未要求保持扭矩，将在断电前强制平缓回归蜷缩休眠位.
        """
        logger.info("正在断开系统连接...")
        if return_to_rest and not keep_torque and self.hardware.is_follower_connected:
            try:
                print("\n[*] 安全退出守护: 正在平缓收回蜷缩休眠姿态 (S曲线平滑过渡)...")
                self.safe_return_to_rest(duration_s=2.0)
                print("[✓] 机械臂已安全回归蜷缩休眠位。")
            except Exception as e:
                logger.warning("退出回归蜷缩位异常: %s", e)

        # 释放控制权以允许状态机平稳切换到 DISCONNECTED
        curr_source = self.state_machine.active_source
        if curr_source is not None:
            self.state_machine.release_control(curr_source)

        self.hardware.disconnect(keep_torque=keep_torque)
        self.state_machine.transition_to(SystemState.DISCONNECTED, "硬件已断开")

    # =========================================================================
    # 2. 控制权独占申请与释放
    # =========================================================================

    def claim_control(self, source: ControlSource) -> bool:
        """申请独占控制权 (进入 TELEOP 或 RECORDING)."""
        return self.state_machine.claim_control(source)

    def release_control(self, source: ControlSource) -> bool:
        """释放控制权 (退回 IDLE 待机)."""
        return self.state_machine.release_control(source)

    # =========================================================================
    # 3. 核心动作提交与安全执行流水线
    # =========================================================================

    def submit_action(self, action: RobotAction) -> tuple[bool, str]:
        """提交并执行动作意图.
        
        执行全流程:
        1. 检查硬件连接;
        2. 读取当前真实关节反馈;
        3. 安全门禁审查 (状态机门禁、来源审查、软限位截断、单步速度平滑限幅);
        4. 下发硬件执行;
        5. 跟踪误差健康监测 (超差过大自动报警停机).
        
        Returns:
            (是否成功下发执行, 结果详细描述)
        """
        if not self.hardware.is_follower_connected:
            return False, "从臂未连接，无法执行动作"

        # 1. 获取当前机械臂物理反馈
        try:
            current_state = self.hardware.read_follower_state()
        except Exception as exc:
            msg = f"读取当前机械臂状态失败: {exc}"
            logger.error(msg)
            return False, msg

        # 2. 强制安全审查与平滑限幅
        is_safe, safe_action, reason = self.safety_controller.filter_and_verify(
            action=action,
            current_state=current_state,
        )
        if not is_safe or safe_action is None:
            logger.warning("动作被安全门禁拦截: %s", reason)
            return False, f"安全拦截: {reason}"

        # 3. 下发给硬件执行器
        try:
            sent_ok = self.hardware.send_action(safe_action)
            if not sent_ok:
                return False, "硬件驱动拒绝或执行失败"
        except Exception as exc:
            msg = f"下发动作到硬件产生异常: {exc}"
            logger.error(msg)
            return False, msg

        # 4. 跟踪误差比对检查 (堵转与卡死监测)
        self.safety_controller.check_tracking_error(
            commanded_action=safe_action,
            current_state=current_state,
        )

        return True, "动作执行成功"

    # =========================================================================
    # 4. 状态与观测数据获取
    # =========================================================================

    def get_joint_state(self) -> JointState:
        """读取当前从臂各关节物理状态反馈."""
        return self.hardware.read_follower_state()

    def get_leader_action(self) -> RobotAction | None:
        """若主臂已连接，捕获主臂当前的示教动作意图."""
        if not self.hardware.leader or not self.hardware.is_leader_connected:
            return None
        if hasattr(self.hardware.leader, "read_action"):
            return self.hardware.leader.read_action()
        l_state = self.hardware.read_leader_state()
        if l_state is None:
            return None
        return RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.LEADER,
            timestamp=l_state.timestamp,
            target_joint_positions=dict(l_state.positions),
            unit=l_state.unit,
            gripper_unit=l_state.gripper_unit,
        )

    def get_observation(self) -> RobotObservation:
        """生成某一时刻的标准多模态观测快照."""
        joint_state = self.get_joint_state()
        return RobotObservation(
            timestamp=time.monotonic(),
            joint_state=joint_state,
            tcp_state=None,  # 运动学就绪后填入
            camera_frames={},  # 相机模块接入后填入
        )

    # =========================================================================
    # 5. 紧急停止与人工复位
    # =========================================================================

    def request_estop(self, reason: str = "用户触发急停") -> None:
        """立刻触发系统急停锁定.
        
        触发后:
        1. 状态机闭锁为 ESTOP_LATCHED, 拒绝一切新动作输入;
        2. 硬件从臂立即锁定当前物理位置并保持力矩.
        """
        self.state_machine.trigger_estop(reason)
        # 确保硬件层立即制动
        if self.hardware.is_follower_connected:
            self.hardware.stop_motion()

    def request_reset(self) -> tuple[bool, str]:
        """请求显式人工复位 (解除急停或故障锁定回到 IDLE)."""
        return self.state_machine.request_reset()

    # =========================================================================
    # 6. 受控平滑运动辅助
    # =========================================================================

    def move_to_middle(
        self,
        step_deg: float = 3.0,
        tolerance_deg: float = 1.0,
        timeout_s: float = 60.0,
    ) -> bool:
        """将机械臂安全平稳移动回校准中位."""
        curr = self.state_machine.current_state
        if curr in (SystemState.ESTOP_LATCHED, SystemState.FAULT):
            logger.warning("急停/故障状态下禁止回中位")
            return False

        # 临时申请进入 TELEOP
        prev_state = curr
        if curr == SystemState.IDLE:
            self.state_machine.transition_to(SystemState.TELEOP, "开始回中位动作")

        try:
            return self.hardware.follower.move_to_middle(
                step_deg=step_deg,
                tolerance_deg=tolerance_deg,
                timeout_s=timeout_s,
            )
        finally:
            if prev_state == SystemState.IDLE:
                self.state_machine.transition_to(SystemState.IDLE, "回中位动作完成")

    # =========================================================================
    # 7. 全局健康与遥测报告
    # =========================================================================

    def get_status_report(self) -> dict[str, Any]:
        """获取系统整机运行状态、控制权所有者及硬件健康全貌报告."""
        safety_rep = self.safety_controller.create_safety_report()
        hw_health = self.hardware.get_health_report()

        return {
            "timestamp": time.monotonic(),
            "system_state": self.state_machine.current_state.value,
            "active_source": self.state_machine.active_source.value if self.state_machine.active_source else None,
            "is_estop": self.state_machine.is_estop,
            "estop_reason": self.state_machine.estop_reason,
            "safety": safety_rep.to_dict(),
            "hardware": hw_health,
        }

