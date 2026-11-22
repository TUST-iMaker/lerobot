"""SO-ARM101 现场演示宏动作执行引擎 (DemoRoutine).

用于现场一键执行预设的展示动作流 (如挥手致意、观察位巡检、模拟抓取与中位复位)，
内置平滑插值算法，动作过渡柔和，绝无突变或机械撞击。
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from core.action_api import ActionAPI
from core.data_types import (
    SO101_ARM_JOINT_NAMES,
    SO101_GRIPPER_JOINT_NAME,
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointUnit,
    RobotAction,
    deg_to_rad,
)
from core.presets import PRESET_POSES, get_preset_pose

logger = logging.getLogger(__name__)


class DemoRoutineRunner:
    """机械臂预设宏动作执行器."""

    def __init__(self, action_api: ActionAPI) -> None:
        self.api = action_api

    def interpolate_and_send(
        self,
        target_positions: dict[str, float],
        duration_s: float = 2.0,
        rate_hz: float = 30.0,
    ) -> bool:
        """从当前关节姿态平滑线性插值过渡到目标姿态."""
        cur_state = self.api.get_joint_state()
        if not cur_state:
            logger.error("无法读取机械臂当前关节状态")
            return False

        cur_state_rad = cur_state.to_radians() if cur_state.unit != JointUnit.RADIAN else cur_state
        start_positions = {j: cur_state_rad.positions.get(j, 0.0) for j in SO101_JOINT_NAMES}

        # 分别按弧度 (臂关节) 与百分比 (gripper) 计算自适应时长，防止混合单位产生百秒级延迟
        max_arm_diff = max(
            abs(start_positions[j] - target_positions.get(j, start_positions[j]))
            for j in SO101_ARM_JOINT_NAMES
        )
        gripper_diff = abs(
            start_positions.get(SO101_GRIPPER_JOINT_NAME, 0.0)
            - target_positions.get(SO101_GRIPPER_JOINT_NAME, start_positions.get(SO101_GRIPPER_JOINT_NAME, 0.0))
        )
        actual_duration = max(duration_s, max_arm_diff / 0.8, gripper_diff / 50.0)

        total_steps = max(10, int(actual_duration * rate_hz))
        dt = 1.0 / rate_hz

        for step in range(1, total_steps + 1):
            alpha = step / float(total_steps)
            # 使用余弦平滑因子 (S 曲线减速加速)
            smooth_alpha = 0.5 * (1.0 - math.cos(alpha * math.pi))

            interp_pos = {}
            for j in SO101_JOINT_NAMES:
                s_val = start_positions[j]
                t_val = target_positions.get(j, s_val)
                interp_pos[j] = s_val + (t_val - s_val) * smooth_alpha

            action = RobotAction(
                action_type=ActionType.JOINT_POSITION,
                source=ControlSource.SCRIPT,
                timestamp=time.monotonic(),
                target_joint_positions=interp_pos,
                unit=JointUnit.RADIAN,
                gripper_unit=JointUnit.RANGE_0_100,
            )
            ok, msg = self.api.submit_action(action)
            if not ok:
                logger.warning("插值单步动作执行未成功: %s", msg)
            time.sleep(dt)

        # 终点平稳保位缓冲 (Settle Buffer: 持续保持最终目标 12 帧约 0.4 秒)
        hold_steps = 12
        for _ in range(hold_steps):
            hold_action = RobotAction(
                action_type=ActionType.JOINT_POSITION,
                source=ControlSource.SCRIPT,
                timestamp=time.monotonic(),
                target_joint_positions=target_positions,
                unit=JointUnit.RADIAN,
                gripper_unit=JointUnit.RANGE_0_100,
            )
            self.api.submit_action(hold_action)
            time.sleep(dt)

        return True

    def run_pose(self, pose_name: str, duration_s: float = 2.0) -> bool:
        """平滑运动至指定的预设姿态."""
        logger.info(">>> 开始执行预设姿态: [%s] (耗时 %.1f s) <<<", pose_name, duration_s)
        target = get_preset_pose(pose_name, unit=JointUnit.RADIAN)
        return self.interpolate_and_send(target, duration_s=duration_s)

    def run_wave_routine(self) -> bool:
        """执行挥手致意展示动线: 回中位 -> 抬起大臂 -> 腕部左右摇摆 2 次 -> 回中位."""
        logger.info(">>> 启动【挥手致意 (Wave Routine)】现场展示动线 <<<")
        # 1. 回中位
        self.run_pose("home", duration_s=1.5)
        time.sleep(0.3)

        # 2. 抬臂预备
        self.run_pose("wave", duration_s=1.5)
        time.sleep(0.2)

        # 3. 手腕摇摆 (摆动 wrist_roll 与 shoulder_pan)
        wave_base = get_preset_pose("wave", unit=JointUnit.RADIAN)
        for _ in range(2):
            left_pose = dict(wave_base)
            left_pose["wrist_roll"] = 0.85
            left_pose["shoulder_pan"] = 0.20
            self.interpolate_and_send(left_pose, duration_s=0.6)

            right_pose = dict(wave_base)
            right_pose["wrist_roll"] = -0.50
            right_pose["shoulder_pan"] = -0.20
            self.interpolate_and_send(right_pose, duration_s=0.6)

        time.sleep(0.3)
        # 4. 平稳收回安全蜷缩休眠位
        logger.info("[*] 挥手结束: 正在平缓收回安全蜷缩休眠姿态...")
        self.api.safe_return_to_rest(duration_s=2.0)
        logger.info("[✓] 挥手致意动线执行完毕.")
        return True

    def run_pick_routine(self) -> bool:
        """执行模拟抓取展示动线: 巡检观察 -> 张开夹爪下探 -> 闭合夹紧 -> 抬起 -> 放置 -> 复位."""
        logger.info(">>> 启动【模拟抓取 (Pick & Place Routine)】现场展示动线 <<<")
        self.run_pose("home", duration_s=1.2)
        self.run_pose("inspect", duration_s=1.5)
        time.sleep(0.5)

        # 张开夹爪下探
        self.run_pose("pick_hover", duration_s=1.5)
        time.sleep(0.5)

        # 闭合夹爪 (RANGE_0_100: 10% 夹紧)
        gripped_pose = get_preset_pose("pick_hover", unit=JointUnit.RADIAN)
        gripped_pose["gripper"] = 10.0
        self.interpolate_and_send(gripped_pose, duration_s=0.8)
        time.sleep(0.5)

        # 抬升
        lift_pose = dict(gripped_pose)
        lift_pose["shoulder_lift"] = 0.25
        lift_pose["elbow_flex"] = 0.15
        self.interpolate_and_send(lift_pose, duration_s=1.2)
        time.sleep(0.5)

        # 下放并松开 (RANGE_0_100: 85% 张开)
        self.interpolate_and_send(gripped_pose, duration_s=1.2)
        open_pose = dict(gripped_pose)
        open_pose["gripper"] = 85.0
        self.interpolate_and_send(open_pose, duration_s=0.8)

        # 平稳收回安全蜷缩休眠位
        logger.info("[*] 抓取表演结束: 正在平缓收回安全蜷缩休眠姿态...")
        self.api.safe_return_to_rest(duration_s=2.0)
        logger.info("[✓] 模拟抓取动线执行完毕.")
        return True
