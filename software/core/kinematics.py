"""SO-ARM101 机械臂正逆运动学解析解计算模块.

基于 SO-ARM101 (SO-100 / SO-101) 5 自由度串联机械臂的几何特性:
1. 底座旋转 (shoulder_pan): 绕竖直 Z 轴旋转 (偏航角 Yaw);
2. 臂平面 3 轴俯仰 (shoulder_lift, elbow_flex, wrist_flex): 确定末端垂直平面的位置与俯仰角 (Pitch);
3. 腕部旋转 (wrist_roll): 确定手腕滚转角 (Roll);
4. 夹爪 (gripper): 不参与连杆位姿求解.

提供极低延迟的解析几何解 (微秒级响应), 内置舵机转向符号对齐与自适应姿态求解 (Adaptive Pitch).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

from core.data_types import (
    SO101_JOINT_NAMES,
    JointState,
    JointUnit,
    Pose3D,
    deg_to_rad,
    rad_to_deg,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class KinematicsConfig:
    """SO-ARM101 机械臂连杆几何尺寸 (单位: 米) 与舵机方向映射."""
    # L1: 底座安装面到 shoulder_lift 俯仰轴的竖直距离
    base_height_m: float = 0.110
    # L2: shoulder_lift 轴到 elbow_flex 轴的大臂连杆长 (实测 13.5cm)
    upper_arm_m: float = 0.135
    # L3: elbow_flex 轴到 wrist_flex 轴的小臂连杆长 (实测 12.5cm)
    forearm_m: float = 0.125
    # L4: wrist_flex 轴到夹爪指尖工具中心点 (TCP) 的有效长度 (实测腕部 7.1cm)
    gripper_offset_m: float = 0.071

    # 关节传感器读数 -> 几何坐标系的符号对齐系数 (中位体系下统一为正向对齐)
    sign_shoulder_pan: float = 1.0
    sign_shoulder_lift: float = 1.0
    sign_elbow_flex: float = 1.0
    sign_wrist_flex: float = 1.0
    sign_wrist_roll: float = 1.0


class SO101Kinematics:
    """SO-ARM101 运动学解析解求解器."""

    def __init__(self, config: KinematicsConfig | None = None) -> None:
        self.config = config or KinematicsConfig()

    def forward_kinematics(
        self,
        joint_positions: dict[str, float],
        unit: JointUnit = JointUnit.RADIAN,
    ) -> tuple[tuple[float, float, float], float, float]:
        """正运动学计算 (FK): 根据各关节读数计算末端 TCP 在底座坐标系下的坐标与姿态.
        
        Args:
            joint_positions: 关节位置字典 {joint_name: value}
            unit: 关节位置数值单位 (RADIAN 或 DEGREE)
            
        Returns:
            ((x, y, z), pitch_rad, roll_rad)
            - x, y, z: 末端在底座坐标系下的位置 (米), x为前, y为左, z为上
            - pitch_rad: 末端在垂直平面内的绝对俯仰角 (弧度)
            - roll_rad: 手腕滚转角 (弧度)
        """
        if unit == JointUnit.DEGREE:
            raw_pan = deg_to_rad(joint_positions.get("shoulder_pan", 0.0))
            raw_lift = deg_to_rad(joint_positions.get("shoulder_lift", 0.0))
            raw_elbow = deg_to_rad(joint_positions.get("elbow_flex", 0.0))
            raw_wrist_p = deg_to_rad(joint_positions.get("wrist_flex", 0.0))
            raw_wrist_r = deg_to_rad(joint_positions.get("wrist_roll", 0.0))
        else:
            raw_pan = float(joint_positions.get("shoulder_pan", 0.0))
            raw_lift = float(joint_positions.get("shoulder_lift", 0.0))
            raw_elbow = float(joint_positions.get("elbow_flex", 0.0))
            raw_wrist_p = float(joint_positions.get("wrist_flex", 0.0))
            raw_wrist_r = float(joint_positions.get("wrist_roll", 0.0))

        # 映射至几何坐标系 (前倾为正)
        pan = self.config.sign_shoulder_pan * raw_pan
        lift = self.config.sign_shoulder_lift * raw_lift
        elbow = self.config.sign_elbow_flex * raw_elbow
        wrist_p = self.config.sign_wrist_flex * raw_wrist_p
        wrist_r = self.config.sign_wrist_roll * raw_wrist_r

        L1 = self.config.base_height_m
        L2 = self.config.upper_arm_m
        L3 = self.config.forearm_m
        L4 = self.config.gripper_offset_m

        # 绝对俯仰角累计 (以物理中位为基准: lift=0 大臂垂直向上; elbow=0 小臂水平向前; wrist_p=0 手腕水平向前)
        phi2 = lift
        phi3 = phi2 + math.pi / 2.0 + elbow
        phi4 = phi3 + wrist_p

        # 平面内各关节位置 (R 为径向水平距离, Z 为高度)
        # 大臂末端 (肘部)
        R2 = L2 * math.sin(phi2)
        Z2 = L1 + L2 * math.cos(phi2)

        # 小臂末端 (腕部)
        R3 = R2 + L3 * math.sin(phi3)
        Z3 = Z2 + L3 * math.cos(phi3)

        # 末端工具中心点 (TCP)
        R4 = R3 + L4 * math.sin(phi4)
        Z4 = Z3 + L4 * math.cos(phi4)

        # 结合底座偏航角 pan 投影至三维空间
        X = R4 * math.cos(pan)
        Y = R4 * math.sin(pan)
        Z = Z4

        # pitch_rad 相对水平面的末端俯仰角 (0 为水平向前)
        pitch_rad = phi4 - math.pi / 2.0
        return (X, Y, Z), pitch_rad, wrist_r

    def compute_tcp_pose(self, state: JointState) -> Pose3D:
        """根据当前 JointState 生成标准 Pose3D 对象."""
        (x, y, z), pitch, roll = self.forward_kinematics(
            joint_positions=state.positions,
            unit=state.unit,
        )
        pan = state.positions.get("shoulder_pan", 0.0)
        yaw = deg_to_rad(pan) if state.unit == JointUnit.DEGREE else pan

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return Pose3D(
            timestamp=state.timestamp,
            frame_id="robot_base",
            position_m=(round(x, 4), round(y, 4), round(z, 4)),
            orientation_xyzw=(round(qx, 4), round(qy, 4), round(qz, 4), round(qw, 4)),
            valid=True,
            kinematics_version="so101_analytical_v2",
        )

    def inverse_kinematics(
        self,
        target_xyz: tuple[float, float, float],
        current_joint_positions: dict[str, float] | None = None,
        desired_pitch_rad: float | None = None,
        preserve_wrist_roll: float = 0.0,
        preserve_gripper: float = 0.0,
    ) -> tuple[bool, dict[str, float] | None, str]:
        """逆运动学计算 (IK): 给定目标空间笛卡尔坐标 (x, y, z)，反求各舵机关节目标角度.
        
        具备自适应俯仰优化 (Adaptive Pitch): 当固定姿态导致超出臂展时，自动微调手腕俯仰以保证末端到达.
        """
        x, y, z = target_xyz
        L1 = self.config.base_height_m
        L2 = self.config.upper_arm_m
        L3 = self.config.forearm_m
        L4 = self.config.gripper_offset_m

        # 1. 底座旋转角
        dist_xy = math.hypot(x, y)
        if dist_xy < 0.01:
            pan_geom = 0.0
        else:
            pan_geom = math.atan2(y, x)

        # 2. 初始候选俯仰角 (0 为水平向前)
        if desired_pitch_rad is not None:
            init_pitch = desired_pitch_rad
        elif current_joint_positions:
            _, init_pitch, _ = self.forward_kinematics(current_joint_positions)
        else:
            init_pitch = 0.0

        # 3. 寻找可达解 (尝试候选 pitch，若超限则自适应搜索最可达 pitch)
        candidate_pitches = [init_pitch]
        # 自适应调整序列 (小范围偏转手腕寻找工作区间)
        for offset_deg in (5, -5, 10, -10, 15, -15, 20, -20, 30, -30, 45, -45):
            candidate_pitches.append(init_pitch + math.radians(offset_deg))

        solved = False
        best_lift = 0.0
        best_elbow = 0.0
        best_wrist_p = 0.0
        err_msg = ""

        max_reach = L2 + L3 - 0.002  # 保留 2mm 裕量防止奇异
        min_reach = abs(L2 - L3) + 0.005

        for pitch_test in candidate_pitches:
            phi4 = pitch_test + math.pi / 2.0
            # 手腕中心位置 (Rw 为手腕径向距离, Zw 为手腕高度)
            Rw = dist_xy - L4 * math.sin(phi4)
            Zw = z - L4 * math.cos(phi4)

            delta_r = Rw
            delta_z = Zw - L1
            D_sq = delta_r * delta_r + delta_z * delta_z
            D = math.sqrt(D_sq)

            if D > max_reach:
                err_msg = f"目标超出最大臂展 (手腕距离 {D:.3f}m > 连杆和 {max_reach:.3f}m)"
                continue
            if D < min_reach:
                err_msg = f"目标处于死区 (手腕距离 {D:.3f}m < 连杆差 {min_reach:.3f}m)"
                continue

            # 两连杆余弦反解: gamma 为大臂与小臂夹角
            cos_gamma = (L2 * L2 + L3 * L3 - D_sq) / (2.0 * L2 * L3)
            cos_gamma = max(-1.0, min(1.0, cos_gamma))
            gamma = math.acos(cos_gamma)
            # 中位定义为 90 度 (gamma = pi/2) 时 elbow = 0
            elbow_geom = math.pi / 2.0 - gamma

            alpha = math.atan2(delta_r, delta_z)
            cos_beta = (L2 * L2 + D_sq - L3 * L3) / (2.0 * L2 * D)
            cos_beta = max(-1.0, min(1.0, cos_beta))
            beta = math.acos(cos_beta)
            lift_geom = alpha - beta

            # phi4 = lift_geom + pi/2 + elbow_geom + wrist_p_geom
            wrist_p_geom = phi4 - (lift_geom + math.pi / 2.0 + elbow_geom)

            best_lift = lift_geom
            best_elbow = elbow_geom
            best_wrist_p = wrist_p_geom
            solved = True
            break

        if not solved:
            return False, None, err_msg

        # 4. 从几何角度反向映射回真实舵机读数
        def wrap_angle(a: float) -> float:
            return (a + math.pi) % (2.0 * math.pi) - math.pi

        raw_pan = wrap_angle(pan_geom / self.config.sign_shoulder_pan)
        raw_lift = wrap_angle(best_lift / self.config.sign_shoulder_lift)
        raw_elbow = wrap_angle(best_elbow / self.config.sign_elbow_flex)
        raw_wrist_p = wrap_angle(best_wrist_p / self.config.sign_wrist_flex)
        raw_wrist_r = wrap_angle(preserve_wrist_roll / self.config.sign_wrist_roll)

        joint_targets: dict[str, float] = {
            "shoulder_pan": raw_pan,
            "shoulder_lift": raw_lift,
            "elbow_flex": raw_elbow,
            "wrist_flex": raw_wrist_p,
            "wrist_roll": raw_wrist_r,
            "gripper": preserve_gripper,
        }

        return True, joint_targets, "逆运动学自适应求解成功"


def solve_horizontal_planar_reach(
    target_R_gripper: float,
    locked_Z_gripper: float,
    kin_cfg: KinematicsConfig | None = None,
    sign_lift: float = 1.0,
    sign_elbow: float = 1.0,
    sign_wrist: float = 1.0,
) -> tuple[bool, float, float, float, str]:
    """严格锁定夹爪指尖高度 locked_Z_gripper 恒定的水平平推逆解.
    
    几何解析:
    - 夹爪在水平面上保持水平朝向;
    - 手腕高度: Z_wrist = locked_Z_gripper (夹爪水平时手腕与夹爪处于同一高度);
    - 手腕水平距离: R_wrist = target_R_gripper - L4 (手腕在指尖正后方 L4 处);
    - 大臂小臂由标准两连杆余弦定理精确解析求解.
    """
    cfg = kin_cfg or KinematicsConfig()
    L1 = cfg.base_height_m
    L2 = cfg.upper_arm_m
    L3 = cfg.forearm_m
    L4 = cfg.gripper_offset_m

    R_wrist = target_R_gripper - L4
    Z_wrist = locked_Z_gripper

    # 手腕相对于肩部的高度差 (恒定常数)
    h = Z_wrist - L1
    D_sq = R_wrist * R_wrist + h * h
    D = math.sqrt(D_sq)

    max_reach = L2 + L3 - 0.003
    min_reach = abs(L2 - L3) + 0.015

    if D > max_reach:
        return False, 0.0, 0.0, 0.0, f"超出最大臂展 (手腕距肩 {D*1000:.1f}mm > 连杆极限 {max_reach*1000:.1f}mm)"
    if D < min_reach:
        return False, 0.0, 0.0, 0.0, f"进入内侧死区 (手腕距肩 {D*1000:.1f}mm < 最小极限 {min_reach*1000:.1f}mm)"

    # 余弦定理求解大臂小臂
    cos_gamma = (L2 * L2 + L3 * L3 - D_sq) / (2.0 * L2 * L3)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    # 中位 gamma=pi/2 时 geom_elbow = 0
    geom_elbow = math.pi / 2.0 - gamma

    cos_beta = (L2 * L2 + D_sq - L3 * L3) / (2.0 * L2 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    psi = math.atan2(R_wrist, h)
    geom_lift = psi - beta

    # 手腕姿态自动水平补偿: 保证夹爪在水平面上绝对水平 (pitch=0)
    # phi4 = geom_lift + pi/2 + geom_elbow + geom_wrist = pi/2
    geom_wrist = - (geom_lift + geom_elbow)

    # 映射回舵机物理读数
    raw_lift = sign_lift * geom_lift
    raw_elbow = sign_elbow * geom_elbow
    raw_wrist = sign_wrist * geom_wrist

    return True, raw_lift, raw_elbow, raw_wrist, "求解成功"

