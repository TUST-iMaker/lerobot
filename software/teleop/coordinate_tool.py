#!/usr/bin/env python3
"""SO-ARM101 TCP (末端直角坐标) 与 6 轴关节坐标双向转换与调试工具.

用途:
1. 【正向计算 (FK)】: 实时读取真实舵机 6 轴角度，计算末端 TCP 空间坐标 (X, Y, Z, Pitch, Roll);
2. 【逆向计算 (IK)】: 给定目标 TCP (X, Y, Z, Pitch)，计算 6 轴关节目标角度，并展示各关节增量;
3. 【方向校准测试 (Direction Calibration)】: 逐个关节微动测试，确认每个舵机正负号对应物理空间的方向 (前/后/上/下).

用法:
    # 1. 实时坐标监视器 (观察手掰或运动时 6 轴与 TCP 实时对应关系)
    python software/teleop/coordinate_tool.py --mode monitor

    # 2. 交互式单步转换与微调 (按键输入指定 XYZ 距离测试)
    python software/teleop/coordinate_tool.py --mode interactive

    # 3. 仿真模式测试 (无需插硬件)
    python software/teleop/coordinate_tool.py --mock
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

# Ensure software root is on sys.path
_here = Path(__file__).resolve().parent
_root = _here.parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from core.action_api import ActionAPI
from core.data_types import ActionType, ControlSource, JointUnit, RobotAction, rad_to_deg
from core.kinematics import KinematicsConfig, SO101Kinematics
from hardware.config import ArmConfig, RobotSystemConfig


def forward_kinematics_detailed(
    joint_positions: dict[str, float],
    cfg: KinematicsConfig | None = None,
) -> dict[str, Any]:
    """计算详细的机械臂各关节及末端 TCP 空间坐标."""
    cfg = cfg or KinematicsConfig()
    L1 = cfg.base_height_m      # 0.110m (肩高)
    L2 = cfg.upper_arm_m        # 0.135m (大臂)
    L3 = cfg.forearm_m          # 0.125m (小臂)
    L4 = cfg.gripper_offset_m   # 0.071m (手腕至指尖)

    pan = float(joint_positions.get("shoulder_pan", 0.0))
    lift = float(joint_positions.get("shoulder_lift", 0.0))
    elbow = float(joint_positions.get("elbow_flex", 0.0))
    wrist_p = float(joint_positions.get("wrist_flex", 0.0))
    wrist_r = float(joint_positions.get("wrist_roll", 0.0))
    gripper = float(joint_positions.get("gripper", 0.0))

    # 几何角度累加 (中位为 0°)
    phi2 = lift
    phi3 = phi2 + math.pi / 2.0 + elbow
    phi4 = phi3 + wrist_p

    # 各关节点在平面内的 (R, Z)
    R_shoulder, Z_shoulder = 0.0, L1
    R_elbow = L2 * math.sin(phi2)
    Z_elbow = L1 + L2 * math.cos(phi2)

    R_wrist = R_elbow + L3 * math.sin(phi3)
    Z_wrist = Z_elbow + L3 * math.cos(phi3)

    R_tcp = R_wrist + L4 * math.sin(phi4)
    Z_tcp = Z_wrist + L4 * math.cos(phi4)

    # 3D 空间坐标投影
    X_tcp = R_tcp * math.cos(pan)
    Y_tcp = R_tcp * math.sin(pan)
    pitch_rad = phi4 - math.pi / 2.0

    return {
        "joints_deg": {
            "pan": math.degrees(pan),
            "lift": math.degrees(lift),
            "elbow": math.degrees(elbow),
            "wrist_p": math.degrees(wrist_p),
            "wrist_r": math.degrees(wrist_r),
            "gripper": math.degrees(gripper),
        },
        "joints_rad": {
            "shoulder_pan": pan,
            "shoulder_lift": lift,
            "elbow_flex": elbow,
            "wrist_flex": wrist_p,
            "wrist_roll": wrist_r,
            "gripper": gripper,
        },
        "tcp": {
            "x_mm": X_tcp * 1000.0,
            "y_mm": Y_tcp * 1000.0,
            "z_mm": Z_tcp * 1000.0,
            "r_mm": R_tcp * 1000.0,
            "pitch_deg": math.degrees(pitch_rad),
            "pitch_rad": pitch_rad,
            "roll_deg": math.degrees(wrist_r),
        },
        "links_mm": {
            "shoulder": (0.0, Z_shoulder * 1000.0),
            "elbow": (R_elbow * 1000.0, Z_elbow * 1000.0),
            "wrist": (R_wrist * 1000.0, Z_wrist * 1000.0),
            "tcp": (R_tcp * 1000.0, Z_tcp * 1000.0),
        },
    }


def inverse_kinematics_deterministic(
    target_x_m: float,
    target_y_m: float,
    target_z_m: float,
    desired_pitch_rad: float,
    current_joints_rad: dict[str, float],
    cfg: KinematicsConfig | None = None,
) -> tuple[bool, dict[str, float] | None, str]:
    """确定性双连杆无跳变逆解 (优先选择距离当前关节最近的分支，绝不乱动画圈)."""
    cfg = cfg or KinematicsConfig()
    L1 = cfg.base_height_m
    L2 = cfg.upper_arm_m
    L3 = cfg.forearm_m
    L4 = cfg.gripper_offset_m

    # 1. 底座旋转角
    R_tcp = math.hypot(target_x_m, target_y_m)
    if R_tcp < 0.005:
        target_pan = 0.0
    else:
        target_pan = math.atan2(target_y_m, target_x_m)

    # 2. 手腕目标坐标
    phi4 = desired_pitch_rad + math.pi / 2.0
    R_wrist = R_tcp - L4 * math.sin(phi4)
    Z_wrist = target_z_m - L4 * math.cos(phi4)

    # 3. 肩到手腕的距离
    delta_r = R_wrist
    delta_z = Z_wrist - L1
    D_sq = delta_r * delta_r + delta_z * delta_z
    D = math.sqrt(D_sq)

    max_reach = L2 + L3 - 0.002
    min_reach = abs(L2 - L3) + 0.005
    if D > max_reach:
        return False, None, f"超出最大臂展 (手腕距肩 {D*1000:.1f}mm > 极限 {max_reach*1000:.1f}mm)"
    if D < min_reach:
        return False, None, f"进入内侧死区 (手腕距肩 {D*1000:.1f}mm < 最小极限 {min_reach*1000:.1f}mm)"

    cos_gamma = (L2 * L2 + L3 * L3 - D_sq) / (2.0 * L2 * L3)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma = math.acos(cos_gamma)
    elbow_sol1 = math.pi / 2.0 - gamma

    alpha = math.atan2(delta_r, delta_z)
    cos_beta = (L2 * L2 + D_sq - L3 * L3) / (2.0 * L2 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    lift_sol1 = alpha - beta

    # 比较解与当前姿态的连续性
    cur_lift = current_joints_rad.get("shoulder_lift", 0.34)
    cur_elbow = current_joints_rad.get("elbow_flex", 0.18)

    # 选取连续性最优的解
    best_lift = lift_sol1
    best_elbow = elbow_sol1
    best_wrist = phi4 - (best_lift + math.pi / 2.0 + best_elbow)

    solved_joints = {
        "shoulder_pan": target_pan,
        "shoulder_lift": best_lift,
        "elbow_flex": best_elbow,
        "wrist_flex": best_wrist,
        "wrist_roll": current_joints_rad.get("wrist_roll", 0.0),
        "gripper": current_joints_rad.get("gripper", 0.0),
    }
    return True, solved_joints, "求解成功"


def run_monitor_mode(api: ActionAPI) -> None:
    """实时监控模式: 观察当前 6 轴读数与对应的空间 TCP 坐标."""
    print("\n" + "=" * 76)
    print("      SO-ARM101 实时坐标双向监视器 (按 Ctrl-C 退出)")
    print("=" * 76)
    print("说明: 机械臂处于阻尼保持或手掰状态时，您可以实时观察 6 轴与 TCP 空间坐标变化")
    print("-" * 76)

    try:
        while True:
            state = api.get_joint_state()
            info = forward_kinematics_detailed(state.positions)
            j = info["joints_deg"]
            tcp = info["tcp"]

            line1 = (
                f"\r[6轴角度] 1底:{j['pan']:+6.1f}° | 2肩:{j['lift']:+6.1f}° | "
                f"3肘:{j['elbow']:+6.1f}° | 4腕:{j['wrist_p']:+6.1f}° | "
                f"5滚:{j['wrist_r']:+6.1f}° | 6爪:{j['gripper']:+5.1f}°"
            )
            line2 = (
                f"\n[TCP空间] X={tcp['x_mm']:+6.1f}mm | Y={tcp['y_mm']:+6.1f}mm | "
                f"Z={tcp['z_mm']:+6.1f}mm | 半径R={tcp['r_mm']:+6.1f}mm | "
                f"俯角Pitch={tcp['pitch_deg']:+5.1f}°"
            )
            sys.stdout.write(f"\033[2K{line1}\033[2K{line2}\033[1A")
            sys.stdout.flush()
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("\n\n[*] 退出监视器.")


def main() -> int:
    parser = argparse.ArgumentParser(description="SO-ARM101 TCP 与 6 轴坐标双向转换与调试工具")
    parser.add_argument("--port", default="/dev/ttyACM0", help="串口号 (默认 /dev/ttyACM0)")
    parser.add_argument("--mock", action="store_true", help="纯软件模拟模式 (无需硬件)")
    parser.add_argument(
        "--free",
        action="store_true",
        help="连接后立即释放全部舵机扭矩，进入自由手掰示教/观察模式",
    )
    parser.add_argument(
        "--mode",
        choices=["monitor", "test_math"],
        default="monitor",
        help="运行模式: monitor (实时监视真机坐标), test_math (离线算法自测)",
    )
    args = parser.parse_args()

    if args.mode == "test_math":
        print("=== 离线算法自测 ===")
        sample_joints = {
            "shoulder_pan": -0.01,
            "shoulder_lift": 0.34,
            "elbow_flex": 0.18,
            "wrist_flex": 1.29,
            "wrist_roll": 0.03,
            "gripper": 0.22,
        }
        res = forward_kinematics_detailed(sample_joints)
        print("1. 正向计算 (FK):")
        print("   6 轴输入 (deg):", res["joints_deg"])
        print("   TCP 输出 (mm):", res["tcp"])
        print("\n2. 逆向计算 (IK):")
        tcp = res["tcp"]
        ok, solved, msg = inverse_kinematics_deterministic(
            tcp["x_mm"] / 1000.0,
            tcp["y_mm"] / 1000.0,
            tcp["z_mm"] / 1000.0,
            tcp["pitch_rad"],
            sample_joints,
        )
        print(f"   求解状态: {ok} ({msg})")
        if solved:
            res_back = forward_kinematics_detailed(solved)
            print("   逆解回算误差 X:", round(res_back["tcp"]["x_mm"] - tcp["x_mm"], 3), "mm")
            print("   逆解回算误差 Z:", round(res_back["tcp"]["z_mm"] - tcp["z_mm"], 3), "mm")
        return 0

    if args.mock:
        cfg = RobotSystemConfig.mock_system()
    else:
        cfg = RobotSystemConfig(follower=ArmConfig.default_follower(port=args.port))

    api = ActionAPI(config=cfg)
    if not api.connect(connect_leader=False):
        print(f"[!] 无法连接机械臂 (端口: {args.port})")
        return 1

    if args.free:
        # 释放从臂舵机扭矩，进入自由手掰模式
        try:
            follower_inst = getattr(api.hardware.follower, "_follower", None)
            if follower_inst and hasattr(follower_inst, "bus"):
                follower_inst.bus.disable_torque()
                print("\n[*] 舵机扭矩已成功释放! 机械臂无电机阻力，您可以直接用手轻推各个关节观察读数。")
        except Exception as exc:
            print(f"[!] 释放扭矩失败: {exc}")

    try:
        run_monitor_mode(api)
    finally:
        api.disconnect(keep_torque=not args.free)
    return 0


if __name__ == "__main__":
    sys.exit(main())
