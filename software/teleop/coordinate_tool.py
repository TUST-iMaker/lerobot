#!/usr/bin/env python3
"""SO-ARM101 TCP (末端直角坐标) 与 6 轴关节坐标双向转换与调试工具 (对齐官方 so101_new_calib.urdf).

用途:
1. 【正向计算 (FK)】: 实时读取真实舵机 6 轴角度，计算末端 TCP 空间坐标 (X, Y, Z, Pitch, Roll);
2. 【逆向计算 (IK)】: 给定目标 TCP (X, Y, Z, Pitch)，计算 6 轴关节目标角度，并展示各关节增量;
3. 【方向校准测试 (Direction Calibration)】: 逐个关节微动测试，确认每个舵机正负号对应物理空间的方向 (前/后/上/下).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

# Ensure software root is on sys.path
_here = Path(__file__).resolve().parent
_root = _here.parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from core.action_api import ActionAPI
from core.config_loader import get_config
from core.data_types import (
    ActionType,
    ControlSource,
    JointUnit,
    RobotAction,
    clamp_gripper_pct,
    rad_to_deg,
)
from core.kinematics import KinematicsConfig, SO101Kinematics
from hardware.config import ArmConfig, RobotSystemConfig


def forward_kinematics_detailed(
    joint_positions: dict[str, float],
    cfg: KinematicsConfig | None = None,
) -> dict[str, Any]:
    """基于官方 URDF 运动学链计算各关节与末端 TCP 空间坐标 (gripper 保持 RANGE_0_100 %)."""
    kin = SO101Kinematics(cfg)

    pan = float(joint_positions.get("shoulder_pan", 0.0))
    lift = float(joint_positions.get("shoulder_lift", 0.0))
    elbow = float(joint_positions.get("elbow_flex", 0.0))
    wrist_p = float(joint_positions.get("wrist_flex", 0.0))
    wrist_r = float(joint_positions.get("wrist_roll", 0.0))
    gripper = clamp_gripper_pct(float(joint_positions.get("gripper", 0.0)))

    (X_tcp, Y_tcp, Z_tcp), pitch_rad, roll_rad = kin.forward_kinematics(joint_positions, unit=JointUnit.RADIAN)
    links = kin.forward_kinematics_links(joint_positions, unit=JointUnit.RADIAN)
    R_tcp = math.hypot(X_tcp, Y_tcp)

    def _rz(pt: tuple[float, float, float]) -> tuple[float, float]:
        return (math.hypot(pt[0], pt[1]) * 1000.0, pt[2] * 1000.0)

    return {
        "joints_deg": {
            "pan": math.degrees(pan),
            "lift": math.degrees(lift),
            "elbow": math.degrees(elbow),
            "wrist_p": math.degrees(wrist_p),
            "wrist_r": math.degrees(wrist_r),
            "gripper": gripper,
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
            "roll_deg": math.degrees(roll_rad),
        },
        "links_mm": {
            "shoulder": _rz(links.get("shoulder_lift", (0.0, 0.0, 0.0))),
            "elbow": _rz(links.get("elbow_flex", (0.0, 0.0, 0.0))),
            "wrist": _rz(links.get("wrist_flex", (0.0, 0.0, 0.0))),
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
    """委托官方 URDF 3D 逆运动学求解器计算目标关节角度."""
    kin = SO101Kinematics(cfg)
    return kin.inverse_kinematics(
        target_xyz=(target_x_m, target_y_m, target_z_m),
        current_joint_positions=current_joints_rad,
        desired_pitch_rad=desired_pitch_rad,
        preserve_wrist_roll=float(current_joints_rad.get("wrist_roll", 0.0)),
        preserve_gripper=float(current_joints_rad.get("gripper", 0.0)),
    )


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
                f"5滚:{j['wrist_r']:+6.1f}° | 6爪:{j['gripper']:>5.1f}%"
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
    parser.add_argument("--port", default=None, help="串口号 (默认从全局配置解析)")
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
            "gripper": 22.0,
        }
        res = forward_kinematics_detailed(sample_joints)
        print("1. 正向计算 (FK):")
        print("   6 轴输入 (arm deg, gripper %):", res["joints_deg"])
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

    target_port = args.port or get_config().follower.port
    if args.mock:
        cfg = RobotSystemConfig.mock_system()
    else:
        cfg = RobotSystemConfig(follower=ArmConfig.default_follower(port=target_port))

    api = ActionAPI(config=cfg)
    if not api.connect(connect_leader=False):
        print(f"[!] 无法连接机械臂 (端口: {target_port})")
        return 1

    if args.free:
        if api.hardware.disable_torque():
            print("\n[*] 舵机扭矩已成功释放! 机械臂无电机阻力，您可以直接用手轻推各个关节观察读数。")
        else:
            print("[!] 释放扭矩失败")

    try:
        run_monitor_mode(api)
    finally:
        api.disconnect(keep_torque=not args.free)
    return 0


if __name__ == "__main__":
    sys.exit(main())
