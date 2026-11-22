#!/usr/bin/env python3
"""SO-ARM101 末端直角坐标 (Cartesian/TCP) 键盘控制正式工具.

核心特性:
1. 水平面移动 (WASD):
   - [a] / [d]: 仅驱动 1 号底座舵机左右旋转, 高度与大臂完全静止;
   - [w] / [s]: 严格在夹爪所在的水平面内前后平推 (前伸/收缩), 夹爪高度 Z 绝对锁定不变;
2. 垂直升降 (R / F):
   - [r] / [f]: 上升 / 下降 (调整当前工作水平面高度 Z);
3. 姿态与夹爪微调:
   - [j] / [k]: 手腕俯仰 Pitch (上翘 / 下俯);
   - [u] / [i]: 手腕旋转 Roll  (逆时针 / 顺时针);
   - [c] / [v]: 夹爪 Gripper   (张开 / 闭合);
4. 系统安全与辅助:
   - [m]: 平稳回校准中位并重设基准水平面;
   - [空格]: 紧急停止 (ESTOP LATCHED);
   - [p]: 显式人工复位 (RESET);
   - [+/-]: 增大 / 减小移动步幅 (0.5 mm ~ 25 mm);
   - [ESC] / [x]: 安全释放扭矩退出.
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import time
from pathlib import Path
from typing import Any

try:
    import termios
    import tty
except ImportError:
    termios = None  # type: ignore[assignment]
    tty = None  # type: ignore[assignment]

# Ensure software root is on sys.path
_here = Path(__file__).resolve().parent
_root = _here.parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from core.action_api import ActionAPI
from core.config_loader import get_config
from core.data_types import (
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    clamp_gripper_pct,
    deg_to_rad,
    rad_to_deg,
)
from core.kinematics import (
    KinematicsConfig,
    SO101Kinematics,
    solve_horizontal_planar_reach,
)
from hardware.config import ArmConfig, RobotSystemConfig
from teleop.coordinate_tool import (
    forward_kinematics_detailed,
    inverse_kinematics_deterministic,
)


class NonBlockingTerminal:
    """跨平台终端无缓冲非阻塞读取上下文管理器 (Linux termios / Windows msvcrt)."""

    def __enter__(self) -> NonBlockingTerminal:
        self.old_settings = None
        if termios is not None and tty is not None and sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        if termios is not None and self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float = 0.05) -> str | None:
        if sys.platform == "win32":
            import msvcrt

            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if msvcrt.kbhit():
                    char = msvcrt.getwch()
                    return "ESC" if char == "\x1b" else char
                time.sleep(0.01)
            return None
        rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if rlist:
            char = sys.stdin.read(1)
            if char == "\x1b":
                return "ESC"
            return char
        return None


def print_banner(step_mm: float) -> None:
    print("\n" + "=" * 70)
    print("        SO-ARM101 末端直角坐标 (WASD) 键盘安全控制正式终端")
    print("=" * 70)
    print(f" 当前平推步长: {step_mm:.1f} mm  |  [+/-] 调节单步步长 (0.5mm ~ 25mm)")
    print(" 水平面移动 (世界坐标系):")
    print("   [w] / [s] : 前进 (+X) / 后退 (-X) (夹爪高度严格恒定, 水平平推)")
    print("   [a] / [d] : 向左 (+Y) / 向右 (-Y) (仅驱动 1 号底座舵机在水平面左右转)")
    print(" 垂直升降与姿态微调:")
    print("   [r] / [f] : 上升 (+Z) / 下降 (-Z) (大臂与小臂协同联动, 机械臂整体升降)")
    print("   [j] / [k] : 俯仰 Pitch (上翘 / 下俯)")
    print("   [u] / [i] : 旋转 Roll  (逆时针 / 顺时针)")
    print("   [c] / [v] : 夹爪 Gripper (张开 / 闭合)")
    print(" 系统控制:")
    print("   [m] 回校准中位并重设基准面      [空格] 紧急停止 (ESTOP)")
    print("   [p] 解除急停复位                [ESC] / [x] 安全释放扭矩退出")
    print("=" * 70 + "\n", flush=True)


def run_cartesian_teleop(args: argparse.Namespace) -> int:
    is_mock = args.mock or args.port == "mock"
    target_port = args.port or get_config().follower.port
    if is_mock:
        arm_cfg = ArmConfig.mock_follower()
        sys_cfg = RobotSystemConfig.mock_system()
        print("[*] 正在以【纯软件仿真模式】启动机械臂 (无需串口与真机)...")
    else:
        arm_cfg = ArmConfig.default_follower(port=target_port)
        sys_cfg = RobotSystemConfig(follower=arm_cfg)
        print(f"[*] 正在通过 ActionAPI 连接机械臂 (端口: {target_port})...")

    api = ActionAPI(config=sys_cfg)
    kin = SO101Kinematics()

    if not api.connect(connect_leader=False):
        print("[!] 机械臂连接失败，请检查端口与配置。")
        return 1

    if not api.claim_control(ControlSource.KEYBOARD):
        print("[!] 申请 KEYBOARD 控制权失败")
        api.disconnect()
        return 1

    # 转向符号对齐官方 URDF shoulder_pan 世界轴 [0, 0, -1]:
    # a (+Y 左移) 需减小 shoulder_pan (-d_pan), d (-Y 右移) 需增大 shoulder_pan (+d_pan),
    # 与 pan_dir = (+1 if a else -1) 相乘故默认为 -1.0 (--invert-ad 可反转).
    sign_ad = 1.0 if args.invert_ad else -1.0
    sign_lift = 1.0
    sign_elbow = 1.0
    sign_wrist = 1.0

    step_mm = args.step_mm
    step_pan_deg = 2.0  # A/D 每次微动旋转度数

    print_banner(step_mm)

    # 就位时原位保持不动 (直接以当前真实姿态为就绪基准，绝不执行额外移动跳变)
    cur_state = api.get_joint_state()
    fk_info = forward_kinematics_detailed(cur_state.positions)
    current_R_gripper = fk_info["tcp"]["r_mm"] / 1000.0
    locked_Z_gripper = fk_info["tcp"]["z_mm"] / 1000.0
    locked_pitch = fk_info["tcp"]["pitch_rad"]
    cur_gripper_val = cur_state.positions.get("gripper", 0.0)
    print(
        f"[*] 原位保持就绪: 当前基准高度 Z={locked_Z_gripper*1000:.1f}mm | "
        f"工作半径 R={current_R_gripper*1000:.1f}mm | "
        f"夹爪俯角 Pitch={math.degrees(locked_pitch):.1f}°\n"
    )

    try:
        with NonBlockingTerminal() as term:
            while True:
                key = term.read_key(timeout_s=0.03)
                if not key:
                    continue

                # 1. 退出按键
                if key in ("ESC", "x", "\x03"):
                    print("\n[*] 接收到退出指令，准备安全关闭...")
                    break

                # 2. 空格急停
                elif key == " ":
                    print("\n[!] >>> 用户拍下空格: 紧急停止! <<<")
                    api.request_estop("键盘空格触发急停")
                    continue

                # 3. 复位申请
                elif key == "p":
                    ok, msg = api.request_reset()
                    print(f"\n[*] 申请系统复位: {msg}")
                    if ok:
                        api.claim_control(ControlSource.KEYBOARD)
                    continue

                # 4. 调整平移步长 (+/-)
                elif key == "+":
                    step_mm = min(25.0, step_mm + 0.5)
                    step_pan_deg = min(5.0, step_pan_deg + 0.2)
                    print(f"\n[*] 平推步长: {step_mm:.1f} mm | 旋转步长: {step_pan_deg:.1f}°")
                    continue
                elif key == "-":
                    step_mm = max(0.5, step_mm - 0.5)
                    step_pan_deg = max(0.5, step_pan_deg - 0.2)
                    print(f"\n[*] 平推步长: {step_mm:.1f} mm | 旋转步长: {step_pan_deg:.1f}°")
                    continue

                # 5. 回校准中位 (m)
                elif key == "m":
                    print("\n[*] 正在回校准中位...")
                    api.move_to_middle()
                    time.sleep(1.0)
                    cur_state = api.get_joint_state()
                    fk_info = forward_kinematics_detailed(cur_state.positions)
                    current_R_gripper = fk_info["tcp"]["r_mm"] / 1000.0
                    locked_Z_gripper = fk_info["tcp"]["z_mm"] / 1000.0
                    locked_pitch = fk_info["tcp"]["pitch_rad"]
                    print(
                        f"[*] 回中位成功! 当前基准高度: Z={locked_Z_gripper*1000:.1f}mm | "
                        f"半径 R={current_R_gripper*1000:.1f}mm | 俯角 Pitch={math.degrees(locked_pitch):.1f}°\n"
                    )
                    continue

                cur_state = api.get_joint_state()
                cur_positions = dict(cur_state.positions)

                # =============================================================
                # A / D 控制: 仅 1 号底座舵机左右平转 (高度完全不变)
                # =============================================================
                if key in ("a", "d"):
                    pan_dir = 1.0 if key == "a" else -1.0
                    d_pan_rad = sign_ad * pan_dir * deg_to_rad(step_pan_deg)
                    target_pan = cur_positions.get("shoulder_pan", 0.0) + d_pan_rad

                    # 严格仅下发 1 号舵机，确保在水平面上左右平转，其余舵机不下发指令以防受传感器噪声干扰
                    target_joints = {"shoulder_pan": target_pan}

                    action = RobotAction(
                        action_type=ActionType.JOINT_POSITION,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_positions=target_joints,
                        unit=JointUnit.RADIAN,
                    )
                    sub_ok, sub_msg = api.submit_action(action)
                    if not sub_ok:
                        print(f"\n[!] 动作安全拦截: {sub_msg}")
                    continue

                # =============================================================
                # W / S 控制: 沿当前 URDF 径向平面水平前后平推 (高度与俯角严格锁定)
                # =============================================================
                elif key in ("w", "s"):
                    ws_dir = 1.0 if key == "w" else -1.0
                    step_m = (step_mm / 1000.0) * ws_dir

                    fk_cur = forward_kinematics_detailed(cur_positions)
                    cur_x_m = fk_cur["tcp"]["x_mm"] / 1000.0
                    cur_y_m = fk_cur["tcp"]["y_mm"] / 1000.0
                    cur_r_m = math.hypot(cur_x_m, cur_y_m)
                    cur_pan = cur_positions.get("shoulder_pan", 0.0)
                    if cur_r_m >= 1e-3:
                        ux, uy = cur_x_m / cur_r_m, cur_y_m / cur_r_m
                    else:
                        # URDF shoulder_pan 轴为 [0, 0, -1], 水平前向单位向量为 (cos(-pan), sin(-pan))
                        ux, uy = math.cos(-cur_pan), math.sin(-cur_pan)

                    target_x = cur_x_m + step_m * ux
                    target_y = cur_y_m + step_m * uy
                    target_R = math.hypot(target_x, target_y)

                    ok, solved, msg = inverse_kinematics_deterministic(
                        target_x_m=target_x,
                        target_y_m=target_y,
                        target_z_m=locked_Z_gripper,
                        desired_pitch_rad=locked_pitch,
                        current_joints_rad=cur_positions,
                    )

                    if not ok or solved is None:
                        continue

                    target_joints = {
                        "shoulder_pan": solved["shoulder_pan"],
                        "shoulder_lift": solved["shoulder_lift"],
                        "elbow_flex": solved["elbow_flex"],
                        "wrist_flex": solved["wrist_flex"],
                    }

                    action = RobotAction(
                        action_type=ActionType.JOINT_POSITION,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_positions=target_joints,
                        unit=JointUnit.RADIAN,
                    )
                    sub_ok, sub_msg = api.submit_action(action)
                    if sub_ok:
                        current_R_gripper = target_R
                        print(
                            f"\r[*] 平推{'前伸 (+W)' if ws_dir > 0 else '后退 (-S)'}: "
                            f"半径 R={current_R_gripper*1000:.1f}mm | 高度 Z={locked_Z_gripper*1000:.1f}mm",
                            end="",
                            flush=True,
                        )
                    else:
                        print(f"\n[!] 动作安全拦截: {sub_msg}")
                    continue

                # =============================================================
                # R / F 控制: 垂直升降 (R整体抬升, F整体下降, 水平位置与俯角严格锁定)
                # =============================================================
                elif key in ("r", "f"):
                    z_dir = 1.0 if key == "r" else -1.0
                    step_m = (step_mm / 1000.0) * z_dir
                    target_Z = locked_Z_gripper + step_m

                    fk_cur = forward_kinematics_detailed(cur_positions)
                    target_x = fk_cur["tcp"]["x_mm"] / 1000.0
                    target_y = fk_cur["tcp"]["y_mm"] / 1000.0

                    ok, solved, msg = inverse_kinematics_deterministic(
                        target_x_m=target_x,
                        target_y_m=target_y,
                        target_z_m=target_Z,
                        desired_pitch_rad=locked_pitch,
                        current_joints_rad=cur_positions,
                    )

                    if not ok or solved is None:
                        continue

                    target_joints = {
                        "shoulder_pan": solved["shoulder_pan"],
                        "shoulder_lift": solved["shoulder_lift"],
                        "elbow_flex": solved["elbow_flex"],
                        "wrist_flex": solved["wrist_flex"],
                    }

                    action = RobotAction(
                        action_type=ActionType.JOINT_POSITION,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_positions=target_joints,
                        unit=JointUnit.RADIAN,
                    )
                    sub_ok, sub_msg = api.submit_action(action)
                    if sub_ok:
                        locked_Z_gripper = target_Z
                        print(
                            f"\r[*] 垂直{'↑抬升 (+R)' if z_dir > 0 else '↓下降 (-F)'}: "
                            f"高度 Z={locked_Z_gripper*1000:.1f}mm | 半径 R={current_R_gripper*1000:.1f}mm",
                            end="",
                            flush=True,
                        )
                    else:
                        print(f"\n[!] 动作安全拦截: {sub_msg}")
                    continue

                # =============================================================
                # 姿态微调与夹爪控制 (J/K/U/I/C/V)
                # =============================================================
                elif key in ("j", "k"):  # 手腕俯仰
                    # 符号对齐: j为上翘(朝上，负方向减小)；k为下俯(朝下，正方向增大)
                    p_dir = -1.0 if key == "j" else 1.0
                    cur_w = cur_positions.get("wrist_flex", 0.0)
                    new_w = cur_w + p_dir * deg_to_rad(3.0)
                    target_joints = {"wrist_flex": new_w}
                    action = RobotAction(
                        action_type=ActionType.JOINT_POSITION,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_positions=target_joints,
                        unit=JointUnit.RADIAN,
                    )
                    sub_ok, _ = api.submit_action(action)
                    if sub_ok:
                        cur_temp = dict(cur_positions)
                        cur_temp["wrist_flex"] = new_w
                        fk_info = forward_kinematics_detailed(cur_temp)
                        locked_pitch = fk_info["tcp"]["pitch_rad"]
                        print(
                            f"\r[*] 手腕俯仰微调: 俯角 Pitch={math.degrees(locked_pitch):.1f}°",
                            end="",
                            flush=True,
                        )
                    continue

                elif key in ("u", "i"):  # 手腕滚转
                    r_dir = 1.0 if key == "i" else -1.0
                    cur_r = cur_positions.get("wrist_roll", 0.0)
                    target_joints = {"wrist_roll": cur_r + r_dir * deg_to_rad(5.0)}
                    action = RobotAction(
                        action_type=ActionType.JOINT_POSITION,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_positions=target_joints,
                        unit=JointUnit.RADIAN,
                    )
                    api.submit_action(action)
                    continue

                elif key in ("c", "v"):  # 夹爪开合 (RANGE_0_100: 0..100%)
                    g_dir = 1.0 if key == "c" else -1.0
                    cur_g = float(cur_positions.get("gripper", cur_gripper_val))
                    target_g = clamp_gripper_pct(cur_g + g_dir * 5.0)
                    target_joints = {"gripper": target_g}
                    action = RobotAction(
                        action_type=ActionType.JOINT_POSITION,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_positions=target_joints,
                        unit=JointUnit.RADIAN,
                        gripper_unit=JointUnit.RANGE_0_100,
                    )
                    sub_ok, _ = api.submit_action(action)
                    if sub_ok:
                        cur_gripper_val = target_g
                    continue

    except KeyboardInterrupt:
        print("\n[*] 检测到中断信号 (Ctrl-C)")
    finally:
        print("\n[*] 正在安全退出并断开设备...")
        api.release_control(ControlSource.KEYBOARD)
        api.disconnect(keep_torque=args.keep_torque)
        print("[*] 退出完成.")

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SO-ARM101 末端直角坐标 (WASD) 键盘控制正式工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port", default=None, help="从臂串口 (默认从全局配置解析)")
    parser.add_argument("--step-mm", type=float, default=5.0, help="单次平移步长 (毫米, 默认 5.0mm)")
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="退出后保持电机扭矩 (默认释放扭矩)",
    )
    parser.add_argument(
        "--invert-ad",
        action="store_true",
        help="反转 A/D 左右转向方向 (默认 A向左, D向右)",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="启用纯软件仿真模式 (无需物理舵机与串口连接)",
    )
    return parser


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    sys.exit(run_cartesian_teleop(args))
