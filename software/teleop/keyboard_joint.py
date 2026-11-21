#!/usr/bin/env python3
"""SO-ARM101 键盘单关节微调与控制端原型验证工具.

通过 ActionAPI 调度底层从臂, 严格经由安全控制器与状态机保障安全.

按键操作:
    1 / q : shoulder_pan  [反向 / 正向]
    2 / w : shoulder_lift [反向 / 正向]
    3 / e : elbow_flex    [反向 / 正向]
    4 / r : wrist_flex    [反向 / 正向]
    5 / t : wrist_roll    [反向 / 正向]
    6 / y : gripper       [张开 / 闭合]
    + / - : 增大 / 减小微调步长
    m     : 安全平稳回校准中位
    空格  : 紧急停止 (ESTOP LATCHED)
    p     : 人工确认复位 (RESET)
    s     : 打印当前状态详细报告
    ESC / x: 安全退出
"""

from __future__ import annotations

import argparse
import logging
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any

# Ensure software root is on sys.path
_here = Path(__file__).resolve().parent
_root = _here.parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from core.action_api import ActionAPI
from core.data_types import (
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    SystemState,
    deg_to_rad,
    rad_to_deg,
)
from hardware.config import ArmConfig, RobotSystemConfig

# 关节按键映射表: {按键: (关节名, 方向符号)}
KEY_MAPPING: dict[str, tuple[str, float]] = {
    "1": ("shoulder_pan", -1.0),
    "q": ("shoulder_pan", 1.0),
    "2": ("shoulder_lift", -1.0),
    "w": ("shoulder_lift", 1.0),
    "3": ("elbow_flex", -1.0),
    "e": ("elbow_flex", 1.0),
    "4": ("wrist_flex", -1.0),
    "r": ("wrist_flex", 1.0),
    "5": ("wrist_roll", -1.0),
    "t": ("wrist_roll", 1.0),
    "6": ("gripper", -1.0),
    "y": ("gripper", 1.0),
}


class NonBlockingTerminal:
    """Linux 终端非阻塞单字符读取上下文管理器."""

    def __enter__(self) -> NonBlockingTerminal:
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout_s: float = 0.05) -> str | None:
        """非阻塞等待单个按键输入."""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if rlist:
            char = sys.stdin.read(1)
            # 处理 ESC 键序列
            if char == "\x1b":
                return "ESC"
            return char
        return None


def print_banner(step_deg: float) -> None:
    """打印控制面板帮助信息."""
    print("\n" + "=" * 66)
    print("             SO-ARM101 键盘安全控制终端 (ActionAPI 原型)")
    print("=" * 66)
    print(f" 当前单步步长: {step_deg:.1f}° ({deg_to_rad(step_deg):.3f} rad) | [+/-] 调整步长")
    print(" 关节控制映射:")
    print("   1 / q: shoulder_pan [-/+]      2 / w: shoulder_lift [-/+]")
    print("   3 / e: elbow_flex   [-/+]      4 / r: wrist_flex    [-/+]")
    print("   5 / t: wrist_roll   [-/+]      6 / y: gripper       [-/+]")
    print(" 系统控制:")
    print("   [m] 安全回校准中位             [空格] 紧急停止 (ESTOP)")
    print("   [p] 解除急停并复位             [s] 打印系统健康快照")
    print("   [ESC] 或 [x] 安全退出")
    print("=" * 66 + "\n", flush=True)


def format_joint_state_line(state: JointState) -> str:
    """格式化关节当前读数单行显示 (显示为角度，直观易读)."""
    parts = []
    for name in SO101_JOINT_NAMES:
        val = state.positions.get(name, 0.0)
        deg = rad_to_deg(val) if state.unit == JointUnit.RADIAN else val
        parts.append(f"{name[:4]}:{deg:>6.1f}°")
    return " | ".join(parts)


def run_keyboard_teleop(args: argparse.Namespace) -> int:
    """键盘遥操作主循环."""
    arm_cfg = ArmConfig.default_follower(port=args.port)
    sys_cfg = RobotSystemConfig(follower=arm_cfg)
    api = ActionAPI(config=sys_cfg)

    print(f"[*] 正在通过 ActionAPI 连接机械臂 (端口: {args.port})...")
    if not api.connect(connect_leader=False):
        print("[!] 连接失败，请检查端口与权限 (如 sudo chmod 666 /dev/ttyACM0)")
        return 1

    # 申请键盘控制权
    if not api.claim_control(ControlSource.KEYBOARD):
        print("[!] 申请 KEYBOARD 控制权失败，当前状态不允许进入手动遥控")
        api.disconnect()
        return 1

    current_step_deg = args.step_deg
    print_banner(current_step_deg)

    last_refresh_time = 0.0
    refresh_interval = 0.2  # 状态行刷新周期 (秒)

    try:
        with NonBlockingTerminal() as term:
            while True:
                now = time.monotonic()

                # 定时刷新状态栏
                if now - last_refresh_time >= refresh_interval:
                    try:
                        cur_state = api.get_joint_state()
                        status = api.get_status_report()
                        state_str = status["system_state"]
                        line = format_joint_state_line(cur_state)
                        sys.stdout.write(f"\r[{state_str}] {line}   ")
                        sys.stdout.flush()
                        last_refresh_time = now
                    except Exception:
                        pass

                key = term.read_key(timeout_s=0.03)
                if not key:
                    continue

                # 1. 退出按键
                if key in ("ESC", "x", "\x03"):  # ESC or x or Ctrl-C
                    print("\n[*] 接收到退出指令，准备安全关闭...")
                    break

                # 2. 急停触发 (空格键)
                elif key == " ":
                    print("\n[!] >>> 用户拍下空格: 触发紧急停止! <<<")
                    api.request_estop("键盘空格触发急停")
                    continue

                # 3. 复位申请 (p 键)
                elif key == "p":
                    ok, msg = api.request_reset()
                    print(f"\n[*] 申请系统复位: {msg}")
                    if ok:
                        api.claim_control(ControlSource.KEYBOARD)
                    continue

                # 4. 调整步长
                elif key == "+":
                    current_step_deg = min(15.0, current_step_deg + 0.5)
                    print(f"\n[*] 单步步长调整为: {current_step_deg:.1f}°")
                    continue
                elif key == "-":
                    current_step_deg = max(0.2, current_step_deg - 0.5)
                    print(f"\n[*] 单步步长调整为: {current_step_deg:.1f}°")
                    continue

                # 5. 回中位动作
                elif key == "m":
                    print("\n[*] 开始执行平滑回中位...")
                    ok = api.move_to_middle(step_deg=args.step_deg)
                    if ok:
                        print("[*] 回中位顺利完成!")
                    else:
                        print("[!] 回中位未正常结束或被中断")
                    continue

                # 6. 打印状态快照
                elif key == "s":
                    print("\n" + "-" * 50)
                    import pprint
                    pprint.pprint(api.get_status_report())
                    print("-" * 50)
                    continue

                # 7. 关节增量控制
                elif key in KEY_MAPPING:
                    joint_name, direction = KEY_MAPPING[key]
                    delta_rad = direction * deg_to_rad(current_step_deg)

                    action = RobotAction(
                        action_type=ActionType.JOINT_DELTA,
                        source=ControlSource.KEYBOARD,
                        timestamp=time.monotonic(),
                        target_joint_deltas={joint_name: delta_rad},
                        unit=JointUnit.RADIAN,
                    )

                    ok, msg = api.submit_action(action)
                    if not ok:
                        print(f"\n[!] 动作被拒绝: {msg}")

    except KeyboardInterrupt:
        print("\n[*] 检测到键盘中断 (Ctrl-C)")
    finally:
        print("\n[*] 正在释放控制权并关闭设备...")
        api.release_control(ControlSource.KEYBOARD)
        api.disconnect(keep_torque=args.keep_torque)
        print("[*] 系统已安全退出，谢谢使用!")

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SO-ARM101 键盘单关节安全遥控验证工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Follower 串口 (默认 /dev/ttyACM0)")
    parser.add_argument("--step-deg", type=float, default=2.0, help="单次按键增量步长 (度, 默认 2.0°)")
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="退出后保持电机扭矩 (默认释放扭矩)",
    )
    return parser


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    sys.exit(run_keyboard_teleop(args))

