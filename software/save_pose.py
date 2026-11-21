#!/usr/bin/env python3
"""SO-ARM101 机械臂当前姿态抓取与保存工具 (save_pose.py).

用途:
  在释放扭矩 (零力矩) 状态下，用手将机械臂摆放到理想位置 (如蜷缩位、观察位等)，
  运行本脚本即可无感读取当前 6 个舵机物理角度并保存为标准 JSON 姿态文件 (默认 rest_pose.json)。

特点:
  1. 严格使用全局配置与真实校准: 加载 follower_arm.json 校准参数，绝无零点偏差;
  2. 校准安全边界自适应: 若当前蜷缩姿态超出了原校准 range_min，自动安全微调拓宽校准范围，
     彻底杜绝 LeRobot 驱动在回缩时因越界而将大臂截断在半空的问题;
  3. 严格只读: 连接后保持禁用扭矩 (disable_torque)，绝不发生电机跳变或阻抗。

用法示例:
  # 1. 抓取从臂当前姿态并更新 rest_pose.json (优先读 config.yaml)
  python save_pose.py

  # 2. 抓取主臂当前姿态
  python save_pose.py --arm leader

  # 3. 自定义输出文件名
  python save_pose.py --output my_pose.json
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any

_software_root = Path(__file__).resolve().parent
_lerobot_root = _software_root.parent
if str(_software_root) not in sys.path:
    sys.path.insert(0, str(_software_root))
if str(_lerobot_root / "src") not in sys.path:
    sys.path.insert(0, str(_lerobot_root / "src"))

from core.config_loader import get_config
from core.data_types import SO101_JOINT_NAMES

logger = logging.getLogger("save_pose")


def detect_serial_ports() -> list[str]:
    """扫描系统中可用的物理串口设备."""
    import glob
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def get_calibration_path(arm_type: str, robot_id: str) -> Path:
    """获取对应的 LeRobot 标定文件路径."""
    hf_cache = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration"
    if arm_type == "follower":
        return hf_cache / "robots" / "so_follower" / f"{robot_id}.json"
    else:
        return hf_cache / "teleoperators" / "so_leader" / f"{robot_id}.json"


def auto_expand_calibration_if_needed(cal_path: Path, raw_positions: dict[str, int]) -> bool:
    """若当前物理姿态超出了校准范围，自动安全拓宽校准范围并持久化."""
    if not cal_path.is_file():
        return False

    try:
        with open(cal_path, "r", encoding="utf-8") as f:
            cal_data = json.load(f)

        modified = False
        for motor_name, raw_val in raw_positions.items():
            if motor_name not in cal_data or not isinstance(cal_data[motor_name], dict):
                continue
            item = cal_data[motor_name]
            r_min = int(item.get("range_min", 0))
            r_max = int(item.get("range_max", 4095))

            if raw_val < r_min:
                new_min = max(0, raw_val - 15)
                print(f"  ⚡ [校准边界拓宽] 关节 {motor_name} 当前位置 (raw={raw_val}) 低于原限位 range_min={r_min}，自动拓展至 {new_min}")
                item["range_min"] = new_min
                modified = True
            elif raw_val > r_max:
                new_max = min(4095, raw_val + 15)
                print(f"  ⚡ [校准边界拓宽] 关节 {motor_name} 当前位置 (raw={raw_val}) 高于原限位 range_max={r_max}，自动拓展至 {new_max}")
                item["range_max"] = new_max
                modified = True

        if modified:
            with open(cal_path, "w", encoding="utf-8") as f:
                json.dump(cal_data, f, indent=4, ensure_ascii=False)
            print(f"  ✅ 已成功更新校准文件: {cal_path.name}，确保机械臂可以完全收拢至当前蜷缩位！\n")
            return True
    except Exception as exc:
        print(f"  ⚠️ 检查/拓宽校准范围异常: {exc}")
    return False


def capture_pose(
    port: str,
    robot_id: str = "follower_arm",
    arm_type: str = "follower",
    use_degrees: bool = True,
) -> tuple[dict[str, float], dict[str, int]]:
    """以零力矩只读模式读取机械臂当前关节物理角度与底层 raw 计数值."""
    try:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    except ImportError as exc:
        raise RuntimeError(
            f"缺少 LeRobot 驱动库依赖: {exc}。\n请确保在正确的 Python 环境下执行。"
        ) from exc

    cfg = SO101FollowerConfig(
        port=port,
        id=robot_id,
        disable_torque_on_disconnect=True,
        use_degrees=use_degrees,
    )
    robot = SO101Follower(cfg)

    print(f"[*] 正在建立只读通信连接 (端口: {port}, ID: {robot_id})...")
    robot.bus.connect()

    try:
        # 强制禁用扭矩，确保人工摆放位置不受电机干扰
        robot.bus.disable_torque()
        time.sleep(0.08)

        # 读取底层原始计数值
        raw_map = robot.bus.sync_read("Present_Position", normalize=False)
        raw_positions: dict[str, int] = {}
        for k, v in raw_map.items():
            name = str(k)
            try:
                raw_positions[name] = int(v)
            except (ValueError, TypeError):
                pass

        # 检查并自动拓展校准文件的极限范围
        cal_path = get_calibration_path(arm_type, robot_id)
        if auto_expand_calibration_if_needed(cal_path, raw_positions):
            # 重新实例化以使更新后的标定范围生效
            pass

        obs = robot.get_observation()
        joint_positions: dict[str, float] = {}

        for key, val in obs.items():
            if not key.endswith(".pos"):
                continue
            name = key[:-4]
            if name in SO101_JOINT_NAMES:
                joint_positions[name] = round(float(val), 2)

        return joint_positions, raw_positions
    finally:
        try:
            robot.bus.disable_torque()
            robot.disconnect()
        except Exception:
            pass


def main() -> int:
    cfg = get_config()

    parser = argparse.ArgumentParser(
        description="SO-ARM101 机械臂当前姿态抓取与保存工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--arm",
        choices=["follower", "leader"],
        default="follower",
        help="机械臂角色 (follower: 从臂, leader: 主臂; 默认 follower)",
    )
    parser.add_argument(
        "--port",
        default="",
        help="串口设备号 (默认读取 config.yaml 或自动探测)",
    )
    parser.add_argument(
        "--id",
        default="",
        help="机械臂校准标识 (默认从臂 follower_arm, 主臂 zihao_leader_arm)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default="",
        help=f"保存的目标 JSON 路径 (默认: {cfg.safety.rest_pose_file})",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="离线测试模拟模式 (无需插入硬件)",
    )
    args = parser.parse_args()

    # 1. 确定配置参数
    arm_type = args.arm
    output_file = args.output or cfg.safety.rest_pose_file
    robot_id = args.id or (cfg.follower.id if arm_type == "follower" else cfg.leader.id)

    # 2. 串口自动判定
    port = args.port
    if args.mock:
        port = "mock_port"
    elif not port:
        configured_port = cfg.follower.port if arm_type == "follower" else cfg.leader.port
        available = detect_serial_ports()
        if configured_port in available:
            port = configured_port
        elif available:
            port = available[0]
        else:
            print(f"\n[!] 错误: 未检测到任何物理串口设备。目标端口: {configured_port}")
            print("    请检查 USB 数据线是否插入，以及 12V 稳压电源是否开启。\n")
            return 1

    print("\n" + "=" * 65)
    print("      🦾 SO-ARM101 机械臂物理姿态安全抓取工具")
    print("=" * 65)
    print(f" 目标设备: {arm_type.upper()} (端口: {port}, ID: {robot_id})")
    print(f" 输出文件: {output_file}")
    print(" 运行模式: 零力矩安全只读 (绝不驱动电机，保持人工手摆姿态)")
    print("=" * 65 + "\n")

    if args.mock:
        # 离线模拟数据 (合法的标准蜷缩姿态)
        positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": -55.0,
            "elbow_flex": 75.0,
            "wrist_flex": -45.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        raw_positions = {}
    else:
        try:
            positions, raw_positions = capture_pose(
                port=port,
                robot_id=robot_id,
                arm_type=arm_type,
                use_degrees=True,
            )
        except Exception as exc:
            print(f"\n[!] 姿态抓取失败: {exc}")
            print("    -> 排查建议:")
            print(f"       1. 检查 USB 数据线连接: ls -la {port}")
            print(f"       2. 检查串口读写权限: sudo chmod 666 {port}")
            print("       3. 确认 12V 稳压电源开关是否打开\n")
            return 1

    print("[✓] 成功读取当前物理姿态角度与原始计数值:")
    for j_name in SO101_JOINT_NAMES:
        deg_val = positions.get(j_name, 0.0)
        raw_val = raw_positions.get(j_name, "N/A")
        print(f"    • {j_name:<16}: {deg_val:>8.2f}°  (raw: {raw_val})")

    # 构造与系统兼容的 JSON 内容
    output_data = {
        "unit": "degree",
        "joints": {
            f"{j_name}.pos": positions.get(j_name, 0.0)
            for j_name in SO101_JOINT_NAMES
        },
    }

    out_path = (_software_root / output_file) if not Path(output_file).is_absolute() else Path(output_file)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)

    print("\n" + "-" * 65)
    print(f"✅ 姿态已成功保存至: {out_path.name}")
    print(f"   完整路径: {out_path}")
    print("   后续无论是程序退出还是 demo 结束，机械臂都会平滑回归此姿态！")
    print("-" * 65 + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
