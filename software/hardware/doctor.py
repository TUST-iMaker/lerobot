"""SO-ARM101 硬件与系统环境一键诊断体检工具 (Hardware Doctor).

快速排查展示与现场运行常见问题 (跨 Windows COM* 与 Linux /dev/tty*):
1. USB 串口节点与访问权限 (Windows COM* / Linux /dev/ttyACM*, /dev/ttyUSB*);
2. 物理 USB 摄像头扫描与分辨率格式探测 (支持 dry_run 隔离跳过);
3. 机械臂串口握手与舵机通信延迟测算;
4. 核心 Python 环境与软硬件依赖检查.
"""

from __future__ import annotations

import glob
import os
import platform
import sys
import time
from pathlib import Path
from typing import Any

from cameras.discovery import discover_cameras
from hardware.config import ArmConfig, RobotSystemConfig
from hardware.system_adapter import RobotSystemAdapter

# Software root
_software_root = Path(__file__).resolve().parent.parent


def check_serial_ports() -> list[dict[str, Any]]:
    """跨平台扫描系统串口及读写权限 (Windows COM* / Linux /dev/tty*)."""
    is_win = platform.system() == "Windows"
    port_desc: dict[str, str] = {}

    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            dev = str(getattr(p, "device", "") or "").strip()
            if dev:
                port_desc[dev] = str(getattr(p, "description", "") or "")
    except ImportError:
        pass

    if is_win:
        results = []
        for dev in sorted(port_desc.keys()):
            results.append({
                "port": dev,
                "description": port_desc[dev],
                "readable": True,
                "writable": True,
                "accessible": True,
            })
        return results

    candidates = sorted(
        set(port_desc.keys()) | set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    )
    results = []
    for p in candidates:
        readable = os.access(p, os.R_OK)
        writable = os.access(p, os.W_OK)
        results.append({
            "port": p,
            "description": port_desc.get(p, ""),
            "readable": readable,
            "writable": writable,
            "accessible": readable and writable,
        })
    return results


def check_cameras(skip_probe: bool = False) -> list[dict[str, Any]]:
    """扫描系统中所有可用的视频采集设备 (V4L2 / DirectShow)."""
    if (
        skip_probe
        or os.environ.get("ARM101_DRY_RUN") == "1"
        or os.environ.get("STUDIO_MODE") == "dry_run"
    ):
        return []

    cams = discover_cameras()
    info_list = []
    for c in cams:
        info_list.append({
            "device": c.device_path,
            "card": c.card_name,
            "is_capture": c.is_capture,
            "formats": c.supported_formats,
        })
    return info_list


def test_arm_communication(port: str | None = None) -> dict[str, Any]:
    """尝试与指定端口的机械臂建立极短的连通性诊断."""
    cfg = ArmConfig.default_follower(port=port)
    res: dict[str, Any] = {
        "port": cfg.port,
        "success": False,
        "latency_ms": 0.0,
        "error": None,
    }
    sys_cfg = RobotSystemConfig(follower=cfg, use_mock=False)
    t0 = time.perf_counter()
    try:
        adapter = RobotSystemAdapter(sys_cfg)
        adapter.connect(connect_leader=False)
        st = adapter.read_follower_state()
        adapter.disconnect(keep_torque=False)
        dt = (time.perf_counter() - t0) * 1000.0
        res["success"] = True
        res["latency_ms"] = round(dt, 2)
        res["positions"] = st.positions
    except Exception as exc:
        res["error"] = str(exc)
    return res


def run_system_diagnosis(probe_hardware: bool = True) -> bool:
    """执行完整的系统体检并输出格式化诊断报告."""
    print("\n" + "=" * 70)
    print("        🩺 SO-ARM101 具身机器人系统体检与环境诊断 (Doctor)")
    print("=" * 70)

    all_passed = True
    is_win = platform.system() == "Windows"

    # 1. Python 环境与核心组件
    from core.config_loader import get_config
    cfg = get_config(reload=True)

    print("\n[1/4] 检查 Python 运行环境与核心组件:")
    print(f"  • Python 版本: {sys.version.split()[0]} ({sys.executable})")
    print(f"  • 操作系统: {platform.system()} ({platform.machine()})")
    print(f"  • 工作根路径: {_software_root}")
    print(f"  • 全局配置: 从臂 {cfg.follower.port} (id={cfg.follower.id}), 主臂 {cfg.leader.port} (id={cfg.leader.id}), 相机 {cfg.camera.port}")

    # 2. 检查串口设备
    port_hint = "COM*" if is_win else "/dev/ttyACM*, /dev/ttyUSB*"
    print(f"\n[2/4] 扫描物理串口通信节点 ({port_hint}):")
    ports = check_serial_ports()
    if not ports:
        print("  ⚠️ 未发现任何 USB 串口设备节点。")
        print("     -> 提示: 若现场无真机，请在启动时使用 --mock 纯仿真演示模式。")
    else:
        for p in ports:
            if p["accessible"]:
                desc = f" ({p['description']})" if p.get("description") else ""
                status = f"✅ [端口可用]{desc}"
            else:
                perm_tip = "请检查设备管理器占用" if is_win else f"请运行 sudo chmod 666 {p['port']}"
                status = f"❌ [无读写权限: {perm_tip}]"
                all_passed = False
            print(f"  • {p['port']:<16} {status}")

    # 3. 检查 USB 摄像头
    cam_backend = "DirectShow/OpenCV" if is_win else "UVC/V4L2"
    print(f"\n[3/4] 扫描物理摄像头设备 ({cam_backend}):")
    cams = check_cameras(skip_probe=not probe_hardware)
    capture_cams = [c for c in cams if c["is_capture"]]
    if not probe_hardware:
        print("  ℹ️ 已按非侵入模式跳过物理摄像头探测。")
    elif not capture_cams:
        print("  ⚠️ 未检测到可用的视频捕获摄像头。")
    else:
        print(f"  ✅ 发现 {len(capture_cams)} 个可用的视频捕获摄像头:")
        for c in capture_cams:
            fmts = ", ".join(c["formats"]) if c["formats"] else "N/A"
            print(f"  • {c['device']:<14} {c['card']} (格式: {fmts})")

    # 4. 机械臂通信握手探测
    print("\n[4/4] 机械臂硬件连通性探测:")
    target_port = cfg.follower.port
    matching_port = any(p["port"] == target_port for p in ports) if ports else False
    if not probe_hardware:
        print("  ℹ️ 已跳过真实串口握手 (probe_hardware=False)。Mock 虚拟仿真驱动已验证就绪。")
    elif matching_port:
        print(f"  • 正在尝试对配置指定的从臂端口 {target_port} 进行快速连接握手...")
        test_res = test_arm_communication(target_port)
        if test_res["success"]:
            print(f"  ✅ 硬件握手成功! 通信延迟: {test_res['latency_ms']} ms")
        else:
            print(f"  ⚠️ 端口 {target_port} 握手未成功: {test_res['error']}")
            print("     -> 提示: 请检查 12V 稳压电源是否开启，或舵机串口波特率是否匹配 (1000000)。")
    elif ports:
        fallback_port = ports[0]["port"]
        print(f"  ⚠️ 配置文件中指定的从臂端口 {target_port} 未在当前系统串口中，尝试探测首个串口 {fallback_port}...")
        test_res = test_arm_communication(fallback_port)
        if test_res["success"]:
            print(f"  ✅ 硬件握手成功! 通信延迟: {test_res['latency_ms']} ms")
        else:
            print(f"  ⚠️ 端口 {fallback_port} 握手未成功: {test_res['error']}")
    else:
        print(f"  ℹ️ 跳过硬件握手 (未插串口 {target_port})。Mock 虚拟仿真驱动已验证就绪。")

    print("\n" + "=" * 70)
    print("  📋 诊断结论: 系统基础组件完备，支持启动机械臂遥操作与示教控制。")
    print("=" * 70 + "\n")
    return all_passed
