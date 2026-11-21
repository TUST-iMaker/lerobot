#!/usr/bin/env python3
"""SO-ARM101 USB 相机管理与实时采集演示脚本.

用法:
    python3 examples/camera_demo.py
    python3 examples/camera_demo.py --device /dev/video0 --frames 30
    python3 examples/camera_demo.py --mock
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# 确保软件根目录在 sys.path
_here = Path(__file__).resolve().parent
_root = _here.parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from cameras import (
    CameraConfig,
    CameraManager,
    discover_cameras,
    list_capture_cameras,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="SO-ARM101 USB 相机管理与采集测试工具")
    parser.add_argument("--device", default=None, help="指定相机设备节点 (如 /dev/video0 或 0，默认自动探测)")
    parser.add_argument("--frames", type=int, default=20, help="测试采集连续帧数 (默认 20)")
    parser.add_argument("--fps", type=int, default=30, help="目标帧率 (默认 30)")
    parser.add_argument("--width", type=int, default=640, help="宽度 (默认 640)")
    parser.add_argument("--height", type=int, default=480, help="高度 (默认 480)")
    parser.add_argument("--mock", action="store_true", help="强制使用仿真 Mock 相机测试")
    parser.add_argument("--save", default="camera_snapshot.jpg", help="保存测试单帧截图路径")

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("               📷 SO-ARM101 USB 相机管理系统")
    print("=" * 70)

    # 1. 扫描系统上的全部物理相机
    print("\n[*] 正在扫描物理 USB 相机设备节点...")
    devices = discover_cameras()
    if not devices:
        print("  [!] 未扫描到任何 /dev/video* 节点")
    else:
        print(f"  [+] 共发现 {len(devices)} 个设备节点:")
        for dev in devices:
            tag = "✅ [视频捕获]" if (dev.is_capture and not dev.is_metadata) else "ℹ️ [元数据/辅助]"
            print(f"      • {dev.device_path} -> {dev.card_name} | {dev.driver_name} | {tag}")
            if dev.supported_formats:
                print(f"        支持格式: {', '.join(dev.supported_formats)}")

    # 2. 构建相机配置
    cm = CameraManager(base_images_dir=_root / "recording" / "data")

    if args.mock:
        print("\n[*] 启动仿真 MockCamera 模式...")
        cfg = CameraConfig.mock_camera("cam_mock", width=args.width, height=args.height, fps=args.fps)
        cm.add_camera(cfg)
    elif args.device:
        print(f"\n[*] 注册指定相机设备: {args.device}...")
        cfg = CameraConfig.default_usb("cam_custom", device=args.device, width=args.width, height=args.height, fps=args.fps)
        cm.add_camera(cfg)
    else:
        print("\n[*] 自动注册系统最佳 USB 相机...")
        cids = cm.auto_discover_and_register(max_cameras=2)
        print(f"  [+] 已注册相机列表: {cids}")

    # 3. 连接相机
    print("\n[*] 正在建立相机硬件连接...")
    results = cm.connect_all()
    for cid, ok in results.items():
        st = "✅ 连接成功" if ok else "❌ 连接失败"
        print(f"  • 相机 [{cid}]: {st}")

    if not any(results.values()):
        print("\n[!] 没有任何相机连接成功，退出。")
        return 1

    # 4. 实时连续帧采集与帧率测算
    target_cam_id = cm.camera_ids[0]
    cam = cm.get_camera(target_cam_id)
    print(f"\n[*] 开始对 [{target_cam_id}] 连续采集 {args.frames} 帧 (后端: {cam.__class__.__name__})...")

    t_start = time.monotonic()
    last_frame = None

    for i in range(args.frames):
        try:
            frame = cam.read_frame()
            last_frame = frame
            time.sleep(1.0 / args.fps)
            print(f"\r  -> 采集进度: [{i+1:2d}/{args.frames}] | 帧ID: #{frame.frame_id:04d} | 分辨率: {frame.width}x{frame.height} | 实时: {cam.actual_fps:4.1f} FPS | 字节: {len(frame.raw_bytes)} B", end="", flush=True)
        except Exception as e:
            print(f"\n[!] 抓帧异常: {e}")
            break

    elapsed = time.monotonic() - t_start
    actual_avg_fps = args.frames / max(0.001, elapsed)
    print(f"\n\n[+] 连续采集完成: 共 {args.frames} 帧, 耗时 {elapsed:.2f}s (平均 {actual_avg_fps:.1f} FPS)")

    # 5. 保存测试截图
    if last_frame and args.save:
        save_path = Path(args.save)
        with open(save_path, "wb") as f:
            f.write(last_frame.to_jpeg_bytes())
        print(f"[+] 测试截图已成功保存至: {save_path.resolve()} ({save_path.stat().st_size} bytes)")

    # 6. 断开释放
    cm.disconnect_all()
    print("[*] 全部相机已安全断开与释放。\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

