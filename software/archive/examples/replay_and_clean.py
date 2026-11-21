#!/usr/bin/env python3
"""SO-ARM101 示教轨迹离线诊断、回放与数据清洗 CLI 工具.

用法示例:
1. 查看所有已录制的示教轨迹:
    python3 examples/replay_and_clean.py --list

2. 分析某条示教轨迹健康指标 (角速度、静止死区、突变诊断、工作空间包围盒):
    python3 examples/replay_and_clean.py --analyze recording/data/sim/test_sim_task/episode_000000.jsonl

3. 自动掐头去尾 (剔除静止死区) 并平滑滤波输出清洗后文件:
    python3 examples/replay_and_clean.py --clean recording/data/sim/test_sim_task/episode_000000.jsonl --auto-trim --smooth 5
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

# 添加工程根目录至 sys.path
_software_root = Path(__file__).resolve().parent.parent
if str(_software_root) not in sys.path:
    sys.path.insert(0, str(_software_root))

from recording.cleaner import EpisodeCleaner, EpisodeData
from recording.recorder import scan_recorded_episodes


def cmd_list(base_dir: Path | None = None) -> None:
    """扫描并列出所有已录制的示教数据集."""
    episodes = scan_recorded_episodes(base_dir)
    if not episodes:
        print("[!] 未找到任何已录制的 .jsonl 示教轨迹文件。")
        return

    print("\n" + "=" * 90)
    print(f" 📂 SO-ARM101 示教轨迹库 (共找到 {len(episodes)} 个录制文件)")
    print("=" * 90)
    print(f"{'环境':<6} | {'任务名':<18} | {'文件名':<24} | {'帧数/时长':<14} | {'大小(KB)':<8} | {'录制时间'}")
    print("-" * 90)

    for ep in episodes:
        env_tag = "[SIM]" if ep["is_sim"] else "[REAL]"
        clean_tag = " [清洗版]" if ep.get("is_cleaned") else ""
        fname = ep["file_name"] + clean_tag
        size_kb = ep.get("file_size_kb", 0.0)
        frames_dur = f"{ep.get('total_frames', 0)}帧 / {ep.get('duration_s', 0.0):.1f}s"
        created = ep.get("created_at", "")[:19]
        print(f"{env_tag:<6} | {ep['task_name']:<18} | {fname:<24} | {frames_dur:<14} | {size_kb:<8} | {created}")
    print("=" * 90 + "\n")


def cmd_analyze(file_path: str) -> None:
    """深度诊断分析示教轨迹."""
    fpath = Path(file_path).resolve()
    if not fpath.is_file():
        print(f"[!] 找不到文件: {fpath}")
        return

    print(f"\n[*] 正在解析示教轨迹: {fpath.name} ...")
    ep = EpisodeData.from_file(fpath)
    stats = ep.analyze()

    print("\n" + "=" * 60)
    print(f" 📊 轨迹统计与健康诊断报告: {ep.task_name} / {fpath.name}")
    print("=" * 60)
    print(f"  • 总 帧 数: {stats.total_frames} 帧")
    print(f"  • 录制时长: {stats.duration_s:.2f} 秒")
    print(f"  • 平均帧率: {stats.average_fps:.1f} FPS")
    print(f"  • 录制环境: {ep.env_type.upper()} ({'仿真' if ep.is_sim else '真机'})")
    print("-" * 60)
    print(" ⏱️ 时序与死区分析:")
    print(f"  • 前置静止帧: {stats.idle_start_frames} 帧 ({stats.idle_start_frames * 0.033:.2f}s)")
    print(f"  • 后置静止帧: {stats.idle_end_frames} 帧 ({stats.idle_end_frames * 0.033:.2f}s)")
    print(f"  • 净有效动作: {stats.effective_frames} 帧 ({stats.effective_duration_s:.2f}s)")
    deadzone_pct = ((stats.idle_start_frames + stats.idle_end_frames) / max(1, stats.total_frames)) * 100
    print(f"  • 死区冗余比: {deadzone_pct:.1f}%")
    print("-" * 60)
    print(" ⚡ 各舵机最大角速度 (rad/s):")
    for j, v in stats.max_joint_velocities.items():
        print(f"    - {j:<14}: {v:.2f} rad/s ({math.degrees(v):.1f}°/s)")
    print("-" * 60)
    print(" 📦 末端工作空间包围盒 (TCP Bounds, 米):")
    b = stats.tcp_bounds
    print(f"    - X (前后): [{b['min_x']:.3f}, {b['max_x']:.3f}] m")
    print(f"    - Y (左右): [{b['min_y']:.3f}, {b['max_y']:.3f}] m")
    print(f"    - Z (上下): [{b['min_z']:.3f}, {b['max_z']:.3f}] m")
    print("-" * 60)
    print(" 🚨 异常突变跳变诊断:")
    if stats.anomalies_count == 0:
        print("    ✅ 轨迹完全平滑，无突变跳点 (0 处异常)")
    else:
        print(f"    ⚠️ 探测到 {stats.anomalies_count} 处单步突变跳变:")
        for an in stats.anomalies_detail[:5]:
            print(f"       [第 {an['frame']} 帧] 关节 {an['joint']}: 跳变 {an['jump_deg']}° (速度 {an['speed_rad_s']} rad/s)")
        if stats.anomalies_count > 5:
            print(f"       ... 还有 {stats.anomalies_count - 5} 处省略")
    print("=" * 60 + "\n")


def cmd_clean(
    file_path: str,
    auto_trim: bool = False,
    range_trim: tuple[int, int] | None = None,
    smooth_window: int = 0,
    output_path: str | None = None,
) -> None:
    """执行轨迹裁剪与平滑清洗."""
    fpath = Path(file_path).resolve()
    if not fpath.is_file():
        print(f"[!] 找不到文件: {fpath}")
        return

    ep = EpisodeData.from_file(fpath)
    orig_count = ep.frame_count
    print(f"[*] 载入待清洗文件: {fpath.name} (原始帧数: {orig_count} 帧, {ep.duration_s:.2f}s)")

    cleaned_ep = ep
    if auto_trim:
        cleaned_ep, cut_s, cut_e = EpisodeCleaner.auto_trim_idle(ep)
        print(f"[*] 自动去除首尾死区: 裁剪前置 {cut_s} 帧, 裁剪后置 {orig_count - 1 - cut_e} 帧")
    elif range_trim:
        s, e = range_trim
        cleaned_ep = EpisodeCleaner.trim_range(ep, s, e)
        print(f"[*] 区间精确裁剪: 保留 [{s} : {e}] 共 {cleaned_ep.frame_count} 帧")

    if smooth_window > 1:
        cleaned_ep = EpisodeCleaner.smooth_trajectory(cleaned_ep, window_size=smooth_window)
        print(f"[*] 应用滑动窗口平滑滤波: 窗口尺寸 = {smooth_window} 帧")

    if output_path:
        out_f = Path(output_path).resolve()
    else:
        stem = fpath.stem
        if "_cleaned" not in stem:
            stem = f"{stem}_cleaned"
        out_f = fpath.parent / f"{stem}.jsonl"

    saved = EpisodeCleaner.save_to_file(cleaned_ep, out_f, overwrite=True)
    reduction = ((orig_count - cleaned_ep.frame_count) / max(1, orig_count)) * 100
    print(f"\n[✓] 清洗成功完成!")
    print(f"    - 原始数据: {orig_count} 帧 ({ep.duration_s:.2f}s)")
    print(f"    - 清洗数据: {cleaned_ep.frame_count} 帧 ({cleaned_ep.duration_s:.2f}s)")
    print(f"    - 数据精简率: {reduction:.1f}%")
    print(f"    - 输出路径: {saved}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-ARM101 示教轨迹回放与数据清洗工具")
    parser.add_argument("--list", action="store_true", help="列出所有已录制的示教轨迹文件")
    parser.add_argument("--analyze", type=str, metavar="FILE", help="深度分析指定轨迹文件的健康状态与死区")
    parser.add_argument("--clean", type=str, metavar="FILE", help="对指定轨迹文件执行清洗并另存")
    parser.add_argument("--auto-trim", action="store_true", help="自动裁剪机械臂动作前后的静止死区")
    parser.add_argument("--range", type=int, nargs=2, metavar=("START", "END"), help="指定起止帧闭区间精确裁剪")
    parser.add_argument("--smooth", type=int, default=0, metavar="WINDOW", help="对各舵机角度执行平滑滤波窗口尺寸 (默认0不平滑)")
    parser.add_argument("-o", "--output", type=str, metavar="OUT_FILE", help="清洗结果保存的目标文件路径")

    args = parser.parse_args()

    if args.list:
        cmd_list()
    elif args.analyze:
        cmd_analyze(args.analyze)
    elif args.clean:
        if not args.auto_trim and not args.range and args.smooth <= 0:
            print("[!] 请至少指定一种清洗策略: --auto-trim, --range START END 或 --smooth WINDOW")
            sys.exit(1)
        cmd_clean(
            args.clean,
            auto_trim=args.auto_trim,
            range_trim=tuple(args.range) if args.range else None,
            smooth_window=args.smooth,
            output_path=args.output,
        )
    else:
        parser.print_help()


if __name__ == "__main__":
    import math
    main()
