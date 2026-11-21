#!/usr/bin/env python3
"""SO-ARM101 具身机器人控制、示教与数据系统总调度入口 (Main CLI Dispatcher).

系统特性:
  1. 全局配置: 统一由 software/config.yaml 控制硬件端口、相机、安全速度与默认数据集;
  2. 安全守护: 任何示教录制、回放及退出操作均自动触发自适应 S 曲线平滑保位返回蜷缩姿态并释放扭矩;
  3. 官方原生: 100% 兼容 Hugging Face LeRobot 官方数据集标准与底层遥操作管道;
  4. 极速全功能 Web 驾驶舱: 零依赖控制中心 (含低延迟相机流、三大模式无缝切换、回蜷缩自动断力矩、100vh 满屏自适应);
  5. 开机防跳变与防翻转: 严格执行四阶段安全上电 (配置 -> 物理编码器对齐 -> 写目标 -> 使能)，杜绝手腕翻转跳变;
  6. 中位基准直连: 回中位 (Home) 优先绑定用户校准的 upright_pose.json 基准姿态;
  7. 串口防竞争与自适应缓冲: 双层 RLock 隔离总线通信，Web 运动中优先内存缓存防冲突，结合 12 帧压实力矩缓冲确保回蜷缩完全到位。

提供统一子命令分发：
  python main.py web             - 启动极速 Web 控制驾驶舱 (视频流/主从遥控/键盘控制/6轴看板)
  python main.py config          - 查看并验证当前 config.yaml 全局系统配置
  python main.py doctor          - 系统环境与硬件全面体检诊断 (串口/权限/相机/通信)
  python main.py teleop          - 主从双臂示教或键盘笛卡尔遥控 (优先读 config.yaml)
  python main.py record          - 官方多模态示教录制 (模式 C: 支持视频，退出自动回蜷缩位)
  python main.py replay          - 官方多模态示教回放 (支持指定 episode，退出自动回蜷缩位)
  python main.py teleop-official - 官方 LeRobot 遥操作控制测试 (带退出回蜷缩位守护)
  python main.py demo            - 执行机械臂预设表演动作 (如挥手 wave / 抓取表演 pick)
  python main.py data            - 查看官方已录制的数据集与 episode 列表
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# 将软件根目录加入模块搜索路径
_software_root = Path(__file__).resolve().parent
if str(_software_root) not in sys.path:
    sys.path.insert(0, str(_software_root))


def cmd_doctor(args: argparse.Namespace) -> int:
    
    """执行硬件与系统环境体检."""
    from hardware.doctor import run_system_diagnosis
    ok = run_system_diagnosis()
    return 0 if ok else 1


def cmd_teleop(args: argparse.Namespace, extra_args: list[str] | None = None) -> int:
    """启动遥操作."""
    if getattr(args, "official", False):
        if getattr(args, "record", False):
            return cmd_record_official(args, extra_args)
        return cmd_teleop_official(args, extra_args)

    if args.mode == "leader":
        from teleop.leader_follower import run_leader_follower
        return run_leader_follower(args)
    else:
        # 默认键盘笛卡尔遥控
        import subprocess
        script = _software_root / "teleop" / "keyboard_ee.py"
        cmd = [sys.executable, str(script)]
        if args.mock:
            cmd.append("--mock")
        if args.follower_port:
            cmd.extend(["--port", args.follower_port])
        return subprocess.call(cmd)


def cmd_record_official(args: argparse.Namespace, extra_args: list[str] | None = None) -> int:
    """透传执行官方 lerobot_record 并在退出时执行安全回蜷缩位守护."""
    from teleop.official_wrapper import run_official_record
    return run_official_record(
        follower_port=getattr(args, "follower_port", None),
        leader_port=getattr(args, "leader_port", None),
        follower_id=getattr(args, "follower_id", None),
        leader_id=getattr(args, "leader_id", None),
        camera_id=getattr(args, "camera_id", "2"),
        camera_enabled=getattr(args, "camera", True),
        repo_id=getattr(args, "repo_id", None),
        num_episodes=getattr(args, "num_episodes", 2),
        single_task=getattr(args, "task", "SO101 示教任务"),
        fps=int(getattr(args, "hz", 30)),
        display_data=getattr(args, "display", True),
        display_mode=getattr(args, "display_mode", "rerun"),
        extra_args=extra_args,
    )


def cmd_teleop_official(args: argparse.Namespace, extra_args: list[str] | None = None) -> int:
    """透传执行官方 lerobot_teleoperate 并在退出时执行安全回蜷缩位守护."""
    from teleop.official_wrapper import run_official_teleoperate
    return run_official_teleoperate(
        follower_port=getattr(args, "follower_port", None),
        leader_port=getattr(args, "leader_port", None),
        follower_id=getattr(args, "follower_id", None),
        leader_id=getattr(args, "leader_id", None),
        camera_id=getattr(args, "camera_id", "2"),
        camera_enabled=getattr(args, "camera", True),
        fps=int(getattr(args, "hz", 30)),
        display_data=getattr(args, "display", True),
        display_mode=getattr(args, "display_mode", "rerun"),
        extra_args=extra_args,
    )


def cmd_replay_official(args: argparse.Namespace, extra_args: list[str] | None = None) -> int:
    """透传执行官方 lerobot_replay 并在播放完毕或退出时执行安全回蜷缩位守护."""
    from teleop.official_wrapper import run_official_replay
    return run_official_replay(
        follower_port=getattr(args, "follower_port", None),
        follower_id=getattr(args, "follower_id", None),
        repo_id=getattr(args, "repo_id", None),
        episode=getattr(args, "episode", None),
        fps=int(getattr(args, "hz", 30)) if getattr(args, "hz", None) else None,
        extra_args=extra_args,
    )


def cmd_config(args: argparse.Namespace) -> int:
    """查看并打印当前 config.yaml 全局系统配置."""
    from core.config_loader import DEFAULT_CONFIG_PATH, get_config
    cfg = get_config(reload=True)
    print("\n" + "=" * 65)
    print(f"⚙️  SO-ARM101 全局系统配置报告 ({DEFAULT_CONFIG_PATH.name})")
    print("=" * 65)
    print(f"  • 配置文件路径: {DEFAULT_CONFIG_PATH}")
    print("\n[1/5] 从臂硬件 (Follower):")
    print(f"  • 串口端口: {cfg.follower.port}")
    print(f"  • 设备标识: {cfg.follower.id} ({cfg.follower.type})")
    print(f"  • 串口波特率: {cfg.follower.baudrate}")
    print("\n[2/5] 主臂硬件 (Leader):")
    print(f"  • 串口端口: {cfg.leader.port}")
    print(f"  • 设备标识: {cfg.leader.id} ({cfg.leader.type})")
    print(f"  • 串口波特率: {cfg.leader.baudrate}")
    if cfg.leader.joint_offsets_deg:
        offsets_str = ", ".join(f"{k}: {v:+.1f}°" for k, v in cfg.leader.joint_offsets_deg.items())
        print(f"  • 关节偏置补偿: {offsets_str}")
    print("\n[3/5] 视觉相机 (Camera):")
    print(f"  • 端口节点: {cfg.camera.port} (索引: {cfg.camera.index_or_path})")
    print(f"  • 采样规格: {cfg.camera.width}x{cfg.camera.height} @ {cfg.camera.fps}fps ({cfg.camera.format})")
    print("\n[4/5] 安全防护 (Safety):")
    print(f"  • 蜷缩姿态文件: {cfg.safety.rest_pose_file} ({'存在' if cfg.safety.rest_pose_path.is_file() else '未找到'})")
    print(f"  • 退出自动回位: {'开启' if cfg.safety.auto_rest_on_exit else '关闭'}")
    print(f"  • S曲线过渡耗时: {cfg.safety.rest_duration_s} 秒")
    print(f"  • 单关节限速: {cfg.safety.max_speed_deg_per_s} °/s")
    print("\n[5/5] 录播操作 (Operation):")
    print(f"  • 控制循环频率: {cfg.operation.fps} Hz")
    print(f"  • 默认数据集仓: {cfg.operation.dataset_repo_id}")
    print(f"  • 默认回放编号: Episode {cfg.operation.default_replay_episode}")
    print(f"  • 可视化状态: {cfg.operation.display_mode} ({'开启' if cfg.operation.display else '关闭'})")
    print("=" * 65 + "\n")
    return 0


def cmd_data(args: argparse.Namespace) -> int:
    """查看已录制的官方 LeRobot 数据集与 Episode 列表."""
    import json
    from pathlib import Path
    hf_lerobot = Path.home() / ".cache" / "huggingface" / "lerobot"
    print("\n" + "=" * 65)
    print("📁  LeRobot 官方多模态数据集目录 (Dataset Catalog)")
    print("=" * 65)
    print(f"数据根目录: {hf_lerobot}\n")

    found_any = False
    if hf_lerobot.is_dir():
        for info_path in sorted(hf_lerobot.glob("**/info.json")):
            ds_dir = info_path.parent.parent
            rel_repo = ds_dir.relative_to(hf_lerobot)
            found_any = True
            print(f"  📦 数据集仓库: {rel_repo}")
            try:
                with open(info_path, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                fps = meta.get("fps", 30)
                total_ep = meta.get("total_episodes", "未知")
                total_frames = meta.get("total_frames", "未知")
                print(f"     • 规格: {total_ep} 个 Episode | {total_frames} 帧 | {fps} Hz")
            except Exception:
                pass
            vids = list(ds_dir.glob("videos/*.mp4"))
            if vids:
                print(f"     • 视频镜头: {len(vids)} 个视角录像")
            print(f"     • 一键回放: python main.py replay --repo-id {rel_repo} --episode 0\n")

    if not found_any:
        print("  [空] 暂无已录制的数据集。")
        print("  -> 使用 python main.py record 开始录制一段新动作。")
    print("=" * 65 + "\n")
    return 0


def cmd_demo(args: argparse.Namespace) -> int:
    """执行预设展示动作动线 (如挥手致意 / 抓取表演 / 单姿态移动)."""
    from core.action_api import ActionAPI
    from core.data_types import ControlSource
    from hardware.config import RobotSystemConfig
    from teleop.demo_routine import DemoRoutineRunner

    print("\n" + "=" * 60)
    print("      🎬 SO-ARM101 现场动作演示启动器 (Demo Routine)")
    print("=" * 60)

    if args.mock:
        print("[*] 运行环境: 仿真 Mock 模式")
        sys_cfg = RobotSystemConfig.mock_system()
    else:
        print(f"[*] 运行环境: 物理真机 (从臂端口: {args.follower_port})")
        sys_cfg = RobotSystemConfig.real_system(follower_port=args.follower_port)

    api = ActionAPI(config=sys_cfg)
    if not api.connect(connect_leader=False):
        print("[!] 无法建立硬件连接")
        return 1

    api.claim_control(ControlSource.SCRIPT)
    runner = DemoRoutineRunner(api)

    try:
        if getattr(args, "routine", None) == "wave":
            print("[*] 正在执行现场演示动线: 挥手致意 (Wave Routine)...")
            runner.run_wave_routine()
        elif getattr(args, "routine", None) == "pick":
            print("[*] 正在执行现场演示动线: 抓取表演 (Pick Routine)...")
            runner.run_pick_routine()
        elif getattr(args, "pose", None):
            print(f"[*] 正在执行单姿态过渡: {args.pose} (耗时 {args.duration:.1f}s)...")
            runner.run_pose(args.pose, duration_s=args.duration)
        else:
            print("[*] 提示: 请使用 --routine wave (挥手致意) 或 --routine pick (抓取表演)。")
            print("[*] 轨迹示教回放请直接使用官方命令: python main.py replay [--episode 0]")
    finally:
        api.release_control(ControlSource.SCRIPT)
        api.disconnect(keep_torque=False)
        print("[✓] 演示会话结束，从臂力矩已安全释放。")

    return 0


def cmd_web(args: argparse.Namespace) -> int:
    """启动轻量极速原型 Web 控制中心."""
    from web_ui import run_web_server
    return run_web_server(port=args.port, mock=args.mock, follower_port=args.follower_port)


def main() -> int:
    from core.config_loader import get_config
    cfg = get_config()

    parser = argparse.ArgumentParser(
        description="SO-ARM101 具身机器人控制、示教与数据系统 (Main CLI)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True, help="子命令列表")

    # 0. web 子命令
    p_web = subparsers.add_parser("web", help="启动轻量极速原型 Web 控制中心 (视频流/Demo/数据看板)")
    p_web.add_argument("--port", type=int, default=8080, help="Web 监听端口 (默认 8080)")
    p_web.add_argument("--mock", action="store_true", help="启用虚拟仿真模式 (无须插硬件)")
    p_web.add_argument("--follower-port", default=cfg.follower.port, help=f"真机从臂端口 (默认 {cfg.follower.port})")
    p_web.set_defaults(func=cmd_web)

    # 1. doctor 子命令
    p_doctor = subparsers.add_parser("doctor", help="执行系统软硬件环境体检与诊断")
    p_doctor.set_defaults(func=cmd_doctor)

    # 2. config 子命令
    p_cfg = subparsers.add_parser("config", help="查看或验证当前 config.yaml 配置内容")
    p_cfg.set_defaults(func=cmd_config)

    # 3. teleop 子命令
    p_teleop = subparsers.add_parser("teleop", help="启动主从臂或键盘遥操作与示教录制")
    p_teleop.add_argument("--mode", choices=["keyboard", "leader"], default="leader", help="遥操作方式 (默认 leader 主从双臂)")
    p_teleop.add_argument("--keep-torque", action="store_true", help="退出时保持从臂扭矩 (默认释放扭矩并归位)")
    p_teleop.add_argument("--mock", action="store_true", help="纯软件虚拟遥操作")
    p_teleop.add_argument("--follower-port", default=cfg.follower.port, help=f"从臂端口 (默认 {cfg.follower.port})")
    p_teleop.add_argument("--leader-port", default=cfg.leader.port, help=f"主臂端口 (默认 {cfg.leader.port})")
    p_teleop.add_argument("--hz", type=float, default=float(cfg.operation.fps), help=f"控制频率 (默认 {cfg.operation.fps}Hz)")
    p_teleop.add_argument(
        "--record",
        action="store_true",
        help="开启示教动作录制功能 (手掰主臂时按 [空格键] 开始/结束动作)",
    )
    p_teleop.add_argument(
        "--task-prefix",
        default="demo_action",
        help="录制动作前缀 (默认 demo_action)",
    )
    p_teleop.add_argument(
        "--camera",
        dest="camera",
        action="store_true",
        default=True,
        help="开启摄像头监看与多模态图像录制 (模式 C，默认开启)",
    )
    p_teleop.add_argument(
        "--no-camera",
        dest="camera",
        action="store_false",
        help="禁用摄像头监看与图像录制 (纯关节模式)",
    )
    p_teleop.add_argument(
        "--camera-id",
        default=str(cfg.camera.index_or_path),
        help=f"摄像头设备编号或路径 (默认 {cfg.camera.index_or_path})",
    )
    p_teleop.add_argument(
        "--official",
        action="store_true",
        help="直接调用官方 LeRobot 底层内核 (带退出平滑回蜷缩位安全守护)",
    )
    p_teleop.set_defaults(func=cmd_teleop)

    # 4. record-official / record 子命令 (带退出安全守护)
    p_rec = subparsers.add_parser(
        "record-official",
        aliases=["record"],
        help="官方 LeRobot 多模态示教录制 (模式 C: 支持视频，带退出自动回蜷缩位守护)",
    )
    p_rec.add_argument("--follower-port", default=cfg.follower.port, help=f"从臂端口 (默认 {cfg.follower.port})")
    p_rec.add_argument("--leader-port", default=cfg.leader.port, help=f"主臂端口 (默认 {cfg.leader.port})")
    p_rec.add_argument("--follower-id", default=cfg.follower.id, help=f"从臂配置标识 (默认 {cfg.follower.id})")
    p_rec.add_argument("--leader-id", default=cfg.leader.id, help=f"主臂配置标识 (默认 {cfg.leader.id})")
    p_rec.add_argument("--repo-id", default=cfg.operation.dataset_repo_id, help=f"保存的数据集仓库名 (默认 {cfg.operation.dataset_repo_id})")
    p_rec.add_argument("--num-episodes", type=int, default=2, help="录制 episode 数 (默认 2)")
    p_rec.add_argument("--task", default="SO101 示教任务", help="示教任务名称描述")
    p_rec.add_argument("--camera-id", default=str(cfg.camera.index_or_path), help=f"摄像头设备编号或路径 (默认 {cfg.camera.index_or_path})")
    p_rec.add_argument("--camera", dest="camera", action="store_true", default=True, help="开启摄像头录制 (默认开启)")
    p_rec.add_argument("--no-camera", dest="camera", action="store_false", help="关闭摄像头录制")
    p_rec.add_argument("--hz", type=float, default=float(cfg.operation.fps), help=f"控制与采样频率 (默认 {cfg.operation.fps}Hz)")
    p_rec.add_argument("--display", dest="display", action="store_true", default=cfg.operation.display, help="开启画面可视化 (Rerun 仪表盘)")
    p_rec.add_argument("--no-display", dest="display", action="store_false", help="关闭画面可视化 (后台静默录制)")
    p_rec.add_argument("--display-mode", default=cfg.operation.display_mode, choices=["rerun", "foxglove"], help=f"可视化后端 (默认 {cfg.operation.display_mode})")
    p_rec.set_defaults(func=cmd_record_official)

    # 5. replay-official / replay 子命令 (带退出安全守护)
    p_rep = subparsers.add_parser(
        "replay-official",
        aliases=["replay"],
        help="官方 LeRobot 数据集示教回放 (带播放结束/退出自动回蜷缩位守护)",
    )
    p_rep.add_argument("--follower-port", default=cfg.follower.port, help=f"从臂端口 (默认 {cfg.follower.port})")
    p_rep.add_argument("--follower-id", default=cfg.follower.id, help=f"从臂配置标识 (默认 {cfg.follower.id})")
    p_rep.add_argument("--repo-id", default=cfg.operation.dataset_repo_id, help=f"保存的数据集仓库名 (默认 {cfg.operation.dataset_repo_id})")
    p_rep.add_argument("--episode", type=int, default=cfg.operation.default_replay_episode, help=f"回放 episode 索引 (默认 {cfg.operation.default_replay_episode})")
    p_rep.add_argument("--hz", type=float, default=float(cfg.operation.fps), help=f"控制与回放频率 (默认 {cfg.operation.fps}Hz)")
    p_rep.set_defaults(func=cmd_replay_official)

    # 6. teleop-official 子命令 (带退出安全守护)
    p_top = subparsers.add_parser(
        "teleop-official",
        help="官方 LeRobot 遥操作控制测试 (带退出自动回蜷缩位守护)",
    )
    p_top.add_argument("--follower-port", default=cfg.follower.port, help=f"从臂端口 (默认 {cfg.follower.port})")
    p_top.add_argument("--leader-port", default=cfg.leader.port, help=f"主臂端口 (默认 {cfg.leader.port})")
    p_top.add_argument("--follower-id", default=cfg.follower.id, help=f"从臂配置标识 (默认 {cfg.follower.id})")
    p_top.add_argument("--leader-id", default=cfg.leader.id, help=f"主臂配置标识 (默认 {cfg.leader.id})")
    p_top.add_argument("--camera-id", default=str(cfg.camera.index_or_path), help=f"摄像头设备编号或路径 (默认 {cfg.camera.index_or_path})")
    p_top.add_argument("--camera", dest="camera", action="store_true", default=True, help="开启摄像头")
    p_top.add_argument("--no-camera", dest="camera", action="store_false", help="关闭摄像头")
    p_top.add_argument("--hz", type=float, default=float(cfg.operation.fps), help=f"控制频率 (默认 {cfg.operation.fps}Hz)")
    p_top.add_argument("--display", action="store_true", default=cfg.operation.display, help="开启画面可视化 (Rerun/Foxglove)")
    p_top.add_argument("--display-mode", default=cfg.operation.display_mode, choices=["rerun", "foxglove"], help=f"可视化后端 (默认 {cfg.operation.display_mode})")
    p_top.set_defaults(func=cmd_teleop_official)

    # 7. data 子命令
    p_data = subparsers.add_parser("data", help="查看官方已录制的数据集与 Episode 列表")
    p_data.set_defaults(func=cmd_data)

    # 8. demo 子命令
    p_demo = subparsers.add_parser("demo", help="执行机械臂预设表演动作 (挥手/抓取/姿态)")
    p_demo.add_argument("--routine", choices=["wave", "pick"], default="wave", help="指定表演动作动线 (默认 wave 挥手致意)")
    p_demo.add_argument("--pose", choices=["home", "rest", "inspect", "pick_hover", "wave"], help="指定单预设姿态")
    p_demo.add_argument("--duration", type=float, default=cfg.safety.rest_duration_s, help=f"动作过渡时长秒 (默认 {cfg.safety.rest_duration_s}s)")
    p_demo.add_argument("--mock", action="store_true", help="使用仿真环境演示 (无须插硬件)")
    p_demo.add_argument("--follower-port", default=cfg.follower.port, help=f"真机从臂端口 (默认 {cfg.follower.port})")
    p_demo.set_defaults(func=cmd_demo)

    args, extra_args = parser.parse_known_args()
    if hasattr(args, "func"):
        import inspect
        sig = inspect.signature(args.func)
        if "extra_args" in sig.parameters:
            return args.func(args, extra_args=extra_args)
        return args.func(args)
    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
