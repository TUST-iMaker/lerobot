#!/usr/bin/env python3
"""SO-ARM101 Leader–Follower 主从机械臂实时遥操作与示教工具.

流程:
1. 同时连接 Follower 从臂 (/dev/ttyACM0) 与 Leader 主臂 (/dev/ttyACM1);
2. 主臂保持零力矩，允许人工轻快拖动示教; 从臂使能扭矩并平稳跟随;
3. 以目标频率 (如 30 Hz) 循环采样主臂关节姿态 -> 经 ActionAPI 安全审计 -> 下发从臂;
4. 任何时刻按 Ctrl-C 或发生通信异常，从臂立即锁定保持当前姿态，绝不脱缰.
"""

from __future__ import annotations

import argparse
import logging
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
from core.data_types import ControlSource, JointUnit, rad_to_deg
from hardware.config import ArmConfig, RobotSystemConfig

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger("teleop")


class NonBlockingTerminal:
    """跨平台终端非阻塞单字符读取上下文管理器 (Linux termios / Windows msvcrt)."""

    def __enter__(self) -> NonBlockingTerminal:
        self.old_settings = None
        if termios is not None and tty is not None and sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        if termios is not None and self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def read_key(self) -> str | None:
        """非阻塞即刻检查单个按键输入 (零延迟)."""
        if sys.platform == "win32":
            import msvcrt

            if msvcrt.kbhit():
                char = msvcrt.getwch()
                return "ESC" if char == "\x1b" else char
            return None
        if not sys.stdin.isatty():
            return None
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            char = sys.stdin.read(1)
            if char == "\x1b":
                return "ESC"
            return char
        return None


def run_leader_follower(args: argparse.Namespace) -> int:
    """主从遥操作与多模态示教录制主循环."""
    cfg = get_config()
    follower_port = getattr(args, "follower_port", None) or cfg.follower.port
    leader_port = getattr(args, "leader_port", None) or cfg.leader.port

    if getattr(args, "mock", False):
        sys_cfg = RobotSystemConfig.mock_system(with_leader=True)
    else:
        follower_cfg = ArmConfig.default_follower(port=follower_port)
        leader_cfg = ArmConfig.default_leader(port=leader_port)
        sys_cfg = RobotSystemConfig(
            follower=follower_cfg,
            leader=leader_cfg,
            frequency_hz=args.hz,
        )

    api = ActionAPI(config=sys_cfg)
    record_mode = getattr(args, "record", False)
    use_camera = getattr(args, "camera", False)
    recorder = None
    camera_mgr = None
    action_idx = 1
    recorded_actions: list[dict[str, Any]] = []

    if record_mode:
        try:
            from archive.recording.recorder import EpisodeRecorder
            recorder = EpisodeRecorder(is_sim=False)
        except Exception:
            print("[*] 提示: 示教录制推荐直接使用官方命令: python main.py record")
            recorder = None

    print("=" * 65)
    print("      SO-ARM101 主从双臂实时遥操作与示教录制系统 (30Hz 闭环)")
    print("=" * 65)
    print(f" 从臂端口: {follower_port} (执行从臂, 闭环使能扭矩)")
    print(f" 主臂端口: {leader_port} (示教主臂, 零力矩拖动)")
    print(f" 控制频率: {args.hz:.1f} Hz (周期 ~{1000/args.hz:.1f} ms)")
    if record_mode:
        print(" 录制模式: 已开启示教录制！")
        print("   -> 按 [空格] 开始录制当前动作，再按 [空格] 结束并保存")
        print("   -> 录制完的动作会自动存为 demo_action_1.jsonl, demo_action_2.jsonl...")
    if use_camera:
        print(" 视觉模式: 已启用模式 C (桌面实时画面监看 + 多模态图像同步录制)")
    print(" 安全机制: 退出时强制自动平滑收回蜷缩位 (rest_pose.json) 并释放扭矩")
    print(" 退出控制: 随时按 [q]、[ESC] 或 [Ctrl-C] 安全退出")
    print("=" * 65 + "\n")

    # 1. 尝试启动摄像头 (模式 C)
    if use_camera:
        try:
            from cameras.teleop_camera import TeleopCameraManager
            camera_mgr = TeleopCameraManager(
                device_index=getattr(args, "camera_id", "auto"),
                width=640,
                height=480,
                fps=args.hz,
            )
            if not camera_mgr.start():
                print("[!] 摄像头启动未成功，将以纯关节遥操作模式继续运行")
                camera_mgr = None
        except Exception as e:
            print(f"[!] 初始化摄像头异常: {e}，将以纯关节模式运行")
            camera_mgr = None

    print("[*] 正在连接双臂硬件...")
    if not api.connect(connect_leader=True):
        print("[!] 硬件连接失败，请核对端口是否正确插入")
        if camera_mgr:
            camera_mgr.close()
        return 1

    if not api.hardware.is_leader_connected:
        print("[!] 警告: 主臂未连接成功，无法进行主从遥操作!")
        if camera_mgr:
            camera_mgr.close()
        api.disconnect(keep_torque=False, return_to_rest=True)
        return 1

    # 申请主臂遥操作控制权
    if not api.claim_control(ControlSource.LEADER):
        print("[!] 申请 LEADER 控制权失败")
        if camera_mgr:
            camera_mgr.close()
        api.disconnect(keep_torque=False, return_to_rest=True)
        return 1

    print("[OK] 主从遥操作就绪! 请轻柔拖动主臂，从臂将实时同步跟随。")
    if record_mode:
        print(f"[*] 提示: 摆好起始姿态后，按 [空格键] 开始录制【动作 {action_idx}】\n")
    else:
        print("     随时按 Ctrl-C 或 [q] 退出并停止遥操作.\n")

    period_s = 1.0 / args.hz
    loop_count = 0
    start_time = time.monotonic()
    last_print = 0.0

    try:
        with NonBlockingTerminal() as term:
            while True:
                t_loop_start = time.perf_counter()

                # 1. 检查键盘或相机窗口的非阻塞按键输入
                key = term.read_key()
                cam_key = None
                if camera_mgr:
                    dur = recorder.current_duration_s if (recorder and recorder.is_recording) else 0.0
                    f_cnt = recorder.current_frame_count if (recorder and recorder.is_recording) else 0
                    cam_key = camera_mgr.render_preview_and_poll_key(
                        is_recording=(recorder and recorder.is_recording),
                        action_idx=action_idx,
                        frame_count=f_cnt,
                        duration_s=dur,
                        fps=args.hz,
                    )

                trigger_key = key or cam_key
                if trigger_key is not None:
                    if trigger_key in ("q", "Q", "ESC"):
                        print("\n[*] 接收到退出指令...")
                        break
                    elif record_mode and trigger_key in (" ", "\r", "\n", "r", "R"):
                        if recorder and recorder.is_recording:
                            # 结束当前动作录制
                            if camera_mgr:
                                camera_mgr.stop_recording()
                            summary = recorder.stop_episode()
                            saved_path = Path(summary["file_path"])
                            convenient_path = _root / "recording" / "data" / f"demo_action_{action_idx}.jsonl"
                            try:
                                import shutil
                                shutil.copyfile(saved_path, convenient_path)
                                summary["convenient_path"] = str(convenient_path)
                            except Exception:
                                pass

                            recorded_actions.append(summary)
                            print(f"\n\n[✓] 动作 {action_idx} 录制完成 (含多模态图像与视频)!")
                            print(f"    - 帧数: {summary['total_frames']} 帧 | 时长: {summary['duration_s']}s")
                            print(f"    - 轨迹: {convenient_path.name} (路径: {convenient_path})")
                            if camera_mgr:
                                print(f"    - 图像集: {saved_path.parent / 'images' / 'cam_robot'}\n")
                            action_idx += 1
                            if action_idx <= 4:
                                print(f"[*] 准备就绪: 请手摆下一个动作，按 [空格键] 开始录制【动作 {action_idx}】...\n")
                            else:
                                print("[*] 提示: 已经录制完成 4 个演示动作! 可继续录制或按 [q] 退出。\n")
                        elif recorder:
                            # 开始录制新动作
                            task_prefix = getattr(args, "task_prefix", "demo_action")
                            task_name = f"{task_prefix}_{action_idx}"
                            ep_file = recorder.start_episode(task_name=task_name)
                            if camera_mgr:
                                camera_mgr.start_recording(task_dir=ep_file.parent)
                            print(f"\n\n🔴 [开始录制: 动作 {action_idx}] 正在采集轨迹与多模态画面... (再次按 [空格键] 结束并保存)\n")

                # 2. 捕获主臂动作意图
                leader_action = api.get_leader_action()
                if leader_action is not None:
                    # 提交给 ActionAPI (经过安全限幅与限位审计下发给从臂)
                    ok, msg = api.submit_action(leader_action)
                    if not ok:
                        now = time.monotonic()
                        if now - last_print >= 1.0:
                            print(f"[!] 动作安全受阻: {msg}")
                            last_print = now

                # 3. 若正在录制，记录当前步骤数据 (关节状态 + 多模态图像帧引用)
                if record_mode and recorder and recorder.is_recording and leader_action is not None:
                    try:
                        f_state = api.hardware.read_follower_state()
                        cam_dict = None
                        if camera_mgr:
                            cam_ref = camera_mgr.record_step(frame_index=recorder.current_frame_count)
                            if cam_ref:
                                cam_dict = {"cam_robot": cam_ref}

                        recorder.record_step(
                            joint_state=f_state,
                            action=leader_action,
                            control_source=ControlSource.LEADER,
                            camera_frames=cam_dict,
                        )
                    except Exception:
                        pass

                loop_count += 1

                # 4. 动态状态行打印 (约每 0.2 秒刷新)
                now = time.monotonic()
                if now - last_print >= 0.2:
                    elapsed = now - start_time
                    actual_fps = loop_count / elapsed
                    if record_mode and recorder and recorder.is_recording:
                        sys.stdout.write(
                            f"\r🔴 [录制中: 动作 {action_idx}] 帧数: {recorder.current_frame_count:>3d} | 时长: {recorder.current_duration_s:>4.1f}s | 频率: {actual_fps:>4.1f}Hz (按 [空格] 结束保存)   "
                        )
                    elif record_mode:
                        sys.stdout.write(
                            f"\r🟢 [主从跟随中] 频率: {actual_fps:>4.1f}Hz (按 [空格] 开始录制动作 {action_idx})   "
                        )
                    else:
                        hw_rep = api.hardware.get_health_report()
                        f_lat = hw_rep["follower"]["health"]["last_read_latency_ms"]
                        l_lat = hw_rep.get("leader", {}).get("health", {}).get("last_read_latency_ms", 0.0)
                        sys.stdout.write(
                            f"\r[运行中] 频率: {actual_fps:>4.1f} Hz | 从臂延迟: {f_lat:>4.1f}ms | 主臂延迟: {l_lat:>4.1f}ms (按 [q] 退出)   "
                        )
                    sys.stdout.flush()
                    last_print = now

                # 5. 精确时钟休眠
                elapsed_loop = time.perf_counter() - t_loop_start
                sleep_s = period_s - elapsed_loop
                if sleep_s > 0:
                    time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n\n[*] 接收到中断信号 (Ctrl-C)，正在停止...")
    finally:
        # 关闭相机资源
        if camera_mgr:
            camera_mgr.close()

        # 如果退出时还在录制，自动安全落盘
        if record_mode and recorder and recorder.is_recording:
            try:
                summary = recorder.stop_episode()
                recorded_actions.append(summary)
                print(f"\n[✓] 退出前自动保存未完成的动作 {action_idx}: {summary['file_path']}")
            except Exception:
                pass

        if recorded_actions:
            print("\n" + "=" * 65)
            print("      🎬 本次会话已录制示教动作清单")
            print("=" * 65)
            for idx, item in enumerate(recorded_actions, 1):
                p_name = item.get("convenient_path") or item.get("file_path")
                print(f" 动作 {idx}: {Path(p_name).name} ({item.get('total_frames', 0)} 帧 / {item.get('duration_s', 0.0)}s)")
            print("=" * 65 + "\n")

        print("\n" + "=" * 65)
        print("🛡️  [安全退出守护] 正在接管从臂：平滑返回蜷缩位并释放扭矩...")
        print("=" * 65)

        keep_torque = getattr(args, "keep_torque", False)
        # 1. 优先平滑回归蜷缩位
        if not keep_torque:
            try:
                api.safe_return_to_rest(duration_s=2.0)
            except Exception as exc:
                print(f"[!] 执行平滑回蜷缩位异常: {exc}")

        # 2. 彻底断开硬件并释放扭矩
        try:
            api.disconnect(keep_torque=keep_torque, return_to_rest=False)
        except Exception as exc:
            print(f"[!] 断开硬件异常: {exc}")

        print("✅ [安全守护完成] 从臂已安全回归蜷缩姿态，扭矩已释放，会话结束。\n")

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SO-ARM101 主从机械臂实时遥操作与示教工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--follower-port", default=None, help="从臂串口 (默认从全局配置解析)")
    parser.add_argument("--leader-port", default=None, help="主臂串口 (默认从全局配置解析)")
    parser.add_argument("--hz", type=float, default=30.0, help="遥操作同步频率 (默认 30 Hz)")
    parser.add_argument("--mock", action="store_true", help="纯软件仿真模式 (无需物理串口)")
    parser.add_argument(
        "--record",
        action="store_true",
        help="开启示教动作录制功能 (手掰主臂时按 [空格键] 开始/结束动作)",
    )
    parser.add_argument(
        "--task-prefix",
        default="demo_action",
        help="录制动作前缀 (默认 demo_action)",
    )
    parser.add_argument(
        "--camera",
        dest="camera",
        action="store_true",
        default=True,
        help="开启摄像头监看与多模态图像录制 (模式 C，默认开启)",
    )
    parser.add_argument(
        "--no-camera",
        dest="camera",
        action="store_false",
        help="禁用摄像头监看与图像录制 (纯关节模式)",
    )
    parser.add_argument(
        "--camera-id",
        default="auto",
        help="摄像头设备编号或路径 (默认 auto，自动探测可用摄像头)",
    )
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="退出后保持从臂扭矩 (默认释放扭矩)",
    )
    return parser



if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    sys.exit(run_leader_follower(args))

