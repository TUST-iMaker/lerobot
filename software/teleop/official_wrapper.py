#!/usr/bin/env python3
"""官方 LeRobot 录制与遥操作透传包装器 (带退出安全守护).

特点:
1. 100% 官方底层: 内部直接调用 HuggingFace LeRobot 官方的 lerobot_record 或 lerobot_teleoperate;
2. 规避 GUI 崩溃: 默认采用官方原生高鲁棒性后台视频流编码 (模式 C)，彻底免除 cv2.imshow 依赖;
3. 严格安全规则守护: 无论录制正常结束还是用户按 Ctrl-C 强制退出，
   程序都会在退出时自动接管从臂，平滑回位到 rest_pose.json 蜷缩位并安全释放扭矩，防止机械臂坠落摔伤。
"""

from __future__ import annotations

import json
import logging
import os
import signal
import subprocess
import sys
import time
from contextlib import suppress
from pathlib import Path

from core.config_loader import get_config

try:
    import termios
except ImportError:
    termios = None

logger = logging.getLogger("official_wrapper")

_software_root = Path(__file__).resolve().parent.parent
_lerobot_root = _software_root.parent


def safe_park_to_rest(port: str | None = None, duration_s: float | None = None) -> bool:
    """在官方程序退出后执行安全守护：平滑回蜷缩位并释放扭矩."""
    cfg = get_config()
    target_port = port or cfg.follower.port
    dur = duration_s if duration_s is not None else cfg.safety.rest_duration_s

    print("\n" + "=" * 60)
    print(f"🛡️  [安全守护] 正在接管从臂 ({target_port})：平滑返回蜷缩位并释放扭矩 (耗时 {dur:.1f}s)...")
    print("=" * 60)

    # 稍作等待以确保官方进程已彻底释放串口句柄
    time.sleep(0.6)

    try:
        from core.action_api import ActionAPI
        from hardware.config import ArmConfig, RobotSystemConfig

        sys_cfg = RobotSystemConfig(
            follower=ArmConfig.default_follower(port=target_port)
        )
        api = ActionAPI(config=sys_cfg)
        if not api.connect(connect_leader=False):
            print("[!] 安全守护：无法重新连接从臂串口以执行回位 (可能串口未释放或已被拔出)")
            return False

        success = api.safe_return_to_rest(duration_s=dur)
        api.disconnect(keep_torque=False, return_to_rest=False)

        if success:
            print("✅ [安全守护] 从臂已安全回归蜷缩姿态并释放扭矩，设备保护完毕。")
        else:
            print("[!] [安全守护] 姿态回归完成，扭矩已释放。")
        return True
    except Exception as exc:
        print(f"[!] 安全守护执行异常: {exc}")
        return False


def run_official_record(
    follower_port: str | None = None,
    leader_port: str | None = None,
    follower_id: str | None = None,
    leader_id: str | None = None,
    camera_id: int | str = 2,
    camera_enabled: bool = True,
    repo_id: str | None = None,
    num_episodes: int = 2,
    single_task: str = "SO101 示教任务",
    fps: int | None = None,
    display_data: bool = True,
    display_mode: str = "rerun",
    extra_args: list[str] | None = None,
) -> int:
    """透传执行官方 lerobot_record，并在退出时执行安全回位守护."""
    cfg = get_config()
    target_follower_port = follower_port or cfg.follower.port
    target_leader_port = leader_port or cfg.leader.port
    target_follower_id = follower_id or cfg.follower.id
    target_leader_id = leader_id or cfg.leader.id
    target_repo_id = repo_id or cfg.operation.dataset_repo_id
    target_fps = fps or cfg.operation.fps

    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.lerobot_record",
        "--robot.type=so101_follower",
        f"--robot.port={target_follower_port}",
        f"--robot.id={target_follower_id}",
        "--teleop.type=so101_leader",
        f"--teleop.port={target_leader_port}",
        f"--teleop.id={target_leader_id}",
        f"--dataset.repo_id={target_repo_id}",
        f"--dataset.num_episodes={num_episodes}",
        f"--dataset.single_task={single_task}",
        f"--dataset.fps={target_fps}",
        f"--display_data={'true' if display_data else 'false'}",
    ]

    if display_data:
        cmd.append(f"--display_mode={display_mode}")

    if camera_enabled:
        cam_idx: int | str
        try:
            cam_idx = int(camera_id)
        except (ValueError, TypeError):
            cam_idx = camera_id

        cam_cfg = json.dumps({
            "cam_robot": {
                "type": "opencv",
                "index_or_path": cam_idx,
                "width": 640,
                "height": 480,
                "fps": fps,
            }
        })
        cmd.append(f"--robot.cameras={cam_cfg}")

    if extra_args:
        cmd.extend(extra_args)

    print("\n" + "=" * 60)
    print("🚀 [官方录制管道启动] (模式 C: 多模态录制，用于后续模型训练)")
    print("=" * 60)
    print(f"[*] 执行命令: {' '.join(cmd)}")
    print("[*] 提示: 按 q、Esc 或 Ctrl-C 停止录制，等待保存和安全回位完成。")
    print("-" * 60 + "\n")

    env = os.environ.copy()
    src_path = str(_lerobot_root / "src")
    env["PYTHONPATH"] = f"{src_path}:{env.get('PYTHONPATH', '')}"

    tty_fd = None
    tty_attrs = None
    process = None
    previous_sigint = None
    interrupted = False
    interrupt_forwarded = False

    def request_stop(_signum, _frame):
        nonlocal interrupted, interrupt_forwarded
        interrupted = True
        if os.name == "posix" and process is not None and not interrupt_forwarded:
            interrupt_forwarded = True
            with suppress(ProcessLookupError):
                # Popen.wait() may hold its wait lock when this handler runs.
                os.kill(process.pid, signal.SIGINT)

    try:
        if termios is not None and sys.stdin.isatty():
            tty_fd = sys.stdin.fileno()
            with suppress(termios.error):
                tty_attrs = termios.tcgetattr(tty_fd)
        previous_sigint = signal.signal(signal.SIGINT, request_stop)
        # On POSIX the wrapper receives the terminal's Ctrl-C and forwards it once.
        # Wait for the child to save and close the serial ports before parking.
        with subprocess.Popen(cmd, env=env, start_new_session=os.name == "posix") as process:
            if interrupted:
                request_stop(signal.SIGINT, None)
            exit_code = process.wait()
            process = None
        if interrupted and exit_code in (0, -signal.SIGINT):
            return 130
        return exit_code
    except Exception as exc:
        print(f"\n[!] 官方录制运行异常: {exc}")
        return 1
    finally:
        process = None
        try:
            if tty_attrs is not None:
                try:
                    termios.tcsetattr(tty_fd, termios.TCSANOW, tty_attrs)
                except termios.error as exc:
                    logger.warning("恢复录制终端属性失败: %s", exc)
        finally:
            try:
                # 严格执行规则 1：必须回蜷缩位并释放扭矩
                safe_park_to_rest(port=target_follower_port)
            finally:
                if previous_sigint is not None:
                    signal.signal(signal.SIGINT, previous_sigint)


def run_official_teleoperate(
    follower_port: str | None = None,
    leader_port: str | None = None,
    follower_id: str | None = None,
    leader_id: str | None = None,
    camera_id: int | str = 2,
    camera_enabled: bool = True,
    fps: int | None = None,
    display_data: bool = True,
    display_mode: str = "rerun",
    extra_args: list[str] | None = None,
) -> int:
    """透传执行官方 lerobot_teleoperate，并在退出时执行安全回位守护."""
    cfg = get_config()
    target_follower_port = follower_port or cfg.follower.port
    target_leader_port = leader_port or cfg.leader.port
    target_follower_id = follower_id or cfg.follower.id
    target_leader_id = leader_id or cfg.leader.id
    target_fps = fps or cfg.operation.fps

    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.lerobot_teleoperate",
        "--robot.type=so101_follower",
        f"--robot.port={target_follower_port}",
        f"--robot.id={target_follower_id}",
        "--teleop.type=so101_leader",
        f"--teleop.port={target_leader_port}",
        f"--teleop.id={target_leader_id}",
        f"--fps={target_fps}",
        f"--display_data={'true' if display_data else 'false'}",
    ]

    if display_data:
        cmd.append(f"--display_mode={display_mode}")

    if camera_enabled:
        cam_idx: int | str
        try:
            cam_idx = int(camera_id)
        except (ValueError, TypeError):
            cam_idx = camera_id

        cam_cfg = json.dumps({
            "cam_robot": {
                "type": "opencv",
                "index_or_path": cam_idx,
                "width": 640,
                "height": 480,
                "fps": fps,
            }
        })
        cmd.append(f"--robot.cameras={cam_cfg}")

    if extra_args:
        cmd.extend(extra_args)

    print("\n" + "=" * 60)
    print("🎮 [官方遥操作测试启动]")
    print("=" * 60)
    print(f"[*] 执行命令: {' '.join(cmd)}")
    print("[*] 提示: 退出 (Ctrl-C) 将自动触发安全回收机制。")
    print("-" * 60 + "\n")

    env = os.environ.copy()
    src_path = str(_lerobot_root / "src")
    env["PYTHONPATH"] = f"{src_path}:{env.get('PYTHONPATH', '')}"

    exit_code = 0
    try:
        res = subprocess.run(cmd, env=env)
        exit_code = res.returncode
    except KeyboardInterrupt:
        print("\n\n[!] 用户中断 (Ctrl-C)，正在退出官方遥操作...")
        exit_code = 130
    except Exception as exc:
        print(f"\n[!] 官方遥操作运行异常: {exc}")
        exit_code = 1
    finally:
        safe_park_to_rest(port=follower_port)

    return exit_code


def run_official_replay(
    follower_port: str | None = None,
    follower_id: str | None = None,
    repo_id: str | None = None,
    episode: int | None = None,
    fps: int | None = None,
    extra_args: list[str] | None = None,
) -> int:
    """透传执行官方 lerobot_replay 并在退出或播放完毕时执行安全回蜷缩位守护."""
    cfg = get_config()
    target_follower_port = follower_port or cfg.follower.port
    target_follower_id = follower_id or cfg.follower.id
    target_repo_id = repo_id or cfg.operation.dataset_repo_id
    target_episode = episode if episode is not None else cfg.operation.default_replay_episode
    target_fps = fps or cfg.operation.fps

    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.lerobot_replay",
        "--robot.type=so101_follower",
        f"--robot.port={target_follower_port}",
        f"--robot.id={target_follower_id}",
        f"--dataset.repo_id={target_repo_id}",
        f"--dataset.episode={target_episode}",
        f"--dataset.fps={target_fps}",
    ]

    if extra_args:
        cmd.extend(extra_args)

    print("\n" + "=" * 60)
    print("🎬 [官方数据集回放启动] (lerobot_replay)")
    print("=" * 60)
    print(f"[*] 执行命令: {' '.join(cmd)}")
    print(f"[*] 数据集: {target_repo_id} | Episode: {target_episode} | 频率: {target_fps}Hz")
    print("[*] 提示: 播放完毕或中途退出 (Ctrl-C) 将自动触发安全回收机制。")
    print("-" * 60 + "\n")

    env = os.environ.copy()
    src_path = str(_lerobot_root / "src")
    env["PYTHONPATH"] = f"{src_path}:{env.get('PYTHONPATH', '')}"

    exit_code = 0
    try:
        res = subprocess.run(cmd, env=env)
        exit_code = res.returncode
    except KeyboardInterrupt:
        print("\n\n[!] 用户中断 (Ctrl-C)，正在退出官方回放...")
        exit_code = 130
    except Exception as exc:
        print(f"\n[!] 官方回放运行异常: {exc}")
        exit_code = 1
    finally:
        safe_park_to_rest(port=target_follower_port)

    return exit_code


