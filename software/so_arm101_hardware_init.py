#!/usr/bin/env python3
"""SO-ARM101 从臂初始化：连接、读取校准文件、使能并回中位。

流程:
    1. 连接 Follower（固定端口 /dev/ttyACM0）;
    2. 读取校准文件（不运行校准）;
    3. 使能 Follower 扭矩并保持当前姿态;
    4. 读取关节位置并核对扭矩状态;
    5. 所有关节回到中位并保持;
    6. 等待确认后断开，默认释放扭矩。

示例:
    # 初始化从臂并把各关节移到中位（退出后释放扭矩）
    python so_arm101_hardware_init.py

    # 退出后保持扭矩
    python so_arm101_hardware_init.py --keep-torque-on-exit

安全说明:
    * 连接 Follower 会配置舵机并使能扭矩，机械臂会尝试保持当前姿态;
    * 连接后会把 Goal_Position 同步到当前姿态，降低旧目标造成突然运动的概率;
    * 初始化完成后会把 Follower 各关节移到中位（来自校准文件）;
    * 中位运动顺序固定为 elbow_flex -> shoulder_lift -> 腕部 -> 底座 -> gripper;
    * 本脚本不再运行校准，中位目标直接来自校准文件;
    * 默认退出时释放扭矩；--keep-torque-on-exit 会保持扭矩，注意舵机发热;
    * 本脚本不替代独立硬件急停。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Ensure software root is on sys.path so core package can be imported
_here = Path(__file__).resolve().parent
_software_dir = _here if (_here / "core").is_dir() else _here.parent
if str(_software_dir) not in sys.path:
    sys.path.insert(0, str(_software_dir))

from core.data_types import (
    MIDDLE_ORDER,
    SO101_JOINT_NAMES,
    JointState,
    JointUnit,
)

FIXED_CALIBRATION_FILE = Path(
    "/home/aguiya/.cache/huggingface/lerobot/calibration/robots/so_follower/zihao_follower_arm.json"
)


def ensure_repo_on_path() -> None:
    """Make src/lerobot importable when running from a repository checkout."""
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "pyproject.toml").is_file() and (candidate / "src" / "lerobot").is_dir():
            src = str(candidate / "src")
            if src not in sys.path:
                sys.path.insert(0, src)
            return


def module_available(name: str) -> bool:
    """Return whether a module can be discovered without importing it."""
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def log(status: str, message: str) -> None:
    """Print one aligned status line."""
    print(f"[{status}] {message}", flush=True)


def section(title: str) -> None:
    """Print a section header."""
    print(f"\n== {title} ==", flush=True)


def countdown(seconds: int = 3) -> None:
    """Give the operator a short window to cancel with Ctrl-C."""
    print(f"{seconds} 秒后开始，按 Ctrl-C 取消 ...", flush=True)
    for remaining in range(seconds, 0, -1):
        print(f"  {remaining}...", flush=True)
        time.sleep(1)


def prompt_enter(message: str) -> None:
    """Wait for Enter, tolerating non-interactive stdin."""
    try:
        input(message)
    except EOFError:
        print()


def read_joint_states(
    getter: Any,
    count: int,
    unit: JointUnit,
    torque_states: dict[str, int] | None = None,
) -> tuple[list[JointState], list[float]]:
    """Read joint positions, wrap into standard JointState objects, and measure latency."""
    samples: list[JointState] = []
    latencies: list[float] = []
    torques_enabled = (
        {k: bool(v == 1) for k, v in torque_states.items()}
        if torque_states
        else None
    )

    for _ in range(count):
        started = time.perf_counter()
        observation = getter()
        latencies.append((time.perf_counter() - started) * 1000.0)
        positions: dict[str, float] = {}
        for key, value in observation.items():
            if not key.endswith(".pos"):
                continue
            joint_name = key[:-4] if key.endswith(".pos") else key
            try:
                positions[joint_name] = float(value)
            except (TypeError, ValueError):
                continue

        state = JointState(
            timestamp=time.monotonic(),
            positions=positions,
            unit=unit,
            torques_enabled=torques_enabled,
        )
        samples.append(state)
        time.sleep(0.03)
    return samples, latencies


def joint_statistics(samples: list[JointState]) -> dict[str, dict[str, float]]:
    """Compute basic statistics for every joint using standard JointState objects."""
    names = sorted(set().union(*(sample.positions.keys() for sample in samples))) if samples else []
    statistics: dict[str, dict[str, float]] = {}
    for name in names:
        values = [sample.positions[name] for sample in samples if name in sample.positions]
        if not values:
            continue
        statistics[name] = {
            "count": len(values),
            "mean": sum(values) / len(values),
            "min": min(values),
            "max": max(values),
            "span": max(values) - min(values),
        }
    return statistics


def print_joint_table(
    statistics: dict[str, dict[str, float]],
    unit: str,
) -> None:
    """Print a compact joint statistics table."""
    print(f"       单位: {unit}", flush=True)
    print(
        f"       {'关节':<22}{'均值':>12}{'最小':>12}{'最大':>12}{'跨度':>12}",
        flush=True,
    )
    for name, item in statistics.items():
        print(
            f"       {name:<22}{item['mean']:>12.3f}{item['min']:>12.3f}"
            f"{item['max']:>12.3f}{item['span']:>12.3f}",
            flush=True,
        )


def read_torque_states(device: Any) -> tuple[dict[str, int] | None, str | None]:
    """Read Torque_Enable from every motor without normalization."""
    try:
        values = device.bus.sync_read("Torque_Enable", normalize=False)
        return {str(name): int(value) for name, value in values.items()}, None
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"


def report_torque(
    device: Any,
    label: str,
    expected_enabled: bool,
) -> tuple[dict[str, int] | None, str | None]:
    """Read and report whether torque matches the expected state."""
    states, error = read_torque_states(device)
    if error:
        log("WARN", f"{label} 扭矩状态读取失败: {error}")
        return None, error

    expected = 1 if expected_enabled else 0
    wrong = {name: value for name, value in states.items() if value != expected}
    state_text = "使能" if expected_enabled else "释放"
    if wrong:
        log("WARN", f"{label} 扭矩状态异常（预期全部{state_text}）: {wrong}")
    else:
        log("OK", f"{label} 扭矩状态: 全部{state_text}")
    return states, None


def connect_follower_safely(follower: Any) -> None:
    """Connect the follower without ever enabling torque at a stale goal."""
    follower.bus.connect()
    for camera in follower.cameras.values():
        camera.connect()

    # Keep torque disabled until Goal_Position matches Present_Position.
    follower.bus.disable_torque()
    present = follower.bus.sync_read("Present_Position", normalize=False)
    follower.bus.sync_write("Goal_Position", present, normalize=False)
    follower.configure()
    log("OK", "Follower 已在当前位置使能扭矩（不会跳向旧目标）")


def confirm(message: str) -> None:
    """Require an interactive Enter confirmation."""
    try:
        input(message)
    except EOFError as exc:
        raise RuntimeError("需要交互确认，但 stdin 已关闭") from exc


def load_calibration_ranges(path: Path) -> dict[str, tuple[int, int]]:
    """Load range_min/range_max from a LeRobot calibration JSON."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"无法读取校准文件 {path}: {exc}") from exc

    ranges: dict[str, tuple[int, int]] = {}
    for motor, values in data.items():
        if not isinstance(values, dict):
            continue
        try:
            range_min = int(values["range_min"])
            range_max = int(values["range_max"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                f"校准文件 {path} 中 {motor} 缺少有效的 range_min/range_max"
            ) from exc
        if range_max < range_min:
            raise RuntimeError(f"校准文件 {path} 中 {motor} 的 range_max < range_min")
        ranges[motor] = (range_min, range_max)
    return ranges


def move_all_to_middle(
    follower: Any,
    args: argparse.Namespace,
    ranges: dict[str, tuple[int, int]],
) -> dict[str, Any]:
    """Move every follower joint to the raw midpoint of the calibration file."""
    bus = follower.bus
    motors = [motor for motor in MIDDLE_ORDER if motor in ranges]
    raw_targets = {
        motor: int(round((ranges[motor][0] + ranges[motor][1]) / 2))
        for motor in motors
    }
    raw_step = max(1, int(round(args.middle_step * 4096 / 360)))
    raw_tolerance = max(1, int(round(args.middle_tolerance * 4096 / 360)))
    log("INFO", f"中位目标为各舵机 raw 中点；单步 {raw_step}，容差 {raw_tolerance}")

    if not args.yes:
        confirm("确认周围无人、运动范围内无障碍后，按 Enter 开始移动到中位 ...")
    log("WARN", "开始中位运动；按 Ctrl-C 可中止（默认释放扭矩）。")

    deadline = time.monotonic() + args.middle_timeout
    last_log = 0.0
    while True:
        present = bus.sync_read("Present_Position", normalize=False)
        action: dict[str, int] = {}
        remaining: dict[str, int] = {}

        for motor in motors:
            current = int(present[motor])
            target = raw_targets[motor]
            error = target - current
            if abs(error) <= raw_tolerance:
                continue
            step = max(-raw_step, min(raw_step, error))
            action[motor] = current + step
            remaining[motor] = abs(error)

        if not action:
            break

        if time.monotonic() > deadline:
            summary = ", ".join(f"{motor}={value}" for motor, value in remaining.items())
            raise TimeoutError(
                f"中位回零在 {args.middle_timeout:.1f}s 内未完成，剩余 raw 误差: {summary}"
            )

        for motor, goal in action.items():
            bus.write("Goal_Position", motor, goal, normalize=False, num_retry=2)

        now = time.monotonic()
        if now - last_log >= 0.5:
            summary = ", ".join(f"{motor}={value}" for motor, value in remaining.items())
            log("INFO", f"  剩余 raw 误差: {summary}")
            last_log = now
        time.sleep(0.02)

    log("OK", "所有关节已到达中位")
    return {"raw_targets": raw_targets}


def release_device(device: Any, label: str, keep_torque: bool) -> None:
    """Release torque as requested and close the device."""
    if device is None:
        return
    try:
        connected = bool(device.is_connected)
    except Exception:
        connected = False

    if connected and not keep_torque:
        try:
            device.bus.disable_torque()
        except Exception as exc:
            log("WARN", f"{label} 释放扭矩失败: {type(exc).__name__}: {exc}")
    try:
        if connected:
            device.disconnect()
    except Exception as exc:
        log("WARN", f"{label} 断开失败: {type(exc).__name__}: {exc}")
        return

    if keep_torque:
        log("WARN", f"{label} 已断开，扭矩保持开启；请留意舵机发热")
    else:
        log("INFO", f"{label} 已断开，扭矩已释放")


def connect_follower(
    args: argparse.Namespace,
) -> tuple[Any, dict[str, Any]]:
    """Connect and initialize the follower, including calibration."""
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    section("Follower 初始化")
    config = SO101FollowerConfig(
        port=args.follower_port,
        id=args.follower_id,
        disable_torque_on_disconnect=False,
        use_degrees=args.use_degrees,
    )
    follower = SO101Follower(config)
    unit = JointUnit.DEGREE if args.use_degrees else JointUnit.NORMALIZED
    info: dict[str, Any] = {
        "port": args.follower_port,
        "id": args.follower_id,
        "calibrated": None,
        "calibration_file": None,
        "unit": unit.value,
        "connect_ms": None,
        "torque": None,
        "joints": {},
        "samples": [],
        "read_ms": None,
        "error": None,
    }

    try:
        log("STEP", f"连接 Follower {args.follower_port} (id={args.follower_id})")
        started = time.perf_counter()
        connect_follower_safely(follower)
        info["connect_ms"] = round((time.perf_counter() - started) * 1000.0, 3)
        log("OK", f"Follower 连接成功（{info['connect_ms']:.0f} ms）")

        try:
            calibrated = bool(follower.is_calibrated)
        except Exception:
            calibrated = None
        info["calibrated"] = calibrated
        info["calibration_file"] = str(follower.calibration_fpath)
        if calibrated is True:
            log("OK", "Follower 校准状态: 已校准")
        elif calibrated is False:
            log("WARN", "Follower 校准状态: 未校准；读数可能无效")
        else:
            log("WARN", "Follower 校准状态: 无法确认")
        log("INFO", f"校准文件: {follower.calibration_fpath}")

        states, _ = report_torque(follower, "Follower", expected_enabled=True)
        info["torque"] = states

        samples: list[JointState] = []
        latencies: list[float] = []
        if calibrated is False:
            log("WARN", "Follower 未校准，跳过归一化关节读数；请先完成校准")
        else:
            samples, latencies = read_joint_states(
                follower.get_observation,
                args.samples,
                unit=unit,
                torque_states=states,
            )
        statistics = joint_statistics(samples)
        info["joints"] = statistics
        info["samples"] = [s.to_dict() for s in samples]
        if latencies:
            info["read_ms"] = {
                "mean": round(sum(latencies) / len(latencies), 3),
                "min": round(min(latencies), 3),
                "max": round(max(latencies), 3),
            }

        if calibrated is not False:
            if not statistics:
                log("FAIL", "Follower 没有读到任何 *.pos 关节字段")
            else:
                # 使用 JointState 自带的防御性校验检查每一帧
                validation_errors = []
                for idx, sample in enumerate(samples):
                    is_valid, err = sample.validate(require_all_joints=False)
                    if not is_valid:
                        validation_errors.append(f"帧 #{idx+1}: {err}")

                if validation_errors:
                    log("FAIL", f"Follower 读数校验失败: {'; '.join(validation_errors)}")
                else:
                    log("OK", f"Follower 读数校验通过: {len(statistics)} 个关节 x {len(samples)} 次采样 (符合 JointState 规范)")
                print_joint_table(statistics, unit.value)
                if info["read_ms"]:
                    log(
                        "INFO",
                        "Follower 读取耗时: "
                        f"mean={info['read_ms']['mean']:.1f} ms, "
                        f"min={info['read_ms']['min']:.1f} ms, "
                        f"max={info['read_ms']['max']:.1f} ms",
                    )
        return follower, info
    except BaseException as exc:
        info["error"] = f"{type(exc).__name__}: {exc}"
        release_device(follower, "Follower", keep_torque=False)
        raise


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(
        description=(
            "SO-ARM101 从臂初始化：连接、读取校准文件、使能并回中位"
            "（会操作真实硬件）"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--port",
        "--follower-port",
        dest="follower_port",
        default="/dev/ttyACM0",
        help="Follower 串口（默认 /dev/ttyACM0）",
    )
    parser.add_argument(
        "--follower-id",
        default="zihao_follower_arm",
        help="Follower 校准 id（默认 zihao_follower_arm）",
    )
    parser.add_argument(
        "--calibration-file",
        type=Path,
        default=FIXED_CALIBRATION_FILE,
        help="Follower 校准 JSON 文件（默认 zihao_follower_arm）",
    )
    parser.add_argument(
        "--keep-torque-on-exit",
        action="store_true",
        help="脚本退出后保持 Follower 扭矩（默认释放）",
    )
    parser.add_argument(
        "--release-torque-on-exit",
        action="store_true",
        help="退出时释放 Follower 扭矩（机械臂会下垂）",
    )
    parser.add_argument(
        "--middle-step",
        type=float,
        default=3.0,
        help="中位回零的单步最大增量（默认 3.0；--no-degrees 时为归一化单位）",
    )
    parser.add_argument(
        "--middle-tolerance",
        type=float,
        default=1.0,
        help="中位回零的到位容差（默认 1.0）",
    )
    parser.add_argument(
        "--middle-timeout",
        type=float,
        default=60.0,
        help="所有关节回到中位的总超时时间，单位秒（默认 60）",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过移动到中位前的交互确认",
    )
    parser.add_argument("--samples", type=int, default=3, help="每个设备读取次数（默认 3）")
    parser.add_argument(
        "--no-degrees",
        dest="use_degrees",
        action="store_false",
        help="使用归一化关节值而不是角度",
    )
    parser.add_argument("--no-pause", action="store_true", help="初始化后不等待 Enter，直接断开")
    parser.add_argument("--no-countdown", action="store_true", help="开始前不等待 3 秒")
    parser.add_argument("--json-out", type=Path, help="保存初始化结果到 JSON 文件")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the follower initialization and return a process exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 100:
        parser.error("--samples 必须在 1 到 100 之间")
    if args.middle_step <= 0:
        parser.error("--middle-step 必须大于 0")
    if args.middle_tolerance < 0:
        parser.error("--middle-tolerance 不能小于 0")
    if args.middle_timeout <= 0:
        parser.error("--middle-timeout 必须大于 0")
    if args.keep_torque_on_exit and args.release_torque_on_exit:
        parser.error("--keep-torque-on-exit 与 --release-torque-on-exit 不能同时使用")

    ensure_repo_on_path()
    section("SO-ARM101 从臂初始化")

    if not module_available("lerobot"):
        log("FAIL", "找不到 lerobot；请在仓库根目录执行 uv sync，或先激活虚拟环境")
        return 1
    if sys.version_info < (3, 12):
        log("FAIL", f"Python {platform.python_version()}；LeRobot 0.6.x 需要 Python >= 3.12")
        return 1

    required = {
        "numpy": "uv sync --locked",
        "draccus": "uv sync --locked",
        "serial": "uv sync --locked --extra hardware",
        "scservo_sdk": "uv sync --locked --extra feetech",
    }
    missing = [f"{module} ({hint})" for module, hint in required.items() if not module_available(module)]
    if missing:
        log("FAIL", "缺少硬件依赖: " + ", ".join(missing))
        return 1

    log("INFO", f"从臂端口: {args.follower_port} | 校准 id: {args.follower_id}")
    log("WARN", "即将连接真实机械臂。请托住手臂，确认周围无人且姿态安全。")
    log("WARN", "Follower 初始化后会使能扭矩并保持当前姿态。")
    log("INFO", "连接后会把目标位置同步到当前姿态，再执行保持。")
    log("INFO", "初始化完成后会把各关节移到中位（来自校准文件）。")
    if args.keep_torque_on_exit:
        log("WARN", "退出后 Follower 扭矩仍保持开启，请留意舵机发热。")
    else:
        log("INFO", "退出时释放 Follower 扭矩。")

    if not args.no_countdown:
        countdown(3)

    follower = None
    follower_info: dict[str, Any] | None = None
    exit_code = 0

    try:
        follower, follower_info = connect_follower(args)

        calibration_file = args.calibration_file
        log("INFO", f"中位回零使用校准文件: {calibration_file}")
        ranges = load_calibration_ranges(calibration_file)
        follower_info["middle"] = move_all_to_middle(follower, args, ranges)

        section("初始化完成")
        state = "已校准" if follower_info.get("calibrated") else "未确认校准"
        torque = follower_info.get("torque") or {}
        if torque and all(value == 1 for value in torque.values()):
            log("OK", f"Follower: {state}，扭矩已使能")
        else:
            log("WARN", f"Follower: {state}，扭矩状态需检查")

        if not args.no_pause:
            if args.keep_torque_on_exit:
                prompt_enter("\n按 Enter 结束并断开（从臂将保持扭矩）...")
            else:
                prompt_enter("\n按 Enter 结束并断开（从臂将释放扭矩）...")
    except KeyboardInterrupt:
        log("WARN", "用户取消")
        exit_code = 130
    except Exception as exc:
        log("FAIL", f"{type(exc).__name__}: {exc}")
        exit_code = 1
    finally:
        section("断开")
        release_device(follower, "Follower", keep_torque=args.keep_torque_on_exit)

        if args.json_out:
            report = {
                "completed_at": datetime.now(timezone.utc).isoformat(),
                "follower": follower_info,
                "keep_torque_on_exit": args.keep_torque_on_exit,
            }
            try:
                args.json_out.parent.mkdir(parents=True, exist_ok=True)
                args.json_out.write_text(
                    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                print(f"\n报告已写入: {args.json_out}", flush=True)
            except Exception as exc:
                log("WARN", f"报告写入失败: {type(exc).__name__}: {exc}")
                exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
