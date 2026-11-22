"""SO-ARM101 硬件层配置定义.

提供机械臂硬件连接参数、跨平台官方校准路径解析 (HF_LEROBOT_CALIBRATION / HF_LEROBOT_HOME / .env)、
角色定义 (Leader / Follower) 以及系统级多臂协同配置.
"""

from __future__ import annotations

import os
import platform
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from core.data_types import JointUnit

_WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
_UNSET: object = object()


def _read_workspace_env() -> dict[str, str]:
    """解析工作区根目录 .env 文件 (不修改全局 os.environ)."""
    env_file = _WORKSPACE_ROOT / ".env"
    env_map: dict[str, str] = {}
    if env_file.is_file():
        try:
            for raw_line in env_file.read_text(encoding="utf-8").splitlines():
                line = raw_line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    env_map[k.strip()] = v.strip().strip('"').strip("'")
        except Exception:
            return {}
    return env_map


def _env_or_dotenv(key: str) -> str:
    val = os.environ.get(key)
    if val is not None and val.strip():
        return val.strip()
    return (_read_workspace_env().get(key) or "").strip()


def _resolve_path_under_workspace(raw: str | Path) -> Path:
    p = Path(raw).expanduser()
    if not p.is_absolute():
        p = (_WORKSPACE_ROOT / p).resolve()
    return p


def resolve_calibration_root() -> Path:
    """按官方优先级解析校准根目录 (HF_LEROBOT_CALIBRATION > HF_LEROBOT_HOME/calibration > workspace/data/calibration > HF_HOME)."""
    cal_env = _env_or_dotenv("HF_LEROBOT_CALIBRATION")
    if cal_env:
        return _resolve_path_under_workspace(cal_env)

    home_env = _env_or_dotenv("HF_LEROBOT_HOME")
    if home_env:
        return _resolve_path_under_workspace(home_env) / "calibration"

    ws_cal = _WORKSPACE_ROOT / "data" / "calibration"
    if ws_cal.is_dir() or (_WORKSPACE_ROOT / ".env").is_file():
        return ws_cal.resolve()

    try:
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION
        return Path(HF_LEROBOT_CALIBRATION)
    except Exception:
        hf_home = _env_or_dotenv("HF_HOME")
        base_hf = _resolve_path_under_workspace(hf_home) if hf_home else (Path.home() / ".cache" / "huggingface")
        return base_hf / "lerobot" / "calibration"


class ArmRole(str, Enum):
    """机械臂角色类型."""
    FOLLOWER = "follower"   # 执行从臂: 使能扭矩，执行动作与保持
    LEADER = "leader"       # 示教主臂: 释放扭矩，用于人工拖动遥操作示教


def resolve_calibration_dir(
    role: ArmRole | str,
    override_dir: Path | str | None = None,
    robot_id: str | None = None,
) -> Path:
    """解析对应角色 (follower/leader) 的叶子校准目录 (兼容传入根目录或叶子目录)."""
    role_val = role.value if isinstance(role, ArmRole) else str(role).lower().strip()
    if role_val == "follower":
        leaf_parts = ("robots", "so_follower")
    elif role_val == "leader":
        leaf_parts = ("teleoperators", "so_leader")
    else:
        raise ValueError(f"未知机械臂角色: {role}")

    if override_dir is not None:
        p = _resolve_path_under_workspace(override_dir)
        has_direct_json = bool(robot_id and (p / f"{robot_id}.json").is_file())
        if (
            not has_direct_json
            and p.name not in ("so_follower", "so_leader")
            and (p.name == "calibration" or (p / "robots").is_dir() or (p / "teleoperators").is_dir())
        ):
            return p.joinpath(*leaf_parts)
        return p

    return resolve_calibration_root().joinpath(*leaf_parts)


def _default_follower_id_and_path() -> tuple[str, Path]:
    cal_dir = resolve_calibration_dir(ArmRole.FOLLOWER)
    for cand in ("arm101_follower_01", "zihao_follower_arm", "follower_arm"):
        if (cal_dir / f"{cand}.json").is_file():
            return cand, cal_dir / f"{cand}.json"
    return "arm101_follower_01", cal_dir / "arm101_follower_01.json"


def _default_leader_id_and_path() -> tuple[str, Path]:
    cal_dir = resolve_calibration_dir(ArmRole.LEADER)
    for cand in ("arm101_leader_01", "zihao_leader_arm", "leader_arm"):
        if (cal_dir / f"{cand}.json").is_file():
            return cand, cal_dir / f"{cand}.json"
    return "arm101_leader_01", cal_dir / "arm101_leader_01.json"


DEFAULT_FOLLOWER_CALIBRATION: Path = _default_follower_id_and_path()[1]
DEFAULT_LEADER_CALIBRATION: Path = _default_leader_id_and_path()[1]


@dataclass(frozen=True)
class ArmConfig:
    """单条机械臂的硬件配置参数."""
    role: ArmRole
    port: str
    robot_id: str
    calibration_fpath: Path = field(default_factory=lambda: DEFAULT_FOLLOWER_CALIBRATION)
    calibration_dir: Path | None = None
    use_degrees: bool = True
    internal_unit: JointUnit = JointUnit.RADIAN
    baudrate: int = 1_000_000
    timeout_s: float = 1.0
    is_mock: bool = False
    joint_offsets_deg: dict[str, float] = field(default_factory=dict)
    max_relative_target: float | dict[str, float] | None = None

    def __post_init__(self) -> None:
        fpath = Path(self.calibration_fpath)
        object.__setattr__(self, "calibration_fpath", fpath)
        if self.calibration_dir is None:
            object.__setattr__(self, "calibration_dir", fpath.parent)
        else:
            object.__setattr__(self, "calibration_dir", Path(self.calibration_dir))

    @classmethod
    def default_follower(
        cls,
        port: str | None = None,
        robot_id: str | None = None,
        calibration_fpath: Path | None = None,
        internal_unit: JointUnit = JointUnit.RADIAN,
        is_mock: bool = False,
        calibration_dir: Path | str | None = None,
        max_relative_target: Any = _UNSET,
    ) -> ArmConfig:
        """生成默认从臂配置 (优先读取 config.yaml 与 .env / HF_LEROBOT_CALIBRATION)."""
        target_port = port
        target_id = robot_id
        baudrate = 1_000_000
        cfg_cal_dir: str | None = None
        cfg_max_step: float | None = 15.0
        try:
            from core.config_loader import get_config
            cfg = get_config()
            if target_port is None:
                target_port = cfg.follower.port
            if target_id is None:
                target_id = cfg.follower.id
            baudrate = cfg.follower.baudrate
            cfg_cal_dir = getattr(cfg.follower, "calibration_dir", None)
            if hasattr(cfg, "safety") and hasattr(cfg.safety, "max_step_deg"):
                cfg_max_step = float(cfg.safety.max_step_deg)
        except Exception:
            pass

        if target_port is None:
            target_port = "COM3" if platform.system() == "Windows" else "/dev/ttyACM0"

        if calibration_dir is not None:
            cal_dir = resolve_calibration_dir(ArmRole.FOLLOWER, override_dir=calibration_dir, robot_id=target_id)
        elif calibration_fpath is not None:
            cal_dir = Path(calibration_fpath).parent
        elif cfg_cal_dir and not _env_or_dotenv("HF_LEROBOT_CALIBRATION"):
            cal_dir = resolve_calibration_dir(ArmRole.FOLLOWER, override_dir=cfg_cal_dir, robot_id=target_id)
        else:
            cal_dir = resolve_calibration_dir(ArmRole.FOLLOWER, robot_id=target_id)

        if target_id is None:
            found_id = None
            for cand in ("arm101_follower_01", "zihao_follower_arm", "follower_arm"):
                if (cal_dir / f"{cand}.json").is_file():
                    found_id = cand
                    break
            if found_id is None and cal_dir.is_dir():
                found = [f for f in cal_dir.glob("*.json") if f.is_file()]
                if found:
                    found_id = found[0].stem
            target_id = found_id or "arm101_follower_01"

        resolved_fpath = Path(calibration_fpath) if calibration_fpath is not None else (cal_dir / f"{target_id}.json")
        resolved_max_rel = cfg_max_step if max_relative_target is _UNSET else max_relative_target

        return cls(
            role=ArmRole.FOLLOWER,
            port=target_port,
            robot_id=target_id,
            calibration_fpath=resolved_fpath,
            calibration_dir=cal_dir,
            use_degrees=True,
            internal_unit=internal_unit,
            baudrate=baudrate,
            is_mock=is_mock,
            max_relative_target=resolved_max_rel,
        )

    @classmethod
    def mock_follower(
        cls,
        robot_id: str = "mock_follower_arm",
        internal_unit: JointUnit = JointUnit.RADIAN,
        calibration_dir: Path | str | None = None,
        calibration_fpath: Path | None = None,
    ) -> ArmConfig:
        """生成仿真/Mock 从臂配置."""
        cal_dir = resolve_calibration_dir(ArmRole.FOLLOWER, override_dir=calibration_dir, robot_id=robot_id)
        resolved_fpath = Path(calibration_fpath) if calibration_fpath is not None else (cal_dir / f"{robot_id}.json")
        return cls(
            role=ArmRole.FOLLOWER,
            port="mock",
            robot_id=robot_id,
            calibration_fpath=resolved_fpath,
            calibration_dir=cal_dir if calibration_fpath is None else resolved_fpath.parent,
            use_degrees=True,
            internal_unit=internal_unit,
            is_mock=True,
        )

    @classmethod
    def default_leader(
        cls,
        port: str | None = None,
        robot_id: str | None = None,
        calibration_fpath: Path | None = None,
        internal_unit: JointUnit = JointUnit.RADIAN,
        is_mock: bool = False,
        calibration_dir: Path | str | None = None,
        max_relative_target: float | dict[str, float] | None = None,
    ) -> ArmConfig:
        """生成默认主臂配置 (优先读取 config.yaml 与 .env / HF_LEROBOT_CALIBRATION)."""
        target_port = port
        target_id = robot_id
        baudrate = 1_000_000
        joint_offsets: dict[str, float] = {}
        cfg_cal_dir: str | None = None
        try:
            from core.config_loader import get_config
            cfg = get_config()
            if target_port is None:
                target_port = cfg.leader.port
            if target_id is None:
                target_id = cfg.leader.id
            baudrate = cfg.leader.baudrate
            joint_offsets = dict(getattr(cfg.leader, "joint_offsets_deg", {}))
            cfg_cal_dir = getattr(cfg.leader, "calibration_dir", None)
        except Exception:
            pass

        if target_port is None:
            target_port = "COM4" if platform.system() == "Windows" else "/dev/ttyACM1"

        if calibration_dir is not None:
            cal_dir = resolve_calibration_dir(ArmRole.LEADER, override_dir=calibration_dir, robot_id=target_id)
        elif calibration_fpath is not None:
            cal_dir = Path(calibration_fpath).parent
        elif cfg_cal_dir and not _env_or_dotenv("HF_LEROBOT_CALIBRATION"):
            cal_dir = resolve_calibration_dir(ArmRole.LEADER, override_dir=cfg_cal_dir, robot_id=target_id)
        else:
            cal_dir = resolve_calibration_dir(ArmRole.LEADER, robot_id=target_id)

        if target_id is None:
            found_id = None
            for cand in ("arm101_leader_01", "zihao_leader_arm", "leader_arm"):
                if (cal_dir / f"{cand}.json").is_file():
                    found_id = cand
                    break
            if found_id is None and cal_dir.is_dir():
                found = [f for f in cal_dir.glob("*.json") if f.is_file()]
                if found:
                    found_id = found[0].stem
            target_id = found_id or "arm101_leader_01"

        resolved_fpath = Path(calibration_fpath) if calibration_fpath is not None else (cal_dir / f"{target_id}.json")

        return cls(
            role=ArmRole.LEADER,
            port=target_port,
            robot_id=target_id,
            calibration_fpath=resolved_fpath,
            calibration_dir=cal_dir,
            use_degrees=True,
            internal_unit=internal_unit,
            baudrate=baudrate,
            is_mock=is_mock,
            joint_offsets_deg=joint_offsets,
            max_relative_target=max_relative_target,
        )

    @classmethod
    def mock_leader(
        cls,
        robot_id: str = "mock_leader_arm",
        internal_unit: JointUnit = JointUnit.RADIAN,
        calibration_dir: Path | str | None = None,
        calibration_fpath: Path | None = None,
    ) -> ArmConfig:
        """生成仿真/Mock 示教主臂配置."""
        cal_dir = resolve_calibration_dir(ArmRole.LEADER, override_dir=calibration_dir, robot_id=robot_id)
        resolved_fpath = Path(calibration_fpath) if calibration_fpath is not None else (cal_dir / f"{robot_id}.json")
        return cls(
            role=ArmRole.LEADER,
            port="mock",
            robot_id=robot_id,
            calibration_fpath=resolved_fpath,
            calibration_dir=cal_dir if calibration_fpath is None else resolved_fpath.parent,
            use_degrees=True,
            internal_unit=internal_unit,
            is_mock=True,
        )


@dataclass(frozen=True)
class RobotSystemConfig:
    """机器人整机系统硬件配置 (从臂 + 可选主臂)."""
    follower: ArmConfig
    leader: ArmConfig | None = None
    frequency_hz: float = 30.0
    use_mock: bool = False

    @classmethod
    def mock_system(
        cls,
        with_leader: bool = False,
        internal_unit: JointUnit = JointUnit.RADIAN,
    ) -> RobotSystemConfig:
        """生成全仿真/Mock 机器人系统配置."""
        follower = ArmConfig.mock_follower(internal_unit=internal_unit)
        leader = ArmConfig.mock_leader(internal_unit=internal_unit) if with_leader else None
        return cls(
            follower=follower,
            leader=leader,
            frequency_hz=30.0,
            use_mock=True,
        )

    @classmethod
    def real_system(
        cls,
        follower_port: str | None = None,
        leader_port: str | None = None,
        frequency_hz: float | None = None,
        follower_calibration: Path | None = None,
        leader_calibration: Path | None = None,
        internal_unit: JointUnit = JointUnit.RADIAN,
        follower_id: str | None = None,
        leader_id: str | None = None,
        calibration_dir: Path | str | None = None,
        max_relative_target: Any = _UNSET,
    ) -> RobotSystemConfig:
        """生成真机硬件机器人系统配置 (Follower 从臂 + 可选 Leader 主臂)."""
        freq = frequency_hz
        if freq is None:
            try:
                from core.config_loader import get_config
                freq = float(get_config().operation.fps)
            except Exception:
                freq = 30.0

        follower = ArmConfig.default_follower(
            port=follower_port,
            robot_id=follower_id,
            calibration_fpath=follower_calibration,
            internal_unit=internal_unit,
            is_mock=False,
            calibration_dir=calibration_dir,
            max_relative_target=max_relative_target,
        )
        leader = (
            ArmConfig.default_leader(
                port=leader_port,
                robot_id=leader_id,
                calibration_fpath=leader_calibration,
                internal_unit=internal_unit,
                is_mock=False,
                calibration_dir=calibration_dir,
            )
            if leader_port
            else None
        )
        return cls(
            follower=follower,
            leader=leader,
            frequency_hz=freq,
            use_mock=False,
        )
