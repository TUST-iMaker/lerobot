"""SO-ARM101 硬件层配置定义.

提供机械臂硬件连接参数、校准文件路径、角色定义 (Leader / Follower) 以及系统级多臂协同配置.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

from core.data_types import JointUnit

DEFAULT_FOLLOWER_CALIBRATION = Path(
    "/home/aguiya/.cache/huggingface/lerobot/calibration/robots/so_follower/zihao_follower_arm.json"
)
DEFAULT_LEADER_CALIBRATION = Path(
    "/home/aguiya/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/zihao_leader_arm.json"
)


class ArmRole(str, Enum):
    """机械臂角色类型."""
    FOLLOWER = "follower"   # 执行从臂: 使能扭矩，执行动作与保持
    LEADER = "leader"       # 示教主臂: 释放扭矩，用于人工拖动遥操作示教


@dataclass(frozen=True)
class ArmConfig:
    """单条机械臂的硬件配置参数."""
    role: ArmRole
    port: str
    robot_id: str
    calibration_fpath: Path = field(default_factory=lambda: DEFAULT_FOLLOWER_CALIBRATION)
    use_degrees: bool = True
    internal_unit: JointUnit = JointUnit.RADIAN
    baudrate: int = 1_000_000
    timeout_s: float = 1.0
    is_mock: bool = False
    joint_offsets_deg: dict[str, float] = field(default_factory=dict)

    @classmethod
    def default_follower(
        cls,
        port: str | None = None,
        robot_id: str | None = None,
        calibration_fpath: Path = DEFAULT_FOLLOWER_CALIBRATION,
        internal_unit: JointUnit = JointUnit.RADIAN,
        is_mock: bool = False,
    ) -> ArmConfig:
        """生成默认从臂配置 (优先读取 config.yaml)."""
        target_port = port
        target_id = robot_id
        baudrate = 1_000_000
        try:
            from core.config_loader import get_config
            cfg = get_config()
            if target_port is None:
                target_port = cfg.follower.port
            if target_id is None:
                target_id = cfg.follower.id
            baudrate = cfg.follower.baudrate
        except Exception:
            pass
        if target_port is None:
            target_port = "/dev/ttyACM0"

        cal_dir = Path("/home/aguiya/.cache/huggingface/lerobot/calibration/robots/so_follower")
        if target_id is None:
            if (cal_dir / "zihao_follower_arm.json").is_file():
                target_id = "zihao_follower_arm"
            elif (cal_dir / "follower_arm.json").is_file():
                target_id = "follower_arm"
            else:
                target_id = "zihao_follower_arm"

        if calibration_fpath is None or calibration_fpath == DEFAULT_FOLLOWER_CALIBRATION:
            candidate = cal_dir / f"{target_id}.json"
            calibration_fpath = candidate if candidate.is_file() else DEFAULT_FOLLOWER_CALIBRATION

        return cls(
            role=ArmRole.FOLLOWER,
            port=target_port,
            robot_id=target_id,
            calibration_fpath=Path(calibration_fpath),
            use_degrees=True,
            internal_unit=internal_unit,
            baudrate=baudrate,
            is_mock=is_mock,
        )

    @classmethod
    def mock_follower(
        cls,
        robot_id: str = "mock_follower_arm",
        internal_unit: JointUnit = JointUnit.RADIAN,
    ) -> ArmConfig:
        """生成仿真/Mock 从臂配置."""
        return cls(
            role=ArmRole.FOLLOWER,
            port="mock",
            robot_id=robot_id,
            calibration_fpath=DEFAULT_FOLLOWER_CALIBRATION,
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
    ) -> ArmConfig:
        """生成默认主臂配置 (优先读取 config.yaml，自动适配 leader_arm 或 zihao_leader_arm 等本地校准文件)."""
        target_port = port
        target_id = robot_id
        baudrate = 1_000_000
        joint_offsets = {}
        try:
            from core.config_loader import get_config
            cfg = get_config()
            if target_port is None:
                target_port = cfg.leader.port
            if target_id is None:
                target_id = cfg.leader.id
            baudrate = cfg.leader.baudrate
            joint_offsets = dict(getattr(cfg.leader, "joint_offsets_deg", {}))
        except Exception:
            pass
        if target_port is None:
            target_port = "/dev/ttyACM1"

        cal_dir = Path("/home/aguiya/.cache/huggingface/lerobot/calibration/teleoperators/so_leader")
        if target_id is None:
            if (cal_dir / "zihao_leader_arm.json").is_file():
                target_id = "zihao_leader_arm"
            elif (cal_dir / "leader_arm.json").is_file():
                target_id = "leader_arm"
            elif cal_dir.is_dir():
                found = [f for f in cal_dir.glob("*.json") if f.is_file()]
                target_id = found[0].stem if found else "leader_arm"
            else:
                target_id = "leader_arm"

        if calibration_fpath is None:
            candidate = cal_dir / f"{target_id}.json"
            calibration_fpath = candidate if candidate.is_file() else DEFAULT_LEADER_CALIBRATION

        return cls(
            role=ArmRole.LEADER,
            port=target_port,
            robot_id=target_id,
            calibration_fpath=Path(calibration_fpath),
            use_degrees=True,
            internal_unit=internal_unit,
            baudrate=baudrate,
            is_mock=is_mock,
            joint_offsets_deg=joint_offsets,
        )

    @classmethod
    def mock_leader(
        cls,
        robot_id: str = "mock_leader_arm",
        internal_unit: JointUnit = JointUnit.RADIAN,
    ) -> ArmConfig:
        """生成仿真/Mock 示教主臂配置."""
        return cls(
            role=ArmRole.LEADER,
            port="mock",
            robot_id=robot_id,
            calibration_fpath=DEFAULT_LEADER_CALIBRATION,
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
        follower_calibration: Path = DEFAULT_FOLLOWER_CALIBRATION,
        leader_calibration: Path = DEFAULT_LEADER_CALIBRATION,
        internal_unit: JointUnit = JointUnit.RADIAN,
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
            calibration_fpath=follower_calibration,
            internal_unit=internal_unit,
            is_mock=False,
        )
        leader = (
            ArmConfig.default_leader(
                port=leader_port,
                calibration_fpath=leader_calibration,
                internal_unit=internal_unit,
                is_mock=False,
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

