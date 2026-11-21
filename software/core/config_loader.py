"""SO-ARM101 全局配置装载器 (Config Loader).

负责解析 software/config.yaml 文件，提供强类型参数对象与安全的全局降级默认值。
支持运行时动态重载与命令行参数优先级覆盖。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

_software_root = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = _software_root / "config.yaml"


@dataclass
class FollowerConfig:
    """从臂 (Follower Arm) 配置."""
    port: str = "/dev/ttyACM0"
    id: str = "zihao_follower_arm"
    type: str = "so101_follower"
    baudrate: int = 1_000_000


@dataclass
class LeaderConfig:
    """主臂 (Leader Arm) 配置."""
    port: str = "/dev/ttyACM1"
    id: str = "zihao_leader_arm"
    type: str = "so_leader"
    baudrate: int = 1_000_000
    joint_offsets_deg: dict[str, float] = field(default_factory=dict)


@dataclass
class CameraConfig:
    """相机 (Camera) 配置."""
    port: str | int = "/dev/video2"
    width: int = 640
    height: int = 480
    fps: int = 30
    format: str = "MJPG"

    @property
    def index_or_path(self) -> int | str:
        """解析为 OpenCV / LeRobot 兼容的端口索引或设备路径."""
        val = str(self.port).strip()
        if val.isdigit():
            return int(val)
        if val.startswith("/dev/video"):
            suffix = val.replace("/dev/video", "")
            if suffix.isdigit():
                return int(suffix)
        return val


@dataclass
class SafetyConfig:
    """安全防护与姿态 (Safety) 配置."""
    rest_pose_file: str = "rest_pose.json"
    home_pose_file: str = "upright_pose.json"
    auto_rest_on_exit: bool = True
    rest_duration_s: float = 2.0
    max_speed_deg_per_s: float = 180.0
    max_step_deg: float = 15.0

    @property
    def rest_pose_path(self) -> Path:
        """获取蜷缩姿态绝对路径."""
        p = Path(self.rest_pose_file)
        if p.is_absolute():
            return p
        return _software_root / p

    @property
    def home_pose_path(self) -> Path:
        """获取中位姿态绝对路径."""
        p = Path(self.home_pose_file)
        if p.is_absolute():
            return p
        return _software_root / p


@dataclass
class OperationConfig:
    """遥操作、录制与回放 (Operation) 配置."""
    fps: int = 30
    dataset_repo_id: str = "local/so101_demo"
    display: bool = True
    display_mode: str = "rerun"
    default_replay_episode: int = 0


@dataclass
class SystemConfig:
    """全局系统配置聚合."""
    follower: FollowerConfig = field(default_factory=FollowerConfig)
    leader: LeaderConfig = field(default_factory=LeaderConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    operation: OperationConfig = field(default_factory=OperationConfig)

    def to_dict(self) -> dict[str, Any]:
        """导出为普通字典."""
        import dataclasses
        return dataclasses.asdict(self)


_CACHED_CONFIG: SystemConfig | None = None


def _parse_yaml(file_path: Path) -> dict[str, Any]:
    """尝试通过 PyYAML 解析配置文件，失败时回退为空字典."""
    if not file_path.is_file():
        return {}
    try:
        import yaml
        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            return data if isinstance(data, dict) else {}
    except ImportError:
        logger.warning("未检测到 PyYAML 依赖，回退到系统内置默认配置")
        return {}
    except Exception as exc:
        logger.warning(f"解析配置文件异常 ({file_path}): {exc}，使用默认配置")
        return {}


def load_config(config_path: Path | str | None = None) -> SystemConfig:
    """从指定路径或默认 software/config.yaml 装载配置."""
    path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
    raw = _parse_yaml(path)

    follower_raw = raw.get("follower", {}) or {}
    leader_raw = raw.get("leader", {}) or {}
    camera_raw = raw.get("camera", {}) or {}
    safety_raw = raw.get("safety", {}) or {}
    operation_raw = raw.get("operation", {}) or {}

    cfg = SystemConfig(
        follower=FollowerConfig(
            port=str(follower_raw.get("port", FollowerConfig.port)),
            id=str(follower_raw.get("id", FollowerConfig.id)),
            type=str(follower_raw.get("type", FollowerConfig.type)),
            baudrate=int(follower_raw.get("baudrate", FollowerConfig.baudrate)),
        ),
        leader=LeaderConfig(
            port=str(leader_raw.get("port", LeaderConfig.port)),
            id=str(leader_raw.get("id", LeaderConfig.id)),
            type=str(leader_raw.get("type", LeaderConfig.type)),
            baudrate=int(leader_raw.get("baudrate", LeaderConfig.baudrate)),
            joint_offsets_deg={
                str(k): float(v)
                for k, v in (leader_raw.get("joint_offsets_deg") or {}).items()
            },
        ),
        camera=CameraConfig(
            port=camera_raw.get("port", CameraConfig.port),
            width=int(camera_raw.get("width", CameraConfig.width)),
            height=int(camera_raw.get("height", CameraConfig.height)),
            fps=int(camera_raw.get("fps", CameraConfig.fps)),
            format=str(camera_raw.get("format", CameraConfig.format)),
        ),
        safety=SafetyConfig(
            rest_pose_file=str(safety_raw.get("rest_pose_file", SafetyConfig.rest_pose_file)),
            auto_rest_on_exit=bool(safety_raw.get("auto_rest_on_exit", SafetyConfig.auto_rest_on_exit)),
            rest_duration_s=float(safety_raw.get("rest_duration_s", SafetyConfig.rest_duration_s)),
            max_speed_deg_per_s=float(safety_raw.get("max_speed_deg_per_s", SafetyConfig.max_speed_deg_per_s)),
            max_step_deg=float(safety_raw.get("max_step_deg", SafetyConfig.max_step_deg)),
        ),
        operation=OperationConfig(
            fps=int(operation_raw.get("fps", OperationConfig.fps)),
            dataset_repo_id=str(operation_raw.get("dataset_repo_id", OperationConfig.dataset_repo_id)),
            display=bool(operation_raw.get("display", OperationConfig.display)),
            display_mode=str(operation_raw.get("display_mode", OperationConfig.display_mode)),
            default_replay_episode=int(operation_raw.get("default_replay_episode", OperationConfig.default_replay_episode)),
        ),
    )
    return cfg


def get_config(reload: bool = False) -> SystemConfig:
    """获取单例配置对象 (带全局缓存)."""
    global _CACHED_CONFIG
    if _CACHED_CONFIG is None or reload:
        _CACHED_CONFIG = load_config()
    return _CACHED_CONFIG

