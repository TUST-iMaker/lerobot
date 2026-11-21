"""Cross-platform configuration and workspace path resolution for Arm101 Studio."""

from __future__ import annotations

import os
import platform
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict

try:
    from dotenv import load_dotenv
except ImportError:  # pragma: no cover - python-dotenv is optional at runtime
    load_dotenv = None


ModeType = Literal["auto", "real", "dry_run"]


def resolve_workspace_root() -> Path:
    """Resolve the LeRobot workspace root directory."""
    env_root = os.environ.get("LEROBOT_WORKSPACE_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()
    return Path(__file__).resolve().parents[2]


def _resolve_under_root(root: Path, raw_path: str, default_rel: str) -> Path:
    val = (raw_path or "").strip() or default_rel
    p = Path(val).expanduser()
    if p.is_absolute():
        return p.resolve()
    return (root / p).resolve()


class StudioConfig(BaseModel):
    """Runtime configuration for Arm101 Studio."""

    model_config = ConfigDict(arbitrary_types_allowed=True)

    workspace_root: Path
    data_dir: Path
    datasets_dir: Path
    calibration_dir: Path
    dry_run_dir: Path
    logs_dir: Path
    configs_dir: Path
    repos_dir: Path
    frontend_dir: Path

    host: str = "127.0.0.1"
    port: int = 8760
    requested_mode: ModeType = "auto"

    follower_port: str
    leader_port: str
    follower_id: str = "arm101_follower_01"
    leader_id: str = "arm101_leader_01"
    front_camera_id: str = "0"
    wrist_camera_id: str = "1"

    def ensure_directories(self) -> None:
        """Ensure runtime data and log directories exist."""
        for path in (
            self.data_dir,
            self.datasets_dir,
            self.calibration_dir,
            self.dry_run_dir,
            self.dry_run_dir / "datasets",
            self.dry_run_dir / "calibration",
            self.logs_dir,
        ):
            path.mkdir(parents=True, exist_ok=True)

    def resolve_robot_profile_path(self) -> Path | None:
        """Return the OS-appropriate robot config file (user override or example)."""
        is_win = platform.system() == "Windows"
        candidates = (
            ("arm101_win.yaml", "arm101_win.example.yaml")
            if is_win
            else ("arm101_ubuntu.yaml", "arm101_ubuntu.example.yaml")
        )
        robots_dir = self.configs_dir / "robots"
        for name in candidates:
            p = robots_dir / name
            if p.exists():
                return p
        return None

    def load_robot_profile(self) -> dict[str, Any]:
        """Load the OS-appropriate robot profile YAML."""
        profile_path = self.resolve_robot_profile_path()
        if profile_path is None:
            return {}
        try:
            loaded = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                loaded["_profile_file"] = str(profile_path.relative_to(self.workspace_root))
                return loaded
        except Exception:
            pass
        return {}

    def load_vla_configs(self) -> list[dict[str, Any]]:
        """Read versioned VLA/training YAML profiles from configs/train/."""
        train_dir = self.configs_dir / "train"
        specs = [
            {
                "id": "act_so101",
                "file": "act_so101.yaml",
                "title": "ACT (Action Chunking with Transformers)",
                "vram": "~6–10 GB (RTX 3060 / 4060 / 4090)",
                "platform": "本地 Ubuntu 或云端单卡",
                "command": (
                    "lerobot-train --config_path=configs/train/act_so101.yaml "
                    "--dataset.repo_id=${HF_USER}/arm101_task"
                ),
            },
            {
                "id": "smolvla_so101_cloud",
                "file": "smolvla_so101_cloud.yaml",
                "title": "SmolVLA (450M 轻量级 VLA 微调)",
                "vram": ">= 16–24 GB (A10G / L4 / RTX 4090)",
                "platform": "云端算力 (AutoDL / Featurize / HF Jobs)",
                "command": (
                    "lerobot-train --config_path=configs/train/smolvla_so101_cloud.yaml "
                    "--dataset.repo_id=${HF_USER}/arm101_task"
                ),
            },
            {
                "id": "pi0_so101_cloud",
                "file": "pi0_so101_cloud.yaml",
                "title": "Pi0 (π0 Flow-Matching VLA)",
                "vram": ">= 24–40 GB (LoRA/冻结视觉) 或 A100 80GB",
                "platform": "云端 GPU (AutoDL / Featurize / HF Jobs)",
                "command": (
                    "lerobot-train --config_path=configs/train/pi0_so101_cloud.yaml "
                    "--dataset.repo_id=${HF_USER}/arm101_task"
                ),
            },
            {
                "id": "pi05_openpi_cloud",
                "file": "pi05_openpi_cloud.yaml",
                "title": "Pi0.5 (π0.5 OpenPI & LeRobot 双方案)",
                "vram": "推理 >8GB / LoRA >22.5GB / 全量 >70GB",
                "platform": "Ubuntu / 云端 Linux (Python 3.11 OpenPI 或 LeRobot)",
                "command": (
                    "uv run scripts/serve_policy.py policy:checkpoint "
                    "--policy.config=pi05_so101_custom --policy.dir=checkpoints/pi05_so101_custom/so101_run1/20000"
                ),
            },
        ]
        results: list[dict[str, Any]] = []
        for spec in specs:
            yaml_path = train_dir / spec["file"]
            raw_text = ""
            parsed: dict[str, Any] = {}
            if yaml_path.exists():
                raw_text = yaml_path.read_text(encoding="utf-8")
                try:
                    data = yaml.safe_load(raw_text)
                    if isinstance(data, dict):
                        parsed = data
                except Exception:
                    parsed = {}
            results.append(
                {
                    **spec,
                    "path": f"configs/train/{spec['file']}",
                    "exists": yaml_path.exists(),
                    "parsed": parsed,
                    "raw_yaml": raw_text,
                }
            )
        return results


def load_config() -> StudioConfig:
    """Load StudioConfig from environment variables and workspace .env."""
    root = resolve_workspace_root()
    env_file = root / ".env"
    if load_dotenv is not None and env_file.exists():
        load_dotenv(env_file, override=False)

    data_dir = _resolve_under_root(root, os.environ.get("HF_LEROBOT_HOME", ""), "data")
    is_win = platform.system() == "Windows"

    raw_mode = os.environ.get("STUDIO_MODE", "auto").strip().lower()
    requested_mode: ModeType = raw_mode if raw_mode in ("auto", "real", "dry_run") else "auto"  # type: ignore[assignment]

    try:
        port = int(os.environ.get("STUDIO_PORT", "8760"))
    except ValueError:
        port = 8760

    cfg = StudioConfig(
        workspace_root=root,
        data_dir=data_dir,
        datasets_dir=data_dir / "datasets",
        calibration_dir=data_dir / "calibration",
        dry_run_dir=data_dir / "dry_run",
        logs_dir=root / "logs",
        configs_dir=root / "configs",
        repos_dir=root / "repos",
        frontend_dir=root / "app" / "frontend",
        host=os.environ.get("STUDIO_HOST", "127.0.0.1").strip() or "127.0.0.1",
        port=port,
        requested_mode=requested_mode,
        follower_port=(
            os.environ.get("ARM101_FOLLOWER_PORT_WIN", "COM3").strip()
            if is_win
            else os.environ.get("ARM101_FOLLOWER_PORT_UBUNTU", "/dev/ttyACM0").strip()
        ),
        leader_port=(
            os.environ.get("ARM101_LEADER_PORT_WIN", "COM4").strip()
            if is_win
            else os.environ.get("ARM101_LEADER_PORT_UBUNTU", "/dev/ttyACM1").strip()
        ),
        follower_id=os.environ.get("ARM101_FOLLOWER_ID", "arm101_follower_01").strip()
        or "arm101_follower_01",
        leader_id=os.environ.get("ARM101_LEADER_ID", "arm101_leader_01").strip()
        or "arm101_leader_01",
        front_camera_id=os.environ.get("ARM101_FRONT_CAMERA_ID", "0").strip() or "0",
        wrist_camera_id=os.environ.get("ARM101_WRIST_CAMERA_ID", "1").strip() or "1",
    )
    cfg.ensure_directories()
    return cfg
