"""Extensible ToolAdapter plugin hub for integrating open-source LeRobot tools."""

from __future__ import annotations

import importlib.util
import platform
import shutil
import sys
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

from app.backend.config import StudioConfig
from app.backend.hardware import HardwareManager
from app.backend.process_manager import ProcessRegistry


class PluginUnavailableError(Exception):
    """Raised when a plugin cannot be launched on the current OS or environment."""

    def __init__(
        self,
        detail: str,
        *,
        code: str = "PLUGIN_UNAVAILABLE",
        install_hint: str | None = None,
    ) -> None:
        super().__init__(detail)
        self.detail = detail
        self.code = code
        self.install_hint = install_hint


class ToolAdapter(ABC):
    """Base contract for external or integrated open-source tool plugins."""

    id: str
    name: str
    category: str
    description: str
    os_support: tuple[str, ...]
    default_url: str | None

    def __init__(
        self,
        config: StudioConfig,
        hardware: HardwareManager,
        process_registry: ProcessRegistry,
    ) -> None:
        self.config = config
        self.hardware = hardware
        self.process_registry = process_registry

    @property
    def proc_id(self) -> str:
        return f"plugin_{self.id}"

    def is_os_supported(self) -> bool:
        return platform.system() in self.os_support

    def is_running(self) -> bool:
        return self.process_registry.is_running(self.proc_id)

    def pid(self) -> int | None:
        managed = self.process_registry.get(self.proc_id)
        if managed is not None and managed.proc.poll() is None:
            return managed.proc.pid
        return None

    @abstractmethod
    def probe(self) -> dict[str, Any]:
        """Return availability, OS support, status, and install instructions."""

    @abstractmethod
    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        """Build the OS subprocess command for launching this tool."""

    def launch_cwd(self) -> Path:
        """Return working directory for subprocess launch (defaults to workspace_root)."""
        return self.config.workspace_root

    def launch(self, args: dict[str, Any] | None = None) -> dict[str, Any]:
        args = args or {}
        info = self.probe()
        if not info["os_supported"]:
            raise PluginUnavailableError(
                f"Plugin '{self.id}' is not supported locally on {platform.system()}.",
                code="OS_UNSUPPORTED",
                install_hint=info.get("install_hint"),
            )
        if not info["available"]:
            raise PluginUnavailableError(
                f"Plugin '{self.id}' dependencies are not installed on this host.",
                code="PLUGIN_UNAVAILABLE",
                install_hint=info.get("install_hint"),
            )
        if self.is_running():
            return {
                "status": "running",
                "plugin_id": self.id,
                "pid": self.pid(),
                "url": info.get("url"),
            }
        cmd = self.build_launch_cmd(args)
        managed = self.process_registry.spawn(
            self.proc_id, self.id, cmd, cwd=self.launch_cwd()
        )
        return {
            "status": "running",
            "plugin_id": self.id,
            "pid": managed.proc.pid,
            "url": info.get("url"),
            "command": cmd,
        }

    def stop(self) -> dict[str, Any]:
        self.process_registry.terminate(self.proc_id)
        return {"status": "stopped", "plugin_id": self.id}


class LeRobotDatasetVizAdapter(ToolAdapter):
    id = "lerobot_dataset_viz"
    name = "LeRobot Dataset Visualizer (Rerun / Foxglove)"
    category = "visualization"
    description = "官方数据集回放可视化工具（Rerun / Foxglove），支持本地与远程数据帧检视。"
    os_support = ("Windows", "Linux", "Darwin")
    default_url = "http://127.0.0.1:9090"

    def probe(self) -> dict[str, Any]:
        has_lerobot = importlib.util.find_spec("lerobot") is not None
        has_datasets = importlib.util.find_spec("datasets") is not None
        has_rerun = importlib.util.find_spec("rerun") is not None
        has_foxglove = importlib.util.find_spec("foxglove") is not None
        available = has_lerobot and has_datasets and (has_rerun or has_foxglove)
        status = "running" if self.is_running() else ("stopped" if available else "unavailable")
        hint = None
        if not available:
            hint = (
                "需要安装数据集与可视化扩展：Windows 请运行 "
                "`scripts\\windows\\setup_win.ps1 -WithCoreScripts`，Ubuntu 请运行 "
                "`uv pip install 'lerobot[dataset]' rerun-sdk`。"
            )
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "status": status,
            "available": available,
            "os_supported": True,
            "os_support": list(self.os_support),
            "url": self.default_url,
            "pid": self.pid(),
            "description": self.description,
            "install_hint": hint,
        }

    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        repo_id = str(args.get("repo_id") or "lerobot/pusht")
        episode_index = int(args.get("episode_index") or 0)
        web_port = int(args.get("web_port") or 9090)
        display_mode = str(args.get("display_mode") or "rerun")
        return [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_dataset_viz",
            "--repo-id",
            repo_id,
            "--episode-index",
            str(episode_index),
            "--mode",
            "distant",
            "--display-mode",
            display_mode,
            "--web-port",
            str(web_port),
        ]


class FeetechServoBenchAdapter(ToolAdapter):
    id = "feetech_servo_bench"
    name = "Feetech MotorLab (STS3215 舵机调试台)"
    category = "hardware_diagnostics"
    description = "Nori-Robotics/feetech-motorlab：飞特 STS3215 舵机 ID 烧录、中位校准与总线扫描工具。"
    os_support = ("Windows", "Linux", "Darwin")
    default_url = "http://127.0.0.1:8765"

    def _find_python_with_scservo(self) -> Path | None:
        if importlib.util.find_spec("scservo_sdk") is not None:
            return Path(sys.executable)
        repo_venv_py = (
            self.config.repos_dir / "feetech-motorlab" / ".venv" / "Scripts" / "python.exe"
            if platform.system() == "Windows"
            else self.config.repos_dir / "feetech-motorlab" / ".venv" / "bin" / "python"
        )
        if repo_venv_py.exists():
            return repo_venv_py
        return None

    def probe(self) -> dict[str, Any]:
        run_py = self.config.repos_dir / "feetech-motorlab" / "run.py"
        py_bin = self._find_python_with_scservo()
        available = run_py.exists() and py_bin is not None
        status = "running" if self.is_running() else ("stopped" if available else "unavailable")
        hint = None
        if not run_py.exists():
            hint = "未找到 repos/feetech-motorlab/run.py，请先执行 `python scripts/common/bootstrap_repos.py`。"
        elif py_bin is None:
            hint = (
                "缺少 `feetech-servo-sdk`（scservo_sdk）。建议在 `repos/feetech-motorlab/.venv` "
                "或当前环境执行 `uv pip install -r repos/feetech-motorlab/requirements.txt`。"
            )
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "status": status,
            "available": available,
            "os_supported": True,
            "os_support": list(self.os_support),
            "url": self.default_url,
            "pid": self.pid(),
            "description": self.description,
            "install_hint": hint,
        }

    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        py_bin = self._find_python_with_scservo() or Path(sys.executable)
        run_py = self.config.repos_dir / "feetech-motorlab" / "run.py"
        port = int(args.get("port") or 8765)
        cmd = [
            str(py_bin),
            str(run_py),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--no-browser",
        ]
        if self.hardware.active_mode == "dry_run" or args.get("simulate"):
            cmd.append("--simulate")
        return cmd


class LeRobotStudioAdapter(ToolAdapter):
    id = "lerobot_studio"
    name = "LeRobot Live Studio (Rerun / Foxglove 遥测流)"
    category = "telemetry_studio"
    description = "通过 lerobot-teleoperate 内置 Rerun / Foxglove 流式界面实时查看关节与图像帧。"
    os_support = ("Windows", "Linux", "Darwin")
    default_url = "http://127.0.0.1:8767"

    def probe(self) -> dict[str, Any]:
        has_lerobot = importlib.util.find_spec("lerobot") is not None
        has_viz = (
            importlib.util.find_spec("rerun") is not None
            or importlib.util.find_spec("foxglove") is not None
        )
        hw_ok = bool(self.hardware.last_probe.get("hardware_ready", False))
        available = has_lerobot and has_viz and hw_ok
        status = "running" if self.is_running() else ("stopped" if available else "unavailable")
        hint = None
        if not has_viz:
            hint = "请安装 rerun-sdk 或 foxglove-sdk（`setup_win.ps1 -WithCoreScripts`）。"
        elif not hw_ok:
            hint = "需连接双臂 SO-101 硬件方可启动实机 Rerun/Foxglove 遥测流（当前可使用 Studio 内置模拟）。"
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "status": status,
            "available": available,
            "os_supported": True,
            "os_support": list(self.os_support),
            "url": self.default_url,
            "pid": self.pid(),
            "description": self.description,
            "install_hint": hint,
        }

    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        display_mode = str(args.get("display_mode") or "rerun")
        return [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_teleoperate",
            "--robot.type=so101_follower",
            f"--robot.port={self.config.follower_port}",
            f"--robot.id={self.config.follower_id}",
            "--teleop.type=so101_leader",
            f"--teleop.port={self.config.leader_port}",
            f"--teleop.id={self.config.leader_id}",
            "--display_data=true",
            f"--display_mode={display_mode}",
        ]


class LeLabExternalAdapter(ToolAdapter):
    id = "lelab_external"
    name = "Hugging Face LeLab Web 控制台"
    category = "external_console"
    description = "huggingface/leLab 官方 Web 控制台，支持独立进程启停（需通过 -WithLeLab 隔离安装）。"
    os_support = ("Windows", "Linux", "Darwin")
    default_url = "http://127.0.0.1:8000"

    def probe(self) -> dict[str, Any]:
        has_pkg = importlib.util.find_spec("lelab") is not None
        has_cli = shutil.which("lelab") is not None
        has_dist = (self.config.repos_dir / "leLab" / "frontend" / "dist" / "index.html").exists()
        available = (has_pkg or has_cli) and has_dist
        status = "running" if self.is_running() else ("stopped" if available else "unavailable")
        hint = None
        if not available:
            hint = (
                "未检测到已构建的 LeLab（含 frontend/dist）。Windows 请先在 repos/leLab/frontend "
                "构建静态资源后运行 `scripts\\windows\\setup_win.ps1 -WithLeLab`。"
            )
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "status": status,
            "available": available,
            "os_supported": True,
            "os_support": list(self.os_support),
            "url": self.default_url,
            "pid": self.pid(),
            "description": self.description,
            "install_hint": hint,
        }

    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        return [sys.executable, "-m", "lelab.scripts.lelab", "--no-open"]


class RemoteVLAPolicyAdapter(ToolAdapter):
    id = "remote_vla_policy"
    name = "Remote VLA Policy Server (OpenPI / LeRobot)"
    category = "vla_inference"
    description = (
        "连接 Ubuntu / 云端 GPU 上的 Physical-Intelligence/openpi (ws://:8000) "
        "或启动本地 lerobot-policy-server。"
    )
    os_support = ("Linux",)
    default_url = "ws://127.0.0.1:8000"

    def probe(self) -> dict[str, Any]:
        is_linux = self.is_os_supported()
        openpi_dir = self.config.repos_dir / "openpi"
        has_zenoh = importlib.util.find_spec("zenoh") is not None
        available = is_linux and (openpi_dir.exists() or has_zenoh)
        status = "running" if self.is_running() else ("stopped" if available else "unavailable")
        hint = (
            "OpenPI (`repos/openpi`, JAX/CUDA12, Python 3.11) 仅支持 Ubuntu / 云端 Linux。"
            "在 Windows 端请在「VLA 与云端配置」标签页填写远程 WebSocket 服务地址进行离线/远程联调。"
            if not is_linux
            else "请确保已运行 `bash scripts/ubuntu/setup_ubuntu.sh --with-openpi` 配置 ~/.venvs/tust-openpi。"
        )
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "status": status,
            "available": available,
            "os_supported": is_linux,
            "os_support": list(self.os_support),
            "url": self.default_url,
            "pid": self.pid(),
            "description": self.description,
            "install_hint": hint,
        }

    def launch_cwd(self) -> Path:
        return self.config.repos_dir / "openpi"

    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        port = int(args.get("port") or 8000)
        raw_cfg = str(args.get("policy_config") or "pi05_droid")
        if raw_cfg == "pi05_so101_custom":
            raw_cfg = "pi05_droid"
        policy_config = raw_cfg
        policy_dir = str(args.get("policy_dir") or "models/checkpoints/pi05_so101")
        return [
            "uv",
            "run",
            str(self.config.repos_dir / "openpi" / "scripts" / "serve_policy.py"),
            "--port",
            str(port),
            "policy:checkpoint",
            f"--policy.config={policy_config}",
            f"--policy.dir={policy_dir}",
        ]


RECOMMENDED_OPEN_SOURCE_DATASETS: list[dict[str, Any]] = [
    {
        "repo_id": "huanianxia/b601rs_open_the_drawer_put_the_block_inside_then_close_the_drawer_20261116_001",
        "task": "打开抽屉、放入方块并关闭抽屉（B601-RS 复合长程任务）",
        "stats": "B601-RS 社区数据集",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets/huanianxia/b601rs_open_the_drawer_put_the_block_inside_then_close_the_drawer_20261116_001",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/huanianxia/b601rs_open_the_drawer_put_the_block_inside_then_close_the_drawer_20261116_001/episode_0",
    },
    {
        "repo_id": "rebot-b601-t1-pick-place",
        "task": "抓取方块并放入盒子",
        "stats": "50 条、43,607 帧",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot-b601-t1-pick-place",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset",
    },
    {
        "repo_id": "rebot_b601_rs_single_arm_block_stacking",
        "task": "抓取并堆叠不同颜色的方块",
        "stats": "60 条、35,185 帧",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_b601_rs_single_arm_block_stacking",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset",
    },
    {
        "repo_id": "rebot_cube",
        "task": "方块操作",
        "stats": "50 条、20,948 帧",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_cube",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset",
    },
    {
        "repo_id": "rebot-b601-t2-language-squares",
        "task": "根据语言指令放置方块",
        "stats": "60 条、47,352 帧",
        "format": "LeRobot v3",
        "stars": "★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot-b601-t2-language-squares",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset",
    },
    {
        "repo_id": "rebot_mini_towel_folding2",
        "task": "折叠小毛巾",
        "stats": "100 条、134,963 帧",
        "format": "LeRobot",
        "stars": "★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_mini_towel_folding2",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset",
    },
    {
        "repo_id": "rebot_shakehands",
        "task": "机械臂握手",
        "stats": "100 条",
        "format": "LeRobot v2",
        "stars": "★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_shakehands",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset",
    },
]


class OpenSourceDatasetHubAdapter(ToolAdapter):
    id = "open_source_datasets"
    name = "B601-RS / LeRobot 开源数据集推荐库"
    category = "datasets"
    description = (
        "内置 7 个精选 LeRobot / B601-RS 开源数据集（含抽屉抓取、堆叠、语言指令、折毛巾、握手），"
        "支持一键直达 Hugging Face / 官方在线 Visualizer 或启动本地 lerobot-dataset-viz 检视。"
    )
    os_support = ("Windows", "Linux", "Darwin")
    default_url = "https://huggingface.co/spaces/lerobot/visualize_dataset"

    def probe(self) -> dict[str, Any]:
        has_lerobot = importlib.util.find_spec("lerobot") is not None
        has_datasets = importlib.util.find_spec("datasets") is not None
        has_viz = (importlib.util.find_spec("rerun") is not None) or (
            importlib.util.find_spec("foxglove") is not None
        )
        local_viz_ready = has_lerobot and has_datasets and has_viz
        status = "running" if self.is_running() else "stopped"
        hint = (
            None
            if local_viz_ready
            else "在线可视化可直接点击条目链接；若需本地 Rerun 检视请运行 `setup_win.ps1 -WithCoreScripts`。"
        )
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "status": status,
            "available": True,
            "local_viz_ready": local_viz_ready,
            "os_supported": True,
            "os_support": list(self.os_support),
            "url": self.default_url,
            "pid": self.pid(),
            "description": self.description,
            "install_hint": hint,
            "datasets": RECOMMENDED_OPEN_SOURCE_DATASETS,
        }

    def build_launch_cmd(self, args: dict[str, Any]) -> list[str]:
        repo_id = str(
            args.get("repo_id")
            or RECOMMENDED_OPEN_SOURCE_DATASETS[0]["repo_id"]
        )
        episode = int(args.get("episode_index") or 0)
        return [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_dataset_viz",
            f"--repo-id={repo_id}",
            f"--episode-index={episode}",
        ]


class PluginRegistry:
    """Registry of extensible ToolAdapter instances."""

    def __init__(
        self,
        config: StudioConfig,
        hardware: HardwareManager,
        process_registry: ProcessRegistry,
    ) -> None:
        self._adapters: dict[str, ToolAdapter] = {}
        for cls in (
            LeRobotDatasetVizAdapter,
            FeetechServoBenchAdapter,
            LeRobotStudioAdapter,
            LeLabExternalAdapter,
            RemoteVLAPolicyAdapter,
            OpenSourceDatasetHubAdapter,
        ):
            adapter = cls(config, hardware, process_registry)
            self._adapters[adapter.id] = adapter

    def register(self, adapter: ToolAdapter) -> None:
        self._adapters[adapter.id] = adapter

    def list_plugins(self) -> list[dict[str, Any]]:
        return [adapter.probe() for adapter in self._adapters.values()]

    def get(self, plugin_id: str) -> ToolAdapter:
        if plugin_id not in self._adapters:
            raise KeyError(f"Unknown plugin '{plugin_id}'.")
        return self._adapters[plugin_id]

    def launch(self, plugin_id: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.get(plugin_id).launch(args)

    def stop(self, plugin_id: str) -> dict[str, Any]:
        return self.get(plugin_id).stop()

    def stop_all(self) -> None:
        for adapter in self._adapters.values():
            try:
                adapter.stop()
            except Exception:
                pass
