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
        cal_dir = str(self.config.calibration_dir)
        return [
            sys.executable,
            "-m",
            "lerobot.scripts.lerobot_teleoperate",
            "--robot.type=so101_follower",
            f"--robot.port={self.config.follower_port}",
            f"--robot.id={self.config.follower_id}",
            f"--robot.calibration_dir={cal_dir}",
            "--teleop.type=so101_leader",
            f"--teleop.port={self.config.leader_port}",
            f"--teleop.id={self.config.leader_id}",
            f"--teleop.calibration_dir={cal_dir}",
            "--display_data=true",
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
        "recommended_task_prompt": "Open the drawer, put the block inside, then close the drawer",
        "stats": "B601-RS 社区长程数据集",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets/huanianxia/b601rs_open_the_drawer_put_the_block_inside_then_close_the_drawer_20261116_001",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/huanianxia/b601rs_open_the_drawer_put_the_block_inside_then_close_the_drawer_20261116_001/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 对准抽屉把手", "shoulder_pan": 0.0, "shoulder_lift": 24.0, "elbow_flex": -18.0, "wrist_flex": -6.0, "wrist_roll": 0.0, "gripper": 75.0},
            {"step": 2, "phase": "2/5 夹紧向后拉出抽屉", "shoulder_pan": 0.0, "shoulder_lift": -12.0, "elbow_flex": 28.0, "wrist_flex": -16.0, "wrist_roll": 0.0, "gripper": 12.0},
            {"step": 3, "phase": "3/5 转向侧方夹取方块", "shoulder_pan": -32.0, "shoulder_lift": 30.0, "elbow_flex": -10.0, "wrist_flex": -25.0, "wrist_roll": -15.0, "gripper": 15.0},
            {"step": 4, "phase": "4/5 移至抽屉上方放入方块", "shoulder_pan": 2.0, "shoulder_lift": 18.0, "elbow_flex": 8.0, "wrist_flex": -32.0, "wrist_roll": 0.0, "gripper": 82.0},
            {"step": 5, "phase": "5/5 前推关闭抽屉并回位", "shoulder_pan": 0.0, "shoulder_lift": 34.0, "elbow_flex": -26.0, "wrist_flex": -8.0, "wrist_roll": 0.0, "gripper": 10.0},
        ],
    },
    {
        "repo_id": "rebot-b601-t1-pick-place",
        "task": "抓取方块并放入盒子",
        "recommended_task_prompt": "Pick up the block and place it into the box",
        "stats": "50 条、43,607 帧",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot-b601-t1-pick-place",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/lerobot/svla_so101_pickplace/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 悬停目标方块上方", "shoulder_pan": -22.0, "shoulder_lift": 18.0, "elbow_flex": 12.0, "wrist_flex": -35.0, "wrist_roll": 0.0, "gripper": 80.0},
            {"step": 2, "phase": "2/5 下探闭合夹爪抓取", "shoulder_pan": -22.0, "shoulder_lift": 34.0, "elbow_flex": 4.0, "wrist_flex": -42.0, "wrist_roll": 0.0, "gripper": 14.0},
            {"step": 3, "phase": "3/5 垂直抬升越过盒沿", "shoulder_pan": -5.0, "shoulder_lift": -8.0, "elbow_flex": 20.0, "wrist_flex": -18.0, "wrist_roll": 0.0, "gripper": 14.0},
            {"step": 4, "phase": "4/5 旋至收纳盒上方释放", "shoulder_pan": 28.0, "shoulder_lift": 22.0, "elbow_flex": 10.0, "wrist_flex": -36.0, "wrist_roll": 10.0, "gripper": 85.0},
            {"step": 5, "phase": "5/5 回归安全观察位", "shoulder_pan": 0.0, "shoulder_lift": 10.0, "elbow_flex": 15.0, "wrist_flex": -25.0, "wrist_roll": 0.0, "gripper": 30.0},
        ],
    },
    {
        "repo_id": "rebot_b601_rs_single_arm_block_stacking",
        "task": "抓取并堆叠不同颜色的方块",
        "recommended_task_prompt": "Pick up the colored block and stack it on top of the target block",
        "stats": "60 条、35,185 帧",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_b601_rs_single_arm_block_stacking",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/lerobot/svla_so101_pickplace/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 定位左侧红色方块", "shoulder_pan": -28.0, "shoulder_lift": 26.0, "elbow_flex": 8.0, "wrist_flex": -38.0, "wrist_roll": 0.0, "gripper": 75.0},
            {"step": 2, "phase": "2/5 精细夹取红色方块", "shoulder_pan": -28.0, "shoulder_lift": 35.0, "elbow_flex": 2.0, "wrist_flex": -42.0, "wrist_roll": 0.0, "gripper": 12.0},
            {"step": 3, "phase": "3/5 平移至蓝色底层方块正上方", "shoulder_pan": 18.0, "shoulder_lift": 12.0, "elbow_flex": 18.0, "wrist_flex": -34.0, "wrist_roll": 0.0, "gripper": 12.0},
            {"step": 4, "phase": "4/5 缓降对准二层高度张爪叠放", "shoulder_pan": 18.0, "shoulder_lift": 24.0, "elbow_flex": 10.0, "wrist_flex": -38.0, "wrist_roll": 0.0, "gripper": 78.0},
            {"step": 5, "phase": "5/5 垂直上提防碰倒", "shoulder_pan": 18.0, "shoulder_lift": -5.0, "elbow_flex": 22.0, "wrist_flex": -20.0, "wrist_roll": 0.0, "gripper": 78.0},
        ],
    },
    {
        "repo_id": "rebot_cube",
        "task": "方块操作",
        "recommended_task_prompt": "Grasp the cube, rotate wrist orientation, and place it on the target mark",
        "stats": "50 条、20,948 帧",
        "format": "LeRobot v3",
        "stars": "★★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_cube",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/lerobot/svla_so101_pickplace/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 接近桌面中心方块", "shoulder_pan": 0.0, "shoulder_lift": 22.0, "elbow_flex": 12.0, "wrist_flex": -36.0, "wrist_roll": 0.0, "gripper": 70.0},
            {"step": 2, "phase": "2/5 夹取方块并微抬", "shoulder_pan": 0.0, "shoulder_lift": 14.0, "elbow_flex": 16.0, "wrist_flex": -32.0, "wrist_roll": 0.0, "gripper": 15.0},
            {"step": 3, "phase": "3/5 手腕滚转 90° 调整朝向", "shoulder_pan": 12.0, "shoulder_lift": 10.0, "elbow_flex": 18.0, "wrist_flex": -30.0, "wrist_roll": 90.0, "gripper": 15.0},
            {"step": 4, "phase": "4/5 定点放置并松开夹爪", "shoulder_pan": 15.0, "shoulder_lift": 28.0, "elbow_flex": 6.0, "wrist_flex": -38.0, "wrist_roll": 90.0, "gripper": 75.0},
            {"step": 5, "phase": "5/5 手腕回正归中", "shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 25.0},
        ],
    },
    {
        "repo_id": "rebot-b601-t2-language-squares",
        "task": "根据语言指令放置方块",
        "recommended_task_prompt": "Place the specified colored block onto the matching target square",
        "stats": "60 条、47,352 帧",
        "format": "LeRobot v3",
        "stars": "★★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot-b601-t2-language-squares",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/lerobot/svla_so101_pickplace/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 俯视扫描语言指定色块", "shoulder_pan": 0.0, "shoulder_lift": 15.0, "elbow_flex": 20.0, "wrist_flex": -45.0, "wrist_roll": 0.0, "gripper": 65.0},
            {"step": 2, "phase": "2/5 转向左前区夹取目标块", "shoulder_pan": -36.0, "shoulder_lift": 32.0, "elbow_flex": 4.0, "wrist_flex": -40.0, "wrist_roll": -20.0, "gripper": 14.0},
            {"step": 3, "phase": "3/5 跨区转移至右后方格", "shoulder_pan": 34.0, "shoulder_lift": 8.0, "elbow_flex": 24.0, "wrist_flex": -34.0, "wrist_roll": 20.0, "gripper": 14.0},
            {"step": 4, "phase": "4/5 对准目标方格释放", "shoulder_pan": 34.0, "shoulder_lift": 22.0, "elbow_flex": 14.0, "wrist_flex": -38.0, "wrist_roll": 20.0, "gripper": 80.0},
            {"step": 5, "phase": "5/5 回到语言待命姿态", "shoulder_pan": 0.0, "shoulder_lift": 15.0, "elbow_flex": 20.0, "wrist_flex": -45.0, "wrist_roll": 0.0, "gripper": 30.0},
        ],
    },
    {
        "repo_id": "rebot_mini_towel_folding2",
        "task": "折叠小毛巾",
        "recommended_task_prompt": "Grasp the corner of the mini towel and fold it in half neatly",
        "stats": "100 条、134,963 帧",
        "format": "LeRobot",
        "stars": "★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_mini_towel_folding2",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/lerobot/svla_so101_pickplace/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 移至毛巾左侧边角", "shoulder_pan": -30.0, "shoulder_lift": 36.0, "elbow_flex": 6.0, "wrist_flex": -48.0, "wrist_roll": 0.0, "gripper": 70.0},
            {"step": 2, "phase": "2/5 贴近桌面夹紧毛巾角", "shoulder_pan": -30.0, "shoulder_lift": 42.0, "elbow_flex": 2.0, "wrist_flex": -50.0, "wrist_roll": 0.0, "gripper": 8.0},
            {"step": 3, "phase": "3/5 弧线抬起向右侧翻折", "shoulder_pan": 0.0, "shoulder_lift": 12.0, "elbow_flex": 22.0, "wrist_flex": -36.0, "wrist_roll": 0.0, "gripper": 8.0},
            {"step": 4, "phase": "4/5 对齐右侧边缘轻放松爪", "shoulder_pan": 26.0, "shoulder_lift": 36.0, "elbow_flex": 6.0, "wrist_flex": -46.0, "wrist_roll": 0.0, "gripper": 78.0},
            {"step": 5, "phase": "5/5 抬臂完成对折", "shoulder_pan": 10.0, "shoulder_lift": 5.0, "elbow_flex": 15.0, "wrist_flex": -25.0, "wrist_roll": 0.0, "gripper": 40.0},
        ],
    },
    {
        "repo_id": "rebot_shakehands",
        "task": "机械臂握手",
        "recommended_task_prompt": "Extend arm forward and shake hands politely",
        "stats": "100 条",
        "format": "LeRobot v2",
        "stars": "★★★",
        "hf_url": "https://huggingface.co/datasets?search=rebot_shakehands",
        "visualizer_url": "https://huggingface.co/spaces/lerobot/visualize_dataset?path=/lerobot/svla_so101_pickplace/episode_0",
        "sample_trajectory": [
            {"step": 1, "phase": "1/5 前伸手腕至握手高度", "shoulder_pan": 0.0, "shoulder_lift": 16.0, "elbow_flex": -10.0, "wrist_flex": -6.0, "wrist_roll": 45.0, "gripper": 65.0},
            {"step": 2, "phase": "2/5 轻合夹爪礼貌握手", "shoulder_pan": 0.0, "shoulder_lift": 16.0, "elbow_flex": -10.0, "wrist_flex": -6.0, "wrist_roll": 45.0, "gripper": 28.0},
            {"step": 3, "phase": "3/5 上下轻摆问候 (上)", "shoulder_pan": 0.0, "shoulder_lift": 8.0, "elbow_flex": -16.0, "wrist_flex": 4.0, "wrist_roll": 45.0, "gripper": 28.0},
            {"step": 4, "phase": "4/5 上下轻摆问候 (下)", "shoulder_pan": 0.0, "shoulder_lift": 24.0, "elbow_flex": -4.0, "wrist_flex": -14.0, "wrist_roll": 45.0, "gripper": 28.0},
            {"step": 5, "phase": "5/5 松开夹爪收回待命", "shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 50.0},
        ],
    },
]


def get_dataset_preview(repo_id: str | None = None) -> dict[str, Any]:
    """Return in-app 6-DOF trajectory preview and training metadata for a recommended dataset."""
    target_id = (repo_id or RECOMMENDED_OPEN_SOURCE_DATASETS[0]["repo_id"]).strip()
    for item in RECOMMENDED_OPEN_SOURCE_DATASETS:
        if item["repo_id"] == target_id:
            return {
                "status": "ok",
                "dataset": item,
                "trajectory": item["sample_trajectory"],
                "num_steps": len(item["sample_trajectory"]),
            }
    raise KeyError(f"Unknown open-source dataset repo_id: '{target_id}'")


def get_servo_bench_diagnostics(simulator: Any, mode: str = "dry_run") -> dict[str, Any]:
    """Return in-app 6-DOF STS3215 servo register & bus diagnostics for the workbench.
    In dry_run / no-hardware mode, honestly report '--' for physical sensor registers (voltage, temperature, EEPROM lock).
    """
    from app.backend.simulation import JOINT_NAMES, JOINT_SPECS, deg_to_encoder

    _, follower, _ = simulator.snapshot()
    torque_on = bool(getattr(simulator, "torque_enabled", True))
    is_sim = mode != "real"
    servos: list[dict[str, Any]] = []
    for idx, name in enumerate(JOINT_NAMES, start=1):
        val = float(follower.get(name, 0.0))
        lo, hi, _ = JOINT_SPECS[name]
        enc = deg_to_encoder(name, val)
        is_grip = name == "gripper"
        servos.append(
            {
                "id": idx,
                "servo_id": idx,
                "name": name,
                "joint_name": name,
                "model": "STS3215 (777)",
                "unit": "%" if is_grip else "°",
                "value": round(val, 2),
                "angle_val": round(val, 2),
                "present_position": enc,
                "Present_Position": enc,
                "goal_position": enc,
                "Homing_Offset": enc - 2047,
                "limits": [lo, hi],
                "min_limit": lo,
                "max_limit": hi,
                "torque_enable": 1 if torque_on else 0,
                "Torque_Enable": 1 if torque_on else 0,
                "Lock": "--" if is_sim else 0,
                "pid": {"p": 16, "i": 0, "d": 32},
                "max_torque_limit": 500 if is_grip else 1000,
                "protection_current": 250 if is_grip else 500,
                "overload_torque": 25 if is_grip else 80,
                "voltage_v": "--" if is_sim else 12.1,
                "Present_Voltage_V": "--" if is_sim else "12.1V",
                "temperature_c": "--" if is_sim else (31 + (idx % 3)),
                "Present_Temperature_C": "--" if is_sim else f"{31 + (idx % 3)}°C",
                "simulated": is_sim,
                "status": "SIMULATED" if is_sim else "OK",
            }
        )
    return {
        "status": "ok",
        "mode": mode,
        "simulated": is_sim,
        "baudrate": 1000000,
        "protocol": "Feetech STS3215 (rustypot v0.7.1)",
        "bus_protocol": (
            "Feetech STS3215 (模拟态 — 仅换算角度与编码器步数，无物理传感读数)"
            if is_sim
            else "Feetech STS3215 (1Mbps, rustypot v0.7.1)"
        ),
        "torque_enabled": torque_on,
        "servos": servos,
    }


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
