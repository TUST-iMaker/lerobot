# LeRobot 规范工作区中文教程体系总览（对齐 `repos/lerobot` v0.7.1）

欢迎来到 **TUST-iMaker LeRobot 规范工作区**文档中心。本教程体系围绕官方 **arm101（SO-ARM101）主从臂**与 **Windows + Ubuntu 双系统协同架构**编写，所有 CLI 命令、YAML 配置模板与 Python 环境路径均与本地克隆的 [`repos/lerobot`](../repos/lerobot/)（`v0.7.1`）、[`repos/openpi`](../repos/openpi/) 及 [`scripts/`](../scripts/)、[`configs/`](../configs/) 严格对齐。

---

## 1. 全套文档导航

| 模块 | 文档路径 | 核心内容 | 对应里程碑 |
|---|---|---|---|
| **参考来源索引** | [`sources/README.md`](sources/README.md) | GitHub、Hugging Face Docs、Seeed Wiki、arXiv 与同济子豪兄飞书 Wiki（含 302 跳转说明及版本漂移勘误） | F7 |
| **双系统环境搭建** | [`01_environment/dual_os_setup.md`](01_environment/dual_os_setup.md) | Windows + Ubuntu/WSL2 共享工作区、`uv` 虚拟环境隔离、`bootstrap_repos.py`、`check_env.py` 与 `udev` | F8 |
| **1. 硬件准备与标定** | [`02_arm101_workflow/01_calibration_and_ports.md`](02_arm101_workflow/01_calibration_and_ports.md) | arm101 装配前置说明、串口探测（`find_ports.ps1` / `lerobot-find-port`）、舵机 ID 设置与标定 JSON | F9 |
| **2. 遥操作与相机** | [`02_arm101_workflow/02_teleoperation.md`](02_arm101_workflow/02_teleoperation.md) | `lerobot-find-cameras`、YAML 与 CLI 遥操作、Rerun/Foxglove 实时可视化、Win/Ubuntu 相机后端差异 | F10 |
| **3. 演示教学与示教** | [`02_arm101_workflow/03_teaching_and_demo.md`](02_arm101_workflow/03_teaching_and_demo.md) | 示教轨迹设计、开环回放 `lerobot-replay`、策略推理与 HIL `lerobot-rollout`、本地软件示教模式 | F11 |
| **4. 数据采集与编辑** | [`02_arm101_workflow/04_data_collection.md`](02_arm101_workflow/04_data_collection.md) | `lerobot-record`（LeRobotDataset v3.0）、键盘快捷键、断点续采、`lerobot-dataset-viz` 与 `lerobot-edit-dataset` | F12 |
| **5. VLA 数据与云训练** | [`03_vla_and_pi/01_vla_data_and_cloud_training.md`](03_vla_and_pi/01_vla_data_and_cloud_training.md) | 数据上传/传输、RTX 5060 (8GB) 与云端 GPU 显存对比、HF Jobs / Featurize / AutoDL、`torchrun` 与 `trackio` | F13 |
| **6. `pi0.7` 与 Pi 系列 VLA** | [`03_vla_and_pi/02_pi_series_and_pi07.md`](03_vla_and_pi/02_pi_series_and_pi07.md) | `pi0.7`（`arXiv:2604.15483v2`）闭源现状核查、`lerobot-annotate`，以及 `pi0` / `pi0_fast` / `pi05` / `openpi` / `smolvla` 实操 | F14 |
| **开源前端调研与集成** | [`04_frontend_and_tools/open_source_frontend_survey.md`](04_frontend_and_tools/open_source_frontend_survey.md) | 10+ 款开源前端/舵机工具横评、许可证与系统限制、依赖隔离原则及 `ToolAdapter` 接入架构 | F15 |
| **本地串联软件指南** | `./04_frontend_and_tools/local_app_guide.md` *(M3 配套文档)* | **Arm101 Studio**（`../app/run_studio.py` / `python -m app.backend.main`）使用与 `dry_run` 无硬件演示指南 | M3 |

> **说明**：`04_frontend_and_tools/local_app_guide.md` 及 [`../app/`](../app/) 源码由后续本地软件模块（M3）维护与交付；本文档体系已提前约定一致的启动入口（`python ../app/run_studio.py` 或 `python -m app.backend.main`）与职责边界。

---

## 2. 按角色推荐阅读路线（Quick Learning Paths）

### 路线 A：Windows 真机操作员（硬件标定 / 遥操作 / 示教采数）
1. [`01_environment/dual_os_setup.md`](01_environment/dual_os_setup.md) — 运行 `setup_win.ps1 -WithCoreScripts`，配置 `.env` 与 `envs/win/lerobot`
2. [`02_arm101_workflow/01_calibration_and_ports.md`](02_arm101_workflow/01_calibration_and_ports.md) — `find_ports.ps1`、设置 STS3215 ID 与完成主从臂零位标定
3. [`02_arm101_workflow/02_teleoperation.md`](02_arm101_workflow/02_teleoperation.md) — 验证双相机（`front` / `wrist`）与 Rerun 实时遥操作
4. [`02_arm101_workflow/03_teaching_and_demo.md`](02_arm101_workflow/03_teaching_and_demo.md) — 使用 `lerobot-replay` 与本地 Studio 示教点验证可重复性
5. [`02_arm101_workflow/04_data_collection.md`](02_arm101_workflow/04_data_collection.md) — 使用 `lerobot-record` 录制 v3.0 数据集并用 `lerobot-dataset-viz` / `lerobot-edit-dataset` 质检

### 路线 B：Ubuntu / 云端算力训练工程师（ACT / SmolVLA / Pi 系列 / OpenPI）
1. [`01_environment/dual_os_setup.md`](01_environment/dual_os_setup.md) — 运行 `setup_ubuntu.sh --with-core-scripts --with-training --with-openpi --setup-udev`
2. [`03_vla_and_pi/01_vla_data_and_cloud_training.md`](03_vla_and_pi/01_vla_data_and_cloud_training.md) — 评估本地 8 GB（RTX 5060）与 24–80 GB 云端 GPU（HF Jobs / Featurize / AutoDL）策略，配置 `trackio` 与 `torchrun`
3. [`03_vla_and_pi/02_pi_series_and_pi07.md`](03_vla_and_pi/02_pi_series_and_pi07.md) — 了解 `pi0.7` 论文进展与未开源现状，跑通 `pi0`、`pi0_fast`、`pi05`（`lerobot` + `openpi`）与 `smolvla` 微调及部署
4. [`02_arm101_workflow/03_teaching_and_demo.md`](02_arm101_workflow/03_teaching_and_demo.md) — 使用 `lerobot-rollout`（含 `rtc` / `dagger`）或 `openpi serve_policy.py` 进行真机推理验证

### 路线 C：工具与前端集成开发者
1. [`sources/README.md`](sources/README.md) — 核查各上游仓库版本、提交哈希与许可证
2. [`04_frontend_and_tools/open_source_frontend_survey.md`](04_frontend_and_tools/open_source_frontend_survey.md) — 掌握冲突隔离边界（为何不直接混装 `leLab@v0.7.0` / `solo-bot` / `phosphobot`）与 `ToolAdapter` 接入规范
3. `./04_frontend_and_tools/local_app_guide.md` *(M3 交付)* 与 [`../app/`](../app/) — 启动与扩展 **Arm101 Studio**（`python -m app.backend.main` / `python app/run_studio.py`）

---

## 3. 版本对齐声明

- **核心运行时**：[`repos/lerobot`](../repos/lerobot/) 当前对齐 **`v0.7.1`**（Python `>=3.12`，22 个 `lerobot-*` CLI 入口）。
- **常见旧教程差异提醒**：早期的 `v0.4.x` / `v0.5.x` 教程（含部分社区或飞书早期笔记）存在命令差异，例如 `.[pi0]` 已更名为 `.[pi]`、`wandb` 参数已迁移为 `trackio`、`lerobot-record --policy.path` 已拆分为 `lerobot-rollout`、`huggingface-cli` 已更新为 `hf auth login`。本文档体系中的命令均已按 `v0.7.1` 源码逐一核验。
