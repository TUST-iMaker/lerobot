# TUST-iMaker LeRobot 规范工作区与 Arm101 Studio

面向多人协作与长期维护的 **LeRobot 标准化工作区（Standardized Workspace）**，锚定于 `TUST-iMaker/lerobot`，围绕官方 **arm101（SO-ARM101）主从臂**构建：

- **非重复上游仓库管理**：通过 `configs/repos.yaml` 与 `scripts/common/bootstrap_repos.py` 统一将所需开源仓库克隆到 `repos/`，杜绝多份重复克隆与散落依赖；
- **Windows + Ubuntu 双系统协同**：共享工作区代码、配置、教程与数据骨架，各自维护系统隔离的 `uv` 虚拟环境（Windows 端负责硬件与本地 UI，Ubuntu 端负责本地 CUDA 训练与 `openpi`）；
- **全流程中文教程体系（`docs/`）**：覆盖双系统配置、arm101 标定/遥操作/示教/数据采集、VLA 云端算力训练与 `pi0.7` / `pi0.5` / `pi0` / `openpi` 方案及开源前端工具调研；
- **本地串联软件（`app/` — Arm101 Studio）与统一控制内核（`software/`）**：融合 `software/core/` 的 SO-101 运动学/安全控制器/预设姿态与 `app/` 的全场景 6 轴火柴棍仿真、独立主从臂双向仿真 + 末端夹爪拖动 IK、交互式标定、遥操作、示教、多人数据规范采集与开源插件入口；无硬件时自动降级为 `dry_run` 模拟模式，并支持 `python software/main.py web` 一键拉起统一界面（加 `--legacy` 可回退旧版）。

---

## 1. 目录结构与模块职责

```text
lerobot/                                   # 根 Git 仓库 (TUST-iMaker/lerobot)
├── README.md                              # 本文件：总览、双系统协作、扩展规范与入口
├── .gitignore                             # 忽略 repos/、envs/、data/*、models/*、logs/* 等本地产物
├── .gitattributes                         # 跨平台换行策略：*.sh 强制 LF，*.ps1 强制 CRLF
├── .env.example                           # 跨系统环境变量模板（端口、缓存、虚拟环境路径）
├── pyproject.toml                         # 根项目元数据与 app/ + 测试依赖（Python >= 3.12）
├── docs/                                  # [文档扩展区] 中文全流程教程与前端工具调研
│   ├── README.md                          # 教程总目录与阅读路线
│   ├── sources/README.md                  # GitHub、Hugging Face、飞书 Wiki 等参考来源索引
│   ├── 01_environment/                    # 双系统与 WSL2 环境配置教程
│   ├── 02_arm101_workflow/                # arm101 标定、遥操作、示教、数据采集教程
│   ├── 03_vla_and_pi/                     # VLA 数据准备、云端租卡训练与 pi0.7/pi0.5/openpi 指南
│   └── 04_frontend_and_tools/             # 开源前端调研对比与本地 Arm101 Studio 使用指南
├── scripts/                               # [脚本区] 跨系统初始化、端口探测与自检工具
│   ├── common/
│   │   ├── bootstrap_repos.py             # 按 configs/repos.yaml 幂等克隆并校验无重复仓库
│   │   └── check_env.py                   # 零第三方依赖的跨平台环境与配置体检脚本
│   ├── windows/
│   │   ├── setup_win.ps1                  # Windows Python 3.12 uv 环境初始化脚本
│   │   └── find_ports.ps1                 # Windows COM 口（CH340/STM32）与摄像头枚举工具
│   └── ubuntu/
│       └── setup_ubuntu.sh                # Ubuntu / WSL2 初始化脚本（严格 LF，含 openpi 与 udev）
├── configs/                               # [配置扩展区] 仓库清单、机械臂参数与训练配置模板
│   ├── repos.yaml                         # 唯一权威的上游开源仓库注册表
│   ├── robots/                            # 机械臂配置模板（arm101_win.example.yaml 等）
│   └── train/                             # ACT / SmolVLA / Pi0 / Pi0.5(OpenPI) 训练配置
├── app/                                   # [本地串联软件] Arm101 Studio（FastAPI + 6轴火柴棍仿真/IK + 零构建 Web 前端）
│   ├── run_studio.py                      # 快捷启动入口（支持 --dry-run / --port）
│   ├── backend/                           # FastAPI + WebSocket 后端、安全/运动学仿真桥接与插件管理器
│   ├── frontend/                          # 原生 HTML5/CSS3/ESM 控制面板（含全场景 6 轴火柴棍仿真）
│   └── tests/                             # Studio 接口与前端行为测试
├── software/                              # [底层控制内核与 CLI] SO-101 运动学、SafetyController、预设姿态与跨平台 CLI
│   ├── main.py                            # 统一 CLI 入口（`web` 默认拉起 Arm101 Studio，`--legacy` 启动旧版）
│   ├── core/                              # SO101Kinematics、SafetyController、ActionAPI 与 rest/upright 姿态
│   └── tests/                             # 底层运动学、安全与跨平台相机/配置单元测试
├── repos/                                 # (gitignored) 上游开源仓库集中存放目录
│   ├── lerobot/                           # huggingface/lerobot
│   ├── leLab/                             # huggingface/leLab
│   ├── openpi/                            # Physical-Intelligence/openpi（Ubuntu/云端专用）
│   ├── SO-ARM100/                         # TheRobotStudio/SO-ARM100（arm101 硬件 CAD/文档）
│   └── feetech-motorlab/                  # Nori-Robotics/feetech-motorlab（舵机调试）
├── envs/                                  # (gitignored) Windows 本地 uv 虚拟环境（envs/win/lerobot）
├── data/                                  # (gitignored 除 README/.gitkeep) 数据集、标定、dry_run 产物
│   ├── datasets/
│   ├── calibration/
│   └── dry_run/
├── models/                                # (gitignored 除 README/.gitkeep) 预训练模型与训练 checkpoints
│   ├── pretrained/
│   └── checkpoints/
├── logs/                                  # (gitignored 除 README/.gitkeep) 进程与调试日志
└── tests/                                 # 工作区端到端验证测试（tests/e2e/）
```

---

## 2. Windows + Ubuntu 双系统协作与软件分工

本工作区支持 **Windows 主机与 Ubuntu（双系统或 WSL2）共用同一套代码、配置、教程与数据目录**（例如在 WSL2 中通过 `/mnt/c/.../lerobot` 访问，或在双系统机器上各自检出 `TUST-iMaker/lerobot`）。由于 Windows NTFS 与 Linux ext4 在文件权限、符号链接和 CUDA/JAX 依赖上存在差异，**两端的虚拟环境严格隔离安装**：

| 维度 | Windows 侧 | Ubuntu / WSL2 侧 |
|---|---|---|
| **虚拟环境位置** | 工作区内 `envs/win/lerobot`（Python 3.12，`uv`） | Linux 原生 ext4 文件系统：`~/.venvs/tust-lerobot`（Python 3.12）与 `~/.venvs/tust-openpi`（Python 3.11，可选） |
| **环境变量覆盖** | `LEROBOT_WIN_VENV` | `LEROBOT_UBUNTU_VENV` / `OPENPI_UBUNTU_VENV` |
| **硬件串口与相机** | `COM*`（DirectShow / MSMF，`find_ports.ps1`） | `/dev/ttyACM*` + `udev` 规则（`/etc/udev/rules.d/99-lerobot-arms.rules`）+ `V4L2` |
| **核心职责分工** | • 飞特/Feetech 舵机初始化（`feetech-motorlab`）<br>• arm101 标定、遥操作、示教、数据采集<br>• 数据集可视化与本地 **Arm101 Studio (`app/`)** 运行 | • 本地单卡/多卡 CUDA 训练（`lerobot-train`）<br>• **Physical-Intelligence/openpi**（JAX/CUDA12，仅支持 Linux，Python 3.11 + `numpy<2.0`）<br>• 远程策略推理服务（`serve_policy.py`）与云端算力（AutoDL / Featurize / HF Jobs）镜像调试 |

> **关键约定**：不要把 Linux `.venv` 建在 Windows NTFS（`/mnt/c/...`）上，也不要在 Windows 环境中安装 `openpi`（其 `jax[cuda12]` 与 `numpy<2.0` 与 Windows `lerobot` 0.7.1 冲突）。

---

## 3. 上游开源仓库清单（`configs/repos.yaml`）

所有上游依赖仓库均通过 `configs/repos.yaml` 声明并统一克隆至 `repos/`，严禁在工作区根目录或子目录重复克隆：

| 本地路径 (`repos/`) | 上游仓库 | 适用系统 | 作用说明 |
|---|---|---|---|
| `repos/lerobot` | [`huggingface/lerobot`](https://github.com/huggingface/lerobot) | Windows / Ubuntu | 官方核心框架（SO-101 驱动、遥操作、录制、ACT/SmolVLA/Pi0/Pi0.5） |
| `repos/leLab` | [`huggingface/leLab`](https://github.com/huggingface/leLab) | Windows / Ubuntu | Hugging Face 官方 Web 控制台（可选隔离安装） |
| `repos/openpi` | [`Physical-Intelligence/openpi`](https://github.com/Physical-Intelligence/openpi) | **仅 Ubuntu / 云端** | Physical Intelligence 官方 OpenPI（`pi0` / `pi0-FAST` / `pi0.5`） |
| `repos/SO-ARM100` | [`TheRobotStudio/SO-ARM100`](https://github.com/TheRobotStudio/SO-ARM100) | Windows / Ubuntu | 官方 SO-ARM100 / SO-ARM101 硬件 CAD（STEP/STL）、URDF 与装配资料 |
| `repos/feetech-motorlab` | [`Nori-Robotics/feetech-motorlab`](https://github.com/Nori-Robotics/feetech-motorlab) | Windows / Ubuntu | 飞特 STS3215 舵机 ID 配置、总线扫描与诊断工具 |

---

## 4. 快速开始（Quickstart）

### 第一步：克隆并检查上游开源仓库（Windows / Ubuntu 通用）

```bash
# 幂等克隆 configs/repos.yaml 中的全部 5 个非重复仓库到 repos/
python scripts/common/bootstrap_repos.py

# 查看当前仓库状态（支持 --json 与 --dry-run）
python scripts/common/bootstrap_repos.py --status
```

### 第二步：按操作系统初始化虚拟环境

**Windows（PowerShell）：**

```powershell
# 1. 非破坏性体检当前系统环境
powershell -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1 -CheckOnly

# 2. 创建 Python 3.12 虚拟环境 envs\win\lerobot 并安装 Studio + editable lerobot[feetech]
powershell -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1

# 3. 枚举已连接的 USB-串口（CH340 / STM32）与摄像头
powershell -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1
```

> 可选参数：`-WithCoreScripts`（安装录制/Rerun/数据集工具链）、`-WithLeLab`（以 `--no-deps` 安装官方 `leLab`）、`-WithPlaywrightBrowser`（下载 Chromium）、`-TorchBackend cu128`（在 Windows 启用 CUDA PyTorch）。

**Ubuntu / WSL2（Bash）：**

```bash
# 1. 仅检查环境与依赖（不修改任何文件）
bash scripts/ubuntu/setup_ubuntu.sh --check-only

# 2. 在 ~/.venvs/tust-lerobot 创建 Python 3.12 环境
bash scripts/ubuntu/setup_ubuntu.sh

# 3. 若需同时配置 OpenPI（Python 3.11）与串口 udev 规则：
bash scripts/ubuntu/setup_ubuntu.sh --with-openpi --setup-udev
```

### 第三步：运行跨平台自检与启动本地软件

```powershell
# 运行零依赖跨系统体检脚本
envs\win\lerobot\Scripts\python.exe scripts\common\check_env.py

# 启动本地 Arm101 Studio（支持无硬件时自动降级到 dry_run 模拟模式）
envs\win\lerobot\Scripts\python.exe -m app.backend.main
# 或使用快捷启动器：
envs\win\lerobot\Scripts\python.exe app\run_studio.py --port 8760
```

---

## 5. 各模块入口与长期扩充规范

- **教程体系入口**：请从 [`docs/README.md`](docs/README.md) 开始阅读（涵盖参考资料来源、双系统配置、arm101 标定/遥操作/示教/采集、VLA 云端训练、`pi0.7`/`pi0.5`/`pi0` 解析及开源前端工具对比）。
- **本地配置文件**：复制 `.env.example` 为 `.env`，复制 `configs/robots/arm101_win.example.yaml`（或 `arm101_ubuntu.example.yaml`）为对应的 `arm101_win.yaml` / `arm101_ubuntu.yaml` 填入本机串口号即可使用，个性化配置已被 `.gitignore` 自动忽略。
- **扩展文档（`docs/`）**：按阶段编号目录新增 Markdown 文档，并在 `docs/README.md` 与 `docs/sources/README.md` 登记引用来源。
- **扩展上游仓库（`configs/repos.yaml`）**：先在 `configs/repos.yaml` 添加条目再执行 `python scripts/common/bootstrap_repos.py`，脚本会自动校验 URL 唯一性并防止重复克隆。
- **扩展配置（`configs/`）**：在 `configs/robots/` 新增机型/站点模板（命名 `*.example.yaml` 以便纳入版本控制），在 `configs/train/` 新增策略训练 YAML。
- **扩展数据与模型（`data/`、`models/`）**：详见 [`data/README.md`](data/README.md) 与 [`models/README.md`](models/README.md)，所有大文件自动保持 gitignored。
- **扩展本地软件与插件（`app/`）**：新增工具适配器可注册至 `app/backend/plugins.py` 的 `ToolAdapter` 注册表，并在 `app/frontend/` 对应面板暴露调用入口（详见 [`docs/04_frontend_and_tools/local_app_guide.md`](docs/04_frontend_and_tools/local_app_guide.md)）。
