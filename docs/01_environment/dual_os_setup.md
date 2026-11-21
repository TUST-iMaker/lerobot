# Windows + Ubuntu 双系统协同与环境初始化指南

本工作区采用 **Windows 与 Ubuntu（双系统或 WSL2）共用同一套代码、教程、配置与数据目录，同时严格隔离两端 Python 虚拟环境**的设计，确保团队成员既能在 Windows 端稳定连接真机硬件与本地前端，又能在 Ubuntu / 云端 GPU 上无痛运行 CUDA 训练与 `Physical-Intelligence/openpi`。

---

## 1. 双系统协同架构与职责分工

```text
共享工作区根目录（Windows: c:\Users\Administrator\Desktop\lerobot  <-->  Ubuntu/WSL2: /mnt/c/.../lerobot 或同步检出）
├── .env                   <-- 从 .env.example 复制（各自机器独立，gitignored）
├── configs/               <-- 共享模板；arm101_win.yaml 与 arm101_ubuntu.yaml 各自维护
├── repos/                 <-- bootstrap_repos.py 统一拉取的 5 个上游仓库（共有一份，严禁重复克隆）
├── data/                  <-- 共享标定 JSON、数据集（LeRobotDataset v3.0）与 dry_run 产物
├── models/                <-- 共享预训练权重与训练 checkpoints
├── envs/win/lerobot/      <-- [仅 Windows] Python 3.12 uv 虚拟环境
└── ~/.venvs/              <-- [仅 Ubuntu/WSL2，位于原生 ext4]
    ├── tust-lerobot/      <-- Python 3.12（lerobot 核心 + 可选训练/core_scripts）
    └── tust-openpi/       <-- Python 3.11（仅 OpenPI 专用，含 jax[cuda12] + numpy<2.0）
```

### 分工矩阵

| 维度 | Windows 侧（主力硬件与现场控制） | Ubuntu / WSL2 侧（本地 CUDA 训练与 OpenPI） |
|---|---|---|
| **虚拟环境路径** | `envs\win\lerobot`（环境变量 `LEROBOT_WIN_VENV`） | `~/.venvs/tust-lerobot` 与 `~/.venvs/tust-openpi`（必须在 Linux ext4，不可放在 `/mnt/c/...`） |
| **串口发现与设备路径** | `COM3` / `COM4`（[`scripts/windows/find_ports.ps1`](../../scripts/windows/find_ports.ps1)） | `/dev/ttyACM0` / `/dev/ttyACM1` + `udev` 规则（`99-lerobot-arms.rules`）+ `dialout` 组 |
| **相机后端** | OpenCV DirectShow（`backend: 700`） | V4L2（`backend: 200`，常见偶数设备号 `/dev/video0`, `/dev/video2`） |
| **核心任务** | 舵机调试（`feetech-motorlab`）、arm101 标定、遥操作、示教采数、数据集可视化与本地 **Arm101 Studio (`app/`)** | 多卡/单卡 `lerobot-train`、**`openpi`（仅支持 Ubuntu 22.04）**、远程策略推理服务（`serve_policy.py`） |

> **关键约定**：`.gitattributes` 已强制 `*.sh` 使用 **LF** 换行、`*.ps1` 使用 **CRLF** 换行；在 Windows 上修改 `.sh` 时务必保持 LF（可运行 `scripts/common/check_env.py` 自动检查）。

---

## 2. 第一步：通用前置准备（`.env` 与上游仓库克隆）

在任意系统拉取工作区后，先复制环境变量模板并初始化上游仓库：

```powershell
# Windows PowerShell（工作区根目录）
Copy-Item .env.example .env
```

```bash
# Ubuntu / WSL2 Bash（工作区根目录）
cp .env.example .env
```

`.env` 中常用关键配置（按需取消注释并修改）：
- `HF_LEROBOT_HOME=./data`：统一把标定文件与本地数据集保存在工作区 `data/` 下，实现双系统共享。
- `HF_ENDPOINT=https://hf-mirror.com`：国内网络环境可切换至 Hugging Face 镜像加速模型与数据集下载。
- `ARM101_FOLLOWER_PORT_WIN` / `ARM101_LEADER_PORT_WIN` 与 `ARM101_FOLLOWER_PORT_UBUNTU` / `ARM101_LEADER_PORT_UBUNTU`。

接着使用 [`scripts/common/bootstrap_repos.py`](../../scripts/common/bootstrap_repos.py) 幂等克隆 [`configs/repos.yaml`](../../configs/repos.yaml) 中的 5 个仓库到 `repos/`：

```bash
# 查看当前 5 个仓库克隆状态（不修改磁盘）
python scripts/common/bootstrap_repos.py --status

# 预览将要执行的克隆/更新命令（--dry-run）
python scripts/common/bootstrap_repos.py --dry-run

# 克隆全部必需上游仓库到 repos/（自动设置 GIT_LFS_SKIP_SMUDGE=1）
python scripts/common/bootstrap_repos.py

# 仅克隆指定仓库（如仅克隆 lerobot 与 SO-ARM100）
python scripts/common/bootstrap_repos.py --only lerobot,SO-ARM100

# 后续在工作区干净时快进更新上游代码
python scripts/common/bootstrap_repos.py --update
```

---

## 3. 第二步（A）：Windows 侧环境初始化

Windows 侧使用 [`scripts/windows/setup_win.ps1`](../../scripts/windows/setup_win.ps1)，由 `uv` 自动安装并绑定 **Python 3.12** 至 `envs\win\lerobot`（即使本机全局安装了 Python 3.14 也不会冲突）。

### 3.1 常用安装命令

```powershell
# 1. 非破坏性预检（检查 git / uv / 仓库 / 换行符，不修改任何环境）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1 -CheckOnly

# 2. 预览安装步骤（DryRun）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1 -DryRun

# 3. 标准安装（推荐给日常遥操作、标定、采数与可视化用户）：
#    默认安装 Studio 依赖 + repos/lerobot[feetech]；
#    加上 -WithCoreScripts 才会安装 datasets / rerun-sdk / pynput（lerobot-record 与 display_data=true 必需！）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1 -WithCoreScripts
```

### 3.2 `-WithCoreScripts` 与可选参数详解

| 参数 | 作用说明 | 何时需要 |
|---|---|---|
| *(无额外参数)* | 仅安装 `pyproject.toml` 核心依赖 + `repos/lerobot[feetech]`（含 `rustypot` / `pyserial`）+ `--no-deps -e .` | 仅使用本地 Studio `dry_run`、舵机端口测试或基础连接 |
| `-WithCoreScripts` | 安装 `repos/lerobot[feetech,core_scripts]`（补全 `datasets`、`torchcodec`、`pynput`、`rerun-sdk`、`foxglove-sdk`） | **使用 `lerobot-record` 采集数据、Rerun 实时显示（`display_data: true`）与 `lerobot-dataset-viz` 时必选** |
| `-WithLeLab` | 补充 `psutil`、`pygrabber` 并以 `--no-deps` 安装 `repos/leLab` | 需要对比体验官方 `leLab` Web UI 时使用 |
| `-WithPlaywrightBrowser` | 执行 `python -m playwright install chromium` | 运行自动化 UI/E2E 浏览器检查或抓取渲染页面时使用 |
| `-TorchBackend cu128` | 透传 `--torch-backend cu128` 给 `uv pip install` | 在具备 NVIDIA GPU（如 RTX 5060 Blackwell `sm_120`）的 Windows 机器启用 CUDA PyTorch |
| `-SkipLerobotInstall` | 跳过 `repos/lerobot` 安装，仅装根项目 `app/` 与测试依赖 | 仅开发/测试 `app/` 无硬件功能时 |

### 3.3 Windows 串口与摄像头探测

```powershell
# 列出当前所有串口（自动识别 CH340/CH343/STM32/CP210x/FTDI）及摄像头 PnP 设备
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1

# JSON 格式输出（便于脚本解析）或包含已断开的历史端口
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1 -Json
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1 -IncludeDisconnected

# 交互式拔线差值探测（先插好再按提示拔掉一条机械臂 USB，即可精确定位 COM 口）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1 -Diff
```

---

## 4. 第二步（B）：Ubuntu / WSL2 侧环境初始化

Ubuntu / WSL2 侧使用 [`scripts/ubuntu/setup_ubuntu.sh`](../../scripts/ubuntu/setup_ubuntu.sh)。注意在 WSL2 中请先用 `uv` 将环境创建在 Linux 原生 ext4 的 `~/.venvs/` 中，切勿写入 `/mnt/c/...`。

### 4.1 常用安装命令

```bash
# 1. 非破坏性检查（不改动系统与虚拟环境）
bash scripts/ubuntu/setup_ubuntu.sh --check-only

# 2. 预览将要执行的安装操作
bash scripts/ubuntu/setup_ubuntu.sh --dry-run

# 3. 创建 Python 3.12 环境 ~/.venvs/tust-lerobot，并安装核心脚本 + 训练工具链（含 trackio）+ udev 串口规则
bash scripts/ubuntu/setup_ubuntu.sh --with-core-scripts --with-training --setup-udev

# 4. 若需同时初始化 Physical-Intelligence/openpi（独立 Python 3.11 环境 ~/.venvs/tust-openpi）
bash scripts/ubuntu/setup_ubuntu.sh --with-core-scripts --with-training --with-openpi --setup-udev
```

### 4.2 参数说明与 `udev` 持久化规则

| 参数 | 作用说明 |
|---|---|
| `--with-core-scripts` | 安装 `lerobot[feetech,core_scripts]`，支持 Ubuntu 端直接采数与 Rerun 可视化 |
| `--with-training` | 安装 `lerobot[feetech,training]`（包含 `datasets` 与 `trackio[gpu]`），支持 `lerobot-train` |
| `--with-openpi` | 在独立目录 `~/.venvs/tust-openpi` 使用 **Python 3.11** 同步 `repos/openpi`（自动 `git submodule update --init --recursive` 与 `GIT_LFS_SKIP_SMUDGE=1 uv sync`），避免 `jax[cuda12]` / `numpy<2.0` 与 `lerobot` 冲突 |
| `--setup-udev` | 写入 `/etc/udev/rules.d/99-lerobot-arms.rules`（匹配 `1a86:7523`、`1a86:55d3`、`0483:5740`、`10c4:ea60`、`0403:*`，赋予 `MODE="0666", GROUP="dialout", TAG+="uaccess"`）并把当前用户加入 `dialout` 组 |
| `--skip-lerobot-install` | 仅安装工作区基础依赖，跳过 `repos/lerobot` |

> **生效提示**：执行 `--setup-udev` 后，请重新插拔 USB 线并重新登录会话（或执行 `newgrp dialout`）以使 `dialout` 组权限生效。

### 4.3 WSL2 专项注意事项
1. **USB 串口透传**：WSL2 默认无法直通 Windows USB 串口，需在 Windows 管理员终端使用 [`usbipd-win`](https://github.com/dorssel/usbipd-win) 执行 `usbipd bind --busid <BUSID>` 与 `usbipd attach --wsl --busid <BUSID>`；绝大多数场景下**直接建议在 Windows 端运行硬件标定与采数**，WSL2/Ubuntu 仅负责训练与 `openpi`。
2. **`libevdev-dev` 依赖**：在 WSL2 / 无头 Linux 安装 `pynput`（包含在 `core_scripts` 中）前，若缺少内核头文件需先运行 `sudo apt install -y libevdev-dev`。
3. **WSL2 本地代理告警**：若在 Windows 开启了 `127.0.0.1` 代理而 WSL2 环境变量沿用 `localhost` 代理，会导致 `uv`/`git` 无法连接外网（需改用镜像网络模式或宿主机网关 IP）。

---

## 5. 第三步：跨平台健康体检（`check_env.py`）

[`scripts/common/check_env.py`](../../scripts/common/check_env.py) 是零第三方依赖的自检脚本，可在安装前后随时运行：

```bash
# Windows（也可直接使用系统 python 运行）
envs\win\lerobot\Scripts\python.exe scripts\common\check_env.py

# Ubuntu
~/.venvs/tust-lerobot/bin/python scripts/common/check_env.py

# CI / 严格检查（任意一项 FAIL 则返回非 0 退出码）及 JSON 导出
python scripts/common/check_env.py --strict
python scripts/common/check_env.py --json
```

检查项目涵盖 6 大类：`os_runtime`、`cli_tools`、`git_and_repos`、`shell_line_endings`（校验 `.sh` 是否被误存为 CRLF）、`venvs`（探测关键包 `fastapi`、`uvicorn`、`pyserial`、`cv2`、`lerobot`、`rustypot` 等）及 `hardware_snapshot`。

---

## 6. Windows 与 Ubuntu 硬件访问差异速查

| 对比项 | Windows | Ubuntu |
|---|---|---|
| **串口命名** | `COM3`, `COM4`, ...（使用 `find_ports.ps1` 或 `lerobot-find-port` 探测） | `/dev/ttyACM0`, `/dev/ttyACM1`（建议配 `udev` 固定别名） |
| **相机索引** | `0`, `1`（使用 `lerobot-find-cameras opencv` 探测；OpenCV `backend: 700` DirectShow / DSHOW 推荐；`1400` MSMF） | `0`, `2` 或 `/dev/video0`, `/dev/video2`（使用 `lerobot-find-cameras opencv` 探测；OpenCV `backend: 200` V4L2，一个硬件通常占两个连续节点，偶数节点为图像流） |
| **高帧率图像格式** | 推荐在配置中启用 `fourcc: MJPG`（防止多路 1080p YUYV 占满 USB 带宽） | 同样推荐 `fourcc: MJPG` |
| **配置文件切换** | 复制 [`configs/robots/arm101_win.example.yaml`](../../configs/robots/arm101_win.example.yaml) -> `arm101_win.yaml` | 复制 [`configs/robots/arm101_ubuntu.example.yaml`](../../configs/robots/arm101_ubuntu.example.yaml) -> `arm101_ubuntu.yaml` |

完成上述环境初始化与 `check_env.py --strict` 检查后，即可进入下一篇：[`01_calibration_and_ports.md`](../02_arm101_workflow/01_calibration_and_ports.md) 开始 `arm101` 端口确认与标定！
