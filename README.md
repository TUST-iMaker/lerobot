# TUST-iMaker LeRobot 工作区 & Arm101 Studio

面向 **SO-ARM101 主从机械臂** 的双系统标准化工作区（`TUST-iMaker/lerobot`）与本地可视化控制平台 **Arm101 Studio**。
支持 **单臂/双臂真机实时联动** 与 **无硬件 `dry_run` 全流程仿真**，底层 100% 对齐官方 `lerobot v0.7.1` 与 `so101_new_calib.urdf`。

---

## 新人 5 分钟快速上手（第一次用按这 4 步走）

> **无机械臂也能直接练手**：没插硬件时软件会自动进入 `dry_run` 仿真模式，所有按钮、3D 四视图、键控和示教回放都能直接体验，不会报错也不会影响真机文件。

### 第 1 步：Windows / Linux 双环境配置与启动（首次运行做一次）

先在任意系统拉取上游开源仓库（`lerobot` / `SO-ARM100` 等，两个系统共用同一份 `repos/`）：
```bash
python scripts/common/bootstrap_repos.py
```

#### 方式 A：Windows 环境（PowerShell — 推荐日常接机械臂、标定、遥操作、采集）
```powershell
# 1. 创建工作区内置虚拟环境 envs\win\lerobot (Python 3.12) 并安装 Studio + lerobot[feetech]
powershell -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1

# 2. （可选）补装官方 lerobot-record / lerobot-replay 所需的 datasets 依赖
envs\win\lerobot\Scripts\python.exe -m pip install -e "repos/lerobot[dataset]"

# 3. 扫描已接主/从臂 COM 口与摄像头索引
powershell -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1

# 4. 启动 Arm101 Studio，然后浏览器打开 http://127.0.0.1:8760
envs\win\lerobot\Scripts\python.exe app\run_studio.py --port 8760
```

#### 方式 B：Linux / Ubuntu / WSL2 环境（Bash — 推荐本地 CUDA 训练、OpenPI 与 Linux 真机）
```bash
# 1. 在 Linux 原生文件系统创建 ~/.venvs/tust-lerobot (Python 3.12) 并配置串口 udev 规则
#    如需同时安装 OpenPI (π0/π0.5, Python 3.11) 请追加 --with-openpi
bash scripts/ubuntu/setup_ubuntu.sh --setup-udev

# 2. （可选）补装官方 lerobot-record / lerobot-replay 所需的 datasets 依赖
~/.venvs/tust-lerobot/bin/python -m pip install -e "repos/lerobot[dataset]"

# 3. 检查串口 (/dev/ttyACM*) 与环境体检
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
~/.venvs/tust-lerobot/bin/python scripts/common/check_env.py

# 4. 启动 Arm101 Studio，然后浏览器打开 http://127.0.0.1:8760
~/.venvs/tust-lerobot/bin/python app/run_studio.py --port 8760
```

> **不想配环境？官方免代码桌面软件备选**：可以直接去矽递科技官网下载 **SenseCraft Robotics** 图形化客户端（支持 Windows / macOS / Linux）：[`https://sensecraft.seeed.cc/`](https://sensecraft.seeed.cc/)（中文说明见 [Seeed Wiki](https://wiki.seeedstudio.com/cn/sensecraft_robotics_overview/)），安装登录即可免代码完成连接、标定、采集与云端训练。

### 第 2 步：连接硬件并看懂顶栏状态
1. 插上主臂、从臂 USB 与 12V 电源，打开网页进入 **「1. 总览与标定」** → 点右上角 **「自动识别 → 保存配置」**（自动写入 `.env`）。
2. **随时看顶栏左上角状态胶囊**，一眼知道现在能不能动：
   - 绿色 **`仿真就绪` / `真机双向就绪`**：可直接操作。
   - 黄色 **`真机待命（未同步）`**：此时掰真机 3D 不会动，点首页绿色按钮 **「开启真机→仿真实时同步」** 或进入 **「2. 遥操作」** 即可联动。
   - 红色 **`急停锁定`**：确认桌面安全后点顶栏 **「解除急停」**。

### 第 3 步：首次使用真机先标定（只需做一次）
- 在 **「1. 总览与标定」** 选中 `so101_follower (从臂)` → 点 **「开始标定」**：
  1. **对中位**：舵机自动变松后，用手扶成 **底座居中、大臂竖直向上、小臂与夹爪水平向前平直**（与画面灰虚线重合）→ 点 **「确认 / 下一步」**。
  2. **扫限位**：逐个把 6 个关节掰到最小和最大极限 → 确认表格数值有变化 → 点 **「确认 / 下一步」** 完成保存。
- 换选 `so101_leader (主臂)` 重复一次即可。

### 第 4 步：日常 4 种最常用玩法
| 你想做什么 | 怎么点 |
| :--- | :--- |
| **主臂带从臂遥操作** | 进 **「2. 遥操作驾驶舱」** → 点 **「开始遥操作」** → 用手移动主臂，从臂和 3D 实时跟随 |
| **电脑控臂 / 多视角观察** | 进 **「3. 6 轴双向仿真」** 拖滑块、拖红圈或按 `WASD + Q/E`；点顶栏 **「弹出独立仿真窗 → 从臂四视图」** 可同时看 4 个视角 |
| **教一遍让从臂线性复现** | 进 **「4. 示教与表演」** → 点 **「开始连续示教」** 主臂带从臂做一段动作 → 点 **「停止并保存」** → 点 **「线性回放选中轨迹」** |
| **采数据训练自主抓取** | 进 **「5. 数据采集」** 录 30~50 条数据 → 按 [采集与训练教程](docs/DATA_COLLECTION_AND_TRAINING_GUIDE.md) 一键训 ACT 上真机 |

> **安全 3 条铁律**：① 任何异常立刻点右上角红色 **「E-STOP 急停」**；② 关机前先点顶栏 **「蜷缩休眠」** 再断电，防止机械臂自由落体摔坏；③ 标定没对齐前不要直接上力大幅运动。

---

## 一、先看这 3 份实用速查文档

| 文档 | 内容 |
| :--- | :--- |
| **[官方与社区工具速查表](docs/OFFICIAL_TOOLS_CHEATSHEET.md)** | 一页看懂 Hugging Face LeRobot、矽递科技、同济子豪兄、Datawhale 现成工具与网址 |
| **[全功能 ↔ 完整 CLI 命令对照表](docs/STUDIO_TO_CLI_COMPLETE_MAP.md)** | 按「点与线」完整列出本系统每一项功能对应的官方/等效 CLI 命令与物理动作 |
| **[数据采集与模型训练实战教程](docs/DATA_COLLECTION_AND_TRAINING_GUIDE.md)** | 从标定检查、采集规范、清洗到 ACT / SmolVLA / π0 训练与真机推理的完整步骤 |

> 更多背景资料与分章节手册见 [`docs/README.md`](docs/README.md)。

---

## 二、快速启动（Quickstart）

### 1. 拉取上游开源仓库（Windows / Ubuntu 通用）
```bash
python scripts/common/bootstrap_repos.py
```

### 2. 初始化本机环境

**Windows（PowerShell，硬件操控与本地 Studio）：**
```powershell
# 创建 envs/win/lerobot (Python 3.12) 并安装 Studio + lerobot[feetech]
powershell -ExecutionPolicy Bypass -File scripts\windows\setup_win.ps1

# 如需官方 lerobot-record / lerobot-replay 数据集读写，补装 dataset 依赖：
envs\win\lerobot\Scripts\python.exe -m pip install -e "repos/lerobot[dataset]"

# 扫描主/从臂 COM 口与摄像头索引
powershell -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1
```

**Ubuntu / WSL2（Bash，本地 CUDA 训练与 OpenPI）：**
```bash
# 创建 ~/.venvs/tust-lerobot；加 --with-openpi --setup-udev 可一并配 OpenPI 与串口规则
bash scripts/ubuntu/setup_ubuntu.sh
```

### 3. 自检并启动 Arm101 Studio
```powershell
# 跨平台环境体检
envs\win\lerobot\Scripts\python.exe scripts\common\check_env.py

# 启动 Arm101 Studio（默认 http://127.0.0.1:8760，自动识别真机或降级 dry_run）
envs\win\lerobot\Scripts\python.exe app\run_studio.py --port 8760
# 或模块化启动：
envs\win\lerobot\Scripts\python.exe -m app.backend.main
```

---

## 三、Arm101 Studio 核心功能一览

- **全局顶栏 & 常驻 3D 舞台**：
  - 实时 **系统就绪状态胶囊**（`仿真就绪` / `真机双向就绪` / `真机→仿真监听中` / `真机待命` / `急停锁定`）、单/双臂真机模式切换、**STS3215 舵机直写上力/卸力**、**安全蜷缩休眠**、**E-STOP 急停与复位**、浅色/深色主题切换。
  - 左侧 3D 舞台基于官方 URDF 正逆运动学渲染（含 3D 平行夹爪滚转与直立底座），支持鼠标红圈平面投影 IK 拖拽，并可 **弹出独立仿真窗（`/sim-viewport`）**，支持 **双臂视图 / 从臂四视图 / 主臂四视图（轴测、正视、侧视、俯视 2×2 联动）**。
- **7 大业务工作台**：

| 标签页 | 核心功能 |
| :--- | :--- |
| **1. 总览与标定** | 硬件串口/相机探测、`.env` 设备映射、系统就绪引导条、交互式零点与行程标定（真机直连 `lerobot-calibrate`，模拟模式隔离演练） |
| **2. 遥操作驾驶舱** | 主从臂双向实时同步（真机动→3D动，界面动→真机动）、双目 **MJPEG 实时视频流**（~25 FPS） |
| **3. 6 轴双向仿真** | 主/从臂 6-DOF 滑块、主从联动互斥防抢控、WASD + Q/E 末端空间键控与边界钳制 |
| **4. 示教与表演** | **官方连续轨迹示教与线性回放**（`lerobot-record` + `lerobot-replay`）、自由示教卸力记录航点、预设姿态与示范动作 |
| **5. 数据采集** | 官方 `LeRobotDataset v3.0` 多回合采集（防 `eval_` 命名误伤、支持 `--resume` 续录） |
| **6. 插件与开源数据** | Feetech MotorLab、LeLab、OpenPI 策略服务与开源数据集入口 |
| **7. VLA 云端训练** | ACT / SmolVLA / π0 / π0.5 配置模板预览与训练命令生成 |

---

## 四、当前目录结构（已精简清理历史遗留文件）

```text
lerobot/
├── README.md                  # 本文件
├── pyproject.toml             # 项目元数据、依赖与 pytest/ruff 配置（Python >= 3.12）
├── .env.example               # 串口、相机与模式环境变量模板（复制为 .env 使用）
├── app/                       # Arm101 Studio 本体（FastAPI + WebSocket + 原生 ESM 前端）
│   ├── run_studio.py          # 启动入口（含 2s 优雅退出超时）
│   ├── backend/               # hardware / simulation / realtime_bridge / workflows / calibrate_runner / plugins
│   ├── frontend/              # index.html / sim_viewport.html / stick_sim.js / app.js / studio_bus.js / styles.css
│   └── tests/                 # Studio API 自动化测试
├── software/                  # 精简后的 SO-101 底层控制与运动学内核（无冗余旧脚本）
│   ├── core/                  # SO101Kinematics (URDF) / SafetyController / StateMachine / ActionAPI / Presets
│   ├── hardware/              # SO101FollowerArm / SO101LeaderArm / MockArmAdapter / RobotSystemAdapter
│   ├── teleop/                # official_wrapper.py（官方 CLI 封装与安全休眠）
│   ├── config.yaml            # 核心安全限位与默认参数
│   ├── rest_pose.json         # 默认安全休眠位
│   ├── upright_pose.json      # 默认中位
│   └── tests/                 # 运动学、安全控制器与驱动单元测试
├── configs/                   # 上游仓库注册表与配置模板
│   ├── repos.yaml             # 权威上游仓库清单
│   ├── robots/                # arm101_win.example.yaml / arm101_ubuntu.example.yaml
│   └── train/                 # act_so101 / smolvla_so101_cloud / pi0_so101_cloud / pi05_openpi_cloud
├── scripts/                   # 跨平台初始化与硬件扫描脚本（bootstrap_repos.py / check_env.py / setup_* / find_ports.ps1）
├── docs/                      # 速查表、CLI 对照表、采集训练教程与参考资料
├── tests/                     # 端到端分级测试套件（tests/e2e/ Tier 1~4）
├── repos/                     # (gitignored) 统一拉取的上游仓库（lerobot / SO-ARM100 / leLab / openpi / feetech-motorlab）
├── envs/                      # (gitignored) Windows 本地 uv 虚拟环境（envs/win/lerobot）
├── data/                      # (gitignored) calibration/ 标定文件、datasets/ 真实数据集、dry_run/ 模拟隔离数据
├── models/                    # (gitignored) pretrained/ 预训练权重与 checkpoints/ 训练产物
└── logs/                      # (gitignored) 运行日志
```

---

## 五、Windows + Ubuntu 双系统分工

| 维度 | Windows 侧 | Ubuntu / WSL2 侧 |
| :--- | :--- | :--- |
| **虚拟环境** | 工作区内 `envs/win/lerobot`（Python 3.12） | Linux 原生 ext4：`~/.venvs/tust-lerobot`（3.12）与 `~/.venvs/tust-openpi`（3.11，可选） |
| **设备接口** | `COM*` + OpenCV DirectShow（`find_ports.ps1`） | `/dev/ttyACM*` + `/etc/udev/rules.d/99-lerobot-arms.rules` + V4L2 |
| **主要职责** | 舵机初始化、标定、遥操作、连续示教回放、数据采集、运行 Arm101 Studio | 本地/云端 CUDA 训练（`lerobot-train`）、`openpi`（JAX/CUDA12）与远程策略推理服务 |

> 注意：不要把 Linux 虚拟环境建在 Windows NTFS（`/mnt/c/...`）上，也不要在 Windows 安装 `openpi`。

---

## 六、上游开源仓库清单（`configs/repos.yaml`）

| 本地路径 | 上游仓库 | 适用系统 | 作用 |
| :--- | :--- | :--- | :--- |
| `repos/lerobot` | [huggingface/lerobot](https://github.com/huggingface/lerobot) | Windows / Ubuntu | 官方核心框架（SO-101 驱动、标定、遥操作、录制/回放、训练推理） |
| `repos/SO-ARM100` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | Windows / Ubuntu | 官方硬件 CAD 与 `so101_new_calib.urdf` 模型 |
| `repos/feetech-motorlab` | [Nori-Robotics/feetech-motorlab](https://github.com/Nori-Robotics/feetech-motorlab) | Windows / Ubuntu | 飞特 STS3215 舵机 ID 写入与寄存器诊断 |
| `repos/leLab` | [huggingface/leLab](https://github.com/huggingface/leLab) | Windows / Ubuntu | Hugging Face 社区 Web 控制台 |
| `repos/openpi` | [Physical-Intelligence/openpi](https://github.com/Physical-Intelligence/openpi) | 仅 Ubuntu / 云端 | Physical Intelligence OpenPI（π0 / π0.5） |
