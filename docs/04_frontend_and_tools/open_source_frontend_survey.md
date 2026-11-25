# LeRobot 开源可视化前端与舵机工具深度调研及规范化接入指南

本文档对 LeRobot 与 **官方 `arm101`（SO-ARM101）主从臂**生态中主流的 **开源可视化前端、Web 工作台及飞特（Feetech STS3215）舵机调试工具** 进行系统性横向对比，阐明本工作区**为什么明确禁止沿用早期散乱的旧结构**，并定义了通过 **`ToolAdapter`（插件适配器）+ `repos/` 子进程环境隔离** 接入现有开源工具的架构规范。

> 若想直接启动本工作区新建的本地串联软件 **Arm101 Studio**，请同时参阅同目录下的 [`local_app_guide.md`](local_app_guide.md)。

---

## 1. 主流开源可视化前端与生态工具全景评测

我们围绕 **硬件标定、遥操作、示教、数据采集、数据集可视化与舵机底层调试** 六大核心需求，对社区内 9 款代表性工具进行了源码级调研与依赖兼容性核查：

| 工具名称 | 上游仓库 / 入口 | 开源协议 | 核心定位与功能亮点 | 依赖版本与系统限制 | 本工作区接入策略 |
|---|---|---|---|---|---|
| **`leLab`** | [`huggingface/leLab`](https://github.com/huggingface/leLab)（已克隆至 [`repos/leLab`](../../repos/leLab/)） | Apache-2.0 | Hugging Face 官方推出的 SO-ARM101 Web 控制台（FastAPI + React），支持端口向导、标定、遥操作、录制与相机旋转配置 | 其 `pyproject.toml` 锁定 `lerobot@v0.7.0`；若与主环境 `v0.7.1` 混装会引发版本降级，且需预编译前端 `static/` 或通过 `uv tool` 独立运行 | 克隆于 `repos/leLab`，通过 `ToolAdapter`（`lelab_external`）以独立子进程或 `uv tool run lelab` 隔离调用 |
| **`rerun`** | `rerun-sdk`（内置于 `lerobot[viz]` / `lerobot-dataset-viz`） | MIT / Apache-2.0 | LeRobot 官方默认的多模态时序可视化引擎，支持 30/60 Hz 实时遥操作与 `LeRobotDataset v3.0` 离线轨迹 + 双相机回看 | 支持 Windows 与 Ubuntu 原生窗口及 Web Viewer，也可导出 `.rrd` 离线文件 | 主环境原生安装，通过 `ToolAdapter`（`lerobot_dataset_viz`）一键拉起本地数据集可视化 |
| **`lerobot-studio`** | `lerobot-studio` / `huggingface.co/spaces/lerobot/visualize_dataset` | Apache-2.0 | 免安装的浏览器端 `LeRobotDataset`（v2 / v3）交互式可视化与回合帧级审查面板 | 纯 Web 端或本地静态启动，适合云端数据集与本地数据集快速抽检 | 通过 `ToolAdapter`（`lerobot_studio`）提供一键浏览器/本地可视化跳转与参数拼接 |
| **`feetech-motorlab`** | [`Nori-Robotics/feetech-motorlab`](https://github.com/Nori-Robotics/feetech-motorlab)（已克隆至 [`repos/feetech-motorlab`](../../repos/feetech-motorlab/)） | MIT | 专为飞特 STS3215 总线舵机打造的 Web 调试台（`app.py`），支持扫描 ID（`1..6`）、修改波特率、实时监控电压/温度/负载与中位标定 | 纯 Python + 轻量 Web 界面，跨平台支持 Windows `COM*` 与 Ubuntu `/dev/ttyACM*` | 克隆于 `repos/feetech-motorlab`，通过 `ToolAdapter`（`feetech_servo_bench`）由受控子进程直接启动与回收 |
| **`sts-studio` / `Seeed_RoboController`** | `sts-studio` / `Seeed-Projects/Seeed_RoboController` | MIT | 飞特 STS 舵机命令行与极简上位机工具集（提供 `servo_disable` 卸力、`servo_middle_calibration` 中位归零、`servo_center_test`） | 适合组装初期的单舵机排障，但缺乏遥操作、双相机同步与 `LeRobotDataset v3.0` 录制能力 | 归纳其舵机卸力与中位校验思想至标定教程，底层舵机可视化统一由 `feetech-motorlab` 承载 |
| **`phosphobot`** | `phospho-app/phosphobot` | MIT | 面向 SO-100/SO-101 的跨平台控制服务端与 Web 仪表盘，支持键盘/手柄/VR 遥操及云端训练对接 | 独立后台守护进程与自有数据格式，部分依赖与 `lerobot v0.7.1` 主环境存在版本冲突 | 不混入主 Python 虚拟环境；需要时建议使用 `uvx` 或独立虚拟环境在外部运行 |
| **`solo-cli`** | `GetSoloTech/solo-cli` | Apache-2.0 | 面向物理 AI 的命令行交互式引导工具，封装了标定、遥操与推理菜单 | 强绑定特定旧版 `lerobot` 依赖树，CLI 交互无法嵌入 Web 前端 | 不混装至主环境；其菜单化引导理念已由本工作区 **Arm101 Studio** Web 界面全面替代 |
| **Foxglove Studio** | `lerobot-teleoperate --display_mode=foxglove` | MPL-2.0 | 工业级机器人 WebSocket 可视化面板（`lerobot[foxglove]`） | 需额外安装 `foxglove-websocket` 客户端 | 作为 `rerun` 的可选替代方案在遥操作与可视化教程中提供命令示例 |
| **SenseCraft Robotics** | 矽递科技（Seeed Studio）桌面客户端 | 专有软件（非开源） | 面向零代码初学者的图形化桌面套件 | 闭源商业软件，不支持深度定制 VLA 训练配置与源码级扩展 | 仅在参考来源中列明，本规范工作区坚持采用 100% 开源可控方案 |

---

## 2. 深度技术剖析：核心开源工具的优劣势与适用边界

### 2.1 核心可视化与综合工作台（`leLab`、`rerun`、`lerobot-studio`）

1. **`leLab`（`huggingface/leLab`）**：
   - **优势**：专门针对 SO-ARM101 设计的现代 Web 界面，内置串口拔插识别向导、标定引导以及相机画面 `90°/180°/270°` 旋转修正（非常适合手腕倒装相机）。
   - **局限**：
     - `repos/leLab/pyproject.toml` 明确固定了 `lerobot @ git+https://github.com/huggingface/lerobot.git@v0.7.0`，落后于本工作区主库 `v0.7.1`；
     - 仓库源码中的 React 前端（`frontend/`）需要 Node.js 构建生成 `static/`，且在无真实机械臂连接时缺乏全流程 `dry_run` 虚拟关节仿真与示教轨迹点编辑器。
2. **`rerun`（`rerun-sdk` + `lerobot-dataset-viz`）**：
   - **优势**：C++/Rust 高性能渲染内核，在 60 FPS 遥操作和回看 `LeRobotDataset v3.0` 时几乎零延迟，完美展示 6 自由度 `observation.state`、`action` 曲线与双路摄像头画面。
   - **局限**：它是一个**纯数据观测器（Viewer）**，本身不提供“开始/停止遥操”、“录制示教点”、“管理数据集”或“一键启动训练”的交互控制按钮。
3. **`lerobot-studio`**：
   - **优势**：无需启动本地桌面 GUI，直接在浏览器中检视数据集的每一个 episode、核对语言指令 `task` 与关节动作对齐情况。
   - **局限**：聚焦于数据集浏览，不包含串口硬件控制与实时主从臂遥操能力。

### 2.2 舵机调试与社区衍生工具（`feetech-motorlab`、`sts-studio`、`phosphobot`、`solo-cli`）

1. **`feetech-motorlab` vs `sts-studio`**：
   - 官方 `arm101` 主从臂共使用 12 颗飞特 **STS3215** 串行总线舵机（从臂 6 颗 `1:345` 减速比；主臂 6 颗混合减速比）。当出现某颗舵机 ID 冲突、通信超时或过载保护时，`lerobot-calibrate` 只能报错而难以单点排障。
   - **`feetech-motorlab`**（已克隆在 `repos/feetech-motorlab`）提供了现代化的 Web 舵机工作台，比传统的 `sts-studio` 或散装命令行脚本更直观，因此被选为本工作区的标准舵机底层诊断插件。
2. **`phosphobot` 与 `solo-cli`**：
   - 这两款工具均试图在 `lerobot` 之上再包一层自有框架，但由于它们各自锁定了不同时期的 `lerobot`、`torch` 与 `fastapi` 版本，如果直接 `pip install` 到主工作区虚拟环境中，会立刻破坏 `lerobot v0.7.1` 的依赖一致性。

---

## 3. 架构规范：为什么坚决【不沿用旧结构】并实行重构

在早期个人实验或参考社区（如部分早期教学脚本、子豪兄早期 `v0.4.3` 散装脚本或混合克隆目录）时，常见的**不规范旧结构**存在以下严重隐患，本工作区**明确禁止沿用旧结构**，并按长期维护标准进行了全面重构：

1. **禁止将脚本散落在仓库根目录或直接污染上游源码**：
   - **旧结构痛点**：在 `lerobot` 仓库根目录随意堆放 `test1.py`、`my_teleop.sh`、`calib.json`，导致 `git pull` 冲突、路径硬编码失效、多人协作时无法区分哪些是官方源码、哪些是团队自研代码。
   - **本工作区规范结构**：上游开源仓库统一隔离在 [`repos/`](../../repos/)（由 [`configs/repos.yaml`](../../configs/repos.yaml) 与 [`scripts/common/bootstrap_repos.py`](../../scripts/common/bootstrap_repos.py) 声明式管理）；本团队的配置、脚本、标定数据与自研前端分别严格归档至 `configs/`、`scripts/`、`data/calibration/` 与 `app/`。
2. **禁止跨版本混装虚拟环境（依赖隔离原则）**：
   - **旧结构痛点**：把 `lerobot v0.4.3`、`lerobot v0.7.1`、`leLab (v0.7.0)`、`openpi (Python 3.11 + JAX)` 强行装进同一个 Conda 环境，导致 `ImportError`、`torchcodec` 崩溃或 CLI 入口被覆盖。
   - **本工作区规范结构**：
     - Windows 使用 `envs/win/lerobot`（Python 3.12，`uv` 管理）；
     - Ubuntu 使用 `~/.venvs/tust-lerobot`（Python 3.12）与独立的 `~/.venvs/tust-openpi`（Python 3.11）；
     - 外部前端工具通过受控子进程（Subprocess）按需调用，绝不污染核心运行时。
3. **禁止硬编码单机串口与绝对路径**：
   - 所有串口（`COM3` vs `/dev/ttyACM0`）、相机索引与虚拟环境路径全部通过 `.env`（模板 `.env.example`）和 `configs/robots/arm101_{win,ubuntu}.example.yaml` 注入，天然兼容 Windows + Ubuntu 双系统切换。

---

## 4. `ToolAdapter`（插件适配器）与子进程隔离接入机制

为了将上述优秀的开源工具**规范并入**本工作区，同时提供开箱即用的遥操作、示教与数据采集串联体验，我们在本地新建的 **Arm101 Studio**（[`app/`](../../app/)）后端设计了统一的 **`ToolAdapter` 适配器注册表**（源码见 [`app/backend/plugins.py`](../../app/backend/plugins.py)）与 **进程树生命周期管理器**（[`app/backend/process_manager.py`](../../app/backend/process_manager.py)）。

### 4.1 `ToolAdapter` 核心契约

每一个接入的外部开源工具在 `app/backend/plugins.py` 中都实现为标准的 `ToolAdapter` 描述对象，包含：
- **`id` & `name`**：唯一插件标识（如 `lerobot_dataset_viz`、`feetech_servo_bench`、`lerobot_studio`、`lelab_external`、`remote_vla_policy`）；
- **`repo_path`**：指向 `repos/<tool_repo>` 的规范相对路径，自动校验本地仓库是否已通过 `bootstrap_repos.py` 克隆就绪；
- **`os_support` & `available`**：动态声明该工具在当前操作系统（`windows` / `ubuntu`）下的支持状态（例如 `remote_vla_policy` / `openpi` 在 Windows 下会清晰提示需在 Ubuntu 侧或远程 GPU 启动，并返回结构化的 `OS_UNSUPPORTED` 引导，而绝不会崩溃）；
- **`command_builder`**：生成隔离的子进程启动命令列表（无 `shell=True` 注入风险）。

### 4.2 零孤儿进程的受控子进程隔离（`ProcessRegistry`）

当用户在 **Arm101 Studio** 前端界面点击“启动插件”或通过 `POST /api/plugins/{plugin_id}/launch` 调用外部工具时：
1. `ProcessRegistry` 在独立的**进程组 / Windows Job Object（`taskkill /F /T`）或 POSIX Session（`os.killpg`）**中拉起子进程（工作目录指向 `repos/<tool>` 或工作区根目录）；
2. 子进程的标准输出与错误流被实时捕获至 `logs/` 与前端控制台（`log-console`）；
3. 当点击“停止”或关闭 Studio 服务时，`ProcessRegistry.terminate_all()` 会递归清理整个子进程树（含孙进程），确保**零孤儿进程、零串口死锁**。

### 4.3 如何向工作区并入新的开源工具

后续团队成员若希望并入新的开源工具（例如自定义的视觉标定面板或新型 VLA 推理服务），只需三步：
1. 在 [`configs/repos.yaml`](../../configs/repos.yaml) 中登记该仓库的 Git URL 与目标目录 `repos/<new_tool>`，运行 `python scripts/common/bootstrap_repos.py`；
2. 在 [`app/backend/plugins.py`](../../app/backend/plugins.py) 的 `PluginManager` 中新增一条 `ToolAdapter` 定义；
3. 重启 **Arm101 Studio**（`python app/run_studio.py`），前端的 **“插件与开源工具（Plugins）”** 标签页即会自动渲染出该工具的状态卡片与一键启动/停止按钮。
