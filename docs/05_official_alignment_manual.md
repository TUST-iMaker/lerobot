# SO-101 / LeRobot 官方原仓库溯源、深度比对与对齐修改说明书

> **编写基线**：本说明书基于工作区 `c:\Users\Administrator\Desktop\lerobot`（分支 `yyzmiao`，提交 `f3cae72`）与本地克隆的官方原仓库（`huggingface/lerobot` `v0.7.1` `b13b952`、`TheRobotStudio/SO-ARM100` `141aea7`、`huggingface/leLab` `1894ab5`、`Nori-Robotics/feetech-motorlab` `2d8fe27`、`Physical-Intelligence/openpi` `215abfb`）及 `envs/win/lerobot` 环境源码级核验编写。  
> **硬件安全声明**：本说明书编写与核验全程严格在只读与 `dry_run` 隔离条件下执行，未访问 `COM11` / `COM12` 真实串口，未触发任何电机或物理相机动作。

---

## 目录

- [1. 官方研究报告](#1-官方研究报告)
  - [1.1 官方原仓库完整溯源与本地 `repos/` 校验](#11-官方原仓库完整溯源与本地-repos-校验)
  - [1.2 `Seeed-Projects/lerobot` 隔离核验结论](#12-seeed-projectslerobot-隔离核验结论)
  - [1.3 本地 `envs/win/lerobot` 依赖盘点与 `placo` Windows 可行性](#13-本地-envswinlerobot-依赖盘点与-placo-windows-可行性)
  - [1.4 官方 `huggingface/lerobot` v0.7.1 + `SO-ARM100` 底层权威规范](#14-官方-huggingfacelerobot-v071--so-arm100-底层权威规范)
  - [1.5 辅助参考仓库定位与边界（`leLab` / `feetech-motorlab` / `openpi`）](#15-辅助参考仓库定位与边界lelab--feetech-motorlab--openpi)
- [2. 我们的和官方的对比](#2-我们的和官方的对比)
  - [2.1 整体架构对比：分裂的控制链路](#21-整体架构对比分裂的控制链路)
  - [2.2 底层控制内核与驱动层（`software/`）逐文件深度比对](#22-底层控制内核与驱动层software逐文件深度比对)
  - [2.3 Arm101 Studio（`app/`）与前端仿真逐文件深度比对](#23-arm101-studioapp与前端仿真逐文件深度比对)
  - [2.4 配置、脚本、打包与文档（`configs/`、`scripts/`、`pyproject.toml`、`docs/`）比对](#24-配置脚本打包与文档configsscriptspyprojecttomldocs比对)
- [3. 修改建议（`/plan` 评审方案）](#3-修改建议plan-评审方案)
  - [3.1 架构对齐总原则与硬件安全边界](#31-架构对齐总原则与硬件安全边界)
  - [3.2 里程碑 M1：`software/` 底层控制与驱动层官方对齐](#32-里程碑-m1software-底层控制与驱动层官方对齐)
  - [3.3 里程碑 M2：`app/` Studio 与前端仿真官方对齐](#33-里程碑-m2app-studio-与前端仿真官方对齐)
  - [3.4 里程碑 M3：`pyproject.toml`、`configs/`、`scripts/` 与 `docs/` 对齐](#34-里程碑-m3pyprojecttomlconfigsscripts-与-docs-对齐)
  - [3.5 里程碑 M4：全量 `dry_run` 回归、单元测试与对抗验收计划](#35-里程碑-m4全量-dry_run-回归单元测试与对抗验收计划)
  - [3.6 需用户 `/plan` 确认的关键决策点](#36-需用户-plan-确认的关键决策点)

---

# 1. 官方研究报告

## 1.1 官方原仓库完整溯源与本地 `repos/` 校验

### 1.1.1 工作区主仓库 `TUST-iMaker/lerobot` 真实 Git 溯源

通过 `git remote -v`、`git log --graph --oneline --all` 与 `git show --stat` 核查确认：**当前工作区 `c:\Users\Administrator\Desktop\lerobot` 并非 `huggingface/lerobot` 的 Git Fork**，而是一个独立新建的自定义外包工程仓库（`origin: https://github.com/TUST-iMaker/lerobot.git`，无 `upstream` remote）：

```text
* f3cae72 (HEAD -> yyzmiao, origin/yyzmiao) fix(sim,docs): eliminate IK drag pitch oscillation and align data spec with official LeRobotDataset v3.0 template (2026-11-22 00:52:53 +0800)
* a4ef691 feat(studio,software): V3 fusion with cross-platform software core, all-scene 6-DOF stick-figure sim, dual-arm IK & team data spec (2026-11-22 00:02:47 +0800)
*   de8a828 merge: bring origin/main (aguiya software/) into yyzmiao for V3 fusion
|\
| *   9a74440 (origin/main, origin/HEAD) Merge pull request #4 from TUST-iMaker/aguiya
| |\
| | * 3148bcd (origin/aguiya) Add SO-ARM101 software (2026-11-21 21:11:58 +0800, 63 files, +13385 under software/ + .gitignore)
| |/
* | bf66922 feat(studio): add interactive web calibration, device selector, and DirectShow thread lock
* | 8163b28 feat: initialize TUST-iMaker LeRobot dual-OS workspace, arm101/VLA tutorials, and Arm101 Studio (2026-11-21 22:07:30 +0800, 63 files, +10988)
|/
* 1112f1c (main) Initial commit (2026-11-11 21:42:26 +0800, yyzmiao, README.md 1 insertion)
```

- **根提交**：`1112f1c22e74c131fa2a36cf71adc52e57299f53` 仅为 `README.md` 1 行占位初始化，不继承 `huggingface/lerobot` 任何历史。
- **双线合流结构**：
  - `software/` 由 `aguiya02` 在 `3148bcd` 提交（后合入 `origin/main` `9a74440`），是一套面向单个 Ubuntu 用户开发的独立 SO-ARM101 控制内核、相机采集、遥操作与旧版 Web UI 代码。
  - `app/`、`configs/`、`docs/`、`scripts/`、`tests/` 由 `yyzmiao` 自 `8163b28` 起搭建，并在 `de8a828`、`a4ef691`、`f3cae72` 与 `software/` 进行 V3 融合。
  - **核心定位**：`app/` 与 `software/` 均为本团队在 `TUST-iMaker/lerobot` 中编写的自定义封装层；真正的官方运行时源码位于 `repos/lerobot/`（且 `envs/win/lerobot` 已以 editable 模式安装 `repos/lerobot/src`）。

### 1.1.2 本地 `repos/` 5 个官方克隆与远端一致性校验

通过 `scripts/common/bootstrap_repos.py --status --json` 结合 `git ls-remote origin HEAD` 与 `git ls-remote --tags origin` 逐一核验（截至 `2026-11-21T17:53Z`），`configs/repos.yaml` 声明的 5 个官方上游仓库已全部克隆至 `repos/`，且本地 `HEAD` 与远端官方主分支 `HEAD` **100% 哈希一致**：

| 本地目录 | 官方远端 URL | 本地 `HEAD`（与 `git ls-remote origin HEAD` 完全一致） | 远端最近提交日期与说明 | 克隆深度 / 标签情况 / 备注 |
|---|---|---|---|---|
| `repos/lerobot` | `https://github.com/huggingface/lerobot.git` | `b13b952556c597d1e0849cadb6b6e7496bc5a2f3` | `2026-11-20 18:59:06 +0000` (`feat(processor): add JointReorderProcessorStep (#5446)`) | `depth=1` 浅克隆；`17:53Z` 快照时与远端 `origin HEAD`（`b13b952`）完全一致，此后远端 `origin/main` 于 `18:26Z` 新增 1 个无关提交 `e5be0c16`（`nat_vla` 策略）；`pyproject.toml:28` 声明 `version = "0.7.1"`；远端有 12 个 tags（`v0.3.2`..`v0.7.0`，其中 `v0.7.0` 为 `410c095`，官方尚未给 `0.7.1` 打 tag）；`GIT_LFS_SKIP_SMUDGE=1` 跳过测试二进制资源 |
| `repos/SO-ARM100` | `https://github.com/TheRobotStudio/SO-ARM100.git` | `141aea7bcac2744d92998e30268c413202a56f96` | `2026-11-20 16:10:12 +0100` (`docs: add BrainUs to South Korea kit vendors (#196)`) | `depth=1` 浅克隆；远端最新 tag 为 `v0.1.1`（`3bb5ec8`）；含 `Simulation/SO101/*.urdf`、`joints_properties.xml`、`assets/*.stl` |
| `repos/leLab` | `https://github.com/huggingface/leLab.git` | `1894ab5ba1a73f8ecea4400426c272739494d104` | `2026-11-06 10:08:13 +0100` (`docs: describe the LeRobot v0.7.0 pin...`) | `depth=1` 浅克隆；`0.1.0`，`pyproject.toml:18` 钉选 `lerobot@v0.7.0` |
| `repos/feetech-motorlab` | `https://github.com/Nori-Robotics/feetech-motorlab.git` | `2d8fe277ee9fae8603b2e2f5ee6564b5969ff8b5` | `2026-08-09 11:17:41 -0700` (`Update websockets requirement...`) | `depth=1` 浅克隆；MIT 协议，FastAPI + WebSockets 舵机调试工具 |
| `repos/openpi` | `https://github.com/Physical-Intelligence/openpi.git` | `215abfb217dbac7d5f1273282331b9b1866c0479` | `2026-08-24 16:27:12 +0000` (`docs(droid): fix config search instruction (#1023)`) | `depth=1` 浅克隆；`0.1.0`，`requires-python >= 3.11`，`numpy<2.0.0`，`jax[cuda12]==0.5.3`（Ubuntu/云端专用） |

> **核验结论**：工作区内没有散落的重复 `.git` 仓库；本地 `repos/*` 已是最新官方状态，无需重新克隆。

---

## 1.2 `Seeed-Projects/lerobot` 隔离核验结论

针对 `docs/sources/README.md:17` 引用的 `https://github.com/Seeed-Projects/lerobot.git`，我们在工作区外部临时目录进行了隔离克隆与逐文件比对（避免污染工作区）：

1. **分支状态**：
   - `origin/main`（`d490e472a5d1a207cfe0255caaaf74b6d66e4656`，`2026-11-06` 提交 `"chore(dependencies): Bump lerobot to 0.7.1 (#5269)"`）：
     - `src/lerobot/robots/so_follower/`：与官方 `repos/lerobot`（`b13b952`）**0 个文件差异**。
     - `src/lerobot/teleoperators/so_leader/`：**0 个文件差异**。
     - `src/lerobot/cameras/`：**0 个文件差异**。
     - `src/lerobot/model/`：**0 个文件差异**。
     - 相对于 `repos/lerobot`（`b13b952`, `2026-11-20`），`Seeed-Projects/lerobot` `main`（`d490e47`, `2026-11-06`）滞后 14 天，整个 `src/lerobot/` 目录树共有 `34` 个文件仅存在于 `repos/lerobot`、`4` 个文件仅存在于 `Seeed-Projects/lerobot`、以及 `107` 个已修改文件（涵盖 `policies/praxis/`、`lance_backend.py`、`motors/damiao/damiao.py`、`scripts/lerobot_{env_server,eval}.py` 等），而前述 SO-101 相关核心模块（`so_follower`、`so_leader`、`cameras`、`model`）仍全部保持 `0` 差异。
   - `origin/DepthCameraSupport`（`fe078af31247a6144200abbfa3c2ce29d0062033`，`2026-03-30`）：
     - 为 `rustypot` 迁移之前的老分支（仍保留 `motors/encoding_utils.py`、`motors/feetech/tables.py` 等旧架构），主要新增 `cameras/orbbec/` 与 `cameras/realsense/camera_rs_{d405,d435i}.py`。
2. **源码溯源扫描**：对本地 `app/` 与 `software/` 全部源码检索 `seeed`、`orbbec`、`d405`、`d435`、`isaac_teleop`，命中结果为 **`0` 处**。本地代码不存在任何来自 `Seeed-Projects/lerobot` 的代码。
3. **测试门禁约束**：`tests/e2e/test_tier1_workspace_and_repos.py:130, 158` 严格断言 `configs/repos.yaml` 与 `repos/` 必须**精确等于 5 个官方仓库**（`{"lerobot", "leLab", "openpi", "SO-ARM100", "feetech-motorlab"}`）。因此 `Seeed-Projects/lerobot` 应继续作为外部文档参考隔离，不加入 `repos/`。

---

## 1.3 本地 `envs/win/lerobot` 依赖盘点与 `placo` Windows 可行性

对本地 Windows 虚拟环境 `envs/win/lerobot`（Python `3.12.13`）进行只读依赖盘点与 PyPI wheel 探测结果如下：

1. **已安装核心包**：
   - `lerobot == 0.7.1`（editable 指向 `c:\Users\Administrator\Desktop\lerobot\repos\lerobot\src`）
   - `tust-lerobot-workspace == 0.1.0`（editable 指向工作区根目录 `.`）
   - `rustypot == 1.14.0`、`feetech-servo-sdk == 1.0.0`（`scservo_sdk` 可导入，由根 `pyproject.toml:21` 为 `feetech-motorlab` 引入）
   - `torch == 2.15.0+cpu`、`torchvision == 0.29.2`、`numpy == 2.2.6`、`pyserial == 3.5`、`opencv-python-headless == 4.13.0.92`、`draccus == 0.11.6`、`huggingface-hub == 2.4.0`
   - Studio/工具链：`fastapi == 0.165.0`、`uvicorn == 0.54.0`、`websockets == 17.4`、`pydantic == 2.14.4`、`pyyaml == 6.0.4`、`pytest == 9.2.0`、`playwright == 1.65.2`、`ruff == 0.17.6`、`cmake == 4.1.3`
2. **未安装包**：`placo`、`cmeel-*`、`pin` / `pinocchio`、`rerun-sdk`、`scipy`、`datasets`、`torchcodec`。
3. **`placo` 在 Windows 上的安装可行性（重大发现）**：
   - 执行 `from lerobot.model.kinematics import RobotKinematics` 时不报错，是因为 `repos/lerobot/src/lerobot/model/kinematics.py:30-34, 61-62` 采取**延迟导入**策略，直到调用 `RobotKinematics.__init__()` 时才由 `require_package("placo", extra="placo-dep")` 抛出 `ImportError`。
   - 使用 `uv pip install --dry-run --only-binary :all: "placo>=0.9.6,<0.9.16"` 验证，并直接查询 PyPI JSON API 证实：**`placo`（包括 `0.9.15` 与 `0.12.0`）以及其全套 C++ 绑定依赖（`pin==3.4.0`、`cmeel-urdfdom==4.0.1`、`cmeel-tinyxml2==10.0.0`、`cmeel-boost==1.87.0.1`、`eiquadprog==1.2.9`、`eigenpy==3.10.3`、`coal-library==3.0.1`）在 PyPI 上拥有 `0` 个 Windows（`win_amd64`）二进制 wheel（`win=[]`）**。在 Windows 上强行安装将回落至复杂的 MSVC + Boost + Eigen 源码编译，极易失败。
   - **对比可用二进制包**：`scipy==1.18.1` 与 `rerun-sdk==0.43.2`（注意 `repos/lerobot/pyproject.toml:140` 对 `rerun-sdk` 声明为 `>=0.24.0,<0.34.0`）均提供开箱即用的 `win_amd64` wheel。因此，Windows 端运动学必须在优先委托 `RobotKinematics`（`placo`）的同时，提供直接解析 `so101_new_calib.urdf` 的等价纯 NumPy/SciPy 3D 求解器作为无损备选路径。

---

## 1.4 官方 `huggingface/lerobot` v0.7.1 + `SO-ARM100` 底层权威规范

### 1.4.1 通信总线与 STS3215 控制表（`SerialMotorsBus` + `rustypot 1.14.0`）

1. **总线替换**：`repos/lerobot/pyproject.toml:175-188` 与 `src/lerobot/motors/motors_bus.py:156-252` 确认，官方 `v0.7.1` 已全面采用基于 **`rustypot`**（`rustypot>=1.13.1,<2.0.0`）与 `pyserial` 的 **`SerialMotorsBus`**。旧版基于 `scservo_sdk` 的 `FeetechMotorsBus`（`src/lerobot/motors/feetech/feetech.py:16-59`）已标记 `DeprecationWarning` 并退化为继承 `SerialMotorsBus` 的兼容壳。官方 `lerobot[feetech]` 运行时不依赖 `feetech-servo-sdk`。
2. **总线默认参数**：`default_baudrate = 1_000_000`，`default_timeout = 50` ms（传入 `rustypot` 为 `0.05` s），仅对 `goal_position` 与 `present_position` 执行归一化（寄存器名大小写不敏感）。
3. **STS3215 协议模型参数（`rustypot.find_model('sts3215')`）**：
   - `model_number = 777`（`0x0309`），`resolution = 4096`（12-bit），`factory_baudrate = 1_000_000`，`homing_offset_sign = -1`，`eeprom_lock = True`。
   - `status_alarms_only = True`：电机返回过载报警位（如 `0x20`）时 `refuses(status)` 返回 `False`，不中断常规 `read` / `sync_read` / `write`。
   - `supports_broadcast_ping = True`，`supports_sync_read = True`，`word_order = little`，操作模式：`{'position': 0, 'velocity': 1, 'pwm': 2, 'step': 3}`。
4. **STS3215 关键控制寄存器与 `rustypot` vs `feetech-motorlab` 编码差异**：

| 地址 | 寄存器名（`rustypot`） | 字节/读写 | 官方 LeRobot 设定值 / 作用 | `sign_magnitude` 位宽差异（`rustypot` vs `feetech-motorlab`） |
|---:|---|---|---|---|
| `0` | `firmware_major_version` | 1B / R | `_handshake()` 校验同型号电机固件主版本一致 | `feetech-motorlab` 控制表未覆盖 `0, 1` |
| `1` | `firmware_minor_version` | 1B / R | `_handshake()` 校验固件次版本一致 | 同上 |
| `3` | `model_number` | 2B / R | 握手比对等于 `777` | 一致 |
| `5` | `id` | 1B / RW | `setup_motors()` 逆序设置 ID `1..6` | 一致 |
| `6` | `baud_rate` | 1B / RW | `0` 代表 `1_000_000` baud | 一致 |
| `7` | `return_delay_time` | 1B / RW | `configure_motors()` 写入 `0`（出厂 `250`=500 µs → 0 µs） | 一致 |
| `9` | `min_position_limit` | 2B / RW | 写入校准 `range_min`（EEPROM） | 一致 |
| `11` | `max_position_limit` | 2B / RW | 写入校准 `range_max`（EEPROM） | 一致 |
| `16` | `max_torque_limit` | 2B / RW | `SOFollower.configure()` 仅对 **`gripper`** 设为 `500` | 一致 |
| `18` | `phase` | 1B / RW | `configure_motors()` 对 `sts3215` 清除 bit 4（`phase & ~0x10`），仅当 bit 4 置位时写入 | 一致 |
| `21` | `p_coefficient` | 1B / RW | `SOFollower.configure()` 设为 `16` | 一致 |
| `22` | `d_coefficient` | 1B / RW | `SOFollower.configure()` 设为 `32` | 一致 |
| `23` | `i_coefficient` | 1B / RW | `SOFollower.configure()` 设为 `0` | 一致 |
| `28` | `protection_current` | 2B / RW | `SOFollower.configure()` 仅对 **`gripper`** 设为 `250` | 一致 |
| `31` | `homing_offset` | 2B / RW | 半圈归零偏移，有效范围 `[-2047, 2047]` | **一致**：11-bit `sign_magnitude`（`sign_bit=11`） |
| `33` | `operating_mode` | 1B / RW | `SOFollower`/`SOLeader` 均设为 `0`（position） | 一致 |
| `36` | `overload_torque` | 1B / RW | `SOFollower.configure()` 仅对 **`gripper`** 设为 `25` | 一致 |
| `40` | `torque_enable` | 1B / RW | 开关力矩；`torque_disabled()` 在 `finally` 中自动恢复使能 | 一致 |
| `41` | `acceleration` | 1B / RW | `configure_motors()` 写入 `254` | 一致 |
| `42` | `goal_position` | 2B / RW | 目标步数（多由 `sync_write` 写入） | **`rustypot` 标记 15-bit `sign_magnitude`**；`feetech-motorlab` 未标记有符号 |
| `46` | `goal_velocity` | 2B / RW | 目标速度 | 一致：15-bit `sign_magnitude` |
| `55` | `lock` | 1B / RW | `eeprom_lock=True`：关力矩时解锁 EEPROM，开力矩时上锁 | 一致 |
| `56` | `present_position` | 2B / R | 当前反馈步数（`sync_read` 读取） | **`rustypot` 标记 15-bit `sign_magnitude`**；`feetech-motorlab` 未标记有符号 |
| `58` | `present_velocity` | 2B / R | 当前速度 | 一致：15-bit `sign_magnitude` |
| `60` | `present_load` | 2B / R | 当前负载 | **`rustypot` 标记 10-bit `sign_magnitude`**；`feetech-motorlab` 未标记 |
| `62` | `present_voltage` | 1B / R | 当前电压（`0.1 V` 单位） | 一致 |
| `63` | `present_temperature` | 1B / R | 当前温度（`°C`） | 一致 |
| `65` | `status` | 1B / R | 电机状态/报警位 | 一致 |
| `66` | `moving` | 1B / R | 运动标志 | 一致 |
| `69` | `present_current` | 2B / R | 当前电流 | 一致 |
| `85` | `maximum_acceleration` | 1B / RW | `configure_motors()` 写入 `254` | **`feetech-motorlab` 控制表缺失 `80..86` 扩展寄存器** |

5. **配置与安全上电顺序（`so_follower.py:111-158`, `so_leader.py:85-121`）**：
   - `SOFollower.connect(calibrate=True)`：`bus.connect()` → 若 `not self.is_calibrated and calibrate` 则执行 `self.calibrate()` → 连接相机 → `self.configure()`。
   - `SOFollower.configure()` 在 `with self.bus.torque_disabled():` 内执行 `configure_motors()`、设 `"position"` 模式、写入 PID（`16/0/32`）和 `gripper` 保护参数（`500/250/25`），退出上下文时自动使能力矩并锁定 EEPROM。
   - `SOLeader.configure()` 调用 `disable_torque()`、`configure_motors()`、设 `"position"` 模式并保持力矩关闭以供人工拖动；同时暴露 `enable_torque()` / `disable_torque()` 以支持 DAgger 接管。
   - `setup_motors()`（`motors_bus.py:471-570`）：按**逆序**（`gripper` → `shoulder_pan`）提示用户逐颗单接舵机，遍历波特率表改写 `id` 与 `baud_rate=1_000_000` 并二次 ping 验证。

### 1.4.2 设备注册、校准路径、JSON Schema 与双臂规格

1. **内部名称与注册类型**：
   - `SO101Follower` / `SO100Follower` 均是 `SOFollower` 的别名（`so_follower.py:224-225`），配置类 `SO101FollowerConfig` 注册名为 `"so101_follower"`，但类内部固定属性为 **`name = "so_follower"`**（`so_follower.py:61`）。
   - `SO101Leader` / `SO100Leader` 均是 `SOLeader` 的别名（`so_leader.py:147-148`），配置类 `SO101LeaderConfig` 注册名为 `"so101_leader"`，类内部固定属性为 **`name = "so_leader"`**（`so_leader.py:57`）。
2. **`HF_LEROBOT_CALIBRATION` 路径规则（`robot.py:59-68`, `teleoperator.py:48-59`, `constants.py:87-104`）**：
   - 环境变量优先级：`HF_LEROBOT_HOME` 默认为 `$HF_HOME/lerobot`（若设置了已废弃的 `LEROBOT_HOME` 环境变量，官方直接抛出 `ValueError`）；`HF_LEROBOT_CALIBRATION` 默认为 `$HF_LEROBOT_HOME/calibration`。
   - **路径解析分支**：
     - 若显式传入 `config.calibration_dir`：直接使用 **`config.calibration_dir / f"{self.id}.json"`**（不会再二次拼接 `robots/so_follower`）。
     - 若未传入 `config.calibration_dir`（`None`）：自动回落到 **`HF_LEROBOT_CALIBRATION / "robots" / "so_follower" / f"{self.id}.json"`** 与 **`HF_LEROBOT_CALIBRATION / "teleoperators" / "so_leader" / f"{self.id}.json"`**。
3. **`MotorCalibration` JSON Schema（`motors_bus.py:138-145`）**：
   以电机名为键，通过 `draccus.dump(dict[str, MotorCalibration], f, indent=4)` 序列化：
   ```json
   {
       "shoulder_pan": {
           "id": 1,
           "drive_mode": 0,
           "homing_offset": -123,
           "range_min": 890,
           "range_max": 3210
       }
   }
   ```
   - `is_calibrated`（`motors_bus.py:719-743`）校验电机当前寄存器的 `Homing_Offset`、`Min_Position_Limit`、`Max_Position_Limit` 与已加载的 `self.calibration` 是否一致；对 Feetech 电机刻意忽略 `drive_mode`（无硬件 `Drive_Mode` 寄存器）和 `id`。
4. **半圈归零公式与全圈关节（`motors_bus.py:799-1008`, `so_follower.py:134-144`）**：
   - 第一步：`reset_calibration()` 将 `Homing_Offset=0, Min=0, Max=4095`，人工将机械臂置于中立零位后，读取 raw `Present_Position`（`normalize=False`）。
   - 计算公式：`offset = sign * ((resolution - 1) // 2 - int(position))`。对 STS3215（`resolution = 4096`, `homing_offset_sign = -1`），等价于：
     $$\text{offset} = \text{clamp}\bigl(\text{int}(\text{position}) - 2047,\ -2047,\ +2047\bigr)$$
     （因硬件寄存器 `31` 为 11-bit sign-magnitude，最大幅值 `2047`；当 `position == 4095` 时 `4095 - 2047 = +2048` 会被钳位至 `+2047`，即存在 1 tick 的边缘钳位差，见 `motors_bus.py:820-835, 854-874`）。
   - 第二步：`record_ranges_of_motion(full_turn_motors=["wrist_roll"])` 采集活动范围，`wrist_roll` 跳过手工摇动直接写入 `[0, 4095]`；其余 5 关节若 `mins[m] == maxes[m]` 直接抛出 `ValueError`。
5. **`SOFollower.send_action()` 与 `max_relative_target`（`so_follower.py:187-214`, `robots/utils.py:137-172`）**：
   - 剥离动作键的 `.pos` 后缀；当 `config.max_relative_target` 非 `None`（支持 `float` 或 per-joint `dict`）时，先 `sync_read("Present_Position", num_retry=config.num_read_retries)`，由 `ensure_safe_goal_position()` 将每个关节单步变化量钳位在 `[-max_diff, +max_diff]` 之内，再 `sync_write("Goal_Position")`。
6. **双臂与主从臂齿轮比**：
   - `BiSOFollower`（`"bi_so_follower"`）与 `BiSOLeader`（`"bi_so_leader"`）通过 `BimanualMixin` 封装 `left_arm`（`id="{id}_left"`）和 `right_arm`（`id="{id}_right"`），共享 `calibration_dir` 并在观测/动作键前加 `left_` / `right_` 前缀。
   - 硬件齿轮比（`repos/lerobot/docs/source/so101.mdx:35-45`、`repos/SO-ARM100/README.md:81-93`）：
     - **Follower（从臂，6 颗）**：全部 6 轴均为 **`1/345`**（`C001`）。
     - **Leader（主臂，6 颗）**：`shoulder_pan` (ID 1) **`1/191`**（`C044`）、`shoulder_lift` (ID 2) **`1/345`**（`C001`）、`elbow_flex` (ID 3) **`1/191`**（`C044`）、`wrist_flex` (ID 4) **`1/147`**（`C046`）、`wrist_roll` (ID 5) **`1/147`**（`C046`）、`gripper` (ID 6) **`1/147`**（`C046`）。

### 1.4.3 关节归一化与单位契约

在 `SOFollower.__init__`（`so_follower.py:64-81`）与 `SOLeader.__init__`（`so_leader.py:61-71`）中，6 个电机按截然不同的归一化契约配置：

- **5 个臂关节**（`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`）：当 `use_degrees=True`（默认）时采用 **`MotorNormMode.DEGREES`**；当 `use_degrees=False` 时采用 `MotorNormMode.RANGE_M100_100`。
- **夹爪 `gripper`**：**恒定采用 `MotorNormMode.RANGE_0_100`**（`0.0` = 完全闭合，`100.0` = 完全张开），即使 `use_degrees=True` 也**绝不**转为角度！

`SerialMotorsBus._normalize()` 与 `_unnormalize()`（`motors_bus.py:289-293, 1010-1067`）的精确数学定义如下（设 `min_ = range_min`，`max_ = range_max`，`max_res = 4095`，`mid = (min_ + max_) / 2`）：

1. **`MotorNormMode.DEGREES`（臂关节，不截断、不套用 `drive_mode`）**：
   $$\text{norm}(v_{\text{raw}}) = \frac{(v_{\text{raw}} - \text{mid}) \times 360}{\text{max\_res}}, \qquad v_{\text{raw}}(d) = \operatorname{int}\!\left(\frac{d \times \text{max\_res}}{360} + \text{mid}\right)$$
   - **关键边界**：`_normalize` 对 `DEGREES` **不进行** `[min_, max_]` 截断，也**不应用** `drive_mode` 符号反转；`_unnormalize` 使用 Python **向零截断 `int(...)`** 而非四舍五入（`repos/lerobot/src/lerobot/motors/motors_bus.py:1063`，例如 `2113.75 → 2113` 而非 `2114`）。
2. **`MotorNormMode.RANGE_0_100`（`gripper`，截断且支持软件 `drive_mode` 反转）**：
   - 正向归一化（输入先截断 `bounded = min(max_, max(min_, v_raw))`）：
     $$\text{norm} = \frac{\text{bounded} - \text{min\_}}{\text{max\_} - \text{min\_}} \times 100 \quad \xrightarrow{\text{若 } \text{drive\_mode}=1} \quad 100 - \text{norm}$$
   - 反向解算（输入先截断 `bounded = min(100.0, max(0.0, val))`，若 `drive_mode=1` 则先做 `100 - bounded`，同样使用**向零截断 `int(...)`**，`motors_bus.py:1059`，例如 `2000.8999 → 2000`）：
     $$v_{\text{raw}} = \operatorname{int}\!\left(\frac{\text{bounded}}{100} \times (\text{max\_} - \text{min\_}) + \text{min\_}\right)$$
3. **`MotorNormMode.RANGE_M100_100`**（当 `use_degrees=False` 时的臂关节）：截断于 `[min_, max_]` / `[-100, 100]`，`drive_mode=1` 时取相反数，反向解算同样采用向零截断 `int(...)`（`motors_bus.py:1055`）。

### 1.4.4 运动学与 URDF 机械几何规范（`so101_new_calib.urdf` + `RobotKinematics`）

官方 SO-101 运动学以 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`（302–440 行）为机械几何基准，零位定义对应 `set_half_turn_homings()` 后的半圈中位（上臂竖直、小臂与夹爪水平前伸，见 `so_follower.py:32-51` `_HOMING_POSITION_DIAGRAM`）。

#### A. `so101_new_calib.urdf` 精确连杆变换与机械限位表

| 关节 / 连杆链 | 局部轴 | URDF `origin xyz` (m) | URDF `origin rpy` (rad) | 世界零位关节原点 `(X, Y, Z)` (mm) | 世界零位旋转轴向 | URDF 机械限位 `lower..upper` (rad / deg) |
|---|---|---|---|---|---|---|
| `base_link → shoulder_link` (`shoulder_pan`) | `0 0 1` | `(0.0388353, -8.98e-9, 0.0624)` | `(3.14159, 0, -3.14159)` | `(38.84, 0.00, 62.40)` | **`[0, 0, -1]`**（俯视顺时针为正） | `[-1.91986, +1.91986]`（`±110.00°`） |
| `shoulder_link → upper_arm_link` (`shoulder_lift`) | `0 0 1` | `(-0.0303992, -0.0182778, -0.0542)` | `(-1.5708, -1.5708, 0)` | `(69.23, -18.28, 116.60)` | `[0, +1, 0]` | `[-1.74533, +1.74533]`（`±100.00°`） |
| `upper_arm_link → lower_arm_link` (`elbow_flex`) | `0 0 1` | `(-0.11257, -0.028, 0)` | `(0, 0, 1.5708)` | `(97.23, -18.28, 229.17)` | `[0, +1, 0]` | `[-1.69000, +1.69000]`（`±96.83°`） |
| `lower_arm_link → wrist_link` (`wrist_flex`) | `0 0 1` | `(-0.1349, 0.0052, 0)` | `(0, 0, -1.5708)` | `(232.13, -18.28, 234.37)` | `[0, +1, 0]` | `[-1.65806, +1.65806]`（`±95.00°`） |
| `wrist_link → gripper_link` (`wrist_roll`) | `0 0 1` | `(0, -0.0611, 0.0181)` | `(1.5708, 0.0486795, 3.14159)` | `(293.23, -0.18, 234.37)` | `[-1, 0, 0]`（含 `2.79°` 倾角） | `[-2.74385, +2.84121]`（`-157.21°..+162.79°`） |
| `gripper_link → moving_jaw_so101_v1_link` (`gripper`) | `0 0 1` | `(0.0202, 0.0188, -0.0234)` | `(1.5708, 0, 0)` | — | — | `[-0.174533, +1.74533]`（`-10°..+100°`，但 LeRobot 传 `RANGE_0_100`） |
| `gripper_link → gripper_frame_link` (`fixed` TCP) | `fixed` | `(-0.0079, -0.0002181, -0.0981274)` | `(0, 3.14159, 0)` | **`(391.36, -0.01, 226.47)`** | — | 官方 `RobotKinematics` 默认 `target_frame_name` |

- **连杆几何关键特征**：
  - 上臂直线长度（`shoulder_lift → elbow_flex`）为 $\sqrt{112.57^2 + 28.00^2} = \mathbf{116.00\text{ mm}}$（含 `28.00 mm` 肘部前向偏置与约 `5°` 标定偏移）。
  - 小臂直线长度（`elbow_flex → wrist_flex`）为 $\sqrt{134.90^2 + 5.20^2} = \mathbf{135.00\text{ mm}}$。
  - 腕部至 TCP 距离（`wrist_flex → gripper_frame_link`）为 `(159.23, +18.27, -7.90) mm`（模长 **`160.47 mm`**），其中 `wrist_roll` 引入的 `+18.10 mm` 侧向偏移刚好抵消肩部的 `-18.28 mm` Y 偏置，使零位 TCP 基本回到中心面 `Y = -0.01 mm`。
- **同源兄弟文件**：
  - `so101_leader_new_calib.urdf`：共享前 5 轴几何，末端为 `trigger_link`（`[0, 0.7854]` rad）与 `handle_frame_link`。
  - `so101_old_calib.urdf`：旧版全伸水平零位，**不得**用于 v0.7.1 新校准流程。
  - `joints_properties.xml`：MuJoCo STS3215 动力学参考参数（`damping=0.60`, `frictionloss=0.052`, `armature=0.028`, `kp=17.8`, `forcerange=[-3.35, 3.35]`）。

#### B. `RobotKinematics` 与 `robot_kinematic_processor.py`

- **`RobotKinematics`（`repos/lerobot/src/lerobot/model/kinematics.py:44-208`）**：基于 `placo.RobotWrapper` + `placo.KinematicsSolver`（`mask_fbase(True)`），I/O 关节角度单位为**度**（内部转弧度），默认 `target_frame_name="gripper_frame_link"`，IK 参数 `max_iters=8, position_weight=1.0, orientation_weight=0.01`，且自动将超出 `len(joint_names)` 的末尾元素（如 `gripper`）原样透传保留。同模块的 `GravityCompensation.torques()` 输入为**弧度**，输出为 `Nm`。
- **`robot_kinematic_processor.py`（6 个 `ProcessorStepRegistry` 步骤）**：
  1. `EEReferenceAndDelta`
  2. `EEBoundsAndSafety`（工作空间边界裁剪 + `max_ee_step_m=0.05` m，`raise_on_jump=True`）
  3. `InverseKinematicsEEToJoints`（针对 5-DOF SO-101 欠驱动特征默认 `orientation_weight=0.01` 软姿态权重）
  4. `GripperVelocityToJoint`（显式将夹爪限定在 **`[0.0, 100.0]`** `RANGE_0_100` 空间，离散指令 `{0=close, 1=stay, 2=open}`）
  5. `ForwardKinematicsJointsToEEObservation` / `Action` / `ForwardKinematicsJointsToEE`
  6. `InverseKinematicsRLStep`

### 1.4.5 相机、CLI 工作流与 `LeRobotDataset v3.0`

1. **`OpenCVCamera`（`repos/lerobot/src/lerobot/cameras/opencv/camera_opencv.py:30-637`）**：
   - 在 Windows 上**导入 `cv2` 之前**必须设置 `os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")`，防止 MSMF 硬件转换挂死打开超时。
   - **DSHOW FOURCC 时序要求**：在 `platform.system() == "Windows"` 下，**先设置 `CAP_PROP_FRAME_WIDTH`、`HEIGHT`、`FPS`，最后设置 `CAP_PROP_FOURCC`**（因为 Windows DSHOW 在修改分辨率/帧率时会静默重置 FOURCC），随后强校验读回分辨率和帧率，不匹配即抛 `RuntimeError`。
   - 后台守护线程持续抓取并应用 `ColorMode.RGB` 与 `Cv2Rotation`；`self.frame_lock` **只包裹 `latest_frame` 指针与时间戳交换**，绝不持有锁进行阻塞 I/O。
   - 提供三种读取语义：`read()`（等待下一帧事件，10s 超时）、`async_read(timeout_ms=200)`、`read_latest(max_age_ms=500)`（非阻塞读取最新帧，`SOFollower.get_observation()` 默认使用此接口，若帧年龄超过 `max_age_ms` 抛 `TimeoutError`）。
   - 相机枚举扫描 `/dev/video*`（Linux）或索引 `0..59`（Windows `MAX_OPENCV_INDEX=60`）。
2. **官方 CLI 入口与 `LeRobotDataset v3.0`**：
   - `lerobot-teleoperate`：默认 `fps=60`，支持 `--display_mode={rerun,foxglove}`。
   - `lerobot-record`：默认 `fps=30`，**禁止传入 `policy`**；支持 `--resume=true` 续采、`--dataset.rgb_encoder.vcodec=...`（嵌套在 `rgb_encoder` 下）、`--dataset.streaming_encoding=true`、`--dataset.encoder_threads=...`；禁止 `eval_` 数据集名前缀；输出 `LeRobotDataset` **`codebase_version="v3.0"`**（`meta/info.json`, `meta/stats.json`, `meta/tasks.parquet`, `meta/episodes/chunk-000/file-000.parquet`, `data/chunk-000/file-000.parquet`, `videos/<key>/chunk-000/file-000.mp4`，`observation.state` / `action` 为 `float32[6]`）。
   - `lerobot-replay`：30fps 开环轨迹回放。
   - `lerobot-rollout`：承接旧版策略推理与 HIL（`--strategy.type={base,sentry,highlight,dagger,episodic}`, `--inference.type={sync,rtc,remote}`）。
   - `lerobot-policy-server`（`remote_inference/configs.py:70-127`）：使用 **Zenoh `ServerConfig`**（`--config_path=...` / `--deployment=...` / `--model.repo_or_path=...` / `--semantics=...` / `--zenoh.*`），不存在 `--host`、`--port` 或 `--policy.path` 参数。

---

## 1.5 辅助参考仓库定位与边界（`leLab` / `feetech-motorlab` / `openpi`）

| 仓库 | 版本 / 哈希 | 官方角色与核心特征 | 与我们及 `repos/lerobot` v0.7.1 的差异边界 |
|---|---|---|---|
| **`huggingface/leLab`** | `0.1.0` (`1894ab5`) | HF 官方 Web 调试面板，强依赖 `lerobot@v0.7.0`；FastAPI + React/Vite（需预编译 `frontend/dist`）。 | 1. 标定路径固定 `~/.cache/huggingface/lerobot/calibration/{robots/so_follower,teleoperators/so_leader}`；<br>2. 前端内置 URDF（`repos/leLab/frontend/public/so-101-urdf/urdf/so101_new_calib.urdf`）定义了 `Rotation/Pitch/Elbow/Wrist_Pitch/Wrist_Roll/Jaw` 关节名，而 Python 后端 `repos/leLab/lelab/teleoperate.py:84` 在映射时对 `gripper` `0..100` 误乘 `* math.pi / 180.0`（已由 `repos/leLab/tests/test_teleoperate.py:89` 实证）；<br>3. 仅作隔离插件借鉴参考，不应混装进 `v0.7.1` 主环境。 |
| **`Nori-Robotics/feetech-motorlab`** | `main` (`2d8fe27`) | 独立跨平台 STS3215/3250/3095 舵机 ID 设置与诊断面板（支持 `--simulate`）。 | 1. 使用 `feetech-servo-sdk` (`scservo_sdk`) + `PacketHandler(0)`，而非 `rustypot`；<br>2. `CONTROL_TABLE` 仅覆盖 `3..69`（缺 `0,1` 固件及 `85`）；`SIGNED_REGISTERS` 仅含 `Homing_Offset`/`Goal_Velocity`/`Present_Velocity`；<br>3. 修改接口要求 `x-motorlab-request: 1` 请求头。 |
| **`Physical-Intelligence/openpi`** | `0.1.0` (`215abfb`) | Physical Intelligence 官方 JAX/PyTorch VLA 框架（`pi0` / `pi0-FAST` / `pi0.5`）。 | 1. 仅支持 Ubuntu + NVIDIA GPU，要求 `Python >= 3.11`, `numpy<2.0.0`, `jax[cuda12]==0.5.3`；<br>2. `repos/openpi/src/openpi/training/config.py` 内置配置包含 `aloha`/`droid`/`libero`（`ur5` 仅出现于 `examples/ur5/README.md` 与 `docs/norm_stats.md` 示例文档中），**无任何原生 `so101` 配置**；<br>3. CLI 使用 `tyro`（如 `serve_policy.py --port 8000 policy:checkpoint --policy.config=...`），不接受 `--config_path` YAML。 |

---

# 2. 我们的和官方的对比

## 2.1 整体架构对比：分裂的控制链路

本地工作区存在两套并行演进但未完全打通的体系：
1. **标准 CLI 链路（`app/backend/workflows.py`）**：在 `real` 模式下直接通过 `ProcessRegistry` 拉起官方 `lerobot-calibrate`（经 `calibrate_runner.py`）、`lerobot-teleoperate`、`lerobot-record`，这一条线在**硬件协议、标定路径和核心参数上最接近官方 `v0.7.1`**。
2. **自定义控制与仿真链路（`software/` + `app/backend/realtime_bridge.py` + `app/backend/simulation.py` + `app/frontend/stick_sim.js`）**：自成一套解析运动学（`110/135/125/71 mm`）、关节限位、`gripper` 单位（混用弧度/度/百分比）、硬编码 Linux 标定路径，且 `realtime_bridge.py` 与 `software/core/action_api.py` 存在 **11 处致命或高危接口/逻辑断裂**。

---

## 2.2 底层控制内核与驱动层（`software/`）逐文件深度比对

### 2.2.1 `software/core/kinematics.py` & `safety_controller.py` vs `so101_new_calib.urdf` & `RobotKinematics`

经过对 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`（302–440 行）做精确 SE(3) 链式计算，并对比 `software/core/kinematics.py:31-125`（`L1=0.110, L2=0.135, L3=0.125, L4=0.071`）与 `safety_controller.py:36-43`：

1. **零位（`q = [0, 0, 0, 0, 0]`）TCP 误差高达 `196.24 mm`**：

| 关节 / 段落 | 官方 `so101_new_calib.urdf` 世界零位增量 / 位置 | 我们 `SO101Kinematics` (`kinematics.py`) | 精确几何偏差与原因 |
|---|---|---|---|
| 基座到肩俯仰 (`shoulder_lift`) | `(69.23, -18.28, 116.60) mm` | `(0.00, 0.00, 110.00) mm` | 缺少前向偏置 `+69.23 mm`、侧向偏置 `-18.28 mm`、高度低 `6.60 mm` |
| 上臂 (`shoulder_lift → elbow_flex`) | 直线长 **`116.00 mm`**（含 `+28.00 mm` X 与 `+112.57 mm` Z） | `L2 = 135.00 mm`（纯 Z 向） | **连杆长度被调包且偏长 `+19.00 mm`**，且丢弃 `+28.00 mm` 肘前偏置 |
| 小臂 (`elbow_flex → wrist_flex`) | 直线长 **`135.00 mm`**（含 `+134.90 mm` X 与 `+5.20 mm` Z） | `L3 = 125.00 mm`（纯 X 向） | **偏短 `-10.00 mm`**（`L2/L3` 与真实机械尺寸互换且测错） |
| 腕部到末端 (`wrist_flex → gripper_frame_link`) | `(159.23, +18.27, -7.90) mm`（直线长 **`160.47 mm`**） | `L4 = 71.00 mm`（纯 X 向） | **严重偏短 `-89.47 mm`**，且丢失抵消肩部偏移的 `+18.10 mm` 侧偏与 `2.79°` 倾角 |
| **零位总 TCP 坐标** | **`(+391.36, -0.01, +226.47) mm`** | **`(+196.00, 0.00, +245.00) mm`** | **欧氏误差 `196.24 mm`**（X 轴少 `195.36 mm`，Z 轴高 `18.53 mm`） |

2. **`shoulder_pan` 旋转方向完全相反（`+0.5 rad` 扰动误差 `317.1 mm`）**：
   - URDF 中 `base_link → shoulder_link` 带有 `rpy=(3.14159, 0, -3.14159)`，使得 `shoulder_pan` 世界 Z 轴指向**正下方 `[0, 0, -1]`**，`+0.5 rad` 向 **`-Y` (`-169.0 mm`)** 转动；
   - `SO101Kinematics` 直接套用 `Y = R * sin(+pan)`（`+Z` 向上），`+0.5 rad` 向 **`+Y` (`+94.0 mm`)** 转动，导致实机左转变右转，误差达 **`317.1 mm`**（`software/teleop/keyboard_ee.py:145` 被迫用 `sign_ad = -1.0` 打补丁掩盖）。
   - 其余 3 俯仰轴方向一致，但受连杆长度错配影响，`+0.5 rad` 绝对位置误差分别为 `184.2 mm`（lift）、`193.4 mm`（elbow）、`190.4 mm`（wrist_flex）。
3. **软限位过宽与预设位超限**：

| 关节 | `SafetyController` (`safety_controller.py:36-43`) | 官方 `so101_new_calib.urdf` | 实测风险与姿态文件越界 |
|---|---|---|---|
| `shoulder_pan` | `[-3.14, +3.14]`（`±179.9°`） | `[-1.91986, +1.91986]`（`±110.00°`） | 允许超出机械止挡 **`+69.9°`** |
| `shoulder_lift` | `[-2.80, +2.80]`（`±160.4°`） | `[-1.74533, +1.74533]`（`±100.00°`） | 允许超限 `+60.4°`；且 `software/rest_pose.json` 为 **`-112.18°`**（已越界！） |
| `elbow_flex` | `[-2.80, +2.80]`（`±160.4°`） | `[-1.69000, +1.69000]`（`±96.83°`） | 允许超限 `+63.6°` |
| `wrist_flex` | `[-2.80, +2.80]`（`±160.4°`） | `[-1.65806, +1.65806]`（`±95.00°`） | 允许超限 `+65.4°`；且 `software/rest_pose.json` 为 **`-104.09°`**（已越界！） |
| `wrist_roll` | `[-3.14, +3.14]`（`±179.9°`） | `[-2.74385, +2.84121]`（`-157.21°..+162.79°`） | 宽出约 `17°..22°`（物理标定为全圈 `[0, 4095]`） |
| `gripper` | `[-1.50, +1.50]` rad | LeRobot 运行时 **`[0.0, 100.0]` (`RANGE_0_100`)** | 单位制彻底错误（见下节） |

   - 同时，`software/upright_pose.json` 中 `wrist_flex = -87.38°`、`wrist_roll = +154.15°`，与官方 `_HOMING_POSITION_DIAGRAM`（5 个臂关节全部 `0°`）不一致。

### 2.2.2 全链路 `gripper` 单位灾难性错位（9 个文件追踪）

官方 `SOFollower` / `SOLeader` 强制规定 `gripper` 为 `MotorNormMode.RANGE_0_100`（`0.0`~`100.0`），但本地代码将其当作角度/弧度层层换算，形成完整故障链：

| 环节 | 文件与行号 | 本地错误实现 | 实际造成的危害 |
|---:|---|---|---|
| 1 | `software/hardware/follower_arm.py:255-256` | `use_degrees=True` 时把包含 `gripper` 在内的整字典标为 `JointUnit.DEGREE`，在 `base.py:107-123` 中乘以 `π/180` | `100%` 张开被算成 `1.7453 rad`；写入前又乘 `180/π` |
| 2 | `software/hardware/leader_arm.py:124-125` | 同上，把主臂 `gripper` `0..100%` 当成度数转弧度 | 主从臂状态结构被错误单位污染 |
| 3 | `software/core/safety_controller.py:42, 155-174` | `DEFAULT_JOINT_LIMITS_RAD["gripper"] = (-1.5, 1.5)` rad；`max_joint_delta_rad = 15°` | **任何大于 `85.94%`（`1.5*180/π`）的夹爪张开指令均被静默截断**，且单步被卡死在 `15%` |
| 4 | `software/core/presets.py:54` & `app/backend/workflows.py:1246` | `POSE_PICK_HOVER_RAD["gripper"] = -0.85` 弧度 | 经 `follower_arm.py:290` 转为 `-48.70`，被 `SerialMotorsBus._unnormalize` 截断为 **`0.0`（闭合）** |
| 5 | `software/teleop/demo_routine.py:143, 157` | `gripper = 0.20`（想闭合）与 `gripper = -0.85`（想张开） | **语义完全颠倒**：`"张开"` 指令发出 `0.0%`（完全闭合），`"闭合"` 发出 `11.46%` |
| 6 | `software/save_pose.py:268`、`rest_pose.json:9`、`upright_pose.json:9` | 把从 `get_observation()` 读出的 `RANGE_0_100` 值（`1.08`, `2.52`）保存为 `"unit": "degree"` | `action_api.py:134` 靠 `deg_to_rad` 再 `rad_to_deg` 负负得正（超过 `85.94%` 仍失败） |
| 7 | `software/hardware/follower_arm.py:355`（`move_to_middle`） | 对所有 6 关节统一发送 `0.0` | 回中位时将夹爪硬关至 `0.0%` 闭合（而非保持或中开 `50%`） |
| 8 | `software/teleop/keyboard_ee.py:394`、`web_ui.py:1339,1359`、`coordinate_tool.py:86,203` | 键盘步进 `±0.08 rad` / `deg_to_rad(4.0)`；监控器用 `math.degrees(gripper)` 打印 `°` | 控制步距错乱、面板将百分比显示成虚假度数 |
| 9 | `software/core/data_types.py:150-180` | `JointState.to_radians()` / `to_degrees()` 对全关节一刀切做角度换算 | 模型层缺乏“臂关节弧度/度 + 夹爪 `RANGE_0_100`”的混合单位建模 |

### 2.2.3 校准路径、驱动适配器与独立脚本（`software/hardware/` & `software/*.py`）

1. **`software/hardware/config.py:14-19, 68, 137`**：硬编码 `DEFAULT_FOLLOWER_CALIBRATION = Path("/home/aguiya/.cache/huggingface/lerobot/calibration/robots/so_follower/zihao_follower_arm.json")` 与对应的 `leader` 路径。忽略 `.env` 中的 `HF_LEROBOT_HOME` / `HF_LEROBOT_CALIBRATION`，在 Windows 和任何非 `aguiya` 环境下直接找不到校准文件。
2. **`software/hardware/follower_arm.py:62-92` & `leader_arm.py:46-72`**：
   - 当前 `ArmConfig` 仅有 `calibration_fpath`（尚未定义 `calibration_dir`，见 `software/hardware/config.py:29-40`），且未将 workspace/`calibration_dir` 路径传给 `SO101FollowerConfig` / `SO101LeaderConfig`，导致 `BaseArmAdapter` 与底层 `SO101Follower` 读的**不是同一份校准文件**。
   - `follower_arm.py:72` 绕过 `SOFollower.connect()` 直接调用 `bus.connect()` + `configure()`，导致 **`is_calibrated` 检查完全被跳过**；且 `max_relative_target` 留空为 `None`（若绕开 `ActionAPI` 直接调用驱动则无单步限幅保护）；`leader_arm.py:60-72` 降级用 `SO101Follower` 时跳过 `configure()`。
3. **`software/so_arm101_hardware_init.py`**：
   - `55-57` 行硬编码 `/home/aguiya/.cache/...`，`463` 默认 `--port=/dev/ttyACM0`。
   - `544-553` 行报错文案仍写 `"LeRobot 0.6.x"`，且强制检测 `import scservo_sdk`（在纯净官方 `lerobot[feetech]` `v0.7.1` 环境下会直接报错退出）。
   - `211-222` 行在 `follower.configure()` 之前就写入 `Goal_Position`；`265-305` 行用 `(range_min + range_max)//2` 替代半圈 `2047`、用 `4096/360` 替代 `4095/360` 且逐电机串行写替代 `sync_write`。
4. **`software/save_pose.py`**：
   - `48-61` 行仅扫描 `/dev/ttyACM*` / `/dev/ttyUSB*`（Windows 无法用），硬编码 `Path.home() / ".cache/..."`。
   - `116-122` 行即使传入 `--arm leader` 也**依然实例化 `SO101Follower`** 打开串口。
   - `144-146` 行 `auto_expand_calibration_if_needed()` 在读取完之后修改磁盘 JSON 并紧跟 `pass`，既未刷新内存 `robot.calibration`，也未通过 `write_calibration()` 写入舵机 EEPROM（`Min/Max_Position_Limit`）。
5. **`software/web_ui.py:1319, 1462`**：按下空格或请求 `POST /api/action/estop` 时调用 `_GLOBAL_API.trigger_emergency_stop(...)`，而 `ActionAPI` 仅定义了 `request_estop()`（`action_api.py:354`）——**旧版 Web UI 触发急停必报 `AttributeError` 崩溃**。
6. **`software/main.py:157` & `software/config.yaml:8-45` & `software/setup_udev.sh`**：
   - `main.py:157` `cmd_data()` 硬编码 `Path.home() / ".cache/huggingface/lerobot"`，忽略 `HF_LEROBOT_HOME`。
   - `software/config.yaml:17` 写 `leader.type: "so_leader"`（官方 CLI `--teleop.type` 注册名为 `"so101_leader"`），`44` 行注释误写 `(rerun / opencv)`（官方为 `rerun / foxglove`）。
   - `software/setup_udev.sh` 写入 `99-so-arm101.rules`，与规范脚本 `scripts/ubuntu/setup_ubuntu.sh` 的 `99-lerobot-arms.rules` 冲突。

### 2.2.4 相机、遥操作、系统体检与测试（`software/cameras/`、`teleop/`、`doctor.py`、`tests/`）

1. **`software/cameras/*` vs 官方 `OpenCVCamera`**：
   - **缺失 Windows MSMF/DSHOW 保护**：未在 `import cv2` 前设置 `OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS="0"`；`opencv_camera.py:45-51` 与 `teleop_camera.py:101-103` 未设置 `FOURCC`（更未遵循 Windows 先宽高帧率后 FOURCC 顺序），且不校验 `.set()` 后实际分辨率/帧率。
   - **配置失效**：`opencv_camera.py:64-75` 完全忽略 `config.color_mode` 与 `config.rotation`。
   - **锁竞争与无新鲜度检验**：`BaseCamera._async_capture_loop`（`base.py:215-220`）在持有 `self._lock` 期间阻塞执行 `_capture_frame_raw()` 硬件读图，导致 `get_latest_frame()` 被锁阻塞，且缺乏 `max_age_ms`（官方 500ms）陈旧超时检查。
   - **发现范围过窄**：`discovery.py:49-67` 在 Windows 仅扫索引 `0..3`（官方扫 `0..59`）且硬编码名称与 `MJPG`；`teleop_camera.py:70-93` 仅扫 `/dev/video{i}`。
2. **`software/teleop/*` & `doctor.py`**：
   - `leader_follower.py:17,19` 与 `keyboard_joint.py:28,30` 在模块顶层直接 `import termios, tty`，**在 Windows 上 `import` 即报 `ModuleNotFoundError`**。
   - `official_wrapper.py`：`33, 148, 269, 328` 行拼接不存在的 `<root>/src` 且使用 POSIX `:` 分隔 `PYTHONPATH`（Windows 下应为 `;`，即 `os.pathsep`）；`131, 252` 行当 `fps=None` 时向 `--robot.cameras` 传入 `"fps": null`（官方 `RobotConfig.__post_init__` 必抛 `ValueError`）；未传递 `--robot.calibration_dir` / `--teleop.calibration_dir`；`282` 行误用 `follower_port`（可能为 `None`）而非 `target_follower_port`。
   - `doctor.py:29` 仅扫描 `/dev/ttyACM*` 与 `/dev/ttyUSB*`，Windows 下永远报无串口。
3. **`software/tests/`**：
   - 8 个纯软件测试文件（共 51 个测试）在注入 `PYTHONPATH="software;software/tests"` 时全数通过，但被根 `pyproject.toml` `testpaths` 排除在外；`test_safety.py:81,101` 仍沿用过期测试范围；`test_camera.py:138` 会真实打开 Windows `cv2.VideoCapture(0..3, CAP_DSHOW)`；**完全缺少**对 `SO101FollowerArm`/`LeaderArm` + mock `SerialMotorsBus`、`gripper` `RANGE_0_100` 以及 URDF 运动学偏差的单元测试。

---

## 2.3 Arm101 Studio（`app/`）与前端仿真逐文件深度比对

### 2.3.1 `app/backend/realtime_bridge.py`：11 处接口与契约缺陷清单

此前 Survey 3 发现 7 处致命 API 不匹配，Explorer 3 逐行核对后共确认 **11 处缺陷**（因现有测试仅覆盖 `dry_run`，这 11 处缺陷在开启 `real` 模式同步时将立即爆发）：

| # | `realtime_bridge.py` 位置与代码 | 真实 `software/` 定义 | 报错类型 / 实际后果 |
|---:|---|---|---|
| 1 | `104-112`：`ArmConfig.default_follower(..., calibration_id=...)` / `default_leader` | `hardware/config.py:43-50, 110-117`：参数为 `robot_id=...`, `calibration_fpath=...` | **`TypeError`**：`unexpected keyword argument 'calibration_id'` |
| 2 | `124, 221`：`api.set_torque(False)` / `api.set_torque(enabled)` | `action_api.py:229-235`：`disable_torque()` / `enable_torque()` | **`AttributeError`**：无 `set_torque` 方法 |
| 3 | `129, 191`：`ControlSource.TELEOP` | `data_types.py:63-71`：`KEYBOARD, LEADER, REPLAY, POLICY, UI, MOUSE, SCRIPT` | **`AttributeError`**：无 `TELEOP` 枚举值 |
| 4 | `137`：`api.adapter` | `action_api.py:52, 58`：`self.hardware` (`RobotSystemAdapter`) | **`AttributeError`**：无 `adapter` 属性 |
| 5 | `161, 168`：`obs.follower_joints` / `obs.leader_joints` | `data_types.py:365-368` & `action_api.py:334`：`obs.joint_state`；主臂经 `api.get_leader_action()` | **`AttributeError`**：无 `follower_joints` 属性 |
| 6 | `189-196`：`RobotAction(action_type=..., source=..., target_joints=..., unit=...)` + `getattr(report, "passed", False)` | `data_types.py:287-295` 必填 `timestamp` 且字段名为 `target_joint_positions`；`action_api.py:275` 返回 `tuple[bool, str]` | **`TypeError`**，且对元组取 `.passed` 恒返 `False` |
| 7 | `203`：`api.emergency_stop(reason=reason)` | `action_api.py:354`：`request_estop(reason=...)` | **`AttributeError`**：无 `emergency_stop` 方法 |
| 8 | `104-112`：未向 `ArmConfig` 传入 `StudioConfig.calibration_dir` | `hardware/config.py:14-19` 默认 `/home/aguiya/.cache/...` | 即使修通前 7 项，仍读错路径（与 Studio `data/calibration/` 脱节） |
| 9 | `162-173, 189-194`：`gripper` 单位与 `math.degrees()` 错位 | `simulation.py:45` 为 `0..100%`；`software/` 当前错作弧度 | `sim_to_real` 发送 `100` 度 (`1.745 rad`) 被截断至 `85.94%`；而一旦 `software/` 对齐 `RANGE_0_100`，`real_to_sim` 的 `math.degrees(50.0)` 将把夹爪算成 **`2864.8°`** |
| 10 | `225-234`：`safe_park_to_rest(port=..., duration_s=...)` | `official_wrapper.py:36-71` 内部构造无 `robot_id`、无 `calibration_dir` 的 `default_follower` | 回停靠位同样踩中 `/home/aguiya/...` 硬编码路径而失败 |
| 11 | `149, 176, 233` & `124`：线程锁外写 `last_error` 与 `except: pass` 吞 `set_torque(False)` | `43-50` 行 `status()` 在 `self._lock` 内读状态 | 竞态写脏状态；且 `except: pass` 掩盖了缺陷 #2，可能让 `real_to_sim` 带着力矩运行 |

### 2.3.2 `app/backend/simulation.py` & `app/frontend/stick_sim.js` / `app.js` / `index.html`

1. **几何与旋转方向偏差**：`simulation.py:51-56`（`LINK_LENGTHS_MM`）与 `stick_sim.js:10-15`（`LINK_MM`）完全复制了 `software/core/kinematics.py` 的 `110, 135, 125, 71 mm` 错误尺寸与 `+Z` `shoulder_pan` 符号（零位 TCP `[196, 0, 245] mm` vs URDF `[391.36, -0.01, 226.47] mm`）。此外，`stick_sim.js` 未在 3D 视图中体现 `wrist_roll` 夹爪滚转姿态变化。
2. **后端软限位与前端不一致**：`stick_sim.js:17-24` 与 `app.js:3-10` 已采用贴合 URDF 的限位（`±110, ±100, ±97, ±95, [-157, 163]°`），但 **`simulation.py:39-46` (`JOINT_SPECS`) 将 5 个臂关节放宽至 `[-180, 180]°`**，导致 `set_sim_joints`、`validate_joint_dict`、`solve_ik_target` 可接受远超真实 URDF 机械极限的角度。
3. **编码器换算公式不一致且自相矛盾**：
   - 臂关节：`simulation.py:64-76` 与 `stick_sim.js:26-32` 使用固定中点 `2048` 与 `4096/360`，未对齐官方 `2047` 半圈中点、`4095/360` 比例及校准文件实际的 `(range_min + range_max) / 2`。
   - **夹爪 `gripper` 互不为逆**：`deg_to_encoder` 与 `estimateEncoder` 使用 `round(2048 + (val - 50) * 20)`（即 `0% -> 1048`, `100% -> 3048`），而同文件的 `encoder_to_deg`（`simulation.py:73-74`）却写 `(raw_pos / 4095.0) * 100.0`（把 `1048` 反算成 `25.59%`，把 `3048` 反算成 `74.43%`！），且均忽略校准 JSON 中的 `[range_min, range_max]`。
4. **`dry_run` 磁盘隔离破口**：`app.js:1146-1160` 点击保存零位/休息位后调用 `workflows.py:1376-1387`（`save_custom_pose`），**即使处于 `dry_run` 模式也会直接覆写 `software/upright_pose.json` / `software/rest_pose.json`**。
5. **数据集规范校验 UI（正向对齐项）**：`index.html:398-401` 与 `app.js:673-694` 对 `LeRobotDataset v3.0` 目录结构、禁用 `eval_` 前缀与 `float32[6]` 特征描述准确，与 `repos/lerobot` v0.7.1 一致。

### 2.3.3 `app/backend/workflows.py`、`calibrate_runner.py`、`main.py`、`plugins.py`

1. **`workflows.py` 校准路径（正向对齐项）**：`272-288` 等行显式传递 `--robot.calibration_dir=<calibration_dir>/robots/so_follower` 与 `--teleop.calibration_dir=<calibration_dir>/teleoperators/so_leader`，经核验与官方 `Robot.__init__`/`Teleoperator.__init__` 拼接规则 100% 匹配。
2. **`workflows.py` 其余 4 处缺陷**：
   - **`build_record_cmd` 复采必撞文件锁（`workflows.py:361`）**：对已存在的 `datasets_dir / dataset_name` 目录未自动附加 `--resume=true`，导致官方 `LeRobotDataset.create()`（`lerobot_record.py:478`）直接报 `FileExistsError`；同时未默认开启推荐的 `--dataset.streaming_encoding=true`。
   - **丢弃 UI 传入的 `push_to_hub`（`main.py:464-471`, `workflows.py:367`）**：`CollectStartRequest` 声明了 `push_to_hub: bool`，但路由层未转发，`workflows.py` 硬编码 `--dataset.push_to_hub=false`。
   - **相机频繁开关且遥操/采集中不可见（`workflows.py:1061-1091`）**：`get_camera_jpeg()` 仅在 `active_workflow in ("idle", "calibrate")` 时打开真实相机，且每次 HTTP 请求都重复打开/读取/释放 `cv2.VideoCapture`（缺少热身、MJPG FOURCC 后置设置与持久化缓存）；在 `teleop`/`collect` 期间一律返回虚拟 2D HUD 图。
   - **启动子进程时未安全停靠（`workflows.py:480, 651, 865`）**：调用 `bridge.disconnect(return_to_rest=False, keep_torque=False)` 直接断开并泄力，若之前处于 `sim_to_real` 悬空姿态存在机械臂坠落风险。
3. **`calibrate_runner.py` 2 处进程控制隐患（`calibrate_runner.py:30-37`）**：
   - **EOF 死循环**：当 `stdin` 关闭向队列放入 `None` 后，`_piped_enter_pressed()` 消费 `None` 返回 `False`，导致官方 `record_ranges_of_motion()` 的 `while not user_pressed_enter:` 循环永不退出，只能等待超时强杀。
   - **脏队列提前退出**：`_piped_enter_pressed()` 只要从队列取到任意非 `None` 字符串就返回 `True`；若用户在 Step 1 连点或误输入残留回车，进入 Step 2 第 1 帧即立刻退出并触发 `ValueError("Some motors have the same min and max values")`。
4. **`plugins.py` 2 处适配器问题**：
   - `RemoteVLAPolicyAdapter`（`plugins.py:368-381`）：以 `cwd=self.config.workspace_root` 执行 `uv run .../repos/openpi/scripts/serve_policy.py`（错用工作区根目录的 `uv` 工程而非 `repos/openpi` 或 `~/.venvs/tust-openpi`），且 `--policy.config=pi05_so101_custom` 在官方 `openpi/training/config.py` 中并不存在。
   - `LeLabExternalAdapter`（`plugins.py:293-329`）：依赖预构建的 `repos/leLab/frontend/dist/index.html` 且自身仍读写 `~/.cache/huggingface/lerobot/calibration/`（与 Studio `data/calibration/` 分离）。

---

## 2.4 配置、脚本、打包与文档（`configs/`、`scripts/`、`pyproject.toml`、`docs/`）比对

1. **YAML 配置验证结果**：
   - `configs/robots/arm101_win.example.yaml`、`arm101_ubuntu.example.yaml`：`draccus.parse(TeleoperateConfig, ...)` **100% 通过** `v0.7.1` 校验。
   - `configs/train/act_so101.yaml`、`pi0_so101_cloud.yaml`、`smolvla_so101_cloud.yaml`：字段与 `TrainPipelineConfig`、`ACTConfig`、`PI0Config`、`SmolVLAConfig`（含 `trackio`、`rename_map`）**100% 对齐**。
   - `configs/train/pi05_openpi_cloud.yaml`：Section A 对齐 `openpi` `TrainConfig`，Section B 对齐 `lerobot-train --policy.type=pi05`。
   - `software/config.yaml`：**未对齐**自定义结构（Linux 硬编码端口与错误的 `leader.type: "so_leader"`）。
2. **`pyproject.toml`（42–50 行）**：
   - `[tool.setuptools.packages.find]` 仅 `include = ["app*"]`，未包含 `software*`；`[tool.pytest.ini_options]` `testpaths = ["app/tests", "tests"]`，遗漏 `software/tests`（注意 `test_tier1_workspace_and_repos.py:106` 强制要求 `"tests"` 和 `"app/tests"` 必须在 `testpaths` 中）。
   - `dependencies` 中的 `feetech-servo-sdk>=1.0.0` 仅用于 `feetech-motorlab` 插件，不是 `lerobot` v0.7.1 运行时依赖。
3. **`docs/` 内部 6 处已核实的命令行与说明偏差**：

| # | 文档位置 | 当前文档写法 | 官方 `v0.7.1` / `scripts/` 真实定义 |
|---:|---|---|---|
| 1 | `docs/03_vla_and_pi/01_vla_data_and_cloud_training.md:124,139` & `02_pi_series_and_pi07.md:55` | `bash scripts/ubuntu/setup_ubuntu.sh ... --with-pi --with-smolvla` | `setup_ubuntu.sh:38-54` 不支持 `--with-pi` / `--with-smolvla`，执行报 `Unknown option` 退出码 2 |
| 2 | `docs/03_vla_and_pi/01_vla_data_and_cloud_training.md:266-270` | `lerobot-policy-server --host=0.0.0.0 --port=8080 --policy.path=...` | 官方 `v0.7.1` 使用 Zenoh `ServerConfig`（`--config_path` / `--deployment` / `--model.repo_or_path` / `--zenoh.*`），无 `--host/--port/--policy.path` |
| 3 | `docs/02_arm101_workflow/04_data_collection.md:66` | `--dataset.vcodec` | 实际参数嵌套在 `rgb_encoder` 下：`--dataset.rgb_encoder.vcodec` |
| 4 | `docs/03_vla_and_pi/02_pi_series_and_pi07.md:94` | `-- policy.device=cuda`（`--` 后多了一个空格） | 应为 `--policy.device=cuda` |
| 5 | `docs/04_frontend_and_tools/local_app_guide.md:63` | 5 个臂关节滑块范围为 `[-180, 180]°` | 反映当前 `simulation.py` 未对齐状态，官方 URDF 限位为 `±110, ±100, ±96.8, ±95, -157.2..+162.8°` |
| 6 | `docs/README.md:21,45` | 标注 `local_app_guide.md` 为 `*(M3 配套文档)*` / `*(M3 交付)*` | 该文档已存在，索引标签过期 |

---

# 3. 修改建议（`/plan` 评审方案）

## 3.1 架构对齐总原则与硬件安全边界

1. **底层唯一真源原则**：
   - 所有电机通信、校准存储路径与 JSON 格式、寄存器配置（`Return_Delay_Time=0`、`Maximum_Acceleration=254`、`Acceleration=254`、`Phase` bit 4 清零、`P/I/D=16/0/32`、`gripper` 保护 `500/250/25`）、归一化公式（体关节 `DEGREES`/`RADIAN` + `gripper` **`RANGE_0_100` `[0, 100]`**）及运动学几何/限位（`so101_new_calib.urdf` + `gripper_frame_link`）**必须 100% 与 `repos/lerobot` `v0.7.1` (`b13b952`) 及 `repos/SO-ARM100` (`141aea7`) 保持完全一致**。
2. **双平台与无 `placo` 兼容原则**：
   - 运动学接口优先委托官方 `lerobot.model.kinematics.RobotKinematics`（`placo` 可用时，如 Ubuntu）；在 Windows（PyPI 无 `placo` / `pin` wheel）环境下，提供直接读取 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` XML 参数的**纯 NumPy/SciPy 3D 链式 FK + 阻尼最小二乘 IK 求解器**，实现与 `RobotKinematics` 毫米级一致的零误差等价行为。
3. **绝对硬件隔离原则**：
   - 保留并加固 `MockArmAdapter`、`MockCamera`、`StudioConfig.active_mode == "dry_run"` 与 `data/dry_run/` 磁盘沙箱；任何测试、前端演示或重构验证默认不得开启 `COM11` / `COM12`。

---

## 3.2 里程碑 M1：`software/` 底层控制与驱动层官方对齐

| 目标文件 | 具体修改内容与对齐契约 |
|---|---|
| `software/core/data_types.py` | 1. 明确区分臂关节（`shoulder_pan..wrist_roll`，弧度/度）与 `gripper`（`RANGE_0_100` `[0.0, 100.0]`）；<br>2. 修正 `JointState.to_radians()` / `to_degrees()` / `RobotAction`，转换时保留 `gripper` `[0.0, 100.0]` 百分比数值不做 `π/180` 误换算（或提供显式 `gripper_unit` 契约并在进入 `SafetyController`/`ArmAdapter` 前保持 `0..100`）。 |
| `software/core/safety_controller.py` | 1. 将 `DEFAULT_JOINT_LIMITS_RAD` 的 5 个臂关节严格对齐 `so101_new_calib.urdf`：`shoulder_pan: (-1.91986, 1.91986)`、`shoulder_lift: (-1.74533, 1.74533)`、`elbow_flex: (-1.69, 1.69)`、`wrist_flex: (-1.65806, 1.65806)`、`wrist_roll: (-2.74385, 2.84121)`；<br>2. 将 `gripper` 独立按 `[0.0, 100.0]` 校验与单步限幅（例如 `max_gripper_delta = 15.0` %/step），彻底消除 `85.94%` 截断和负弧度误闭问题。 |
| `software/core/kinematics.py` | 1. 将 `SO101Kinematics` 替换为直接解析 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` 的 3D 运动学引擎（若 `placo` 可导入则可优先委托 `RobotKinematics`，Windows 无 `placo` 时用纯 NumPy 精确 SE(3) 连杆链 + 阻尼最小二乘 IK）；<br>2. 目标末端帧对齐 `gripper_frame_link`，零位 TCP 为 `(391.36, -0.01, 226.47) mm`，`shoulder_pan` 旋转轴严格取 URDF 世界 `[0, 0, -1]`，严格遵守 URDF 关节限位。 |
| `software/core/presets.py` & `software/rest_pose.json` & `software/upright_pose.json` | 1. `presets.py` 中所有 `gripper` 预设值改用 `0.0..100.0`（如闭合 `0.0..5.0`，张开 `60.0..85.0`），移除 `-0.85` 负数；<br>2. 修正 `rest_pose.json` 超限关节（`shoulder_lift` 约束在 `±100°` 内，`wrist_flex` 约束在 `±95°` 内），校准 `upright_pose.json` 对齐官方半圈 `0°` 零位基准，并将夹爪标注/拆分清楚。 |
| `software/core/config_loader.py` & `software/hardware/config.py` | 1. 移除 `/home/aguiya/.cache/...` 硬编码；<br>2. 依据官方 `HF_LEROBOT_CALIBRATION` / `HF_LEROBOT_HOME` / `.env` 与 `configs/robots/` 解析校准目录，将 `calibration_dir`（指向 `.../robots/so_follower` 与 `.../teleoperators/so_leader`）和 `robot_id` 显式写入 `ArmConfig`。 |
| `software/hardware/follower_arm.py` & `leader_arm.py` | 1. 将 `ArmConfig.calibration_dir` 透传给 `SO101FollowerConfig` / `SO101LeaderConfig`，确保适配层与官方总线读取同一份 `<id>.json`；<br>2. 保留 4 阶段无跳变上电保护（先读 `Present_Position` 同步写 `Goal_Position` 再使能），并在连接流程中补齐 **`is_calibrated` 校验**；<br>3. 为 `SO101FollowerConfig` 正确透传可配置的 `max_relative_target`（默认安全单步步长），使其与官方 `ensure_safe_goal_position()` 双重生效；<br>4. 修复 `read_state()` / `send_action()`：`body` 5 轴走 `DEGREE <-> RADIAN`，`gripper` 严格保持 `RANGE_0_100` `[0, 100]` 原值透传；<br>5. 修复 `move_to_middle()`：5 臂关节归 `0.0°`，`gripper` 保持当前位置或设中位 `50.0%`，严禁写 `0.0` 强行闭合。 |
| `software/core/action_api.py` | 同步 `gripper` `[0, 100]` 轨迹插值逻辑（包括 `safe_return_to_rest` 与 `submit_action`），并在需要时通过配置传入 `calibration_dir`。 |
| `software/so_arm101_hardware_init.py` & `save_pose.py` & `web_ui.py` | 1. `so_arm101_hardware_init.py`：移除 `/home/aguiya/...` 与 `scservo_sdk` 检测（改用 `rustypot` / `SerialMotorsBus`），改写初始化顺序为 `configure()` 后对齐当前位置再使能，使用官方 `set_half_turn_homings()` (`2047`) 与 `sync_write`；<br>2. `save_pose.py`：用 `serial.tools.list_ports` 跨平台扫描端口；接入 `HF_LEROBOT_CALIBRATION`；`--arm leader` 改用 `SO101Leader`；扩限后调用 `bus.write_calibration()` 更新内存与舵机 EEPROM；分离 `gripper`（`%`）与臂关节（`deg`）标注；<br>3. `web_ui.py`：修复 `1319, 1462` 行 `trigger_emergency_stop` → `request_estop`，修正键盘 `gripper` 为 `0..100` 步进。 |
| `software/hardware/doctor.py` & `software/cameras/*` & `software/teleop/*` & `software/setup_udev.sh` | 1. `doctor.py`：使用 `serial.tools.list_ports` 支持 Windows `COM*` 与 Linux `/dev/tty*`；<br>2. `software/cameras/`：添加 Windows `OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS="0"`、先宽高帧率后 FOURCC、校验实得分辨率/帧率、应用 `color_mode`/`rotation`、将 `BaseCamera` 锁缩窄至指针交换、增加 `max_age_ms=500` 超时检验、扩大 Windows 索引扫描上限或直接复用官方 `OpenCVCamera`；<br>3. `teleop/`：将 `leader_follower.py` / `keyboard_joint.py` 改为延迟或跨平台 (`msvcrt`/`termios`) 导入；`official_wrapper.py` 修复 `os.pathsep`、移除虚假 `<root>/src`、过滤 `"fps": null`、透传 `--robot.calibration_dir` / `--teleop.calibration_dir`、修复 `target_follower_port`；`keyboard_ee.py` 与 `demo_routine.py` 对齐新 URDF 方向与 `gripper [0, 100]`；<br>4. 统一 `setup_udev.sh` 规则名为 `99-lerobot-arms.rules` 或明确委派给 `scripts/ubuntu/setup_ubuntu.sh`。 |

---

## 3.3 里程碑 M2：`app/` Studio 与前端仿真官方对齐

| 目标文件 | 具体修改内容与对齐契约 |
|---|---|
| `app/backend/realtime_bridge.py` | 1. **修复全部 11 处缺陷**：`calibration_id` → `robot_id` 并显式传入 `calibration_fpath`/`calibration_dir`（指向 `StudioConfig.calibration_dir`）；`set_torque` → `enable_torque`/`disable_torque`（移除吞异常的 `except: pass`）；`ControlSource.TELEOP` → `ControlSource.LEADER` / `ControlSource.UI`；`api.adapter` → `api.hardware`；`obs.follower_joints`/`leader_joints` → `obs.joint_state` 与 `api.get_leader_action()`；构造 `RobotAction` 补全 `timestamp` 与 `target_joint_positions` 并正确解包 `tuple[bool, str]`；`emergency_stop` → `request_estop`；<br>2. **单位对齐**：体关节在 `ActionAPI` 弧度与 Studio 度数间转换，`gripper` 在 `0..100` 空间直通（**严禁**对 `gripper` 调用 `math.degrees`）；<br>3. 修复 `safe_park_to_rest` 透传 `robot_id` 与 `calibration_dir`；将 `last_error` 写入纳入 `self._lock`，同时修改 `disconnect()`（`realtime_bridge.py:52-57`），确保在 `self._lock` **之外**设置 `self._stop_event.set()` 并调用 `self._poll_thread.join(...)`，避免退出阶段轮询线程捕获异常写 `last_error` 时造成最长 1.5s 锁阻塞/死锁等待。 |
| `app/backend/simulation.py` & `app/frontend/stick_sim.js` | 1. 升级 `simulation.py`（`compute_fk_details`、`solve_ik_target`）与 `stick_sim.js`（`computeStickFK`）至 **`so101_new_calib.urdf` 精确 3D 变换链**（零位 TCP `[391.36, -0.01, 226.47] mm`、`shoulder_pan` `-Z` 旋转方向、正确连杆长度与侧向偏置），并在前端绘制 `wrist_roll` 夹爪滚转姿态；<br>2. 将 `simulation.py` `JOINT_SPECS` 从 `±180°` 收紧至 URDF 限位（`±110, ±100, ±96.83, ±95, [-157.21, 162.79]°`, `gripper [0, 100]%`）；<br>3. 修复 `deg_to_encoder` / `encoder_to_deg` / `estimateEncoder`：使用官方 `2047` 半圈中心与 `4095/360` 比例，优先读取当前 `data/calibration/.../<id>.json` 中的 `range_min`/`range_max`（含 `drive_mode`），并让 `gripper` 正反算公式严格互逆。 |
| `app/backend/workflows.py` & `calibrate_runner.py` & `main.py` | 1. `calibrate_runner.py`：修复 `_piped_enter_pressed()` 遇 `None` (EOF) 抛 `EOFError` 退出死循环，并在进入 `record_ranges_of_motion` 前清空历史残留换行，防止零移动崩溃；<br>2. `workflows.py`：`build_record_cmd` 当 `datasets_dir / dataset_name` 已存在时自动附加 `--resume=true`，开启 `--dataset.streaming_encoding=true`，并在 `main.py:464-471` 透传 `req.push_to_hub`；<br>3. `dry_run` 下 `save_custom_pose` 写入 `data/dry_run/` 而非污染 `software/*.json`；<br>4. 在调用 `start_calibrate`/`start_teleop`/`start_collect` 抢占断开 `bridge` 前，若 `sim_to_real` 激活则安全停靠或显式记录提示；优化相机预览（闲时复用心跳缓存或长连接句柄，遵循 Windows DSHOW FOURCC 顺序）。 |
| `app/backend/plugins.py` | 1. `RemoteVLAPolicyAdapter`：使用 `cwd = repos/openpi`（或检查 `~/.venvs/tust-openpi`），并提供可配置的真实 `openpi` policy config 名（不再硬编码不存在的 `pi05_so101_custom`）；<br>2. `LeLabExternalAdapter`：在状态信息与说明中明确提示 `leLab` 依赖独立 `~/.cache` 校准路径与预构建 `frontend/dist`。 |

---

## 3.4 里程碑 M3：`pyproject.toml`、`configs/`、`scripts/` 与 `docs/` 对齐

1. **`pyproject.toml`**：将 `software*` 纳入 `[tool.setuptools.packages.find]`（或保留标准模块化结构），并将 `"software/tests"` 加入 `[tool.pytest.ini_options] testpaths`（必须保留 `"tests"` 与 `"app/tests"` 以满足 `test_tier1_workspace_and_repos.py:106`）；同时需配套配置 `pythonpath = [".", "software", "software/tests"]`（或改用包限定导入 `from software.core...`），并对 `software/tests/test_camera.py:138` 的 `discover_cameras()` 增加硬件隔离门控（mock 或条件跳过），确保直接运行 `pytest` 时不触碰物理相机。将 `feetech-servo-sdk` 标注为插件工具依赖或保留以支持 `feetech-motorlab`。
2. **`configs/robots/*.yaml` & `software/config.yaml`**：统一默认参数契约，清理 `software/config.yaml` 的 Linux 硬编码或改为引用工作区 `.env` / `configs/robots/` 模板。
3. **`scripts/ubuntu/setup_ubuntu.sh` & `docs/*.md`**：
   - 修复 `docs/` 已核实的 6 处偏差：同步 `setup_ubuntu.sh` 与文档中的 `--with-pi` / `--with-smolvla`（建议在 `setup_ubuntu.sh` 中增量支持这两个别名参数或统一文档命令）、修正 `lerobot-policy-server` 为 Zenoh `ServerConfig` 示例、修正 `--dataset.rgb_encoder.vcodec`、修正 `--policy.device=cuda` 空格错误、更新 `local_app_guide.md` 至对齐后的 URDF 关节限位、移除 `docs/README.md` 过期的 `*(M3 配套文档)*` 标记并加入本说明书链接。

---

## 3.5 里程碑 M4：全量 `dry_run` 回归、单元测试与对抗验收计划

所有测试严禁接触真实串口 `COM11` / `COM12`，全部在 `dry_run` 与 Mock 总线隔离下执行：
1. **新增底层官方对齐单元测试（`software/tests/` + `app/tests/`）**：
   - 使用 Mock `SerialMotorsBus` 端到端测试 `SO101FollowerArm` / `SO101LeaderArm`：`gripper` `RANGE_0_100`（含 `0%`, `50%`, `100%` 全开不截断）、体关节 `DEGREES`、自定义 `calibration_dir` 透传、`is_calibrated` 检查与 `max_relative_target` 透传。
   - **URDF 数值比对测试**：直接解析 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`，断言 `SO101Kinematics` 与 `simulation.py` 零位 TCP 坐标等于 `(391.36, -0.01, 226.47) mm`（误差 `< 0.1 mm`）、`shoulder_pan` `+0.5 rad` 向 `-Y` 移动、关节限位严格等于 URDF。
   - **Encoder 互逆测试**：断言 `deg_to_encoder` 与 `encoder_to_deg` 对臂关节与夹爪（含自定义 `range_min/range_max` 与 `drive_mode`）严格互逆，误差 `<= 1` tick。
   - **`SO101RealBridge` Mock 链路测试**：在 `MockArmAdapter` 后端下测试 `realtime_bridge.py` 全部 11 处路径（连接、切主从同步、`sim_to_real`、`real_to_sim`、`set_torque`、`request_estop`、`safe_park_to_rest`），确保 0 个 `AttributeError`/`TypeError`。
   - **`calibrate_runner.py` 边界测试**：模拟 `stdin` EOF 关闭与提前回车脏队列，验证不死循环且不误退。
2. **全量回归套件**：
   - `envs\win\lerobot\Scripts\pytest.exe tests/e2e -m tier1 -v`（`STUDIO_MODE=dry_run`）
   - `envs\win\lerobot\Scripts\pytest.exe app/tests/test_studio_api.py -v`（`STUDIO_MODE=dry_run`）
   - `envs\win\lerobot\Scripts\pytest.exe software/tests -v`（排除真实硬件探测或全量 mock）
   - Playwright 无头浏览器检查前端 `stick_sim.js` 零位渲染与 6 轴限位滑块。

---

## 3.6 需用户 `/plan` 确认的关键决策点

在启动 **M1–M4 代码修改**前，请用户审阅并确认以下 4 项关键决策：

1. **决策点 A — Windows 无 `placo` 下的运动学实现方案**：
   - 由于 PyPI 上 `placo` / `pin` 无 `win_amd64` 二进制 wheel，建议采用：**当 `placo` 可用（如 Ubuntu）时直接委托官方 `RobotKinematics`，在 Windows 环境下自动使用直接解析 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` 的纯 NumPy/SciPy 3D 求解器**（两者零位 TCP 与旋转轴向 100% 几何一致）。是否同意此方案？（如同意，M1 将按需在 `envs/win/lerobot` 安装纯二进制 `scipy` wheel 或采用纯 NumPy 实现）。
2. **决策点 B — `software/` 控制内核的定位**：
   - 建议保留 `software/` 目录并将其**重构对齐为官方 `v0.7.1` 的安全门面与双臂协调层**（修复 `gripper`、`calibration_dir`、URDF 运动学、`cameras`、Windows 兼容与 `realtime_bridge` 契约），同时保留 `app/` 直接调用官方 `lerobot-*` CLI 的标准录制/标定工作流。是否确认保留 `software/` 并双线对齐？
3. **决策点 C — `setup_ubuntu.sh` 与文档参数 `--with-pi` / `--with-smolvla` 的统一方向**：
   - 建议在 `scripts/ubuntu/setup_ubuntu.sh` 中**新增兼容参数 `--with-pi` 与 `--with-smolvla`**（分别映射为安装 `repos/lerobot[pi]` 与 `repos/lerobot[smolvla]`），同时保留原有 `--with-core-scripts`/`--with-training` 参数，使脚本与现有教学文档完全一致。是否同意？
4. **决策点 D — `gripper` 内部统一单位**：
   - 确认全工作区（`software/` 内部模型、`SafetyController`、`presets.py`、`ActionAPI`、`realtime_bridge.py`、`simulation.py`、`stick_sim.js`）**将 `gripper` 统一定义为与官方 `SOFollower` 完全一致的 `0.0 ~ 100.0`（`RANGE_0_100` 百分比，0=闭合，100=全开）**，不再使用弧度或度数表示夹爪开合度。是否确认？


---

# 4. 修改实施与全量验证结果（M1–M4 交付记录）

经用户确认批准第 1–3 部分及全部 4 个决策点（A、B、C、D）后，已按计划完整实施 M1–M4 修改并通过全量 `dry_run` / Mock 自动化验收：

## 4.1 M1：`software/` 底层控制与驱动层官方对齐（已完成）
- **决策点 A（URDF 3D 运动学对齐）**：`software/core/kinematics.py` 已实现直接解析官方 `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` 的纯 NumPy 3D SE(3) 连杆链 + 多初值阻尼最小二乘（DLS）Jacobian 逆运动学（并在 `placo` 可用时支持委托官方 `RobotKinematics`）。零位 TCP 精确对齐 `(391.36147, -0.00921, 226.46971) mm`，`shoulder_pan` 旋转轴严格取 URDF `[0, 0, -1]`。
- **决策点 D（`gripper` 统一 `RANGE_0_100`）**：`data_types.py`、`safety_controller.py`、`presets.py`、`action_api.py`、`follower_arm.py`、`leader_arm.py`、`mock_arm.py`、`rest_pose.json`、`upright_pose.json`、`demo_routine.py`、`keyboard_ee.py`、`web_ui.py` 全链路统一将 5 个臂关节（弧度/度）与 `gripper`（`RANGE_0_100` `[0.0, 100.0]%`）解耦处理，彻底消除 `85.94%` 截断与 `-0.85` 反转 bug。
- **校准路径与驱动加固**：`hardware/config.py` 与 `config_loader.py` 移除 `/home/aguiya/.cache/...` 硬编码，统一按 `HF_LEROBOT_CALIBRATION > HF_LEROBOT_HOME/calibration > workspace/data/calibration` 解析，并将 `calibration_dir` 透传至 `SO101FollowerConfig` / `SO101LeaderConfig`，补齐 `is_calibrated` 检查、`max_relative_target` 透传与 4 阶段无跳变上电保护。
- **跨平台相机与工具脚本**：`software/cameras/` 补齐 Windows MSMF 禁用、FOURCC 顺序、分辨率校验、`color_mode`/`rotation`、`max_age_ms=500` 超时与 `0..59` 摄像头扫描；`so_arm101_hardware_init.py`、`save_pose.py`、`doctor.py`、`official_wrapper.py`、`leader_follower.py`、`keyboard_joint.py` 全部完成 Windows/Linux 跨平台与官方 `v0.7.1` 对齐。

## 4.2 M2：`app/` Studio 与前端火柴棍仿真官方对齐（已完成）
- **`app/backend/realtime_bridge.py`**：修复全部 11 处契约缺陷（`robot_id`/`calibration_dir` 透传、`enable_torque`/`disable_torque`、`ControlSource.UI`、`api.hardware`、`obs.joint_state`、`get_leader_action()`、`RobotAction` `timestamp`+`target_joint_positions`+`tuple[bool, str]` 解包、`request_estop`、`gripper` `0..100%` 直通、`safe_park_to_rest` 参数透传、以及 `disconnect()` 锁外 `join` 防死锁）。
- **`app/backend/simulation.py` & `app/frontend/stick_sim.js`**：升级为 `so101_new_calib.urdf` 精确 3D SE(3) 变换链（零位 TCP `(391.36, -0.01, 226.47) mm`，Python 与 JS 跨 5 组姿态误差 `< 0.1 mm`）；`JOINT_SPECS` 收紧至 URDF 软限位（`±110°, ±100°, ±97°, ±95°, [-157.21°, 162.79°], [0, 100]%`）；`deg_to_encoder` / `encoder_to_deg` / `estimateEncoder` 对齐官方 `2047` 半圈中心、`4095/360` 比例及自定义 `range_min/range_max/drive_mode` 严格互逆。
- **`app/backend/calibrate_runner.py` & `workflows.py` & `plugins.py`**：`_piped_enter_pressed()` 遇 `None` 抛 `EOFError` 并在 Step 1 后排空残留换行；`build_record_cmd` 自动附加 `--resume=true`（目录已存在时）、`--dataset.streaming_encoding=true` 并透传 `push_to_hub`；`dry_run` 下 `save_custom_pose` 隔离写入 `data/dry_run/`；`RemoteVLAPolicyAdapter` 使用 `cwd=repos/openpi` 与真实 `openpi` 配置名。

## 4.3 M3：`pyproject.toml`、`configs/`、`scripts/` 与 `docs/` 对齐（已完成）
- `pyproject.toml`：`include = ["app*", "software*"]`，`testpaths = ["app/tests", "tests", "software/tests"]`，`pythonpath = [".", "software", "software/tests"]`。
- `software/config.yaml`：`leader.type: "so101_leader"`，`display_mode` 注释更新为 `(rerun / foxglove)`。
- **决策点 C**：`scripts/ubuntu/setup_ubuntu.sh` 新增 `--with-pi` 与 `--with-smolvla` 参数支持。
- `docs/`：修复全部 6 处 CLI 参数、Zenoh `ServerConfig`、URDF 关节限位与目录索引偏差。

## 4.4 M4：全量 `dry_run` 与 Mock 自动化测试矩阵（100% 通过）
- `software/tests`：**80 passed**（含 URDF SE(3) 运动学、Mock `SerialMotorsBus` 驱动、安全控制器与跨平台脚本测试）
- `app/tests`：**11 passed**（含 Playwright 全 7 页零控制台错误与 V3/V4 接口测试）
- `tests/e2e`（Tier 1 + Tier 2 官方对齐 + Tier 3 跨模块组合 + Tier 4 真实场景端到端）：**145 passed**
- `scripts/common/check_env.py --strict`：**6/6 PASS**
- 全程在 `dry_run` 与 Mock 总线隔离下完成，**0 次触碰真实 `COM11` / `COM12` 串口**。
