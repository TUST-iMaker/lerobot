# `arm101` 串口探测、舵机 ID 配置与标定指南

本教程面向官方 **arm101（SO-ARM101）主从机械臂**，介绍从串口识别、飞特（Feetech）STS3215 舵机 ID/波特率配置到中位标定的全流程操作（对齐 [`repos/lerobot`](../../repos/lerobot/) `v0.7.1`）。

> **硬件装配前置说明**：本系列教程假设 `arm101` 主臂（Leader）与从臂（Follower）机械结构与舵机连线已完成装配并通电（主臂 `so101_leader` 含混合减速比 STS3215：肩/肘 `1/191`、底座/基座或指定关节 `1/345`、腕爪 `1/147`；从臂 `so101_follower` 统一使用 6 颗 `1/345` STS3215）。如需核对物料清单（BOM）、3D 打印件（STL/STEP）、齿轮比或 URDF 模型，请参阅本地克隆仓库 [`repos/SO-ARM100/`](../../repos/SO-ARM100/)（`README.md` 与 `Simulation/SO101/`）。

---

## 1. 准备工作：激活环境与检查配置文件

在工作区根目录执行：

```powershell
# Windows：激活 Python 3.12 虚拟环境
.\envs\win\lerobot\Scripts\Activate.ps1

# 复制示例配置并准备个人配置（已被 .gitignore 自动忽略）
Copy-Item configs\robots\arm101_win.example.yaml configs\robots\arm101_win.yaml
```

```bash
# Ubuntu：激活 Python 3.12 虚拟环境
source ~/.venvs/tust-lerobot/bin/activate

cp configs/robots/arm101_ubuntu.example.yaml configs/robots/arm101_ubuntu.yaml
```

若工作区根目录下存在 `.env`（复制自 [`.env.example`](../../.env.example)）且设置 `HF_LEROBOT_HOME=./data`，标定文件默认写入 `data/calibration/`；未设置时写入 `~/.cache/huggingface/lerobot/calibration/`。

---

## 2. 第一步：探测主从臂串口号

请仅通过 USB 数据线将 **从臂（Follower）** 与 **主臂（Leader）** 接入电脑，并确认从臂 12V（或 5V/12V 对应规格）电源已开启、Waveshare 转接板跳线置于 USB 模式（B 档）。

### 2.1 Windows（推荐优先使用工作区脚本）

工作区内置 [`scripts/windows/find_ports.ps1`](../../scripts/windows/find_ports.ps1)，可自动识别 `CH340 (1A86:7523)`、`CH343 (1A86:55D3)`、`STM32`、`CP210x`、`FTDI` 等设备：

```powershell
# 1. 快速列出当前在线串口与摄像头
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1

# 2. 交互式拔线差值探测：按提示先保持插线，再拔掉其中一条臂的 USB，精确给出对应 COM 口
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1 -Diff

# 3. 输出 JSON（供脚本或排障使用）
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\windows\find_ports.ps1 -Json
```

### 2.2 Ubuntu / Linux 串口探测与权限

```bash
# 查看当前串口设备（通常为 /dev/ttyACM0, /dev/ttyACM1）
ls -l /dev/ttyACM* /dev/serial/by-id/

# 若未在安装时运行 --setup-udev，可临时赋权或补跑 udev 配置：
bash scripts/ubuntu/setup_ubuntu.sh --setup-udev
# 或临时测试：sudo chmod 666 /dev/ttyACM*
```

### 2.3 使用官方 `lerobot-find-port`

`lerobot` 也提供了交互式端口探测命令：

```bash
lerobot-find-port
```

> **重要警告**：根据 `repos/lerobot/src/lerobot/scripts/lerobot_find_port.py` 源码，`lerobot-find-port` **没有任何 CLI 参数解析器（传入 `--help` 会被忽略）**，运行后会直接打印当前端口并阻塞等待 `input("Remove the USB cable from your MotorsBus and press Enter when done.")`。当且仅当恰好拔掉 1 个设备时才会打印目标端口号，否则会抛出 `OSError`。

将探测结果分别记下，例如：
- Windows：`Follower = COM3`，`Leader = COM4`
- Ubuntu：`Follower = /dev/ttyACM0`，`Leader = /dev/ttyACM1`

并同步填入 `configs/robots/arm101_win.yaml`（或 `arm101_ubuntu.yaml`）与 `.env`。

---

## 3. 第二步：配置飞特 STS3215 舵机 ID 与波特率

> **何时需要本步骤**：若整套 arm101 出厂已写好 1–6 号 ID 且波特率为 `1_000_000`，可直接跳过进入第 4 节；如果是全新舵机或需要重设 ID，请执行本节。

`lerobot-setup-motors` 会按**逆序从末端夹爪（ID 6）到肩部底座（ID 1）**逐颗写入目标 ID 与 `1_000_000` 波特率。**每次只能将控制板连接到提示的那 1 颗舵机上（切勿整条菊花链同时通电配置，否则会冲突）！**

```bash
# 1. 配置从臂（so101_follower），Windows 示例端口 COM3（Ubuntu 替换为 /dev/ttyACM0）
lerobot-setup-motors \
  --robot.type=so101_follower \
  --robot.port=COM3

# 2. 配置主臂（so101_leader），Windows 示例端口 COM4（Ubuntu 替换为 /dev/ttyACM1）
lerobot-setup-motors \
  --teleop.type=so101_leader \
  --teleop.port=COM4
```

按终端提示依次单独接上 `gripper`（6）→ `wrist_roll`（5）→ `wrist_flex`（4）→ `elbow_flex`（3）→ `shoulder_lift`（2）→ `shoulder_pan`（1）并回车确认。

> **可视化替代方案**：如果不习惯命令行单颗插拔，也可通过本地克隆的 [`repos/feetech-motorlab`](../../repos/feetech-motorlab/) 启动网页工作台（支持中位一键置 `2048` 与快速 ID 工站），详见 [`open_source_frontend_survey.md`](../04_frontend_and_tools/open_source_frontend_survey.md)。

---

## 4. 第三步：主从臂零位与行程标定（`lerobot-calibrate`）

### 4.1 关键限制提醒（不要混用遥操作 YAML）

`configs/robots/arm101_win.example.yaml` 与 `arm101_ubuntu.example.yaml` 遵循 `TeleoperateConfig`（同时包含 `robot:` 和 `teleop:` 两个顶层键）。根据 `lerobot_calibrate.py` 源码，**`lerobot-calibrate` 每次只能接收 `--robot.*` 或 `--teleop.*` 其中之一，不能直接传 `arm101_win.yaml`**！请分别对从臂和主臂执行标定命令。

### 4.2 标定从臂（`so101_follower`）与主臂（`so101_leader`）

**Windows 示例（推荐显式指定 `--robot.calibration_dir` 存入工作区 `data/calibration/`）：**

```powershell
# 1. 标定从臂
lerobot-calibrate `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower

# 2. 标定主臂
lerobot-calibrate `
  --teleop.type=so101_leader `
  --teleop.port=COM4 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader
```

**Ubuntu 示例：**

```bash
# 1. 标定从臂
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower

# 2. 标定主臂
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=arm101_leader_01 \
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader
```

### 4.3 交互操作步骤与内部机制

执行后终端会引导以下两步：
1. **中位归零（Middle Pose Homing）**：程序关闭舵机扭矩并切为位置模式，提示将机械臂所有关节手动摆放到**行程中位（直立/半折标准参考姿态）**后按 `Enter`。此时 LeRobot 调用 `set_half_turn_homings()`，将各关节当前物理刻度映射并写入 EEPROM `Homing_Offset`（对应约 `2047` 中位基准）。
2. **全行程扫描（Range of Motion Recording）**：按提示依次将除 `wrist_roll` 外的每个关节平稳掰动至机械限位的最小与最大角度，完成后按 `Enter` 结束。注意源码中 `full_turn_motors=["wrist_roll"]` 会自动写入全圈范围 `[0, 4095]`，其余关节写入实际测得的安全行程边界。
3. **复用或重标定提示**：若该 `--*.id` 已经存在历史标定文件，程序会提示：
   `Press ENTER to use provided calibration file associated with the id ..., or type 'c' and press ENTER to run calibration:`
   输入 `c` 并回车即可覆盖重新标定。

---

## 5. 标定 JSON 存储路径规范（避坑必看）

在 `repos/lerobot` `v0.7.1` 中，`SO101Follower` 继承自 `SOFollower`（`name = "so_follower"`），`SO101Leader` 继承自 `SOLeader`（`name = "so_leader"`）。因此**文件夹名是 `so_follower` 和 `so_leader`，而不是 `so101_follower`**：

| 配置方式 | 从臂标定 JSON 路径 | 主臂标定 JSON 路径 |
|---|---|---|
| **使用本工作区配置**（`calibration_dir` / `.env`） | `data/calibration/robots/so_follower/<id>.json` | `data/calibration/teleoperators/so_leader/<id>.json` |
| **未指定 `calibration_dir` 与 `HF_LEROBOT_HOME`** | `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json` | `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<id>.json` |

> **协作提醒**：只要在后续 `lerobot-teleoperate`、`lerobot-record`、`lerobot-replay` 中保持 `--robot.id`、`--teleop.id` 以及 `calibration_dir` 与标定时完全一致，无需每次重复标定。若重新拆装过舵机齿轮或更换了从臂机器，务必使用新的 `id` 或输入 `c` 重新标定！

下一步：前往 [`02_teleoperation.md`](02_teleoperation.md) 连接摄像头并启动主从臂实时遥操作。
