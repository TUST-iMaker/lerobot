# Arm101 Studio 全功能 ↔ 完整 CLI 命令对照表（点与线全集）

> 本文以 **「点（数据 / 配置 / 硬件节点）」** 和 **「线（完整可执行命令 → 完整物理/系统动作）」** 列出本系统**每一个功能**对应的完整 CLI 命令，无省略、无占位符歧义。
> 约定：本机 Windows 解释器前缀为 `envs\win\lerobot\Scripts\`；当前默认串口从臂 `COM12`、主臂 `COM11`、相机前置 `0` / 腕部 `1`（按实际 `.env` 替换即可）。

---

## 一、系统「点」（全部核心数据、配置与硬件节点）

| 点位类型 | 节点名称 | 准确路径 / 标识符 | 内容是什么 |
| :--- | :--- | :--- | :--- |
| **硬件点** | 主臂 (Leader) | `COM11` (`so101_leader`, ID: `arm101_leader_01`) | 6 颗 STS3215 舵机（ID 1~6），默认无扭矩，供人手扳动输入关节角 |
| **硬件点** | 从臂 (Follower) | `COM12` (`so101_follower`, ID: `arm101_follower_01`) | 6 颗 STS3215 舵机（ID 1~6），受控上力执行目标位置 |
| **硬件点** | 视觉相机 | `front` (索引 `0`)、`wrist` (索引 `1`) | OpenCV DirectShow 640×480@30fps 图像流 |
| **配置点** | 本地设备映射 | `.env` | 保存 `FOLLOWER_PORT`、`LEADER_PORT`、`FRONT_CAMERA_ID`、`WRIST_CAMERA_ID`、`STUDIO_MODE` |
| **标定点** | 从臂零点与限位 | `data/calibration/robots/so_follower/arm101_follower_01.json` | 6 轴 `id`、`drive_mode`、`homing_offset`、`range_min`、`range_max`（写进舵机 EEPROM） |
| **标定点** | 主臂零点与限位 | `data/calibration/teleoperators/so_leader/arm101_leader_01.json` | 同上，主臂专用 |
| **姿态点** | 中位 / 休眠位 | `software/upright_pose.json`、`software/rest_pose.json` | 6 轴目标角度（`degree` + `gripper: 0~100%`） |
| **运动学点** | 官方 URDF 模型 | `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` | 5-DOF 臂长、关节轴向、限位与末端夹爪坐标系 |
| **数据点** | 真实录制数据集 | `data/datasets/<dataset_name>/` | 官方 `LeRobotDataset v3.0`（`meta/info.json` + `data/*.parquet` + `videos/*.mp4`） |
| **数据点** | 模拟隔离区 | `data/dry_run/` | 模拟标定、模拟航点 `waypoints.json`、模拟连续轨迹 `linear_teach/`（绝不污染真机数据） |
| **训练点** | 训练配置模板 | `configs/train/{act_so101,smolvla_so101_cloud,pi0_so101_cloud,pi05_openpi_cloud}.yaml` | 对应 ACT、SmolVLA、π0、π0.5 的完整超参数模板 |
| **模型点** | 训练权重产物 | `models/checkpoints/<run_name>/pretrained_model` | 训练完成后可直接加载到真机推理的策略权重目录 |

---

## 二、系统「线」（Studio 全功能 → 完整 CLI 命令 → 完整动作）

### 1. 启动与全局安全（顶栏功能）

#### 1.1 启动 Arm101 Studio Web 服务
- **起点 → 终点**：读取 `.env` 与硬件端口 → 启动 `http://127.0.0.1:8760`
- **完整命令**：
```powershell
envs\win\lerobot\Scripts\python.exe app\run_studio.py --host 127.0.0.1 --port 8760 --mode auto
```
- **完整动作**：启动 FastAPI + WebSocket `/ws/telemetry` + 静态前端；`auto` 模式下检测到 ≥1 条机械臂串口自动进 `real`，否则进 `dry_run`。

#### 1.2 顶栏「点击卸力 / 点击上力」（STS3215 寄存器 40 直接控制）
- **起点 → 终点**：串口 `COM12`（从臂，卸力时含 `COM11`）→ 舵机寄存器 `Torque_Enable (addr=40)`
- **完整等效命令（Studio 底层直写）**：
```powershell
# 释放从臂扭矩（可手掰）
envs\win\lerobot\Scripts\python.exe -c "import scservo_sdk as s; p=s.PortHandler('COM12'); pk=s.PacketHandler(0); p.openPort(); p.setBaudRate(1000000); [pk.write1ByteTxRx(p,i,40,0) for i in range(1,7)]; p.closePort()"

# 使能从臂扭矩（锁紧持位）
envs\win\lerobot\Scripts\python.exe -c "import scservo_sdk as s; p=s.PortHandler('COM12'); pk=s.PacketHandler(0); p.openPort(); p.setBaudRate(1000000); [pk.write1ByteTxRx(p,i,40,1) for i in range(1,7)]; p.closePort()"
```
- **完整动作**：向 ID 1~6 舵机写入 `Torque_Enable=0`（全部松弛可手掰）或 `1`（在当前位置立刻锁紧持位，不跳变）。

#### 1.3 顶栏「蜷缩休眠」
- **起点 → 终点**：读取 `software/rest_pose.json` + 标定文件 → 驱动从臂 `COM12`
- **完整等效命令**：
```powershell
envs\win\lerobot\Scripts\python.exe -c "import sys; sys.path.insert(0,'software'); from teleop.official_wrapper import safe_park_to_rest; safe_park_to_rest(port='COM12', duration_s=2.0, robot_id='arm101_follower_01', calibration_dir='data/calibration/robots/so_follower')"
```
- **完整动作**：2 秒内平滑插值将从臂折叠回 `rest_pose.json` 安全休眠姿态，到达后自动释放扭矩，避免摔臂或撞桌面。

#### 1.4 顶栏「E-STOP 急停 / 解除急停」
- **起点 → 终点**：安全状态机 `RobotStateMachine` + `COM12` 舵机
- **完整动作**：触发时立即终止所有运行中的子进程与运动循环，状态机锁死在 `ESTOP_LATCHED` 并释放从臂扭矩，拒绝一切运动指令（HTTP 409）；点击解除后恢复控制权。

---

### 2. Tab 1：总览、硬件探测与零点标定

#### 2.1 探测串口与相机（对应「重新探测硬件」）
- **起点 → 终点**：扫描系统 USB 串口 + OpenCV 摄像头 → 输出设备列表
- **完整官方 CLI 命令**：
```powershell
# 1) 拔插式识别机械臂串口
envs\win\lerobot\Scripts\lerobot-find-port.exe

# 2) 扫描可用摄像头索引与分辨率
envs\win\lerobot\Scripts\lerobot-find-cameras.exe opencv
```
- **完整动作**：识别出主/从臂真实 `COM` 口号与可用相机索引（用于填入 `.env`）。

#### 2.2 零点与行程标定（对应「开始标定」）
- **起点 → 终点**：串口 `COM12` / `COM11` → 生成并写入 `data/calibration/.../<robot_id>.json` + 舵机 EEPROM
- **完整官方 CLI 命令（从臂）**：
```powershell
envs\win\lerobot\Scripts\lerobot-calibrate.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower
```
- **完整官方 CLI 命令（主臂）**：
```powershell
envs\win\lerobot\Scripts\lerobot-calibrate.exe `
  --teleop.type=so101_leader `
  --teleop.port=COM11 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader
```
- **完整动作**：
  1. 连接总线并自动释放 6 颗舵机扭矩（机械臂变松）。
  2. 提示摆到中位（底座居中、大臂竖直向上、小臂与夹爪水平向前平直）→ 按 `Enter` 写入各轴 `homing_offset` 使当前位置为 `0°`（编码器 `2047`）。
  3. 逐个将 6 个关节掰到最小/最大极限 → 按 `Enter` 记录 `range_min` / `range_max` 并保存到 JSON 与舵机 EEPROM。

#### 2.3 新舵机首次写 ID（扩展前置步骤）
- **完整官方 CLI 命令**：
```powershell
envs\win\lerobot\Scripts\lerobot-setup-motors.exe --robot.type=so101_follower --robot.port=COM12
envs\win\lerobot\Scripts\lerobot-setup-motors.exe --teleop.type=so101_leader --teleop.port=COM11
```
- **完整动作**：按提示一次只接 1 颗舵机，依次把 6 颗舵机的总线 ID 写成 `1~6` 并配置 1Mbps 波特率。

---

### 3. Tab 2：遥操作驾驶舱（主从实时联动 + 双目相机）

#### 3.1 主从臂遥操作（对应「开始遥操作」）
- **起点 → 终点**：主臂 `COM11` 实时编码器 + 相机 `0/1` → 从臂 `COM12` 实时跟随
- **完整官方 CLI 命令（无相机纯主从）**：
```powershell
envs\win\lerobot\Scripts\lerobot-teleoperate.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --teleop.type=so101_leader `
  --teleop.port=COM11 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader `
  --fps=60 `
  --display_data=false
```
- **完整官方 CLI 命令（带前置+腕部双相机与 Rerun 可视化）**：
```powershell
envs\win\lerobot\Scripts\lerobot-teleoperate.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --teleop.type=so101_leader `
  --teleop.port=COM11 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader `
  --fps=60 `
  --display_data=true
```
- **完整动作**：以 60Hz 读取主臂 6 轴角度并经标定归一化后实时发送给从臂执行；Studio 内额外接入双向桥接使 3D 画面同步跟随。

---

### 4. Tab 3：6 轴双向仿真、红圈 IK 与键控（Studio 原生功能）

- **说明**：3D 视觉火柴人渲染、红圈平面投影 IK、WASD/QE 键盘步进与独立四视图窗口是 Studio 原生交互层，**不对应单一官方 CLI**；其底层复用官方 URDF（`so101_new_calib.urdf`）与 `software/core/kinematics.py`。
- **完整等效验证命令（正逆运动学自检）**：
```powershell
envs\win\lerobot\Scripts\python.exe -c "import sys; sys.path.insert(0,'software'); from core.kinematics import SO101Kinematics; from core.data_types import JointUnit; k=SO101Kinematics(); print(k.forward_kinematics({'shoulder_pan':0,'shoulder_lift':0,'elbow_flex':0,'wrist_flex':0,'wrist_roll':0}, unit=JointUnit.DEGREE))"
```
- **完整动作**：输入 6 轴角度或末端 `(X, Y, Z)` 坐标，计算精确 URDF 末端位姿并在 `sim_to_real` 开启时经限幅安全门禁下发给从臂 `COM12`。

---

### 5. Tab 4：示教与表演（连续轨迹线性回放 + 关键点示教）

#### 5.1 连续轨迹示教录制（对应「开始连续示教」）
- **起点 → 终点**：主臂 `COM11` + 从臂 `COM12` → `data/datasets/<轨迹名>/`（30Hz 连续帧）
- **完整官方 CLI 命令**：
```powershell
envs\win\lerobot\Scripts\lerobot-record.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --teleop.type=so101_leader `
  --teleop.port=COM11 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader `
  --dataset.repo_id=local/demo_linear_01 `
  --dataset.root=data/datasets/demo_linear_01 `
  --dataset.single_task="linear teaching demo" `
  --dataset.num_episodes=1 `
  --dataset.episode_time_s=30 `
  --dataset.reset_time_s=3 `
  --dataset.fps=30 `
  --dataset.push_to_hub=false `
  --dataset.streaming_encoding=true `
  --play_sounds=false `
  --display_data=false
```
- **完整动作**：主臂手操、从臂实时跟随，以 30Hz 录下整段连续关节轨迹并保存为标准 LeRobot 数据集（第 `0` 段）。

#### 5.2 连续轨迹线性回放（对应「线性回放选中轨迹」）
- **起点 → 终点**：读取 `data/datasets/<轨迹名>/` 指定 episode → 驱动从臂 `COM12`
- **完整官方 CLI 命令**：
```powershell
envs\win\lerobot\Scripts\lerobot-replay.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --dataset.repo_id=local/demo_linear_01 `
  --dataset.root=data/datasets/demo_linear_01 `
  --dataset.episode=0 `
  --dataset.fps=30 `
  --play_sounds=false
```
- **完整动作**：自动脱离主臂，仅控制从臂以 30Hz 逐帧原样线性复现刚才录制的整段连续动作。

#### 5.3 关键帧航点示教与预设姿态（Studio 原生）
- **起点 → 终点**：读取当前关节角写入 `data/dry_run/waypoints.json` → 余弦插值驱动从臂
- **完整动作**：点击「自由示教 (卸力同步)」释放扭矩并开启 `real_to_sim`，手掰到关键位置记录航点；点击「回放轨迹」或预设动作（中位/休眠/观察/悬停/挥手/抓取）平滑插值依次执行。

---

### 6. Tab 5：LeRobotDataset v3.0 数据采集与管理

#### 6.1 多回合带视觉数据采集（对应「开始采集」）
- **起点 → 终点**：主从臂 `COM11`/`COM12` + 双目相机 `0/1` → `data/datasets/<dataset_name>/`
- **完整官方 CLI 命令**：
```powershell
envs\win\lerobot\Scripts\lerobot-record.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --teleop.type=so101_leader `
  --teleop.port=COM11 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader `
  --dataset.repo_id=local/so101_pick_cube `
  --dataset.root=data/datasets/so101_pick_cube `
  --dataset.single_task="Pick up the red cube and place it in the box" `
  --dataset.num_episodes=50 `
  --dataset.episode_time_s=30 `
  --dataset.reset_time_s=10 `
  --dataset.fps=30 `
  --dataset.push_to_hub=false `
  --dataset.streaming_encoding=true `
  --play_sounds=false `
  --display_data=false
```
- **完整动作**：循环录制 50 个回合（每回合最长 30 秒，中间 10 秒复位场景），同步保存 6 轴 `observation.state`、`action` 及双目 `MP4` 视频。若目录已存在，追加 `--resume=true` 续录。

#### 6.2 数据可视化检查与坏回合清洗（官方配套 CLI）
- **完整官方 CLI 命令**：
```powershell
# 可视化查看第 0 回合视频 + 6 轴曲线
envs\win\lerobot\Scripts\lerobot-dataset-viz.exe `
  --repo-id=local/so101_pick_cube `
  --root=data/datasets/so101_pick_cube `
  --episode-index=0

# 删除录坏的第 3、7 回合（输出干净新数据集）
envs\win\lerobot\Scripts\lerobot-edit-dataset.exe `
  --repo_id=local/so101_pick_cube `
  --root=data/datasets/so101_pick_cube `
  --operation.type=delete_episodes `
  --operation.episode_indices="[3, 7]"
```
- **完整动作**：在 Rerun 窗口中逐帧核对画面与关节曲线，剔除失败回合，确保训练集干净。

---

### 7. Tab 6：插件与开源数据

#### 7.1 Feetech MotorLab（舵机底层调试）
- **完整 CLI 命令**：
```powershell
repos\feetech-motorlab\.venv\Scripts\python.exe repos\feetech-motorlab\run.py
```
- **完整动作**：启动舵机 Web 调试台，读写 STS3215 全量 EEPROM/RAM 寄存器、测 PID/电压/温度/负载。

#### 7.2 LeLab（开源 Web 控制面板）
- **完整 CLI 命令**：
```powershell
envs\win\lerobot\Scripts\python.exe -m uvicorn backend.main:app --app-dir repos/leLab --host 127.0.0.1 --port 8000
```
- **完整动作**：启动社区 LeLab Web 服务。

---

### 8. Tab 7：VLA 训练与真机推理部署

#### 8.1 训练策略模型（ACT / SmolVLA / π0）
- **起点 → 终点**：读取 `data/datasets/<dataset_name>/` + `configs/train/*.yaml` → 产出 `models/checkpoints/<run_name>/pretrained_model`
- **完整官方 CLI 命令（ACT，单卡 8~12GB 即可）**：
```powershell
envs\win\lerobot\Scripts\lerobot-train.exe `
  --config_path=configs/train/act_so101.yaml `
  --dataset.repo_id=local/so101_pick_cube `
  --dataset.root=data/datasets/so101_pick_cube `
  --output_dir=models/checkpoints/act_so101_pick_cube `
  --job_name=act_so101_pick_cube `
  --policy.device=cuda
```
- **完整官方 CLI 命令（SmolVLA，16~24GB 显存）**：
```powershell
envs\win\lerobot\Scripts\lerobot-train.exe `
  --config_path=configs/train/smolvla_so101_cloud.yaml `
  --dataset.repo_id=local/so101_pick_cube `
  --dataset.root=data/datasets/so101_pick_cube `
  --output_dir=models/checkpoints/smolvla_so101_pick_cube `
  --job_name=smolvla_so101_pick_cube `
  --policy.device=cuda
```
- **完整官方 CLI 命令（π0，40GB+ 云端 GPU）**：
```powershell
envs\win\lerobot\Scripts\lerobot-train.exe `
  --config_path=configs/train/pi0_so101_cloud.yaml `
  --dataset.repo_id=local/so101_pick_cube `
  --dataset.root=data/datasets/so101_pick_cube `
  --output_dir=models/checkpoints/pi0_so101_pick_cube `
  --job_name=pi0_so101_pick_cube `
  --policy.device=cuda
```
- **完整动作**：加载本地数据集进行模仿学习训练，定期保存 Checkpoint 到 `models/checkpoints/`。

#### 8.2 训练好的模型上真机自主执行（本地推理 / 远程 VLA 推理）
- **起点 → 终点**：加载 `models/checkpoints/<run_name>/pretrained_model` + 实时相机 `0/1` → 自主驱动从臂 `COM12`
- **完整官方 CLI 命令（本机直接推理执行）**：
```powershell
envs\win\lerobot\Scripts\lerobot-record.exe `
  --robot.type=so101_follower `
  --robot.port=COM12 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" `
  --policy.path=models/checkpoints/act_so101_pick_cube/pretrained_model `
  --dataset.repo_id=local/eval_act_so101_pick_cube `
  --dataset.root=data/datasets/eval_act_so101_pick_cube `
  --dataset.single_task="Pick up the red cube and place it in the box" `
  --dataset.num_episodes=5 `
  --dataset.episode_time_s=30 `
  --dataset.push_to_hub=false
```
- **完整官方 CLI 命令（OpenPI 云端策略服务，Ubuntu/Linux）**：
```bash
cd repos/openpi
uv run scripts/serve_policy.py policy:checkpoint --policy.config=pi05_so101 --policy.dir=/path/to/checkpoint
```
- **完整动作**：无需人手碰主臂，模型实时看相机画面与当前关节角，连续输出动作指令自主完成抓取任务，同时把评估过程录进 `eval_*` 数据集用于统计成功率。
