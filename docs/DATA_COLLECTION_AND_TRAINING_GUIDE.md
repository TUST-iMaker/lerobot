# SO-ARM101 数据采集与模型训练实战教程

> 目标：用最短步骤跑通 **「硬件准备 → 采出高质量数据 → 检查清洗 → 训练模型 → 真机自主抓取」** 全流程。

---

## 第 0 步：补齐本机数据与训练依赖（只需做一次）

当前 Windows 环境已装好机械臂驱动，若要运行官方 `lerobot-record`、`lerobot-replay` 和 `lerobot-train`，先补装 `dataset`（若在本机 NVIDIA GPU 训练，确保已装 CUDA 版 PyTorch）：

```powershell
# 补装 datasets / parquet / 视频读写依赖（录制与回放必需）
uv pip install --python envs\win\lerobot\Scripts\python.exe -e "repos/lerobot[dataset]"

# 验证安装成功（无报错即 OK）
envs\win\lerobot\Scripts\lerobot-replay.exe --help
```

---

## 第 1 步：采集前 3 项必查清单（防止白采 50 条）

1. **中位标定必须准确**：
   - 主臂（`COM11`）和从臂（`COM12`）都要在**大臂竖直向上、小臂与夹爪水平向前平直**的姿势完成标定。
   - 验证方法：跑 `lerobot-teleoperate`（或 Studio 遥操作），主臂动哪边从臂就跟哪边，且在 3D 视图中位偏差 `< 5°`。
2. **双目相机画面固定不动**：
   - 前置相机（`front`）能同时看清**整个桌面工作区 + 目标物体 + 目标放置区**；
   - 腕部相机（`wrist`）能清楚看到**夹爪指尖**（抓取对齐靠它）。
   - 采集全程**不要碰歪相机支架**，否则训练出来的模型坐标全偏。
3. **光照与背景稳定**：
   - 避免强反光或频繁开关灯；桌子边缘贴个小定位标记，方便每回合复位物体。

---

## 第 2 步：正式采集高质量数据集（推荐 30~50 回合）

### 方式 A：直接用官方 CLI 采集（最稳、最标准）
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
  --dataset.episode_time_s=25 `
  --dataset.reset_time_s=8 `
  --dataset.fps=30 `
  --dataset.push_to_hub=false `
  --dataset.streaming_encoding=true `
  --play_sounds=false `
  --display_data=false
```

### 方式 B：在 Arm101 Studio 界面点按钮采集
打开 Studio → 切到 **「5. 数据采集」** → 填数据集名（**严禁以 `eval_` 开头**）、任务指令、回合数 → 点 **「开始采集」**（底层调用的就是上面同一条命令）。

### 采集黄金法则（决定模型能不能学会）
- **节奏稳、不犹豫**：动作流畅连贯，不要在半空中反复停顿发抖（模型会学你的发抖）。
- **每回合略微变化物体位置**：在目标区域内随机挪动 3~8 cm、略微转角度，泛化能力会提升一个档次；**不要 50 次全放在完全同一个毫米点**。
- **善用快捷键**：
  - `→`（右方向键）：动作提前做完，立刻结束并保存本回合，进入复位倒计时。
  - `←`（左方向键）：刚才没抓好/撞到了，**直接废弃重录本回合**（宁缺毋滥）。
  - `ESC`：中途随时安全退出并保留已录完的回合（下次加 `--resume=true` 接着录）。
- **采完立刻用 `lerobot-replay` 抽检 1 条**：
  ```powershell
  envs\win\lerobot\Scripts\lerobot-replay.exe `
    --robot.type=so101_follower --robot.port=COM12 --robot.id=arm101_follower_01 `
    --robot.calibration_dir=data/calibration/robots/so_follower `
    --dataset.repo_id=local/so101_pick_cube --dataset.root=data/datasets/so101_pick_cube `
    --dataset.episode=0
  ```
  如果从臂回放第 0 条能丝滑复现刚刚的抓取，说明数据采集 100% 合格！

---

## 第 3 步：检查与清洗数据（可选但强烈推荐）

```powershell
# 1) 回看第 0 条视频与 6 轴曲线（需安装 rerun-sdk）
envs\win\lerobot\Scripts\lerobot-dataset-viz.exe `
  --repo-id=local/so101_pick_cube `
  --root=data/datasets/so101_pick_cube `
  --episode-index=0

# 2) 如果发现第 4、12 条录坏了，直接删掉
envs\win\lerobot\Scripts\lerobot-edit-dataset.exe `
  --repo_id=local/so101_pick_cube `
  --root=data/datasets/so101_pick_cube `
  --operation.type=delete_episodes `
  --operation.episode_indices="[4, 12]"
```

---

## 第 4 步：选择模型并开始训练

### 选哪个模型？（一张表选对）

| 模型 | 显存需求 | 训练耗时（50 条数据） | 适用场景 | 本项目配置模板 |
| :--- | :--- | :--- | :--- | :--- |
| **ACT（强烈推荐新手首发）** | **8 ~ 12 GB**（RTX 3060/4060 即可） | 约 30 ~ 60 分钟 | 单一固定任务（抓方块、放盒、按按钮），最稳、最快出效果 | `configs/train/act_so101.yaml` |
| **SmolVLA** | **16 ~ 24 GB**（RTX 4090 / A10） | 约 1 ~ 2 小时 | 带自然语言指令、多物体轻泛化 | `configs/train/smolvla_so101_cloud.yaml` |
| **π0 / π0.5 (OpenPI)** | **40 ~ 80 GB**（云端 A100 / H100） | 数小时（LoRA 微调） | 多任务、强泛化 VLA 大模型 | `configs/train/pi0_so101_cloud.yaml`<br>`configs/train/pi05_openpi_cloud.yaml` |

### 4.1 本地单卡训练 ACT（最推荐）
```powershell
envs\win\lerobot\Scripts\lerobot-train.exe `
  --config_path=configs/train/act_so101.yaml `
  --dataset.repo_id=local/so101_pick_cube `
  --dataset.root=data/datasets/so101_pick_cube `
  --output_dir=models/checkpoints/act_so101_pick_cube `
  --job_name=act_so101_pick_cube `
  --policy.device=cuda
```
- **怎么看训练好没好**：观察终端日志里的 `loss`（ACT 通常从 `~1.5` 降到 `0.05 ~ 0.15` 以下即开始能抓稳）。
- **权重保存在哪**：`models/checkpoints/act_so101_pick_cube/checkpoints/last/pretrained_model`（或对应步数目录）。

### 4.2 云端服务器训练（AutoDL / 算力平台 / Ubuntu GPU）
1. 把本机 `data/datasets/so101_pick_cube/` 整个文件夹打包上传到云端服务器同路径（或传到 Hugging Face Hub）。
2. 在云端 Linux 上用相同命令训练（把 `envs\win\lerobot\Scripts\lerobot-train.exe` 换成 `lerobot-train`）。
3. 训练完把 `pretrained_model/` 文件夹下载回本机 `models/checkpoints/act_so101_pick_cube/pretrained_model/`。

---

## 第 5 步：模型上真机自主执行与验收

> **安全提醒**：首次上真机测试时，手放在电源开关或 Studio 急停键旁，确保桌面无杂物。

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

### 常见效果不佳快速排查
- **手一直在目标旁边晃、差 1~2 cm 抓不到**：检查腕部相机角度是否被碰歪；多补录 20 条「微调对准抓取」的回合（`--resume=true`）再继续训。
- **一启动就往反方向冲或抖动**：`--robot.calibration_dir` 或 `--robot.id` 与采集时不一致，导致关节零点对不上。
- **动作太慢或太僵硬**：确认推理时 `--robot.cameras` 的分辨率和帧率（`640×480@30fps`）与采集时完全一致。
