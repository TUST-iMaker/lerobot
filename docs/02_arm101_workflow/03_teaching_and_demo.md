# `arm101` 演示教学、示教回放与在环策略执行指南

在完成稳定的主从遥操作（[`02_teleoperation.md`](02_teleoperation.md)）后，本篇聚焦 **“如何教得稳、怎么回放验真、如何人机协同纠偏”**，串联起示教规范、`lerobot-replay` 开环回放、`lerobot-rollout`（v0.7.1 新增自主执行与 HIL 示教入口）以及本地软件 **Arm101 Studio** 的示教模式。

---

## 1. 示教轨迹设计与采集前示范要点

高质量示教数据决定了后续 ACT、SmolVLA 与 Pi 系列策略的收敛速度与成功率。操作员在正式录数前应先做 3–5 次无录制演练（Dry Practice），遵守以下准则：

1. **起始归位与任务边界一致**：每条演示开始前，尽量将主从臂归位到约定的舒适预备姿态（Home Pose），目标物摆放在摄像机视野中心的可达区域内，减少无意义的悬空晃动。
2. **动作平滑、避免高频急停**：主臂移动速度保持均匀；抓取与放置阶段适当放慢速度以便视觉编码器捕获接触前后细节。若担心新手操作过快撞击桌面，可在配置文件开启 `max_relative_target: 10.0`（参见 [`configs/robots/arm101_win.example.yaml`](../../configs/robots/arm101_win.example.yaml)）。
3. **视角与光照一致性**：固定桌面相机（`front`）与腕部相机（`wrist`）支架角度，避免录制过程中误碰支架或产生强烈逆光遮挡。
4. **控制单条有效时长**：常见抓放任务控制在 `15–40 秒` 即可；保持每次演示任务语义单一且清晰（配合统一的 `--dataset.single_task` 描述）。

---

## 2. 开环示教回放验证（`lerobot-replay`）

录制或获得示教数据集后，**在投入大规模训练前，务必用 `lerobot-replay` 抽取 1 条回合进行开环回放**，以验证：
- 机械臂标定零位与齿轮比映射是否准确；
- 控制帧率（`fps`）与舵机响应延迟下，从臂能否复现示教轨迹；
- 任务桌面物体位置是否处于可重复执行区间。

```bash
# 从默认本地缓存（或 HF Hub）回放第 0 条 episode（注意：从臂会真实运动，请清空周围障碍！）
lerobot-replay \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --dataset.repo_id=${HF_USER}/arm101_pick_place \
  --dataset.episode=0
```

若数据集存储在本地指定目录（例如未上传 Hub，保存在 `data/datasets/...`），需显式传入 `--dataset.root`：

```bash
lerobot-replay \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --dataset.repo_id=local/arm101_pick_place \
  --dataset.root=data/datasets/arm101_pick_place \
  --dataset.episode=0 \
  --play_sounds=true
```

> **安全提示**：`lerobot-replay` 为**纯开环回放**（按录制时的 `action` 序列与元数据 `fps` 下发目标关节角，不会根据相机画面避障）。执行前务必确认桌面摆放与录制该 episode 时一致，并随时准备断电急停。

---

## 3. 策略自主执行与在环纠偏 / HIL 示教（`lerobot-rollout`）

> **v0.7.1 重要架构变化**：早期 `lerobot <= 0.5.x` 使用 `lerobot-record --policy.path=...` 进行策略真机推理；在当前的 **`v0.7.1`** 中，`lerobot-record` 已禁止传入 `policy`，所有策略自主执行、推理评估与人机在环纠偏（Human-in-the-Loop, HIL / DAgger）均由 **`lerobot-rollout`** 统一负责！

### 3.1 `lerobot-rollout` 五种执行策略（`--strategy.type`）

| `--strategy.type` | 适用场景 | 是否需 `--teleop.*` | 是否保存 `--dataset.*` | 核心作用 |
|---|---|---|---|---|
| `base` | 纯自主策略演示与验收 | 否 | **禁止**传入 | 加载 `--policy.path` 让从臂自主完成任务 |
| `sentry` | 自主执行 + 异常监控录制 | 可选 | 必须 | 记录策略自主运行过程，便于排查失败案例 |
| `highlight` | 关键片段采样 | 可选 | 必须 | 高光/关键阶段标注与数据沉淀 |
| `dagger` | **人机在环纠偏示教（HIL）** | **必须** | **必须** | 策略自主执行时，人类操作员可随时接管主臂（Leader）修正动作并把纠偏轨迹写入新数据集 |
| `episodic` | 分回合交互评估与录制 | 可选 | 必须 | 按回合统计并落盘评估/演示结果 |

同时支持三种推理模式（`--inference.type`）：
- `sync`（默认）：本地同步前向推理；
- `rtc`（Real-Time Chunking）：实时动作分块平滑过渡，降低推理卡顿感（`pi05` 等支持 `rtc_training_max_delay`）；
- `remote`：连接远程推理服务器（如 Ubuntu / 云端 GPU 上的 `lerobot-policy-server`）。

### 3.2 常用运行示例

**1）纯策略自主执行演示（`base`）：**

```bash
lerobot-rollout \
  --strategy.type=base \
  --inference.type=sync \
  --policy.path=models/checkpoints/act_so101/checkpoints/last/pretrained_model \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30} }" \
  --task="Pick up the block and place it in the box" \
  --fps=30 \
  --duration=60 \
  --display_data=true
```

**2）DAgger 人机在环纠偏示教（`dagger`）：**

当策略在边缘位置卡住时，操作员可通过主臂直接接管纠偏并记录到新数据集：

```bash
lerobot-rollout \
  --strategy.type=dagger \
  --inference.type=sync \
  --policy.path=models/checkpoints/smolvla_so101/checkpoints/last/pretrained_model \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30} }" \
  --teleop.type=so101_leader \
  --teleop.port=COM4 \
  --teleop.id=arm101_leader_01 \
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader \
  --dataset.repo_id=${HF_USER}/arm101_hil_dagger \
  --dataset.root=data/datasets/arm101_hil_dagger \
  --dataset.push_to_hub=false \
  --task="Pick up the block and place it in the box" \
  --interactive=true
```

> **重要提醒**：`--robot.cameras` 的相机命名（如 `front`、`wrist`）与分辨率/旋转方向必须与训练模型时使用的数据集一致，否则会因特征键缺失或画面错位导致推理失败。

---

## 4. 使用本地串联软件（Arm101 Studio）进行图形化示教与仿真演练

如果希望在浏览器界面中完成**示教关键帧（Waypoint）记录、点位回放**，或者当前电脑暂未连接真实 `arm101` 硬件（例如课堂教学、算法联调、无硬件演示），可直接启动工作区本地串联软件 **Arm101 Studio**（[`../../app/`](../../app/)）：

```powershell
# 启动方式一（快捷启动器，支持自动检测硬件并在无硬件时降级到 dry_run 模式）
envs\win\lerobot\Scripts\python.exe app\run_studio.py --port 8760

# 或强制以无硬件模拟模式启动：
envs\win\lerobot\Scripts\python.exe app\run_studio.py --dry-run --port 8760

# 启动方式二（直接运行后端模块）
envs\win\lerobot\Scripts\python.exe -m app.backend.main
```

在 **Teaching（示教）** 面板中可完成：
- **Record Waypoint**：记录当前真实机械臂（或 `dry_run` 虚拟 6-DOF 滑块关节角）作为示教路标点；
- **Replay Waypoints**：按顺序平滑回放全部已记录示教点，用于教学演示或快速测试运动路径；
- **Clear**：清空当前路标序列重新示教。

关于本地串联软件的详细界面说明、REST/WebSocket API 与 `dry_run` 产物流向（`data/dry_run/`），请参阅配套文档 [`../04_frontend_and_tools/local_app_guide.md`](../04_frontend_and_tools/local_app_guide.md) *(由 M3 模块提供)*。

下一步：前往 [`04_data_collection.md`](04_data_collection.md) 开始大规模规范化数据集录制、可视化质检与统计量重算。
