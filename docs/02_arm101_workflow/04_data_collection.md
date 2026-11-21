# `arm101` 数据采集、可视化质检与数据集编辑指南

本篇面向 `repos/lerobot` `v0.7.1` 的 **`LeRobotDataset v3.0`** 格式（Parquet 状态/动作表 + MP4 多视角视频），系统讲解如何用 `lerobot-record` 采集高质量真机数据集、如何避免命名与续采陷阱，以及如何使用 `lerobot-dataset-viz` 与 `lerobot-edit-dataset` 清理坏回合和补齐分位数统计。

> **依赖提醒**：`lerobot-record` 与 `lerobot-dataset-viz` 依赖 `datasets`、`torchcodec`、`pynput` 与 `rerun-sdk`，请确保初始化环境时已加 `-WithCoreScripts`（Windows）或 `--with-core-scripts`（Ubuntu）。

---

## 1. 启动真机数据采集（`lerobot-record`）

### 1.1 纯本地采集推荐命令（不立即上传 HF Hub）

注意 `v0.7.1` 中 **`--dataset.push_to_hub` 默认为 `true`**！如果尚未登录 Hugging Face 或仅想先保存在工作区 `data/datasets/` 下进行质检，请务必显式设置 `--dataset.push_to_hub=false` 与 `--dataset.no_stamp=true`：

```powershell
# Windows 示例
lerobot-record `
  --robot.type=so101_follower `
  --robot.port=COM3 `
  --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: MJPG, backend: 700}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30, fourcc: MJPG, backend: 700} }" `
  --teleop.type=so101_leader `
  --teleop.port=COM4 `
  --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader `
  --dataset.repo_id=tust/arm101_pick_cube `
  --dataset.root=data/datasets/arm101_pick_cube `
  --dataset.num_episodes=50 `
  --dataset.episode_time_s=45 `
  --dataset.reset_time_s=15 `
  --dataset.fps=30 `
  --dataset.single_task="Pick up the red cube and place it into the tray" `
  --dataset.push_to_hub=false `
  --dataset.no_stamp=true `
  --display_data=true
```

*(Ubuntu 下将 `port` 改为 `/dev/ttyACM0` / `/dev/ttyACM1`，相机 `backend` 改为 `200` 或按 V4L2 偶数索引配置。)*

### 1.2 录制过程中的键盘快捷键（源码核验）

根据 `repos/lerobot/src/lerobot/utils/keyboard_input.py`，录制过程中支持以下按键即时控制回合流转（Windows/X11 使用 `pynput` 全局监听，Wayland 或无图形终端自动退回 `TerminalKeyListener`）：

| 按键（任选其一） | 内部事件 | 实际效果 |
|---|---|---|
| `→`（右方向键） 或 `n` | `exit_early = True` | **提前结束当前阶段**：在录制中表示“本条任务已完成，立即保存并进入复位倒计时”；在复位倒计时中表示“已摆好桌面，立即开始下一条” |
| `←`（左方向键） 或 `r` | `rerecord_episode = True` | **放弃并重录本条**：丢弃刚才出错的回合（不计入最终条数），立即重新录制当前序号 |
| `Esc` 或 `q` | `stop_recording = True` | **中止整场采集**：安全结束当前会话并收尾写入数据集索引/统计文件 |

---

## 2. 常见参数与踩坑详解

### 2.1 `--dataset.no_stamp=true` 与自动时间戳
- 默认情况下（`--dataset.no_stamp=false`），`lerobot-record` 会调用 `stamp_repo_id()` 自动在 `repo_id` 末尾追加时间戳后缀 **`_YYYYMMDD_HHMMSS`**（例如 `tust/arm101_pick_cube_20261121_143000`）。
- 许多同学在训练时直接写死原名称导致 `DatasetNotFoundError`。**推荐做法**：要么加上 `--dataset.no_stamp=true` 保持固定目录名；要么记住带时间戳的完整名称。

### 2.2 断点续采（`--resume=true`）
若计划分批采集（如先采 20 条，检查无误后再补采 30 条）：
1. 必须传入 **`--resume=true`** 以及**完全一致的 `--dataset.root` 和 `--dataset.repo_id`**；
2. **`--dataset.num_episodes` 在续采模式下代表“本次额外新增的回合数”**（例如写 `30` 会从第 20 条继续录到第 50 条，而不是只录到 30 条）。

### 2.3 重要勘误：`--dataset.video_backend=pyav` 属于哪个命令？
查阅 `repos/lerobot` v0.7.1 源码（`configs/dataset.py` vs `configs/default.py`）需特别区分：
- **`lerobot-record` 使用的是 `DatasetRecordConfig`**：它**没有** `--dataset.video_backend` 参数！录制侧影响视频编码的性能参数是 `--dataset.streaming_encoding=true`（边录边编码，显著缩短收尾等待时间）、`--dataset.vcodec` 与 `--dataset.encoder_threads`。若强行给 `lerobot-record` 传 `--dataset.video_backend=pyav` 会报未知字段错误。
- **`lerobot-train`（以及所有读取 `LeRobotDataset` 的评估/数据加载脚本）使用的是 `DatasetConfig`**：它**才拥有 `--dataset.video_backend=pyav`**（默认在 `torchcodec` 不可用或 RTX 50 系列 Blackwell 遇到预览版 `torchvision`/`torchcodec` 兼容性问题时，可在**训练命令**里加上 `--dataset.video_backend=pyav`）。

### 2.4 采集后上传 Hugging Face Hub（如先用了 `push_to_hub=false`）
先使用新 CLI 登录并上传：

```bash
hf auth login
hf upload ${HF_USER}/arm101_pick_cube data/datasets/arm101_pick_cube --repo-type dataset
```

---

## 3. 数据集可视化质检（`lerobot-dataset-viz`）

采集完成后，**强烈建议随机抽查首、中、尾若干回合**，确认双视角相机未黑屏、画面未倒置、动作与状态同步。

> **CLI 语法注意**：`lerobot-dataset-viz` 底层使用 `argparse`（**短横线连字符 `--repo-id`**），不同于 `lerobot-record` / `lerobot-train` 的点号写法（`--dataset.repo_id`）。

```bash
# 1. 本地打开 Rerun 窗口查看第 0 条 episode（位于 data/datasets/arm101_pick_cube）
lerobot-dataset-viz \
  --repo-id tust/arm101_pick_cube \
  --root data/datasets/arm101_pick_cube \
  --episode-index 0 \
  --mode local \
  --display-mode rerun

# 2. 若在无显示器的远程服务器上查看，可保存为 .rrd 文件下载到本地 Rerun 打开：
lerobot-dataset-viz \
  --repo-id tust/arm101_pick_cube \
  --root data/datasets/arm101_pick_cube \
  --episode-index 0 \
  --save 1 \
  --output-dir outputs/viz

# 3. 也支持切换为 Foxglove WebSocket 模式：
lerobot-dataset-viz --repo-id tust/arm101_pick_cube --root data/datasets/arm101_pick_cube --episode-index 0 --display-mode foxglove
```

---

## 4. 数据集清洗与统计量重算（`lerobot-edit-dataset`）

如果质检发现少数几条回合抓取失败、手部遮挡镜头，或需要为 `pi05` 补全分位数统计，请使用 `lerobot-edit-dataset`：

### 4.1 删除坏回合（`delete_episodes`）

```bash
# 删除第 0, 2, 5 号坏回合，并将清洗结果保存为新数据集目录（安全非破坏）
lerobot-edit-dataset \
  --repo_id tust/arm101_pick_cube \
  --root data/datasets/arm101_pick_cube \
  --new_repo_id tust/arm101_pick_cube_clean \
  --new_root data/datasets/arm101_pick_cube_clean \
  --operation.type=delete_episodes \
  --operation.episode_indices "[0, 2, 5]"
```

### 4.2 重算 `q01`/`q99` 分位数统计（`recompute_stats`，`pi05` 必备）

`PI05Config` 默认采用 `QUANTILES` 归一化（需要 `meta/stats.json` 中存在 `q01` 和 `q99`）。对于老版本或缺失分位数的数据集，运行：

```bash
# 方式 A：输出到新 repo_id
lerobot-edit-dataset \
  --repo_id tust/arm101_pick_cube_clean \
  --root data/datasets/arm101_pick_cube_clean \
  --new_repo_id tust/arm101_pick_cube_qstats \
  --new_root data/datasets/arm101_pick_cube_qstats \
  --operation.type=recompute_stats

# 方式 B：原地覆盖（必须同时指定相同的 --new_repo_id / --new_root 与 --operation.overwrite=true）
lerobot-edit-dataset \
  --repo_id tust/arm101_pick_cube_clean \
  --root data/datasets/arm101_pick_cube_clean \
  --new_repo_id tust/arm101_pick_cube_clean \
  --new_root data/datasets/arm101_pick_cube_clean \
  --operation.type=recompute_stats \
  --operation.overwrite=true
```

`lerobot-edit-dataset` 还支持 `merge`（多批次数据集拼接）、`split`、`modify_tasks`、`reencode_videos` 等操作，详细参数可通过 `lerobot-edit-dataset --help` 查看。

下一步：进入 [`../03_vla_and_pi/01_vla_data_and_cloud_training.md`](../03_vla_and_pi/01_vla_data_and_cloud_training.md) 学习 **VLA 训练数据准备、本地 8 GB vs 云端 GPU 算力选型与云平台训练全流程**。
