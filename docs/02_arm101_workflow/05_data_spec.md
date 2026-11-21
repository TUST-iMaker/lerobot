# 5. 官方标准数据采集规范与模板（LeRobotDataset v3.0 / Hugging Face Hub）

本规范**完全对齐 Hugging Face 官方 [`repos/lerobot`](../../repos/lerobot/)（`v0.7.1`）内置的 `LeRobotDataset v3.0` 标准模板**（源码位于 `repos/lerobot/docs/source/lerobot-dataset-v3.mdx`、`src/lerobot/datasets/feature_utils.py` 与 `src/lerobot/datasets/card_template.md`），并同步内置于 **Arm101 Studio → 数据采集** 页面。

---

## 1. 官方 `LeRobotDataset v3.0` 目录结构模板

使用官方 `lerobot-record` 录制后，会自动生成如下 v3.0 分片目录（多个 Episode 合并存放在 Parquet/MP4 分片中，由 `meta/` 索引边界）：

```text
<dataset_root>/
├── README.md                                              # 官方 Dataset Card（来自 card_template.md，含在线可视化徽章）
├── meta/
│   ├── info.json                                          # 核心 Schema（codebase_version="v3.0"、features、fps、分片路径模板）
│   ├── stats.json                                         # 全局归一化统计量（mean/std/min/max/q01/q99）
│   ├── tasks.parquet                                      # 自然语言任务指令 -> task_index 映射表
│   └── episodes/
│       └── chunk-000/file-000.parquet                     # 每个 Episode 的起止偏移、长度与任务索引
├── data/
│   └── chunk-000/file-000.parquet                         # 逐帧表格数据（timestamp、state、action、索引列）
└── videos/
    ├── observation.images.front/chunk-000/file-000.mp4    # 前置全局相机 MP4 分片
    └── observation.images.wrist/chunk-000/file-000.mp4    # 腕部手眼相机 MP4 分片
```

---

## 2. 官方 `meta/info.json` 标准模板（SO-ARM101 6-DOF）

以下字段与 `repos/lerobot/src/lerobot/datasets/feature_utils.py` 的 `create_empty_dataset_info()` 及 `DEFAULT_FEATURES` 严格一致：

```json
{
  "codebase_version": "v3.0",
  "robot_type": "so101_follower",
  "fps": 30,
  "chunks_size": 1000,
  "data_files_size_in_mb": 100,
  "video_files_size_in_mb": 200,
  "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
  "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4",
  "features": {
    "observation.state": {
      "dtype": "float32",
      "shape": [6],
      "names": [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos"
      ]
    },
    "action": {
      "dtype": "float32",
      "shape": [6],
      "names": [
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos"
      ]
    },
    "observation.images.front": {
      "dtype": "video",
      "shape": [480, 640, 3],
      "names": ["height", "width", "channels"]
    },
    "observation.images.wrist": {
      "dtype": "video",
      "shape": [480, 640, 3],
      "names": ["height", "width", "channels"]
    },
    "timestamp":      { "dtype": "float32", "shape": [1], "names": null },
    "frame_index":    { "dtype": "int64",   "shape": [1], "names": null },
    "episode_index":  { "dtype": "int64",   "shape": [1], "names": null },
    "index":          { "dtype": "int64",   "shape": [1], "names": null },
    "task_index":     { "dtype": "int64",   "shape": [1], "names": null }
  }
}
```

### 关键约束（官方强制）
1. **命名规范（`repo_id`）**：格式为 `${HF_USER}/<dataset_name>`（如 `tust_imaker/so101_pick_cube`），仅允许英文/数字/下划线/连字符；**严禁使用 `eval_` 前缀**（官方保留给策略评估数据）。
2. **6 轴单位与顺序**：统一 `use_degrees=true`（角度制），严格按照上述 6 个 `.pos` 字段顺序记录 `observation.state`（从臂状态）与 `action`（主臂指令）。
3. **相机键名**：团队统一使用 `front`（全局视角）与 `wrist`（腕部视角），`640×480 @ 30 FPS`；单相机时固定用 `front`，确保多人数据可直接合并。
4. **任务指令（`single_task`）**：使用简洁英文祈使句（如 `Grab the black cube`），多人协作采集同一任务时保持字符串完全一致。

---

## 3. 官方 `README.md` Dataset Card 模板

上传到 Hugging Face Hub 时，`LeRobotDataset.push_to_hub()` 会自动按 `repos/lerobot/src/lerobot/datasets/card_template.md` 生成数据集卡片，并附带官方在线可视化按钮：

```markdown
---
tags:
  - LeRobot
  - robotics
---

This dataset was created using [LeRobot](https://github.com/huggingface/lerobot).

<a href="https://huggingface.co/spaces/lerobot/visualize_dataset?path=${HF_USER}/so101_pick_cube">
  Visualize this dataset on LeRobot Dataset Visualizer
</a>
```

---

## 4. 官方录制并上传 Hugging Face 云端（Hub）标准命令

### 第一步：登录 Hugging Face 账号（Write Token）
在 [Hugging Face Settings → Access Tokens](https://huggingface.co/settings/tokens) 生成一个 **Write** 权限的 Token，然后在终端登录：

```bash
hf auth login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
hf auth whoami
```

### 第二步：使用官方 `lerobot-record` 录制并自动推送至云端

```bash
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=COM11 \
  --robot.id=arm101_follower_01 \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=COM12 \
  --teleop.id=arm101_leader_01 \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/so101_pick_cube \
  --dataset.num_episodes=50 \
  --dataset.single_task="Pick up the red cube and place it into the tray" \
  --dataset.fps=30 \
  --dataset.push_to_hub=true \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

### 第三步：云端在线可视化与流式训练
- **浏览器在线回放**：打开 `https://huggingface.co/spaces/lerobot/visualize_dataset?path=${HF_USER}/so101_pick_cube`，直接查看双相机视频 + 6 轴状态/动作曲线。
- **训练机免下载流式训练**（AutoDL / Featurize / HF Jobs / Ubuntu）：
  ```bash
  lerobot-train \
    --dataset.repo_id=${HF_USER}/so101_pick_cube \
    --dataset.streaming=true \
    --policy.type=act \
    --output_dir=outputs/train/act_so101
  ```
- **合并多成员数据集 / 删除坏回合**：使用官方 `lerobot-edit-dataset`（合并后自动重算 `meta/stats.json`，含 `pi0.5` 所需的 `q01/q99` 分位数）。
