# VLA 训练数据准备与云端租用算力训练全流程指南

本指南面向 **官方 `arm101`（SO-ARM101）主从臂**采集完成的 `LeRobotDataset v3.0` 数据集，详细说明如何进行 **VLA（Vision-Language-Action）训练前的数据规范化处理**、如何根据显存需求选择合适的 **云端算力平台（AutoDL、Featurize、Hugging Face Jobs）**，以及如何使用 `lerobot-train`、多卡 `accelerate` / `torchrun` / `FSDP` 训练并将 `models/checkpoints` 权重部署回本地进行 `lerobot-eval` / `lerobot-rollout` 验证。

---

## 1. VLA 训练前的数据集准备与规范化检查

在将本地采集的数据集送入 `act`、`smolvla`、`pi0` 或 `pi05` 训练之前，必须确保数据集目录结构、统计量文件（`meta/stats.json`）以及相机特征键名（`rename_map`）符合 VLA 预训练模型的输入契约。

### 1.1 检查 `LeRobotDataset v3.0` 目录与 `meta/stats.json` 分位数

采集完成后，数据集默认位于工作区的 `data/datasets/<dataset_name>/` 下，标准结构如下：

```text
data/datasets/arm101_pick_cube/
├── meta/
│   ├── info.json          # 数据集版本 (v3.0)、fps (30)、特征维度与相机分辨率
│   ├── stats.json         # 各特征的 mean / std / min / max / q01 / q99 统计量
│   ├── tasks.parquet      # 语言指令表 (single_task / subtask 描述)
│   └── episodes/          # 各回合元数据分片
├── data/
│   └── chunk-000/         # Apache Parquet 状态 (observation.state) 与动作 (action)
└── videos/
    ├── observation.images.front/
    └── observation.images.wrist/
```

> [!IMPORTANT]
> **`pi05` 等模型强制要求 `q01` / `q99` 分位数统计（Quantile Normalization）**：
> 如果你的数据集由旧版脚本转换而来，或 `meta/stats.json` 中缺少 `q01` 与 `q99` 字段，在启动 `pi05` 训练时会抛出 `ValueError: QUANTILES normalization mode requires q01 and q99 stats`。
> 请在上传或训练前运行 `lerobot-edit-dataset` 原地补算统计量：

```bash
lerobot-edit-dataset \
  --repo_id tust/arm101_pick_cube \
  --root data/datasets/arm101_pick_cube \
  --new_repo_id tust/arm101_pick_cube \
  --operation.type=recompute_stats \
  --operation.overwrite=true
```

### 1.2 相机特征键映射（`rename_map`）

本工作区针对 `arm101` 统一将双相机命名为 `front`（全局主视角）和 `wrist`（腕部第一视角），对应数据集键名：
- `observation.images.front`
- `observation.images.wrist`

而部分 VLA 基座模型（例如 `lerobot/smolvla_base`）在预训练时使用的视觉特征键名为 `observation.images.camera1`、`observation.images.camera2`。在 `repos/lerobot` `v0.7.1` 中，无需修改数据集文件，只需在训练 YAML 或 CLI 中配置 **`rename_map`** 即可在数据加载阶段自动完成零拷贝键名映射：

```yaml
# 示例：configs/train/smolvla_so101_cloud.yaml 中的 rename_map 配置
rename_map:
  observation.images.front: observation.images.camera1
  observation.images.wrist: observation.images.camera2
```

或在命令行中直接传入 JSON：

```bash
--rename_map='{"observation.images.front": "observation.images.camera1", "observation.images.wrist": "observation.images.camera2"}'
```

### 1.3 数据集上传 Hugging Face Hub 或离线打包传输

根据云端实例的网络环境，推荐以下两种数据传输方式：

#### 方式 A：通过 Hugging Face Hub 同步（推荐 HF Jobs / 海外云实例）
```bash
# 1. 登录 Hugging Face CLI（注意 v0.7.1 使用 hf 而非已弃用的 huggingface-cli）
hf auth login

# 2. 上传本地清洗后的数据集到你的 Hugging Face 账号
hf upload ${HF_USER}/arm101_pick_cube data/datasets/arm101_pick_cube --repo-type dataset
```

#### 方式 B：离线压缩包 + `scp` / 平台网盘直传（推荐 AutoDL / Featurize 国内实例）
```bash
# 1. 在本地工作区打包数据集
tar -czvf arm101_pick_cube.tar.gz -C data/datasets arm101_pick_cube

# 2. 通过 scp 上传至租用的 AutoDL / Featurize 数据盘
scp -P <SSH_PORT> arm101_pick_cube.tar.gz root@<REMOTE_HOST>:/root/autodl-tmp/

# 3. 在云端实例解压并使用 --dataset.root 与 --dataset.local_files_only=true 离线训练
mkdir -p /root/autodl-tmp/lerobot_workspace/data/datasets
tar -xzvf /root/autodl-tmp/arm101_pick_cube.tar.gz -C /root/autodl-tmp/lerobot_workspace/data/datasets
```

---

## 2. 本地显卡 vs 云端租用算力选型对比

不同模仿学习与 VLA 策略对显存（VRAM）的要求跨度极大。下表给出面向 `arm101` 的实测显存与算力租用建议：

| 算法 / 模型 | 参数规模 | 最低训练显存 | 推荐显卡配置 | 适用场景与本工作区配置模板 |
|---|---|---|---|---|
| **ACT** (`act`) | ~80M | **6 – 10 GB** | 本地 Ubuntu RTX 3060 / 4060 / 5060 (8GB) 或单卡云 GPU | 单任务固定点位快速基线验证；模板：[`configs/train/act_so101.yaml`](../../configs/train/act_so101.yaml) |
| **SmolVLA** (`smolvla`) | ~450M | **16 – 24 GB** | 云端 RTX 4090 (24GB) / L4 (24GB) / A10G (24GB) | 冻结视觉编码器 + 微调动作专家，性价比最高的轻量级多任务 VLA；模板：[`configs/train/smolvla_so101_cloud.yaml`](../../configs/train/smolvla_so101_cloud.yaml) |
| **$\pi_0$ / $\pi_0$-FAST** (`pi0` / `pi0_fast`) | ~3.3B | **24 GB**（冻结/小 Batch）– **48+ GB**（全参） | 云端 RTX 4090 (24GB, `freeze_vision_encoder=true`) / L40S (48GB) / A100 (80GB) | 复杂操作与高泛化能力 VLA；模板：[`configs/train/pi0_so101_cloud.yaml`](../../configs/train/pi0_so101_cloud.yaml) |
| **$\pi_{0.5}$** (`pi05` / `openpi`) | ~3.3B | **>22.5 GB**（LoRA）– **>70 GB**（全量微调） | 云端 RTX 4090 (24GB LoRA) / 单卡或双卡 A100 80GB / H100 80GB (FSDP) | 支持开放世界语义与分位数归一化的进阶 VLA；模板：[`configs/train/pi05_openpi_cloud.yaml`](../../configs/train/pi05_openpi_cloud.yaml) |

---

## 3. 三大云端算力租用平台实操流程（AutoDL / Featurize / Hugging Face Jobs）

### 3.1 AutoDL 算力云配置与训练流程（国内首选）

1. **租用实例与挂载路径**：
   - 在 AutoDL 控制台选择 **Ubuntu 22.04 + CUDA 12.1/12.4** 镜像（推荐 RTX 4090 24GB 或 A100 80GB）。
   - **重要存储注意**：AutoDL 系统盘（`/`）通常仅 30 GB，**所有仓库、虚拟环境、Hugging Face 缓存与 `models/checkpoints` 必须放在 `/root/autodl-tmp/` 数据盘**！
2. **云端环境一键初始化**：
   ```bash
   export HF_ENDPOINT="https://hf-mirror.com"
   export HF_HOME="/root/autodl-tmp/hf_cache"
   export LEROBOT_UBUNTU_VENV="/root/autodl-tmp/venvs/tust-lerobot"
   export OPENPI_UBUNTU_VENV="/root/autodl-tmp/venvs/tust-openpi"

   cd /root/autodl-tmp
   git clone https://github.com/TUST-iMaker/lerobot.git lerobot_workspace
   cd lerobot_workspace
   # 同步本工作区脚本与配置后，一键克隆上游仓库并创建训练环境
   python3 scripts/common/bootstrap_repos.py
   bash scripts/ubuntu/setup_ubuntu.sh --with-core-scripts --with-training --with-pi --with-smolvla
   source "${LEROBOT_UBUNTU_VENV}/bin/activate"
   ```

### 3.2 Featurize 算力平台配置流程（对齐同济子豪兄飞书 Wiki 方案）

1. **实例选择与数据集拉取**：
   - 在 **Featurize**（`https://featurize.cn`）选择 RTX 4090 或 A100 实例，工作目录建议使用 `/home/featurize/work`。
   - 如果已将数据集上传至 Featurize 数据集仓库，可在实例终端直接高速拉取：
     ```bash
     featurize dataset download <dataset-uuid>
     ```
2. **安装环境与登录凭证**：
   ```bash
   cd /home/featurize/work/lerobot
   bash scripts/ubuntu/setup_ubuntu.sh --with-core-scripts --with-training --with-pi --with-smolvla
   source ~/.venvs/tust-lerobot/bin/activate
   hf auth login
   ```

### 3.3 Hugging Face Jobs 云端无服务器训练（`--job.target`）

`repos/lerobot` `v0.7.1` 原生集成了 **Hugging Face Jobs（HF Jobs）** 远程调度能力。如果你拥有 Hugging Face Pro / 企业算力配额，无需手动 SSH 登录云服务器，直接在本地终端附加 `--job.target` 即可将训练任务提交到云端 GPU 容器：

```bash
# 本地一键提交 SmolVLA 训练任务到 HF Jobs 的 A10G 节点
lerobot-train \
  --config_path=configs/train/smolvla_so101_cloud.yaml \
  --dataset.repo_id=${HF_USER}/arm101_pick_cube \
  --policy.repo_id=${HF_USER}/smolvla_arm101_pick_cube \
  --policy.push_to_hub=true \
  --trackio.enable=true \
  --trackio.space_id=${HF_USER}/tust-trackio \
  --job.target=a10g-small \
  --job.timeout=12h
```

> [!WARNING]
> **使用 HF Jobs 的两个关键注意事项**：
> 1. 远程容器结束后本地磁盘即销毁，因此**必须设置 `--policy.push_to_hub=true`**，否则训练完的权重无法保存。
> 2. 远程 `trackio` 日志数据库默认写在容器内部，若想持久化查看实时训练曲线，必须同时指定 **`--trackio.space_id=${HF_USER}/tust-trackio`**。

---

## 4. `lerobot-train` 单卡与多卡（Accelerate / FSDP）训练命令

### 4.1 单卡训练（ACT / SmolVLA / Pi0）

激活 Ubuntu/云端虚拟环境（`source ~/.venvs/tust-lerobot/bin/activate`）后，可直接调用工作区预置的 YAML 模板：

```bash
# 1. 训练 ACT 基线模型（显存占用 ~8GB）
lerobot-train \
  --config_path=configs/train/act_so101.yaml \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --output_dir=models/checkpoints/act_so101_pick_cube

# 2. 微调 SmolVLA 模型（显存占用 ~18GB，自动应用 rename_map）
lerobot-train \
  --config_path=configs/train/smolvla_so101_cloud.yaml \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --output_dir=models/checkpoints/smolvla_so101_pick_cube

# 3. 微调 Pi0 模型（24GB 显卡建议开启 freeze_vision_encoder=true 与 train_expert_only=true）
lerobot-train \
  --config_path=configs/train/pi0_so101_cloud.yaml \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --policy.freeze_vision_encoder=true \
  --policy.train_expert_only=true \
  --batch_size=8 \
  --output_dir=models/checkpoints/pi0_so101_pick_cube
```

> **RTX 50 系列（Blackwell）视频解码兼容提示**：若在 RTX 5060 / 5090 等新架构显卡上训练时遇到 `torchcodec` 报错，请在 `lerobot-train` 命令后追加 `--dataset.video_backend=pyav`。

### 4.2 多卡分布式训练（`accelerate launch` / `torchrun` / `FSDP`）

当租用 2–8 张 GPU（例如双卡 RTX 4090 或 4 卡 A100）训练大型 VLA 时，使用 `accelerate launch`（或 `torchrun`）启动多卡数据并行或 FSDP 分片训练：

```bash
# 使用 accelerate launch 启动双卡 bfloat16 混合精度训练
accelerate launch \
  --multi_gpu \
  --num_processes=2 \
  --mixed_precision=bf16 \
  $(which lerobot-train) \
  --config_path=configs/train/pi0_so101_cloud.yaml \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --batch_size=16 \
  --output_dir=models/checkpoints/pi0_so101_multi_gpu
```

- **环境变量注意**：`lerobot` `v0.7.1` 默认忽略全局 `accelerate config` YAML 隐式注入的环境变量以防配置漂移；若确实需要复用宿主机的 accelerate 配置文件（例如自定义 DeepSpeed / FSDP 配置），请显式导出 `export LEROBOT_ALLOW_ACCELERATE_ENV=1`。
- 若使用 `repos/openpi` 训练 `pi05`，可直接通过 `--fsdp-devices=2` 开启 JAX/PyTorch FSDP 模型分片（详见下一篇 [`02_pi_series_and_pi07.md`](02_pi_series_and_pi07.md)）。

---

## 5. 模型权重（`models/checkpoints`）回传、评估与真机部署

### 5.1 Checkpoint 目录结构与下载回本地

`lerobot-train` 按 `save_freq` 定期将权重保存在 `models/checkpoints/<job_name>/checkpoints/` 下：

```text
models/checkpoints/smolvla_so101_pick_cube/
└── checkpoints/
    ├── 005000/pretrained_model/
    ├── 010000/pretrained_model/
    ├── 020000/pretrained_model/
    │   ├── config.json
    │   ├── model.safetensors
    │   └── train_config.json
    └── last -> 020000/
```

从 AutoDL / Featurize 将训练好的最优权重拉回本地工作区 `models/checkpoints/`：

```bash
# 在本地终端执行 scp 下载 last/pretrained_model
scp -P <SSH_PORT> -r \
  root@<REMOTE_HOST>:/root/autodl-tmp/lerobot_workspace/models/checkpoints/smolvla_so101_pick_cube/checkpoints/last/pretrained_model \
  models/checkpoints/smolvla_so101_best
```

### 5.2 仿真基准评估（`lerobot-eval`）与远程策略服务（`lerobot-policy-server`）

- **离线/仿真评估（`lerobot-eval`）**：
  ```bash
  lerobot-eval \
    --policy.path=models/checkpoints/smolvla_so101_best \
    --eval.n_episodes=10 \
    --eval.batch_size=1 \
    --output_dir=logs/eval_smolvla
  ```
- **分离式客户端-服务端推理（`lerobot-policy-server`）**：
  当本地 Windows 轻薄本连接着 `arm101` 真机，但没有大显存 GPU 时，可以在局域网 Ubuntu 主机或云端 GPU 上启动基于 Zenoh `ServerConfig` 的 `lerobot-policy-server`，让本地机械臂通过网络请求动作块：
  ```bash
  # 在 GPU 服务端启动策略推理服务（对齐 v0.7.1 Zenoh ServerConfig）
  lerobot-policy-server \
    --deployment=server \
    --model.repo_or_path=models/checkpoints/smolvla_so101_best \
    --model.device=cuda \
    --zenoh.connect_endpoints="[]"
  ```

### 5.3 真机闭环推理与人机协同部署（`lerobot-rollout`）

在 `repos/lerobot` `v0.7.1` 中，真机推理已由专用的 **`lerobot-rollout`** 承载（取代了旧版 `v0.4.x` / `v0.5.x` 的 `lerobot-record --policy.path`）：

```bash
lerobot-rollout \
  --strategy.type=base \
  --policy.path=models/checkpoints/smolvla_so101_best \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --robot.max_relative_target=10.0 \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: MJPG}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30, fourcc: MJPG} }" \
  --task="Pick up the red cube and place it into the tray" \
  --duration=60 \
  --device=cuda
```

> [!TIP]
> **真机首次推理安全守则**：首次测试新训练的 Checkpoint 时，务必加上 `--robot.max_relative_target=10.0` 限制单步最大关节跳变角度，并将手放在从臂 12V 电源开关旁，防止模型输出异常动作导致打齿或碰撞。
