# `pi0.7`（$\pi_{0.7}$）权威核查与 `pi0` / `pi0.5` / `openpi` / `smolvla` 全系列 VLA 实战指南

本文档针对用户重点关注的 **`pi0.7`（$\pi_{0.7}$）** 进行基于一手论文与官方仓库源码的事实核查，明确区分 **“已发表论文但暂未开源权重的模型”** 与 **“当前可直接下载权重并在 `arm101` 上微调部署的开源 VLA 方案”**，并提供 `huggingface/lerobot`（`v0.7.1`）与 `Physical-Intelligence/openpi` 两套完整的可执行流水线。

---

## 1. `pi0.7`（$\pi_{0.7}$）权威事实核查（arXiv:2604.15483v2）

### 1.1 论文出处与发布时间

- **论文标题**：*$\pi_{0.7}$: a Steerable Generalist Robotic Foundation Model with Emergent Capabilities*
- **发布机构**：Physical Intelligence (PI)
- **arXiv 编号与链接**：[`arXiv:2604.15483v2`](https://arxiv.org/abs/2604.15483)（2026 年 4 月 16 日提交 v1，2026 年 4 月 24 日修订 v2）
- **官方项目主页**：`https://pi.website/pi07`

### 1.2 核心技术架构（~5B 参数规模）

根据 `arXiv:2604.15483v2` 论文正文，$\pi_{0.7}$ 是 Physical Intelligence 在 $\pi_0$、$\pi_0\text{-FAST}$、$\pi_{0.5}$ 与 $\pi^*_{0.6}$ 之后推出的新一代可引导（Steerable）通用具身基座模型：

1. **骨干网络与参数规模（总计约 5B 参数）**：
   - **VLM 视觉语言主干（~4B）**：基于 **Gemma 3 4B**，其中包含一个约 **400M 参数的 MEM 视频历史视觉编码器**（支持跨帧时空压缩，能够编码多帧历史视觉上下文，而非仅看当前单帧图像）。
   - **动作专家模块（Action Expert，~860M）**：采用基于流匹配（Flow Matching）的连续动作生成网络，并结合 **知识绝缘（Knowledge Insulation）** 训练机制（使用离散 FAST 动作 token 监督 VLM 主干，同时阻断动作专家梯度回传破坏 VLM 语义表征）。
2. **多模态可引导上下文条件化（Steerable Multimodal Context Conditioning）**：
   - $\pi_{0.7}$ 的输入提示不仅包含顶层自然语言任务（Task Prompt），还可以动态接收：**细粒度子任务指令（Subtask Text）**、**世界模型生成的视觉子目标图像（Visual Subgoal Images）**、**回合元数据标签（速度/质量/纠错标志）** 以及 **控制模式标识**。
   - 这种设计使模型能够利用海量混合质量数据（包含失败自主轨迹、人类第一视角视频、互联网图文数据）进行预训练，并在推理时通过高层语言或视觉子目标实时引导机械臂产生涌现的组合泛化能力。

### 1.3 开源现状核查结论（重要：暂未开源 / 未公开发布权重）

> [!WARNING]
> **100% 事实核查结论：`pi0.7` 目前【未开源 / 未公开发布权重与代码】**
> 经对 [`Physical-Intelligence/openpi`](../../repos/openpi/)（本地 `repos/openpi`）、[`huggingface/lerobot`](../../repos/lerobot/)（`v0.7.1`，本地 `repos/lerobot/src/lerobot/policies/`）以及 Hugging Face Hub 官方模型库（`huggingface.co/lerobot`）的全面核查：
> - **`pi0.7`（$\pi_{0.7}$）目前处于闭源 / 没有公开权重状态**（仅公开了 `arXiv:2604.15483v2` 论文与演示页面，`openpi` 与 `lerobot` 仓库中均未发布 `pi07` 代码或权重；同理，$\pi^*_{0.6}$ 也未发布公开权重）。
> - **当前已完整开源（Apache-2.0）、可直接下载预训练权重并在 `arm101` 上微调部署的 Physical Intelligence 模型为**：**`pi0`（$\pi_0$）**、**`pi0_fast`（$\pi_0\text{-FAST}$）** 与 **`pi05`（$\pi_{0.5}$）**，以及 Hugging Face 官方轻量级 **`smolvla`**。

### 1.4 面向 $\pi_{0.7}$ / $\pi_{0.5}$ 范式的数据集子任务标注前瞻（`lerobot-annotate`）

虽然 $\pi_{0.7}$ 权重暂未开放，但其核心的 **“高层任务 + 细粒度子任务阶段标注（Subtask Conditioning）”** 已经在 $\pi_{0.5}$ 与 `repos/lerobot` `v0.7.1` 的 **`lerobot-annotate`** 工具中落地。你可以使用 `lerobot-annotate` 为采集好的 `arm101` 数据集打上时间段子任务标签（如 `"reach red cube"` -> `"grasp cube"` -> `"place in tray"`），既可用于当前 `pi05` 微调，也能无缝兼容未来开放的任何可引导 VLA 模型。

---

## 2. Pi 系列与开源 VLA 家族全景对比

| 模型代号 | `lerobot` `policy.type` | Hugging Face 预训练权重 ID | `openpi` 权重路径 | 核心特点与适用场景 |
|---|---|---|---|---|
| **$\pi_{0.7}$** (`pi0.7`) | *(未开源 / 暂无)* | *(未公开发布权重)* | *(未公开发布权重)* | ~5B（Gemma3-4B + 860M Action Expert），多模态子目标可引导，参考论文 `arXiv:2604.15483v2` |
| **$\pi_{0.5}$** (`pi05`) | `pi05` | `lerobot/pi05_base`<br>`lerobot/pi05_libero_base` | `gs://openpi-assets/checkpoints/pi05_base` | **当前最强的开源 PI 模型**：支持子任务分层推理、离散状态输入与分位数（Quantiles）归一化 |
| **$\pi_0$** (`pi0`) | `pi0` | `lerobot/pi0_base`<br>`lerobot/pi0_libero_base` | `gs://openpi-assets/checkpoints/pi0_base` | PaliGemma-2B + 300M Flow-Matching 动作专家，高频连续控制平滑度极佳 |
| **$\pi_0\text{-FAST}$** (`pi0_fast`) | `pi0_fast` | `lerobot/pi0fast-base` | `gs://openpi-assets/checkpoints/pi0_fast_base` | 采用 FAST（频域自回归动作分词器），自回归收敛速度显著快于纯扩散/流匹配 |
| **SmolVLA** (`smolvla`) | `smolvla` | `lerobot/smolvla_base` | *(LeRobot 原生)* | ~450M 轻量级 VLA，单张 16–24GB 显卡即可高速微调，适合教学与快速迭代 |

---

## 3. 基于 `huggingface/lerobot` (v0.7.1) 的可执行 VLA 微调流水线

在 Ubuntu 或云端算力实例中，请先确保已安装 `[pi]` 与 `[smolvla]` 扩展依赖（可通过 `bash scripts/ubuntu/setup_ubuntu.sh --with-pi --with-smolvla` 一键安装；注意 `v0.7.1` 的扩展名为 `.[pi]` 而非旧教程中的 `.[pi0]`）。

### 3.1 关键参数辨析：`--policy.pretrained_path` vs `--policy.path`

在 `repos/lerobot` `v0.7.1` 中微调预训练 VLA 时，这两个参数的行为截然不同：
- **`--policy.type=<type> --policy.pretrained_path=<hf_repo>`**（**推荐用于 `pi0` / `pi05` / `pi0_fast` 跨本体微调**）：只加载预训练模型的神经网络权重，而**根据你当前 `arm101` 数据集的实际特征维度和相机名称重新构建 `config.json`**。
- **`--policy.path=<hf_repo_or_local_dir>`**（**用于 `smolvla` 模板或续训已有 Checkpoint**）：同时加载远端的 `config.json` 与权重，此时**严禁再传 `--policy.type`**，且若远端 `config.json` 的相机键名（如 `camera1`）与本地数据集（`front`/`wrist`）不同，必须配合 `rename_map` 使用。

### 3.2 `pi0` 微调与推理（对应 `configs/train/pi0_so101_cloud.yaml`）

```bash
# 使用本工作区预置的 pi0_so101_cloud.yaml 启动微调
lerobot-train \
  --config_path=configs/train/pi0_so101_cloud.yaml \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --policy.type=pi0 \
  --policy.pretrained_path=lerobot/pi0_base \
  --policy.dtype=bfloat16 \
  --policy.gradient_checkpointing=true \
  --policy.compile_model=true \
  --output_dir=models/checkpoints/pi0_so101
```

- 若在 24 GB 显存（如单卡 RTX 4090 / A10G）上运行，请加上 `--policy.freeze_vision_encoder=true --policy.train_expert_only=true --batch_size=8`。

### 3.3 `pi0_fast` 微调与推理

`pi0_fast` 使用频域动作 Token 化（FAST），适合需要更快训练收敛速度的桌面抓取任务：

```bash
lerobot-train \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --policy.type=pi0_fast \
  --policy.pretrained_path=lerobot/pi0fast-base \
  --policy.push_to_hub=false \
  --policy.dtype=bfloat16 \
  --policy.gradient_checkpointing=true \
  --policy.device=cuda \
  --batch_size=16 \
  --steps=10000 \
  --output_dir=models/checkpoints/pi0_fast_so101
```
*(注：上式中 `--policy.device=cuda` 请确保紧贴无空格。)*

```bash
# 标准紧凑写法：
lerobot-train \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --policy.type=pi0_fast \
  --policy.pretrained_path=lerobot/pi0fast-base \
  --policy.push_to_hub=false \
  --policy.dtype=bfloat16 \
  --policy.gradient_checkpointing=true \
  --policy.device=cuda \
  --batch_size=16 \
  --steps=10000 \
  --output_dir=models/checkpoints/pi0_fast_so101
```

### 3.4 `pi05`（$\pi_{0.5}$）微调与分位数归一化处理

`pi05` 默认对状态和动作采用 **`QUANTILES`（1% 与 99% 分位数）归一化**，以抑制遥操作过程中的离群尖峰：

```bash
# 第一步：确保数据集 meta/stats.json 含有 q01 与 q99 分位数
lerobot-edit-dataset \
  --repo_id tust/arm101_pick_cube \
  --root data/datasets/arm101_pick_cube \
  --new_repo_id tust/arm101_pick_cube \
  --operation.type=recompute_stats \
  --operation.overwrite=true

# 第二步：启动 pi05 微调（对应 configs/train/pi05_openpi_cloud.yaml Section B）
lerobot-train \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --policy.type=pi05 \
  --policy.pretrained_path=lerobot/pi05_base \
  --policy.push_to_hub=false \
  --policy.dtype=bfloat16 \
  --policy.gradient_checkpointing=true \
  --policy.device=cuda \
  --batch_size=16 \
  --steps=20000 \
  --output_dir=models/checkpoints/pi05_so101
```

> **备选回退方案**：如果无法修改只读数据集的 `stats.json`，可在训练命令中显式覆盖归一化映射：
> `--policy.normalization_mapping='{"ACTION":"MEAN_STD","STATE":"MEAN_STD","VISUAL":"IDENTITY"}'`

### 3.5 `smolvla` 轻量级微调（对应 `configs/train/smolvla_so101_cloud.yaml`）

```bash
lerobot-train \
  --config_path=configs/train/smolvla_so101_cloud.yaml \
  --dataset.repo_id=tust/arm101_pick_cube \
  --dataset.root=data/datasets/arm101_pick_cube \
  --output_dir=models/checkpoints/smolvla_so101
```

---

## 4. 基于 `Physical-Intelligence/openpi`（`repos/openpi`）的 Ubuntu 原生工作流

对于需要深入定制 JAX / PyTorch 底层模型结构、使用 LoRA 微调或启动 WebSocket 远程策略服务的场景，本工作区已在 [`repos/openpi`](../../repos/openpi/) 克隆了官方 **OpenPI** 仓库，并在 [`configs/train/pi05_openpi_cloud.yaml`](../../configs/train/pi05_openpi_cloud.yaml) 中提供了完整的配置参数对照表。

### 4.1 操作系统与显存硬性门槛（Ubuntu 22.04 + Python 3.11）

根据 `repos/openpi/README.md` 官方说明：
- **仅支持 Ubuntu / Linux（推荐 Ubuntu 22.04）**：由于依赖 `jax[cuda12]` 与特定的 Triton/XLA 算子，OpenPI 不支持在原生 Windows 上直接训练；双系统用户请在 **Ubuntu 侧**（或云端 Linux 实例）运行。
- **独立 Python 3.11 虚拟环境**：`openpi` 要求 Python `3.11`（而 `lerobot` `v0.7.1` 要求 Python `>=3.12`）。本工作区的 `scripts/ubuntu/setup_ubuntu.sh --with-openpi` 会自动在 `~/.venvs/tust-openpi` 创建隔离的 Python 3.11 环境，绝不与 `~/.venvs/tust-lerobot` 冲突。
- **官方显存要求（VRAM Requirements）**：
  - **推理（Inference）**：**`> 8 GB`**（如 RTX 4060 Ti / 4090）
  - **LoRA 微调（LoRA Fine-Tuning）**：**`> 22.5 GB`**（如单卡 RTX 4090 24GB / A10G 24GB，配置 `paligemma_variant="gemma_2b_lora"`、`action_expert_variant="gemma_300m_lora"`、`ema_decay=None`）
  - **全量微调（Full Fine-Tuning）**：**`> 70 GB`**（如单卡 A100 80GB / H100 80GB，或多卡设置 `--fsdp-devices=<N>`）

### 4.2 OpenPI 标准四步工作流（`compute_norm_stats.py` -> `train.py` -> `serve_policy.py`）

在 Ubuntu 终端进入 `repos/openpi` 并激活 `tust-openpi` 环境：

```bash
cd repos/openpi
source "${OPENPI_UBUNTU_VENV:-$HOME/.venvs/tust-openpi}/bin/activate"
```

#### 第一步：计算数据集归一化统计量（`compute_norm_stats.py`）
在参照 [`configs/train/pi05_openpi_cloud.yaml`](../../configs/train/pi05_openpi_cloud.yaml) 的 `openpi_train_config` 在 `src/openpi/training/config.py` 注册好你的 `pi05_so101_custom` 配置后，必须先计算归一化统计文件：

```bash
uv run scripts/compute_norm_stats.py --config-name pi05_so101_custom
```

#### 第二步：启动 JAX 或 PyTorch 微调训练（`train.py` / `train_pytorch.py`）
```bash
# 方式 A：JAX 原生训练（默认推荐，通过 XLA_PYTHON_CLIENT_MEM_FRACTION 预分配 90% 显存）
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi05_so101_custom \
  --exp-name=so101_run1 \
  --overwrite

# 方式 B：PyTorch 训练（需先用 examples/convert_jax_model_to_pytorch.py 转换基础权重）
uv run scripts/train_pytorch.py pi05_so101_custom \
  --exp_name=so101_run1 \
  --save_interval=1000
```

#### 第三步：启动 OpenPI WebSocket 策略推理服务（`serve_policy.py`）
训练完成后，使用 `scripts/serve_policy.py` 在 `8000` 端口启动低延迟 WebSocket 策略服务器：

```bash
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_so101_custom \
  --policy.dir=checkpoints/pi05_so101_custom/so101_run1/20000 \
  --port=8000
```

#### 第四步：从本地 Arm101 Studio 或真机客户端连接推理
- 启动 `serve_policy.py` 后，无论服务端运行在局域网 Ubuntu 主机还是通过 SSH 隧道（`ssh -L 8000:127.0.0.1:8000 ...`）转发的云端 GPU 上，本地 **Arm101 Studio**（`app/`）的 **“插件与外部工具（`remote_vla_policy`）”** 以及 **“VLA & Pi0.7 中心”** 均可直接检测并对接该端点，实现 Windows/Ubuntu 跨系统协同推理。
