# `models/` — 本地模型与训练权重目录

本目录用于集中管理下载的预训练基础模型与本地/云端回传的训练检查点（checkpoints）。除本 `README.md` 与子目录 `.gitkeep` 外，**所有模型权重文件均被 `.gitignore` 忽略**。

## 目录结构与用途

- `models/pretrained/` — 本地下载或转换的预训练基础模型（例如 SmolVLA、Pi0、Pi0.5、ACT 基础权重或本地推理快照）。
- `models/checkpoints/` — 训练产出的权重目录（`configs/train/*.yaml` 默认将 `output_dir` 指向 `models/checkpoints/<run_name>`）。

## 扩展与云端回传约定

- 在 AutoDL / Featurize / HF Jobs 等云端 GPU 训练完成后，建议将最终验证可用的 checkpoint 同步回 `models/checkpoints/<policy>_<task>_<date>/`，或在 Hugging Face Hub 上打标签保存。
- 运行 `openpi`（Ubuntu 侧）的大体积基础权重（`gs://openpi-assets/checkpoints/...`）默认缓存在 `OPENPI_DATA_HOME`（`~/.cache/openpi`），如需离线归档可软链接或复制到 `models/pretrained/`。
