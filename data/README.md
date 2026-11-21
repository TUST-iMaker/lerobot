# `data/` — 本地数据目录

本目录用于存放与机器臂工作流相关的本地运行数据。除本 `README.md` 与各子目录的 `.gitkeep` 骨架外，**所有实际采集的数据集、标定文件与演练产物均被 `.gitignore` 忽略**，不会污染 Git 仓库。

## 目录结构与用途

- `data/datasets/` — `lerobot-record`（以及本地 Studio）采集的 LeRobot v3.0 数据集，通常通过 `HF_LEROBOT_HOME=./data` 定位。
- `data/calibration/` — SO-ARM101 主/从臂关节零点与限位标定文件：
  - `data/calibration/robots/so_follower/<id>.json`
  - `data/calibration/teleoperators/so_leader/<id>.json`
- `data/dry_run/` — 无硬件连接时本地 Arm101 Studio 模拟运行生成的示教轨迹点（waypoints）与演练录制数据集。

## 扩展与团队协作约定

- 新增任务数据集时，建议按 `<任务名>_<日期>` 或 `<hf_user>/<dataset_name>` 命名存放，避免多人覆盖同一目录。
- 重要数据集应通过 `huggingface-cli` / `lerobot-record --dataset.push_to_hub=true` 推送到 Hugging Face Hub 进行版本化共享；Ubuntu 训练机可通过同一 Hub ID 或局域网/共享磁盘同步拉取。
