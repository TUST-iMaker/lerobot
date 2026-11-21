# 5. 多人协作数据采集规范（SO-ARM101 / LeRobotDataset v3.0）

为确保团队成员在不同工位、不同操作系统（Windows / Ubuntu）采集的演示数据能够**无缝合并、直接用于 ACT / SmolVLA / Pi0 / Pi0.5 训练**，所有成员在录制数据集前请统一遵循本规范。本规范也已直接内置到 **Arm101 Studio → 数据采集** 页面中，录制前会自动执行实时合规校验。

---

## 1. 数据集命名规范（强制）

| 规范项 | 要求 | 示例 |
|---|---|---|
| **格式模板** | `<成员>_<动作>_<物体>_<YYYYMMDD>`（或 `<repo_owner>/<上述名称>`） | `yyz_pick_cube_20261121`、`aguiya_stack_cup_20261121` |
| **字符集** | 仅允许小写英文字母、数字、下划线 `_`、连字符 `-` | 禁止空格、中文路径或特殊符号 |
| **禁用前缀** | **严禁使用 `eval_` 开头**（`lerobot` 将 `eval_` 保留为策略评估输出，录制时会直接报错） | `eval_test` ❌ |
| **本地存放目录** | 统一位于 `data/datasets/<dataset_name>/`（`dry_run` 演练数据位于 `data/dry_run/datasets/`） | 自动被 `.gitignore` 忽略，避免误提交大文件 |

---

## 2. 强制采集字段与 6 轴关节顺序（对齐 `repos/lerobot v0.7.1`）

### 2.1 状态与动作维度（必须 6 轴且统一角度制）

所有数据集必须同时记录 `observation.state`（从臂当前关节状态）与 `action`（主臂指令动作），且 `use_degrees=true`（角度制，与 `arm101` 官方 `so101_follower` / `so101_leader` 标定一致）：

| 索引 | 舵机 ID | 关节字段名 | 单位 | 有效范围 |
|---|---|---|---|---|
| 0 | ID 1 | `shoulder_pan.pos` | `deg`（°） | `[-110°, +110°]` |
| 1 | ID 2 | `shoulder_lift.pos` | `deg`（°） | `[-95°, +95°]` |
| 2 | ID 3 | `elbow_flex.pos` | `deg`（°） | `[-95°, +95°]` |
| 3 | ID 4 | `wrist_flex.pos` | `deg`（°） | `[-95°, +95°]` |
| 4 | ID 5 | `wrist_roll.pos` | `deg`（°） | `[-150°, +150°]` |
| 5 | ID 6 | `gripper.pos` | `%`（开合度） | `[0%, 100%]` |

> **关键检查点**：严禁混用弧度制与角度制，严禁跳过零位标定直接采数。标定参考姿态统一采用官方中位图（大臂竖直向上、小臂与夹爪水平前伸、动爪在上、定爪在下）。

### 2.2 标定 ID 兼容性说明

- 团队主从臂标定文件可采用 `arm101_follower` / `arm101_leader`（本仓库 `.env.example` 默认）或个人前缀（如 `zihao_follower` / `zihao_leader`）。
- **核心要求**：只要在标定时严格对齐官方中位（`0°` 上臂竖直、小臂水平前伸）并完成完整行程扫角，不同标定 ID 生成的 `observation.state` / `action` 角度空间完全一致，可直接合并训练。

---

## 3. 相机与视觉通道规范

| 相机键名（Camera Key） | 角色 | 推荐分辨率 | 帧率 | 说明 |
|---|---|---|---|---|
| `observation.images.front`（简写 `front`） | 全局第三视角（俯视/斜前视角） | `640 × 480` | `30 FPS` | **必选**；覆盖完整桌面作业区与机械臂底座 |
| `observation.images.wrist`（简写 `wrist`） | 腕部第一视角（手眼相机） | `640 × 480` | `30 FPS` | **强烈推荐**；抓取精细对准时对 ACT / Pi0.5 提升显著 |

- **单相机约定**：若工位仅接 1 个摄像头，统一使用键名 `front`（不要随意命名为 `cam0` / `top` / `usb0`），防止多成员数据合并时出现特征键不匹配。
- **双相机方向**：采集前先在 **Arm101 Studio → 遥操作与相机** 页确认 `front` 与 `wrist` 画面方向一致（如接反可一键点击「互换双相机」）。

---

## 4. 任务文本、帧率与统计量规范

1. **任务描述（`single_task`）**：统一使用**简洁英文祈使句**，描述“动作 + 目标物 + 放置位置”，例如：
   - `Pick up the red cube and place it in the bowl`
   - `Grasp the blue cup and stack it on the plate`
   - 同一任务的多个成员应复制完全相同的 `single_task` 字符串（便于 VLA 语言条件对齐）。
2. **采集帧率（`fps`）**：统一固定为 **`30 FPS`**（与 `configs/train/` 中的 ACT / SmolVLA / Pi0 / Pi0.5 预设一致）。
3. **单条时长与集数建议**：
   - 单条 `episode_time_s`：`15–30 s`（动作连贯，避免长时间停顿）。
   - 起止姿态：每条 Episode 尽量从 `Home` 或 `Inspect` 预备姿态起手，完成后回到安全悬停态再按 `→` 结束。
   - 规模参考：单任务演示验证建议 `≥ 20` 条；ACT / SmolVLA 稳定泛化建议 `50–100` 条；多成员合并时建议每人覆盖略有差异的物体摆放位置。
4. **Pi0.5 / OpenPI 统计量要求**：`pi05` 依赖分位数归一化统计量（`q01` / `q99`），使用 `lerobot v0.7.1` 的 `lerobot-record` 录制会自动生成 `meta/stats.json`。合并多个成员数据集时使用 `lerobot-edit-dataset` 重新汇总统计量。

---

## 5. 采集后质检清单（QC Checklist）

每批数据录制完成后，在上传 Hugging Face 或传输给训练机之前，请完成以下检查：

1. **Studio 火柴棍轨迹回放**：在 **Arm101 Studio → 数据采集** 页的「已采集 Episodes」列表中，点击每条记录旁的 **「火柴棍回放」**，确认 6 轴机械臂动作顺滑、无关节跳变、无整条静止的废片。
2. **官方可视化抽检**：使用 `lerobot-dataset-viz`（或 Studio 插件页一键启动）抽检视频帧与关节曲线是否时间同步。
3. **剔除坏轨迹**：若某条 Episode 出现抓取失败、撞桌或相机卡顿，在录制时按 `←` 立即重录，或事后通过 `lerobot-edit-dataset` 删除对应 `episode_index`。
4. **完整性核对**：确认数据集目录包含 `meta/info.json`、`meta/episodes/`、`data/` 与 `videos/`（`dry_run` 模拟数据集则包含 `meta/info.json` 与 `episodes/*.json`）。
