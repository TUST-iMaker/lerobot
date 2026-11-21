# 本地串联软件操作指南：TUST-iMaker Arm101 Studio

**Arm101 Studio（`app/`）** 是本工作区配套的本地串联控制台，旨在将官方 **arm101（SO-ARM101）主从臂** 的硬件探测、零点标定、双臂遥操作、演示示教、数据采集、开源前端工具与 VLA 云端配置整合至同一离线 Web 界面中，并在无硬件连接时自动降级为 **`dry_run` 6-DOF 模拟模式**，方便教学演示与流程预演。

---

## 1. 启动前准备与双系统说明

1. **确认虚拟环境已就绪**：
   - **Windows 侧**：使用工作区内置环境 `envs\win\lerobot\Scripts\python.exe`（通过 `scripts\windows\setup_win.ps1` 初始化）。
   - **Ubuntu / WSL2 侧**：使用原生 ext4 环境 `~/.venvs/tust-lerobot/bin/python`（通过 `scripts/ubuntu/setup_ubuntu.sh` 初始化）。
2. **可选配置 `.env`**：
   - 复制根目录 `.env.example` 为 `.env`，按实际串口号修改 `ARM101_FOLLOWER_PORT_WIN` / `ARM101_LEADER_PORT_WIN`（Windows `COM*`）或 `ARM101_FOLLOWER_PORT_UBUNTU` / `ARM101_LEADER_PORT_UBUNTU`（Ubuntu `/dev/ttyACM*`）。
   - `STUDIO_HOST` 默认 `127.0.0.1`，`STUDIO_PORT` 默认 `8760`，`STUDIO_MODE` 默认 `auto`。

### 启动命令

```powershell
# Windows 快捷启动（自动探测硬件，无机械臂时自动降级为 dry_run）
envs\win\lerobot\Scripts\python.exe app\run_studio.py --port 8760

# 显式强制 dry_run 且不自动弹窗
envs\win\lerobot\Scripts\python.exe app\run_studio.py --dry-run --no-browser

# 或直接使用 uvicorn / 模块入口启动
envs\win\lerobot\Scripts\python.exe -m app.backend.main
```

```bash
# Ubuntu / WSL2 启动
~/.venvs/tust-lerobot/bin/python app/run_studio.py --port 8760 --no-browser
```

启动后在浏览器访问 `http://127.0.0.1:8760`。

---

## 2. 界面布局与运行模式说明

页面顶部常驻状态栏包含以下核心指示器：

- **运行模式徽标（`mode-badge`）**：
  - `DRY-RUN 模拟模式`（琥珀色）：未检测到双臂 USB 串口或手动选择 `dry_run` 时启用，所有遥操作、示教、采集与相机预览均在本地安全仿真器中运行，不会操作真实串口。
  - `REAL 硬件模式`（绿色）：当检测到双臂 SO-101 串口且 `lerobot` 可用时激活。若无硬件连接强行切换 `real`，后端会返回 `409 Conflict` 提示缺失部件并安全保持 `dry_run`。
- **硬件状态横幅（`hardware-status-banner`）**：实时展示缺失硬件标签与双语排查建议（已自动忽略 Windows 蓝牙虚拟串口如 `COM8` / `COM9`，避免误判为机械臂）。
- **底部实时日志台（`log-console`）**：通过 WebSocket `/ws/telemetry` 实时流式显示后台工作流日志；当 WS 断开时前端自动回退至 `1500ms` HTTP 轮询并按指数退避重连。

---

## 3. 六大核心模块实操步骤

### 3.1 总览与标定（`总览与标定` 标签页）

1. 点击 **重新探测硬件** 刷新串口列表（识别 CH340/STM32 等 USB-串口芯片）、OpenCV 摄像头（Windows DSHOW / Linux V4L2）及系统能力矩阵。
2. **单臂零点标定**：选择目标臂 `so101_follower` 或 `so101_leader`，点击 **开始标定**。
   - **重要安全规范**：官方 `lerobot-calibrate` 每次只允许传入 `--robot.*` 或 `--teleop.*` 之一，Studio 后台已严格隔离参数构造。
   - `dry_run` 模式下会在 `data/dry_run/calibration/` 生成模拟标定预览 JSON，不覆盖 `data/calibration/` 中的实机标定文件。

### 3.2 遥操作与 6-DOF 监控（`遥操作` 标签页）

1. 设置目标帧率（默认 `60` FPS），点击 **启动遥操作**。
2. 查看左侧 **6-DOF 关节状态**：严格对齐 SO-101 官方 6 个舵机通道：
   - `shoulder_pan`、`shoulder_lift`、`elbow_flex`、`wrist_flex`、`wrist_roll`（单位：度 `[-180, 180]°`）
   - `gripper`（夹爪开合百分比 `[0, 100]%`）
3. 在 `dry_run` 模式下拖动任一滑块，右侧 **前置相机 (`front`)** 与 **腕部相机 (`wrist`)** 的 2D 运动学投影 HUD 画面会同步响应机械臂姿态变化。

### 3.3 演示示教与轨迹回放（`示教与回放` 标签页）

1. 在遥操作中将机械臂调整到关键姿态后，输入可选航点备注（如 `grasp_ready`），点击 **记录示教点**。
2. 航点自动持久化到 `data/dry_run/waypoints.json`，并在表格中展示完整 6-DOF 关节快照。
3. 记录 1 个以上航点后点击 **回放轨迹**，系统按顺序平滑插值驱动虚拟从臂复现示教动作；点击 **清空轨迹** 可重置列表。

### 3.4 数据采集（`数据采集` 标签页）

1. 填写 **数据集名称**（注意：遵循 `lerobot-record` 规范，禁止使用 `eval_` 前缀）、**任务指令（`single_task`）**、**采集回合数**与 **FPS**。
2. 点击 **开始采集**，在 `dry_run` 模式下会在 `data/dry_run/datasets/<dataset_name>/` 生成标准 `meta/info.json` 与逐回合 `episodes/episode_XXXX.json` 轨迹数据；在 `real` 模式下会自动附加 `--dataset.push_to_hub=false` 避免离线环境报错。
3. 点击 **停止采集** 或等待回合结束，下方 **已落盘 Episode 列表** 会自动刷新展示帧数与磁盘路径。

### 3.5 开源插件中心（`开源插件中心` 标签页）

Studio 通过 `app/backend/plugins.py` 中的 `ToolAdapter` 注册表接入外部开源工具，并自动检测本机环境可用性：

| 插件 ID | 对应工具 | 本机适配说明 |
|---|---|---|
| `lerobot_dataset_viz` | `lerobot-dataset-viz` (Rerun / Foxglove) | 需安装 `datasets` 与 `rerun-sdk`（`setup_win.ps1 -WithCoreScripts`） |
| `feetech_servo_bench` | `Nori-Robotics/feetech-motorlab` | 需安装 `feetech-servo-sdk`（`scservo_sdk`），支持 `--simulate` |
| `lerobot_studio` | LeRobot 实时 Rerun/Foxglove 遥测流 | 需安装可视化 SDK 且连接实机双臂 |
| `lelab_external` | `huggingface/leLab` | 需构建前端静态资源并运行 `-WithLeLab` 安装 |
| `remote_vla_policy` | `Physical-Intelligence/openpi` / `lerobot-policy-server` | `openpi` 本地运行仅支持 **Ubuntu / 云端 Linux**，Windows 端通过远程 WS 端点联调 |

### 3.6 VLA 与云端配置（`VLA 与云端配置` 标签页）

1. 直观浏览工作区 `configs/train/` 下的 4 套配置模板：`act_so101.yaml`、`smolvla_so101_cloud.yaml`、`pi0_so101_cloud.yaml`、`pi05_openpi_cloud.yaml`，包括显存需求与直接可复制的训练/服务命令。
2. 内置 **`pi0.7` (`arXiv:2604.15483v2`) 状态提示卡**：明确标注其当前闭源状态，引导团队优先基于公开可复现的 `pi05` / `pi0` / `smolvla` 开展训练。
3. 支持输入远程 `openpi serve_policy.py` WebSocket 地址并生成 5 步 6-DOF Action Chunk 预测预览。

---

## 4. 常见问题排查

- **插上电源仍处于 `dry_run`？**
  检查 Windows 设备管理器是否识别为 CH340/STM32 USB 串口（运行 `scripts\windows\find_ports.ps1`），并确认已将真实端口号写入 `.env`；Ubuntu 用户请检查是否已加入 `dialout` 组并应用 `/etc/udev/rules.d/` 规则。
- **插件提示「依赖未就绪」？**
  Studio 会返回清晰的 `409 Conflict` 与 `install_hint` 安装指令，不会崩溃退出；按卡片提示安装对应可选依赖后点击「刷新插件状态」即可。
- **运行自动化测试**：
  ```powershell
  envs\win\lerobot\Scripts\pytest.exe app\tests\test_studio_api.py -v
  ```

---

## 5. 核心 HTTP 与 WebSocket API 端点速查

`app.backend.main`（由 `app/run_studio.py` 启动）对外暴露以下 REST 与 WebSocket 接口，供前端或自动化脚本调用：

| 方法与路径 | 说明 |
|---|---|
| `GET /api/health` | 返回服务健康状态、操作系统、Python 版本、工作区根目录以及当前模式（`real` / `dry_run`） |
| `GET /api/hardware/probe` | 探测串口（自动过滤蓝牙虚拟串口）、OpenCV 摄像头、`lerobot` 可用性及排障建议 |
| `POST /api/mode` | 切换运行模式（`auto` / `real` / `dry_run`） |
| `POST /api/workflows/calibrate/start` & `/stop` | 启动或停止单臂标定（`so101_follower` / `so101_leader`） |
| `POST /api/workflows/teleop/start`, `/stop`, `/joints` | 启动/停止遥操作，或在 `dry_run` 模式下更新 6-DOF 虚拟主臂关节角 |
| `POST /api/workflows/teaching/record_waypoint`, `/replay`, `/clear` & `GET /waypoints` | 演示示教关键帧记录、列表查询、平滑轨迹回放与清空 |
| `POST /api/workflows/collect/start`, `/stop` & `GET /episodes` | 数据集采集启停与已落盘 Episode 列表查询 |
| `GET /api/cameras/{cam_id}/frame` | 获取 `front` (`0`) 或 `wrist` (`1`) 相机的实时 JPEG 帧 |
| `GET /api/plugins` & `POST /api/plugins/{plugin_id}/launch`, `/stop` | 查询、启动或停止 `ToolAdapter` 开源工具插件 |
| `WS /ws/telemetry` | 10 Hz 实时推送 6-DOF 主从臂关节角、工作流状态与尾部日志流 |
