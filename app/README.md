# TUST-iMaker Arm101 Studio (`app/`)

**Arm101 Studio** 是面向官方 **SO-ARM101 (`arm101`)** 主从臂的本地串联控制台，基于 **FastAPI + WebSocket + 零构建离线前端（HTML5/CSS3/ESM）**，在无硬件连接时自动安全降级至 **`dry_run` 6-DOF 模拟模式**。

## 1. 目录结构

```text
app/
├── __init__.py
├── run_studio.py              # CLI 启动器（--host/--port/--mode/--dry-run/--no-browser）
├── backend/
│   ├── config.py              # 跨平台路径、.env 读取、robot/train YAML 加载
│   ├── hardware.py            # 串口探测（过滤蓝牙 COM）、OpenCV 相机枚举、模式解析
│   ├── process_manager.py     # 进程组隔离与树级清理（Windows taskkill /T, POSIX killpg）
│   ├── simulation.py          # SO-101 6-DOF 运动学状态模拟、严格参数校验与 Pillow HUD JPEG
│   ├── workflows.py           # 标定、遥操作、示教航点、数据采集编排器
│   ├── plugins.py             # ToolAdapter 开源工具插件中心
│   └── main.py                # FastAPI 应用、lifespan 生命周期与 WS /ws/telemetry
├── frontend/
│   ├── index.html             # 零外部 CDN 依赖的单页应用
│   ├── styles.css             # 工业控制台主题样式
│   └── app.js                 # 原生 ESM 状态管理、WebSocket 自动重连与轮询兜底
└── tests/
    └── test_studio_api.py     # Pytest + HTTPX + WebSockets + Playwright 端到端测试
```

## 2. 启动方式

在工作区根目录执行：

```powershell
# Windows（使用工作区 uv 虚拟环境）
envs\win\lerobot\Scripts\python.exe -m app.backend.main

# 或使用带参数启动器
envs\win\lerobot\Scripts\python.exe app\run_studio.py --host 127.0.0.1 --port 8760 --dry-run --no-browser
```

```bash
# Ubuntu / WSL2
~/.venvs/tust-lerobot/bin/python app/run_studio.py --host 127.0.0.1 --port 8760 --no-browser
```

默认访问地址：`http://127.0.0.1:8760`。

## 3. HTTP & WebSocket API 契约

- `GET /api/health`：返回服务状态、OS、Python 版本、当前运行模式 (`real` / `dry_run`) 与工作区路径。
- `GET /api/hardware/probe`：探测串口（自动过滤蓝牙虚拟串口）、摄像头、`lerobot` 依赖与系统能力。
- `POST /api/mode`：切换 `auto` / `real` / `dry_run`；无硬件请求 `real` 时返回 `409 Conflict`（`HARDWARE_NOT_READY`）并安全保持 `dry_run`。
- `POST /api/workflows/calibrate/start` & `/stop`：单臂标定（每次严格仅传入 `--robot.*` 或 `--teleop.*`）。
- `POST /api/workflows/teleop/start` & `/stop` & `/joints`：主从臂遥操作及 6-DOF 虚拟关节控制（严格边界校验）。
- `POST /api/workflows/teaching/record_waypoint`、`GET /waypoints`、`POST /replay`、`POST /clear`：示教航点持久化至 `data/dry_run/waypoints.json` 并支持平滑插值回放。
- `POST /api/workflows/collect/start` & `/stop` & `GET /episodes`：数据采集（`real` 强制 `--dataset.push_to_hub=false`；`dry_run` 落盘至 `data/dry_run/datasets/`）。
- `GET /api/cameras/{cam_id}/frame`：返回 `image/jpeg` 实时相机或 2D 运动学合成测试帧（未知 ID 返回 `404`）。
- `GET /api/plugins`、`POST /api/plugins/{plugin_id}/launch` & `/stop`：管理开源工具插件。
- `GET /api/vla/configs` & `POST /api/vla/mock_predict`：查看 `configs/train/*.yaml` 并生成 6-DOF 策略预览。
- `WS /ws/telemetry`：高频推送主从臂 6-DOF 关节角、工作流状态与实时日志尾部。

## 4. 扩展新工具插件 (`ToolAdapter`)

在 `app/backend/plugins.py` 中继承 `ToolAdapter`，实现 `probe()` 与 `build_launch_cmd(args)`，并在 `PluginRegistry.__init__` 注册即可自动出现在前端「开源插件中心」。

## 5. 测试与代码检查

```powershell
envs\win\lerobot\Scripts\ruff.exe check app
envs\win\lerobot\Scripts\pytest.exe app\tests\test_studio_api.py -v
```

> 若本地未下载 Playwright 默认 Revision 的 Chromium，`test_studio_api.py` 会自动回退检索 `%LOCALAPPDATA%\ms-playwright` 或系统 Edge/Chrome 可执行文件。
