# `arm101` 摄像头探测与实时遥操作指南

完成 [`01_calibration_and_ports.md`](01_calibration_and_ports.md) 的串口识别与主从臂标定后，本篇介绍如何探测摄像头、配置双系统遥操作 YAML，并通过 `lerobot-teleoperate` 与 Rerun / Foxglove 验证实时主从同步。

> **环境前提**：若需要开启 `--display_data=true`（Rerun 实时画面与关节曲线），请确认初始化环境时已包含 `core_scripts`（Windows 运行 `setup_win.ps1 -WithCoreScripts`，Ubuntu 运行 `setup_ubuntu.sh --with-core-scripts`）。

---

## 1. 第一步：探测并验证摄像头（`lerobot-find-cameras`）

插上外接摄像头（常见方案为桌面全局视角 `front` + 机械臂腕部视角 `wrist`）后，执行：

```bash
# 探测全部 OpenCV 摄像头，并抓取测试图像到 outputs/captured_images/
lerobot-find-cameras opencv

# 如需使用 Intel RealSense 深度相机
lerobot-find-cameras realsense

# 可选参数：自定义抓拍目录、预热秒数与录制时长
lerobot-find-cameras opencv --output-dir outputs/captured_images --warmup-s 1 --record-time-s 2.0
```

命令执行后会在终端打印每个识别到的摄像头 ID / 设备路径、默认分辨率与 FPS，并在 `outputs/captured_images/` 保存抓拍图片。**请打开抓拍图确认以下三点**：
1. 哪个索引对应 `front`（第三视角全局画面），哪个索引对应 `wrist`（夹爪腕部画面）；
2. 画面是否倒置（若腕部相机倒装，记录需在配置中设置 `rotation: 180`）；
3. 画面是否顺畅（若多路 USB 相机同时接入出现黑屏或掉帧，在配置中开启 `fourcc: MJPG`）。

---

## 2. 第二步：配置 `arm101` 遥操作模板（YAML）

工作区已在 `configs/robots/` 下为双系统分别提供对齐 `lerobot v0.7.1 TeleoperateConfig` 的配置模板：
- Windows：[`configs/robots/arm101_win.example.yaml`](../../configs/robots/arm101_win.example.yaml)
- Ubuntu：[`configs/robots/arm101_ubuntu.example.yaml`](../../configs/robots/arm101_ubuntu.example.yaml)

复制为本地配置文件（`configs/robots/arm101_win.yaml` 或 `arm101_ubuntu.yaml`）后，重点核对以下字段：

```yaml
robot:
  type: so101_follower
  port: COM3                        # Ubuntu 下为 /dev/ttyACM0
  id: arm101_follower_01            # 必须与 lerobot-calibrate 时一致
  calibration_dir: data/calibration/robots/so_follower
  disable_torque_on_disconnect: true
  # max_relative_target: 10.0       # 初次联调建议取消注释以限制单步最大角度增量，防止猛甩
  use_degrees: true
  cameras:
    front:
      type: opencv
      index_or_path: 0              # Ubuntu V4L2 常见为 0 / /dev/video0
      width: 640
      height: 480
      fps: 30
      color_mode: rgb
      rotation: 0                   # 若倒装改为 180
      warmup_s: 1
      # fourcc: MJPG                # 多路相机高分辨率/高帧率强烈建议开启
      # backend: 700                # Windows 推荐 700 (DSHOW)；Ubuntu 推荐 200 (V4L2)
    wrist:
      type: opencv
      index_or_path: 1              # Ubuntu V4L2 常见为偶数节点 2 / /dev/video2
      width: 640
      height: 480
      fps: 30
      color_mode: rgb
      rotation: 0
      warmup_s: 1

teleop:
  type: so101_leader
  port: COM4                        # Ubuntu 下为 /dev/ttyACM1
  id: arm101_leader_01
  calibration_dir: data/calibration/teleoperators/so_leader
  use_degrees: true

fps: 60
display_data: true
display_mode: rerun                 # "rerun" 或 "foxglove"
display_compressed_images: false
```

---

## 3. 第三步：启动实时遥操作（`lerobot-teleoperate`）

### 方式一：通过工作区 YAML 启动（推荐，可复现且参数整洁）

```powershell
# Windows
lerobot-teleoperate --config_path=configs/robots/arm101_win.yaml
```

```bash
# Ubuntu
lerobot-teleoperate --config_path=configs/robots/arm101_ubuntu.yaml
```

也可在 `--config_path` 基础上临时覆盖任意参数，例如限制限时试跑 30 秒或关闭可视化：

```bash
lerobot-teleoperate --config_path=configs/robots/arm101_win.yaml --teleop_time_s=30 --display_data=false
```

### 方式二：纯 CLI 内联传参（适合无配置文件快速验证）

**无摄像头最小主从同步测试：**

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --teleop.type=so101_leader \
  --teleop.port=COM4 \
  --teleop.id=arm101_leader_01 \
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader \
  --fps=60
```

**带双摄像头与 Rerun 可视化的内联启动：**

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=COM3 \
  --robot.id=arm101_follower_01 \
  --robot.calibration_dir=data/calibration/robots/so_follower \
  --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30} }" \
  --teleop.type=so101_leader \
  --teleop.port=COM4 \
  --teleop.id=arm101_leader_01 \
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader \
  --fps=60 \
  --display_data=true \
  --display_mode=rerun
```

> 如需使用 Foxglove Studio 查看遥测流，将参数改为 `--display_mode=foxglove`（默认监听 WebSocket）。

---

## 4. Windows 与 Ubuntu 遥操作一致性与常见故障排查

| 现象 | 常见原因 | 解决方案 |
|---|---|---|
| 启动时从臂突然猛冲或抖动剧烈 | 1) 主从臂起始姿态相差过大；2) 未启用单步限幅；3) 标定 ID 不匹配 | 启动前先把主从臂摆到接近的初始姿态；在 YAML 中取消注释 `max_relative_target: 10.0`；确认 `id` 与 `calibration_dir` 对应同一台机械臂 |
| Windows 打开相机极慢或报错 | OpenCV 默认 `MSMF`（1400）在某些 USB 相机初始化卡顿 | 在配置中设置 `backend: 700`（DirectShow）并开启 `fourcc: MJPG` |
| Ubuntu 相机索引错位或报打不开 | V4L2 一个摄像头占两个节点（如 `/dev/video0` 和 `/dev/video1`），且仅偶数节点输出图像 | 改用偶数索引 `0` / `2`，或直接填稳定路径 `index_or_path: /dev/v4l/by-id/...`，并设置 `backend: 200` |
| 多路相机插在同一 USB Hub 掉帧/报错 | 原始 YUYV 未压缩格式耗尽 USB 2.0 带宽 | 两路相机均配置 `fourcc: MJPG`、分辨率 `640x480`、`fps: 30`，或分插到主机不同 USB 根控制器端口 |
| 报错 `ModuleNotFoundError: No module named 'rerun'` | 安装虚拟环境时未带 `-WithCoreScripts` / `--with-core-scripts` | 先设为 `--display_data=false` 验证控制，或按 [`dual_os_setup.md`](../01_environment/dual_os_setup.md) 补装 `core_scripts` |
| 退出后端口仍被占用 | 异常强杀进程导致串口句柄未释放 | 重新插拔 USB 线；若通过 `app/` **Arm101 Studio** 启动，由 `ProcessRegistry` 自动管理子进程关闭 |

下一步：前往 [`03_teaching_and_demo.md`](03_teaching_and_demo.md) 学习 **演示教学 / 示教规范、开环回放验证与策略部署**。
