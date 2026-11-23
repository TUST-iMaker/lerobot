# SO-ARM101 官方与社区现成工具速查表

> 一页看完：有哪些官方/社区现成工具、网址在哪、各自能干什么、怎么直接跑（不造轮子）。

---

## 一、你最需要的「连续示教 → 线性回放」官方现成工具

**Hugging Face LeRobot 官方自带**，不需要自己造轮子：
- **`lerobot-record`**：你用手操作主臂（`COM11`），从臂（`COM12`）实时跟随，后台按 **30Hz 连续记录整段运动轨迹**（支持按 `→` 随时结束本段、`←` 重录、`ESC` 停止）。
- **`lerobot-replay`**：读取刚才录好的某一段，让从臂（`COM12`）**一帧不落地线性回放整段连续动作**（回放时只动从臂，不用主臂）。

### 1. 本机补装 1 个缺的包（当前环境只缺 `datasets`）
```powershell
envs\win\lerobot\Scripts\python.exe -m pip install -e "repos/lerobot[dataset]"
```

### 2. 直接可用的录制 & 线性回放命令（已填好你的 COM11 / COM12 与标定目录）

**开始连续示教录制（录 1 段本地离线保存，不上传云端）：**
```powershell
envs\win\lerobot\Scripts\lerobot-record.exe `
  --robot.type=so101_follower --robot.port=COM12 --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --teleop.type=so101_leader --teleop.port=COM11 --teleop.id=arm101_leader_01 `
  --teleop.calibration_dir=data/calibration/teleoperators/so_leader `
  --dataset.repo_id=local/demo_teach `
  --dataset.root=data/datasets/demo_teach `
  --dataset.single_task="linear teaching demo" `
  --dataset.num_episodes=1 --dataset.episode_time_s=30 --dataset.reset_time_s=5 `
  --dataset.push_to_hub=false
```
> 录制期间键盘操作：`→`（右方向键）提前结束当前段并保存；`←` 废弃重录；`ESC` 退出。

**线性回放刚才录的第 0 段动作（从臂原样复现）：**
```powershell
envs\win\lerobot\Scripts\lerobot-replay.exe `
  --robot.type=so101_follower --robot.port=COM12 --robot.id=arm101_follower_01 `
  --robot.calibration_dir=data/calibration/robots/so_follower `
  --dataset.repo_id=local/demo_teach `
  --dataset.root=data/datasets/demo_teach `
  --dataset.episode=0
```

---

## 二、官方与主流社区资源地图（一眼找对地方）

| 来源 | 官方路径 / 仓库 | 核心功能 | 能做哪些操作 / 展示了什么 |
| :--- | :--- | :--- | :--- |
| **Hugging Face LeRobot 官方** | [github.com/huggingface/lerobot](https://github.com/huggingface/lerobot)<br>[huggingface.co/docs/lerobot](https://huggingface.co/docs/lerobot) | 最权威的底层驱动与全流程 CLI 工具链 | 找串口、配舵机 ID、标定、主从遥操作、**连续轨迹录制 (`record`)**、**线性回放 (`replay`)**、数据可视化/剪辑、ACT/Diffusion/SmolVLA/π0 训练与真机推理 |
| **TheRobotStudio (SO-ARM100/101 硬件官方)** | [github.com/TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | 机械臂硬件本体与官方仿真模型 | 3D 打印件 STL、物料清单 BOM、装配说明、官方 `so101_new_calib.urdf` 运动学模型 |
| **矽递科技 Seeed Studio 官方** | [wiki.seeedstudio.com/lerobot_so100m](https://wiki.seeedstudio.com/lerobot_so100m/)<br>**SenseCraft Robotics**（免代码客户端） | 面向 SO-ARM100/101 的保姆级图文文档 + 免代码图形软件 | 硬件排错（供电/串口/中位校准）、Windows/Ubuntu/Jetson 全套命令、**SenseCraft** 支持不敲代码点按钮完成校准、遥操作、采集与云端训练 |
| **同济子豪兄 (TommyZihao)** | [github.com/TommyZihao](https://github.com/TommyZihao)<br>[github.com/TommyZihao/vlm_arm](https://github.com/TommyZihao/vlm_arm)<br>[huggingface.co/TommyZihao](https://huggingface.co/TommyZihao) | 国内最全的中文实战教程、多模态机械臂项目与公开真机数据集 | B 站「SO101 教程一站通」与飞书文档（舵机组装、中位校准、抓物示教、训练部署）；`vlm_arm` 展示大模型 + 视觉 + 机械臂联动；HF 主页提供现成 `so_follower` 真机数据集 |
| **Datawhale 开源社区** | [github.com/datawhalechina/dive-into-embodied-ai](https://github.com/datawhalechina/dive-into-embodied-ai) | 中文开源《从零搭建具身智能机器人》 | 专门章节讲 `SO-101 + LeRobot` 真机部署与模仿学习实战，中文踩坑记录全 |

---

## 三、你电脑里已经装好的官方 `lerobot-*` 命令速查

路径都在 `envs\win\lerobot\Scripts\` 下，直接按场景用：

| 分类 | 命令 | 一句话用途 | 什么时候用 |
| :--- | :--- | :--- | :--- |
| **硬件准备** | `lerobot-find-port` | 拔插识别主/从臂对应 `COM` 口 | 换 USB 口、不确定哪个是主/从臂 |
| | `lerobot-find-cameras` | 扫描可用摄像头索引与分辨率 | 接新相机、查相机编号 |
| | `lerobot-setup-motors` | 给新买的 STS3215 舵机逐个写 ID（1~6） | 刚装好新臂 / 换了新舵机 |
| | `lerobot-calibrate` | 标定机械臂中位（0°）与各轴物理限位 | 首次使用、3D 姿态和真机对不上时 |
| | `lerobot-find-joint-limits` | 读取并确认各关节软硬限位 | 排查关节卡死或角度越界 |
| **操控与示教** | `lerobot-teleoperate` | 纯主从遥操作（动主臂 → 从臂实时跟） | 试手感、检查主从同步是否顺畅 |
| | **`lerobot-record`** | **主从连续示教录制（30Hz 轨迹 + 可选相机画面）** | **录制连续动作 / 采集训练数据** |
| | **`lerobot-replay`** | **从臂线性回放已录制的整段连续轨迹** | **一键复现刚才教过的整段连贯动作** |
| **数据管理** | `lerobot-dataset-viz` | Rerun 可视化回看录好的视频 + 6 轴曲线 | 检查哪一段录得好、有没有抖动 |
| | `lerobot-edit-dataset` | 删除/裁剪/合并某几段录好的数据 | 剔除失误的录制片段 |
| **AI 训练推理** | `lerobot-train` | 用录好的数据集训练 ACT / SmolVLA / π0 策略 | 离线训练自主抓取模型（需 GPU） |
| | `lerobot-eval` / `lerobot-policy-server` | 加载训练好的模型让真机自主执行 | 模型训练完上真机测试 |

---

## 四、一句话建议：怎么配合现在的 Studio 用最省事？

1. **想立刻体验“主臂教一遍 → 从臂线性回放整段”**：先跑上面「一、1」补装 `datasets`，然后直接跑 `lerobot-record` + `lerobot-replay`（完全用官方现成工具）。
2. **如果你希望把 `lerobot-record` + `lerobot-replay` 直接做成 Studio 第 4 页的「开始连续示教 / 停止 / 线性回放」按钮**：告诉我一声，我可以直接在后端接官方 `lerobot-record` 和 `lerobot-replay`，不自己重造轮子。
