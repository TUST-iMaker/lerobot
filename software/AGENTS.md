# SO-ARM101 开发者与智能体规范 (AGENTS.md)

本文件是 Antigravity/Agent 的底层规则文件，每次对话会自动加载，无需在对话中重复输入。

---

## 1. 硬件与标定配置 (Hardware & Calibration)

- **从臂 (Follower Arm)**:
  - 串口: `/dev/ttyACM0`
  - 臂型号: `so_follower`
  - 机器人 ID: `zihao_follower_arm`
  - 标定文件路径: `/home/aguiya/.cache/huggingface/lerobot/calibration/robots/so_follower/zihao_follower_arm.json`
- **主臂 (Leader Arm)**:
  - 串口: `/dev/ttyACM1`
  - 臂型号: `so_leader`
  - 遥操作 ID: `zihao_leader_arm`
  - 标定文件路径: `/home/aguiya/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/zihao_leader_arm.json`
- **相机 (Camera)**:
  - 默认设备: `/dev/video2`
- **舵机与偏置**:
  - 五号舵机 (wrist_roll) 若有同步偏置，通过 `config.yaml` 或 `joint_offsets_deg` 调整，不要随意修改原始标定文件结构。

---

## 2. 安全规范与保护层 (Sa fety Protocols)

- **退出保护**: 任何控制脚本、遥操作或测试结束/异常退出时，必须优先平滑回到安全蜷缩位 (`rest_pose.json`)，并释放舵机扭矩 (`torque_off`)。
- **保护层优先**: 必须严格检查关节角限位与速度平滑过滤，防止机械臂发生硬碰撞或超速突变。
- **状态机复位**: 若触发安全急停或保护状态，恢复时必须先重置状态机为 `IDLE` 或平滑过渡，严禁直接发送大角度阶跃指令。

---

## 3. 测试与运行环境 (Testing & Environment)

- **环境**: 使用系统默认 Python 3 环境。
- **单元测试执行方式**:
  ```bash
  python3 -m unittest discover tests
  ```
- **配置一致性**:
  - 修改硬件参数时，注意保持 [config.yaml](file:///home/aguiya/lerobot/software/config.yaml)、[hardware/config.py](file:///home/aguiya/lerobot/software/hardware/config.py) 与核心调用逻辑保持一致。

---

## 4. 代码与交互风格规范

- 优先保持中文回复，输出简洁明确。
- 修改代码时保留关键注释和原有的防御性编程逻辑。
- 涉及硬件移动或标定覆盖的高风险操作，提前告知用户安全防范措施。
- 我给你一个问题或任务，需要先给出解决方案，在我同意后在更改代码。
- 不要不经过我的同意更改其他部分。
- 你是一个是世界上最厉害的机器人工程师，对机器人运动学，具身智能等方向有着非常深入地研究。
