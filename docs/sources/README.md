# 参考来源与权威引用注册表（Sources & Citation Registry）

本文档集中登记本工作区教程、配置与软件调研所依据的 **GitHub 仓库、Hugging Face 官方文档、Seeed Studio Wiki、arXiv 论文与同济子豪兄飞书 Wiki**，记录其版本、许可证、本地映射路径及实测可访问状态，确保多人协作时所有引用可追溯、无死链。

---

## 1. GitHub 上游仓库清单

| 仓库 | 在线地址 | 本地克隆目录 / 状态 | 当前验证版本 / Commit | 许可证 | 角色与备注 |
|---|---|---|---|---|---|
| **TUST-iMaker/lerobot** | <https://github.com/TUST-iMaker/lerobot> | 工作区根目录 `.` | `1112f1c`（`main`/`aguiya`/`yyzmiao`） | 未在仓库声明 | **当前仅为 16 字节 starter 占位仓库**（仅含 `README.md`：`# imaker-lerobot`），作为规范工作区根仓库锚点，运行时必须搭配上游 `huggingface/lerobot` |
| **huggingface/lerobot** | <https://github.com/huggingface/lerobot> | [`repos/lerobot`](../../repos/lerobot/) | `v0.7.1`（`b13b952`） | `Apache-2.0` | **核心运行时与训练框架**（Python `>=3.12`，22 个 `lerobot-*` CLI，含 SO-101、`ACT`、`SmolVLA`、`Pi0`/`Pi0-FAST`/`Pi0.5`） |
| **TheRobotStudio/SO-ARM100** | <https://github.com/TheRobotStudio/SO-ARM100> | [`repos/SO-ARM100`](../../repos/SO-ARM100/) | `141aea7` | `Apache-2.0` | 官方 SO-ARM100 / SO-ARM101 机械 BOM、STL/STEP 3D 打印件、`Simulation/SO101/*.urdf` 与齿轮比规格 |
| **Physical-Intelligence/openpi** | <https://github.com/Physical-Intelligence/openpi> | [`repos/openpi`](../../repos/openpi/) | `0.1.0`（`215abfb`） | `Apache-2.0` | Physical Intelligence 官方 JAX/PyTorch VLA 仓库（支持 `pi0` / `pi0-FAST` / `pi0.5`，**仅支持 Ubuntu 22.04 + NVIDIA GPU**） |
| **huggingface/leLab** | <https://github.com/huggingface/leLab> | [`repos/leLab`](../../repos/leLab/) | `0.1.0`（`1894ab5`） | `Apache-2.0` | Hugging Face 官方 SO-ARM101 Web UI（FastAPI + React/Vite，固定依赖 `lerobot@v0.7.0`，需隔离安装） |
| **Nori-Robotics/feetech-motorlab** | <https://github.com/Nori-Robotics/feetech-motorlab> | [`repos/feetech-motorlab`](../../repos/feetech-motorlab/) | `2d8fe27` | `MIT` | 跨平台飞特 STS3215 舵机调试 Web 面板（FastAPI + WebSockets，独立于 LeRobot，支持 `--simulate`） |
| **Seeed-Projects/lerobot** | <https://github.com/Seeed-Projects/lerobot> | 未重复克隆（见隔离原则） | `v0.7.1`（`main` + `DepthCameraSupport`） | `Apache-2.0` | Seeed 维护的 LeRobot `v0.7.1` 分支（额外提供 RealSense/Orbbec 深度相机分支） |
| **ioai-tech/lerobot-studio** | <https://github.com/ioai-tech/lerobot-studio> | 在线工具 / npm 包 | `v1.7.0` | `MIT` | 浏览器端 LeRobot 数据集可视化工具（<https://lerobot.studio/> 自动跳转至 <https://mylerobot.com/>） |
| **kotek-7/sts-studio** | <https://github.com/kotek-7/sts-studio> | 外部可选桌面工具 | `v0.6.1` | `MIT` | 基于 PySide6 的 STS 舵机跨平台调试工具（支持 JSON CLI 与虚拟演示模式） |
| **phospho-app/phosphobot** | <https://github.com/phospho-app/phosphobot> | 外部调研参考（未混装） | `v0.3.134`（2025-12 停更） | `MIT` | 遥操作与采集服务（含 PyBullet/Next.js 等重型依赖，已长期未更新） |
| **GetSoloTech/solo-cli** | <https://github.com/GetSoloTech/solo-cli> | 外部调研参考（未混装） | `v0.6.1` | `Apache-2.0` | CLI 脚手架（强制绑定冲突的 `solo-bot` 分支，禁止装入主虚拟环境） |

> 克隆与防重原则：上述本地克隆均由 [`configs/repos.yaml`](../../configs/repos.yaml) 声明并由 [`scripts/common/bootstrap_repos.py`](../../scripts/common/bootstrap_repos.py) 幂等拉取至 `repos/`，严禁手动在工作区散落重复克隆。

---

## 2. Hugging Face 官方文档（均实测 HTTP 200）

所有 URL 均与 [`repos/lerobot/docs/source/`](../../repos/lerobot/docs/source/) 源码及在线站点逐一核验为 `HTTP 200` 有效页面：

| 主题 | 权威在线链接 | 对应本地源码参考 |
|---|---|---|
| SO-101（arm101）官方教程 | <https://huggingface.co/docs/lerobot/so101> | `repos/lerobot/docs/source/so101.mdx` |
| 真实机械臂模仿学习全流程（含 HF Jobs） | <https://huggingface.co/docs/lerobot/il_robots> | `repos/lerobot/docs/source/il_robots.mdx` |
| 安装指南（含 Trackio 迁移对照表） | <https://huggingface.co/docs/lerobot/installation> | `repos/lerobot/docs/source/installation.mdx` |
| 硬件接入通用指南（Bring Your Own Hardware） | <https://huggingface.co/docs/lerobot/integrate_hardware> | `repos/lerobot/docs/source/integrate_hardware.mdx` |
| 训练算力硬件指南（含 HF Jobs `--job.target`） | <https://huggingface.co/docs/lerobot/hardware_guide> | `repos/lerobot/docs/source/hardware_guide.mdx` |
| 多卡分布式训练（`torchrun`） | <https://huggingface.co/docs/lerobot/multi_gpu_training> | `repos/lerobot/docs/source/multi_gpu_training.mdx` |
| PEFT / LoRA 微调指南 | <https://huggingface.co/docs/lerobot/peft_training> | `repos/lerobot/docs/source/peft_training.mdx` |
| Hugging Face Jobs 官方文档 | <https://huggingface.co/docs/huggingface_hub/en/guides/jobs> | — |
| LeRobotDataset v3.0 数据格式规范 | <https://huggingface.co/docs/lerobot/lerobot-dataset-v3> | `repos/lerobot/docs/source/lerobot-dataset-v3.mdx` |
| ACT 策略文档 | <https://huggingface.co/docs/lerobot/act> | `repos/lerobot/docs/source/act.mdx` |
| SmolVLA 策略文档 | <https://huggingface.co/docs/lerobot/smolvla> | `repos/lerobot/docs/source/smolvla.mdx` |
| Pi0 策略文档 | <https://huggingface.co/docs/lerobot/pi0> | `repos/lerobot/docs/source/pi0.mdx` |
| Pi0.5 策略文档 | <https://huggingface.co/docs/lerobot/pi05> | `repos/lerobot/docs/source/pi05.mdx` |
| Pi0-FAST 策略文档 | <https://huggingface.co/docs/lerobot/pi0fast> | `repos/lerobot/docs/source/pi0fast.mdx` |
| LeLab 官方文档 | <https://huggingface.co/docs/lerobot/lelab> | `repos/lerobot/docs/source/lelab.mdx` |

> **避坑提示（v0.7.x 已失效或不存在的旧别名）**：`/docs/lerobot/hf_jobs` 与 `/docs/lerobot/bring_your_own_hardware` 在 v0.7.x 站点仅返回 *"doesn't exist in v0.7.0"* 占位提示。请始终使用上表中的 `/hardware_guide`、`/il_robots`、`/integrate_hardware` 与 `/docs/huggingface_hub/en/guides/jobs`。

---

## 3. Seeed Studio Wiki 与软件资料（实测 HTTP 200）

| 页面名称 | 有效链接 | 说明与避坑 |
|---|---|---|
| **SO-ARM100 & SO-ARM101 新版入门指南** | <https://wiki.seeedstudio.com/lerobot_so100m_new/> | 涵盖 `Seeed-Projects/lerobot`、`fourcc: "MJPG"`、RTX 50 系列 `--dataset.video_backend=pyav`、SenseCraft 与深度相机说明 |
| **SO-ARM100 经典文档** | <https://wiki.seeedstudio.com/lerobot_so100m/> | 早期经典参考页（HTTP 200） |
| **SenseCraft Robotics SO-ARM101 指南** | <https://wiki.seeedstudio.com/sensecraft_robotics_so_arm101/> | Seeed 无代码桌面客户端使用指南（仅支持 Windows / macOS） |
| **SenseCraft 下载门户** | <https://sensecraft.seeed.cc/en/download> | SenseCraft 客户端官方下载入口 |

> **避坑提示**：网络上常见的 `https://wiki.seeedstudio.com/so_arm101/` 实测返回 **HTTP 404**，请使用 `lerobot_so100m_new/` 或 `sensecraft_robotics_so_arm101/`。

---

## 4. arXiv 学术论文与 Physical Intelligence 资料

| 资料 | 链接 | 状态与核心核查结论 |
|---|---|---|
| **$\pi_{0.7}$ 论文（arXiv:2604.15483v2）** | <https://arxiv.org/abs/2604.15483> | 2026-04 提交与修订（*$\pi_{0.7}$: a Steerable Generalist Robotic Foundation Model with Emergent Capabilities*），HTML 版可正常读取完整架构（约 5B 参数：Gemma3-4B VLM + 400M MEM 视频编码器 + 860M 流匹配动作专家） |
| **$\pi_{0.7}$ 项目主页** | <https://pi.website/pi07> | 直接程序化 HTTP 抓取会触发 **HTTP 429 Too Many Requests** 限流；`repos/lerobot/docs/source/annotation_pipeline.mdx` 引用该链接用于说明 `lerobot-annotate` 的子任务标注范式。**截至当前核查，`pi0.7` 与 `pi*0.6` 均未在 `openpi`、`lerobot` 或 HF Hub 开源代码与权重** |

---

## 5. 指定飞书 Wiki（`同济子豪兄带你玩LeRobot具身智能`）

- **主目录链接**：<https://zihao-ai.feishu.cn/wiki/TS6swApHbinx01kHDi5cf5n5n8c>
- **公开标识与作者**：飞书标注为`互联网公开`（组织：上海豪兄教育科技有限公司；作者：同济子豪兄、朱耀晖（矽递科技 Seeed Studio）、刘越（同济大学））。

### 5.1 直接访问异常记录与处理方式
- **非浏览器 HTTP 直接访问结果（失败）**：使用 `urllib` / `curl` 直接请求主链接会遭遇 **`HTTP 302` 无限重定向循环**，跳转至 `https://login.feishu.cn/accounts/trap?app_id=2&query_scope=all&...&with_guest=1` -> `accounts.feishu.cn/accounts/page/login`。
- **实际处理方式（成功）**：通过无头浏览器（Playwright Chromium）加载公共页面，成功渲染完整目录树并提取全部核心子节点链接，供团队成员在浏览器中按需查阅：

| 章节节点 | 飞书 Wiki 验证链接 | 关键参考内容 |
|---|---|---|
| **目录（根节点）** | <https://zihao-ai.feishu.cn/wiki/TS6swApHbinx01kHDi5cf5n5n8c> | 全课程章节目录 |
| **第二步：安装环境** | <https://zihao-ai.feishu.cn/wiki/DAbNwM5OPiuINgkF6kRcd3B8n6d> | Ubuntu / Mac / Windows 分平台安装指引 |
| └─ **Windows 电脑** | <https://zihao-ai.feishu.cn/wiki/UJp0wZsXNisA6MkDcPhcEzy1nfh> | Miniconda、清华源、`ffmpeg` 安装流程 |
| **第五步：遥操作** | <https://zihao-ai.feishu.cn/wiki/SpKMwmKbTihjtbk3TNpcqJMlnTg> | 各系统串口遥操演示 |
| **第七步：采集数据集-真机** | <https://zihao-ai.feishu.cn/wiki/NHcewDnbfilHiyk4yZMcbQRVnPd> | 真机示教录制、握手200任务、回放与注意事项 |
| **第八步：训练模型** | <https://zihao-ai.feishu.cn/wiki/CCLZw7LGqiUboWku9NRcWZJenyx> | 支持算法总览、ACT/SmolVLA/Pi0/Pi0.5/Diffusion/WALL-OSS |
| └─ **云 GPU 训练环境配置** | <https://zihao-ai.feishu.cn/wiki/PFCKw8kCfi9MrRkzy4zcIRRvnCf> | Featurize 租卡、`featurize dataset download <uuid>` 与代理提示 |
| └─ **训练命令行-pi0** | <https://zihao-ai.feishu.cn/wiki/NsYCwnhEUis0qrkLWrsccqORnle> | `TommyZihao/lerobot_zihao_dataset_shake_hands` 微调参数示例 |
| └─ **训练命令行-pi0.5** | <https://zihao-ai.feishu.cn/wiki/IJkiwXNxBidDtgkdwOmcfxOdnEf> | `pi0.5` 云端训练参考 |
| **第九步：模型推理** | <https://zihao-ai.feishu.cn/wiki/U1jcwvDJ5ipjLIkQVQzcNKpcnvJ> | 多策略真机部署及 DGX Spark / RDK S100 推理 |
| **蚂蚁灵波 LingBot-VLA 后训练全流程** | <https://zihao-ai.feishu.cn/wiki/ShOLwj7XmiR8FzkL2bAcbVmmndf> | 扩展 VLA 后训练参考 |

### 5.2 历史教程与 `v0.7.1` 版本漂移勘误表

阅读飞书 Wiki 或早期 `TommyZihao/lerobot`（`v0.4.3`）笔记时，在本工作区 `v0.7.1` 下请按以下对照替换命令：

| 旧教程 / 飞书历史写法 | 本工作区 `v0.7.1` 正确写法 | 原因说明 |
|---|---|---|
| `pip install -e ".[pi0]"` | `uv pip install -e "repos/lerobot[pi]"` | `v0.7.1` 中 extra 名称为 `pi`（无 `pi0`） |
| `--wandb.enable=true` / `--wandb.project=...` | `--trackio.enable=true` / `--trackio.project=...` / `--trackio.space_id=...` | `v0.7.1` 已用 `trackio` 取代 `wandb` 命令行参数（仅旧 checkpoint 自动迁移） |
| `lerobot-record --policy.path=...` | `lerobot-rollout --strategy.type=base --policy.path=...` | `v0.7.1` 中 `lerobot-record` 禁止传入 `policy`，推理与 HIL 统一由 `lerobot-rollout` 承担 |
| `huggingface-cli login` | `hf auth login` | 新版 `huggingface_hub` 统一使用 `hf` CLI |
| Conda 混装与手动随处 `git clone` | `uv` 隔离环境 + `scripts/common/bootstrap_repos.py` | 避免依赖冲突与仓库重复克隆 |
