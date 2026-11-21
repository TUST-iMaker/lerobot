# `logs/` — 运行与调试日志目录

本目录用于保存本地 Studio 后端、硬件进程输出、环境自检导出报告与训练运行日志。除本 `README.md` 与 `.gitkeep` 外，**所有 `.log` 与运行日志文件均被 `.gitignore` 忽略**。

## 约定

- 本地 Arm101 Studio 启动的子进程日志（如 `lerobot-teleoperate`、`lerobot-record`、`feetech-motorlab`）按时间或工作流名写入本目录。
- 提交 Issue 或向团队排查硬件/环境问题时，可附带 `python scripts/common/check_env.py --json` 输出与对应日志片段。
