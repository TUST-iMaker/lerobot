#!/usr/bin/env bash
# SO-ARM101 机械臂与 USB 摄像头免 sudo 访问权限一键配置脚本
set -e

echo "============================================================"
echo " ⚙️  配置 SO-ARM101 硬件免 sudo 读写权限 (udev 规则)"
echo "============================================================"

# 1. 将当前用户加入 dialout 与 video 组
echo "[*] 将用户 $(whoami) 加入 dialout, video, plugdev 用户组..."
sudo usermod -a -G dialout,video,plugdev "$USER" || true

# 2. 写入 udev 规则文件 (与 scripts/ubuntu/setup_ubuntu.sh 统一命名)
RULE_FILE="/etc/udev/rules.d/99-lerobot-arms.rules"
echo "[*] 正在写入硬件规则到 ${RULE_FILE} ..."

sudo tee "$RULE_FILE" > /dev/null << 'EOF'
# SO-ARM101 串口适配器免密读写权限 (CH340 / CP2102 / FTDI / Feetech)
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"

# USB 摄像头视频流免密读取权限
KERNEL=="video*", MODE="0666", GROUP="video"
EOF

# 3. 重新加载规则并立即触发
echo "[*] 重新加载 udev 规则并触发应用..."
sudo udevadm control --reload-rules || true
sudo udevadm trigger || true

echo ""
echo "============================================================"
echo " [✓] 配置成功！重新拔插 USB 机械臂与摄像头后即可直接免密使用。"
echo "============================================================"
