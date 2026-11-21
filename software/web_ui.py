#!/usr/bin/env python3
"""SO-ARM101 具身机器人轻量极速原型控制中心 (Web UI).

基于 Python 原生 http.server 与现代 Web 技术栈构建，零第三方 Web 依赖，开箱即用。
核心功能矩阵:
  1. 腕部/桌面摄像头实时低延迟 MJPEG 视频流;
  2. 三大核心控制模式无缝切换 (演示动线 Demo / 主从遥操作 Teleop / 网页与键盘控制 Keyboard);
  3. 回蜷缩位置自动平滑停稳并安全释放扭矩 (防发热/防蜂鸣/节能休眠);
  4. 6 轴关节物理角度与硬件状态毫秒级遥测看板;
  5. 100vh 满屏视口全自适应响应式驾驶舱布局 (无滚动条、零底部留白);
  6. 软件急停闭锁 (E-Stop) 与安全恢复门禁.
"""

from __future__ import annotations

import argparse
import io
import json
import logging
import math
import os
import socketserver
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

# 确保软件根目录加入模块搜索路径
_software_root = Path(__file__).resolve().parent
_lerobot_root = _software_root.parent
if str(_software_root) not in sys.path:
    sys.path.insert(0, str(_software_root))
if str(_lerobot_root / "src") not in sys.path:
    sys.path.insert(0, str(_lerobot_root / "src"))

from core.action_api import ActionAPI
from core.config_loader import get_config
from core.data_types import (
    ActionType,
    ControlSource,
    JointUnit,
    RobotAction,
    SO101_JOINT_NAMES,
    deg_to_rad,
    rad_to_deg,
)
from hardware.config import RobotSystemConfig
from teleop.demo_routine import DemoRoutineRunner

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

logger = logging.getLogger("web_ui")

# 全局单例控制实例与锁
_API_LOCK = threading.Lock()
_GLOBAL_API: ActionAPI | None = None
_GLOBAL_RUNNER: DemoRoutineRunner | None = None
_IS_BUSY = False
_BUSY_ACTION_NAME = ""
_LAST_LOGS: list[str] = []

# 工作模式: "demo" | "teleop" | "keyboard"
_ACTIVE_MODE = "demo"
_TELEOP_RUNNING = False
_TELEOP_THREAD: threading.Thread | None = None


def add_web_log(msg: str) -> None:
    """向前端控制台追加日志."""
    timestamp = time.strftime("%H:%M:%S")
    entry = f"[{timestamp}] {msg}"
    _LAST_LOGS.append(entry)
    if len(_LAST_LOGS) > 50:
        _LAST_LOGS.pop(0)


class MockVideoFeed:
    """当无物理摄像头或处于 Mock 模式时的高保真合成视觉画面."""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        self.width = width
        self.height = height
        self._frame_count = 0

    def get_jpeg_frame(self) -> bytes:
        self._frame_count += 1
        t = time.time()

        if cv2 is not None and np is not None:
            img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            img[:] = (18, 22, 28)  # 深灰底色

            # 绘制网格与坐标线
            for x in range(0, self.width, 40):
                cv2.line(img, (x, 0), (x, self.height), (28, 35, 45), 1)
            for y in range(0, self.height, 40):
                cv2.line(img, (0, y), (self.width, y), (28, 35, 45), 1)

            # 中心动态十字瞄准线
            cx, cy = self.width // 2, self.height // 2
            cv2.circle(img, (cx, cy), 60, (0, 180, 255), 1)
            cv2.line(img, (cx - 70, cy), (cx + 70, cy), (0, 180, 255), 1)
            cv2.line(img, (cx, cy - 70), (cx, cy + 70), (0, 180, 255), 1)

            # 动态模拟扫描线
            scan_y = int((math.sin(t * 3.0) * 0.5 + 0.5) * self.height)
            cv2.line(img, (0, scan_y), (self.width, scan_y), (0, 255, 180), 1)

            # 叠加文字 HUD
            time_str = time.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(img, "SO-ARM101 SYNTHETIC CAMERA STREAM", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1)
            cv2.putText(img, f"TIME: {time_str} | FPS: ~30.0", (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 180, 200), 1)
            cv2.putText(img, "[SIMULATION MODE / CAMERA STANDBY]", (20, self.height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 120), 1)

            _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return buf.tobytes()
        else:
            return b""


class CameraStreamer:
    """相机视频采集与流媒体管理器."""

    def __init__(self, camera_device: str | int = 2, width: int = 640, height: int = 480, fps: int = 30) -> None:
        self.device = camera_device
        self.width = width
        self.height = height
        self.fps = fps
        self._cap: Any = None
        self._lock = threading.Lock()
        self._mock_feed = MockVideoFeed(width, height)
        self._is_real_camera = False

    def start(self) -> None:
        if cv2 is None:
            logger.info("未安装 OpenCV，切换至仿真相机画面")
            return

        idx = self.device
        if isinstance(idx, str) and idx.startswith("/dev/video"):
            try:
                idx = int(idx.replace("/dev/video", ""))
            except Exception:
                pass

        try:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                self._cap = cap
                self._is_real_camera = True
                logger.info("物理相机 (%s) 连接成功", self.device)
            else:
                logger.warning("物理相机 (%s) 打开失败，将展示数字待机画面", self.device)
        except Exception as exc:
            logger.warning("连接相机产生异常: %s", exc)

    def get_jpeg(self) -> bytes:
        with self._lock:
            if self._is_real_camera and self._cap is not None:
                ret, frame = self._cap.read()
                if ret and frame is not None:
                    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    return buf.tobytes()
            return self._mock_feed.get_jpeg_frame()

    def close(self) -> None:
        with self._lock:
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None


_CAMERA_STREAMER: CameraStreamer | None = None


# 嵌入式超现代 100vh 满屏自适应驾驶舱页面
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>SO-ARM101 具身机械臂控制驾驶舱</title>
  <style>
    :root {
      --bg: #0a0d14;
      --card-bg: #121721;
      --card-border: #1f293d;
      --accent: #00e5ff;
      --accent-glow: rgba(0, 229, 255, 0.25);
      --success: #00e676;
      --danger: #ff1744;
      --warning: #ffb300;
      --purple: #b388ff;
      --text: #e6edf3;
      --text-dim: #8b949e;
      --font-mono: "Fira Code", Menlo, Monaco, Consolas, monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body {
      height: 100vh;
      width: 100vw;
      background: var(--bg);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      overflow: hidden;
      display: flex;
      flex-direction: column;
      padding: 10px;
    }

    /* 顶部紧凑状态栏 */
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 8px 16px;
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 10px;
      margin-bottom: 10px;
      flex-shrink: 0;
      box-shadow: 0 4px 15px rgba(0,0,0,0.4);
    }
    .header-left {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .header-title {
      font-size: 1.15rem;
      font-weight: 700;
      color: var(--accent);
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .mode-tabs {
      display: flex;
      background: rgba(0,0,0,0.4);
      border-radius: 8px;
      padding: 3px;
      border: 1px solid var(--card-border);
    }
    .mode-tab {
      padding: 5px 14px;
      border-radius: 6px;
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
      border: none;
      background: transparent;
      color: var(--text-dim);
      transition: all 0.2s ease;
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .mode-tab:hover { color: var(--text); }
    .mode-tab.active {
      background: var(--accent);
      color: #000;
      box-shadow: 0 0 10px var(--accent-glow);
    }
    .mode-tab.active-teleop {
      background: var(--purple);
      color: #000;
      box-shadow: 0 0 10px rgba(179,136,255,0.4);
    }
    .mode-tab.active-key {
      background: var(--success);
      color: #000;
      box-shadow: 0 0 10px rgba(0,230,118,0.4);
    }

    .header-badges {
      display: flex;
      gap: 8px;
      align-items: center;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      border-radius: 6px;
      font-size: 0.75rem;
      font-weight: 600;
      background: rgba(255,255,255,0.04);
      border: 1px solid var(--card-border);
      font-family: var(--font-mono);
    }
    .badge.green { border-color: rgba(0,230,118,0.3); color: var(--success); }
    .badge.red { border-color: rgba(255,23,68,0.3); color: var(--danger); }
    .badge.blue { border-color: rgba(0,229,255,0.3); color: var(--accent); }
    .badge.purple { border-color: rgba(179,136,255,0.3); color: var(--purple); }
    .badge.yellow { border-color: rgba(255,179,0,0.3); color: var(--warning); }
    .dot { width: 7px; height: 7px; border-radius: 50%; background: currentColor; }

    /* 主网格容器 (弹性填满视口剩余高度) */
    .dashboard-grid {
      flex: 1;
      display: grid;
      grid-template-columns: 1.15fr 0.85fr;
      gap: 10px;
      min-height: 0; /* 极其关键：允许容器收缩并填满 */
    }

    .column {
      display: flex;
      flex-direction: column;
      gap: 10px;
      min-height: 0;
      height: 100%;
    }

    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 10px;
      padding: 12px 14px;
      display: flex;
      flex-direction: column;
      box-shadow: 0 4px 15px rgba(0,0,0,0.3);
    }
    .card-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.88rem;
      font-weight: 700;
      color: var(--text);
      margin-bottom: 8px;
      padding-bottom: 6px;
      border-bottom: 1px solid rgba(255,255,255,0.06);
      flex-shrink: 0;
    }

    /* 视频卡片自适应拉伸 */
    .video-card {
      flex: 1;
      min-height: 0;
    }
    .video-wrapper {
      position: relative;
      flex: 1;
      width: 100%;
      min-height: 0;
      background: #000;
      border-radius: 8px;
      overflow: hidden;
      display: flex;
      align-items: center;
      justify-content: center;
      border: 1px solid var(--card-border);
    }
    .video-feed {
      width: 100%;
      height: 100%;
      object-fit: contain;
    }
    .video-badge {
      position: absolute;
      top: 8px;
      left: 8px;
      background: rgba(0,0,0,0.7);
      backdrop-filter: blur(4px);
      padding: 3px 8px;
      border-radius: 4px;
      font-size: 0.72rem;
      font-family: var(--font-mono);
      color: var(--success);
      display: flex;
      align-items: center;
      gap: 5px;
    }
    .rec-dot {
      width: 7px;
      height: 7px;
      background: var(--danger);
      border-radius: 50%;
      animation: pulse 1.5s infinite;
    }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }

    /* 控制面板区域 */
    .control-card {
      flex-shrink: 0;
      min-height: 220px;
    }
    .panel-view { display: none; }
    .panel-view.active { display: block; }

    /* Demo 按钮网格 */
    .btn-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .btn {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      padding: 10px 12px;
      border-radius: 6px;
      font-size: 0.85rem;
      font-weight: 600;
      border: 1px solid transparent;
      cursor: pointer;
      transition: all 0.15s ease;
      color: #fff;
    }
    .btn:hover:not(:disabled) { transform: translateY(-1px); filter: brightness(1.15); }
    .btn:active:not(:disabled) { transform: translateY(1px); }
    .btn:disabled { opacity: 0.4; cursor: not-allowed; }
    .btn-wave { background: #0066cc; border-color: #0080ff; }
    .btn-pick { background: #00875a; border-color: #00b377; }
    .btn-home { background: #4a5568; border-color: #718096; }
    .btn-rest { background: #6b46c1; border-color: #805ad5; }
    .btn-torque { background: #374151; border-color: #4b5563; font-size: 0.8rem; }
    .btn-estop {
      grid-column: span 2;
      background: linear-gradient(135deg, #d32f2f, #990000);
      border: 1px solid #ff5252;
      padding: 11px;
      font-size: 0.95rem;
      box-shadow: 0 4px 15px rgba(211,47,47,0.35);
    }
    .btn-reset {
      grid-column: span 2;
      background: #2d3748;
      border-color: #4a5568;
    }

    /* 遥操作控制面板 */
    .teleop-panel {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    .teleop-box {
      background: rgba(0,0,0,0.3);
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 10px;
      display: flex;
      justify-content: space-around;
      align-items: center;
      font-family: var(--font-mono);
      font-size: 0.8rem;
    }

    /* 键盘驾驶控制面板 */
    .keyboard-panel {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .dpad-container {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 6px;
      max-width: 280px;
      margin: 0 auto;
    }
    .dpad-btn {
      background: #1f293d;
      border: 1px solid #2d3a54;
      color: var(--text);
      padding: 8px;
      border-radius: 6px;
      font-size: 0.8rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 2px;
    }
    .dpad-btn:hover { background: #2d3a54; border-color: var(--accent); }
    .dpad-btn:active { background: var(--accent); color: #000; }
    .key-sub { font-size: 0.65rem; color: var(--accent); font-family: var(--font-mono); }

    /* 关节数据看板 */
    .joint-list {
      display: flex;
      flex-direction: column;
      gap: 7px;
    }
    .joint-item {
      display: flex;
      flex-direction: column;
      gap: 2px;
    }
    .joint-label {
      display: flex;
      justify-content: space-between;
      font-size: 0.78rem;
      font-family: var(--font-mono);
      color: var(--text-dim);
    }
    .joint-bar-bg {
      width: 100%;
      height: 6px;
      background: rgba(255,255,255,0.06);
      border-radius: 3px;
      overflow: hidden;
      position: relative;
    }
    .joint-bar-fill {
      height: 100%;
      width: 50%;
      background: var(--accent);
      border-radius: 3px;
      transition: width 0.1s ease;
    }

    /* 终端日志自适应弹性填满卡片 */
    .log-card {
      flex: 1;
      min-height: 0;
      display: flex;
      flex-direction: column;
    }
    .log-box {
      flex: 1;
      min-height: 100px;
      background: #080b0f;
      border: 1px solid var(--card-border);
      border-radius: 6px;
      padding: 8px 10px;
      font-family: var(--font-mono);
      font-size: 0.74rem;
      overflow-y: auto;
      color: #9cdcfe;
      line-height: 1.4;
    }

    .status-toast {
      padding: 6px 10px;
      border-radius: 6px;
      font-size: 0.8rem;
      margin-bottom: 8px;
      display: none;
    }
    .toast-info { background: rgba(0,229,255,0.12); border: 1px solid var(--accent); color: var(--accent); }
    .toast-err { background: rgba(255,23,68,0.15); border: 1px solid var(--danger); color: var(--danger); }
  </style>
</head>
<body>

  <!-- 顶部状态栏 -->
  <header class="header">
    <div class="header-left">
      <div class="header-title">
        <span>🦾 SO-ARM101 驾驶舱</span>
        <span style="font-size: 0.68rem; background: rgba(0,229,255,0.12); color: var(--accent); padding: 2px 6px; border-radius: 4px;">WEB V2.0</span>
      </div>

      <!-- 模式快速切换标签 -->
      <div class="mode-tabs">
        <button class="mode-tab active" id="tab-demo" onclick="switchMode('demo')">🤖 演示动线 (Demo)</button>
        <button class="mode-tab" id="tab-teleop" onclick="switchMode('teleop')">🎮 主从遥控 (Teleop)</button>
        <button class="mode-tab" id="tab-keyboard" onclick="switchMode('keyboard')">⌨️ 键盘控制 (Keyboard)</button>
      </div>
    </div>

    <div class="header-badges">
      <div class="badge green" id="badge-torque"><span class="dot"></span><span id="txt-torque">扭矩使能</span></div>
      <div class="badge green" id="badge-state"><span class="dot"></span><span id="txt-state">IDLE 就绪</span></div>
      <div class="badge blue" id="badge-mode"><span class="dot"></span><span id="txt-mode">真机连接</span></div>
      <div class="badge green" id="badge-port"><span class="dot"></span><span id="txt-port">/dev/ttyACM0</span></div>
    </div>
  </header>

  <!-- 满屏主网格容器 -->
  <div class="dashboard-grid">
    <!-- 左侧列: 实时视频流 (自适应拉伸) + 当前模式控制面板 -->
    <div class="column">
      <div class="card video-card">
        <div class="card-header">
          <span>📹 机械臂实时视觉监看 (Live Stream)</span>
          <span style="font-size: 0.72rem; color: var(--text-dim);" id="cam-spec">640x480 @ 30fps</span>
        </div>
        <div class="video-wrapper">
          <img class="video-feed" src="/video_feed" alt="机械臂实时视觉">
          <div class="video-badge"><span class="rec-dot"></span> 实时采集</div>
        </div>
      </div>

      <div class="card control-card">
        <div class="card-header">
          <span id="panel-title">🎮 动作与姿态控制中心</span>
          <span id="busy-indicator" style="font-size: 0.78rem; color: var(--warning); display: none;">⏳ 执行中...</span>
        </div>

        <div id="toast" class="status-toast toast-info"></div>

        <!-- 1. 演示模式面板 (Demo View) -->
        <div class="panel-view active" id="view-demo">
          <div class="btn-grid">
            <button class="btn btn-wave" onclick="runRoutine('wave')" id="btn-wave">
              <span>👋</span> 挥手致意 (Wave)
            </button>
            <button class="btn btn-pick" onclick="runRoutine('pick')" id="btn-pick">
              <span>🦾</span> 抓取表演 (Pick)
            </button>
            <button class="btn btn-home" onclick="runPose('home')" id="btn-home">
              <span>🎯</span> 回直立中位 (Home)
            </button>
            <button class="btn btn-rest" onclick="runPose('rest')" id="btn-rest" title="平稳回蜷缩位并自动断电休眠释放力矩">
              <span>🛌</span> 回蜷缩休眠 (Rest + 断力矩)
            </button>
            <button class="btn btn-torque" onclick="toggleTorque()" id="btn-torque-toggle">
              <span>⚡</span> 切换舵机扭矩状态
            </button>
            <button class="btn btn-home" onclick="runPose('pick_hover')" id="btn-hover">
              <span>👀</span> 抓取悬停 (Hover)
            </button>
            <button class="btn btn-estop" onclick="triggerEstop()" id="btn-estop">
              <span>🛑</span> 紧急停止 (E-STOP)
            </button>
            <button class="btn btn-reset" onclick="requestReset()" id="btn-reset" style="display: none;">
              <span>🔄</span> 解除急停复位
            </button>
          </div>
        </div>

        <!-- 2. 主从遥操作面板 (Teleop View) -->
        <div class="panel-view" id="view-teleop">
          <div class="teleop-panel">
            <div class="teleop-box">
              <div>
                <div style="color: var(--text-dim); font-size: 0.7rem;">主臂端口 (Leader)</div>
                <div id="teleop-leader-port" style="color: var(--accent); font-weight: 700;">/dev/ttyACM1</div>
              </div>
              <div>
                <div style="color: var(--text-dim); font-size: 0.7rem;">从臂端口 (Follower)</div>
                <div id="teleop-follower-port" style="color: var(--success); font-weight: 700;">/dev/ttyACM0</div>
              </div>
              <div>
                <div style="color: var(--text-dim); font-size: 0.7rem;">跟随频率</div>
                <div id="teleop-fps-display" style="color: var(--purple); font-weight: 700;">~30 Hz 实时审计</div>
              </div>
            </div>

            <div style="display: flex; gap: 8px;">
              <button class="btn btn-pick" id="btn-teleop-start" onclick="startTeleop()" style="flex: 1; padding: 12px;">
                <span>🚀</span> 启动主从实时跟随 (Start Follow)
              </button>
              <button class="btn btn-home" id="btn-teleop-stop" onclick="stopTeleop()" style="flex: 1; padding: 12px;" disabled>
                <span>⏹️</span> 停止跟随并回蜷缩 (Stop & Rest)
              </button>
            </div>
            <div style="font-size: 0.72rem; color: var(--text-dim); text-align: center;">
              提示: 主臂保持零力矩可随意拖动，从臂以安全速度平稳镜像跟随。
            </div>
          </div>
        </div>

        <!-- 3. 键盘控制面板 (Keyboard View) -->
        <div class="panel-view" id="view-keyboard">
          <div class="keyboard-panel">
            <div style="display: flex; justify-content: space-between; align-items: center; font-size: 0.75rem; color: var(--text-dim); margin-bottom: 4px;">
              <span>💡 网页聚焦时直接按键盘 WASD/RF/CV 驾驶</span>
              <span style="color: var(--accent); font-family: var(--font-mono);">单步: 2.5°</span>
            </div>

            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
              <!-- 方向移动组 -->
              <div class="dpad-container">
                <div></div>
                <button class="dpad-btn" onclick="sendStepKey('w')"><span>前推/大臂</span><span class="key-sub">[W]</span></button>
                <div></div>
                <button class="dpad-btn" onclick="sendStepKey('a')"><span>左旋/底座</span><span class="key-sub">[A]</span></button>
                <button class="dpad-btn" onclick="sendStepKey('s')"><span>后拉/大臂</span><span class="key-sub">[S]</span></button>
                <button class="dpad-btn" onclick="sendStepKey('d')"><span>右旋/底座</span><span class="key-sub">[D]</span></button>
              </div>

              <!-- 升降与末端组 -->
              <div style="display: flex; flex-direction: column; gap: 6px;">
                <div style="display: flex; gap: 6px;">
                  <button class="dpad-btn" style="flex: 1;" onclick="sendStepKey('r')"><span>升高/小臂</span><span class="key-sub">[R]</span></button>
                  <button class="dpad-btn" style="flex: 1;" onclick="sendStepKey('f')"><span>降低/小臂</span><span class="key-sub">[F]</span></button>
                </div>
                <div style="display: flex; gap: 6px;">
                  <button class="dpad-btn" style="flex: 1;" onclick="sendStepKey('c')"><span>夹爪张开</span><span class="key-sub">[C]</span></button>
                  <button class="dpad-btn" style="flex: 1;" onclick="sendStepKey('v')"><span>夹爪闭合</span><span class="key-sub">[V]</span></button>
                </div>
                <div style="display: flex; gap: 6px;">
                  <button class="dpad-btn" style="flex: 1;" onclick="sendStepKey('u')"><span>手腕逆旋</span><span class="key-sub">[U]</span></button>
                  <button class="dpad-btn" style="flex: 1;" onclick="sendStepKey('i')"><span>手腕顺旋</span><span class="key-sub">[I]</span></button>
                </div>
              </div>
            </div>
          </div>
        </div>

      </div>
    </div>

    <!-- 右侧列: 6 轴角度遥测 + 状态指标 + 终端日志 (自适应垂直拉伸到底) -->
    <div class="column">
      <div class="card">
        <div class="card-header">
          <span>📊 6 轴关节物理角度遥测 (度 °)</span>
          <span style="font-size: 0.72rem; color: var(--accent);" id="refresh-rate">5Hz 实时同步</span>
        </div>
        <div class="joint-list" id="joint-container">
          <!-- 动态渲染关节列表 -->
        </div>
      </div>

      <div class="card" style="flex-shrink: 0;">
        <div class="card-header">
          <span>⚙️ 核心基础数据指标</span>
        </div>
        <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; font-size: 0.8rem; font-family: var(--font-mono);">
          <div style="background: rgba(255,255,255,0.03); padding: 6px 8px; border-radius: 6px;">
            <div style="color: var(--text-dim); font-size: 0.68rem;">控制权归属</div>
            <div id="stat-source" style="font-weight: 700; color: var(--accent);">IDLE</div>
          </div>
          <div style="background: rgba(255,255,255,0.03); padding: 6px 8px; border-radius: 6px;">
            <div style="color: var(--text-dim); font-size: 0.68rem;">舵机扭矩状态</div>
            <div id="stat-torque-txt" style="font-weight: 700; color: var(--success);">ENABLED (6/6)</div>
          </div>
          <div style="background: rgba(255,255,255,0.03); padding: 6px 8px; border-radius: 6px;">
            <div style="color: var(--text-dim); font-size: 0.68rem;">安全姿态基准</div>
            <div id="stat-rest-f" style="font-weight: 700; color: var(--purple); font-size: 0.72rem;">rest_pose.json</div>
          </div>
        </div>
      </div>

      <!-- 动态自适应伸缩日志面板 -->
      <div class="card log-card">
        <div class="card-header">
          <span>📝 终端日志与运动快照</span>
          <button style="background: transparent; border: none; color: var(--text-dim); font-size: 0.72rem; cursor: pointer;" onclick="clearLogs()">清空日志</button>
        </div>
        <div class="log-box" id="log-console">
          <!-- 实时日志注入 -->
        </div>
      </div>
    </div>
  </div>

  <script>
    const JOINTS = [
      { name: "shoulder_pan", label: "基座回转 (Pan)", min: -150, max: 150 },
      { name: "shoulder_lift", label: "大臂俯仰 (Lift)", min: -120, max: 120 },
      { name: "elbow_flex", label: "小臂肘部 (Elbow)", min: -120, max: 120 },
      { name: "wrist_flex", label: "手腕俯仰 (Wrist Flex)", min: -120, max: 120 },
      { name: "wrist_roll", label: "手腕翻滚 (Wrist Roll)", min: -180, max: 180 },
      { name: "gripper", label: "末端夹爪 (Gripper)", min: -20, max: 110 }
    ];

    let currentMode = "demo";
    let isTorqueEnabled = true;

    // 初始化渲染关节槽位
    const container = document.getElementById("joint-container");
    JOINTS.forEach(j => {
      const div = document.createElement("div");
      div.className = "joint-item";
      div.innerHTML = `
        <div class="joint-label">
          <span>${j.label}</span>
          <span id="val-${j.name}" style="color: var(--text); font-weight: 700;">0.00°</span>
        </div>
        <div class="joint-bar-bg">
          <div class="joint-bar-fill" id="bar-${j.name}"></div>
        </div>
      `;
      container.appendChild(div);
    });

    function showToast(msg, isError = false) {
      const t = document.getElementById("toast");
      t.innerText = msg;
      t.className = isError ? "status-toast toast-err" : "status-toast toast-info";
      t.style.display = "block";
      setTimeout(() => { t.style.display = "none"; }, 3500);
    }

    // 模式切换
    function switchMode(mode) {
      currentMode = mode;
      document.querySelectorAll(".mode-tab").forEach(t => t.className = "mode-tab");
      document.querySelectorAll(".panel-view").forEach(v => v.classList.remove("active"));

      if (mode === "demo") {
        document.getElementById("tab-demo").className = "mode-tab active";
        document.getElementById("view-demo").classList.add("active");
        document.getElementById("panel-title").innerText = "🎮 动作与姿态控制中心 (Demo 模式)";
      } else if (mode === "teleop") {
        document.getElementById("tab-teleop").className = "mode-tab active-teleop";
        document.getElementById("view-teleop").classList.add("active");
        document.getElementById("panel-title").innerText = "🎮 主从双臂实时遥操作 (Teleop 模式)";
      } else if (mode === "keyboard") {
        document.getElementById("tab-keyboard").className = "mode-tab active-key";
        document.getElementById("view-keyboard").classList.add("active");
        document.getElementById("panel-title").innerText = "⌨️ 网页与键盘直接遥控 (Keyboard 模式)";
      }
      fetch("/api/mode", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: mode })
      });
    }

    // 监听键盘按键驾驶
    window.addEventListener("keydown", (e) => {
      if (currentMode !== "keyboard") return;
      const k = e.key.toLowerCase();
      if (["w","s","a","d","r","f","c","v","u","i"," "].includes(k)) {
        e.preventDefault();
        sendStepKey(k);
      }
    });

    async function sendStepKey(key) {
      try {
        const res = await fetch("/api/control/key", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ key: key })
        });
        const data = await res.json();
        if (!data.ok) showToast(data.message, true);
      } catch (e) {
        showToast("按键发送失败: " + e, true);
      }
    }

    async function toggleTorque() {
      const nextState = !isTorqueEnabled;
      showToast(nextState ? "正在使能恢复舵机扭矩..." : "正在释放舵机扭矩进入休眠...");
      try {
        const res = await fetch("/api/action/torque", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ enable: nextState })
        });
        const data = await res.json();
        if (!data.ok) showToast(data.message, true);
      } catch (e) {
        showToast("切换扭矩失败: " + e, true);
      }
    }

    async function startTeleop() {
      showToast("正在启动主从双臂遥操作跟随...");
      try {
        const res = await fetch("/api/teleop/start", { method: "POST" });
        const data = await res.json();
        if (!data.ok) {
          showToast(data.message, true);
        } else {
          document.getElementById("btn-teleop-start").disabled = true;
          document.getElementById("btn-teleop-stop").disabled = false;
        }
      } catch (e) {
        showToast("遥操作启动异常: " + e, true);
      }
    }

    async function stopTeleop() {
      showToast("正在停止遥操作并平缓收回蜷缩位...");
      try {
        await fetch("/api/teleop/stop", { method: "POST" });
        document.getElementById("btn-teleop-start").disabled = false;
        document.getElementById("btn-teleop-stop").disabled = true;
      } catch (e) {
        showToast("遥操作停止异常: " + e, true);
      }
    }

    async function fetchStatus() {
      try {
        const res = await fetch("/api/status");
        if (!res.ok) return;
        const data = await res.json();

        // 刷新关节数据
        const pos = data.joints || {};
        JOINTS.forEach(j => {
          const val = pos[j.name] !== undefined ? pos[j.name] : 0.0;
          document.getElementById(`val-${j.name}`).innerText = `${val.toFixed(2)}°`;
          const range = j.max - j.min;
          const pct = Math.max(0, Math.min(100, ((val - j.min) / range) * 100));
          document.getElementById(`bar-${j.name}`).style.width = `${pct}%`;
        });

        // 刷新状态栏
        document.getElementById("txt-state").innerText = data.system_state;
        document.getElementById("stat-source").innerText = data.active_source || "IDLE (无)";
        document.getElementById("txt-port").innerText = data.hardware_port || "/dev/ttyACM0";
        if (data.target_fps) {
          const fpsElem = document.getElementById("teleop-fps-display");
          if (fpsElem) fpsElem.innerText = `~${data.target_fps} Hz 实时审计`;
        }

        // 扭矩状态展示
        isTorqueEnabled = !!data.torque_enabled;
        const tBadge = document.getElementById("badge-torque");
        const tTxt = document.getElementById("txt-torque");
        const statTorqueTxt = document.getElementById("stat-torque-txt");
        if (isTorqueEnabled) {
          tBadge.className = "badge green";
          tTxt.innerText = "⚡ 扭矩使能";
          statTorqueTxt.innerText = "ENABLED (使能保持)";
          statTorqueTxt.style.color = "var(--success)";
        } else {
          tBadge.className = "badge purple";
          tTxt.innerText = "💤 扭矩已释放 (休眠)";
          statTorqueTxt.innerText = "RELEASED (已释放断电)";
          statTorqueTxt.style.color = "var(--purple)";
        }

        if (data.is_mock) {
          document.getElementById("txt-mode").innerText = "仿真模式 (Mock)";
          document.getElementById("badge-mode").className = "badge blue";
        } else {
          document.getElementById("txt-mode").innerText = "真机连接 (Real)";
          document.getElementById("badge-mode").className = "badge green";
        }

        // 急停状态
        if (data.is_estop) {
          document.getElementById("badge-state").className = "badge red";
          document.getElementById("txt-state").innerText = "急停锁定 (ESTOP)";
          document.getElementById("btn-reset").style.display = "block";
          document.getElementById("btn-estop").disabled = true;
        } else {
          document.getElementById("badge-state").className = "badge green";
          document.getElementById("btn-reset").style.display = "none";
          document.getElementById("btn-estop").disabled = false;
        }

        // Busy 状态
        const busyIndicator = document.getElementById("busy-indicator");
        if (data.is_busy) {
          busyIndicator.innerText = `⏳ 执行中: ${data.busy_action}`;
          busyIndicator.style.display = "inline";
          setButtonsDisabled(true);
        } else {
          busyIndicator.style.display = "none";
          setButtonsDisabled(false);
        }

        // 遥操作跟随状态同步
        if (data.is_teleop_running) {
          document.getElementById("btn-teleop-start").disabled = true;
          document.getElementById("btn-teleop-stop").disabled = false;
        }

        // 日志
        if (data.logs) {
          const logBox = document.getElementById("log-console");
          logBox.innerHTML = data.logs.join("<br>");
          logBox.scrollTop = logBox.scrollHeight;
        }

      } catch (e) {
        console.error("轮询异常:", e);
      }
    }

    function setButtonsDisabled(disabled) {
      document.querySelectorAll(".btn-grid button:not(#btn-estop):not(#btn-reset)").forEach(b => {
        b.disabled = disabled;
      });
    }

    async function runRoutine(name) {
      showToast(`正在启动演示动线: ${name}...`);
      try {
        const res = await fetch("/api/action/routine", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ routine: name })
        });
        const resData = await res.json();
        if (!resData.ok) showToast(resData.message, true);
      } catch (e) {
        showToast("请求发送异常: " + e, true);
      }
    }

    async function runPose(name) {
      showToast(name === "rest" ? "正在平滑回归蜷缩位并释放扭矩休眠..." : `正在平滑移动至姿态: ${name}...`);
      try {
        const res = await fetch("/api/action/pose", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ pose: name })
        });
        const resData = await res.json();
        if (!resData.ok) showToast(resData.message, true);
      } catch (e) {
        showToast("请求发送异常: " + e, true);
      }
    }

    async function triggerEstop() {
      showToast("🛑 触发紧急停止！机械臂原地锁紧", true);
      await fetch("/api/action/estop", { method: "POST" });
    }

    async function requestReset() {
      showToast("正在请求解除急停复位...");
      await fetch("/api/action/reset", { method: "POST" });
    }

    function clearLogs() {
      document.getElementById("log-console").innerHTML = "";
    }

    // 5Hz 状态轮询
    setInterval(fetchStatus, 200);
    fetchStatus();
  </script>
</body>
</html>
"""


class WebRequestHandler(BaseHTTPRequestHandler):
    """处理前端 HTTP 请求、REST API、遥操作与 MJPEG 视频流."""

    def log_message(self, format: str, *args: Any) -> None:
        pass

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode("utf-8"))

        elif self.path == "/video_feed":
            self.send_response(HTTPStatus.OK)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()

            streamer = _CAMERA_STREAMER
            try:
                while True:
                    if streamer:
                        frame_bytes = streamer.get_jpeg()
                        if frame_bytes:
                            self.wfile.write(b"--FRAME\r\n")
                            self.send_header("Content-Type", "image/jpeg")
                            self.send_header("Content-Length", str(len(frame_bytes)))
                            self.end_headers()
                            self.wfile.write(frame_bytes)
                            self.wfile.write(b"\r\n")
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path == "/api/status":
            self.handle_get_status()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

    def do_POST(self) -> None:
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length) if content_length > 0 else b"{}"
        try:
            payload = json.loads(body.decode("utf-8")) if body else {}
        except Exception:
            payload = {}

        if self.path == "/api/mode":
            self.handle_post_mode(payload)
        elif self.path == "/api/action/routine":
            self.handle_post_routine(payload)
        elif self.path == "/api/action/pose":
            self.handle_post_pose(payload)
        elif self.path == "/api/action/torque":
            self.handle_post_torque(payload)
        elif self.path == "/api/action/estop":
            self.handle_post_estop()
        elif self.path == "/api/action/reset":
            self.handle_post_reset()
        elif self.path == "/api/teleop/start":
            self.handle_post_teleop_start()
        elif self.path == "/api/teleop/stop":
            self.handle_post_teleop_stop()
        elif self.path == "/api/control/key":
            self.handle_post_control_key(payload)
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

    def handle_get_status(self) -> None:
        cfg = get_config()
        joints_deg: dict[str, float] = {}
        sys_state = "DISCONNECTED"
        active_source = None
        is_estop = False
        is_mock = False
        torque_enabled = True

        if _GLOBAL_API is not None:
            try:
                follower_obj = getattr(_GLOBAL_API.hardware, "follower", None)
                last_cached = getattr(follower_obj, "_last_state", None)

                # 若机械臂正忙于执行运动或正在遥操作，优先读取内存缓存，避免高频打架争抢串口
                if (_IS_BUSY or _TELEOP_RUNNING) and last_cached is not None:
                    st = last_cached
                else:
                    st = _GLOBAL_API.get_joint_state()

                sys_state = _GLOBAL_API.state_machine.current_state.value
                active_source = (
                    _GLOBAL_API.state_machine.active_source.value
                    if _GLOBAL_API.state_machine.active_source
                    else None
                )
                is_estop = _GLOBAL_API.state_machine.is_estop
                is_mock = _GLOBAL_API.hardware.config.use_mock

                # 检查扭矩使能数量
                if follower_obj is not None:
                    h = getattr(follower_obj, "_health", None)
                    if h is not None:
                        torque_enabled = (h.torque_enabled_count > 0)

                if st and st.positions:
                    for k in SO101_JOINT_NAMES:
                        val = st.positions.get(k, 0.0)
                        joints_deg[k] = round(rad_to_deg(val) if st.unit == JointUnit.RADIAN else val, 2)
            except Exception:
                pass

        data = {
            "joints": joints_deg,
            "system_state": sys_state,
            "active_source": active_source,
            "is_estop": is_estop,
            "is_busy": _IS_BUSY,
            "busy_action": _BUSY_ACTION_NAME,
            "hardware_port": cfg.follower.port,
            "is_mock": is_mock,
            "torque_enabled": torque_enabled,
            "active_mode": _ACTIVE_MODE,
            "is_teleop_running": _TELEOP_RUNNING,
            "target_fps": getattr(cfg.operation, "fps", 30),
            "logs": _LAST_LOGS,
        }
        self.send_json(data)

    def handle_post_mode(self, payload: dict[str, Any]) -> None:
        global _ACTIVE_MODE, _TELEOP_RUNNING
        mode = payload.get("mode", "demo")
        old_mode = _ACTIVE_MODE
        _ACTIVE_MODE = mode
        add_web_log(f"控制模式切换: [{old_mode}] -> [{mode}]")

        if old_mode == "teleop" and mode != "teleop":
            self.stop_teleop_internal()

        if _GLOBAL_API is not None:
            curr_src = _GLOBAL_API.state_machine.active_source
            if mode == "keyboard":
                if curr_src != ControlSource.KEYBOARD:
                    if curr_src:
                        _GLOBAL_API.release_control(curr_src)
                    _GLOBAL_API.claim_control(ControlSource.KEYBOARD)
            elif mode == "demo":
                if curr_src == ControlSource.KEYBOARD:
                    _GLOBAL_API.release_control(ControlSource.KEYBOARD)

        self.send_json({"ok": True, "mode": mode})

    def handle_post_torque(self, payload: dict[str, Any]) -> None:
        enable = bool(payload.get("enable", True))
        if _GLOBAL_API is None:
            self.send_json({"ok": False, "message": "硬件未就绪"})
            return

        if enable:
            ok = _GLOBAL_API.enable_torque()
            add_web_log("⚡ 用户手动恢复舵机扭矩")
        else:
            ok = _GLOBAL_API.disable_torque()
            add_web_log("💤 用户手动释放舵机扭矩进入休眠")

        self.send_json({"ok": ok, "torque_enabled": enable})

    def handle_post_routine(self, payload: dict[str, Any]) -> None:
        global _IS_BUSY, _BUSY_ACTION_NAME
        routine = payload.get("routine", "wave")

        if _IS_BUSY:
            self.send_json({"ok": False, "message": f"机械臂正在执行 [{_BUSY_ACTION_NAME}]，请稍后"})
            return

        if _GLOBAL_API is None or _GLOBAL_RUNNER is None:
            self.send_json({"ok": False, "message": "硬件未初始化或连接失败"})
            return

        if _GLOBAL_API.state_machine.is_estop:
            self.send_json({"ok": False, "message": "系统处于急停锁定中，请先解除急停"})
            return

        _IS_BUSY = True
        _BUSY_ACTION_NAME = f"动线: {routine}"

        def _worker():
            global _IS_BUSY, _BUSY_ACTION_NAME
            with _API_LOCK:
                add_web_log(f"开始执行动线: {routine}")
                try:
                    curr_src = _GLOBAL_API.state_machine.active_source
                    if curr_src != ControlSource.SCRIPT:
                        if curr_src is not None:
                            _GLOBAL_API.release_control(curr_src)
                        _GLOBAL_API.claim_control(ControlSource.SCRIPT)

                    if routine == "wave":
                        _GLOBAL_RUNNER.run_wave_routine()
                    elif routine == "pick":
                        _GLOBAL_RUNNER.run_pick_routine()
                    # 完成后自动安全回蜷缩并释放扭矩
                    _GLOBAL_API.safe_return_to_rest(duration_s=2.0, release_torque=True)
                    add_web_log(f"动线 [{routine}] 执行完成并已安全回蜷缩位释放扭矩。")
                except Exception as exc:
                    add_web_log(f"动线执行异常: {exc}")
                finally:
                    curr_src = _GLOBAL_API.state_machine.active_source
                    if curr_src == ControlSource.SCRIPT:
                        _GLOBAL_API.release_control(ControlSource.SCRIPT)
                    _IS_BUSY = False
                    _BUSY_ACTION_NAME = ""

        threading.Thread(target=_worker, daemon=True).start()
        self.send_json({"ok": True, "message": f"正在启动动线: {routine}"})

    def handle_post_pose(self, payload: dict[str, Any]) -> None:
        global _IS_BUSY, _BUSY_ACTION_NAME
        pose = payload.get("pose", "home")

        if _IS_BUSY:
            self.send_json({"ok": False, "message": f"正在执行 [{_BUSY_ACTION_NAME}]，请稍后"})
            return

        if _GLOBAL_API is None:
            self.send_json({"ok": False, "message": "硬件未就绪"})
            return

        _IS_BUSY = True
        _BUSY_ACTION_NAME = f"姿态: {pose}"

        def _worker():
            global _IS_BUSY, _BUSY_ACTION_NAME
            with _API_LOCK:
                add_web_log(f"正在平滑过渡到姿态: {pose}")
                try:
                    curr_src = _GLOBAL_API.state_machine.active_source
                    if curr_src != ControlSource.SCRIPT:
                        if curr_src is not None:
                            _GLOBAL_API.release_control(curr_src)
                        _GLOBAL_API.claim_control(ControlSource.SCRIPT)

                    if pose == "rest":
                        # 回到蜷缩位后自动释放扭矩
                        _GLOBAL_API.safe_return_to_rest(duration_s=2.0, release_torque=True)
                        add_web_log("姿态 [rest] 已就位，扭矩已释放进入休眠。")
                    else:
                        _GLOBAL_RUNNER.run_pose(pose, duration_s=2.0)
                        add_web_log(f"姿态 [{pose}] 已就位")
                except Exception as exc:
                    add_web_log(f"姿态移动异常: {exc}")
                finally:
                    curr_src = _GLOBAL_API.state_machine.active_source
                    if curr_src == ControlSource.SCRIPT:
                        _GLOBAL_API.release_control(ControlSource.SCRIPT)
                    _IS_BUSY = False
                    _BUSY_ACTION_NAME = ""

        threading.Thread(target=_worker, daemon=True).start()
        self.send_json({"ok": True, "message": f"正在移动至姿态: {pose}"})

    def handle_post_control_key(self, payload: dict[str, Any]) -> None:
        """处理网页按键或触控方向盘微调."""
        if _GLOBAL_API is None:
            self.send_json({"ok": False, "message": "硬件未就绪"})
            return

        key = str(payload.get("key", "")).lower()
        if not key:
            self.send_json({"ok": False, "message": "按键为空"})
            return

        if key == " ":
            _GLOBAL_API.trigger_emergency_stop("键盘空格触发急停")
            add_web_log("🛑 键盘拍下空格键触发急停！")
            self.send_json({"ok": True, "message": "急停已触发"})
            return

        # 确保持有 KEYBOARD 控制权
        curr_src = _GLOBAL_API.state_machine.active_source
        if curr_src != ControlSource.KEYBOARD:
            if curr_src is not None:
                _GLOBAL_API.release_control(curr_src)
            _GLOBAL_API.claim_control(ControlSource.KEYBOARD)

        # 读取当前真实关节角度
        st = _GLOBAL_API.get_joint_state()
        if not st or not st.positions:
            self.send_json({"ok": False, "message": "无法读取机械臂当前关节状态"})
            return

        pos = dict(st.positions)
        step_rad = deg_to_rad(2.5)
        step_gripper = deg_to_rad(4.0)

        # 按键映射
        if key == "a":
            pos["shoulder_pan"] = pos.get("shoulder_pan", 0.0) + step_rad
        elif key == "d":
            pos["shoulder_pan"] = pos.get("shoulder_pan", 0.0) - step_rad
        elif key == "w":
            pos["shoulder_lift"] = pos.get("shoulder_lift", 0.0) - step_rad
        elif key == "s":
            pos["shoulder_lift"] = pos.get("shoulder_lift", 0.0) + step_rad
        elif key == "r":
            pos["elbow_flex"] = pos.get("elbow_flex", 0.0) - step_rad
        elif key == "f":
            pos["elbow_flex"] = pos.get("elbow_flex", 0.0) + step_rad
        elif key == "u":
            pos["wrist_roll"] = pos.get("wrist_roll", 0.0) - deg_to_rad(3.5)
        elif key == "i":
            pos["wrist_roll"] = pos.get("wrist_roll", 0.0) + deg_to_rad(3.5)
        elif key == "c":
            pos["gripper"] = pos.get("gripper", 0.0) + step_gripper
        elif key == "v":
            pos["gripper"] = pos.get("gripper", 0.0) - step_gripper
        else:
            self.send_json({"ok": False, "message": f"未知按键: {key}"})
            return

        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=time.monotonic(),
            target_joint_positions=pos,
            unit=JointUnit.RADIAN,
        )
        ok, reason = _GLOBAL_API.submit_action(action)
        self.send_json({"ok": ok, "message": reason})

    def handle_post_teleop_start(self) -> None:
        global _TELEOP_RUNNING, _TELEOP_THREAD
        if _GLOBAL_API is None:
            self.send_json({"ok": False, "message": "系统未就绪"})
            return

        if _TELEOP_RUNNING:
            self.send_json({"ok": True, "message": "遥操作已在运行中"})
            return

        cfg = get_config()
        # 尝试连接主臂
        if not _GLOBAL_API.hardware.is_leader_connected and _GLOBAL_API.hardware.leader:
            try:
                _GLOBAL_API.hardware.leader.connect()
            except Exception as exc:
                add_web_log(f"⚠️ 主臂连接失败 ({cfg.leader.port}): {exc}")
                self.send_json({"ok": False, "message": f"未检测到主臂硬件 ({cfg.leader.port})，请插入主臂后再试"})
                return

        if not _GLOBAL_API.hardware.is_leader_connected:
            self.send_json({"ok": False, "message": f"主臂未连接，请先插入主臂 USB ({cfg.leader.port})"})
            return

        # 申请 LEADER 控制权
        curr_src = _GLOBAL_API.state_machine.active_source
        if curr_src != ControlSource.LEADER:
            if curr_src:
                _GLOBAL_API.release_control(curr_src)
            _GLOBAL_API.claim_control(ControlSource.LEADER)

        _TELEOP_RUNNING = True

        def _teleop_worker():
            global _TELEOP_RUNNING
            fps = float(getattr(cfg.operation, "fps", 30))
            period_s = 1.0 / fps if fps > 0 else 1.0 / 30.0
            add_web_log(f"🚀 主从双臂遥操作跟随流水线已启动 (~{fps:.0f}Hz 高精度闭环)")
            try:
                while _TELEOP_RUNNING and _GLOBAL_API and not _GLOBAL_API.state_machine.is_estop:
                    t_start = time.perf_counter()
                    leader_st = _GLOBAL_API.hardware.read_leader_state()
                    if leader_st and leader_st.positions:
                        action = RobotAction(
                            action_type=ActionType.JOINT_POSITION,
                            source=ControlSource.LEADER,
                            timestamp=time.monotonic(),
                            target_joint_positions=leader_st.positions,
                            unit=leader_st.unit,
                        )
                        _GLOBAL_API.submit_action(action)
                    elapsed = time.perf_counter() - t_start
                    sleep_s = period_s - elapsed
                    if sleep_s > 0:
                        time.sleep(sleep_s)
            except Exception as exc:
                add_web_log(f"遥操作异常中断: {exc}")
            finally:
                _TELEOP_RUNNING = False
                add_web_log("遥操作循环已退出")

        _TELEOP_THREAD = threading.Thread(target=_teleop_worker, daemon=True)
        _TELEOP_THREAD.start()
        self.send_json({"ok": True, "message": "主从遥操作跟随已启动"})

    def handle_post_teleop_stop(self) -> None:
        self.stop_teleop_internal()
        # 退出后自动平缓回蜷缩并释放扭矩
        if _GLOBAL_API is not None:
            def _rest_worker():
                _GLOBAL_API.safe_return_to_rest(duration_s=2.0, release_torque=True)
            threading.Thread(target=_rest_worker, daemon=True).start()
        self.send_json({"ok": True, "message": "遥操作已停止，机械臂平缓返回休眠位"})

    def stop_teleop_internal(self) -> None:
        global _TELEOP_RUNNING
        _TELEOP_RUNNING = False
        if _GLOBAL_API is not None:
            curr_src = _GLOBAL_API.state_machine.active_source
            if curr_src == ControlSource.LEADER:
                _GLOBAL_API.release_control(ControlSource.LEADER)

    def handle_post_estop(self) -> None:
        global _TELEOP_RUNNING
        _TELEOP_RUNNING = False
        if _GLOBAL_API is not None:
            _GLOBAL_API.trigger_emergency_stop("Web 前端触发急停")
            add_web_log("🛑 Web 前端拍下紧急停止按钮！")
        self.send_json({"ok": True, "message": "急停已触发"})

    def handle_post_reset(self) -> None:
        if _GLOBAL_API is not None:
            ok, msg = _GLOBAL_API.request_reset()
            add_web_log(f"人工复位申请: {'成功' if ok else '拒绝'} ({msg})")
            self.send_json({"ok": ok, "message": msg})
        else:
            self.send_json({"ok": False, "message": "API 未就绪"})

    def send_json(self, data: dict[str, Any], status: int = HTTPStatus.OK) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)


def run_web_server(port: int = 8080, mock: bool = False, follower_port: str = "/dev/ttyACM0") -> int:
    """初始化硬件并启动 Web 控制中心服务."""
    global _GLOBAL_API, _GLOBAL_RUNNER, _CAMERA_STREAMER

    cfg = get_config()
    print("=" * 66)
    print("      SO-ARM101 具身机械臂极速控制中心 (Web Cockpit V2.0)")
    print("=" * 66)
    print(f"[*] 监听端口: {port}")
    print(f"[*] 从臂端口: {follower_port}")
    print(f"[*] 主臂端口: {cfg.leader.port}")
    print(f"[*] 相机设备: {cfg.camera.index_or_path} ({cfg.camera.width}x{cfg.camera.height} @ {cfg.camera.fps}fps)")
    print(f"[*] 仿真模式: {'是 (Mock 仿真)' if mock else '否 (Real 真实硬件)'}")
    print("=" * 66)

    # 1. 初始化相机采集推流器
    _CAMERA_STREAMER = CameraStreamer(
        camera_device=cfg.camera.index_or_path,
        width=cfg.camera.width,
        height=cfg.camera.height,
        fps=cfg.camera.fps,
    )
    if not mock:
        _CAMERA_STREAMER.start()

    # 2. 初始化核心动作 API 与从臂
    try:
        if mock:
            sys_cfg = RobotSystemConfig.mock_system()
        else:
            sys_cfg = RobotSystemConfig.real_system(
                follower_port=follower_port,
                leader_port=cfg.leader.port,
            )

        api = ActionAPI(config=sys_cfg)
        connected = api.connect(connect_leader=False)
        if not connected and not mock:
            print(f"[!] 警告: 从臂连接失败 ({follower_port})，自动降级为仿真模式")
            sys_cfg = RobotSystemConfig.mock_system()
            api = ActionAPI(config=sys_cfg)
            api.connect(connect_leader=False)

        _GLOBAL_API = api
        _GLOBAL_RUNNER = DemoRoutineRunner(api)
        add_web_log("系统硬件初始化完成，控制中心就绪。")
    except Exception as exc:
        print(f"[!] 硬件连接异常: {exc}")
        add_web_log(f"硬件连接异常: {exc}")

    # 3. 启动多线程 HTTP 服务
    server_address = ("", port)
    server = ThreadingHTTPServer(server_address, WebRequestHandler)
    server.daemon_threads = True

    print(f"\n[✓] Web 控制驾驶舱已成功启动!")
    print(f"👉 本地浏览器访问:  http://localhost:{port}")
    print(f"👉 局域网设备访问:  http://0.0.0.0:{port}\n")
    print("[*] 按 Ctrl+C 可随时安全退出服务 (退出自动平缓回归蜷缩休眠姿态)...")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] 正在关闭 Web 控制中心...")
    finally:
        server.server_close()
        if _CAMERA_STREAMER:
            _CAMERA_STREAMER.close()
        if _GLOBAL_API:
            _GLOBAL_API.disconnect(keep_torque=False, return_to_rest=True)
        print("[✓] Web 控制中心已安全停止，舵机已断电保护。")

    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SO-ARM101 极速原型 Web 控制中心")
    parser.add_argument("--port", type=int, default=8080, help="Web 监听端口 (默认 8080)")
    parser.add_argument("--mock", action="store_true", help="启用虚拟仿真模式 (无须插硬件)")
    parser.add_argument("--follower-port", default="/dev/ttyACM0", help="从臂端口")
    args = parser.parse_args()

    sys.exit(run_web_server(port=args.port, mock=args.mock, follower_port=args.follower_port))
