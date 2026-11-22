"""SO-ARM101 遥操作实时视觉监看与多模态示教同步录制引擎 (teleop_camera.py).

模式 C 核心能力:
1. 实时预览监看 (HUD Overlay): 桌面弹出 OpenCV 轻量小窗口，显示实时画面并叠加录制状态与操作提示;
2. 双轨键鼠交互: 用户既可以在终端按空格/Q，也可以在相机监看窗口按空格/Q，体验一体化;
3. 多模态训练集采集:
   - 示教录制期间，逐帧落盘高画质 JPEG (images/cam_robot/frame_XXXXXX.jpg);
   - 同步以 30FPS 生成标准 MP4 视频 (video_cam_robot.mp4);
   - 严格生成符合 core.data_types.CameraFrameRef 的时序引用，为 ACT / Diffusion Policy 模仿学习模型训练提供原生支持;
4. 独立后台采集线程: 无论图像编码或绘图耗时多久，相机帧读取都在后台平稳进行，绝不拖慢 30Hz 机械臂伺服控制循环.
"""

from __future__ import annotations

import logging
import os
import platform
import threading
import time
from pathlib import Path
from typing import Any

if platform.system() == "Windows" and "OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS" not in os.environ:
    os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore[assignment]
try:
    import numpy as np
except ImportError:
    np = None  # type: ignore[assignment]

from core.data_types import CameraFrameRef

logger = logging.getLogger("teleop_camera")


class TeleopCameraManager:
    """遥操作实时监看与多模态图像录制管理器."""

    def __init__(
        self,
        device_index: int | str = "auto",
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
    ) -> None:
        self.requested_device = device_index
        self.target_width = width
        self.target_height = height
        self.target_fps = fps

        self._cap: cv2.VideoCapture | None = None
        self._device_path: str = ""
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()

        self._latest_frame: np.ndarray | None = None
        self._latest_timestamp: float = 0.0
        self._frame_seq: int = 0

        # 录制相关状态
        self._is_recording = False
        self._current_task_dir: Path | None = None
        self._images_dir: Path | None = None
        self._video_writer: cv2.VideoWriter | None = None
        self._rec_frame_idx = 0

    @classmethod
    def auto_detect_camera_index(cls) -> int:
        """自动探测最优先的机械臂摄像头 (跨 Windows DirectShow 与 Linux /dev/video*)."""
        if cv2 is None:
            return 0
        is_win = platform.system() == "Windows"
        for candidate in (2, 0, 1, 3):
            if not is_win:
                dev_p = f"/dev/video{candidate}"
                if not Path(dev_p).exists():
                    continue
                cap = cv2.VideoCapture(candidate)
            else:
                cap = (
                    cv2.VideoCapture(candidate, cv2.CAP_DSHOW)
                    if hasattr(cv2, "CAP_DSHOW")
                    else cv2.VideoCapture(candidate)
                )
            try:
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        return candidate
            finally:
                cap.release()
        return 0

    def start(self) -> bool:
        """启动相机捕获与后台读取线程."""
        if cv2 is None:
            print("[!] 未安装 OpenCV (cv2)，无法启动摄像头。")
            return False

        if self.requested_device == "auto":
            dev_idx = self.auto_detect_camera_index()
        else:
            try:
                dev_idx = int(self.requested_device)
            except ValueError:
                # 如 /dev/video2
                dev_str = str(self.requested_device)
                dev_idx = int(dev_str.replace("/dev/video", "")) if "video" in dev_str else 0

        is_win = platform.system() == "Windows"
        self._device_path = f"camera:{dev_idx}" if is_win else f"/dev/video{dev_idx}"
        print(f"[*] 正在初始化摄像头: {self._device_path} (目标分辨率: {self.target_width}x{self.target_height})...")

        if is_win and hasattr(cv2, "CAP_DSHOW"):
            cap = cv2.VideoCapture(dev_idx, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                cap = cv2.VideoCapture(dev_idx)
        else:
            cap = cv2.VideoCapture(dev_idx)

        if not cap.isOpened():
            print(f"[!] 无法打开摄像头: {self._device_path}")
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
        cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        if is_win:
            try:
                fourcc = cv2.VideoWriter.fourcc(*"MJPG") if hasattr(cv2.VideoWriter, "fourcc") else cv2.VideoWriter_fourcc(*"MJPG")
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            except Exception:
                pass

        self._cap = cap
        self._running = True
        self._thread = threading.Thread(target=self._capture_worker, daemon=True)
        self._thread.start()

        # 等待第一帧捕获完成
        for _ in range(20):
            if self._latest_frame is not None:
                break
            time.sleep(0.05)

        if self._latest_frame is None:
            print(f"[!] 摄像头 {self._device_path} 已打开但未读取到有效画面")
            return False

        h, w = self._latest_frame.shape[:2]
        print(f"[✓] 摄像头就绪: {self._device_path} (实测分辨率: {w}x{h})")
        return True

    def _capture_worker(self) -> None:
        """后台独立抓帧工作线程，保持低延迟最新帧."""
        while self._running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            with self._lock:
                self._latest_frame = frame
                self._latest_timestamp = time.monotonic()
                self._frame_seq += 1

            time.sleep(0.005)

    def get_latest_frame(self, max_age_ms: int | None = 500) -> tuple[np.ndarray | None, float]:
        """获取当前最新的一帧图像及单调时间戳 (支持 max_age_ms 新鲜度检验)."""
        with self._lock:
            if self._latest_frame is None:
                return None, 0.0
            frame = self._latest_frame.copy()
            ts = self._latest_timestamp
        if max_age_ms is not None and ts > 0.0 and (time.monotonic() - ts) * 1000.0 > max_age_ms:
            return None, 0.0
        return frame, ts

    def start_recording(self, task_dir: Path) -> None:
        """开启多模态数据录制 (创建图片目录与 MP4 视频写入器)."""
        self._current_task_dir = task_dir
        self._images_dir = task_dir / "images" / "cam_robot"
        self._images_dir.mkdir(parents=True, exist_ok=True)

        w, h = self.target_width, self.target_height
        if self._latest_frame is not None:
            h, w = self._latest_frame.shape[:2]

        video_path = task_dir / "video_cam_robot.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._video_writer = cv2.VideoWriter(str(video_path), fourcc, self.target_fps, (w, h))

        self._rec_frame_idx = 0
        self._is_recording = True
        logger.info("[*] 多模态图像与视频录制已启动 -> %s", self._images_dir)

    def record_step(self, frame_index: int) -> CameraFrameRef | None:
        """在示教单步中持久化当前相机帧，并返回 CameraFrameRef 元数据."""
        if not self._is_recording or self._latest_frame is None or self._images_dir is None:
            return None

        frame, ts = self.get_latest_frame()
        if frame is None:
            return None

        h, w = frame.shape[:2]
        # 1. 写入单帧 JPEG
        filename = f"frame_{frame_index:06d}.jpg"
        save_path = self._images_dir / filename
        cv2.imwrite(str(save_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])

        # 2. 写入 MP4 视频帧
        if self._video_writer and self._video_writer.isOpened():
            self._video_writer.write(frame)

        self._rec_frame_idx += 1

        rel_path = f"images/cam_robot/{filename}"
        return CameraFrameRef(
            camera_id="cam_robot",
            frame_id=frame_index,
            timestamp=ts,
            width=w,
            height=h,
            color_format="BGR",
            relative_path=rel_path,
        )

    def stop_recording(self) -> None:
        """结束当前 Episode 图像与视频录制."""
        self._is_recording = False
        if self._video_writer:
            try:
                self._video_writer.release()
            except Exception:
                pass
            self._video_writer = None
        self._current_task_dir = None
        self._images_dir = None
        logger.info("[*] 多模态图像与视频录制已落盘保存")

    def render_preview_and_poll_key(
        self,
        is_recording: bool,
        action_idx: int,
        frame_count: int = 0,
        duration_s: float = 0.0,
        fps: float = 30.0,
    ) -> str | None:
        """在桌面上渲染带 HUD 监看窗口，并捕获窗口快捷键 (零延迟轮询)."""
        frame, _ = self.get_latest_frame()
        if frame is None:
            return None

        display = frame.copy()
        h, w = display.shape[:2]

        # 1. 顶部半透明 HUD 背景条
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 42), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, display, 0.3, 0, display)

        # 2. 状态标签与指示灯
        if is_recording:
            # 录制中: 红色闪烁呼吸点
            blink = int(time.time() * 2) % 2 == 0
            dot_color = (0, 0, 255) if blink else (50, 50, 180)
            cv2.circle(display, (20, 21), 7, dot_color, -1)
            text_status = f"REC [Action {action_idx}]  {frame_count} frames | {duration_s:.1f}s"
            cv2.putText(display, text_status, (36, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(display, "[SPACE] Stop & Save", (w - 210, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        else:
            # 待机/跟随中: 绿色常亮圆点
            cv2.circle(display, (20, 21), 7, (0, 220, 0), -1)
            text_status = f"LIVE [FOLLOWING] {fps:.1f} Hz"
            cv2.putText(display, text_status, (36, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display, f"[SPACE] Rec Action {action_idx}", (w - 230, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

        # 底部提示小条
        cv2.putText(
            display,
            "Press [SPACE] to Record/Save | [q] to Exit",
            (15, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (200, 200, 200),
            1,
        )

        try:
            win_name = f"SO-ARM101 Live Camera ({self._device_path})"
            cv2.imshow(win_name, display)
            key_code = cv2.waitKey(1) & 0xFF

            if key_code == 255 or key_code == 0:
                return None
            if key_code in (ord("q"), ord("Q"), 27):  # ESC or q
                return "q"
            if key_code == 32:  # 空格
                return " "
            if key_code in (ord("r"), ord("R")):
                return "r"
            return chr(key_code) if 32 <= key_code <= 126 else None
        except Exception:
            # 环境缺少 HighGUI 库 (如 opencv-python-headless) 或无 X11 显示环境时，平稳降级
            if not getattr(self, "_headless_warned", False):
                print(f"[INFO] 检测到无窗口显示环境 (Headless)，已关闭弹窗预览；摄像头在后台全速捕获与录制。")
                self._headless_warned = True
            return None

    def close(self) -> None:
        """关闭相机与预览窗口."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

        if self._is_recording:
            self.stop_recording()

        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        logger.info("[*] 相机模块已完全关闭释放")
