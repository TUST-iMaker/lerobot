"""Cross-platform serial port, OpenCV camera, and runtime capability probing."""

from __future__ import annotations

import importlib.util
import os
import platform
import re
import shutil
import sys
import threading
from pathlib import Path
from typing import Any

from app.backend.config import ModeType, StudioConfig

# Process-wide lock to serialize OpenCV / DirectShow camera access across threads.
CAMERA_IO_LOCK = threading.Lock()

if platform.system() == "Windows":
    # Must be set before importing cv2 to prevent slow MSMF hardware transforms on Windows.
    os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")


USB_SERIAL_CHIPS: dict[tuple[str, str], str] = {
    ("1A86", "7523"): "CH340 (Feetech/SO-101)",
    ("1A86", "55D3"): "CH343 (USB-Serial)",
    ("0483", "5740"): "STM32 USB CDC",
    ("0483", "5711"): "STM32 USB CDC",
    ("10C4", "EA60"): "CP210x",
}

_VID_PID_RE = re.compile(r"VID_([0-9A-Fa-f]{4}).*?PID_([0-9A-Fa-f]{4})")


def _classify_port(
    device: str,
    description: str,
    hwid: str,
    vid_int: int | None,
    pid_int: int | None,
) -> dict[str, Any]:
    hwid_upper = (hwid or "").upper()
    desc_lower = (description or "").lower()

    vid = f"{vid_int:04X}" if vid_int is not None else None
    pid = f"{pid_int:04X}" if pid_int is not None else None
    if (vid is None or pid is None) and hwid_upper:
        m = _VID_PID_RE.search(hwid_upper)
        if m:
            vid = vid or m.group(1)
            pid = pid or m.group(2)

    is_bluetooth = (
        "BTHENUM" in hwid_upper
        or hwid_upper.startswith("BTH")
        or "bluetooth" in desc_lower
        or "蓝牙" in (description or "")
    )

    tag = "Unknown Serial"
    is_robot_candidate = False
    if is_bluetooth:
        tag = "Bluetooth Serial"
        is_robot_candidate = False
    elif vid and pid and (vid, pid) in USB_SERIAL_CHIPS:
        tag = USB_SERIAL_CHIPS[(vid, pid)]
        is_robot_candidate = True
    elif vid == "0403":
        tag = "FTDI"
        is_robot_candidate = True
    elif vid and pid:
        tag = f"USB Serial ({vid}:{pid})"
        is_robot_candidate = True
    elif "USB" in hwid_upper or "ACM" in device.upper():
        tag = "USB-Serial"
        is_robot_candidate = True

    real_device = os.path.realpath(device) if platform.system() != "Windows" else device
    permission_ok = (
        True if platform.system() == "Windows" else os.access(real_device, os.R_OK | os.W_OK)
    )

    return {
        "device": device,
        "real_device": real_device,
        "tag": tag,
        "description": description or device,
        "vid": vid,
        "pid": pid,
        "hwid": hwid,
        "is_bluetooth": is_bluetooth,
        "is_robot_candidate": is_robot_candidate,
        "permission_ok": permission_ok,
    }


def _find_cli(name: str) -> bool:
    if shutil.which(name):
        return True
    scripts_dir = Path(sys.executable).parent
    for suffix in ("", ".exe"):
        if (scripts_dir / f"{name}{suffix}").exists():
            return True
    return False


class HardwareManager:
    """Probes serial ports, cameras, packages, and resolves operating mode."""

    def __init__(self, config: StudioConfig) -> None:
        self.config = config
        self.requested_mode: ModeType = config.requested_mode
        self.active_mode: str = "dry_run"
        self.last_probe: dict[str, Any] = {}

    def probe_serial_ports(self) -> list[dict[str, Any]]:
        """Discover serial ports while filtering Bluetooth SPP and phantom UARTs."""
        ports: list[dict[str, Any]] = []
        seen_real: set[str] = set()
        try:
            import serial.tools.list_ports

            for info in serial.tools.list_ports.comports():
                dev = getattr(info, "device", "") or ""
                if not dev:
                    continue
                if platform.system() != "Windows" and re.match(r"^/dev/ttyS\d+$", dev):
                    hwid_val = getattr(info, "hwid", "") or ""
                    if not hwid_val or hwid_val.lower() == "n/a":
                        continue
                entry = _classify_port(
                    device=dev,
                    description=getattr(info, "description", "") or "",
                    hwid=getattr(info, "hwid", "") or "",
                    vid_int=getattr(info, "vid", None),
                    pid_int=getattr(info, "pid", None),
                )
                key = entry["real_device"]
                if key not in seen_real:
                    seen_real.add(key)
                    ports.append(entry)
        except Exception:
            pass

        if platform.system() == "Linux":
            for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
                base = Path(pattern).parent
                glob_pat = Path(pattern).name
                if not base.exists():
                    continue
                for candidate in sorted(base.glob(glob_pat)):
                    real_path = os.path.realpath(candidate)
                    if real_path in seen_real:
                        continue
                    seen_real.add(real_path)
                    ports.append(
                        _classify_port(
                            device=str(candidate),
                            description=candidate.name,
                            hwid="USB",
                            vid_int=None,
                            pid_int=None,
                        )
                    )
        return ports

    def probe_cameras(self) -> list[dict[str, Any]]:
        """Enumerate cameras safely without blocking on slow indices."""
        cameras: list[dict[str, Any]] = []
        try:
            import cv2
        except Exception:
            return cameras

        prev_level = None
        try:
            if hasattr(cv2, "getLogLevel") and hasattr(cv2, "setLogLevel"):
                prev_level = cv2.getLogLevel()
                cv2.setLogLevel(0)
        except Exception:
            prev_level = None

        try:
            with CAMERA_IO_LOCK:
                if platform.system() == "Windows":
                    dshow_names: list[str] = []
                    try:
                        import comtypes  # type: ignore[import-not-found]
                        from pygrabber.dshow_graph import FilterGraph  # type: ignore[import-not-found]

                        comtypes.CoInitialize()
                        try:
                            dshow_names = list(FilterGraph().get_input_devices())
                        finally:
                            comtypes.CoUninitialize()
                    except Exception:
                        dshow_names = []

                    max_idx = min(len(dshow_names), 4) if dshow_names else 3
                    for idx in range(max_idx):
                        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                        try:
                            if cap.isOpened():
                                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640)
                                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
                                name = dshow_names[idx] if idx < len(dshow_names) else f"Camera {idx}"
                                logical_id = (
                                    "front"
                                    if str(idx) == self.config.front_camera_id
                                    else (
                                        "wrist"
                                        if str(idx) == self.config.wrist_camera_id
                                        else f"cam_{idx}"
                                    )
                                )
                                cameras.append(
                                    {
                                        "id": logical_id,
                                        "raw_id": str(idx),
                                        "index_or_path": idx,
                                        "name": name,
                                        "backend": "DSHOW",
                                        "width": w,
                                        "height": h,
                                        "available": True,
                                    }
                                )
                        finally:
                            cap.release()
                else:
                    dev_paths = sorted(
                        Path("/dev").glob("video*"),
                        key=lambda p: (
                            int(m.group(1))
                            if (m := re.search(r"(\d+)$", p.name))
                            else 999
                        ),
                    )
                    for dev_path in dev_paths[:8]:
                        idx_match = re.search(r"(\d+)$", dev_path.name)
                        idx_str = idx_match.group(1) if idx_match else dev_path.name
                        sys_name_file = Path(f"/sys/class/video4linux/{dev_path.name}/name")
                        name = (
                            sys_name_file.read_text(encoding="utf-8", errors="replace").strip()
                            if sys_name_file.exists()
                            else dev_path.name
                        )
                        cap = cv2.VideoCapture(str(dev_path), cv2.CAP_V4L2)
                        try:
                            if cap.isOpened():
                                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640)
                                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
                                logical_id = (
                                    "front"
                                    if idx_str == self.config.front_camera_id
                                    else (
                                        "wrist"
                                        if idx_str == self.config.wrist_camera_id
                                        else dev_path.name
                                    )
                                )
                                cameras.append(
                                    {
                                        "id": logical_id,
                                        "raw_id": str(dev_path),
                                        "index_or_path": str(dev_path),
                                        "name": name,
                                        "backend": "V4L2",
                                        "width": w,
                                        "height": h,
                                        "available": True,
                                    }
                                )
                        finally:
                            cap.release()
        except Exception:
            pass
        finally:
            if prev_level is not None and hasattr(cv2, "setLogLevel"):
                try:
                    cv2.setLogLevel(prev_level)
                except Exception:
                    pass
        return cameras

    def probe_capabilities(self) -> dict[str, Any]:
        """Inspect OS platform, Python packages, and CLI availability."""
        pkg_names = (
            "lerobot",
            "datasets",
            "rerun",
            "foxglove",
            "pygrabber",
            "lelab",
            "scservo_sdk",
            "torch",
        )
        packages = {name: importlib.util.find_spec(name) is not None for name in pkg_names}
        cli_names = (
            "lerobot-calibrate",
            "lerobot-teleoperate",
            "lerobot-record",
            "lerobot-replay",
            "lerobot-dataset-viz",
            "lerobot-policy-server",
            "ffmpeg",
        )
        cli_tools = {name.replace("-", "_"): _find_cli(name) for name in cli_names}
        sys_name = platform.system()
        is_wsl = False
        if sys_name == "Linux":
            try:
                is_wsl = "microsoft" in platform.release().lower()
            except Exception:
                is_wsl = False
        return {
            "os": sys_name,
            "release": platform.release(),
            "is_wsl": is_wsl,
            "python_version": platform.python_version(),
            "serial_backend": "COM/Win32" if sys_name == "Windows" else "ttyACM/V4L2",
            "camera_backend": "DSHOW" if sys_name == "Windows" else "V4L2",
            "can_run_lerobot": packages["lerobot"],
            "can_run_openpi_local": sys_name == "Linux",
            "can_probe_dshow": sys_name == "Windows",
            "packages": packages,
            "cli_tools": cli_tools,
        }

    def _relabel_cameras(self, cameras: list[dict[str, Any]]) -> None:
        for cam in cameras:
            raw = str(cam.get("raw_id", cam.get("index_or_path", "")))
            if raw == self.config.front_camera_id:
                cam["id"] = "front"
            elif raw == self.config.wrist_camera_id:
                cam["id"] = "wrist"
            else:
                cam["id"] = f"cam_{raw}" if raw.isdigit() else raw

    def auto_match_devices(
        self,
        serial_ports: list[dict[str, Any]],
        cameras: list[dict[str, Any]],
        *,
        force: bool = False,
    ) -> bool:
        """Auto-assign detected candidate serial ports and cameras when missing or forced."""
        changed = False
        cand_devices = [p["device"] for p in serial_ports if p.get("is_robot_candidate")]

        if cand_devices:
            if force or self.config.follower_port not in cand_devices:
                pick = next(
                    (d for d in cand_devices if force or d != self.config.leader_port),
                    cand_devices[0],
                )
                if pick != self.config.follower_port:
                    self.config.follower_port = pick
                    changed = True

            if (
                force
                or self.config.leader_port not in cand_devices
                or self.config.leader_port == self.config.follower_port
            ):
                remaining = [d for d in cand_devices if d != self.config.follower_port]
                if remaining and remaining[0] != self.config.leader_port:
                    self.config.leader_port = remaining[0]
                    changed = True

        cam_raw_ids = [str(c.get("raw_id", c.get("index_or_path", ""))) for c in cameras]
        if cam_raw_ids:
            if force or (
                self.config.front_camera_id not in ("", "none")
                and self.config.front_camera_id not in cam_raw_ids
            ):
                pick = next(
                    (c for c in cam_raw_ids if force or c != self.config.wrist_camera_id),
                    cam_raw_ids[0],
                )
                if pick != self.config.front_camera_id:
                    self.config.front_camera_id = pick
                    changed = True

            if force or (
                self.config.wrist_camera_id not in ("", "none")
                and (
                    self.config.wrist_camera_id not in cam_raw_ids
                    or self.config.wrist_camera_id == self.config.front_camera_id
                )
            ):
                remaining_cams = [c for c in cam_raw_ids if c != self.config.front_camera_id]
                if remaining_cams and remaining_cams[0] != self.config.wrist_camera_id:
                    self.config.wrist_camera_id = remaining_cams[0]
                    changed = True

        self._relabel_cameras(cameras)
        return changed

    def get_calibration_status(self) -> dict[str, Any]:
        follower_file = (
            self.config.calibration_dir
            / "robots"
            / "so_follower"
            / f"{self.config.follower_id}.json"
        )
        leader_file = (
            self.config.calibration_dir
            / "teleoperators"
            / "so_leader"
            / f"{self.config.leader_id}.json"
        )
        return {
            "follower_calibrated": follower_file.is_file(),
            "leader_calibrated": leader_file.is_file(),
            "follower_file": str(follower_file),
            "leader_file": str(leader_file),
        }

    def probe(self, *, auto_match: bool = True) -> dict[str, Any]:
        """Run complete hardware + capability probe and update active mode if auto."""
        serial_ports = self.probe_serial_ports()
        cameras = self.probe_cameras()
        auto_matched = self.auto_match_devices(serial_ports, cameras, force=False) if auto_match else False
        os_capabilities = self.probe_capabilities()
        lerobot_available = bool(os_capabilities["packages"]["lerobot"])

        candidate_ports = [p for p in serial_ports if p["is_robot_candidate"]]
        bluetooth_ports = [p for p in serial_ports if p["is_bluetooth"]]

        missing_hardware: list[str] = []
        if len(candidate_ports) == 0:
            missing_hardware.extend(["follower_arm", "leader_arm"])
        elif len(candidate_ports) == 1:
            missing_hardware.append("leader_arm")

        if any(not p["permission_ok"] for p in candidate_ports):
            missing_hardware.append("serial_permission")

        if len(cameras) == 0:
            missing_hardware.append("camera")

        if not lerobot_available:
            missing_hardware.append("lerobot_package")

        hardware_ready = bool(
            lerobot_available
            and len(candidate_ports) >= 2
            and all(p["permission_ok"] for p in candidate_ports[:2])
        )

        guidance: list[str] = []
        if len(candidate_ports) < 2:
            if platform.system() == "Windows":
                guidance.append(
                    "未检测到双臂 SO-101 USB 串口，已自动降级为 dry_run 模拟模式。"
                    "连接硬件后请运行 scripts/windows/find_ports.ps1 确认 COM 端口并配置 .env。"
                )
            else:
                guidance.append(
                    "未检测到双臂 SO-101 USB 串口（/dev/ttyACM*），已自动降级为 dry_run 模拟模式。"
                    "请检查 USB 连接及 udev/dialout 权限。"
                )
        if bluetooth_ports:
            bt_names = ", ".join(p["device"] for p in bluetooth_ports)
            guidance.append(f"已自动过滤蓝牙虚拟串口（不作为机械臂端口）：{bt_names}")
        if "serial_permission" in missing_hardware:
            guidance.append(
                "检测到串口权限不足，请在 Ubuntu 运行: sudo usermod -aG dialout $USER 并应用 udev 规则。"
            )
        if len(cameras) == 0:
            guidance.append(
                "未检测到可用物理摄像头，相机预览将使用 2D 运动学合成测试图（DRY-RUN）。"
            )
        if hardware_ready:
            guidance.append(
                f"已检测到 SO-101 串口硬件（当前映射 从臂={self.config.follower_port} / 主臂={self.config.leader_port}），可在下方手动调整并保存。"
            )

        if self.requested_mode == "dry_run":
            self.active_mode = "dry_run"
        elif self.requested_mode in ("auto", "real"):
            self.active_mode = "real" if hardware_ready else "dry_run"

        result = {
            "serial_ports": serial_ports,
            "cameras": cameras,
            "lerobot_available": lerobot_available,
            "os_capabilities": os_capabilities,
            "hardware_ready": hardware_ready,
            "missing_hardware": missing_hardware,
            "guidance": guidance,
            "active_mode": self.active_mode,
            "requested_mode": self.requested_mode,
            "device_selection": {
                "follower_port": self.config.follower_port,
                "leader_port": self.config.leader_port,
                "follower_id": self.config.follower_id,
                "leader_id": self.config.leader_id,
                "front_camera_id": self.config.front_camera_id,
                "wrist_camera_id": self.config.wrist_camera_id,
                "auto_matched": auto_matched,
            },
            "calibration_status": self.get_calibration_status(),
        }
        self.last_probe = result
        return result

    def set_mode(self, mode: ModeType) -> tuple[bool, dict[str, Any]]:
        """Switch operating mode cleanly; refuse real mode with 409 payload if hardware missing."""
        probe_data = self.last_probe if (mode == "dry_run" and self.last_probe) else self.probe()
        hardware_ready = bool(probe_data["hardware_ready"])

        if mode == "dry_run":
            self.requested_mode = "dry_run"
            self.active_mode = "dry_run"
            if self.last_probe:
                self.last_probe["requested_mode"] = "dry_run"
                self.last_probe["active_mode"] = "dry_run"
            return True, {
                "requested_mode": "dry_run",
                "mode": "dry_run",
                "hardware_ready": hardware_ready,
                "missing_hardware": probe_data["missing_hardware"],
                "guidance": probe_data["guidance"],
                "message": "Switched to dry_run simulation mode.",
            }

        if mode == "auto":
            self.requested_mode = "auto"
            self.active_mode = "real" if hardware_ready else "dry_run"
            return True, {
                "requested_mode": "auto",
                "mode": self.active_mode,
                "hardware_ready": hardware_ready,
                "missing_hardware": probe_data["missing_hardware"],
                "guidance": probe_data["guidance"],
                "message": f"Auto mode resolved to {self.active_mode}.",
            }

        # mode == "real"
        if not hardware_ready:
            self.active_mode = "dry_run"
            return False, {
                "detail": "Cannot switch to real mode: required arm101 hardware not detected.",
                "code": "HARDWARE_NOT_READY",
                "requested_mode": "real",
                "mode": self.active_mode,
                "hardware_ready": False,
                "missing_hardware": probe_data["missing_hardware"],
                "guidance": probe_data["guidance"],
            }

        self.requested_mode = "real"
        self.active_mode = "real"
        return True, {
            "requested_mode": "real",
            "mode": "real",
            "hardware_ready": True,
            "missing_hardware": probe_data["missing_hardware"],
            "guidance": probe_data["guidance"],
            "message": "Switched to real hardware mode.",
        }
