"""SO-101 6-DOF kinematic state simulator, FK/IK solver (backed by software/core/kinematics.py), and JPEG HUD renderer."""

from __future__ import annotations

import io
import math
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from PIL import Image, ImageDraw

# Ensure `software/` is importable so we share the exact verified SO101Kinematics kernel
_SOFTWARE_DIR = str(Path(__file__).resolve().parents[2] / "software")
if _SOFTWARE_DIR not in sys.path:
    sys.path.append(_SOFTWARE_DIR)

try:
    from core.data_types import JointUnit as _SoftwareJointUnit
    from core.kinematics import (
        KinematicsConfig as _KinematicsConfig,
        SO101Kinematics as _SO101Kinematics,
        solve_horizontal_planar_reach as _solve_horizontal_planar_reach,
    )

    _KINEMATICS_SOLVER = _SO101Kinematics(_KinematicsConfig())
except Exception:  # pragma: no cover - fallback if software/ unavailable
    _SoftwareJointUnit = None
    _SO101Kinematics = None
    _solve_horizontal_planar_reach = None
    _KINEMATICS_SOLVER = None


# Canonical SO-101 6-DOF joints aligned with repos/lerobot & software/core/kinematics.py
# (min, max, official homing default: upper arm upright, forearm & gripper horizontal forward)
JOINT_SPECS: dict[str, tuple[float, float, float]] = {
    "shoulder_pan": (-180.0, 180.0, 0.0),
    "shoulder_lift": (-180.0, 180.0, 0.0),
    "elbow_flex": (-180.0, 180.0, 0.0),
    "wrist_flex": (-180.0, 180.0, 0.0),
    "wrist_roll": (-180.0, 180.0, 0.0),
    "gripper": (0.0, 100.0, 20.0),
}

JOINT_NAMES: tuple[str, ...] = tuple(JOINT_SPECS.keys())

# Verified physical SO-ARM101 link dimensions (mm) from software/core/kinematics.py
LINK_LENGTHS_MM: dict[str, float] = {
    "base_height": 110.0,   # L1: table/base to shoulder_lift pitch axis
    "upper_arm": 135.0,     # L2: shoulder_lift -> elbow_flex
    "forearm": 125.0,       # L3: elbow_flex -> wrist_flex
    "gripper_tcp": 71.0,    # L4: wrist_flex -> gripper TCP
}


def default_joints() -> dict[str, float]:
    """Return a fresh copy of canonical official homing joint angles."""
    return {name: spec[2] for name, spec in JOINT_SPECS.items()}


def deg_to_encoder(joint: str, value: float) -> int:
    """Estimate 12-bit STS3215 encoder tick (0..4095, center 2048) from degrees / gripper %."""
    if joint == "gripper":
        return max(0, min(4095, int(round(2048 + (value - 50.0) * 20.0))))
    return max(0, min(4095, int(round(2048 + value * (4096.0 / 360.0)))))


def encoder_to_deg(joint: str, raw_pos: float) -> float:
    """Convert raw 12-bit STS3215 encoder tick to degrees (or 0..100% for gripper)."""
    if joint == "gripper":
        return round(max(0.0, min(100.0, (raw_pos / 4095.0) * 100.0)), 2)
    deg = (raw_pos - 2048.0) * (360.0 / 4096.0)
    return round(max(-180.0, min(180.0, deg)), 2)


def validate_joint_dict(raw: Any, *, require_non_empty: bool = True) -> dict[str, float]:
    """Validate partial or full 6-DOF SO-101 joint dictionary (or 6-element sequence)."""
    if isinstance(raw, (list, tuple)):
        if len(raw) != len(JOINT_NAMES):
            raise ValueError(
                f"Joints list must contain exactly {len(JOINT_NAMES)} values ({', '.join(JOINT_NAMES)})."
            )
        raw = dict(zip(JOINT_NAMES, raw))
    if not isinstance(raw, dict):
        raise ValueError("Joints payload must be a JSON object mapping joint names to numbers.")
    if require_non_empty and len(raw) == 0:
        raise ValueError("At least one SO-101 joint must be specified.")

    validated: dict[str, float] = {}
    for key, val in raw.items():
        if key not in JOINT_SPECS:
            raise ValueError(
                f"Unknown joint '{key}'. Allowed joints: {', '.join(JOINT_NAMES)}."
            )
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            raise ValueError(f"Joint '{key}' must be a finite number (got {type(val).__name__}).")
        fval = float(val)
        if not math.isfinite(fval):
            raise ValueError(f"Joint '{key}' must be finite (got {val}).")
        lo, hi, _ = JOINT_SPECS[key]
        if fval < lo or fval > hi:
            raise ValueError(f"Joint '{key}'={fval} out of valid range [{lo}, {hi}].")
        validated[key] = round(fval, 3)
    return validated


def compute_fk_details(joints: dict[str, float]) -> dict[str, Any]:
    """Compute exact 3D joint positions (mm), TCP pose, encoder estimates, and homing alignment."""
    pan = math.radians(float(joints.get("shoulder_pan", 0.0)))
    lift = math.radians(float(joints.get("shoulder_lift", 0.0)))
    elbow = math.radians(float(joints.get("elbow_flex", 0.0)))
    wrist_p = math.radians(float(joints.get("wrist_flex", 0.0)))
    wrist_r = float(joints.get("wrist_roll", 0.0))
    grip = float(joints.get("gripper", 20.0))

    L1 = LINK_LENGTHS_MM["base_height"]
    L2 = LINK_LENGTHS_MM["upper_arm"]
    L3 = LINK_LENGTHS_MM["forearm"]
    L4 = LINK_LENGTHS_MM["gripper_tcp"]

    # Aligned with software/core/kinematics.py & official _HOMING_POSITION_DIAGRAM:
    # lift=0 -> upper arm straight up (+Z); elbow=0 -> forearm horizontal forward (+R); wrist_p=0 -> TCP horizontal forward
    phi2 = lift
    phi3 = phi2 + math.pi / 2.0 + elbow
    phi4 = phi3 + wrist_p

    r1, z1 = 0.0, L1
    r2, z2 = L2 * math.sin(phi2), L1 + L2 * math.cos(phi2)
    r3, z3 = r2 + L3 * math.sin(phi3), z2 + L3 * math.cos(phi3)
    r4, z4 = r3 + L4 * math.sin(phi4), z3 + L4 * math.cos(phi4)

    cp, sp = math.cos(pan), math.sin(pan)
    nodes = {
        "base": [0.0, 0.0, 0.0],
        "shoulder": [0.0, 0.0, round(z1, 2)],
        "elbow": [round(r2 * cp, 2), round(r2 * sp, 2), round(z2, 2)],
        "wrist": [round(r3 * cp, 2), round(r3 * sp, 2), round(z3, 2)],
        "tcp": [round(r4 * cp, 2), round(r4 * sp, 2), round(z4, 2)],
    }
    pitch_deg = round(math.degrees(phi4 - math.pi / 2.0), 2)
    body_errors = [abs(float(joints.get(k, 0.0))) for k in JOINT_NAMES[:5]]
    max_homing_err = round(max(body_errors), 2) if body_errors else 0.0

    return {
        "nodes_mm": nodes,
        "link_lengths_mm": dict(LINK_LENGTHS_MM),
        "tcp_pose": {
            "x_mm": nodes["tcp"][0],
            "y_mm": nodes["tcp"][1],
            "z_mm": nodes["tcp"][2],
            "r_mm": round(r4, 2),
            "pitch_deg": pitch_deg,
            "roll_deg": round(wrist_r, 2),
            "gripper_pct": round(grip, 2),
        },
        "encoders": {k: deg_to_encoder(k, float(joints.get(k, 0.0))) for k in JOINT_NAMES},
        "homing_max_error_deg": max_homing_err,
        "homing_aligned": max_homing_err <= 8.0,
    }


def solve_ik_target(
    *,
    x_mm: float,
    y_mm: float,
    z_mm: float,
    current_joints: dict[str, float],
    pitch_deg: float | None = None,
    gripper: float | None = None,
) -> dict[str, float]:
    """Solve 6-DOF SO-101 IK using aguiya's verified SO101Kinematics (mm -> degrees)."""
    cur_fk = compute_fk_details(current_joints)
    target_pitch_rad = (
        math.radians(pitch_deg)
        if pitch_deg is not None
        else math.radians(cur_fk["tcp_pose"]["pitch_deg"])
    )
    target_roll_rad = math.radians(float(current_joints.get("wrist_roll", 0.0)))
    target_grip = (
        max(0.0, min(100.0, float(gripper)))
        if gripper is not None
        else float(current_joints.get("gripper", 20.0))
    )

    if _KINEMATICS_SOLVER is not None:
        ok, solved_rad, err_msg = _KINEMATICS_SOLVER.inverse_kinematics(
            target_xyz=(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0),
            current_joint_positions={
                k: math.radians(float(current_joints.get(k, 0.0))) for k in JOINT_NAMES[:5]
            },
            desired_pitch_rad=target_pitch_rad,
            preserve_wrist_roll=target_roll_rad,
            preserve_gripper=target_grip,
        )
        if not ok or solved_rad is None:
            raise ValueError(
                err_msg
                or f"目标点 ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm 超出 SO-101 物理工作空间范围。"
            )
        result: dict[str, float] = {}
        max_step = 24.0
        for k in JOINT_NAMES[:5]:
            lo, hi, _ = JOINT_SPECS[k]
            deg_val = max(lo, min(hi, math.degrees(float(solved_rad[k]))))
            cur_val = float(current_joints.get(k, 0.0))
            delta = deg_val - cur_val
            if abs(delta) > max_step:
                deg_val = cur_val + math.copysign(max_step, delta)
            result[k] = round(max(lo, min(hi, deg_val)), 2)
        result["gripper"] = round(target_grip, 2)
        return result

    raise RuntimeError("SO101Kinematics solver unavailable.")


class Arm101Simulator:
    """Thread-safe 6-DOF SO-101 dual-arm simulator, FK/IK controller, and JPEG HUD renderer."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.leader_joints: dict[str, float] = default_joints()
        self.follower_joints: dict[str, float] = default_joints()
        self.linked_arms: bool = True
        self.torque_enabled: bool = True
        self.estop_latched: bool = False
        self.max_step_deg: float = 15.0
        self.step_count: int = 0
        self._frame_counters: dict[str, int] = {"front": 0, "wrist": 0}
        self._started_at = time.monotonic()

    def reset(self) -> None:
        with self._lock:
            self.leader_joints = default_joints()
            self.follower_joints = default_joints()
            self.step_count = 0

    def set_linked_arms(self, linked: bool) -> bool:
        with self._lock:
            self.linked_arms = bool(linked)
            return self.linked_arms

    def set_leader_joints(self, updates: dict[str, float], step_immediately: bool = True) -> None:
        with self._lock:
            for name, val in updates.items():
                if name in JOINT_SPECS:
                    self.leader_joints[name] = round(float(val), 3)
            if step_immediately and self.linked_arms and not self.estop_latched:
                self.step(0.05)

    def set_follower_joints(self, updates: dict[str, float]) -> None:
        with self._lock:
            if self.estop_latched:
                return
            for name, val in updates.items():
                if name in JOINT_SPECS:
                    self.follower_joints[name] = round(float(val), 3)
            self.step_count += 1

    def set_arm_joints(
        self,
        arm: Literal["leader", "follower", "both"],
        updates: dict[str, float],
        *,
        step_immediately: bool = True,
    ) -> tuple[dict[str, float], dict[str, float]]:
        with self._lock:
            if arm == "leader":
                self.set_leader_joints(updates, step_immediately=step_immediately)
            elif arm == "follower":
                self.set_follower_joints(updates)
            else:
                for name, val in updates.items():
                    if name in JOINT_SPECS:
                        v = round(float(val), 3)
                        self.leader_joints[name] = v
                        if not self.estop_latched:
                            self.follower_joints[name] = v
                self.step_count += 1
            return dict(self.leader_joints), dict(self.follower_joints)

    def step_cartesian_key(self, key: str, step_mm: float = 8.0) -> dict[str, Any]:
        """Execute aguiya's WASD height-locked Cartesian / wrist / gripper key control on follower."""
        k = (key or "").strip().lower()
        with self._lock:
            if self.estop_latched:
                raise RuntimeError("急停已锁定 (E-STOP)，请先点击解除急停。")
            cur = dict(self.follower_joints)
            fk = compute_fk_details(cur)
            tcp = fk["tcp_pose"]

            if k == " ":
                self.estop_latched = True
                self.torque_enabled = False
                return {"action": "estop", "follower_joints": dict(self.follower_joints)}
            if k == "m":
                self.follower_joints = default_joints()
                if self.linked_arms:
                    self.leader_joints = default_joints()
                self.step_count += 1
                return {"action": "home", "follower_joints": dict(self.follower_joints)}
            if k in ("a", "d"):
                delta = step_mm * 0.6 * (1.0 if k == "a" else -1.0)
                cur["shoulder_pan"] = max(-110.0, min(110.0, round(cur["shoulder_pan"] + delta, 2)))
            elif k in ("j", "k"):
                delta = 4.0 * (-1.0 if k == "j" else 1.0)
                cur["wrist_flex"] = max(-95.0, min(95.0, round(cur["wrist_flex"] + delta, 2)))
            elif k in ("u", "i"):
                delta = 6.0 * (1.0 if k == "u" else -1.0)
                cur["wrist_roll"] = max(-157.0, min(163.0, round(cur["wrist_roll"] + delta, 2)))
            elif k in ("c", "v"):
                delta = 8.0 * (1.0 if k == "c" else -1.0)
                cur["gripper"] = max(0.0, min(100.0, round(cur["gripper"] + delta, 2)))
            elif k in ("w", "s", "r", "f"):
                pan_rad = math.radians(cur["shoulder_pan"])
                r_now = tcp["r_mm"]
                z_now = tcp["z_mm"]
                if k == "w":
                    r_tgt, z_tgt = r_now + step_mm, z_now
                elif k == "s":
                    r_tgt, z_tgt = max(60.0, r_now - step_mm), z_now
                elif k == "r":
                    r_tgt, z_tgt = r_now, z_now + step_mm
                else:
                    r_tgt, z_tgt = r_now, max(30.0, z_now - step_mm)

                x_tgt = r_tgt * math.cos(pan_rad)
                y_tgt = r_tgt * math.sin(pan_rad)
                cur = solve_ik_target(
                    x_mm=x_tgt,
                    y_mm=y_tgt,
                    z_mm=z_tgt,
                    current_joints=cur,
                    pitch_deg=tcp["pitch_deg"],
                    gripper=cur["gripper"],
                )
            else:
                raise ValueError(f"Unsupported control key '{key}'.")

            self.follower_joints = cur
            if self.linked_arms:
                self.leader_joints = dict(cur)
            self.step_count += 1
            return {
                "action": f"key:{k}",
                "follower_joints": dict(self.follower_joints),
                "leader_joints": dict(self.leader_joints),
                "fk": compute_fk_details(self.follower_joints),
            }

    def step(self, dt: float = 0.05) -> None:
        """Critically damped exponential convergence from leader to follower."""
        alpha = min(1.0, max(0.05, dt * 12.0))
        with self._lock:
            if self.estop_latched or not self.linked_arms:
                return
            for name in JOINT_NAMES:
                cur = self.follower_joints[name]
                tgt = self.leader_joints[name]
                self.follower_joints[name] = round(cur + (tgt - cur) * alpha, 3)
            self.step_count += 1

    def snapshot(self) -> tuple[dict[str, float], dict[str, float], int]:
        with self._lock:
            return dict(self.leader_joints), dict(self.follower_joints), self.step_count

    def render_camera_jpeg(
        self,
        cam_id: str,
        *,
        mode: str = "dry_run",
        active_workflow: str = "idle",
        width: int = 640,
        height: int = 480,
    ) -> bytes:
        """Render an authentic 2D kinematic projection + HUD telemetry frame as JPEG bytes."""
        with self._lock:
            joints = dict(self.follower_joints)
            count = self._frame_counters.get(cam_id, 0) + 1
            self._frame_counters[cam_id] = count

        img = Image.new("RGB", (width, height), color=(15, 23, 42))
        draw = ImageDraw.Draw(img)

        for x in range(0, width, 40):
            draw.line([(x, 44), (x, height)], fill=(30, 41, 59), width=1)
        for y in range(44, height, 40):
            draw.line([(0, y), (width, y)], fill=(30, 41, 59), width=1)

        draw.rectangle([(0, 0), (width, 44)], fill=(30, 41, 59))
        badge_color = (245, 158, 11) if mode == "dry_run" else (16, 185, 129)
        draw.rectangle([(12, 10), (132, 34)], fill=badge_color)
        draw.text((20, 16), f"MODE: {mode.upper()}", fill=(15, 23, 42))
        draw.text(
            (148, 16),
            f"ARM101 CAM:{cam_id.upper()} | WF:{active_workflow.upper()} | #{count}",
            fill=(226, 232, 240),
        )

        pan = math.radians(joints["shoulder_pan"])
        lift = math.radians(joints["shoulder_lift"] - 90.0)
        elbow = math.radians(joints["elbow_flex"] + 90.0)
        wrist = math.radians(joints["wrist_flex"])
        roll = math.radians(joints["wrist_roll"])
        grip_pct = joints["gripper"] / 100.0

        if cam_id == "wrist":
            cx = int(width * 0.5 + math.sin(pan) * 90)
            cy = int(height * 0.54 + math.sin(wrist) * 45)
            draw.rectangle([(cx - 28, cy - 28), (cx + 28, cy + 28)], outline=(6, 182, 212), width=3)
            draw.text((cx - 22, cy - 8), "CUBE", fill=(6, 182, 212))
            jaw_gap = 18 + int(grip_pct * 55)
            dx, dy = math.cos(roll), math.sin(roll)
            nx, ny = -dy, dx
            for sign in (-1, 1):
                px = int(width * 0.5 + nx * jaw_gap * sign)
                py = int(height * 0.56 + ny * jaw_gap * sign)
                draw.rectangle([(px - 10, py - 24), (px + 10, py + 24)], fill=(248, 250, 252))
        else:
            bx, by = int(width * 0.36 + math.sin(pan) * 40), int(height * 0.78)
            draw.rectangle([(bx - 42, by), (bx + 42, by + 22)], fill=(71, 85, 105))

            l1, l2, l3 = 115.0, 105.0, 60.0
            a1 = lift
            x1, y1 = bx + l1 * math.cos(a1), by + l1 * math.sin(a1)
            a2 = a1 + elbow
            x2, y2 = x1 + l2 * math.cos(a2), y1 + l2 * math.sin(a2)
            a3 = a2 + wrist
            x3, y3 = x2 + l3 * math.cos(a3), y2 + l3 * math.sin(a3)

            draw.line([(bx, by), (x1, y1)], fill=(6, 182, 212), width=10)
            draw.line([(x1, y1), (x2, y2)], fill=(56, 189, 248), width=8)
            draw.line([(x2, y2), (x3, y3)], fill=(16, 185, 129), width=6)

            for jx, jy, r in ((bx, by, 9), (x1, y1, 8), (x2, y2, 7), (x3, y3, 6)):
                draw.ellipse([(jx - r, jy - r), (jx + r, jy + r)], fill=(245, 158, 11))

            jaw_open = 6 + grip_pct * 22
            px_perp, py_perp = -math.sin(a3), math.cos(a3)
            for sign in (-1, 1):
                gx0 = x3 + px_perp * jaw_open * sign
                gy0 = y3 + py_perp * jaw_open * sign
                gx1 = gx0 + 20 * math.cos(a3)
                gy1 = gy0 + 20 * math.sin(a3)
                draw.line([(gx0, gy0), (gx1, gy1)], fill=(248, 250, 252), width=4)

        draw.rectangle([(12, height - 102), (332, height - 12)], fill=(15, 23, 42), outline=(51, 65, 85))
        ts = datetime.now(timezone.utc).strftime("%H:%M:%S UTC")
        draw.text(
            (22, height - 94),
            f"pan:{joints['shoulder_pan']:6.1f}  lift:{joints['shoulder_lift']:6.1f}",
            fill=(226, 232, 240),
        )
        draw.text(
            (22, height - 74),
            f"elb:{joints['elbow_flex']:6.1f}  wflx:{joints['wrist_flex']:6.1f}",
            fill=(226, 232, 240),
        )
        draw.text(
            (22, height - 54),
            f"wrol:{joints['wrist_roll']:5.1f}  grip:{joints['gripper']:5.1f}%",
            fill=(226, 232, 240),
        )
        draw.text((22, height - 32), f"step:{self.step_count} | {ts}", fill=(148, 163, 184))

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        return buf.getvalue()
