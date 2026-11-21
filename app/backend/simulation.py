"""SO-101 6-DOF kinematic state simulator, strict joint validator, and JPEG HUD renderer."""

from __future__ import annotations

import io
import math
import threading
import time
from datetime import datetime, timezone
from typing import Any

from PIL import Image, ImageDraw


# Canonical SO-101 6-DOF joints aligned with repos/lerobot/src/lerobot/robots/so_follower/so_follower.py
JOINT_SPECS: dict[str, tuple[float, float, float]] = {
    "shoulder_pan": (-180.0, 180.0, 0.0),
    "shoulder_lift": (-180.0, 180.0, -25.0),
    "elbow_flex": (-180.0, 180.0, 45.0),
    "wrist_flex": (-180.0, 180.0, 30.0),
    "wrist_roll": (-180.0, 180.0, 0.0),
    "gripper": (0.0, 100.0, 20.0),
}

JOINT_NAMES: tuple[str, ...] = tuple(JOINT_SPECS.keys())


def default_joints() -> dict[str, float]:
    """Return a fresh copy of canonical home joint angles."""
    return {name: spec[2] for name, spec in JOINT_SPECS.items()}


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


class Arm101Simulator:
    """Thread-safe 6-DOF SO-101 leader/follower simulator and JPEG frame generator."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.leader_joints: dict[str, float] = default_joints()
        self.follower_joints: dict[str, float] = default_joints()
        self.step_count: int = 0
        self._frame_counters: dict[str, int] = {"front": 0, "wrist": 0}
        self._started_at = time.monotonic()

    def reset(self) -> None:
        with self._lock:
            self.leader_joints = default_joints()
            self.follower_joints = default_joints()
            self.step_count = 0

    def set_leader_joints(self, updates: dict[str, float], step_immediately: bool = True) -> None:
        with self._lock:
            for name, val in updates.items():
                if name in JOINT_SPECS:
                    self.leader_joints[name] = float(val)
            if step_immediately:
                self.step(0.05)

    def set_follower_joints(self, updates: dict[str, float]) -> None:
        with self._lock:
            for name, val in updates.items():
                if name in JOINT_SPECS:
                    self.follower_joints[name] = float(val)

    def step(self, dt: float = 0.05) -> None:
        """Critically damped exponential convergence from leader to follower."""
        alpha = min(1.0, max(0.05, dt * 12.0))
        with self._lock:
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

        # Engineering background grid
        for x in range(0, width, 40):
            draw.line([(x, 44), (x, height)], fill=(30, 41, 59), width=1)
        for y in range(44, height, 40):
            draw.line([(0, y), (width, y)], fill=(30, 41, 59), width=1)

        # Header HUD bar
        draw.rectangle([(0, 0), (width, 44)], fill=(30, 41, 59))
        badge_color = (245, 158, 11) if mode == "dry_run" else (16, 185, 129)
        draw.rectangle([(12, 10), (132, 34)], fill=badge_color)
        draw.text((20, 16), f"MODE: {mode.upper()}", fill=(15, 23, 42))
        draw.text(
            (148, 16),
            f"ARM101 CAM:{cam_id.upper()} | WF:{active_workflow.upper()} | #{count}",
            fill=(226, 232, 240),
        )

        # 2D kinematic projection
        pan = math.radians(joints["shoulder_pan"])
        lift = math.radians(joints["shoulder_lift"] - 65.0)
        elbow = math.radians(joints["elbow_flex"])
        wrist = math.radians(joints["wrist_flex"])
        roll = math.radians(joints["wrist_roll"])
        grip_pct = joints["gripper"] / 100.0

        if cam_id == "wrist":
            # Wrist-eye perspective: gripper jaw + target cube offset by pan/wrist
            cx = int(width * 0.5 + math.sin(pan) * 90)
            cy = int(height * 0.54 + math.sin(wrist) * 45)
            # Simulated workspace target object
            draw.rectangle([(cx - 28, cy - 28), (cx + 28, cy + 28)], outline=(6, 182, 212), width=3)
            draw.text((cx - 22, cy - 8), "CUBE", fill=(6, 182, 212))
            # Parallel gripper fingers rotated by wrist_roll
            jaw_gap = 18 + int(grip_pct * 55)
            dx = math.cos(roll)
            dy = math.sin(roll)
            nx, ny = -dy, dx
            for sign in (-1, 1):
                px = int(width * 0.5 + nx * jaw_gap * sign)
                py = int(height * 0.56 + ny * jaw_gap * sign)
                draw.rectangle([(px - 10, py - 24), (px + 10, py + 24)], fill=(248, 250, 252))
        else:
            # Front/side elevation linkage
            bx, by = int(width * 0.36 + math.sin(pan) * 40), int(height * 0.78)
            draw.rectangle([(bx - 42, by), (bx + 42, by + 22)], fill=(71, 85, 105))

            l1, l2, l3 = 115.0, 100.0, 58.0
            a1 = lift
            x1 = bx + l1 * math.cos(a1)
            y1 = by + l1 * math.sin(a1)
            a2 = a1 + elbow
            x2 = x1 + l2 * math.cos(a2)
            y2 = y1 + l2 * math.sin(a2)
            a3 = a2 + wrist
            x3 = x2 + l3 * math.cos(a3)
            y3 = y2 + l3 * math.sin(a3)

            draw.line([(bx, by), (x1, y1)], fill=(6, 182, 212), width=10)
            draw.line([(x1, y1), (x2, y2)], fill=(56, 189, 248), width=8)
            draw.line([(x2, y2), (x3, y3)], fill=(16, 185, 129), width=6)

            for jx, jy, r in ((bx, by, 9), (x1, y1, 8), (x2, y2, 7), (x3, y3, 6)):
                draw.ellipse([(jx - r, jy - r), (jx + r, jy + r)], fill=(245, 158, 11))

            # Gripper jaws
            jaw_open = 6 + grip_pct * 22
            px_perp = -math.sin(a3)
            py_perp = math.cos(a3)
            for sign in (-1, 1):
                gx0 = x3 + px_perp * jaw_open * sign
                gy0 = y3 + py_perp * jaw_open * sign
                gx1 = gx0 + 20 * math.cos(a3)
                gy1 = gy0 + 20 * math.sin(a3)
                draw.line([(gx0, gy0), (gx1, gy1)], fill=(248, 250, 252), width=4)

        # Telemetry box on right/bottom
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
