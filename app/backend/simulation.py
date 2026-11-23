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


# Canonical SO-101 6-DOF joints aligned with repos/lerobot & repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf
JOINT_SPECS: dict[str, tuple[float, float, float]] = {
    "shoulder_pan": (-110.0, 110.0, 0.0),
    "shoulder_lift": (-100.0, 100.0, 0.0),
    "elbow_flex": (-97.0, 97.0, 0.0),
    "wrist_flex": (-95.0, 95.0, 0.0),
    "wrist_roll": (-157.21, 162.79, 0.0),
    "gripper": (0.0, 100.0, 20.0),
}

JOINT_NAMES: tuple[str, ...] = tuple(JOINT_SPECS.keys())

# Verified physical SO-ARM101 link dimensions (mm) from so101_new_calib.urdf
LINK_LENGTHS_MM: dict[str, float] = {
    "base_height": 116.6,   # L1: base -> shoulder_lift
    "upper_arm": 116.0,     # L2: shoulder_lift -> elbow_flex
    "forearm": 135.0,       # L3: elbow_flex -> wrist_flex
    "gripper_tcp": 160.5,   # L4: wrist_flex -> gripper_frame_link TCP
}


def default_joints() -> dict[str, float]:
    """Return a fresh copy of canonical official homing joint angles."""
    return {name: spec[2] for name, spec in JOINT_SPECS.items()}


def deg_to_encoder(
    joint: str,
    value: float,
    *,
    range_min: int | None = None,
    range_max: int | None = None,
    drive_mode: int = 0,
) -> int:
    """Convert degrees (or 0..100% for gripper) to 12-bit STS3215 encoder tick (0..4095, half-turn center 2047)."""
    r_min = 0 if range_min is None else int(range_min)
    r_max = 4095 if range_max is None else int(range_max)
    span = max(1, r_max - r_min)
    if joint == "gripper":
        pct = max(0.0, min(100.0, float(value)))
        norm = (100.0 - pct) if drive_mode else pct
        tick = int(round(r_min + (norm / 100.0) * span))
        return max(r_min, min(r_max, tick))
    mid = (float(r_min) + float(r_max)) / 2.0 if (range_min is not None and range_max is not None) else 2047.0
    sign = -1.0 if drive_mode else 1.0
    tick = int(round(mid + sign * float(value) * (4095.0 / 360.0)))
    return max(r_min, min(r_max, tick))


def encoder_to_deg(
    joint: str,
    raw_pos: float,
    *,
    range_min: int | None = None,
    range_max: int | None = None,
    drive_mode: int = 0,
) -> float:
    """Convert raw 12-bit STS3215 encoder tick to degrees (or 0..100% for gripper)."""
    r_min = 0 if range_min is None else int(range_min)
    r_max = 4095 if range_max is None else int(range_max)
    span = max(1, r_max - r_min)
    if joint == "gripper":
        pct = ((float(raw_pos) - r_min) / span) * 100.0
        if drive_mode:
            pct = 100.0 - pct
        return round(max(0.0, min(100.0, pct)), 2)
    mid = (float(r_min) + float(r_max)) / 2.0 if (range_min is not None and range_max is not None) else 2047.0
    sign = -1.0 if drive_mode else 1.0
    deg = sign * (float(raw_pos) - mid) * (360.0 / 4095.0)
    lo, hi, _ = JOINT_SPECS.get(joint, (-180.0, 180.0, 0.0))
    return round(max(lo, min(hi, deg)), 2)


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
    """Compute exact 3D URDF joint positions (mm), TCP pose, encoder estimates, and homing alignment."""
    wrist_r = float(joints.get("wrist_roll", 0.0))
    grip = float(joints.get("gripper", 20.0))

    if _KINEMATICS_SOLVER is not None and _SoftwareJointUnit is not None:
        (tx, ty, tz), pitch_rad, _ = _KINEMATICS_SOLVER.forward_kinematics(
            joints, unit=_SoftwareJointUnit.DEGREE
        )
        j_pos = _KINEMATICS_SOLVER.forward_kinematics_links(
            joints, unit=_SoftwareJointUnit.DEGREE
        )

        def _mm_list(arr: Any) -> list[float]:
            return [round(float(arr[0]) * 1000.0, 5), round(float(arr[1]) * 1000.0, 5), round(float(arr[2]) * 1000.0, 5)]

        nodes = {
            "base": [0.0, 0.0, 0.0],
            "shoulder": _mm_list(j_pos.get("shoulder_lift", (0.0, 0.0, 0.1166))),
            "elbow": _mm_list(j_pos.get("elbow_flex", (0.0, 0.0, 0.2326))),
            "wrist": _mm_list(j_pos.get("wrist_flex", (0.135, 0.0, 0.2326))),
            "tcp": [round(tx * 1000.0, 5), round(ty * 1000.0, 5), round(tz * 1000.0, 5)],
        }
        r_mm = math.hypot(nodes["tcp"][0], nodes["tcp"][1])
        pitch_deg = round(math.degrees(pitch_rad), 2)
    else:
        nodes = {
            "base": [0.0, 0.0, 0.0],
            "shoulder": [8.44, -18.28, 116.6],
            "elbow": [36.44, -18.28, 229.17],
            "wrist": [171.34, -13.08, 229.17],
            "tcp": [391.36147, -0.00921, 226.46971],
        }
        r_mm = math.hypot(nodes["tcp"][0], nodes["tcp"][1])
        pitch_deg = 0.0

    body_errors = [abs(float(joints.get(k, 0.0))) for k in JOINT_NAMES[:5]]
    max_homing_err = round(max(body_errors), 2) if body_errors else 0.0

    return {
        "nodes_mm": nodes,
        "link_lengths_mm": dict(LINK_LENGTHS_MM),
        "tcp_pose": {
            "x_mm": nodes["tcp"][0],
            "y_mm": nodes["tcp"][1],
            "z_mm": nodes["tcp"][2],
            "r_mm": round(r_mm, 3),
            "pitch_deg": pitch_deg,
            "roll_deg": round(wrist_r, 2),
            "gripper_pct": round(grip, 2),
        },
        "encoders": {k: deg_to_encoder(k, float(joints.get(k, 0.0))) for k in JOINT_NAMES},
        "homing_max_error_deg": max_homing_err,
        "homing_aligned": max_homing_err <= 8.0,
    }


def _finalize_ik_result(
    solved_rad: dict[str, float],
    current_joints: dict[str, float],
    target_grip: float,
    max_step: float = 24.0,
) -> dict[str, float]:
    # Scale joint deltas proportionally if any single joint exceeds max_step
    # so the TCP stays on the continuous IK path instead of distorting downward.
    deltas: dict[str, float] = {}
    max_abs_delta = 0.0
    for k in JOINT_NAMES[:4]:
        lo, hi, _ = JOINT_SPECS[k]
        deg_val = max(lo, min(hi, math.degrees(float(solved_rad[k]))))
        cur_val = float(current_joints.get(k, 0.0))
        d = deg_val - cur_val
        deltas[k] = d
        if abs(d) > max_abs_delta:
            max_abs_delta = abs(d)

    scale = (max_step / max_abs_delta) if max_abs_delta > max_step else 1.0
    result: dict[str, float] = {}
    for k in JOINT_NAMES[:4]:
        lo, hi, _ = JOINT_SPECS[k]
        cur_val = float(current_joints.get(k, 0.0))
        result[k] = round(max(lo, min(hi, cur_val + deltas[k] * scale)), 2)

    # 纯 XYZ 位置 IK 严格锁定 #5 wrist_roll，禁止利用 7.9mm 偏心乱转腕部
    lo_r, hi_r, _ = JOINT_SPECS["wrist_roll"]
    result["wrist_roll"] = round(max(lo_r, min(hi_r, float(current_joints.get("wrist_roll", 0.0)))), 2)
    result["gripper"] = round(target_grip, 2)
    return result


def solve_ik_target_with_status(
    *,
    x_mm: float,
    y_mm: float,
    z_mm: float,
    current_joints: dict[str, float],
    pitch_deg: float | None = None,
    gripper: float | None = None,
    clamp_to_workspace: bool = False,
    lock_pitch: bool = True,
) -> tuple[dict[str, float], bool, str]:
    """Solve 5-DOF+Gripper SO-101 IK (locking #5 wrist_roll); optionally clamp to URDF workspace boundary."""
    cur_fk = compute_fk_details(current_joints)
    target_pitch_rad: float | None
    if not lock_pitch:
        target_pitch_rad = None
    elif pitch_deg is not None:
        target_pitch_rad = math.radians(pitch_deg)
    else:
        target_pitch_rad = math.radians(cur_fk["tcp_pose"]["pitch_deg"])
    target_roll_rad = math.radians(float(current_joints.get("wrist_roll", 0.0)))
    target_grip = (
        max(0.0, min(100.0, float(gripper)))
        if gripper is not None
        else float(current_joints.get("gripper", 20.0))
    )

    if _KINEMATICS_SOLVER is None:
        raise RuntimeError("SO101Kinematics solver unavailable.")

    cur_rad = {k: math.radians(float(current_joints.get(k, 0.0))) for k in JOINT_NAMES[:5]}
    if clamp_to_workspace:
        solved_rad, clamped, msg = _KINEMATICS_SOLVER.inverse_kinematics_clamped(
            target_xyz=(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0),
            current_joint_positions=cur_rad,
            desired_pitch_rad=target_pitch_rad,
            preserve_wrist_roll=target_roll_rad,
            preserve_gripper=target_grip,
            orientation_weight=0.0 if not lock_pitch else None,
        )
        return _finalize_ik_result(solved_rad, current_joints, target_grip), clamped, msg

    ok, solved_rad, err_msg = _KINEMATICS_SOLVER.inverse_kinematics(
        target_xyz=(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0),
        current_joint_positions=cur_rad,
        desired_pitch_rad=target_pitch_rad,
        preserve_wrist_roll=target_roll_rad,
        preserve_gripper=target_grip,
        orientation_weight=0.0 if not lock_pitch else None,
    )
    if not ok or solved_rad is None:
        raise ValueError(
            err_msg
            or f"目标点 ({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) mm 超出 SO-101 物理工作空间范围。"
        )
    return _finalize_ik_result(solved_rad, current_joints, target_grip), False, err_msg


def solve_ik_target(
    *,
    x_mm: float,
    y_mm: float,
    z_mm: float,
    current_joints: dict[str, float],
    pitch_deg: float | None = None,
    gripper: float | None = None,
) -> dict[str, float]:
    """Solve strict SO-101 IK using verified SO101Kinematics (mm -> degrees)."""
    solved, _, _ = solve_ik_target_with_status(
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        current_joints=current_joints,
        pitch_deg=pitch_deg,
        gripper=gripper,
        clamp_to_workspace=False,
    )
    return solved


class Arm101Simulator:
    """Thread-safe 6-DOF SO-101 dual-arm simulator, FK/IK controller, and JPEG HUD renderer."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.leader_joints: dict[str, float] = default_joints()
        self.follower_joints: dict[str, float] = default_joints()
        # Separate commanded target from measured encoder feedback to prevent PD gravity-droop ratchet
        self.follower_cmd_joints: dict[str, float] | None = None
        self.linked_arms: bool = True
        self.real_leader_active: bool = False
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

    def set_follower_joints(
        self,
        updates: dict[str, float],
        *,
        from_real_feedback: bool = False,
    ) -> None:
        with self._lock:
            if self.estop_latched and not from_real_feedback:
                return
            for name, val in updates.items():
                if name in JOINT_SPECS:
                    self.follower_joints[name] = round(float(val), 3)
            if not from_real_feedback:
                self.follower_cmd_joints = dict(self.follower_joints)
            self.step_count += 1

    def clear_follower_cmd_target(self) -> None:
        with self._lock:
            self.follower_cmd_joints = None

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
            # Start incremental step from last commanded target (not gravity-drooped present_position)
            cur = dict(self.follower_cmd_joints or self.follower_joints)
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
            elif k in ("w", "s", "q", "e", "r", "f"):
                yaw_rad = math.atan2(float(tcp["y_mm"]), float(tcp["x_mm"]))
                r_now = float(tcp["r_mm"])
                z_now = float(tcp["z_mm"])
                if k == "w":
                    r_tgt, z_tgt = r_now + step_mm, z_now
                elif k == "s":
                    r_tgt, z_tgt = max(65.0, r_now - step_mm), z_now
                elif k in ("q", "r"):
                    z_tgt = min(505.0, z_now + step_mm)
                    # Spherical workspace sliding: when stretched near outer shell (R_max ~ 400mm from shoulder (69.2, 116.6)),
                    # smoothly pull r_tgt inward along the reachable sphere so Q climbs monotonically instead of stalling.
                    sh_r, sh_z, r_safe = 69.2, 116.6, 398.0
                    dz_sh = z_tgt - sh_z
                    dr_sh = max(0.0, r_now - sh_r)
                    if dr_sh * dr_sh + dz_sh * dz_sh > r_safe * r_safe:
                        max_dr = math.sqrt(max(0.0, r_safe * r_safe - dz_sh * dz_sh))
                        r_tgt = max(65.0, sh_r + max_dr)
                    else:
                        r_tgt = r_now
                else:
                    r_tgt, z_tgt = r_now, max(20.0, z_now - step_mm)

                x_tgt = r_tgt * math.cos(yaw_rad)
                y_tgt = r_tgt * math.sin(yaw_rad)
                is_vertical = k in ("q", "e", "r", "f")
                cur, clamped, boundary_msg = solve_ik_target_with_status(
                    x_mm=x_tgt,
                    y_mm=y_tgt,
                    z_mm=z_tgt,
                    current_joints=cur,
                    pitch_deg=None if is_vertical else tcp["pitch_deg"],
                    gripper=cur["gripper"],
                    clamp_to_workspace=True,
                    lock_pitch=not is_vertical,
                )
            else:
                raise ValueError(f"Unsupported control key '{key}'.")

            self.follower_joints = cur
            self.follower_cmd_joints = dict(cur)
            if self.linked_arms and not self.real_leader_active:
                self.leader_joints = dict(cur)
            self.step_count += 1
            return {
                "action": f"key:{k}",
                "clamped": bool(locals().get("clamped", False)),
                "boundary_message": str(locals().get("boundary_msg", "")) if locals().get("clamped", False) else "",
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
        theme: str = "light",
        width: int = 640,
        height: int = 480,
    ) -> bytes:
        """Render an authentic 2D kinematic projection + HUD telemetry frame as JPEG bytes."""
        with self._lock:
            joints = dict(self.follower_joints)
            count = self._frame_counters.get(cam_id, 0) + 1
            self._frame_counters[cam_id] = count

        is_dark = theme == "dark"
        bg_col = (15, 20, 28) if is_dark else (248, 249, 250)
        grid_col = (36, 44, 58) if is_dark else (231, 233, 238)
        top_bg = (24, 30, 42) if is_dark else (241, 243, 245)
        border_col = (46, 56, 74) if is_dark else (218, 222, 228)
        text_col = (241, 245, 249) if is_dark else (39, 39, 42)
        hud_bg = (22, 27, 36) if is_dark else (255, 255, 255)
        jaw_col = (226, 232, 240) if is_dark else (24, 24, 27)

        img = Image.new("RGB", (width, height), color=bg_col)
        draw = ImageDraw.Draw(img)

        for x in range(0, width, 40):
            draw.line([(x, 44), (x, height)], fill=grid_col, width=1)
        for y in range(44, height, 40):
            draw.line([(0, y), (width, y)], fill=grid_col, width=1)

        draw.rectangle([(0, 0), (width, 44)], fill=top_bg)
        draw.line([(0, 44), (width, 44)], fill=border_col, width=1)
        if is_dark:
            badge_color = (78, 53, 12) if mode == "dry_run" else (20, 74, 42)
            badge_text_color = (253, 224, 71) if mode == "dry_run" else (134, 239, 172)
        else:
            badge_color = (251, 243, 219) if mode == "dry_run" else (237, 243, 236)
            badge_text_color = (143, 94, 0) if mode == "dry_run" else (40, 90, 44)
        draw.rectangle([(12, 10), (132, 34)], fill=badge_color, outline=border_col)
        draw.text((20, 16), f"MODE: {mode.upper()}", fill=badge_text_color)
        draw.text(
            (148, 16),
            f"ARM101 CAM:{cam_id.upper()} | WF:{active_workflow.upper()} | #{count}",
            fill=text_col,
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
            cube_col = (56, 189, 248) if is_dark else (37, 99, 235)
            draw.rectangle([(cx - 28, cy - 28), (cx + 28, cy + 28)], outline=cube_col, width=3)
            draw.text((cx - 18, cy - 6), "CUBE", fill=cube_col)
            jaw_gap = 18 + int(grip_pct * 55)
            dx, dy = math.cos(roll), math.sin(roll)
            nx, ny = -dy, dx
            for sign in (-1, 1):
                px = int(width * 0.5 + nx * jaw_gap * sign)
                py = int(height * 0.56 + ny * jaw_gap * sign)
                draw.rectangle([(px - 10, py - 24), (px + 10, py + 24)], fill=jaw_col)
        else:
            bx, by = int(width * 0.36 + math.sin(pan) * 40), int(height * 0.78)
            draw.rectangle([(bx - 42, by), (bx + 42, by + 22)], fill=(100, 116, 139))

            l1, l2, l3 = 115.0, 105.0, 60.0
            a1 = lift
            x1, y1 = bx + l1 * math.cos(a1), by + l1 * math.sin(a1)
            a2 = a1 + elbow
            x2, y2 = x1 + l2 * math.cos(a2), y1 + l2 * math.sin(a2)
            a3 = a2 + wrist
            x3, y3 = x2 + l3 * math.cos(a3), y2 + l3 * math.sin(a3)

            draw.line([(bx, by), (x1, y1)], fill=(37, 99, 235), width=10)
            draw.line([(x1, y1), (x2, y2)], fill=(2, 132, 199), width=8)
            draw.line([(x2, y2), (x3, y3)], fill=(14, 165, 233), width=6)

            for jx, jy, r in ((bx, by, 8), (x1, y1, 7), (x2, y2, 6), (x3, y3, 5)):
                draw.ellipse([(jx - r, jy - r), (jx + r, jy + r)], fill=hud_bg, outline=jaw_col)

            jaw_open = 6 + grip_pct * 22
            px_perp, py_perp = -math.sin(a3), math.cos(a3)
            for sign in (-1, 1):
                gx0 = x3 + px_perp * jaw_open * sign
                gy0 = y3 + py_perp * jaw_open * sign
                gx1 = gx0 + 20 * math.cos(a3)
                gy1 = gy0 + 20 * math.sin(a3)
                draw.line([(gx0, gy0), (gx1, gy1)], fill=jaw_col, width=4)

        draw.rectangle([(12, height - 102), (332, height - 12)], fill=hud_bg, outline=border_col)
        ts = datetime.now(timezone.utc).strftime("%H:%M:%S UTC")
        draw.text(
            (22, height - 94),
            f"pan:{joints['shoulder_pan']:6.1f}  lift:{joints['shoulder_lift']:6.1f}",
            fill=text_col,
        )
        draw.text(
            (22, height - 74),
            f"elb:{joints['elbow_flex']:6.1f}  wflx:{joints['wrist_flex']:6.1f}",
            fill=text_col,
        )
        draw.text(
            (22, height - 54),
            f"wrol:{joints['wrist_roll']:5.1f}  grip:{joints['gripper']:5.1f}%",
            fill=text_col,
        )
        draw.text((22, height - 32), f"step:{self.step_count} | {ts}", fill=(100, 116, 139))

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        return buf.getvalue()
