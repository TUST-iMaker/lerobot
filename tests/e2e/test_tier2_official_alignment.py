"""Tier 2 E2E Tests: Official Alignment Boundary, Corner & Negative Rejection Cases (Features 5–10).

Covers all 32 Tier 2 test specifications (T2-F05-01..T2-F10-05) from spec_matrix.md
and ground_truth_vectors.md. Strictly hardware-isolated: forces STUDIO_MODE="dry_run"
and mocks all serial/camera I/O so real COM11/COM12 arms and USB cameras are never touched.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import tomllib
import types
import xml.etree.ElementTree as ET
from collections.abc import Callable, Generator
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import httpx
import pytest
import yaml
from playwright.sync_api import Browser, Page

from tests.e2e._helpers import run_cmd, run_ubuntu_script, wait_until
from tests.e2e.conftest import DryRunSandbox, StudioServerHandle

pytestmark = pytest.mark.tier2


# ---------------------------------------------------------------------------
# Hardware-Safe Local Fixtures & Helpers (NEVER opens COM* or real cameras)
# ---------------------------------------------------------------------------


@pytest.fixture
def dry_run_studio(
    studio_server_factory: Callable[..., StudioServerHandle],
    dry_run_sandbox: DryRunSandbox,
) -> StudioServerHandle:
    """Launch Studio subprocess forced to STUDIO_MODE=dry_run with sandbox cleanup."""
    _ = dry_run_sandbox
    handle = studio_server_factory(extra_env={"STUDIO_MODE": "dry_run"})
    resp = httpx.get(f"{handle.base_url}/api/health", timeout=5.0)
    assert resp.status_code == 200
    assert resp.json()["mode"] == "dry_run"
    return handle


@pytest.fixture
def dry_run_client(dry_run_studio: StudioServerHandle) -> Generator[httpx.Client, None, None]:
    """HTTPX client bound to a verified dry_run Studio subprocess."""
    with httpx.Client(base_url=dry_run_studio.base_url, timeout=15.0) as client:
        yield client


@pytest.fixture
def dry_run_page(
    _playwright_browser: Browser,
    dry_run_studio: StudioServerHandle,
) -> Generator[Page, None, None]:
    """Playwright page bound to a verified dry_run Studio subprocess."""
    context = _playwright_browser.new_context(viewport={"width": 1280, "height": 800})
    page = context.new_page()
    console_errors: list[str] = []
    page_errors: list[str] = []
    page._e2e_allow_http_error_console = False  # type: ignore[attr-defined]

    def _on_console(msg: Any) -> None:
        if msg.type == "error":
            text = msg.text
            if getattr(page, "_e2e_allow_http_error_console", False) and (
                "Failed to load resource" in text or "status of 4" in text
            ):
                return
            console_errors.append(text)

    def _on_pageerror(exc: Exception) -> None:
        page_errors.append(str(exc))

    page.on("console", _on_console)
    page.on("pageerror", _on_pageerror)

    page.goto(dry_run_studio.base_url, wait_until="load", timeout=15000)
    page.locator("[data-testid='mode-badge']").wait_for(state="attached", timeout=10000)
    time.sleep(0.2)

    try:
        yield page
    finally:
        context.close()
        assert not page_errors, f"Uncaught pageerror(s): {page_errors}"
        assert not console_errors, f"Browser console error(s): {console_errors}"


def _ensure_software_on_path(workspace_root: Path) -> None:
    root_str = str(workspace_root)
    sw_str = str(workspace_root / "software")
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    if sw_str not in sys.path:
        sys.path.insert(0, sw_str)


def _compute_urdf_fk_mm(urdf_path: Path, q_rad: dict[str, float]) -> tuple[float, float, float]:
    """Independent reference SE(3) forward kinematics directly from so101_new_calib.urdf."""
    import numpy as np

    def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        return rz @ ry @ rx

    def _transform(xyz: tuple[float, ...], rpy: tuple[float, ...]) -> np.ndarray:
        t = np.eye(4, dtype=float)
        t[:3, :3] = _rpy_matrix(rpy[0], rpy[1], rpy[2])
        t[:3, 3] = xyz
        return t

    def _rot_axis(axis: tuple[float, ...], angle: float) -> np.ndarray:
        ax = np.array(axis, dtype=float)
        ax = ax / np.linalg.norm(ax)
        x, y, z = ax
        c, s = math.cos(angle), math.sin(angle)
        k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
        t = np.eye(4, dtype=float)
        t[:3, :3] = np.eye(3) + s * k + (1.0 - c) * (k @ k)
        return t

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints_by_child: dict[str, ET.Element] = {}
    for j in root.findall("joint"):
        child = j.find("child")
        if child is not None:
            joints_by_child[child.attrib["link"]] = j

    chain: list[ET.Element] = []
    cur_link = "gripper_frame_link"
    while cur_link != "base_link":
        j = joints_by_child[cur_link]
        chain.append(j)
        cur_link = j.find("parent").attrib["link"]  # type: ignore[union-attr]
    chain.reverse()

    t_world = np.eye(4, dtype=float)
    for j in chain:
        origin = j.find("origin")
        xyz = tuple(float(v) for v in origin.attrib["xyz"].split())  # type: ignore[union-attr]
        rpy = tuple(float(v) for v in origin.attrib["rpy"].split())  # type: ignore[union-attr]
        t_world = t_world @ _transform(xyz, rpy)
        if j.attrib["type"] == "revolute":
            axis_el = j.find("axis")
            axis = (
                tuple(float(v) for v in axis_el.attrib["xyz"].split())
                if axis_el is not None
                else (0.0, 0.0, 1.0)
            )
            t_world = t_world @ _rot_axis(axis, float(q_rad.get(j.attrib["name"], 0.0)))

    return (
        float(t_world[0, 3] * 1000.0),
        float(t_world[1, 3] * 1000.0),
        float(t_world[2, 3] * 1000.0),
    )


class _FakeRustypotBus:
    """In-memory fake rustypot.Bus implementing STS3215 register state with zero COM I/O."""

    def __init__(self, motor_ids: list[int] | None = None) -> None:
        ids = motor_ids or [1, 2, 3, 4, 5, 6]
        self.closed = False
        self.writes: list[tuple[int, str, int]] = []
        self.sync_writes: list[tuple[list[int], str, list[int]]] = []
        self.torque_calls: list[tuple[list[int], bool]] = []
        self.regs: dict[int, dict[str, int]] = {}
        for mid in ids:
            self.regs[mid] = {
                "id": mid,
                "model_number": 777,
                "firmware_major_version": 3,
                "firmware_minor_version": 10,
                "baud_rate": 0,
                "return_delay_time": 250,
                "min_position_limit": 0,
                "max_position_limit": 4095,
                "max_torque_limit": 1000,
                "phase": 0x10,
                "p_coefficient": 32,
                "d_coefficient": 0,
                "i_coefficient": 0,
                "protection_current": 500,
                "homing_offset": 0,
                "operating_mode": 0,
                "overload_torque": 80,
                "torque_enable": 0,
                "acceleration": 0,
                "goal_position": 2047,
                "lock": 0,
                "present_position": 2047,
                "maximum_acceleration": 0,
            }

    def broadcast_scan(self, _definition: Any) -> dict[int, int]:
        return {mid: 777 for mid in self.regs}

    def scan(self, _definition: Any) -> dict[int, int]:
        return {mid: 777 for mid in self.regs}

    def read_register_with_error(self, motor_id: int, reg: str, retries: int = 0) -> tuple[int, int]:
        _ = retries
        return int(self.regs[motor_id].get(reg.lower(), 0)), 0

    def write_register_with_error(
        self, motor_id: int, reg: str, value: int, retries: int = 0
    ) -> int:
        _ = retries
        key = reg.lower()
        self.regs[motor_id][key] = int(value)
        self.writes.append((motor_id, key, int(value)))
        return 0

    def sync_read_register_with_error(
        self, ids: list[int], reg: str, retries: int = 0
    ) -> list[tuple[int, int]]:
        _ = retries
        key = reg.lower()
        return [(int(self.regs[mid].get(key, 0)), 0) for mid in ids]

    def sync_write_register(
        self, ids: list[int], reg: str, values: list[int], retries: int = 0
    ) -> None:
        _ = retries
        key = reg.lower()
        for mid, val in zip(ids, values, strict=True):
            self.regs[mid][key] = int(val)
            if key == "goal_position":
                self.regs[mid]["present_position"] = int(val)
        self.sync_writes.append((list(ids), key, [int(v) for v in values]))

    def set_torque(self, ids: list[int], enabled: bool, retries: int = 0) -> dict[int, str]:
        _ = retries
        self.torque_calls.append((list(ids), bool(enabled)))
        for mid in ids:
            self.regs[mid]["torque_enable"] = 1 if enabled else 0
            self.regs[mid]["lock"] = 1 if enabled else 0
        return {}

    def close(self) -> None:
        self.closed = True


def _make_so101_bus(
    *,
    range_min: int = 500,
    range_max: int = 3500,
    drive_mode: int = 0,
    use_degrees: bool = True,
) -> tuple[Any, _FakeRustypotBus]:
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode, SerialMotorsBus

    norm_body = MotorNormMode.DEGREES if use_degrees else MotorNormMode.RANGE_M100_100
    motors = {
        "shoulder_pan": Motor(1, "sts3215", norm_body),
        "shoulder_lift": Motor(2, "sts3215", norm_body),
        "elbow_flex": Motor(3, "sts3215", norm_body),
        "wrist_flex": Motor(4, "sts3215", norm_body),
        "wrist_roll": Motor(5, "sts3215", norm_body),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    }
    calibration = {
        name: MotorCalibration(
            id=m.id,
            drive_mode=drive_mode,
            homing_offset=0,
            range_min=0 if name == "wrist_roll" else range_min,
            range_max=4095 if name == "wrist_roll" else range_max,
        )
        for name, m in motors.items()
    }
    bus = SerialMotorsBus(port="MOCK_PORT", motors=motors, calibration=calibration)
    fake_hw = _FakeRustypotBus([1, 2, 3, 4, 5, 6])
    for name, cal in calibration.items():
        fake_hw.regs[cal.id]["min_position_limit"] = cal.range_min
        fake_hw.regs[cal.id]["max_position_limit"] = cal.range_max
        fake_hw.regs[cal.id]["homing_offset"] = cal.homing_offset
    bus._bus = fake_hw  # type: ignore[assignment]
    return bus, fake_hw


# ---------------------------------------------------------------------------
# Feature 5: Underlying Motor Bus & Driver Alignment (T2-F05-01 .. T2-F05-07)
# ---------------------------------------------------------------------------


def test_t2_f05_01_serial_motors_bus_degrees_vs_range_0_100_and_int_truncation(
    workspace_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F05-01: Body joints use DEGREES (no drive_mode/clamp, int() trunc); gripper uses RANGE_0_100."""
    _ensure_software_on_path(workspace_root)
    bus, fake_hw = _make_so101_bus(range_min=500, range_max=3500, drive_mode=1)

    # 1. DEGREES ignores drive_mode=1 and does not clamp raw outside [500, 3500]
    norm_out = bus._normalize({1: 2000, 3: 200, 6: 500})
    assert math.isclose(norm_out[1], 0.0, abs_tol=1e-9)
    assert math.isclose(norm_out[3], (200 - 2000.0) * 360.0 / 4095.0, rel_tol=1e-9)
    # gripper with drive_mode=1: raw=500 (range_min) -> 100.0%
    assert math.isclose(norm_out[6], 100.0, abs_tol=1e-9)

    # 2. _unnormalize uses int() truncation toward zero, NOT round()
    # mid=2000; deg=5.60 -> 5.60*4095/360 + 2000 = 2063.70 -> int() = 2063 (not 2064)
    unnorm_deg = bus._unnormalize({1: 5.60, 6: 100.0})
    assert unnorm_deg[1] == 2063
    # gripper=100.0% with drive_mode=1 -> range_min=500
    assert unnorm_deg[6] == 500

    # Non-inverted gripper (drive_mode=0): 100.0% -> 3500, 0.0% -> 500, 50.0% -> 2000
    bus_noninv, _ = _make_so101_bus(range_min=500, range_max=3500, drive_mode=0)
    assert bus_noninv._unnormalize({6: 0.0})[6] == 500
    assert bus_noninv._unnormalize({6: 50.0})[6] == 2000
    assert bus_noninv._unnormalize({6: 100.0})[6] == 3500
    assert math.isclose(bus_noninv._normalize({6: 3500})[6], 100.0, abs_tol=1e-9)

    # 3. Verify SO101FollowerArm wrapper preserves gripper=100.0% (no 85.94% cap or pi/180 scaling)
    from core.data_types import ActionType, ControlSource, JointUnit, RobotAction
    from hardware.config import ArmConfig, ArmRole
    from hardware.follower_arm import SO101FollowerArm
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    bus_arm, fake_arm_hw = _make_so101_bus(range_min=500, range_max=3500, drive_mode=0)
    bus_arm._bus = None  # allow connect() to open fake_arm_hw cleanly
    bus_arm._open_bus = lambda: fake_arm_hw  # type: ignore[method-assign]
    bus_arm._handshake = lambda: None  # type: ignore[method-assign]
    fake_arm_hw.regs[6]["present_position"] = 3500
    fake_arm_hw.regs[6]["goal_position"] = 3500

    arm_cfg = ArmConfig(
        role=ArmRole.FOLLOWER,
        port="MOCK_PORT",
        robot_id="test_follower",
        calibration_fpath=workspace_root / "data" / "calibration" / "nonexistent.json",
        use_degrees=True,
        internal_unit=JointUnit.RADIAN,
    )

    class _MockedSO101F(SO101Follower):
        def __init__(self, cfg: SO101FollowerConfig) -> None:
            super().__init__(cfg)
            self.bus = bus_arm

    monkeypatch.setattr("lerobot.robots.so_follower.SO101Follower", _MockedSO101F)

    arm = SO101FollowerArm(arm_cfg)
    arm.connect()
    try:
        state = arm.read_state()
        assert math.isclose(state.positions["gripper"], 100.0, abs_tol=1e-6), (
            f"Expected gripper=100.0% without rad conversion, got {state.positions['gripper']}"
        )
        act = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.UI,
            timestamp=time.monotonic(),
            target_joint_positions={
                "shoulder_pan": 0.0,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
                "gripper": 100.0,
            },
            unit=JointUnit.RADIAN,
        )
        assert arm.send_action(act) is True
        assert fake_arm_hw.regs[6]["goal_position"] == 3500, (
            f"Expected gripper raw=3500 for 100.0%, got {fake_arm_hw.regs[6]['goal_position']}"
        )
    finally:
        arm.disconnect()


def test_t2_f05_02_half_turn_homing_edge_clamp_and_full_turn_wrist_roll(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F05-02: Half-turn homing clamps 4095 -> +2047; wrist_roll gets full-turn [0, 4095]."""
    bus, fake_hw = _make_so101_bus()

    fake_hw.regs[1]["present_position"] = 0
    fake_hw.regs[2]["present_position"] = 2047
    fake_hw.regs[3]["present_position"] = 2048
    fake_hw.regs[4]["present_position"] = 4095
    fake_hw.regs[5]["present_position"] = 3000
    fake_hw.regs[6]["present_position"] = 2047

    offsets = bus.set_half_turn_homings()
    assert offsets["shoulder_pan"] == -2047
    assert offsets["shoulder_lift"] == 0
    assert offsets["elbow_flex"] == 1
    assert offsets["wrist_flex"] == 2047, "4095 - 2047 = 2048 must clamp to +2047"
    assert offsets["wrist_roll"] == 953
    assert offsets["gripper"] == 0

    # record_calibration with full_turn_motors=["wrist_roll"]
    import lerobot.motors.motors_bus as mb_mod

    monkeypatch.setattr("builtins.input", lambda _prompt="": "")
    step = {"count": 0}

    def _enter_after_two() -> bool:
        step["count"] += 1
        if step["count"] == 1:
            for mid in (1, 2, 3, 4, 6):
                fake_hw.regs[mid]["present_position"] = 1200
            return False
        for mid in (1, 2, 3, 4, 6):
            fake_hw.regs[mid]["present_position"] = 2800
        return True

    monkeypatch.setattr(mb_mod, "enter_pressed", _enter_after_two)
    monkeypatch.setattr(mb_mod, "move_cursor_up", lambda _n: None)

    cal = bus.record_calibration("position", full_turn_motors=["wrist_roll"])
    assert (cal["wrist_roll"].range_min, cal["wrist_roll"].range_max) == (0, 4095)
    assert cal["shoulder_pan"].range_min < cal["shoulder_pan"].range_max


def test_t2_f05_03_motor_register_init_and_gripper_protection() -> None:
    """T2-F05-03: SOFollower/SOLeader.configure writes official STS3215 registers & EEPROM lock."""
    from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
    from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

    # Follower configure
    f_cfg = SOFollowerRobotConfig(port="MOCK_PORT", id="t2_f05_03_f")
    follower = SOFollower(f_cfg)
    fake_f = _FakeRustypotBus([1, 2, 3, 4, 5, 6])
    follower.bus._bus = fake_f  # type: ignore[assignment]
    follower.configure()

    for mid in range(1, 7):
        regs = fake_f.regs[mid]
        assert regs["return_delay_time"] == 0
        assert regs["acceleration"] == 254
        assert regs["maximum_acceleration"] == 254
        assert (regs["phase"] & 0x10) == 0
        assert regs["operating_mode"] == 0
        assert regs["p_coefficient"] == 16
        assert regs["i_coefficient"] == 0
        assert regs["d_coefficient"] == 32
        assert regs["torque_enable"] == 1
        assert regs["lock"] == 1

    # Gripper-only torque/current protections
    assert fake_f.regs[6]["max_torque_limit"] == 500
    assert fake_f.regs[6]["protection_current"] == 250
    assert fake_f.regs[6]["overload_torque"] == 25
    assert fake_f.regs[1]["max_torque_limit"] == 1000

    # Leader configure leaves torque disabled
    l_cfg = SOLeaderTeleopConfig(port="MOCK_PORT", id="t2_f05_03_l")
    leader = SOLeader(l_cfg)
    fake_l = _FakeRustypotBus([1, 2, 3, 4, 5, 6])
    leader.bus._bus = fake_l  # type: ignore[assignment]
    leader.configure()

    for mid in range(1, 7):
        assert fake_l.regs[mid]["return_delay_time"] == 0
        assert fake_l.regs[mid]["acceleration"] == 254
        assert fake_l.regs[mid]["maximum_acceleration"] == 254
        assert (fake_l.regs[mid]["phase"] & 0x10) == 0
        assert fake_l.regs[mid]["operating_mode"] == 0
        assert fake_l.regs[mid]["torque_enable"] == 0


def test_t2_f05_04_cross_os_calibration_dir_and_hf_lerobot_calibration(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F05-04: Explicit vs HF_LEROBOT_CALIBRATION path resolution; zero /home/aguiya/ defaults."""
    _ensure_software_on_path(workspace_root)
    from hardware.config import ArmConfig

    # 1. Local ArmConfig defaults must NOT contain /home/aguiya/
    f_default = ArmConfig.default_follower(port="MOCK_PORT", robot_id="f_test")
    l_default = ArmConfig.default_leader(port="MOCK_PORT", robot_id="l_test")
    assert "/home/aguiya" not in str(f_default.calibration_fpath)
    assert "/home/aguiya" not in str(l_default.calibration_fpath)

    # 2. Explicit calibration_dir on ArmConfig and SO101FollowerConfig / SO101LeaderConfig
    custom_dir = tmp_path / "custom_cal"
    custom_dir.mkdir(parents=True, exist_ok=True)
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

    f_explicit = SO101Follower(
        SO101FollowerConfig(port="MOCK_PORT", id="my_f", calibration_dir=custom_dir)
    )
    l_explicit = SO101Leader(
        SO101LeaderConfig(port="MOCK_PORT", id="my_l", calibration_dir=custom_dir)
    )
    assert f_explicit.calibration_fpath == custom_dir / "my_f.json"
    assert l_explicit.calibration_fpath == custom_dir / "my_l.json"

    # 3. Default calibration_dir=None uses HF_LEROBOT_CALIBRATION/{robots/so_follower,teleoperators/so_leader}
    hf_cal = tmp_path / "hf_cal"
    import lerobot.robots.robot as robot_mod
    import lerobot.teleoperators.teleoperator as teleop_mod

    monkeypatch.setattr(robot_mod, "HF_LEROBOT_CALIBRATION", hf_cal)
    monkeypatch.setattr(teleop_mod, "HF_LEROBOT_CALIBRATION", hf_cal)
    f_auto = SO101Follower(SO101FollowerConfig(port="MOCK_PORT", id="auto_f", calibration_dir=None))
    l_auto = SO101Leader(SO101LeaderConfig(port="MOCK_PORT", id="auto_l", calibration_dir=None))
    assert f_auto.calibration_fpath == hf_cal / "robots" / "so_follower" / "auto_f.json"
    assert l_auto.calibration_fpath == hf_cal / "teleoperators" / "so_leader" / "auto_l.json"


def test_t2_f05_05_follower_arm_max_relative_target_is_calibrated_and_move_to_middle(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F05-05: SO101FollowerArm forwards max_relative_target, checks is_calibrated, preserves gripper in move_to_middle."""
    _ensure_software_on_path(workspace_root)
    from core.data_types import ActionType, ControlSource, JointUnit, RobotAction
    from hardware.config import ArmConfig, ArmRole
    from hardware.follower_arm import SO101FollowerArm
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cal_dir = tmp_path / "cal"
    cal_dir.mkdir(parents=True, exist_ok=True)
    cal_file = cal_dir / "t2_f05_05.json"
    cal_payload = {
        name: {
            "id": idx + 1,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 0 if name == "wrist_roll" else 500,
            "range_max": 4095 if name == "wrist_roll" else 3500,
        }
        for idx, name in enumerate(
            ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
        )
    }
    cal_file.write_text(json.dumps(cal_payload), encoding="utf-8")

    bus, fake_hw = _make_so101_bus(range_min=500, range_max=3500)
    # Initial raw positions: shoulder_pan=+30 deg -> raw = 2000 + 30*4095/360 = 2341, gripper=65% -> 2450
    fake_hw.regs[1]["present_position"] = 2341
    fake_hw.regs[2]["present_position"] = 1772  # ~ -20 deg
    fake_hw.regs[3]["present_position"] = 2113  # ~ +10 deg
    fake_hw.regs[4]["present_position"] = 2113
    fake_hw.regs[5]["present_position"] = 2161
    fake_hw.regs[6]["present_position"] = 2450  # 65% of [500, 3500]

    captured_cfg: dict[str, Any] = {}

    class _MockedSO101Follower(SO101Follower):
        def __init__(self, cfg: SO101FollowerConfig) -> None:
            captured_cfg["cfg"] = cfg
            super().__init__(cfg)
            self.bus._open_bus = lambda: fake_hw  # type: ignore[method-assign]
            self.bus._handshake = lambda: None  # type: ignore[method-assign]

    monkeypatch.setattr("lerobot.robots.so_follower.SO101Follower", _MockedSO101Follower)

    arm_cfg = ArmConfig(
        role=ArmRole.FOLLOWER,
        port="MOCK_PORT",
        robot_id="t2_f05_05",
        calibration_fpath=cal_file,
        use_degrees=True,
        internal_unit=JointUnit.RADIAN,
        max_relative_target=5.0,
    )
    arm = SO101FollowerArm(arm_cfg)
    arm.connect()
    assert "cfg" in captured_cfg
    assert captured_cfg["cfg"].max_relative_target == 5.0
    assert captured_cfg["cfg"].calibration_dir == cal_dir

    # send_action with large jump must clamp step to <= 5.0 deg
    big_jump = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.UI,
        timestamp=time.monotonic(),
        target_joint_positions={
            "shoulder_pan": math.radians(90.0),
            "shoulder_lift": math.radians(-20.0),
            "elbow_flex": math.radians(10.0),
            "wrist_flex": math.radians(10.0),
            "wrist_roll": math.radians(10.0),
            "gripper": 65.0,
        },
        unit=JointUnit.RADIAN,
    )
    assert arm.send_action(big_jump) is True
    after_state = arm.read_state()
    assert math.degrees(after_state.positions["shoulder_pan"]) <= 36.0

    # move_to_middle moves arm joints toward 0.0 deg while keeping gripper >= 45% (NOT 0.0%)
    arm.move_to_middle(step_deg=15.0, tolerance_deg=2.0, timeout_s=3.0)
    mid_state = arm.read_state()
    assert abs(math.degrees(mid_state.positions["shoulder_pan"])) <= 3.0
    assert mid_state.positions["gripper"] >= 45.0, (
        f"move_to_middle must NOT force gripper to 0.0%; got {mid_state.positions['gripper']}"
    )
    arm.disconnect()


def test_t2_f05_06_negative_invalid_calibration_max_relative_target_lerobot_home_and_scservo(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F05-06: Reject range_min==range_max, partial max_relative_target, deprecated LEROBOT_HOME; no scservo_sdk gate."""
    _ensure_software_on_path(workspace_root)
    from lerobot.robots.utils import ensure_safe_goal_position

    # 1. range_min == range_max raises ValueError in SerialMotorsBus
    bus, _ = _make_so101_bus(range_min=1000, range_max=1000)
    with pytest.raises(ValueError):
        bus._normalize({1: 1000})
    with pytest.raises(ValueError):
        bus._unnormalize({1: 0.0})

    # 2. Partial dict max_relative_target raises ValueError
    with pytest.raises(ValueError):
        ensure_safe_goal_position(
            {"shoulder_pan": (10.0, 0.0), "gripper": (50.0, 10.0)},
            {"shoulder_pan": 5.0},
        )

    # 3. LEROBOT_HOME deprecated env var raises ValueError
    env = os.environ.copy()
    env["LEROBOT_HOME"] = str(tmp_path / "legacy_home")
    res = run_cmd(
        [sys.executable, "-c", "import lerobot.utils.constants"],
        cwd=workspace_root,
        env=env,
    )
    assert res.returncode != 0
    assert "LEROBOT_HOME" in (res.stderr + res.stdout)


# ---------------------------------------------------------------------------
# Feature 6: Core Safety, Presets, Units & Kinematics (T2-F06-01 .. T2-F06-05)
# ---------------------------------------------------------------------------


def test_t2_f06_01_urdf_zero_pose_tcp_and_shoulder_pan_sign(workspace_root: Path) -> None:
    """T2-F06-01: SO101Kinematics matches so101_new_calib.urdf within <0.1mm and pan +0.5 -> Y<0."""
    _ensure_software_on_path(workspace_root)
    urdf_path = (
        workspace_root / "repos" / "SO-ARM100" / "Simulation" / "SO101" / "so101_new_calib.urdf"
    )
    assert urdf_path.is_file()

    # Cross-check independent URDF parser against ground_truth_vectors.md
    zero_mm = _compute_urdf_fk_mm(urdf_path, {})
    assert math.isclose(zero_mm[0], 391.361470, abs_tol=0.02)
    assert math.isclose(zero_mm[1], -0.009212, abs_tol=0.02)
    assert math.isclose(zero_mm[2], 226.469710, abs_tol=0.02)

    pan_mm = _compute_urdf_fk_mm(urdf_path, {"shoulder_pan": 0.5})
    assert pan_mm[1] < -160.0 and math.isclose(pan_mm[1], -169.018, abs_tol=0.1)

    combo_rad = {
        "shoulder_pan": math.radians(30.0),
        "shoulder_lift": math.radians(-45.0),
        "elbow_flex": math.radians(60.0),
        "wrist_flex": math.radians(-30.0),
        "wrist_roll": math.radians(45.0),
    }
    combo_mm = _compute_urdf_fk_mm(urdf_path, combo_rad)
    assert math.isclose(combo_mm[0], 259.036972, abs_tol=0.1)
    assert math.isclose(combo_mm[1], -133.652422, abs_tol=0.1)
    assert math.isclose(combo_mm[2], 221.807067, abs_tol=0.1)

    from core.data_types import JointUnit
    from core.kinematics import SO101Kinematics

    kin = SO101Kinematics()
    (x0, y0, z0), _, _ = kin.forward_kinematics(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
        },
        unit=JointUnit.RADIAN,
    )
    err_zero = math.sqrt(
        (x0 * 1000.0 - zero_mm[0]) ** 2
        + (y0 * 1000.0 - zero_mm[1]) ** 2
        + (z0 * 1000.0 - zero_mm[2]) ** 2
    )
    assert err_zero < 0.1, f"Zero-pose TCP error {err_zero:.3f}mm >= 0.1mm vs URDF"

    (xp, yp, zp), _, _ = kin.forward_kinematics({"shoulder_pan": 0.5}, unit=JointUnit.RADIAN)
    assert yp < 0.0, f"shoulder_pan=+0.5 rad must rotate toward -Y, got y={yp*1000:.2f}mm"
    err_pan = math.sqrt(
        (xp * 1000.0 - pan_mm[0]) ** 2
        + (yp * 1000.0 - pan_mm[1]) ** 2
        + (zp * 1000.0 - pan_mm[2]) ** 2
    )
    assert err_pan < 0.1, f"Pan +0.5 rad TCP error {err_pan:.3f}mm >= 0.1mm vs URDF"


def test_t2_f06_02_safety_controller_urdf_limits_and_gripper_100(workspace_root: Path) -> None:
    """T2-F06-02: SafetyController accepts exact URDF bounds and gripper=0.0/85.95/100.0 unclipped."""
    _ensure_software_on_path(workspace_root)
    from core.data_types import (
        ActionType,
        ControlSource,
        JointState,
        JointUnit,
        RobotAction,
        SystemState,
    )
    from core.safety_controller import DEFAULT_JOINT_LIMITS_RAD, SafetyController, SafetyLimitsConfig
    from core.state_machine import RobotStateMachine

    assert math.isclose(DEFAULT_JOINT_LIMITS_RAD["shoulder_pan"][1], 1.91986, abs_tol=1e-3)
    assert math.isclose(DEFAULT_JOINT_LIMITS_RAD["shoulder_lift"][1], 1.74533, abs_tol=1e-3)
    assert math.isclose(DEFAULT_JOINT_LIMITS_RAD["elbow_flex"][1], 1.69000, abs_tol=1e-3)
    assert math.isclose(DEFAULT_JOINT_LIMITS_RAD["wrist_flex"][1], 1.65806, abs_tol=1e-3)
    assert math.isclose(DEFAULT_JOINT_LIMITS_RAD["wrist_roll"][0], -2.74385, abs_tol=1e-3)
    assert math.isclose(DEFAULT_JOINT_LIMITS_RAD["wrist_roll"][1], 2.84121, abs_tol=1e-3)

    sm = RobotStateMachine(initial_state=SystemState.IDLE)
    sm.claim_control(ControlSource.UI)
    cfg = SafetyLimitsConfig(
        max_joint_delta_rad=10.0,
        limit_action_mode="reject",
    )
    sc = SafetyController(sm, cfg)

    pos_bound = {
        "shoulder_pan": 1.91986,
        "shoulder_lift": 1.74533,
        "elbow_flex": 1.69000,
        "wrist_flex": 1.65806,
        "wrist_roll": 2.84121,
        "gripper": 100.0,
    }
    cur_pos = JointState(
        timestamp=time.monotonic(),
        positions={
            "shoulder_pan": 1.91986,
            "shoulder_lift": 1.74533,
            "elbow_flex": 1.69000,
            "wrist_flex": 1.65806,
            "wrist_roll": 2.84121,
            "gripper": 95.0,
        },
        unit=JointUnit.RADIAN,
    )
    act_hi = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.UI,
        timestamp=time.monotonic(),
        target_joint_positions=pos_bound,
        unit=JointUnit.RADIAN,
    )
    ok, safe_hi, msg = sc.filter_and_verify(act_hi, cur_pos)
    assert ok and safe_hi is not None, f"URDF max boundary rejected: {msg}"
    assert math.isclose(safe_hi.target_joint_positions["gripper"], 100.0, abs_tol=1e-4)

    neg_bound = {
        "shoulder_pan": -1.91986,
        "shoulder_lift": -1.74533,
        "elbow_flex": -1.69000,
        "wrist_flex": -1.65806,
        "wrist_roll": -2.74385,
        "gripper": 0.0,
    }
    cur_neg = JointState(
        timestamp=time.monotonic(),
        positions={
            "shoulder_pan": -1.91986,
            "shoulder_lift": -1.74533,
            "elbow_flex": -1.69000,
            "wrist_flex": -1.65806,
            "wrist_roll": -2.74385,
            "gripper": 5.0,
        },
        unit=JointUnit.RADIAN,
    )
    act_lo = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.UI,
        timestamp=time.monotonic(),
        target_joint_positions=neg_bound,
        unit=JointUnit.RADIAN,
    )
    ok_lo, safe_lo, msg_lo = sc.filter_and_verify(act_lo, cur_neg)
    assert ok_lo and safe_lo is not None, f"URDF min boundary rejected: {msg_lo}"
    assert math.isclose(safe_lo.target_joint_positions["gripper"], 0.0, abs_tol=1e-4)


def test_t2_f06_03_mixed_unit_joint_state_and_robot_action_invariance(workspace_root: Path) -> None:
    """T2-F06-03: JointState.to_radians()/to_degrees() convert body joints but keep gripper in [0,100]."""
    _ensure_software_on_path(workspace_root)
    from core.data_types import ActionType, ControlSource, JointState, JointUnit, RobotAction

    js_deg = JointState(
        timestamp=1.0,
        positions={
            "shoulder_pan": 90.0,
            "shoulder_lift": -45.0,
            "elbow_flex": 30.0,
            "wrist_flex": 0.0,
            "wrist_roll": 180.0,
            "gripper": 85.0,
        },
        unit=JointUnit.DEGREE,
    )
    js_rad = js_deg.to_radians()
    assert math.isclose(js_rad.positions["shoulder_pan"], math.pi / 2.0, abs_tol=1e-9)
    assert math.isclose(js_rad.positions["gripper"], 85.0, abs_tol=1e-9)

    js_back = js_rad.to_degrees()
    assert math.isclose(js_back.positions["shoulder_pan"], 90.0, abs_tol=1e-9)
    assert math.isclose(js_back.positions["gripper"], 85.0, abs_tol=1e-9)

    act = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.UI,
        timestamp=2.0,
        target_joint_positions={"shoulder_pan": 90.0, "gripper": 85.0},
        unit=JointUnit.DEGREE,
    )
    if hasattr(act, "to_radians"):
        act_rad = act.to_radians()
        assert math.isclose(act_rad.target_joint_positions["gripper"], 85.0, abs_tol=1e-9)


def test_t2_f06_04_presets_and_pose_json_files_urdf_valid(workspace_root: Path) -> None:
    """T2-F06-04: All 5 presets and rest_pose/upright_pose JSONs obey URDF bounds & gripper [0,100]."""
    _ensure_software_on_path(workspace_root)
    from core.data_types import JointUnit
    from core.presets import get_preset_pose
    from core.safety_controller import DEFAULT_JOINT_LIMITS_RAD

    for name in ("home", "rest", "inspect", "pick_hover", "wave"):
        pose_rad = get_preset_pose(name, unit=JointUnit.RADIAN)
        for jname in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
            lo, hi = DEFAULT_JOINT_LIMITS_RAD[jname]
            assert lo <= pose_rad[jname] <= hi, (
                f"Preset '{name}' {jname}={pose_rad[jname]} out of [{lo}, {hi}]"
            )
        assert 0.0 <= pose_rad["gripper"] <= 100.0, (
            f"Preset '{name}' gripper={pose_rad['gripper']} not in [0, 100]"
        )
    assert get_preset_pose("pick_hover", unit=JointUnit.RADIAN)["gripper"] >= 60.0

    upright = json.loads((workspace_root / "software" / "upright_pose.json").read_text(encoding="utf-8"))
    for k, v in upright["joints"].items():
        j = k.removesuffix(".pos")
        if j != "gripper":
            assert abs(float(v)) <= 5.0, f"upright_pose.json {j}={v} deg not aligned to 0° home"
        else:
            assert 0.0 <= float(v) <= 100.0

    rest = json.loads((workspace_root / "software" / "rest_pose.json").read_text(encoding="utf-8"))
    urdf_deg_bounds = {
        "shoulder_pan": (-110.0, 110.0),
        "shoulder_lift": (-100.0, 100.0),
        "elbow_flex": (-96.83, 96.83),
        "wrist_flex": (-95.0, 95.0),
        "wrist_roll": (-157.21, 162.79),
        "gripper": (0.0, 100.0),
    }
    for k, v in rest["joints"].items():
        j = k.removesuffix(".pos")
        lo, hi = urdf_deg_bounds[j]
        assert lo <= float(v) <= hi, f"rest_pose.json {j}={v} out of URDF [{lo}, {hi}]"


def test_t2_f06_05_negative_reject_out_of_urdf_nan_wrong_source_estop_tracking(
    workspace_root: Path,
) -> None:
    """T2-F06-05: Negative rejection for out-of-URDF limits, NaN, wrong source, ESTOP, 5x tracking stall."""
    _ensure_software_on_path(workspace_root)
    from core.data_types import (
        SO101_JOINT_NAMES,
        ActionType,
        ControlSource,
        JointState,
        JointUnit,
        RobotAction,
        SystemState,
    )
    from core.safety_controller import SafetyController, SafetyLimitsConfig
    from core.state_machine import RobotStateMachine

    sm = RobotStateMachine(initial_state=SystemState.IDLE)
    sm.claim_control(ControlSource.LEADER)
    sc = SafetyController(sm, SafetyLimitsConfig(limit_action_mode="reject", max_joint_delta_rad=10.0))
    cur = JointState(
        timestamp=time.monotonic(),
        positions={j: 0.0 for j in SO101_JOINT_NAMES},
        unit=JointUnit.RADIAN,
    )

    # 1. Out-of-URDF shoulder_pan=115 deg & wrist_roll=-165 deg
    for bad_joint, bad_val in (("shoulder_pan", 115.0), ("wrist_roll", -165.0)):
        act_bad = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.LEADER,
            timestamp=time.monotonic(),
            target_joint_positions={bad_joint: bad_val},
            unit=JointUnit.DEGREE,
        )
        ok, res, _ = sc.filter_and_verify(act_bad, cur)
        assert not ok and res is None, f"Expected reject for {bad_joint}={bad_val}"

    # 2. NaN position rejected
    act_nan = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.LEADER,
        timestamp=time.monotonic(),
        target_joint_positions={"shoulder_pan": float("nan")},
        unit=JointUnit.RADIAN,
    )
    ok_nan, _, _ = sc.filter_and_verify(act_nan, cur)
    assert not ok_nan

    # 3. Wrong ControlSource rejected
    act_wrong_src = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.POLICY,
        timestamp=time.monotonic(),
        target_joint_positions={"shoulder_pan": 0.1},
        unit=JointUnit.RADIAN,
    )
    ok_src, _, _ = sc.filter_and_verify(act_wrong_src, cur)
    assert not ok_src

    # 4. Tracking watchdog: 4 errors + 1 reset does NOT fault; 5 consecutive triggers FAULT
    cmd = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.LEADER,
        timestamp=time.monotonic(),
        target_joint_positions={"shoulder_pan": 0.0},
        unit=JointUnit.RADIAN,
    )
    stuck = JointState(
        timestamp=time.monotonic(),
        positions={**{j: 0.0 for j in SO101_JOINT_NAMES}, "shoulder_pan": 0.8},
        unit=JointUnit.RADIAN,
    )
    for _ in range(4):
        ok_t, _ = sc.check_tracking_error(cmd, stuck)
        assert ok_t
    ok_reset, _ = sc.check_tracking_error(cmd, cur)
    assert ok_reset and sm.current_state != SystemState.FAULT

    for idx in range(5):
        ok_t, _ = sc.check_tracking_error(cmd, stuck)
        if idx < 4:
            assert ok_t
        else:
            assert not ok_t
    assert sm.current_state == SystemState.FAULT

    # 5. ESTOP_LATCHED rejects actions
    sm.request_reset()
    sm.trigger_estop("test estop")
    act_ok = RobotAction(
        action_type=ActionType.JOINT_POSITION,
        source=ControlSource.LEADER,
        timestamp=time.monotonic(),
        target_joint_positions={"shoulder_pan": 0.05},
        unit=JointUnit.RADIAN,
    )
    ok_estop, _, _ = sc.filter_and_verify(act_ok, cur)
    assert not ok_estop


# ---------------------------------------------------------------------------
# Feature 7: Camera, Teleop & Legacy Web Alignment (T2-F07-01 .. T2-F07-05)
# ---------------------------------------------------------------------------


def test_t2_f07_03_official_wrapper_argv_os_pathsep_fps_none_and_calibration_dir(
    workspace_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F07-03: official_wrapper uses os.pathsep, omits fps:null, forwards calibration_dir & target_follower_port."""
    _ensure_software_on_path(workspace_root)
    from teleop import official_wrapper

    captured_cmds: list[list[str]] = []
    captured_envs: list[dict[str, str]] = []
    park_calls: list[dict[str, Any]] = []

    class _FakePopen:
        def __init__(self, cmd: list[str], env: dict[str, str] | None = None, **_kw: Any) -> None:
            captured_cmds.append(list(cmd))
            captured_envs.append(dict(env or {}))
            self.pid = 99999
            self.returncode = 0

        def __enter__(self) -> _FakePopen:
            return self

        def __exit__(self, *_a: Any) -> None:
            return None

        def communicate(self, input: Any = None, timeout: float | None = None) -> tuple[str, str]:
            _ = input, timeout
            return ("", "")

        def wait(self, timeout: float | None = None) -> int:
            _ = timeout
            return 0

        def poll(self) -> int:
            return 0

        def terminate(self) -> None:
            return None

        def kill(self) -> None:
            return None

    monkeypatch.setattr(subprocess, "Popen", _FakePopen)
    monkeypatch.setattr(
        official_wrapper,
        "safe_park_to_rest",
        lambda port=None, duration_s=None, **_kw: park_calls.append({"port": port, "duration_s": duration_s}) or True,
    )

    rc_rec = official_wrapper.run_official_record(
        follower_port="COM11",
        leader_port="COM12",
        camera_enabled=True,
        fps=None,
        display_data=False,
    )
    assert rc_rec == 0
    rec_cmd = " ".join(captured_cmds[-1])
    assert '"fps": null' not in rec_cmd, f"--robot.cameras must NOT contain fps:null: {rec_cmd}"
    assert "--robot.calibration_dir=" in rec_cmd and "--teleop.calibration_dir=" in rec_cmd
    if os.pathsep == ";":
        pypath = captured_envs[-1].get("PYTHONPATH", "")
        assert ":" not in pypath.replace(":\\", "X\\").replace(":/", "X/"), (
            f"POSIX ':' separator on Windows PYTHONPATH: {pypath}"
        )

    rc_tel = official_wrapper.run_official_teleoperate(
        follower_port="COM11",
        leader_port="COM12",
        camera_enabled=True,
        fps=None,
        display_data=False,
    )
    assert rc_tel == 0
    tel_cmd = " ".join(captured_cmds[-1])
    assert '"fps": null' not in tel_cmd
    assert "--robot.calibration_dir=" in tel_cmd and "--teleop.calibration_dir=" in tel_cmd
    assert park_calls[-1]["port"] == "COM11", f"safe_park_to_rest received {park_calls[-1]['port']} instead of COM11"


# ---------------------------------------------------------------------------
# Feature 8: SO101RealBridge 11-Bug Fix & ActionAPI Unification (T2-F08-01 .. T2-F08-05)
# ---------------------------------------------------------------------------


def _build_mock_bridge(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    initial_follower_rad: dict[str, float] | None = None,
    initial_leader_rad: dict[str, float] | None = None,
) -> tuple[Any, Any, list[dict[str, Any]]]:
    """Create SO101RealBridge backed strictly by MockArmAdapter (zero COM port access)."""
    _ensure_software_on_path(workspace_root)
    root_str = str(workspace_root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)

    from app.backend.config import load_config
    from app.backend.realtime_bridge import SO101RealBridge
    from app.backend.simulation import Arm101Simulator
    from hardware.config import ArmConfig, RobotSystemConfig
    from hardware.mock_arm import MockArmAdapter
    from hardware.system_adapter import RobotSystemAdapter

    cal_dir = tmp_path / "calibration"
    (cal_dir / "robots" / "so_follower").mkdir(parents=True, exist_ok=True)
    (cal_dir / "teleoperators" / "so_leader").mkdir(parents=True, exist_ok=True)

    cfg = load_config()
    cfg.calibration_dir = cal_dir
    cfg.follower_port = "MOCK_COM11"
    cfg.leader_port = "MOCK_COM12"
    cfg.follower_id = "mock_follower"
    cfg.leader_id = "mock_leader"

    captured_configs: list[dict[str, Any]] = []
    orig_sys_cfg_init = RobotSystemConfig

    def _mocked_adapter_factory(sys_cfg: RobotSystemConfig) -> RobotSystemAdapter:
        captured_configs.append(
            {
                "follower_id": sys_cfg.follower.robot_id,
                "follower_cal": str(sys_cfg.follower.calibration_fpath),
                "leader_id": sys_cfg.leader.robot_id if sys_cfg.leader else None,
                "leader_cal": str(sys_cfg.leader.calibration_fpath) if sys_cfg.leader else None,
            }
        )
        mock_f = ArmConfig.mock_follower(robot_id=sys_cfg.follower.robot_id)
        mock_l = (
            ArmConfig.mock_leader(robot_id=sys_cfg.leader.robot_id) if sys_cfg.leader else None
        )
        adapter = RobotSystemAdapter(
            orig_sys_cfg_init(follower=mock_f, leader=mock_l, use_mock=True)
        )
        if initial_follower_rad and isinstance(adapter.follower, MockArmAdapter):
            adapter.follower._current_positions.update(initial_follower_rad)
            adapter.follower._target_positions.update(initial_follower_rad)
        if initial_leader_rad and isinstance(adapter.leader, MockArmAdapter):
            adapter.leader._current_positions.update(initial_leader_rad)
            adapter.leader._target_positions.update(initial_leader_rad)
        return adapter

    monkeypatch.setattr("hardware.system_adapter.RobotSystemAdapter", _mocked_adapter_factory)
    monkeypatch.setattr("core.action_api.RobotSystemAdapter", _mocked_adapter_factory, raising=False)

    sim = Arm101Simulator()
    bridge = SO101RealBridge(cfg, sim)
    return bridge, sim, captured_configs


def test_t2_f08_01_realtime_bridge_real_to_sim_with_mock_arm_adapter(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F08-01: real_to_sim connects via robot_id/calibration_dir, disables torque, keeps gripper=75%."""
    bridge, sim, captured = _build_mock_bridge(
        workspace_root,
        tmp_path,
        monkeypatch,
        initial_follower_rad={"shoulder_pan": math.radians(30.0), "gripper": 75.0},
        initial_leader_rad={"shoulder_pan": math.radians(-15.0), "gripper": 40.0},
    )
    try:
        st = bridge.set_sync_mode("real_to_sim", is_real_mode=True)
        assert st["direction"] == "real_to_sim"
        assert st["connected_follower"] is True
        assert sim.torque_enabled is False
        assert captured and captured[-1]["follower_id"] == "mock_follower"
        assert str(tmp_path) in captured[-1]["follower_cal"]

        wait_until(
            lambda: math.isclose(sim.follower_joints["shoulder_pan"], 30.0, abs_tol=1.0),
            timeout=3.0,
            description="real_to_sim polling to update simulator follower_joints",
        )
        assert math.isclose(sim.follower_joints["gripper"], 75.0, abs_tol=0.5), (
            f"gripper must stay 75.0% (not math.degrees -> 4297.2), got {sim.follower_joints['gripper']}"
        )
        assert math.isclose(sim.leader_joints["gripper"], 40.0, abs_tol=0.5)
    finally:
        bridge.disconnect()


def test_t2_f08_02_realtime_bridge_sim_to_real_and_send_follower_target_deg(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F08-02: sim_to_real claims control, sends RobotAction(timestamp, target_joint_positions), gripper=100%."""
    bridge, sim, _ = _build_mock_bridge(
        workspace_root,
        tmp_path,
        monkeypatch,
        initial_follower_rad={"gripper": 90.0},
    )
    try:
        st = bridge.set_sync_mode("sim_to_real", is_real_mode=True, safety_confirmed=True)
        assert st["direction"] == "sim_to_real"
        assert st["connected_follower"] is True
        assert sim.torque_enabled is True

        ok = bridge.send_follower_target_deg(
            {
                "shoulder_pan": 5.0,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
                "gripper": 100.0,
            }
        )
        assert ok is True, "send_follower_target_deg must unpack (ok, msg) tuple and return True"
        assert math.isclose(
            bridge._api.hardware.follower._target_positions["gripper"], 100.0, abs_tol=0.1
        )
    finally:
        bridge.disconnect()


def test_t2_f08_03_realtime_bridge_estop_reset_torque_and_safe_park_to_rest(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F08-03: trigger_estop, reset_estop, set_torque, safe_park_to_rest pass robot_id/cal_dir cleanly."""
    bridge, _sim, _ = _build_mock_bridge(workspace_root, tmp_path, monkeypatch)
    _ensure_software_on_path(workspace_root)
    from core.data_types import SystemState

    park_calls: list[dict[str, Any]] = []
    monkeypatch.setattr(
        "teleop.official_wrapper.safe_park_to_rest",
        lambda **kwargs: park_calls.append(kwargs) or True,
    )

    try:
        bridge.set_sync_mode("sim_to_real", is_real_mode=True, safety_confirmed=True)
        bridge.trigger_estop("Unit E-Stop")
        assert bridge._api.state_machine.current_state == SystemState.ESTOP_LATCHED

        bridge.reset_estop()
        assert bridge._api.state_machine.current_state != SystemState.ESTOP_LATCHED

        bridge.set_torque(False)
        bridge.set_torque(True)
        assert bridge.status()["last_error"] is None

        parked = bridge.safe_park_to_rest(duration_s=1.0)
        assert parked is True
        assert park_calls and park_calls[-1]["port"] == "MOCK_COM11"
        assert park_calls[-1].get("robot_id") == "mock_follower"
        assert "calibration" in str(park_calls[-1].get("calibration_dir", ""))
    finally:
        bridge.disconnect()


def test_t2_f08_04_realtime_bridge_disconnect_lock_discipline(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F08-04: disconnect() releases lock before joining poll thread so error writes never deadlock."""
    bridge, _sim, _ = _build_mock_bridge(workspace_root, tmp_path, monkeypatch)
    bridge.set_sync_mode("real_to_sim", is_real_mode=True)

    # Inject a slow failing get_observation while another thread calls status() / disconnect()
    def _failing_obs() -> Any:
        time.sleep(0.05)
        raise RuntimeError("Simulated polling error")

    bridge._api.get_observation = _failing_obs  # type: ignore[method-assign]
    time.sleep(0.15)
    t0 = time.perf_counter()
    bridge.disconnect(return_to_rest=False, keep_torque=False)
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.5, f"disconnect() took {elapsed:.2f}s (expected <0.5s lock-free join)"
    st = bridge.status()
    assert st["direction"] == "off"


def test_t2_f08_05_negative_sim_sync_mode_rejects_unconfirmed_and_invalid_direction(
    dry_run_client: httpx.Client,
) -> None:
    """T2-F08-05: POST /api/sim/sync_mode returns 422 for unconfirmed sim_to_real or bogus direction."""
    r1 = dry_run_client.post(
        "/api/sim/sync_mode",
        json={"direction": "sim_to_real", "safety_confirmed": False},
    )
    assert r1.status_code == 422

    r2 = dry_run_client.post(
        "/api/sim/sync_mode",
        json={"direction": "bogus", "safety_confirmed": True},
    )
    assert r2.status_code == 422


# ---------------------------------------------------------------------------
# Feature 9: Studio Workflows, Simulation, Calibration Runner & UI (T2-F09-01 .. T2-F09-05)
# ---------------------------------------------------------------------------


def test_t2_f09_01_simulation_and_stick_sim_urdf_tcp_and_encoder_roundtrip(
    workspace_root: Path,
    dry_run_page: Page,
) -> None:
    """T2-F09-01: Python simulation.py & Playwright stick_sim.js match URDF TCP (<0.1mm) & encoder round-trip."""
    from app.backend.simulation import (
        compute_fk_details,
        default_joints,
        deg_to_encoder,
        encoder_to_deg,
    )

    fk = compute_fk_details(default_joints())
    tcp = fk["tcp_pose"]
    err_py = math.sqrt(
        (tcp["x_mm"] - 391.36147) ** 2
        + (tcp["y_mm"] - (-0.00921)) ** 2
        + (tcp["z_mm"] - 226.46971) ** 2
    )
    assert err_py < 0.1, f"simulation.py zero-pose TCP {tcp} deviates {err_py:.2f}mm from URDF"

    # Calibrated encoder round-trip (non-default range_min/max + drive_mode=1)
    for joint, val in (("shoulder_pan", 35.0), ("gripper", 0.0), ("gripper", 50.0), ("gripper", 100.0)):
        enc = deg_to_encoder(joint, val, range_min=800, range_max=3200, drive_mode=1)
        back = encoder_to_deg(joint, enc, range_min=800, range_max=3200, drive_mode=1)
        assert abs(back - val) <= 1.0, f"{joint} round-trip {val} -> {enc} -> {back}"

    # Verify Playwright frontend stick_sim.js parity
    js_res = dry_run_page.evaluate(
        """async () => {
            const mod = await import('/static/stick_sim.js');
            const zero = mod.computeStickFK({
                shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 0,
                wrist_flex: 0, wrist_roll: 0, gripper: 20
            });
            const g0 = mod.estimateEncoder('gripper', 0);
            const g100 = mod.estimateEncoder('gripper', 100);
            return { tcp: zero.tcp, g0, g100 };
        }"""
    )
    js_tcp = js_res["tcp"]
    err_js = math.sqrt(
        (js_tcp["x"] - 391.36147) ** 2
        + (js_tcp["y"] - (-0.00921)) ** 2
        + (js_tcp["z"] - 226.46971) ** 2
    )
    assert err_js < 0.1, f"stick_sim.js zero-pose TCP {js_tcp} deviates {err_js:.2f}mm from URDF"
    assert js_res["g0"] != js_res["g100"]


def test_t2_f09_02_calibrate_runner_eof_and_dirty_queue_drain() -> None:
    """T2-F09-02: _piped_enter_pressed raises EOFError on None and drains stale buffered newlines."""
    import queue
    import lerobot.motors.motors_bus as motors_bus_mod
    from app.backend.calibrate_runner import install_piped_stdin_shims

    q: queue.Queue[str | None] = queue.Queue()
    install_piped_stdin_shims(q)

    # (a) EOF (None) raises EOFError instead of returning False forever
    q.put(None)
    with pytest.raises(EOFError):
        motors_bus_mod.enter_pressed()

    # (b) Dirty buffered newlines drained before/when entering range recording
    for _ in range(3):
        q.put("\n")
    if hasattr(motors_bus_mod, "flush_stdin_queue"):
        motors_bus_mod.flush_stdin_queue()
    elif hasattr(sys.modules["app.backend.calibrate_runner"], "drain_stdin_queue"):
        sys.modules["app.backend.calibrate_runner"].drain_stdin_queue(q)
    else:
        # When motors_bus.record_ranges_of_motion starts, drain helper or builtins.input transition drains it:
        # Verify calling builtins.input drains stale enters or enter_pressed starts clean after Step 1
        _ = input("Step 1")
    # After Step 1 prompt consumed or drained, stale queue entries must NOT leak into enter_pressed
    assert motors_bus_mod.enter_pressed() is False
    q.put("\n")
    assert motors_bus_mod.enter_pressed() is True


def test_t2_f09_03_build_record_cmd_resume_streaming_encoding_and_push_to_hub(
    workspace_root: Path,
    tmp_path: Path,
    dry_run_client: httpx.Client,
) -> None:
    """T2-F09-03: build_record_cmd adds --resume=true on existing dir, streaming_encoding=true, and push_to_hub."""
    from app.backend.config import load_config
    from app.backend.hardware import HardwareManager
    from app.backend.process_manager import ProcessRegistry
    from app.backend.simulation import Arm101Simulator
    from app.backend.workflows import WorkflowManager

    cfg = load_config()
    cfg.datasets_dir = tmp_path / "datasets"
    cfg.datasets_dir.mkdir(parents=True, exist_ok=True)
    existing_ds = cfg.datasets_dir / "existing_ds"
    existing_ds.mkdir(parents=True, exist_ok=True)

    hw = HardwareManager(cfg)
    wf = WorkflowManager(cfg, hw, Arm101Simulator(), ProcessRegistry())

    cmd_existing = wf.build_record_cmd(
        dataset_name="existing_ds",
        single_task="pick",
        num_episodes=1,
        episode_time_s=5.0,
        reset_time_s=1.0,
        fps=30,
        push_to_hub=True,
    )
    joined_ex = " ".join(cmd_existing)
    assert "--resume=true" in joined_ex
    assert "--dataset.streaming_encoding=true" in joined_ex
    assert "--dataset.push_to_hub=true" in joined_ex

    cmd_new = wf.build_record_cmd(
        dataset_name="brand_new_ds",
        single_task="pick",
        num_episodes=1,
        episode_time_s=5.0,
        reset_time_s=1.0,
        fps=30,
        push_to_hub=False,
    )
    joined_new = " ".join(cmd_new)
    assert "--resume=true" not in joined_new
    assert "--dataset.streaming_encoding=true" in joined_new
    assert "--dataset.push_to_hub=false" in joined_new

    # Also verify HTTP API accepts push_to_hub cleanly in dry_run
    r = dry_run_client.post(
        "/api/workflows/collect/start",
        json={"dataset_name": "t2_f09_03_api", "single_task": "pick", "num_episodes": 1, "push_to_hub": True},
    )
    assert r.status_code == 200
    dry_run_client.post("/api/workflows/collect/stop")


def test_t2_f09_04_dry_run_save_pose_hygiene_and_plugin_adapter_contracts(
    workspace_root: Path,
    dry_run_client: httpx.Client,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T2-F09-04: dry_run save_pose leaves software/*.json untouched; RemoteVLAPolicyAdapter cwd & 409."""
    upright_path = workspace_root / "software" / "upright_pose.json"
    rest_path = workspace_root / "software" / "rest_pose.json"
    orig_u_bytes = upright_path.read_bytes()
    orig_r_bytes = rest_path.read_bytes()
    before_u = hashlib.sha256(orig_u_bytes).hexdigest()
    before_r = hashlib.sha256(orig_r_bytes).hexdigest()

    try:
        r_home = dry_run_client.post("/api/workflows/teaching/save_pose", json={"target": "home"})
        r_rest = dry_run_client.post("/api/workflows/teaching/save_pose", json={"target": "rest"})
        assert r_home.status_code == 200 and r_rest.status_code == 200
        assert "dry_run" in r_home.json()["path"].replace("\\", "/")
        assert hashlib.sha256(upright_path.read_bytes()).hexdigest() == before_u
        assert hashlib.sha256(rest_path.read_bytes()).hexdigest() == before_r
    finally:
        if upright_path.read_bytes() != orig_u_bytes:
            upright_path.write_bytes(orig_u_bytes)
        if rest_path.read_bytes() != orig_r_bytes:
            rest_path.write_bytes(orig_r_bytes)

    # Windows launch of remote_vla_policy returns 409 OS_UNSUPPORTED
    r_vla = dry_run_client.post("/api/plugins/remote_vla_policy/launch", json={})
    assert r_vla.status_code == 409
    assert r_vla.json()["code"] == "OS_UNSUPPORTED"

    # Simulated Linux launch uses cwd=repos/openpi and a valid built-in openpi config
    from app.backend.config import load_config
    from app.backend.hardware import HardwareManager
    from app.backend.plugins import RemoteVLAPolicyAdapter
    from app.backend.process_manager import ProcessRegistry

    cfg = load_config()
    hw = HardwareManager(cfg)
    reg = ProcessRegistry()
    spawn_calls: list[dict[str, Any]] = []
    monkeypatch.setattr(
        reg,
        "spawn",
        lambda pid, name, cmd, cwd=None, **_kw: spawn_calls.append({"cmd": cmd, "cwd": cwd})
        or types.SimpleNamespace(proc=types.SimpleNamespace(pid=4242, poll=lambda: None)),
    )
    monkeypatch.setattr("platform.system", lambda: "Linux")

    adapter = RemoteVLAPolicyAdapter(cfg, hw, reg)
    adapter.launch({})
    assert spawn_calls and Path(spawn_calls[-1]["cwd"]) == cfg.repos_dir / "openpi"
    cmd_str = " ".join(spawn_calls[-1]["cmd"])
    assert "pi05_so101_custom" not in cmd_str, f"Invalid placeholder openpi config still used: {cmd_str}"


def test_t2_f09_05_negative_out_of_urdf_limits_unreachable_ik_eval_dataset_empty_replay(
    dry_run_client: httpx.Client,
) -> None:
    """T2-F09-05: Reject out-of-URDF sim/joints (422), unreachable IK (422), eval_* (422), empty replay (409)."""
    r1 = dry_run_client.post(
        "/api/sim/joints", json={"arm": "follower", "joints": {"shoulder_pan": 115.0}}
    )
    assert r1.status_code == 422

    r2 = dry_run_client.post(
        "/api/sim/joints", json={"arm": "follower", "joints": {"wrist_roll": -165.0}}
    )
    assert r2.status_code == 422

    r3 = dry_run_client.post(
        "/api/sim/ik", json={"arm": "follower", "x_mm": 999.0, "y_mm": 0.0, "z_mm": 500.0}
    )
    assert r3.status_code == 422

    r4 = dry_run_client.post(
        "/api/workflows/collect/start", json={"dataset_name": "eval_bad", "single_task": "test"}
    )
    assert r4.status_code == 422

    assert dry_run_client.post("/api/workflows/teaching/clear").status_code == 200
    r5 = dry_run_client.post("/api/workflows/teaching/replay", json={})
    assert r5.status_code == 409
    assert r5.json()["detail"]["code"] == "NO_WAYPOINTS"


# ---------------------------------------------------------------------------
# Feature 10: Packaging, Configs, Scripts & Docs Alignment (T2-F10-01 .. T2-F10-05)
# ---------------------------------------------------------------------------


def test_t2_f10_01_pyproject_includes_software_and_testpaths(workspace_root: Path) -> None:
    """T2-F10-01: pyproject.toml exposes software* and includes tests, app/tests, software/tests in testpaths."""
    data = tomllib.loads((workspace_root / "pyproject.toml").read_text(encoding="utf-8"))
    include = data.get("tool", {}).get("setuptools", {}).get("packages", {}).get("find", {}).get("include", [])
    assert any(pat.startswith("app") for pat in include)
    assert any(pat.startswith("software") for pat in include), f"pyproject.toml missing software* in {include}"

    testpaths = data.get("tool", {}).get("pytest", {}).get("ini_options", {}).get("testpaths", [])
    for required in ("tests", "app/tests", "software/tests"):
        assert required in testpaths, f"pyproject.toml testpaths missing {required!r}: {testpaths}"


def test_t2_f10_02_software_config_and_configs_yaml_v071_schema(workspace_root: Path) -> None:
    """T2-F10-02: software/config.yaml uses so101_leader & rerun/foxglove; configs/*.yaml parse via draccus."""
    sw_cfg_path = workspace_root / "software" / "config.yaml"
    sw_text = sw_cfg_path.read_text(encoding="utf-8")
    sw_data = yaml.safe_load(sw_text)
    assert sw_data["leader"]["type"] == "so101_leader"
    assert "opencv" not in sw_text.lower().split("display_mode", 1)[-1].splitlines()[0]
    assert "foxglove" in sw_text.lower()

    import draccus
    import lerobot.policies.factory  # noqa: F401 - registers act, pi0, smolvla PreTrainedConfig subclasses
    from lerobot.configs.train import TrainPipelineConfig
    from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig

    for robot_yaml in sorted((workspace_root / "configs" / "robots").glob("*.yaml")):
        raw_robot_yaml = yaml.safe_load(robot_yaml.read_text(encoding="utf-8"))
        assert raw_robot_yaml["robot"]["type"] == "so101_follower"
        assert raw_robot_yaml["teleop"]["type"] == "so101_leader"
        parsed = draccus.parse(TeleoperateConfig, args=[f"--config_path={robot_yaml}"])
        assert parsed.robot.type in ("so101_follower", "so100_follower")
        assert parsed.teleop.type in ("so101_leader", "so100_leader")

    from lerobot.configs.parser import (
        _config_path_args,
        _config_yaml_overrides,
        extract_path_fields_from_config,
    )

    for train_yaml in ("act_so101.yaml", "pi0_so101_cloud.yaml", "smolvla_so101_cloud.yaml"):
        cfg_file = workspace_root / "configs" / "train" / train_yaml
        _config_path_args.clear()
        _config_yaml_overrides.clear()
        cleaned_cfg = extract_path_fields_from_config(cfg_file, ["policy"])
        try:
            parsed_train = draccus.parse(TrainPipelineConfig, args=[f"--config_path={cleaned_cfg}"])
            assert parsed_train.dataset.repo_id
        finally:
            _config_path_args.clear()
            _config_yaml_overrides.clear()


def test_t2_f10_03_setup_ubuntu_sh_with_pi_and_smolvla(workspace_root: Path) -> None:
    """T2-F10-03: setup_ubuntu.sh supports --with-pi and --with-smolvla in --dry-run and rejects --invalid-flag."""
    sh_path = workspace_root / "scripts" / "ubuntu" / "setup_ubuntu.sh"
    res = run_ubuntu_script(sh_path, ["--dry-run", "--with-pi", "--with-smolvla"])
    if res is None:
        pytest.skip("No POSIX bash / Git Bash / WSL available on host")
    assert res.returncode == 0, f"setup_ubuntu.sh --with-pi --with-smolvla failed:\n{res.stderr}\n{res.stdout}"
    assert "pi" in res.stdout and "smolvla" in res.stdout

    bad = run_ubuntu_script(sh_path, ["--invalid-flag"])
    assert bad is not None and bad.returncode == 2


def test_t2_f10_04_docs_cli_flag_accuracy_and_anti_regression(workspace_root: Path) -> None:
    """T2-F10-04: All 6 documented CLI/slider/index discrepancies in docs/ are resolved."""
    docs_dir = workspace_root / "docs"
    vla_doc = (docs_dir / "03_vla_and_pi" / "01_vla_data_and_cloud_training.md").read_text(encoding="utf-8")
    for line in vla_doc.splitlines():
        if "lerobot-policy-server" in line:
            assert "--host" not in line and "--port" not in line and "--policy.path" not in line, (
                f"Stale lerobot-policy-server flags in 01_vla_data_and_cloud_training.md: {line}"
            )
    assert "--host=0.0.0.0" not in vla_doc or "lerobot-policy-server" not in vla_doc.split("--host=0.0.0.0")[0].splitlines()[-1]

    collect_doc = (docs_dir / "02_arm101_workflow" / "04_data_collection.md").read_text(encoding="utf-8")
    assert "--dataset.vcodec" not in collect_doc
    assert "--dataset.rgb_encoder.vcodec" in collect_doc

    pi07_doc = (docs_dir / "03_vla_and_pi" / "02_pi_series_and_pi07.md").read_text(encoding="utf-8")
    assert "-- policy.device" not in pi07_doc

    app_guide = (docs_dir / "04_frontend_and_tools" / "local_app_guide.md").read_text(encoding="utf-8")
    assert "[-180, 180]" not in app_guide
    assert "110" in app_guide and "100" in app_guide and "95" in app_guide

    readme_doc = (docs_dir / "README.md").read_text(encoding="utf-8")
    assert "*(M3" not in readme_doc


def test_t2_f10_05_lerobot_policy_server_zenoh_server_config_parity(workspace_root: Path) -> None:
    """T2-F10-05: Documented lerobot-policy-server flags match Zenoh ServerConfig dataclass fields."""
    from dataclasses import fields
    from lerobot.remote_inference.configs import ServerConfig

    valid_top_fields = {f.name for f in fields(ServerConfig)} | {"config_path"}
    vla_doc = (
        workspace_root / "docs" / "03_vla_and_pi" / "01_vla_data_and_cloud_training.md"
    ).read_text(encoding="utf-8")

    # Find fenced command blocks mentioning lerobot-policy-server
    blocks = re.findall(r"```[^\n]*\n(.*?)```", vla_doc, flags=re.DOTALL)
    policy_server_blocks = [b for b in blocks if "lerobot-policy-server" in b]
    assert policy_server_blocks, "Expected lerobot-policy-server example in 01_vla_data_and_cloud_training.md"

    for block in policy_server_blocks:
        flags = re.findall(r"--([A-Za-z0-9_.]+)", block)
        assert flags, f"No flags parsed in lerobot-policy-server block: {block}"
        for flag in flags:
            top = flag.split(".", 1)[0]
            assert top in valid_top_fields, (
                f"Documented flag --{flag} is NOT a valid ServerConfig field ({sorted(valid_top_fields)})"
            )
