"""Tier 3 E2E Tests: Pairwise Cross-Feature Interactions & Shared Contracts (T3-01..T3-12).

Covers all 12 pairwise cross-feature workflows from spec_matrix.md §4.2.
Strictly hardware-isolated: forces STUDIO_MODE="dry_run" and mocks all serial/camera I/O
so real COM11/COM12 arms and USB cameras are never accessed.
"""

from __future__ import annotations

import hashlib
import json
import math
import queue
import sys
import time
import tomllib
import types
from collections.abc import Callable, Generator
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import httpx
import pytest
import yaml
from playwright.sync_api import Browser, Page

from tests.e2e._helpers import run_cmd, wait_until
from tests.e2e.conftest import DryRunSandbox, StudioServerHandle

pytestmark = pytest.mark.tier3


# ---------------------------------------------------------------------------
# Hardware-Safe Local Fixtures & Helpers
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


def _ensure_paths(workspace_root: Path) -> None:
    root_str = str(workspace_root)
    sw_str = str(workspace_root / "software")
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    if sw_str not in sys.path:
        sys.path.insert(0, sw_str)


class _FakeRustypotBus:
    """In-memory STS3215 fake rustypot.Bus for zero-COM end-to-end bus verification."""

    def __init__(self) -> None:
        self.regs: dict[int, dict[str, int]] = {
            mid: {
                "id": mid,
                "model_number": 777,
                "firmware_major_version": 3,
                "firmware_minor_version": 10,
                "return_delay_time": 0,
                "min_position_limit": 500,
                "max_position_limit": 3500,
                "max_torque_limit": 1000,
                "phase": 0,
                "p_coefficient": 16,
                "i_coefficient": 0,
                "d_coefficient": 32,
                "protection_current": 500,
                "homing_offset": 0,
                "operating_mode": 0,
                "overload_torque": 80,
                "torque_enable": 1,
                "acceleration": 254,
                "goal_position": 2000,
                "lock": 1,
                "present_position": 2000,
                "maximum_acceleration": 254,
            }
            for mid in range(1, 7)
        }
        self.sync_writes: list[tuple[list[int], str, list[int]]] = []

    def broadcast_scan(self, _defn: Any) -> dict[int, int]:
        return {mid: 777 for mid in self.regs}

    def scan(self, _defn: Any) -> dict[int, int]:
        return {mid: 777 for mid in self.regs}

    def read_register_with_error(self, motor_id: int, reg: str, retries: int = 0) -> tuple[int, int]:
        _ = retries
        return int(self.regs[motor_id].get(reg.lower(), 0)), 0

    def write_register_with_error(
        self, motor_id: int, reg: str, value: int, retries: int = 0
    ) -> int:
        _ = retries
        self.regs[motor_id][reg.lower()] = int(value)
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
        for mid in ids:
            self.regs[mid]["torque_enable"] = 1 if enabled else 0
            self.regs[mid]["lock"] = 1 if enabled else 0
        return {}

    def close(self) -> None:
        return None


# ---------------------------------------------------------------------------
# T3-01 .. T3-12 Pairwise Cross-Feature Tests
# ---------------------------------------------------------------------------


def test_t3_01_f5_f6_motor_bus_to_safety_controller_and_action_api_units(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T3-01 (F5 <-> F6): ActionAPI -> SafetyController -> SO101FollowerArm -> SerialMotorsBus preserves gripper=100%."""
    _ensure_paths(workspace_root)
    from core.action_api import ActionAPI
    from core.data_types import ActionType, ControlSource, JointUnit, RobotAction
    from core.safety_controller import SafetyController, SafetyLimitsConfig
    from core.state_machine import RobotStateMachine
    from hardware.config import ArmConfig, ArmRole, RobotSystemConfig
    from hardware.system_adapter import RobotSystemAdapter
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cal_dir = tmp_path / "cal"
    cal_dir.mkdir(parents=True, exist_ok=True)
    cal_file = cal_dir / "t3_01.json"
    cal_data = {
        name: {
            "id": idx + 1,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 500,
            "range_max": 3500,
        }
        for idx, name in enumerate(
            ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
        )
    }
    cal_file.write_text(json.dumps(cal_data), encoding="utf-8")

    fake_hw = _FakeRustypotBus()

    class _MockedFollower(SO101Follower):
        def __init__(self, cfg: SO101FollowerConfig) -> None:
            super().__init__(cfg)
            self.bus._open_bus = lambda: fake_hw  # type: ignore[method-assign]
            self.bus._handshake = lambda: None  # type: ignore[method-assign]

    monkeypatch.setattr("lerobot.robots.so_follower.SO101Follower", _MockedFollower)

    arm_cfg = ArmConfig(
        role=ArmRole.FOLLOWER,
        port="MOCK_PORT",
        robot_id="t3_01",
        calibration_fpath=cal_file,
        use_degrees=True,
        internal_unit=JointUnit.RADIAN,
    )
    adapter = RobotSystemAdapter(RobotSystemConfig(follower=arm_cfg, use_mock=False))
    sm = RobotStateMachine()
    sc = SafetyController(sm, SafetyLimitsConfig(max_joint_delta_rad=10.0))
    api = ActionAPI(hardware=adapter, state_machine=sm, safety_controller=sc)

    assert api.connect(connect_leader=False) is True
    assert api.claim_control(ControlSource.UI) is True
    try:
        # Step gripper smoothly within SafetyController max_gripper_delta (15%/step) up to 100.0%
        for grip_step in (65.0, 80.0, 95.0, 100.0):
            action = RobotAction(
                action_type=ActionType.JOINT_POSITION,
                source=ControlSource.UI,
                timestamp=time.monotonic(),
                target_joint_positions={
                    "shoulder_pan": math.radians(105.0),
                    "shoulder_lift": 0.0,
                    "elbow_flex": 0.0,
                    "wrist_flex": 0.0,
                    "wrist_roll": 0.0,
                    "gripper": grip_step,
                },
                unit=JointUnit.RADIAN,
            )
            ok, msg = api.submit_action(action)
            assert ok is True, f"submit_action failed at gripper={grip_step}: {msg}"
        # SerialMotorsBus must receive raw range_max=3500 for gripper=100.0% (no 85.94% cap!)
        assert fake_hw.regs[6]["goal_position"] == 3500, (
            f"Expected gripper raw=3500 for 100.0%, got {fake_hw.regs[6]['goal_position']}"
        )
    finally:
        api.disconnect(keep_torque=False, return_to_rest=False)


def test_t3_02_f5_f8_calibration_storage_propagated_across_bridge(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T3-02 (F5 <-> F8): Custom calibration_dir <id>.json propagates from StudioConfig -> Bridge -> ArmConfig -> SO101FollowerConfig."""
    _ensure_paths(workspace_root)
    from app.backend.config import load_config
    from app.backend.realtime_bridge import SO101RealBridge
    from app.backend.simulation import Arm101Simulator
    from hardware.config import ArmConfig, RobotSystemConfig
    from hardware.system_adapter import RobotSystemAdapter
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cal_root = tmp_path / "studio_cal"
    f_dir = cal_root / "robots" / "so_follower"
    l_dir = cal_root / "teleoperators" / "so_leader"
    f_dir.mkdir(parents=True, exist_ok=True)
    l_dir.mkdir(parents=True, exist_ok=True)

    cal_payload = {
        name: {
            "id": idx + 1,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": 600,
            "range_max": 3400,
        }
        for idx, name in enumerate(
            ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
        )
    }
    (f_dir / "custom_follower_id.json").write_text(json.dumps(cal_payload), encoding="utf-8")
    (l_dir / "custom_leader_id.json").write_text(json.dumps(cal_payload), encoding="utf-8")

    fake_hw = _FakeRustypotBus()
    for mid in range(1, 7):
        fake_hw.regs[mid]["min_position_limit"] = 600
        fake_hw.regs[mid]["max_position_limit"] = 3400

    seen_driver_cfgs: list[SO101FollowerConfig] = []

    class _MockedFollower(SO101Follower):
        def __init__(self, cfg: SO101FollowerConfig) -> None:
            seen_driver_cfgs.append(cfg)
            super().__init__(cfg)
            self.bus._open_bus = lambda: fake_hw  # type: ignore[method-assign]
            self.bus._handshake = lambda: None  # type: ignore[method-assign]

    monkeypatch.setattr("lerobot.robots.so_follower.SO101Follower", _MockedFollower)

    cfg = load_config()
    cfg.calibration_dir = cal_root
    cfg.follower_port = "MOCK_PORT_F"
    cfg.leader_port = "MOCK_PORT_L"
    cfg.follower_id = "custom_follower_id"
    cfg.leader_id = "custom_leader_id"

    bridge = SO101RealBridge(cfg, Arm101Simulator())
    try:
        st = bridge.set_sync_mode("sim_to_real", is_real_mode=True, safety_confirmed=True)
        assert st["connected_follower"] is True
        assert seen_driver_cfgs, "SO101FollowerConfig was not instantiated"
        assert seen_driver_cfgs[-1].id == "custom_follower_id"
        assert seen_driver_cfgs[-1].calibration_dir == f_dir
        assert bridge._api.hardware.follower.config.calibration_fpath == f_dir / "custom_follower_id.json"
    finally:
        bridge.disconnect()


def test_t3_03_f6_f9_kinematics_simulation_and_stick_sim_js_parity(
    workspace_root: Path,
    dry_run_client: httpx.Client,
    dry_run_page: Page,
) -> None:
    """T3-03 (F6 <-> F9): software/core/kinematics.py, simulation.py, and stick_sim.js agree <0.1mm across 5 poses."""
    _ensure_paths(workspace_root)
    from core.data_types import JointUnit
    from core.kinematics import SO101Kinematics

    kin = SO101Kinematics()
    poses = [
        {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 20.0},
        {"shoulder_pan": 30.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 50.0},
        {"shoulder_pan": -25.0, "shoulder_lift": 20.0, "elbow_flex": -15.0, "wrist_flex": 10.0, "wrist_roll": 120.0, "gripper": 95.0},
        {"shoulder_pan": 30.0, "shoulder_lift": -45.0, "elbow_flex": 60.0, "wrist_flex": -30.0, "wrist_roll": 45.0, "gripper": 80.0},
        {"shoulder_pan": 0.0, "shoulder_lift": 35.0, "elbow_flex": 25.0, "wrist_flex": -60.0, "wrist_roll": -90.0, "gripper": 10.0},
    ]

    for pose in poses:
        resp = dry_run_client.post(
            "/api/sim/joints",
            json={"arm": "follower", "joints": pose, "linked": False},
        )
        assert resp.status_code == 200
        sim_tcp = resp.json()["follower_fk"]["tcp_pose"]
        (kx, ky, kz), _, _ = kin.forward_kinematics(pose, unit=JointUnit.DEGREE)
        js_tcp = dry_run_page.evaluate(
            """async (j) => {
                const mod = await import('/static/stick_sim.js');
                return mod.computeStickFK(j).tcp;
            }""",
            pose,
        )
        err_sw_sim = math.sqrt(
            (kx * 1000.0 - sim_tcp["x_mm"]) ** 2
            + (ky * 1000.0 - sim_tcp["y_mm"]) ** 2
            + (kz * 1000.0 - sim_tcp["z_mm"]) ** 2
        )
        err_js_sim = math.sqrt(
            (js_tcp["x"] - sim_tcp["x_mm"]) ** 2
            + (js_tcp["y"] - sim_tcp["y_mm"]) ** 2
            + (js_tcp["z"] - sim_tcp["z_mm"]) ** 2
        )
        assert err_sw_sim < 0.1, f"Pose {pose}: software vs simulation FK diff={err_sw_sim:.3f}mm"
        assert err_js_sim < 0.1, f"Pose {pose}: stick_sim.js vs simulation FK diff={err_js_sim:.3f}mm"

    # Also test /api/sim/ik round-trip to a reachable URDF target near zero pose
    ik_resp = dry_run_client.post(
        "/api/sim/ik",
        json={"arm": "follower", "x_mm": 350.0, "y_mm": -50.0, "z_mm": 210.0, "gripper": 95.0},
    )
    assert ik_resp.status_code == 200
    assert math.isclose(ik_resp.json()["follower_joints"]["gripper"], 95.0, abs_tol=0.1)


def test_t3_04_f8_f9_bridge_and_workflow_estop_blocks_and_resets(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T3-04 (F8 <-> F9): Active sim_to_real bridge latches ESTOP on /api/safety/estop, blocks 409, and recovers on reset."""
    _ensure_paths(workspace_root)
    from fastapi.testclient import TestClient
    from app.backend.config import load_config
    from app.backend.main import create_app
    from core.data_types import SystemState
    from hardware.config import ArmConfig, RobotSystemConfig
    from hardware.system_adapter import RobotSystemAdapter

    cfg = load_config()
    cfg.requested_mode = "dry_run"
    cfg.data_dir = tmp_path / "data"
    cfg.datasets_dir = tmp_path / "datasets"
    cfg.calibration_dir = tmp_path / "calibration"
    cfg.dry_run_dir = tmp_path / "dry_run"
    cfg.logs_dir = tmp_path / "logs"
    cfg.ensure_directories()

    def _mock_adapter(sys_cfg: RobotSystemConfig) -> RobotSystemAdapter:
        mf = ArmConfig.mock_follower(robot_id=sys_cfg.follower.robot_id)
        ml = ArmConfig.mock_leader(robot_id=sys_cfg.leader.robot_id) if sys_cfg.leader else None
        return RobotSystemAdapter(RobotSystemConfig(follower=mf, leader=ml, use_mock=True))

    monkeypatch.setattr("hardware.system_adapter.RobotSystemAdapter", _mock_adapter)
    monkeypatch.setattr("core.action_api.RobotSystemAdapter", _mock_adapter, raising=False)

    app = create_app(cfg)
    with TestClient(app) as client:
        # Force real-mode bridge with MockArmAdapter inside TestClient
        st = app.state.runtime.workflows.bridge.set_sync_mode(
            "sim_to_real", is_real_mode=True, safety_confirmed=True
        )
        assert st["connected_follower"] is True

        r_estop = client.post("/api/safety/estop", json={"reason": "T3-04 E-Stop"})
        assert r_estop.status_code == 200
        assert app.state.runtime.workflows.bridge._api.state_machine.current_state == SystemState.ESTOP_LATCHED

        assert (
            client.post("/api/sim/joints", json={"arm": "follower", "joints": {"shoulder_pan": 10.0}}).status_code
            == 409
        )
        assert client.post("/api/safety/torque", json={"enabled": True}).status_code == 409

        r_reset = client.post("/api/safety/reset")
        assert r_reset.status_code == 200
        assert app.state.runtime.workflows.bridge._api.state_machine.current_state != SystemState.ESTOP_LATCHED

        r_ok = client.post(
            "/api/sim/joints", json={"arm": "follower", "joints": {"shoulder_pan": 10.0}}
        )
        assert r_ok.status_code == 200


def test_t3_05_f8_f9_bridge_preemption_on_teleop_and_calibrate_start(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T3-05 (F8 <-> F9): Starting teleop or calibrate disconnects active sim_to_real bridge before spawning CLI."""
    _ensure_paths(workspace_root)
    import asyncio
    from app.backend.config import load_config
    from app.backend.hardware import HardwareManager
    from app.backend.process_manager import ProcessRegistry
    from app.backend.simulation import Arm101Simulator
    from app.backend.workflows import WorkflowManager
    from hardware.config import ArmConfig, RobotSystemConfig
    from hardware.system_adapter import RobotSystemAdapter

    cfg = load_config()
    cfg.data_dir = tmp_path / "data"
    cfg.datasets_dir = tmp_path / "datasets"
    cfg.calibration_dir = tmp_path / "cal"
    cfg.dry_run_dir = tmp_path / "dry_run"
    cfg.logs_dir = tmp_path / "logs"
    cfg.ensure_directories()

    def _mock_adapter(sys_cfg: RobotSystemConfig) -> RobotSystemAdapter:
        mf = ArmConfig.mock_follower(robot_id=sys_cfg.follower.robot_id)
        return RobotSystemAdapter(RobotSystemConfig(follower=mf, use_mock=True))

    monkeypatch.setattr("hardware.system_adapter.RobotSystemAdapter", _mock_adapter)
    monkeypatch.setattr("core.action_api.RobotSystemAdapter", _mock_adapter, raising=False)

    hw = HardwareManager(cfg)
    hw.active_mode = "real"
    reg = ProcessRegistry()
    events: list[str] = []

    def _fake_spawn(pid: str, name: str, cmd: list[str], **_kw: Any) -> Any:
        events.append(f"spawn:{pid}:bridge_connected={wf.bridge.connected_follower}")
        return types.SimpleNamespace(proc=types.SimpleNamespace(pid=7777, poll=lambda: None), returncode=None)

    monkeypatch.setattr(reg, "spawn", _fake_spawn)
    monkeypatch.setattr(reg, "is_running", lambda _pid: True)
    monkeypatch.setattr(reg, "terminate", lambda _pid: None)
    monkeypatch.setattr("teleop.official_wrapper.safe_park_to_rest", lambda **_kw: True)

    wf = WorkflowManager(cfg, hw, Arm101Simulator(), reg)

    async def _run() -> None:
        wf.bridge.set_sync_mode("sim_to_real", is_real_mode=True, safety_confirmed=True)
        assert wf.bridge.connected_follower is True
        await wf.start_teleop()
        assert wf.bridge.connected_follower is False
        assert events and "bridge_connected=False" in events[-1]
        await wf.stop_teleop()

        wf.bridge.set_sync_mode("sim_to_real", is_real_mode=True, safety_confirmed=True)
        assert wf.bridge.connected_follower is True
        await wf.start_calibrate(target="follower")
        assert wf.bridge.connected_follower is False
        assert "bridge_connected=False" in events[-1]
        await wf.stop_calibrate()

    with ThreadPoolExecutor(max_workers=1) as pool:
        pool.submit(lambda: asyncio.run(_run())).result(timeout=10.0)


def test_t3_06_f7_f9_official_wrapper_and_workflows_cli_command_parity(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T3-06 (F7 <-> F9): official_wrapper and workflows.py emit consistent lerobot v0.7.1 CLI flags & calibration_dir."""
    _ensure_paths(workspace_root)
    import subprocess
    from app.backend.config import load_config
    from app.backend.hardware import HardwareManager
    from app.backend.process_manager import ProcessRegistry
    from app.backend.simulation import Arm101Simulator
    from app.backend.workflows import WorkflowManager
    from teleop import official_wrapper

    cfg = load_config()
    cfg.front_camera_id = "0"
    cfg.wrist_camera_id = "none"
    wf = WorkflowManager(cfg, HardwareManager(cfg), Arm101Simulator(), ProcessRegistry())

    wf_teleop = wf.build_teleop_cmd(fps=60, follower_port="COM11", leader_port="COM12", use_cameras=True)
    wf_record = wf.build_record_cmd(
        dataset_name="t3_06_ds",
        single_task="pick",
        num_episodes=1,
        episode_time_s=5.0,
        reset_time_s=1.0,
        fps=30,
    )

    wrapper_cmds: list[list[str]] = []

    class _FakePopen:
        def __init__(self, cmd: list[str], **_kw: Any) -> None:
            wrapper_cmds.append(list(cmd))
            self.pid = 8888

        def __enter__(self) -> _FakePopen:
            return self

        def __exit__(self, *_a: Any) -> None:
            return None

        def wait(self) -> int:
            return 0

    monkeypatch.setattr(subprocess, "Popen", _FakePopen)
    monkeypatch.setattr(official_wrapper, "safe_park_to_rest", lambda **_kw: True)
    official_wrapper.run_official_teleoperate(
        follower_port="COM11", leader_port="COM12", camera_enabled=True, fps=None, display_data=False
    )
    official_wrapper.run_official_record(
        follower_port="COM11", leader_port="COM12", camera_enabled=True, fps=None, display_data=False
    )

    for cmd_list in (wf_teleop, wf_record, wrapper_cmds[0], wrapper_cmds[1]):
        joined = " ".join(cmd_list)
        assert "--robot.type=so101_follower" in joined
        assert "--teleop.type=so101_leader" in joined
        assert "--robot.calibration_dir=" in joined
        assert "--teleop.calibration_dir=" in joined
        assert '"fps": null' not in joined


def test_t3_07_f6_f9_presets_workflows_and_dry_run_save_pose_isolation(
    workspace_root: Path,
    dry_run_client: httpx.Client,
) -> None:
    """T3-07 (F6 <-> F9): pick_hover opens gripper >=60%, rest disables torque, dry_run save_pose spares software/*.json."""
    upright_p = workspace_root / "software" / "upright_pose.json"
    rest_p = workspace_root / "software" / "rest_pose.json"
    orig_u_bytes = upright_p.read_bytes()
    orig_r_bytes = rest_p.read_bytes()
    sha_u = hashlib.sha256(orig_u_bytes).hexdigest()
    sha_r = hashlib.sha256(orig_r_bytes).hexdigest()

    try:
        r_hover = dry_run_client.post(
            "/api/workflows/teaching/pose", json={"pose": "pick_hover", "duration_s": 0.2}
        )
        assert r_hover.status_code == 200
        assert r_hover.json()["target_joints"]["gripper"] >= 60.0

        wait_until(
            lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
            timeout=5.0,
            description="pick_hover preset completion",
        )

        r_rest = dry_run_client.post(
            "/api/workflows/teaching/pose", json={"pose": "rest", "duration_s": 0.2}
        )
        assert r_rest.status_code == 200
        wait_until(
            lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
            timeout=5.0,
            description="rest preset completion",
        )

        r_pick = dry_run_client.post("/api/workflows/teaching/routine", json={"routine": "pick"})
        assert r_pick.status_code == 200
        wait_until(
            lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
            timeout=8.0,
            description="pick routine completion",
        )

        r_save = dry_run_client.post("/api/workflows/teaching/save_pose", json={"target": "rest"})
        assert r_save.status_code == 200
        assert "dry_run" in r_save.json()["path"].replace("\\", "/")
        assert hashlib.sha256(upright_p.read_bytes()).hexdigest() == sha_u
        assert hashlib.sha256(rest_p.read_bytes()).hexdigest() == sha_r
    finally:
        if upright_p.read_bytes() != orig_u_bytes:
            upright_p.write_bytes(orig_u_bytes)
        if rest_p.read_bytes() != orig_r_bytes:
            rest_p.write_bytes(orig_r_bytes)


def test_t3_08_f7_f9_concurrent_dry_run_camera_frames_across_workflows(
    dry_run_studio: StudioServerHandle,
    dry_run_client: httpx.Client,
) -> None:
    """T3-08 (F7 <-> F9): Concurrent /api/cameras/{front,wrist}/frame return valid JPEGs in idle, teleop, collect."""
    def _fetch_both() -> list[bytes]:
        def _one(cid: str) -> bytes:
            r = httpx.get(f"{dry_run_studio.base_url}/api/cameras/{cid}/frame", timeout=5.0)
            assert r.status_code == 200
            assert r.headers["content-type"].startswith("image/jpeg")
            assert r.content.startswith(b"\xff\xd8")
            return r.content

        with ThreadPoolExecutor(max_workers=2) as pool:
            return list(pool.map(_one, ("front", "wrist")))

    # Idle
    assert len(_fetch_both()) == 2

    # Teleop
    assert dry_run_client.post("/api/workflows/teleop/start", json={}).status_code == 200
    assert dry_run_client.post(
        "/api/workflows/teleop/joints", json={"joints": {"shoulder_pan": 22.0, "gripper": 88.0}}
    ).status_code == 200
    assert len(_fetch_both()) == 2
    assert dry_run_client.post("/api/workflows/teleop/stop").status_code == 200

    # Collect
    assert dry_run_client.post(
        "/api/workflows/collect/start",
        json={"dataset_name": "t3_08_cam_test", "single_task": "cam", "num_episodes": 1, "episode_time_s": 0.5},
    ).status_code == 200
    assert len(_fetch_both()) == 2
    assert dry_run_client.post("/api/workflows/collect/stop").status_code == 200


def test_t3_09_f9_f10_vla_configs_yaml_parity_and_playwright_stick_preview(
    workspace_root: Path,
    dry_run_client: httpx.Client,
    dry_run_page: Page,
) -> None:
    """T3-09 (F9 <-> F10): /api/vla/configs matches configs/train/*.yaml and mock_predict drives VLA stick canvas."""
    r = dry_run_client.get("/api/vla/configs")
    assert r.status_code == 200
    cfgs = r.json()["configs"]
    assert len(cfgs) >= 4
    for item in cfgs:
        p = workspace_root / item["path"]
        assert p.is_file(), f"Missing VLA config YAML on disk: {p}"
        parsed = yaml.safe_load(p.read_text(encoding="utf-8"))
        assert isinstance(parsed, dict) and parsed

    dry_run_page.locator("[data-testid='tab-vla']").click()
    dry_run_page.locator("[data-testid='btn-vla-preview']").click()
    wait_until(
        lambda: "action_chunk" in (dry_run_page.locator("#vla-preview-output").text_content() or ""),
        timeout=5.0,
        description="VLA preview output populated",
    )
    chunk = json.loads(dry_run_page.locator("#vla-preview-output").text_content() or "{}").get(
        "action_chunk", []
    )
    assert len(chunk) == 5
    for step in chunk:
        assert 0.0 <= step["gripper"] <= 100.0


def test_t3_10_f5_f7_f9_calibrate_runner_shims_and_workflow_line_parser(
    workspace_root: Path,
) -> None:
    """T3-10 (F5 <-> F7 <-> F9): calibrate_runner EOF/queue drain + WorkflowManager._on_calibrate_line table & sim sync."""
    from app.backend.calibrate_runner import install_piped_stdin_shims
    from app.backend.config import load_config
    from app.backend.hardware import HardwareManager
    from app.backend.process_manager import ProcessRegistry
    from app.backend.simulation import Arm101Simulator
    from app.backend.workflows import WorkflowManager
    import lerobot.motors.motors_bus as mb_mod

    q: queue.Queue[str | None] = queue.Queue()
    install_piped_stdin_shims(q)
    q.put(None)
    with pytest.raises(EOFError):
        mb_mod.enter_pressed()

    cfg = load_config()
    sim = Arm101Simulator()
    wf = WorkflowManager(cfg, HardwareManager(cfg), sim, ProcessRegistry())
    wf._cal_state["target"] = "follower"

    wf._on_calibrate_line("[CAL_PROMPT] Move arm to middle and press ENTER")
    assert wf._cal_state["stage"] == "waiting_middle"
    assert wf._cal_state["waiting_for_input"] is True

    wf._on_calibrate_line("Recording positions. Press ENTER to stop...")
    assert wf._cal_state["stage"] == "recording_range"

    wf._on_calibrate_line("shoulder_pan | 890 | 2047 | 3210")
    wf._on_calibrate_line("gripper | 1500 | 2800 | 2800")
    snap = wf.get_calibration_snapshot()
    assert len(snap["live_table"]) == 2
    assert math.isclose(sim.follower_joints["shoulder_pan"], 0.0, abs_tol=0.2)

    wf._on_calibrate_line("Calibration saved to data/calibration/robots/so_follower/arm101.json")
    assert wf._cal_state["stage"] == "completed"


def test_t3_11_f7_f10_package_discovery_and_web_ui_estop_with_mock_arm(
    workspace_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T3-11 (F7 <-> F10): Root pyproject software* package discovery + web_ui E-Stop latch/reset via ActionAPI."""
    data = tomllib.loads((workspace_root / "pyproject.toml").read_text(encoding="utf-8"))
    include = data.get("tool", {}).get("setuptools", {}).get("packages", {}).get("find", {}).get("include", [])
    testpaths = data.get("tool", {}).get("pytest", {}).get("ini_options", {}).get("testpaths", [])
    assert any(p.startswith("software") for p in include)
    assert "software/tests" in testpaths

    _ensure_paths(workspace_root)
    import web_ui
    from core.action_api import ActionAPI
    from core.data_types import SystemState
    from hardware.config import RobotSystemConfig
    from hardware.system_adapter import RobotSystemAdapter

    adapter = RobotSystemAdapter(RobotSystemConfig.mock_system(with_leader=False))
    api = ActionAPI(hardware=adapter)
    assert api.connect(connect_leader=False) is True
    monkeypatch.setattr(web_ui, "_GLOBAL_API", api)

    responses: list[dict[str, Any]] = []
    handler = web_ui.WebRequestHandler.__new__(web_ui.WebRequestHandler)
    handler.send_json = lambda payload, status=200: responses.append({"status": status, "body": payload})  # type: ignore[method-assign]

    try:
        handler.handle_post_estop()
        assert responses[-1]["body"]["ok"] is True
        assert api.state_machine.current_state == SystemState.ESTOP_LATCHED

        handler.handle_post_reset()
        assert responses[-1]["body"]["ok"] is True
        assert api.state_machine.current_state != SystemState.ESTOP_LATCHED
    finally:
        api.disconnect(keep_torque=False, return_to_rest=False)


def test_t3_12_f9_f10_dom_joint_sliders_match_urdf_limits_and_local_app_guide(
    workspace_root: Path,
    dry_run_page: Page,
) -> None:
    """T3-12 (F9 <-> F10): DOM slider min/max in Studio match URDF limits and docs/04_frontend_and_tools/local_app_guide.md."""
    dry_run_page.locator("[data-testid='tab-sim']").click()
    expected = {
        "shoulder_pan": (-110.0, 110.0),
        "shoulder_lift": (-100.0, 100.0),
        "elbow_flex": (-97.0, 97.0),
        "wrist_flex": (-95.0, 95.0),
        "wrist_roll": (-157.0, 163.0),
        "gripper": (0.0, 100.0),
    }
    for jname, (exp_min, exp_max) in expected.items():
        sl = dry_run_page.locator(f"#sim-follower-slider-{jname}")
        assert sl.count() == 1
        actual_min = float(sl.get_attribute("min") or 0)
        actual_max = float(sl.get_attribute("max") or 0)
        assert abs(actual_min - exp_min) <= 0.5, f"{jname} DOM min={actual_min} vs {exp_min}"
        assert abs(actual_max - exp_max) <= 0.5, f"{jname} DOM max={actual_max} vs {exp_max}"

    guide = (workspace_root / "docs" / "04_frontend_and_tools" / "local_app_guide.md").read_text(
        encoding="utf-8"
    )
    assert "[-180, 180]" not in guide
    for tok in ("110", "100", "95"):
        assert tok in guide
