"""Tier 4 E2E Tests: Real-World End-to-End Application Scenarios (T4-01..T4-06).

Covers all 6 full end-to-end user workloads from spec_matrix.md §4.3 across
HTTP, WebSocket, Playwright UI, CLI, and MockArmAdapter kernel paths.
Strictly hardware-isolated: forces STUDIO_MODE="dry_run" and mocks all serial/camera I/O.
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
import time
from collections.abc import Callable, Generator
from pathlib import Path
from typing import Any

import httpx
import pytest
import websockets.sync.client as ws_sync
from playwright.sync_api import Browser, Page

from tests.e2e._helpers import run_ubuntu_script, wait_until
from tests.e2e.conftest import DryRunSandbox, StudioServerHandle

pytestmark = pytest.mark.tier4


# ---------------------------------------------------------------------------
# Hardware-Safe Local Fixtures
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


# ---------------------------------------------------------------------------
# T4-01 .. T4-06 Real-World End-to-End Application Scenarios
# ---------------------------------------------------------------------------


def test_t4_01_new_user_onboarding_probe_calibrate_and_verify_homing(
    dry_run_studio: StudioServerHandle,
    dry_run_client: httpx.Client,
    dry_run_page: Page,
    dry_run_sandbox: DryRunSandbox,
) -> None:
    """T4-01: Cold-boot dry_run -> probe hardware -> calibrate follower (Middle + ROM) -> verify homing TCP."""
    assert "DRY-RUN" in (dry_run_page.locator("[data-testid='mode-badge']").text_content() or "")
    dry_run_page.locator("[data-testid='btn-probe-hardware']").click()
    for sel in (
        "select-follower-port",
        "select-leader-port",
        "select-front-camera",
        "select-wrist-camera",
    ):
        dry_run_page.locator(f"[data-testid='{sel}']").wait_for(state="attached", timeout=5000)

    # Start follower calibration via API/UI flow and step through Middle -> ROM -> completed
    r_start = dry_run_client.post("/api/workflows/calibrate/start", json={"target": "follower"})
    assert r_start.status_code == 200 and r_start.json()["mode"] == "dry_run"

    wait_until(
        lambda: dry_run_client.get("/api/workflows/status").json()["calibration"]["stage"] == "waiting_middle",
        timeout=5.0,
        description="calibration stage waiting_middle",
    )
    assert dry_run_client.post("/api/workflows/calibrate/input", json={"value": ""}).status_code == 200

    wait_until(
        lambda: dry_run_client.get("/api/workflows/status").json()["calibration"]["stage"] == "recording_range",
        timeout=5.0,
        description="calibration stage recording_range",
    )
    time.sleep(0.25)
    assert dry_run_client.post("/api/workflows/calibrate/input", json={"value": ""}).status_code == 200

    final_st = wait_until(
        lambda: (
            lambda st: st if st["calibration"]["stage"] == "completed" else None
        )(dry_run_client.get("/api/workflows/status").json()),
        timeout=5.0,
        description="calibration stage completed",
    )
    saved_path = Path(final_st["calibration"]["saved_path"])
    assert saved_path.is_file()
    assert dry_run_sandbox.dry_run_dir.resolve() in saved_path.resolve().parents

    js_tcp = dry_run_page.evaluate(
        """async () => {
            const mod = await import('/static/stick_sim.js');
            return mod.computeStickFK({
                shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 0,
                wrist_flex: 0, wrist_roll: 0, gripper: 20
            }).tcp;
        }"""
    )
    err_mm = math.sqrt(
        (js_tcp["x"] - 391.36147) ** 2
        + (js_tcp["y"] - (-0.00921)) ** 2
        + (js_tcp["z"] - 226.46971) ** 2
    )
    assert err_mm < 0.1, f"Homing stick FK TCP {js_tcp} error {err_mm:.2f}mm >= 0.1mm"


def test_t4_02_operator_teleop_dual_arm_sim_ik_and_estop_recovery(
    dry_run_studio: StudioServerHandle,
    dry_run_client: httpx.Client,
    dry_run_page: Page,
) -> None:
    """T4-02: WS telemetry -> unlink/link dual-arm sim -> IK + WASD -> gripper=100% -> E-Stop -> Reset -> Rest."""
    dry_run_page.locator("[data-testid='tab-teleop']").click()
    dry_run_page.locator("[data-testid='tab-sim']").click()

    with ws_sync.connect(dry_run_studio.ws_url, open_timeout=5.0) as ws:
        frame = json.loads(ws.recv(timeout=5.0))
    assert len(frame["follower_joints"]) == 6 and len(frame["leader_joints"]) == 6

    # Unlink arms and move leader independently
    r_leader = dry_run_client.post(
        "/api/sim/joints",
        json={"arm": "leader", "joints": {"shoulder_pan": 25.0}, "linked": False},
    )
    assert r_leader.status_code == 200
    assert math.isclose(r_leader.json()["leader_joints"]["shoulder_pan"], 25.0, abs_tol=0.1)
    assert math.isclose(r_leader.json()["follower_joints"]["shoulder_pan"], 0.0, abs_tol=0.1)

    # Link arms, test IK & WASD & full gripper=100.0%
    r_ik = dry_run_client.post(
        "/api/sim/ik",
        json={"arm": "follower", "x_mm": 340.0, "y_mm": -40.0, "z_mm": 215.0, "gripper": 100.0},
    )
    assert r_ik.status_code == 200
    assert math.isclose(r_ik.json()["follower_joints"]["gripper"], 100.0, abs_tol=0.1)

    for key in ("w", "a", "r", "c"):
        rk = dry_run_client.post("/api/workflows/control/key", json={"key": key, "step_mm": 6.0})
        assert rk.status_code == 200

    # Trigger E-Stop -> verify 409 block on follower motion and torque re-enable
    assert dry_run_client.post("/api/safety/estop", json={"reason": "T4-02 E-Stop"}).status_code == 200
    assert (
        dry_run_client.post(
            "/api/sim/joints", json={"arm": "follower", "joints": {"shoulder_pan": 0.0}}
        ).status_code
        == 409
    )
    assert dry_run_client.post("/api/safety/torque", json={"enabled": True}).status_code == 409

    # Reset and return to rest (verifying torque disabled after rest)
    assert dry_run_client.post("/api/safety/reset").status_code == 200
    assert (
        dry_run_client.post(
            "/api/workflows/teaching/pose", json={"pose": "rest", "duration_s": 0.25}
        ).status_code
        == 200
    )
    wait_until(
        lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
        timeout=5.0,
        description="rest preset completion after E-Stop reset",
    )
    with ws_sync.connect(dry_run_studio.ws_url, open_timeout=5.0) as ws:
        post_rest_frame = json.loads(ws.recv(timeout=5.0))
    assert post_rest_frame["safety"]["estop_latched"] is False
    assert post_rest_frame["safety"]["torque_enabled"] is False


def test_t4_03_demonstrator_teaching_custom_pose_save_and_pick_replay(
    workspace_root: Path,
    dry_run_client: httpx.Client,
    dry_run_sandbox: DryRunSandbox,
) -> None:
    """T4-03: Record 3 waypoints (0% & 100% gripper) -> save_pose dry_run -> replay & pick -> software/*.json untouched."""
    upright_p = workspace_root / "software" / "upright_pose.json"
    rest_p = workspace_root / "software" / "rest_pose.json"
    orig_u_bytes = upright_p.read_bytes()
    orig_r_bytes = rest_p.read_bytes()
    sha_u = hashlib.sha256(orig_u_bytes).hexdigest()
    sha_r = hashlib.sha256(orig_r_bytes).hexdigest()

    try:
        assert dry_run_client.post("/api/workflows/teaching/clear").status_code == 200
        poses = [
            ("wp_open", {"shoulder_pan": 0.0, "shoulder_lift": 15.0, "elbow_flex": 10.0, "wrist_flex": -20.0, "wrist_roll": 0.0, "gripper": 100.0}),
            ("wp_close", {"shoulder_pan": 20.0, "shoulder_lift": 25.0, "elbow_flex": 15.0, "wrist_flex": -30.0, "wrist_roll": 15.0, "gripper": 0.0}),
            ("wp_lift", {"shoulder_pan": -15.0, "shoulder_lift": 5.0, "elbow_flex": 5.0, "wrist_flex": -10.0, "wrist_roll": 0.0, "gripper": 50.0}),
        ]
        for label, joints in poses:
            assert dry_run_client.post("/api/sim/joints", json={"arm": "follower", "joints": joints}).status_code == 200
            rw = dry_run_client.post("/api/workflows/teaching/record_waypoint", json={"label": label})
            assert rw.status_code == 200

        assert dry_run_client.post("/api/workflows/teaching/save_pose", json={"target": "home"}).status_code == 200
        assert dry_run_client.post("/api/workflows/teaching/save_pose", json={"target": "rest"}).status_code == 200

        assert dry_run_client.post("/api/workflows/teaching/replay", json={"speed": 2.0, "loop": False}).status_code == 200
        wait_until(
            lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
            timeout=8.0,
            description="teaching waypoint replay completion",
        )
        assert dry_run_client.post("/api/workflows/teaching/routine", json={"routine": "pick"}).status_code == 200
        wait_until(
            lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
            timeout=8.0,
            description="pick demo routine completion",
        )

        assert (dry_run_sandbox.dry_run_dir / "waypoints.json").is_file()
        assert hashlib.sha256(upright_p.read_bytes()).hexdigest() == sha_u
        assert hashlib.sha256(rest_p.read_bytes()).hexdigest() == sha_r
    finally:
        if upright_p.read_bytes() != orig_u_bytes:
            upright_p.write_bytes(orig_u_bytes)
        if rest_p.read_bytes() != orig_r_bytes:
            rest_p.write_bytes(orig_r_bytes)


def test_t4_04_dataset_collector_multi_session_resume_and_ui_replay(
    workspace_root: Path,
    tmp_path: Path,
    dry_run_client: httpx.Client,
    dry_run_page: Page,
) -> None:
    """T4-04: Collect 2 episodes -> inspect 6-DOF frames -> verify build_record_cmd --resume & UI spec card."""
    r_start = dry_run_client.post(
        "/api/workflows/collect/start",
        json={
            "repo_id": "tust/e2e_pick_cube",
            "single_task": "Pick up cube",
            "num_episodes": 2,
            "episode_time_s": 0.4,
            "reset_time_s": 0.05,
            "fps": 30,
        },
    )
    assert r_start.status_code == 200

    wait_until(
        lambda: dry_run_client.get("/api/workflows/status").json()["active_workflow"] == "idle",
        timeout=10.0,
        description="2-episode collection completion",
    )

    eps = dry_run_client.get("/api/workflows/collect/episodes").json()["episodes"]
    matching = [e for e in eps if e["dataset"] == "e2e_pick_cube"]
    assert len(matching) >= 2

    ep0 = dry_run_client.get("/api/workflows/collect/episodes/e2e_pick_cube/0")
    assert ep0.status_code == 200
    frames = ep0.json()["frames"]
    assert len(frames) >= 1
    for fr in frames:
        assert len(fr["observation.state"]) == 6 and len(fr["action"]) == 6
        assert 0.0 <= fr["observation.state"][5] <= 100.0

    # Verify eval_* rejection and build_record_cmd --resume=true + streaming_encoding=true
    assert (
        dry_run_client.post(
            "/api/workflows/collect/start", json={"dataset_name": "eval_bad", "single_task": "x"}
        ).status_code
        == 422
    )

    from app.backend.config import load_config
    from app.backend.hardware import HardwareManager
    from app.backend.process_manager import ProcessRegistry
    from app.backend.simulation import Arm101Simulator
    from app.backend.workflows import WorkflowManager

    cfg = load_config()
    cfg.datasets_dir = tmp_path / "datasets"
    (cfg.datasets_dir / "e2e_pick_cube").mkdir(parents=True, exist_ok=True)
    wf = WorkflowManager(cfg, HardwareManager(cfg), Arm101Simulator(), ProcessRegistry())
    cmd = " ".join(
        wf.build_record_cmd(
            dataset_name="e2e_pick_cube",
            single_task="Pick up cube",
            num_episodes=2,
            episode_time_s=5.0,
            reset_time_s=1.0,
            fps=30,
        )
    )
    assert "--resume=true" in cmd and "--dataset.streaming_encoding=true" in cmd

    # Verify Playwright Collect tab & stick replay
    dry_run_page.locator("[data-testid='tab-collect']").click()
    spec_text = dry_run_page.locator("[data-testid='data-spec-card']").text_content() or ""
    assert "v3.0" in spec_text
    dry_run_page.locator("#btn-refresh-episodes").click()
    wait_until(
        lambda: dry_run_page.locator(".btn-ep-stick-replay[data-dataset='e2e_pick_cube']").count() >= 1,
        timeout=5.0,
        description="episode replay buttons in Collect tab",
    )
    dry_run_page.locator(".btn-ep-stick-replay[data-dataset='e2e_pick_cube']").first.click()
    dry_run_page.locator("[data-testid='canvas-collect-stick']").wait_for(state="visible", timeout=5000)


def test_t4_05_cloud_vla_prep_plugin_hub_and_mock_action_chunk_preview(
    workspace_root: Path,
    dry_run_client: httpx.Client,
    dry_run_page: Page,
) -> None:
    """T4-05: setup_ubuntu.sh --with-pi/--with-smolvla + Plugins tab lifecycle + VLA 5-step chunk preview."""
    sh_res = run_ubuntu_script(
        workspace_root / "scripts" / "ubuntu" / "setup_ubuntu.sh",
        ["--dry-run", "--with-pi", "--with-smolvla"],
    )
    if sh_res is not None:
        assert sh_res.returncode == 0
        assert "pi" in sh_res.stdout and "smolvla" in sh_res.stdout

    # Playwright Plugins tab: all 6 adapters listed (including open_source_datasets); feetech_servo_bench launches w/ --simulate & stops; remote_vla 409
    dry_run_page.locator("[data-testid='tab-plugins']").click()
    wait_until(
        lambda: dry_run_page.locator("[data-plugin-id]").count() >= 6,
        timeout=5.0,
        description="all 6 plugin cards rendered",
    )
    launch_fb = dry_run_client.post("/api/plugins/feetech_servo_bench/launch", json={})
    assert launch_fb.status_code == 200
    assert "--simulate" in " ".join(launch_fb.json().get("command", []))
    assert dry_run_client.post("/api/plugins/feetech_servo_bench/stop").status_code == 200

    vla_launch = dry_run_client.post("/api/plugins/remote_vla_policy/launch", json={})
    assert vla_launch.status_code == 409 and vla_launch.json()["code"] == "OS_UNSUPPORTED"

    # Playwright VLA tab: configs + pi07_notice + mock_predict 5-step chunk
    dry_run_page.locator("[data-testid='tab-vla']").click()
    dry_run_page.locator("#pi07-notice-card").wait_for(state="visible", timeout=5000)
    assert dry_run_page.locator("#vla-configs-grid .card").count() >= 4
    dry_run_page.locator("[data-testid='btn-vla-preview']").click()
    wait_until(
        lambda: "action_chunk" in (dry_run_page.locator("#vla-preview-output").text_content() or ""),
        timeout=5.0,
        description="VLA mock_predict output rendered",
    )
    chunk = json.loads(dry_run_page.locator("#vla-preview-output").text_content() or "{}")["action_chunk"]
    assert len(chunk) == 5
    for step in chunk:
        assert 0.0 <= step["gripper"] <= 100.0


def test_t4_06_mock_hardware_kernel_full_lifecycle_via_realtime_bridge(
    workspace_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """T4-06: Full MockArmAdapter + ActionAPI + SO101RealBridge lifecycle (real_to_sim -> sim_to_real -> estop -> reset -> park -> disconnect)."""
    _ensure_paths(workspace_root)
    from app.backend.config import load_config
    from app.backend.realtime_bridge import SO101RealBridge
    from app.backend.simulation import Arm101Simulator
    from core.data_types import SystemState
    from hardware.config import ArmConfig, RobotSystemConfig
    from hardware.mock_arm import MockArmAdapter
    from hardware.system_adapter import RobotSystemAdapter

    cfg = load_config()
    cfg.calibration_dir = tmp_path / "cal"
    (cfg.calibration_dir / "robots" / "so_follower").mkdir(parents=True, exist_ok=True)
    (cfg.calibration_dir / "teleoperators" / "so_leader").mkdir(parents=True, exist_ok=True)
    cfg.follower_port = "MOCK_COM11"
    cfg.leader_port = "MOCK_COM12"

    def _mock_adapter(sys_cfg: RobotSystemConfig) -> RobotSystemAdapter:
        mf = ArmConfig.mock_follower(robot_id=sys_cfg.follower.robot_id)
        ml = ArmConfig.mock_leader(robot_id=sys_cfg.leader.robot_id) if sys_cfg.leader else None
        ad = RobotSystemAdapter(RobotSystemConfig(follower=mf, leader=ml, use_mock=True))
        if isinstance(ad.follower, MockArmAdapter):
            ad.follower._current_positions["gripper"] = 80.0
            ad.follower._target_positions["gripper"] = 80.0
        if isinstance(ad.leader, MockArmAdapter):
            ad.leader._current_positions["shoulder_pan"] = math.radians(18.0)
            ad.leader._current_positions["gripper"] = 80.0
        return ad

    monkeypatch.setattr("hardware.system_adapter.RobotSystemAdapter", _mock_adapter)
    monkeypatch.setattr("core.action_api.RobotSystemAdapter", _mock_adapter, raising=False)
    monkeypatch.setattr("teleop.official_wrapper.safe_park_to_rest", lambda **_kw: True)

    sim = Arm101Simulator()
    bridge = SO101RealBridge(cfg, sim)
    try:
        # 1. real_to_sim: torque disabled, gripper=80.0 stays 80.0%
        bridge.set_sync_mode("real_to_sim", is_real_mode=True)
        assert sim.torque_enabled is False
        wait_until(
            lambda: math.isclose(sim.leader_joints["shoulder_pan"], 18.0, abs_tol=1.0),
            timeout=3.0,
            description="leader drag synced to simulator",
        )
        assert math.isclose(sim.follower_joints["gripper"], 80.0, abs_tol=0.5)

        # 2. sim_to_real: send gripper=100%
        bridge.set_sync_mode("sim_to_real", is_real_mode=True, safety_confirmed=True)
        assert bridge.send_follower_target_deg(
            {
                "shoulder_pan": 5.0,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0,
                "gripper": 100.0,
            }
        ) is True

        # 3. E-Stop -> Reset -> safe_park_to_rest -> disconnect
        bridge.trigger_estop("T4-06 lifecycle estop")
        assert bridge._api.state_machine.current_state == SystemState.ESTOP_LATCHED
        bridge.reset_estop()
        assert bridge._api.state_machine.current_state != SystemState.ESTOP_LATCHED

        assert bridge.safe_park_to_rest(duration_s=0.5) is True
        t0 = time.perf_counter()
        bridge.disconnect()
        assert time.perf_counter() - t0 < 0.5
        assert bridge.status()["direction"] == "off"
    finally:
        bridge.disconnect()
