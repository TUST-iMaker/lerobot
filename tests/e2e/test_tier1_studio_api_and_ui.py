"""Tier 1 E2E Tests: Local Arm101 Studio Backend, Workflows, Plugins & UI (F16–F19).

Total: 20 tests (5 per feature). Strictly opaque-box via HTTP, WebSocket, CLI, and Playwright DOM.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import httpx
import pytest
import websockets.sync.client as ws_sync
from playwright.sync_api import Page

from tests.e2e._helpers import (
    REQUIRED_STUDIO_TESTIDS,
    contains_chinese,
    run_cmd,
    wait_until,
)
from tests.e2e.conftest import DryRunSandbox, StudioServerHandle

pytestmark = pytest.mark.tier1


# ---------------------------------------------------------------------------
# Feature 16: R4 Local Backend & Graceful Degradation — 5 tests
# ---------------------------------------------------------------------------


def test_f16_health_endpoint_contract(http_client: httpx.Client) -> None:
    """T1-F16-01: GET /api/health returns 200 with required metadata and operating mode."""
    resp = http_client.get("/api/health")
    assert resp.status_code == 200
    data = resp.json()
    assert data["status"] == "ok"
    assert data["os"]
    assert data["python_version"].startswith("3.")
    assert data["mode"] in ("real", "dry_run")
    assert data["workspace_root"]


def test_f16_hardware_probe_contract(http_client: httpx.Client) -> None:
    """T1-F16-02: GET /api/hardware/probe returns structured port, camera, and guidance info."""
    resp = http_client.get("/api/hardware/probe")
    assert resp.status_code == 200
    data = resp.json()
    assert isinstance(data["serial_ports"], list)
    assert isinstance(data["cameras"], list)
    assert isinstance(data["lerobot_available"], bool)
    assert isinstance(data["os_capabilities"], dict)
    assert isinstance(data["missing_hardware"], list)
    assert isinstance(data["guidance"], list)


def test_f16_mode_switch_dry_run_contract(http_client: httpx.Client) -> None:
    """T1-F16-03: POST /api/mode sets dry_run cleanly and reflects in GET /api/health."""
    resp = http_client.post("/api/mode", json={"mode": "dry_run"})
    assert resp.status_code == 200
    body = resp.json()
    assert body.get("mode") == "dry_run"

    health = http_client.get("/api/health").json()
    assert health["mode"] == "dry_run"


def test_f16_camera_frame_returns_valid_jpeg(http_client: httpx.Client) -> None:
    """T1-F16-04: GET /api/cameras/0/frame returns a valid JPEG image in dry_run mode."""
    http_client.post("/api/mode", json={"mode": "dry_run"})
    resp = http_client.get("/api/cameras/0/frame")
    assert resp.status_code == 200
    assert resp.headers["content-type"].startswith("image/jpeg")
    assert resp.content.startswith(b"\xff\xd8")
    assert len(resp.content) > 100


def test_f16_ws_telemetry_streams_6dof_state(
    studio_server: StudioServerHandle,
    http_client: httpx.Client,
) -> None:
    """T1-F16-05: WS /ws/telemetry connects and streams JSON 6-DOF leader/follower telemetry."""
    http_client.post("/api/mode", json={"mode": "dry_run"})
    with ws_sync.connect(studio_server.ws_url, open_timeout=5.0) as ws:
        raw = ws.recv(timeout=5.0)
    frame = json.loads(raw)
    assert isinstance(frame, dict)
    # Check joint angles (6 DOF) and workflow/mode state in telemetry payload
    joined_keys = " ".join(frame.keys()).lower()
    assert any(k in joined_keys for k in ("leader", "follower", "joints", "state"))
    # Find any 6-element joint container in the frame
    has_6dof = False
    for val in frame.values():
        if isinstance(val, (list, dict)) and len(val) >= 6:
            has_6dof = True
            break
    assert has_6dof, f"Expected 6-DOF joint payload in WS telemetry frame: {frame}"


# ---------------------------------------------------------------------------
# Feature 17: R4 Core Workflow Endpoints (Calibrate / Teleop / Teach / Collect) — 5 tests
# ---------------------------------------------------------------------------


def test_f17_calibrate_start_and_stop(http_client: httpx.Client) -> None:
    """T1-F17-01: Calibration workflow starts, reports status, and stops cleanly."""
    http_client.post("/api/mode", json={"mode": "dry_run"})
    start_resp = http_client.post("/api/workflows/calibrate/start", json={"role": "follower"})
    assert start_resp.status_code == 200

    status_resp = http_client.get("/api/workflows/status")
    assert status_resp.status_code == 200
    status_data = status_resp.json()
    assert isinstance(status_data, dict)

    stop_resp = http_client.post("/api/workflows/calibrate/stop")
    assert stop_resp.status_code == 200


def test_f17_teleop_start_joints_and_stop(http_client: httpx.Client) -> None:
    """T1-F17-02: Teleop starts, accepts 6-DOF virtual leader joints, and stops cleanly."""
    http_client.post("/api/mode", json={"mode": "dry_run"})
    assert http_client.post("/api/workflows/teleop/start", json={}).status_code == 200

    joints_payload = {
        "joints": {
            "shoulder_pan": 12.5,
            "shoulder_lift": -15.0,
            "elbow_flex": 30.0,
            "wrist_flex": 10.0,
            "wrist_roll": -5.0,
            "gripper": 25.0,
        }
    }
    upd = http_client.post("/api/workflows/teleop/joints", json=joints_payload)
    assert upd.status_code == 200

    stop = http_client.post("/api/workflows/teleop/stop")
    assert stop.status_code == 200


def test_f17_teaching_record_and_list_waypoints(
    http_client: httpx.Client,
    dry_run_sandbox: DryRunSandbox,
) -> None:
    """T1-F17-03: Teaching clear + record_waypoint persists waypoint visible via GET."""
    _ = dry_run_sandbox
    http_client.post("/api/mode", json={"mode": "dry_run"})
    assert http_client.post("/api/workflows/teaching/clear").status_code == 200

    rec = http_client.post(
        "/api/workflows/teaching/record_waypoint",
        json={"label": "wp_home", "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    )
    assert rec.status_code == 200

    listed = http_client.get("/api/workflows/teaching/waypoints")
    assert listed.status_code == 200
    body = listed.json()
    waypoints = body if isinstance(body, list) else body.get("waypoints", [])
    assert len(waypoints) >= 1


def test_f17_teaching_replay_and_clear(
    http_client: httpx.Client,
    dry_run_sandbox: DryRunSandbox,
) -> None:
    """T1-F17-04: Teaching replay executes recorded waypoints and clear resets the list."""
    _ = dry_run_sandbox
    http_client.post("/api/mode", json={"mode": "dry_run"})
    http_client.post("/api/workflows/teaching/clear")
    for idx, val in enumerate((5.0, 15.0)):
        r = http_client.post(
            "/api/workflows/teaching/record_waypoint",
            json={"label": f"wp_{idx}", "joints": [val, 0.0, 0.0, 0.0, 0.0, 0.0]},
        )
        assert r.status_code == 200

    replay = http_client.post("/api/workflows/teaching/replay", json={})
    assert replay.status_code == 200

    wait_until(
        lambda: http_client.get("/api/workflows/status").status_code == 200,
        timeout=10.0,
        description="workflows status after replay",
    )
    cleared = http_client.post("/api/workflows/teaching/clear")
    assert cleared.status_code == 200
    body = http_client.get("/api/workflows/teaching/waypoints").json()
    waypoints = body if isinstance(body, list) else body.get("waypoints", [])
    assert len(waypoints) == 0


def test_f17_collect_start_stop_and_episodes(
    http_client: httpx.Client,
    dry_run_sandbox: DryRunSandbox,
) -> None:
    """T1-F17-05: Data collection start/stop records episode metadata and dry_run artifact."""
    http_client.post("/api/mode", json={"mode": "dry_run"})
    start = http_client.post(
        "/api/workflows/collect/start",
        json={"repo_id": "tust/dry_run_test", "task": "pick_cube"},
    )
    assert start.status_code == 200

    wait_until(
        lambda: http_client.get("/api/workflows/status").json() is not None,
        timeout=5.0,
        description="collect workflow active",
    )
    stop = http_client.post("/api/workflows/collect/stop")
    assert stop.status_code == 200

    eps_resp = http_client.get("/api/workflows/collect/episodes")
    assert eps_resp.status_code == 200
    eps_body = eps_resp.json()
    episodes = eps_body if isinstance(eps_body, list) else eps_body.get("episodes", [])
    assert len(episodes) >= 1
    assert dry_run_sandbox.list_new_files(), "Expected collection artifact under data/dry_run/"


# ---------------------------------------------------------------------------
# Feature 18: R4 Open-Source Tool Plugin Hub — 5 tests
# ---------------------------------------------------------------------------


def test_f18_plugins_list_returns_core_adapters(http_client: httpx.Client) -> None:
    """T1-F18-01: GET /api/plugins lists core open-source tool adapters."""
    resp = http_client.get("/api/plugins")
    assert resp.status_code == 200
    body = resp.json()
    plugins = body if isinstance(body, list) else body.get("plugins", [])
    assert len(plugins) >= 4
    joined = json.dumps(plugins, ensure_ascii=False).lower()
    for expected in ("rerun", "feetech", "lerobot", "lelab"):
        assert expected in joined, f"Expected plugin adapter keyword {expected!r} in {joined}"


def test_f18_plugins_expose_os_capability_metadata(http_client: httpx.Client) -> None:
    """T1-F18-02: Each plugin item exposes identifier and OS/availability status metadata."""
    body = http_client.get("/api/plugins").json()
    plugins = body if isinstance(body, list) else body.get("plugins", [])
    assert plugins
    for item in plugins:
        assert "id" in item or "plugin_id" in item
        assert "name" in item
        assert any(k in item for k in ("available", "status", "os_support", "supported"))


def test_f18_plugin_launch_and_stop_lifecycle(http_client: httpx.Client) -> None:
    """T1-F18-03: Launching and stopping an available plugin succeeds cleanly."""
    body = http_client.get("/api/plugins").json()
    plugins = body if isinstance(body, list) else body.get("plugins", [])
    available = [
        p
        for p in plugins
        if p.get("available", True) and p.get("supported", True) and "openpi" not in str(p).lower()
    ]
    assert available, f"No launchable plugin found in {plugins}"
    plugin_id = available[0].get("id") or available[0].get("plugin_id")

    launch = http_client.post(f"/api/plugins/{plugin_id}/launch", json={})
    assert launch.status_code == 200

    stop = http_client.post(f"/api/plugins/{plugin_id}/stop")
    assert stop.status_code == 200


def test_f18_remote_vla_plugin_surfaces_status_safely(http_client: httpx.Client) -> None:
    """T1-F18-04: OpenPI / remote VLA adapter surfaces Ubuntu/remote status without crashing."""
    body = http_client.get("/api/plugins").json()
    plugins = body if isinstance(body, list) else body.get("plugins", [])
    vla_plugins = [
        p
        for p in plugins
        if any(kw in json.dumps(p).lower() for kw in ("openpi", "vla", "policy"))
    ]
    assert vla_plugins, f"Expected remote VLA / openpi adapter in {plugins}"


def test_f18_run_studio_cli_help_and_module_import(workspace_root: Path) -> None:
    """T1-F18-05: app/run_studio.py --help exits 0 and documents required CLI flags."""
    run_studio = workspace_root / "app" / "run_studio.py"
    assert run_studio.is_file(), "Missing app/run_studio.py"
    res = run_cmd([sys.executable, str(run_studio), "--help"], cwd=workspace_root)
    assert res.returncode == 0, f"run_studio.py --help failed: {res.stderr}"
    for flag in ("--host", "--port", "--dry-run", "--no-browser"):
        assert flag in res.stdout, f"run_studio.py --help missing {flag}"


# ---------------------------------------------------------------------------
# Feature 19: R4 Zero-Build Web Frontend & App Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f19_root_serves_zero_build_html_esm(
    workspace_root: Path,
    http_client: httpx.Client,
) -> None:
    """T1-F19-01: GET / serves zero-build HTML5/CSS3/ESM frontend files."""
    for rel in ("index.html", "styles.css", "app.js"):
        assert (workspace_root / "app" / "frontend" / rel).is_file(), f"Missing app/frontend/{rel}"
    resp = http_client.get("/")
    assert resp.status_code == 200
    assert "text/html" in resp.headers.get("content-type", "")
    assert "app.js" in resp.text


def test_f19_playwright_renders_all_required_tabs_and_badges(browser_page: Page) -> None:
    """T1-F19-02: Playwright loads / and renders required status badges, tabs, and containers."""
    for testid in (
        "hardware-status-banner",
        "mode-badge",
        "tab-dashboard",
        "tab-teleop",
        "tab-teaching",
        "tab-collect",
        "tab-plugins",
        "tab-vla",
        "plugin-list",
        "log-console",
    ):
        loc = browser_page.locator(f"[data-testid='{testid}']")
        loc.wait_for(state="attached", timeout=5000)


def test_f19_playwright_all_required_action_buttons_present(browser_page: Page) -> None:
    """T1-F19-03: All required workflow action buttons exist with exact data-testid hooks."""
    for testid in REQUIRED_STUDIO_TESTIDS:
        assert browser_page.locator(f"[data-testid='{testid}']").count() >= 1, (
            f"Missing required UI hook: data-testid={testid!r}"
        )


def test_f19_playwright_zero_console_and_page_errors(browser_page: Page) -> None:
    """T1-F19-04: Clicking through all 6 tabs produces zero console errors or pageerrors."""
    for tab in (
        "tab-teleop",
        "tab-teaching",
        "tab-collect",
        "tab-plugins",
        "tab-vla",
        "tab-dashboard",
    ):
        browser_page.locator(f"[data-testid='{tab}']").click()
    # fixture teardown asserts zero console_errors and zero page_errors


def test_f19_local_app_guide_matches_cli_and_endpoints(workspace_root: Path) -> None:
    """T1-F19-05: docs/04_frontend_and_tools/local_app_guide.md matches Studio CLI & API."""
    guide = workspace_root / "docs" / "04_frontend_and_tools" / "local_app_guide.md"
    assert guide.is_file(), "Missing docs/04_frontend_and_tools/local_app_guide.md"
    text = guide.read_text(encoding="utf-8")
    assert contains_chinese(text)
    for tok in ("app.backend.main", "run_studio.py", "/api/health", "/ws/telemetry", "dry_run"):
        assert tok in text, f"local_app_guide.md missing Studio doc token: {tok}"
