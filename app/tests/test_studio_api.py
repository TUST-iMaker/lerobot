"""Comprehensive API, WebSocket, ProcessRegistry, and Playwright UI tests for Arm101 Studio."""

from __future__ import annotations

import ctypes
import io
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import httpx
import pytest
import uvicorn
import websockets
from PIL import Image
from playwright.sync_api import sync_playwright

from app.backend.config import StudioConfig, load_config
from app.backend.hardware import _classify_port
from app.backend.main import create_app
from app.backend.process_manager import ProcessRegistry


REQUIRED_TESTIDS = (
    "hardware-status-banner",
    "mode-badge",
    "tab-dashboard",
    "tab-teleop",
    "tab-teaching",
    "tab-collect",
    "tab-plugins",
    "tab-vla",
    "btn-probe-hardware",
    "btn-start-teleop",
    "btn-stop-teleop",
    "btn-teach-waypoint",
    "btn-replay-waypoints",
    "btn-start-collect",
    "btn-stop-collect",
    "plugin-list",
    "log-console",
)


def _pick_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    if sys.platform == "win32":
        process_query_limited = 0x1000
        still_active = 259
        handle = ctypes.windll.kernel32.OpenProcess(process_query_limited, False, pid)
        if not handle:
            return False
        try:
            exit_code = ctypes.c_ulong()
            ok = ctypes.windll.kernel32.GetExitCodeProcess(handle, ctypes.byref(exit_code))
            if not ok:
                return False
            return exit_code.value == still_active
        finally:
            ctypes.windll.kernel32.CloseHandle(handle)
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _launch_chromium(playwright: Any):
    """Launch headless Chromium with offline fallback to cached ms-playwright / Edge binaries."""
    try:
        return playwright.chromium.launch(headless=True)
    except Exception:
        pass

    cache_roots = []
    if sys.platform == "win32":
        local_app_data = os.environ.get("LOCALAPPDATA", "")
        if local_app_data:
            cache_roots.append(Path(local_app_data) / "ms-playwright")
    else:
        cache_roots.append(Path.home() / ".cache" / "ms-playwright")

    patterns = (
        ("**/chrome-headless-shell.exe", "**/chrome.exe")
        if sys.platform == "win32"
        else ("**/chrome-headless-shell", "**/chrome")
    )
    for root in cache_roots:
        if not root.exists():
            continue
        for pat in patterns:
            matches = sorted(root.glob(pat), reverse=True)
            for exe in matches:
                if exe.is_file():
                    return playwright.chromium.launch(headless=True, executable_path=str(exe))

    for channel in ("msedge", "chrome"):
        try:
            return playwright.chromium.launch(headless=True, channel=channel)
        except Exception:
            continue

    raise RuntimeError(
        "No Playwright Chromium/Edge binary available offline. "
        "Run scripts\\windows\\setup_win.ps1 -WithPlaywrightBrowser."
    )


def _isolated_config(tmp_path: Path) -> StudioConfig:
    base = load_config()
    dry = tmp_path / "data" / "dry_run"
    return base.model_copy(
        update={
            "env_file": tmp_path / ".env",
            "data_dir": tmp_path / "data",
            "datasets_dir": tmp_path / "data" / "datasets",
            "calibration_dir": tmp_path / "data" / "calibration",
            "dry_run_dir": dry,
            "logs_dir": tmp_path / "logs",
            "requested_mode": "auto",
        }
    )


@pytest.fixture()
def live_server(tmp_path: Path) -> Iterator[tuple[str, StudioConfig]]:
    cfg = _isolated_config(tmp_path)
    app = create_app(config=cfg)
    port = _pick_free_port()
    uv_cfg = uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning")
    server = uvicorn.Server(uv_cfg)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()

    base_url = f"http://127.0.0.1:{port}"
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        try:
            r = httpx.get(f"{base_url}/api/health", timeout=0.5)
            if r.status_code == 200:
                break
        except Exception:
            pass
        time.sleep(0.08)
    else:
        server.should_exit = True
        thread.join(timeout=2.0)
        pytest.fail("Live Studio server did not become healthy within 10s.")

    try:
        yield base_url, cfg
    finally:
        server.should_exit = True
        thread.join(timeout=5.0)


# ----------------------------------------------------------------------
# Group 1: Hardware Probing, Mode Switching & Offline Assets
# ----------------------------------------------------------------------
def test_bluetooth_ports_filtered_from_robot_candidates() -> None:
    bt_win = _classify_port(
        device="COM8",
        description="蓝牙链接上的标准串行 (COM8)",
        hwid="BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_VID&0001",
        vid_int=None,
        pid_int=None,
    )
    assert bt_win["is_bluetooth"] is True
    assert bt_win["is_robot_candidate"] is False

    ch340 = _classify_port(
        device="COM3",
        description="USB-SERIAL CH340 (COM3)",
        hwid="USB VID:PID=1A86:7523 SER=1234",
        vid_int=0x1A86,
        pid_int=0x7523,
    )
    assert ch340["is_bluetooth"] is False
    assert ch340["is_robot_candidate"] is True
    assert "CH340" in ch340["tag"]


def test_health_mode_probe_and_offline_assets(live_server: tuple[str, StudioConfig]) -> None:
    base_url, _ = live_server
    with httpx.Client(base_url=base_url, timeout=5.0) as client:
        health = client.get("/api/health")
        assert health.status_code == 200
        hdata = health.json()
        assert hdata["status"] == "ok"
        assert hdata["mode"] in ("real", "dry_run")
        assert "workspace_root" in hdata

        probe = client.get("/api/hardware/probe")
        assert probe.status_code == 200
        pdata = probe.json()
        for key in (
            "serial_ports",
            "cameras",
            "lerobot_available",
            "os_capabilities",
            "missing_hardware",
            "guidance",
        ):
            assert key in pdata

        # Mode validation & clean real-without-hardware handling
        assert client.post("/api/mode", json={"mode": "invalid"}).status_code == 422
        assert client.post("/api/mode", json={"mode": "dry_run"}).status_code == 200
        assert client.post("/api/mode", json={"mode": "auto"}).status_code == 200

        real_res = client.post("/api/mode", json={"mode": "real"})
        if not pdata["hardware_ready"]:
            assert real_res.status_code == 409
            rdata = real_res.json()
            assert rdata["code"] == "HARDWARE_NOT_READY"
            assert rdata["mode"] == "dry_run"

        # Offline static assets & all 17 required data-testid hooks
        root_html = client.get("/")
        assert root_html.status_code == 200
        html_text = root_html.text
        assert not re.search(r"<(script|link)[^>]+https?://", html_text, re.IGNORECASE)
        for testid in REQUIRED_TESTIDS:
            assert f'data-testid="{testid}"' in html_text

        css_res = client.get("/static/styles.css")
        assert css_res.status_code == 200
        assert "@import" not in css_res.text

        js_res = client.get("/static/app.js")
        assert js_res.status_code == 200


# ----------------------------------------------------------------------
# Group 2: Core Workflows, Validation & Dry-Run Disk Artifacts
# ----------------------------------------------------------------------
def test_calibrate_teleop_teaching_collect_and_cameras(
    live_server: tuple[str, StudioConfig],
) -> None:
    base_url, cfg = live_server
    with httpx.Client(base_url=base_url, timeout=10.0) as client:
        assert client.post("/api/mode", json={"mode": "dry_run"}).status_code == 200
        # Calibrate lifecycle + command isolation check (never both robot & teleop)
        cal_start = client.post("/api/workflows/calibrate/start", json={"target": "follower"})
        assert cal_start.status_code == 200
        cmd_str = " ".join(cal_start.json()["command"])
        assert "--robot.type=so101_follower" in cmd_str
        assert "--teleop." not in cmd_str
        assert client.post("/api/workflows/calibrate/stop").status_code == 200

        cal_leader = client.post("/api/workflows/calibrate/start", json={"target": "leader"})
        assert cal_leader.status_code == 200
        leader_cmd_str = " ".join(cal_leader.json()["command"])
        assert "--teleop.type=so101_leader" in leader_cmd_str
        assert "--robot." not in leader_cmd_str
        assert client.post("/api/workflows/calibrate/stop").status_code == 200

        # Teleop lifecycle & strict 6-DOF validation
        assert client.post("/api/workflows/teleop/start", json={"fps": 60}).status_code == 200
        valid_j = client.post(
            "/api/workflows/teleop/joints",
            json={"joints": {"shoulder_pan": 22.5, "gripper": 75.0}},
        )
        assert valid_j.status_code == 200
        assert valid_j.json()["leader_joints"]["shoulder_pan"] == 22.5

        assert client.post("/api/workflows/teleop/joints", json={"joints": {}}).status_code == 422
        assert (
            client.post("/api/workflows/teleop/joints", json={"joints": {"unknown_joint": 1.0}})
            .status_code
            == 422
        )
        assert (
            client.post("/api/workflows/teleop/joints", json={"joints": {"shoulder_pan": 999.0}})
            .status_code
            == 422
        )
        assert (
            client.post("/api/workflows/teleop/joints", json={"joints": {"gripper": 105.0}})
            .status_code
            == 422
        )
        assert (
            client.post("/api/workflows/teleop/joints", json={"joints": {"gripper": True}})
            .status_code
            == 422
        )
        assert client.post("/api/workflows/teleop/stop").status_code == 200

        # Teaching waypoints: empty replay -> 409; record -> list -> replay -> clear
        assert client.post("/api/workflows/teaching/clear").status_code == 200
        assert client.post("/api/workflows/teaching/replay").status_code == 409

        r1 = client.post("/api/workflows/teaching/record_waypoint", json={"label": "wp_alpha"})
        r2 = client.post(
            "/api/workflows/teaching/record_waypoint",
            json={"label": "wp_beta", "joints": {"shoulder_pan": -30.0, "gripper": 15.0}},
        )
        assert r1.status_code == 200 and r2.status_code == 200
        wps = client.get("/api/workflows/teaching/waypoints").json()
        assert wps["count"] == 2
        assert (cfg.dry_run_dir / "waypoints.json").exists()

        rep = client.post("/api/workflows/teaching/replay", json={"speed": 4.0, "loop": False})
        assert rep.status_code == 200
        time.sleep(0.5)
        assert client.post("/api/workflows/teaching/clear").status_code == 200
        assert client.get("/api/workflows/teaching/waypoints").json()["count"] == 0

        # Collect: reject eval_ prefix; start -> run briefly -> stop -> episodes artifact check
        bad_ds = client.post(
            "/api/workflows/collect/start",
            json={"dataset_name": "eval_forbidden", "single_task": "test"},
        )
        assert bad_ds.status_code == 422

        col_start = client.post(
            "/api/workflows/collect/start",
            json={
                "dataset_name": "studio_smoke_ds",
                "single_task": "Pick cube",
                "num_episodes": 1,
                "episode_time_s": 1.0,
                "fps": 30,
            },
        )
        assert col_start.status_code == 200
        time.sleep(0.35)
        col_stop = client.post("/api/workflows/collect/stop")
        assert col_stop.status_code == 200
        eps = client.get("/api/workflows/collect/episodes").json()
        assert eps["count"] >= 1
        assert (cfg.dry_run_dir / "datasets" / "studio_smoke_ds" / "meta" / "info.json").exists()

        # Camera JPEG frames
        for cam_id in ("front", "wrist"):
            cam_res = client.get(f"/api/cameras/{cam_id}/frame")
            assert cam_res.status_code == 200
            assert cam_res.headers["content-type"].startswith("image/jpeg")
            assert cam_res.content[:2] == b"\xff\xd8"
            img = Image.open(io.BytesIO(cam_res.content))
            assert img.size == (640, 480)

        assert client.get("/api/cameras/nonexistent_cam/frame").status_code == 404


def test_real_data_dry_run_directory_artifact_and_cleanup() -> None:
    """Verify default workspace data/dry_run/ directory works cleanly and leaves no debris."""
    app = create_app()
    cfg = load_config()
    test_ds = "pytest_cleanup_check"
    ds_path = cfg.dry_run_dir / "datasets" / test_ds
    wp_path = cfg.dry_run_dir / "waypoints.json"
    cal_dir = cfg.dry_run_dir / "calibration"
    prev_wp = wp_path.read_text(encoding="utf-8") if wp_path.exists() else None
    try:
        from fastapi.testclient import TestClient

        with TestClient(app) as client:
            assert client.post("/api/mode", json={"mode": "dry_run"}).status_code == 200
            res = client.post(
                "/api/workflows/collect/start",
                json={"dataset_name": test_ds, "single_task": "Cleanup check", "num_episodes": 1},
            )
            assert res.status_code == 200
            time.sleep(0.25)
            assert client.post("/api/workflows/collect/stop").status_code == 200
            assert (ds_path / "meta" / "info.json").exists()
    finally:
        if ds_path.exists():
            shutil.rmtree(ds_path, ignore_errors=True)
        if prev_wp is None and wp_path.exists():
            wp_path.unlink()
        elif prev_wp is not None:
            wp_path.write_text(prev_wp, encoding="utf-8")
        if cal_dir.exists() and not any(cal_dir.iterdir()):
            cal_dir.rmdir()


# ----------------------------------------------------------------------
# Group 3: Plugins, VLA Configs & ProcessRegistry Zero-Orphan Cleanup
# ----------------------------------------------------------------------
def test_plugins_and_vla_endpoints(live_server: tuple[str, StudioConfig]) -> None:
    base_url, _ = live_server
    with httpx.Client(base_url=base_url, timeout=5.0) as client:
        plugins_res = client.get("/api/plugins")
        assert plugins_res.status_code == 200
        plugins = plugins_res.json()["plugins"]
        ids = {p["id"] for p in plugins}
        assert {
            "lerobot_dataset_viz",
            "feetech_servo_bench",
            "lerobot_studio",
            "lelab_external",
            "remote_vla_policy",
        }.issubset(ids)

        assert client.post("/api/plugins/unknown_tool/launch").status_code == 404
        assert client.post("/api/plugins/unknown_tool/stop").status_code == 404

        if sys.platform == "win32":
            openpi_res = client.post("/api/plugins/remote_vla_policy/launch", json={"args": {}})
            assert openpi_res.status_code == 409
            assert openpi_res.json()["code"] == "OS_UNSUPPORTED"

        vla_res = client.get("/api/vla/configs")
        assert vla_res.status_code == 200
        assert len(vla_res.json()["configs"]) == 4
        assert "pi07_notice" in vla_res.json()

        pred_res = client.post("/api/vla/mock_predict", json={"policy_name": "pi05_so101_custom"})
        assert pred_res.status_code == 200
        assert len(pred_res.json()["action_chunk"]) == 5


def test_process_registry_reaps_grandchild_zero_orphans(tmp_path: Path) -> None:
    registry = ProcessRegistry(logs_dir=tmp_path / "logs")
    child_code = (
        "import subprocess, sys, time; "
        "gc = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)']); "
        "print(f'GC_PID={gc.pid}', flush=True); "
        "time.sleep(60)"
    )
    managed = registry.spawn("orphan_test", "orphan_test", [sys.executable, "-c", child_code])
    parent_pid = managed.proc.pid

    grandchild_pid = 0
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        for line in list(managed.log_tail):
            if line.startswith("GC_PID="):
                grandchild_pid = int(line.split("=", 1)[1])
                break
        if grandchild_pid:
            break
        time.sleep(0.05)

    assert grandchild_pid > 0, "Child process did not report grandchild PID."
    assert _pid_alive(parent_pid)
    assert _pid_alive(grandchild_pid)

    registry.terminate_all()
    time.sleep(0.2)
    assert not _pid_alive(parent_pid)
    assert not _pid_alive(grandchild_pid)


# ----------------------------------------------------------------------
# Group 4: WebSocket Telemetry Streaming
# ----------------------------------------------------------------------
@pytest.mark.asyncio
async def test_ws_telemetry_streams_6dof_and_logs(live_server: tuple[str, StudioConfig]) -> None:
    base_url, _ = live_server
    ws_url = base_url.replace("http://", "ws://") + "/ws/telemetry"
    async with websockets.connect(ws_url, open_timeout=5.0) as ws:
        raw = await ws.recv()
        msg = json.loads(raw)
        assert msg["mode"] in ("real", "dry_run")
        assert "active_workflow" in msg
        assert "workflow_status" in msg
        assert "log_tail" in msg
        for group in ("leader_joints", "follower_joints"):
            assert set(msg[group].keys()) == {
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            }


# ----------------------------------------------------------------------
# Group 5: Playwright Browser E2E Smoke Test
# ----------------------------------------------------------------------
def test_playwright_ui_smoke_and_zero_console_errors(
    live_server: tuple[str, StudioConfig],
) -> None:
    base_url, _ = live_server
    console_errors: list[str] = []
    page_errors: list[str] = []

    with sync_playwright() as p:
        browser = _launch_chromium(p)
        try:
            page = browser.new_page()
            page.on(
                "console",
                lambda msg: console_errors.append(msg.text) if msg.type == "error" else None,
            )
            page.on("pageerror", lambda err: page_errors.append(str(err)))

            page.goto(base_url, wait_until="domcontentloaded")
            page.get_by_test_id("mode-badge").wait_for(state="visible", timeout=5000)
            page.get_by_test_id("hardware-status-banner").wait_for(state="visible", timeout=5000)
            page.get_by_test_id("log-console").wait_for(state="visible", timeout=5000)
            page.locator("#mode-select").select_option("dry_run")
            page.wait_for_function(
                "() => document.querySelector('[data-testid=\"mode-badge\"]')?.innerText?.includes('DRY-RUN')",
                timeout=5000,
            )

            # Dashboard probe, manual device config, and interactive calibration
            page.get_by_test_id("tab-dashboard").click()
            page.get_by_test_id("btn-probe-hardware").click()
            page.wait_for_timeout(200)
            page.get_by_test_id("btn-swap-arms").click()
            page.get_by_test_id("btn-swap-cameras").click()
            page.get_by_test_id("btn-save-device-config").click()
            page.wait_for_timeout(200)

            # Interactive calibration in UI
            page.get_by_test_id("btn-start-calibrate").click()
            page.get_by_test_id("btn-calibrate-enter").wait_for(state="visible", timeout=5000)
            page.get_by_test_id("btn-calibrate-enter").click()
            page.wait_for_timeout(250)
            page.get_by_test_id("btn-calibrate-enter").click()
            page.wait_for_timeout(250)

            # Teleop & Camera JPEG verification
            page.get_by_test_id("tab-teleop").click()
            page.wait_for_function(
                """() => {
                    const f = document.querySelector('[data-testid="camera-preview-front"]');
                    const w = document.querySelector('[data-testid="camera-preview-wrist"]');
                    return f && f.complete && f.naturalWidth > 0 && w && w.complete && w.naturalWidth > 0;
                }""",
                timeout=5000,
            )
            page.get_by_test_id("btn-start-teleop").click()
            page.wait_for_timeout(250)
            page.get_by_test_id("btn-stop-teleop").click()

            # Teaching waypoints
            page.get_by_test_id("tab-teaching").click()
            page.get_by_test_id("btn-teach-waypoint").click()
            page.wait_for_timeout(200)
            page.get_by_test_id("btn-replay-waypoints").click()
            page.wait_for_timeout(450)

            # Data collection
            page.get_by_test_id("tab-collect").click()
            page.get_by_test_id("btn-start-collect").click()
            page.wait_for_timeout(350)
            page.get_by_test_id("btn-stop-collect").click()
            page.wait_for_timeout(200)

            # Plugins & VLA tabs
            page.get_by_test_id("tab-plugins").click()
            assert page.locator('[data-testid="plugin-list"] .card').count() >= 5

            page.get_by_test_id("tab-vla").click()
            assert "pi0.7" in page.locator("#pi07-notice-card").inner_text()

            assert console_errors == [], f"Unexpected console errors: {console_errors}"
            assert page_errors == [], f"Unexpected page errors: {page_errors}"
        finally:
            browser.close()


def test_hardware_auto_match_and_manual_config_persistence(
    live_server: tuple[str, StudioConfig],
) -> None:
    """Verify auto_match_devices + POST /api/hardware/config validation & .env persistence."""
    from app.backend.hardware import HardwareManager

    base_url, cfg = live_server
    stale_cfg = cfg.model_copy(
        update={
            "follower_port": "COM3",
            "leader_port": "COM4",
            "front_camera_id": "9",
            "wrist_camera_id": "8",
        }
    )
    hw = HardwareManager(stale_cfg)
    fake_ports = [
        {"device": "COM8", "is_robot_candidate": False},
        {"device": "COM11", "is_robot_candidate": True},
        {"device": "COM12", "is_robot_candidate": True},
    ]
    fake_cams = [
        {"id": "cam_0", "raw_id": "0", "index_or_path": 0},
        {"id": "cam_1", "raw_id": "1", "index_or_path": 1},
    ]
    changed = hw.auto_match_devices(fake_ports, fake_cams, force=False)
    assert changed is True
    assert stale_cfg.follower_port == "COM11"
    assert stale_cfg.leader_port == "COM12"
    assert fake_cams[0]["id"] == "front"
    assert fake_cams[1]["id"] == "wrist"

    with httpx.Client(base_url=base_url, timeout=10.0) as client:
        # Reject duplicate arm port or duplicate camera
        dup_arm = client.post(
            "/api/hardware/config",
            json={"follower_port": "COM11", "leader_port": "COM11", "persist_env": False},
        )
        assert dup_arm.status_code == 422

        dup_cam = client.post(
            "/api/hardware/config",
            json={"front_camera_id": "0", "wrist_camera_id": "0", "persist_env": False},
        )
        assert dup_cam.status_code == 422

        # Save valid manual selection to isolated .env
        ok = client.post(
            "/api/hardware/config",
            json={
                "follower_port": "COM12",
                "leader_port": "COM11",
                "front_camera_id": "1",
                "wrist_camera_id": "0",
                "persist_env": True,
            },
        )
        assert ok.status_code == 200
        body = ok.json()
        assert body["device_selection"]["follower_port"] == "COM12"
        assert body["device_selection"]["leader_port"] == "COM11"
        assert body["device_selection"]["front_camera_id"] == "1"
        assert body["device_selection"]["wrist_camera_id"] == "0"
        assert cfg.env_file.exists()
        env_text = cfg.env_file.read_text(encoding="utf-8")
        assert "COM12" in env_text and "COM11" in env_text


def test_interactive_calibration_flow_and_stdin_shim(
    live_server: tuple[str, StudioConfig],
) -> None:
    """Verify two-step interactive calibration via API and piped stdin shims."""
    import builtins
    import queue
    from app.backend.calibrate_runner import install_piped_stdin_shims

    base_url, cfg = live_server
    with httpx.Client(base_url=base_url, timeout=10.0) as client:
        assert client.post("/api/mode", json={"mode": "dry_run"}).status_code == 200
        # Idle input -> 409
        assert client.post("/api/workflows/calibrate/input", json={"value": ""}).status_code == 409

        start = client.post("/api/workflows/calibrate/start", json={"target": "follower"})
        assert start.status_code == 200
        time.sleep(0.15)
        st1 = client.get("/api/workflows/status").json()
        assert st1["calibrate_active"] is True
        assert st1["calibration"]["stage"] == "waiting_middle"
        assert st1["calibration"]["waiting_for_input"] is True

        # Step 1: confirm middle
        assert client.post("/api/workflows/calibrate/input", json={"value": ""}).status_code == 200
        time.sleep(0.25)
        st2 = client.get("/api/workflows/status").json()
        assert st2["calibration"]["stage"] == "recording_range"
        assert len(st2["calibration"]["live_table"]) == 6

        # Step 2: finish range recording
        assert client.post("/api/workflows/calibrate/input", json={"value": ""}).status_code == 200
        time.sleep(0.2)
        st3 = client.get("/api/workflows/status").json()
        assert st3["active_workflow"] == "idle"
        assert st3["calibration"]["stage"] == "completed"
        assert (cfg.dry_run_dir / "calibration" / f"follower_{cfg.follower_id}.json").is_file()

    # Unit test piped stdin shim (restore builtins.input afterward)
    orig_input = builtins.input
    try:
        import lerobot.motors.motors_bus as mb
        import lerobot.utils.utils as lu

        orig_lu_enter = lu.enter_pressed
        orig_mb_enter = mb.enter_pressed
        q: queue.Queue[str | None] = queue.Queue()
        install_piped_stdin_shims(q)
        q.put("c\n")
        assert builtins.input("Test prompt") == "c"
        assert mb.enter_pressed() is False
        q.put("\n")
        assert mb.enter_pressed() is True
    finally:
        builtins.input = orig_input
        lu.enter_pressed = orig_lu_enter
        mb.enter_pressed = orig_mb_enter
