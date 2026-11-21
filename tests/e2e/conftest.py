"""Shared pytest fixtures and marker registration for E2E verification suite.

IMPORTANT:
- Registers `tier1`, `tier2`, `tier3`, `tier4` here via `pytest_configure` so we do NOT
  create `tests/e2e/pytest.ini` (which would shadow root `pyproject.toml` and break
  `asyncio_mode = "auto"`).
- Uses only stdlib + httpx + playwright (no `psutil`, no `app.backend.*` imports).
"""

from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Callable, Generator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import httpx
import pytest
from playwright.sync_api import Browser, Error as PlaywrightError, Page, Playwright, sync_playwright

_WORKSPACE_ROOT = str(Path(__file__).resolve().parents[2])
if _WORKSPACE_ROOT not in sys.path:
    sys.path.insert(0, _WORKSPACE_ROOT)

from tests.e2e._helpers import find_free_port, is_port_open, wait_until


def pytest_configure(config: pytest.Config) -> None:
    """Register custom E2E tier markers without shadowing root pyproject.toml."""
    config.addinivalue_line("markers", "tier1: Tier 1 isolated feature coverage tests")
    config.addinivalue_line("markers", "tier2: Tier 2 boundary, negative, and hygiene tests")
    config.addinivalue_line("markers", "tier3: Tier 3 cross-feature integration tests")
    config.addinivalue_line("markers", "tier4: Tier 4 real-world end-to-end onboarding scenarios")


@dataclass
class StudioServerHandle:
    """Opaque handle to a running Arm101 Studio server subprocess."""

    base_url: str
    ws_url: str
    port: int
    pid: int
    process: subprocess.Popen[str]
    log_path: Path

    def read_logs(self) -> str:
        if self.log_path.exists():
            return self.log_path.read_text(encoding="utf-8", errors="replace")
        return ""


@dataclass
class DryRunSandbox:
    """Tracks `data/dry_run/` artifacts created during a test for clean isolation."""

    dry_run_dir: Path
    pre_existing_files: set[Path]

    def list_current_files(self) -> set[Path]:
        if not self.dry_run_dir.exists():
            return set()
        return {
            p.relative_to(self.dry_run_dir)
            for p in self.dry_run_dir.rglob("*")
            if p.is_file()
        }

    def list_new_files(self) -> set[Path]:
        return self.list_current_files() - self.pre_existing_files


def _terminate_process_tree(proc: subprocess.Popen[str], port: int) -> None:
    """Terminate a process group cleanly and verify the bound port is released."""
    if proc.poll() is None:
        if sys.platform == "win32":
            # Kill entire process tree so uvicorn child processes cannot linger
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                capture_output=True,
                text=True,
                check=False,
            )
        else:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)

    assert proc.poll() is not None, f"Studio process {proc.pid} did not terminate cleanly"
    wait_until(
        lambda: not is_port_open(port),
        timeout=5.0,
        interval=0.1,
        description=f"port {port} to be released after server teardown",
    )


def _launch_studio_subprocess(
    workspace_root: Path,
    *,
    cmd: list[str] | None = None,
    extra_env: dict[str, str] | None = None,
    port: int | None = None,
) -> StudioServerHandle:
    """Start Arm101 Studio on an ephemeral port and wait for /api/health == 200."""
    chosen_port = port or find_free_port()
    target_cmd = cmd or [sys.executable, "-m", "app.backend.main"]
    env = os.environ.copy()
    env.update(
        {
            "STUDIO_HOST": "127.0.0.1",
            "STUDIO_PORT": str(chosen_port),
            "STUDIO_MODE": "auto",
            "PYTHONUNBUFFERED": "1",
            "PYTHONIOENCODING": "utf-8",
        }
    )
    if extra_env:
        env.update(extra_env)

    log_fd, log_name = tempfile.mkstemp(prefix="studio_e2e_", suffix=".log")
    os.close(log_fd)
    log_path = Path(log_name)
    log_file = log_path.open("w", encoding="utf-8", errors="replace")

    popen_kwargs: dict[str, Any] = {
        "cwd": str(workspace_root),
        "env": env,
        "stdout": log_file,
        "stderr": subprocess.STDOUT,
        "text": True,
    }
    if sys.platform == "win32":
        popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        popen_kwargs["start_new_session"] = True

    proc = subprocess.Popen(target_cmd, **popen_kwargs)
    log_file.close()

    base_url = f"http://127.0.0.1:{chosen_port}"
    ws_url = f"ws://127.0.0.1:{chosen_port}/ws/telemetry"
    handle = StudioServerHandle(
        base_url=base_url,
        ws_url=ws_url,
        port=chosen_port,
        pid=proc.pid,
        process=proc,
        log_path=log_path,
    )

    try:
        def _health_ready() -> bool:
            if proc.poll() is not None:
                logs = handle.read_logs()
                raise RuntimeError(
                    f"Studio server exited prematurely (rc={proc.returncode}):\n{logs}"
                )
            try:
                resp = httpx.get(f"{base_url}/api/health", timeout=1.0)
                return resp.status_code == 200
            except httpx.HTTPError:
                return False

        wait_until(
            _health_ready,
            timeout=15.0,
            interval=0.2,
            description=f"Studio /api/health on port {chosen_port}",
        )
    except Exception:
        _terminate_process_tree(proc, chosen_port)
        log_path.unlink(missing_ok=True)
        raise

    return handle


def _launch_chromium_with_fallback(playwright: Playwright) -> Browser:
    """Launch headless Chromium, falling back to msedge/chrome on Windows if unprovisioned."""
    errors: list[str] = []
    try:
        return playwright.chromium.launch(headless=True)
    except PlaywrightError as exc:
        errors.append(f"default chromium: {exc}")

    if sys.platform == "win32":
        for channel in ("msedge", "chrome"):
            try:
                return playwright.chromium.launch(channel=channel, headless=True)
            except PlaywrightError as exc:
                errors.append(f"channel={channel}: {exc}")

        # Fallback to any cached headless shell in ms-playwright
        ms_pw_dir = Path.home() / "AppData" / "Local" / "ms-playwright"
        if ms_pw_dir.exists():
            shells = sorted(
                ms_pw_dir.glob("chromium_headless_shell-*/chrome-headless-shell-win64/*.exe"),
                reverse=True,
            )
            for shell_exe in shells:
                try:
                    return playwright.chromium.launch(executable_path=str(shell_exe), headless=True)
                except PlaywrightError as exc:
                    errors.append(f"{shell_exe}: {exc}")

    pytest.fail(
        "Failed to launch Playwright Chromium browser. Run "
        "`envs\\win\\lerobot\\Scripts\\python.exe -m playwright install chromium` "
        "(or `scripts\\windows\\setup_win.ps1 -WithPlaywrightBrowser`).\n"
        + "\n".join(errors)
    )


@pytest.fixture(scope="session")
def workspace_root() -> Path:
    """Return the root TUST-iMaker/lerobot workspace Path."""
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def _playwright_browser() -> Generator[Browser, None, None]:
    """Session-scoped headless Chromium browser instance."""
    pw = sync_playwright().start()
    browser = _launch_chromium_with_fallback(pw)
    try:
        yield browser
    finally:
        browser.close()
        pw.stop()


@pytest.fixture
def studio_server_factory(
    workspace_root: Path,
) -> Generator[Callable[..., StudioServerHandle], None, None]:
    """Factory fixture to launch custom Studio server instances and enforce clean teardown."""
    handles: list[StudioServerHandle] = []

    def _start(
        *,
        cmd: list[str] | None = None,
        extra_env: dict[str, str] | None = None,
        port: int | None = None,
    ) -> StudioServerHandle:
        handle = _launch_studio_subprocess(
            workspace_root,
            cmd=cmd,
            extra_env=extra_env,
            port=port,
        )
        handles.append(handle)
        return handle

    try:
        yield _start
    finally:
        for handle in reversed(handles):
            try:
                _terminate_process_tree(handle.process, handle.port)
            finally:
                handle.log_path.unlink(missing_ok=True)


@pytest.fixture
def studio_server(
    studio_server_factory: Callable[..., StudioServerHandle],
) -> StudioServerHandle:
    """Function-scoped cold-booted Arm101 Studio server on an ephemeral port."""
    return studio_server_factory()


@pytest.fixture
def http_client(studio_server: StudioServerHandle) -> Generator[httpx.Client, None, None]:
    """HTTPX client bound to the ephemeral `studio_server`."""
    with httpx.Client(base_url=studio_server.base_url, timeout=15.0) as client:
        yield client


@pytest.fixture
def browser_page(
    _playwright_browser: Browser,
    studio_server: StudioServerHandle,
) -> Generator[Page, None, None]:
    """Isolated Playwright Page opened at `studio_server.base_url` with error tracking."""
    context = _playwright_browser.new_context(viewport={"width": 1280, "height": 800})
    page = context.new_page()
    console_errors: list[str] = []
    page_errors: list[str] = []
    # Tests that intentionally trigger 4xx fetch responses can set this attribute to True
    page._e2e_allow_http_error_console = False  # type: ignore[attr-defined]
    page._e2e_console_errors = console_errors  # type: ignore[attr-defined]
    page._e2e_page_errors = page_errors  # type: ignore[attr-defined]

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

    page.goto(studio_server.base_url, wait_until="load", timeout=15000)
    page.locator("[data-testid='mode-badge']").wait_for(state="attached", timeout=10000)
    # Give initial async fetch/WS hydration a brief beat to settle
    time.sleep(0.2)

    try:
        yield page
    finally:
        context.close()
        assert not page_errors, f"Uncaught pageerror(s): {page_errors}"
        assert not console_errors, f"Browser console error(s): {console_errors}"


@pytest.fixture
def dry_run_sandbox(workspace_root: Path) -> Generator[DryRunSandbox, None, None]:
    """Snapshot `data/dry_run/` before test and remove only test-created artifacts."""
    dry_run_dir = workspace_root / "data" / "dry_run"
    pre_existing = set()
    if dry_run_dir.exists():
        pre_existing = {
            p.relative_to(dry_run_dir)
            for p in dry_run_dir.rglob("*")
            if p.is_file()
        }
    sandbox = DryRunSandbox(dry_run_dir=dry_run_dir, pre_existing_files=pre_existing)
    try:
        yield sandbox
    finally:
        if dry_run_dir.exists():
            for item in sorted(dry_run_dir.rglob("*"), reverse=True):
                if item.is_file():
                    rel = item.relative_to(dry_run_dir)
                    if rel.name != ".gitkeep" and rel not in pre_existing:
                        item.unlink(missing_ok=True)
                elif item.is_dir():
                    if not any(item.iterdir()):
                        shutil.rmtree(item, ignore_errors=True)
