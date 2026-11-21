"""Opaque-box helpers for TUST-iMaker LeRobot E2E verification suite.

Strictly opaque-box: never imports ``app.*`` or ``psutil``.
"""

from __future__ import annotations

import os
import re
import shutil
import socket
import subprocess
import sys
import time
import tomllib
from collections.abc import Callable
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import yaml

# Known non-CLI proper nouns / filenames that start with "lerobot-"
NON_CLI_LEROBOT_TOKENS: frozenset[str] = frozenset(
    {
        "lerobot-studio",
        "lerobot-arms",
        "lerobot-v3",
        "lerobot-dataset",
        "lerobot-so101",
    }
)

# Dataset-dependent CLIs that fail `--help` with ImportError when `datasets` is not installed
DATASET_DEPENDENT_CLIS: frozenset[str] = frozenset(
    {
        "lerobot-record",
        "lerobot-replay",
        "lerobot-dataset-viz",
        "lerobot-edit-dataset",
        "lerobot-train",
        "lerobot-rollout",
        "lerobot-eval",
    }
)

REQUIRED_STUDIO_TESTIDS: tuple[str, ...] = (
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


def find_free_port() -> int:
    """Allocate an ephemeral free TCP port on 127.0.0.1."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def is_port_open(port: int, host: str = "127.0.0.1") -> bool:
    """Return True if a TCP connection succeeds on host:port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.3)
        return sock.connect_ex((host, port)) == 0


def wait_until(
    predicate: Callable[[], Any],
    *,
    timeout: float = 10.0,
    interval: float = 0.2,
    description: str = "condition",
) -> Any:
    """Poll predicate until it returns a truthy value or timeout expires."""
    deadline = time.monotonic() + timeout
    last_exc: Exception | None = None
    while True:
        try:
            result = predicate()
            if result:
                return result
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
        if time.monotonic() >= deadline:
            detail = f" (last error: {last_exc})" if last_exc is not None else ""
            raise AssertionError(f"Timed out after {timeout:.1f}s waiting for {description}{detail}")
        time.sleep(interval)


def normalize_git_url(url: str) -> str:
    """Normalize HTTPS/SSH Git URLs for duplicate & remote-match detection."""
    raw = url.strip()
    if raw.startswith("git@") and ":" in raw:
        host_part, path_part = raw.split(":", 1)
        host = host_part.split("@", 1)[-1].lower()
        path = path_part.strip("/")
    else:
        parsed = urlparse(raw)
        host = parsed.netloc.split("@")[-1].lower()
        path = parsed.path.strip("/")
    if path.endswith(".git"):
        path = path[:-4]
    return f"{host}/{path}".lower()


def run_cmd(
    cmd: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    timeout: float = 30.0,
) -> subprocess.CompletedProcess[str]:
    """Run a subprocess synchronously with UTF-8 decoding."""
    merged_env = os.environ.copy()
    merged_env.setdefault("PYTHONIOENCODING", "utf-8")
    if env:
        merged_env.update(env)
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        env=merged_env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        check=False,
    )


def run_git(repo_dir: Path, *args: str) -> subprocess.CompletedProcess[str]:
    """Execute a git command inside ``repo_dir``."""
    return run_cmd(["git", "-C", str(repo_dir), *args])


def load_yaml(path: Path) -> Any:
    """Parse a YAML file using PyYAML safe_load."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def parse_lerobot_entrypoints(workspace_root: Path) -> set[str]:
    """Read ground-truth [project.scripts] CLI names from repos/lerobot/pyproject.toml."""
    pyproject_path = workspace_root / "repos" / "lerobot" / "pyproject.toml"
    assert pyproject_path.is_file(), f"Missing lerobot pyproject.toml: {pyproject_path}"
    data = tomllib.loads(pyproject_path.read_text(encoding="utf-8"))
    scripts = data.get("project", {}).get("scripts", {})
    assert isinstance(scripts, dict) and scripts, "repos/lerobot [project.scripts] is empty"
    return set(scripts.keys())


def parse_openpi_train_configs(workspace_root: Path) -> set[str]:
    """Extract registered TrainConfig names from repos/openpi/src/openpi/training/config.py."""
    config_py = workspace_root / "repos" / "openpi" / "src" / "openpi" / "training" / "config.py"
    assert config_py.is_file(), f"Missing openpi config.py: {config_py}"
    text = config_py.read_text(encoding="utf-8")
    return set(re.findall(r'TrainConfig\(\s*name=["\']([^"\']+)["\']', text))


def contains_chinese(text: str) -> bool:
    """Return True if text contains CJK Unified Ideographs."""
    return bool(re.search(r"[\u4e00-\u9fff]", text))


def strip_fenced_code_blocks(text: str) -> str:
    """Remove fenced ```...``` blocks for link/prose scanning."""
    return re.sub(r"```.*?```", "", text, flags=re.DOTALL)


def extract_markdown_links(md_path: Path) -> list[tuple[str, str]]:
    """Return list of (label, target) for non-image markdown links outside code blocks."""
    raw = md_path.read_text(encoding="utf-8")
    cleaned = strip_fenced_code_blocks(raw)
    # Remove inline code spans so syntax examples don't look like links
    cleaned = re.sub(r"`[^`]*`", "", cleaned)
    matches: list[tuple[str, str]] = []
    for match in re.finditer(r"(?<!!)\[([^\]]+)\]\(([^)]+)\)", cleaned):
        label = match.group(1).strip()
        target = match.group(2).strip()
        # Strip optional title part: path "title"
        if " " in target and not target.startswith("<"):
            target = target.split(" ", 1)[0]
        target = target.strip("<>")
        matches.append((label, target))
    return matches


def find_broken_markdown_links(
    md_path: Path,
    workspace_root: Path,
    *,
    skip_prefixes: tuple[str, ...] = (),
) -> list[str]:
    """Return broken relative markdown targets from ``md_path``."""
    broken: list[str] = []
    for _label, target in extract_markdown_links(md_path):
        if not target or target.startswith(("#", "http://", "https://", "mailto:")):
            continue
        clean_target = target.split("#", 1)[0].split("?", 1)[0].strip()
        if not clean_target:
            continue
        if any(clean_target.startswith(prefix) for prefix in skip_prefixes):
            continue
        resolved = (md_path.parent / clean_target).resolve()
        if not resolved.exists():
            # Also allow workspace-root-relative markdown links if explicitly prefixed
            alt_resolved = (workspace_root / clean_target).resolve()
            if not alt_resolved.exists():
                broken.append(f"{md_path.relative_to(workspace_root)} -> {target}")
    return broken


def extract_lerobot_cli_tokens(md_text: str) -> set[str]:
    """Extract `lerobot-*` CLI command tokens from markdown text."""
    # Match standalone lerobot-<subcommand> tokens, ignoring prefixes like `99-lerobot-arms.rules`
    # or suffixes like `.rules` / `.yaml` / `.py`.
    tokens: set[str] = set()
    for m in re.finditer(r"(?<![A-Za-z0-9_/-])(lerobot-[a-z0-9]+(?:-[a-z0-9]+)*)(?![A-Za-z0-9_./-])", md_text):
        tok = m.group(1)
        if tok in NON_CLI_LEROBOT_TOKENS:
            continue
        tokens.add(tok)
    return tokens


def extract_workspace_relative_paths(md_text: str) -> set[str]:
    """Extract concrete workspace-relative paths under core top-level dirs."""
    pattern = re.compile(
        r"(?<![A-Za-z0-9_./-])((?:scripts|configs|app|repos|docs)/"
        r"[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*)"
    )
    paths: set[str] = set()
    for match in pattern.finditer(md_text):
        candidate = match.group(1).rstrip(".,;:)")
        # Skip template placeholders or user-copied local files documented in setup
        if any(ch in candidate for ch in ("<", ">", "{", "}", "*")):
            continue
        if candidate.endswith("/"):
            candidate = candidate.rstrip("/")
        if candidate in (
            "configs/robots/arm101_win.yaml",
            "configs/robots/arm101_ubuntu.yaml",
        ):
            continue
        # Skip pure top-level folder names like "docs" or "scripts" (already covered elsewhere)
        if "/" not in candidate:
            continue
        paths.add(candidate)
    return paths


def find_hardcoded_user_paths(text: str) -> list[str]:
    """Detect machine-specific user paths (C:\\Users\\<name>, /home/<user>/)."""
    findings: list[str] = []
    # Windows user paths (ignore generic placeholders like <user>)
    for match in re.finditer(r"[A-Za-z]:[/\\]Users[/\\]([A-Za-z0-9_.-]+)", text):
        user = match.group(1).lower()
        if user not in ("<user>", "user", "your_user", "username"):
            findings.append(match.group(0))
    # Linux home paths (ignore $HOME, <user>, your_user, ubuntu/user if placeholder? Flag concrete users)
    for match in re.finditer(r"/home/([A-Za-z0-9_.-]+)/", text):
        user = match.group(1).lower()
        if user not in ("<user>", "user", "your_user", "username", "$user"):
            findings.append(match.group(0))
    return findings


def find_secret_leaks(text: str) -> list[str]:
    """Detect non-placeholder HF/OpenAI/W&B secret tokens."""
    findings: list[str] = []
    for match in re.finditer(r"\b(hf_[A-Za-z0-9]{20,}|sk-[A-Za-z0-9]{20,}|wandb_[A-Za-z0-9]{20,})\b", text):
        findings.append(match.group(1)[:12] + "...")
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        for key in ("HF_TOKEN", "WANDB_API_KEY"):
            if stripped.startswith(f"{key}="):
                val = stripped.split("=", 1)[1].strip().strip('"').strip("'")
                if val and val.lower() not in (
                    "",
                    "null",
                    "none",
                    "your_token",
                    "your_hf_token",
                    "<token>",
                    "${hf_token}",
                ):
                    findings.append(f"{key}=<non-empty>")
    return findings


def resolve_bash_executable() -> tuple[str, str] | None:
    """Return (kind, executable_path) for bash syntax/execution checks."""
    if sys.platform != "win32":
        bash = shutil.which("bash")
        return ("posix", bash) if bash else None
    git_bash = Path(r"C:\Program Files\Git\bin\bash.exe")
    if git_bash.is_file():
        return ("git_bash", str(git_bash))
    wsl = shutil.which("wsl")
    if wsl:
        return ("wsl", wsl)
    return None


def to_wsl_path(win_path: Path) -> str:
    """Convert a Windows Path like C:\\Users\\... to /mnt/c/Users/..."""
    resolved = win_path.resolve()
    drive = resolved.drive.rstrip(":").lower()
    tail = resolved.as_posix().split(":", 1)[-1].lstrip("/")
    return f"/mnt/{drive}/{tail}"


def validate_shell_script_lf_and_syntax(sh_path: Path) -> None:
    """Enforce strict LF bytes and bash syntax validity on ``sh_path``."""
    raw = sh_path.read_bytes()
    assert b"\r\n" not in raw, f"CRLF detected in shell script: {sh_path}"
    assert b"\r" not in raw, f"Stray CR byte detected in shell script: {sh_path}"
    text = raw.decode("utf-8")
    assert text.startswith("#!/usr/bin/env bash") or text.startswith("#!/bin/bash"), (
        f"Missing bash shebang in {sh_path}"
    )

    bash_info = resolve_bash_executable()
    if bash_info is not None:
        kind, exe = bash_info
        if kind == "wsl":
            res = run_cmd([exe, "--", "bash", "-n", to_wsl_path(sh_path)])
        else:
            res = run_cmd([exe, "-n", str(sh_path)])
        assert res.returncode == 0, f"bash -n failed for {sh_path}: {res.stderr or res.stdout}"
        return

    # Pure-Python static fallback if no bash binary exists on host
    for open_kw, close_kw in (("if ", "fi"), ("case ", "esac"), ("while ", "done")):
        opens = len(re.findall(rf"(?:^|[;\s]){open_kw}", text, flags=re.MULTILINE))
        closes = len(re.findall(rf"(?:^|[;\s]){close_kw}\b", text, flags=re.MULTILINE))
        assert opens == closes, f"Unbalanced {open_kw.strip()}/{close_kw} in {sh_path}: {opens} vs {closes}"


def run_ubuntu_script(sh_path: Path, args: list[str]) -> subprocess.CompletedProcess[str] | None:
    """Execute ``setup_ubuntu.sh`` via POSIX bash, Git Bash, or WSL if available."""
    bash_info = resolve_bash_executable()
    if bash_info is None:
        return None
    kind, exe = bash_info
    if kind == "wsl":
        return run_cmd([exe, "--", "bash", to_wsl_path(sh_path), *args], timeout=30.0)
    return run_cmd([exe, str(sh_path), *args], timeout=30.0)


def validate_powershell_ast(ps1_path: Path) -> None:
    """Validate PowerShell script syntax via AST parser on Windows or static check on POSIX."""
    ps_exe = shutil.which("powershell") or shutil.which("pwsh")
    if ps_exe:
        escaped = str(ps1_path).replace("'", "''")
        cmd = (
            "$tokens = $null; $errors = $null; "
            f"$null = [System.Management.Automation.Language.Parser]::ParseFile('{escaped}', "
            "[ref]$tokens, [ref]$errors); "
            "if ($errors.Count -gt 0) { "
            "$errors | ForEach-Object { Write-Error $_.ToString() }; exit 1 "
            "}"
        )
        res = run_cmd([ps_exe, "-NoProfile", "-Command", cmd])
        assert res.returncode == 0, f"PowerShell AST parse failed for {ps1_path}: {res.stderr}"
        return

    # Cross-OS static structural fallback when pwsh is unavailable on Ubuntu
    text = ps1_path.read_text(encoding="utf-8")
    assert text.count("{") == text.count("}"), f"Unbalanced braces in {ps1_path}"
    assert text.count("(") == text.count(")"), f"Unbalanced parens in {ps1_path}"
    assert "param(" in text.lower(), f"Missing param() block in {ps1_path}"
