#!/usr/bin/env python3
"""Cross-OS zero-dependency environment & workspace integrity checker."""

from __future__ import annotations

import argparse
import glob
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


def normalize_git_url(url: str) -> str:
    raw = url.strip()
    if raw.startswith("git@") and ":" in raw:
        host_part, path_part = raw.split(":", 1)
        host = host_part.split("@", 1)[-1].lower()
        path = path_part.strip("/")
    else:
        parsed = urlparse(raw)
        host = parsed.netloc.split("@")[-1].lower()
        path = parsed.path.strip("/")
    path = path.removesuffix(".git")
    return f"{host}/{path}".lower()


def parse_repos_yaml_stdlib(config_path: Path) -> list[dict[str, str]]:
    if not config_path.exists():
        return []
    repos: list[dict[str, str]] = []
    current: dict[str, str] | None = None
    in_repos = False
    for raw in config_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        stripped = line.lstrip()
        indent = len(line) - len(stripped)
        if indent == 0 and stripped.startswith("repos:"):
            in_repos = True
            continue
        if indent == 0 and not stripped.startswith("-"):
            in_repos = False
            continue
        if not in_repos:
            continue
        if stripped.startswith("- "):
            if current:
                repos.append(current)
            current = {}
            rest = stripped[2:].strip()
            if ":" in rest:
                k, v = rest.split(":", 1)
                current[k.strip()] = v.strip().strip('"').strip("'")
        elif current is not None and ":" in stripped:
            k, v = stripped.split(":", 1)
            current[k.strip()] = v.strip().strip('"').strip("'")
    if current:
        repos.append(current)
    return repos


def run_capture(cmd: list[str], timeout: int = 10) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
        return proc.returncode, proc.stdout.strip(), proc.stderr.strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, "", str(exc)


def check_os_runtime(workspace_root: Path) -> dict[str, Any]:
    system = platform.system()
    is_wsl = False
    if system == "Linux":
        try:
            proc_ver = Path("/proc/version").read_text(encoding="utf-8", errors="ignore").lower()
            is_wsl = "microsoft" in proc_ver or "wsl" in proc_ver
        except OSError:
            pass
    ws_disk = shutil.disk_usage(workspace_root)
    home_disk = shutil.disk_usage(Path.home())
    return {
        "status": "PASS",
        "os": system,
        "release": platform.release(),
        "is_wsl": is_wsl,
        "python_version": platform.python_version(),
        "python_executable": sys.executable,
        "workspace_free_gb": round(ws_disk.free / (1024**3), 2),
        "home_free_gb": round(home_disk.free / (1024**3), 2),
    }


def check_cli_tools() -> dict[str, Any]:
    tools = ["uv", "git", "ffmpeg", "node", "pnpm", "nvidia-smi"]
    if platform.system() == "Linux":
        tools.append("udevadm")
    found: dict[str, str | None] = {name: shutil.which(name) for name in tools}
    if platform.system() == "Linux" and not found.get("uv"):
        for candidate in (Path.home() / ".local/bin/uv", Path.home() / ".cargo/bin/uv"):
            if candidate.exists():
                found["uv"] = str(candidate)
                break
    gpu_info = None
    if found.get("nvidia-smi"):
        rc, out, _ = run_capture(
            [found["nvidia-smi"], "--query-gpu=name,memory.total", "--format=csv,noheader"]
        )
        if rc == 0 and out:
            gpu_info = out
    status = "PASS" if found.get("uv") and found.get("git") else "WARN"
    return {"status": status, "tools": found, "gpu": gpu_info}


def check_git_and_repos(workspace_root: Path) -> dict[str, Any]:
    rc, root_origin, _ = run_capture(["git", "-C", str(workspace_root), "remote", "get-url", "origin"])
    configured = parse_repos_yaml_stdlib(workspace_root / "configs" / "repos.yaml")
    repos_dir = workspace_root / "repos"
    repo_statuses: list[dict[str, Any]] = []
    seen_urls: dict[str, list[str]] = {}

    for entry in configured:
        name = entry.get("name", "")
        expected_url = entry.get("url", "")
        target = repos_dir / name
        r_rc, r_origin, _ = (
            run_capture(["git", "-C", str(target), "remote", "get-url", "origin"])
            if (target / ".git").exists()
            else (1, "", "")
        )
        ok = r_rc == 0 and normalize_git_url(r_origin) == normalize_git_url(expected_url)
        repo_statuses.append(
            {
                "name": name,
                "exists": target.exists(),
                "git_ok": ok,
                "origin": r_origin or None,
            }
        )

    # Duplicate git clone detection across workspace top-level and repos/*
    candidates: list[tuple[str, Path]] = []
    if repos_dir.exists():
        for child in sorted(repos_dir.iterdir()):
            if (child / ".git").exists():
                candidates.append((f"repos/{child.name}", child))
    for child in sorted(workspace_root.iterdir()):
        if child.is_dir() and child.name != "repos" and (child / ".git").exists():
            candidates.append((child.name, child))

    for label, path in candidates:
        c_rc, c_origin, _ = run_capture(["git", "-C", str(path), "remote", "get-url", "origin"])
        if c_rc == 0 and c_origin:
            seen_urls.setdefault(normalize_git_url(c_origin), []).append(label)

    duplicates = {u: locs for u, locs in seen_urls.items() if len(locs) > 1}
    missing = [r["name"] for r in repo_statuses if not r["git_ok"]]
    status = "PASS" if rc == 0 and not missing and not duplicates else ("FAIL" if duplicates else "WARN")
    return {
        "status": status,
        "root_origin": root_origin if rc == 0 else None,
        "configured_count": len(configured),
        "missing_or_invalid": missing,
        "duplicate_clones": duplicates,
        "repos": repo_statuses,
    }


def check_shell_line_endings(workspace_root: Path) -> dict[str, Any]:
    sh_files = sorted((workspace_root / "scripts").rglob("*.sh"))
    crlf_files: list[str] = []
    for fpath in sh_files:
        if b"\r\n" in fpath.read_bytes():
            crlf_files.append(str(fpath.relative_to(workspace_root)))
    return {
        "status": "FAIL" if crlf_files else "PASS",
        "checked_files": [str(p.relative_to(workspace_root)) for p in sh_files],
        "crlf_violations": crlf_files,
    }


def probe_venv_imports(python_exe: Path) -> dict[str, Any]:
    modules = [
        "fastapi",
        "uvicorn",
        "websockets",
        "serial",
        "cv2",
        "yaml",
        "pytest",
        "httpx",
        "playwright",
        "lerobot",
        "rustypot",
    ]
    code = (
        "import importlib, json\n"
        f"mods = {modules!r}\n"
        "res = {}\n"
        "for m in mods:\n"
        "  try:\n"
        "    importlib.import_module(m); res[m] = True\n"
        "  except Exception as e:\n"
        "    res[m] = str(e)\n"
        "print(json.dumps(res))\n"
    )
    rc, out, err = run_capture([str(python_exe), "-c", code], timeout=25)
    if rc != 0:
        return {"ok": False, "error": err or out, "imports": {}}
    try:
        parsed = json.loads(out.splitlines()[-1])
        missing = [k for k, v in parsed.items() if v is not True]
        return {"ok": len(missing) == 0, "missing_imports": missing, "imports": parsed}
    except (json.JSONDecodeError, IndexError) as exc:
        return {"ok": False, "error": f"Failed parsing probe output: {exc}", "imports": {}}


def check_venvs(workspace_root: Path) -> dict[str, Any]:
    system = platform.system()
    if system == "Windows":
        win_rel = os.environ.get("LEROBOT_WIN_VENV", "envs/win/lerobot")
        win_venv = Path(win_rel) if Path(win_rel).is_absolute() else workspace_root / win_rel
        py = win_venv / "Scripts" / "python.exe"
        if not py.exists():
            return {"status": "WARN", "lerobot_venv": str(win_venv), "exists": False}
        probe = probe_venv_imports(py)
        return {
            "status": "PASS" if probe["ok"] else "WARN",
            "lerobot_venv": str(win_venv),
            "python": str(py),
            "exists": True,
            **probe,
        }

    lerobot_venv = Path(os.environ.get("LEROBOT_UBUNTU_VENV", str(Path.home() / ".venvs" / "tust-lerobot")))
    openpi_venv = Path(os.environ.get("OPENPI_UBUNTU_VENV", str(Path.home() / ".venvs" / "tust-openpi")))
    py_lerobot = lerobot_venv / "bin" / "python"
    py_openpi = openpi_venv / "bin" / "python"
    res: dict[str, Any] = {
        "lerobot_venv": str(lerobot_venv),
        "lerobot_exists": py_lerobot.exists(),
        "openpi_venv": str(openpi_venv),
        "openpi_exists": py_openpi.exists(),
    }
    if py_lerobot.exists():
        probe = probe_venv_imports(py_lerobot)
        res["lerobot_probe"] = probe
        res["status"] = "PASS" if probe["ok"] else "WARN"
    else:
        res["status"] = "WARN"
    return res


def check_hardware_snapshot(workspace_root: Path) -> dict[str, Any]:
    system = platform.system()
    if system == "Windows":
        win_py = workspace_root / os.environ.get("LEROBOT_WIN_VENV", "envs/win/lerobot") / "Scripts" / "python.exe"
        if win_py.exists():
            rc, out, _ = run_capture(
                [
                    str(win_py),
                    "-c",
                    (
                        "import json, serial.tools.list_ports; "
                        "print(json.dumps([{'port': p.device, 'desc': p.description, 'vid': p.vid, 'pid': p.pid} "
                        "for p in serial.tools.list_ports.comports()]))"
                    ),
                ]
            )
            if rc == 0 and out:
                try:
                    return {"status": "PASS", "serial_ports": json.loads(out)}
                except json.JSONDecodeError:
                    pass
        return {"status": "PASS", "serial_ports": [], "note": "Run scripts/windows/find_ports.ps1 for full PnP scan."}

    ttys = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    videos = sorted(glob.glob("/dev/video*"))
    return {"status": "PASS", "serial_ports": ttys, "video_devices": videos}


def main(argv: list[str] | None = None) -> int:
    default_ws = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="Check TUST-iMaker LeRobot workspace health.")
    parser.add_argument("--workspace", type=Path, default=default_ws, help="Workspace root path.")
    parser.add_argument("--json", action="store_true", help="Print JSON output.")
    parser.add_argument("--strict", action="store_true", help="Exit 1 if any check fails.")
    args = parser.parse_args(argv)

    workspace_root = args.workspace.resolve()
    if not workspace_root.exists():
        print(f"ERROR: Workspace path does not exist: {workspace_root}", file=sys.stderr)
        return 2

    checks = {
        "os_runtime": check_os_runtime(workspace_root),
        "cli_tools": check_cli_tools(),
        "git_and_repos": check_git_and_repos(workspace_root),
        "shell_line_endings": check_shell_line_endings(workspace_root),
        "venvs": check_venvs(workspace_root),
        "hardware_snapshot": check_hardware_snapshot(workspace_root),
    }

    has_fail = any(v.get("status") == "FAIL" for v in checks.values())
    report = {
        "workspace_root": str(workspace_root),
        "overall_status": "FAIL" if has_fail else "OK",
        "checks": checks,
    }

    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        print(f"=== Workspace Check: {workspace_root} ===")
        for name, data in checks.items():
            print(f"[{data['status']}] {name}")
            if name == "git_and_repos":
                print(
                    f"       origin={data['root_origin']}, configured={data['configured_count']}, "
                    f"missing={data['missing_or_invalid']}, duplicates={data['duplicate_clones']}"
                )
            elif name == "shell_line_endings" and data["crlf_violations"]:
                print(f"       CRLF violations: {data['crlf_violations']}")
            elif name == "venvs":
                print(f"       details: {data}")

    if args.strict and has_fail:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
