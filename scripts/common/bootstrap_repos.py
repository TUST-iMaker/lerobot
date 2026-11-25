#!/usr/bin/env python3
"""Idempotent, non-duplicate upstream repository bootstrapper for TUST-iMaker/lerobot."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


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
    path = path.removesuffix(".git")
    return f"{host}/{path}".lower()


def _parse_scalar(val: str) -> Any:
    val = val.strip()
    if not val:
        return ""
    if (val.startswith('"') and val.endswith('"')) or (val.startswith("'") and val.endswith("'")):
        return val[1:-1]
    low = val.lower()
    if low == "true":
        return True
    if low == "false":
        return False
    if low in ("null", "none", "~"):
        return None
    if val.startswith("[") and val.endswith("]"):
        inner = val[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(item) for item in inner.split(",")]
    try:
        return int(val)
    except ValueError:
        return val


def _strip_inline_comment(line: str) -> str:
    in_single = False
    in_double = False
    for idx, ch in enumerate(line):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == "#" and not in_single and not in_double and (idx == 0 or line[idx - 1].isspace()):
            return line[:idx]
    return line


def load_repos_config(config_path: Path) -> list[dict[str, Any]]:
    """Load configs/repos.yaml using PyYAML if available, else stdlib fallback."""
    text = config_path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(text) or {}
        repos = data.get("repos", [])
        if not isinstance(repos, list):
            raise TypeError("'repos' in config must be a list.")
        return repos
    except ImportError:
        pass

    # Minimal stdlib YAML list-of-dicts parser for pre-venv execution
    repos: list[dict[str, Any]] = []
    in_repos = False
    current: dict[str, Any] | None = None
    item_indent: int | None = None
    active_list_key: str | None = None

    for raw_line in text.splitlines():
        cleaned = _strip_inline_comment(raw_line).rstrip()
        if not cleaned.strip():
            continue
        stripped = cleaned.lstrip()
        indent = len(cleaned) - len(stripped)

        if indent == 0 and stripped.startswith("repos:"):
            in_repos = True
            item_indent = None
            active_list_key = None
            continue
        if indent == 0 and not stripped.startswith("-"):
            in_repos = False
            active_list_key = None
            continue
        if not in_repos:
            continue

        if stripped.startswith("- "):
            if item_indent is None:
                item_indent = indent
            if indent > item_indent and current is not None and active_list_key is not None:
                # Multi-line list value on active_list_key
                if not isinstance(current[active_list_key], list):
                    current[active_list_key] = []
                current[active_list_key].append(_parse_scalar(stripped[2:]))
                continue

            active_list_key = None
            if current is not None:
                repos.append(current)
            current = {}
            rest = stripped[2:].strip()
            if ":" in rest:
                k, v = rest.split(":", 1)
                key = k.strip()
                val_str = v.strip()
                if val_str:
                    current[key] = _parse_scalar(val_str)
                else:
                    current[key] = []
                    active_list_key = key
        elif current is not None and ":" in stripped:
            k, v = stripped.split(":", 1)
            key = k.strip()
            val_str = v.strip()
            if val_str:
                current[key] = _parse_scalar(val_str)
                active_list_key = None
            else:
                current[key] = []
                active_list_key = key

    if current is not None:
        repos.append(current)
    return repos


def validate_no_duplicates(repos: list[dict[str, Any]]) -> None:
    seen_names: set[str] = set()
    seen_urls: dict[str, str] = {}
    for entry in repos:
        name = str(entry.get("name", "")).strip()
        url = str(entry.get("url", "")).strip()
        if not name or not url:
            raise ValueError(f"Invalid repo entry missing name or url: {entry}")
        if name in seen_names:
            raise ValueError(f"Duplicate repo name in config: {name}")
        seen_names.add(name)
        norm = normalize_git_url(url)
        if norm in seen_urls:
            raise ValueError(
                f"Duplicate repo URL detected: '{name}' ({url}) duplicates '{seen_urls[norm]}'"
            )
        seen_urls[norm] = name


def git_output(repo_dir: Path, *args: str) -> tuple[int, str, str]:
    proc = subprocess.run(
        ["git", "-C", str(repo_dir), *args],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def inspect_repo(entry: dict[str, Any], dest_root: Path) -> dict[str, Any]:
    name = str(entry["name"])
    url = str(entry["url"])
    expected_norm = normalize_git_url(url)
    target_dir = dest_root / name

    info: dict[str, Any] = {
        "name": name,
        "url": url,
        "branch": entry.get("branch", "main"),
        "path": str(target_dir),
        "os_support": entry.get("os_support", ["windows", "ubuntu"]),
        "state": "missing",
        "origin_url": None,
        "head": None,
        "dirty": None,
    }

    if not target_dir.exists():
        return info

    git_dir = target_dir / ".git"
    if not git_dir.exists():
        has_files = any(target_dir.iterdir())
        info["state"] = "non_git_nonempty" if has_files else "empty_dir"
        return info

    rc, origin_out, _ = git_output(target_dir, "remote", "get-url", "origin")
    info["origin_url"] = origin_out if rc == 0 else None
    _, head_out, _ = git_output(target_dir, "rev-parse", "--short", "HEAD")
    info["head"] = head_out or None
    _, status_out, _ = git_output(target_dir, "status", "--porcelain")
    info["dirty"] = bool(status_out)

    if rc == 0 and normalize_git_url(origin_out) == expected_norm:
        info["state"] = "cloned"
    else:
        info["state"] = "remote_mismatch"
    return info


def build_clone_command(entry: dict[str, Any], target_dir: Path) -> list[str]:
    cmd = ["git", "clone"]
    depth = entry.get("depth")
    if isinstance(depth, int) and depth > 0:
        cmd.extend(["--depth", str(depth)])
    branch = entry.get("branch")
    if branch:
        cmd.extend(["--branch", str(branch), "--single-branch"])
    if entry.get("submodules") is True:
        cmd.append("--recurse-submodules")
    cmd.extend([str(entry["url"]), str(target_dir)])
    return cmd


def sync_repo(
    entry: dict[str, Any],
    dest_root: Path,
    *,
    update: bool = False,
    dry_run: bool = False,
) -> dict[str, Any]:
    status = inspect_repo(entry, dest_root)
    target_dir = Path(status["path"])
    state = status["state"]

    if state == "cloned":
        if update:
            if status["dirty"]:
                status["action"] = "skipped_dirty"
                return status
            cmd = ["git", "-C", str(target_dir), "pull", "--ff-only"]
            env = os.environ.copy()
            if entry.get("lfs_skip_smudge"):
                env["GIT_LFS_SKIP_SMUDGE"] = "1"
            if dry_run:
                status["action"] = "dry_run_update"
                status["command"] = (
                    "GIT_LFS_SKIP_SMUDGE=1 " if entry.get("lfs_skip_smudge") else ""
                ) + " ".join(cmd)
                return status
            proc = subprocess.run(
                cmd,
                check=False,
                env=env,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            if proc.returncode != 0:
                status["action"] = "update_failed"
                status["error"] = proc.stderr.strip() or proc.stdout.strip()
                raise RuntimeError(f"Failed updating {entry['name']}: {status['error']}")
            status["action"] = "updated"
            _, head_out, _ = git_output(target_dir, "rev-parse", "--short", "HEAD")
            status["head"] = head_out or status["head"]
            return status
        status["action"] = "skipped_existing"
        return status

    if state == "remote_mismatch":
        raise RuntimeError(
            f"Repo '{entry['name']}' at {target_dir} has origin '{status['origin_url']}', "
            f"expected '{entry['url']}'."
        )

    if state == "non_git_nonempty":
        raise RuntimeError(
            f"Target directory '{target_dir}' exists and is non-empty without .git. "
            "Remove or clean it before cloning."
        )

    cmd = build_clone_command(entry, target_dir)
    env = os.environ.copy()
    if entry.get("lfs_skip_smudge"):
        env["GIT_LFS_SKIP_SMUDGE"] = "1"

    if dry_run:
        status["action"] = "dry_run_clone"
        status["command"] = ("GIT_LFS_SKIP_SMUDGE=1 " if entry.get("lfs_skip_smudge") else "") + " ".join(cmd)
        return status

    dest_root.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(cmd, check=False, env=env, text=True, encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        raise RuntimeError(f"git clone failed for {entry['name']} (exit {proc.returncode})")

    refreshed = inspect_repo(entry, dest_root)
    refreshed["action"] = "cloned"
    return refreshed


def parse_only_filter(only_args: list[str] | None) -> set[str] | None:
    if not only_args:
        return None
    selected: set[str] = set()
    for item in only_args:
        for part in item.split(","):
            name = part.strip()
            if name:
                selected.add(name)
    return selected or None


def main(argv: list[str] | None = None) -> int:
    workspace_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Idempotently clone and verify non-duplicate upstream repositories."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=workspace_root / "configs" / "repos.yaml",
        help="Path to repos.yaml (default: <workspace>/configs/repos.yaml)",
    )
    parser.add_argument(
        "--dest",
        type=Path,
        default=workspace_root / "repos",
        help="Destination folder for cloned repos (default: <workspace>/repos)",
    )
    parser.add_argument(
        "--only",
        action="append",
        help="Only operate on named repo(s); repeatable or comma-separated.",
    )
    parser.add_argument("--status", action="store_true", help="Inspect repository status without cloning.")
    parser.add_argument("--update", action="store_true", help="Fast-forward pull clean existing clones.")
    parser.add_argument("--dry-run", action="store_true", help="Print planned actions without changing disk.")
    parser.add_argument("--json", action="store_true", help="Output machine-readable JSON.")
    args = parser.parse_args(argv)

    config_path = args.config.resolve()
    dest_root = args.dest.resolve()
    if not config_path.exists():
        print(f"ERROR: Config not found: {config_path}", file=sys.stderr)
        return 2

    repos = load_repos_config(config_path)
    validate_no_duplicates(repos)

    only_set = parse_only_filter(args.only)
    if only_set is not None:
        known = {r["name"] for r in repos}
        unknown = sorted(only_set - known)
        if unknown:
            print(f"ERROR: Unknown repo name(s) in --only: {', '.join(unknown)}", file=sys.stderr)
            return 2
        repos = [r for r in repos if r["name"] in only_set]

    results: list[dict[str, Any]] = []
    exit_code = 0

    for entry in repos:
        if args.status:
            info = inspect_repo(entry, dest_root)
            info["action"] = "status"
            results.append(info)
        else:
            try:
                res = sync_repo(entry, dest_root, update=args.update, dry_run=args.dry_run)
                results.append(res)
            except RuntimeError as exc:
                results.append({"name": entry["name"], "action": "error", "error": str(exc)})
                exit_code = 1

    # Detect duplicate URLs across existing repos in dest_root
    seen_existing: dict[str, list[str]] = {}
    if dest_root.exists():
        for child in sorted(dest_root.iterdir()):
            if (child / ".git").exists():
                rc, origin, _ = git_output(child, "remote", "get-url", "origin")
                if rc == 0 and origin:
                    seen_existing.setdefault(normalize_git_url(origin), []).append(child.name)
    duplicate_dirs = {u: names for u, names in seen_existing.items() if len(names) > 1}
    if duplicate_dirs:
        exit_code = 1

    payload = {
        "config": str(config_path),
        "dest": str(dest_root),
        "duplicate_clones": duplicate_dirs,
        "repos": results,
    }

    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
    else:
        for r in results:
            if "error" in r:
                print(f"[ERROR] {r['name']}: {r['error']}")
            elif args.status:
                print(
                    f"[{r['state'].upper()}] {r['name']} -> {r['path']} "
                    f"(head={r['head']}, branch={r['branch']})"
                )
            else:
                extra = f" | cmd: {r['command']}" if "command" in r else ""
                print(f"[{r['action'].upper()}] {r['name']} ({r['state']}){extra}")
        if duplicate_dirs:
            print(f"[ERROR] Duplicate repository clones detected: {duplicate_dirs}", file=sys.stderr)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
