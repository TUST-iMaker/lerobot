"""Tier 1 E2E Tests: Workspace Bootstrap, Non-Duplicate Repos & Directory Layout (F1–F3).

Total: 15 tests (5 per feature). Strictly opaque-box against M1 deliverables.
"""

from __future__ import annotations

import json
import sys
import tomllib
from pathlib import Path

import pytest

from tests.e2e._helpers import (
    contains_chinese,
    load_yaml,
    normalize_git_url,
    run_cmd,
    run_git,
)

pytestmark = pytest.mark.tier1


# ---------------------------------------------------------------------------
# Feature 1: Root Repo Init (TUST-iMaker/lerobot) — 5 tests
# ---------------------------------------------------------------------------


def test_f1_root_git_origin_is_tust_imaker(workspace_root: Path) -> None:
    """T1-F01-01: Root workspace is a git repo whose origin is TUST-iMaker/lerobot."""
    assert (workspace_root / ".git").exists(), "Workspace root must contain .git"
    res = run_git(workspace_root, "remote", "get-url", "origin")
    assert res.returncode == 0, f"git remote get-url origin failed: {res.stderr}"
    assert normalize_git_url(res.stdout) == "github.com/tust-imaker/lerobot", (
        f"Unexpected root origin: {res.stdout!r}"
    )


def test_f1_gitignore_covers_generated_and_local_dirs(workspace_root: Path) -> None:
    """T1-F01-02: .gitignore excludes repos, envs, local data/models/logs, .env, personal robot yamls."""
    gitignore = workspace_root / ".gitignore"
    assert gitignore.is_file(), "Missing root .gitignore"
    text = gitignore.read_text(encoding="utf-8")
    required_patterns = (
        "repos/",
        "envs/",
        ".env",
        "configs/robots/*.yaml",
        "!configs/robots/*.example.yaml",
        "data/*",
        "models/*",
        "logs/*",
    )
    for pat in required_patterns:
        assert pat in text, f".gitignore missing required rule: {pat!r}"


def test_f1_gitattributes_enforces_lf_for_shell_scripts(workspace_root: Path) -> None:
    """T1-F01-03: .gitattributes enforces LF line endings for shell scripts and cross-OS files."""
    gitattr = workspace_root / ".gitattributes"
    assert gitattr.is_file(), "Missing root .gitattributes"
    text = gitattr.read_text(encoding="utf-8")
    assert "*.sh text eol=lf" in text
    assert "*.ps1 text eol=crlf" in text


def test_f1_env_example_defines_cross_os_keys(workspace_root: Path) -> None:
    """T1-F01-04: .env.example defines required dual-OS venv, arm101, and Studio keys."""
    env_example = workspace_root / ".env.example"
    assert env_example.is_file(), "Missing root .env.example"
    text = env_example.read_text(encoding="utf-8")
    parsed_keys: set[str] = set()
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        parsed_keys.add(stripped.split("=", 1)[0].strip())

    required_keys = {
        "LEROBOT_WIN_VENV",
        "LEROBOT_UBUNTU_VENV",
        "OPENPI_UBUNTU_VENV",
        "HF_LEROBOT_HOME",
        "ARM101_FOLLOWER_PORT_WIN",
        "ARM101_LEADER_PORT_WIN",
        "ARM101_FOLLOWER_PORT_UBUNTU",
        "ARM101_LEADER_PORT_UBUNTU",
        "STUDIO_HOST",
        "STUDIO_PORT",
        "STUDIO_MODE",
    }
    missing = required_keys - parsed_keys
    assert not missing, f".env.example missing keys: {sorted(missing)}"


def test_f1_root_pyproject_declares_py312_and_studio_deps(workspace_root: Path) -> None:
    """T1-F01-05: pyproject.toml parses cleanly with >=3.12 and core Studio dependencies."""
    pyproject = workspace_root / "pyproject.toml"
    assert pyproject.is_file(), "Missing root pyproject.toml"
    data = tomllib.loads(pyproject.read_text(encoding="utf-8"))
    project = data["project"]
    assert project["requires-python"] == ">=3.12"
    deps_joined = "\n".join(project.get("dependencies", []))
    for pkg in ("fastapi", "uvicorn", "websockets", "pyserial", "pyyaml", "pydantic"):
        assert pkg in deps_joined, f"pyproject.toml missing dependency: {pkg}"
    testpaths = data.get("tool", {}).get("pytest", {}).get("ini_options", {}).get("testpaths", [])
    assert "tests" in testpaths and "app/tests" in testpaths


# ---------------------------------------------------------------------------
# Feature 2: Non-Duplicate Upstream Repo Cloning — 5 tests
# ---------------------------------------------------------------------------


def test_f2_repos_yaml_lists_five_required_upstreams(workspace_root: Path) -> None:
    """T1-F02-01: configs/repos.yaml registers all 5 required upstream repositories."""
    repos_yaml = workspace_root / "configs" / "repos.yaml"
    assert repos_yaml.is_file(), "Missing configs/repos.yaml"
    data = load_yaml(repos_yaml)
    entries = {item["name"]: item for item in data.get("repos", [])}
    expected = {
        "lerobot": "github.com/huggingface/lerobot",
        "leLab": "github.com/huggingface/lelab",
        "openpi": "github.com/physical-intelligence/openpi",
        "SO-ARM100": "github.com/therobotstudio/so-arm100",
        "feetech-motorlab": "github.com/nori-robotics/feetech-motorlab",
    }
    assert set(entries.keys()) == set(expected.keys())
    for name, norm_url in expected.items():
        assert normalize_git_url(entries[name]["url"]) == norm_url
    assert "ubuntu" in entries["openpi"]["os_support"]
    assert "windows" not in entries["openpi"]["os_support"]


def test_f2_all_five_repos_cloned_with_matching_origins(workspace_root: Path) -> None:
    """T1-F02-02: Every repo in configs/repos.yaml is cloned under repos/ with matching origin."""
    data = load_yaml(workspace_root / "configs" / "repos.yaml")
    for entry in data["repos"]:
        name = entry["name"]
        repo_dir = workspace_root / "repos" / name
        assert (repo_dir / ".git").exists(), f"Missing cloned git repo at {repo_dir}"
        res = run_git(repo_dir, "remote", "get-url", "origin")
        assert res.returncode == 0, f"Failed to read git origin for {name}: {res.stderr}"
        assert normalize_git_url(res.stdout) == normalize_git_url(entry["url"]), (
            f"Origin mismatch for {name}: {res.stdout} != {entry['url']}"
        )


def test_f2_no_duplicate_repo_clones_in_workspace(workspace_root: Path) -> None:
    """T1-F02-03: bootstrap_repos.py --status --json reports zero duplicate clones."""
    script = workspace_root / "scripts" / "common" / "bootstrap_repos.py"
    res = run_cmd([sys.executable, str(script), "--status", "--json"], cwd=workspace_root)
    assert res.returncode == 0, f"bootstrap_repos.py --status --json failed: {res.stderr}"
    payload = json.loads(res.stdout)
    assert payload["duplicate_clones"] == {}
    assert len(payload["repos"]) == 5
    assert all(r["state"] == "cloned" for r in payload["repos"])


def test_f2_bootstrap_repos_status_and_dry_run_exit_zero(workspace_root: Path) -> None:
    """T1-F02-04: bootstrap_repos.py --dry-run runs idempotently with exit 0 on cloned repos."""
    script = workspace_root / "scripts" / "common" / "bootstrap_repos.py"
    res = run_cmd([sys.executable, str(script), "--dry-run", "--json"], cwd=workspace_root)
    assert res.returncode == 0, f"bootstrap_repos.py --dry-run failed: {res.stderr}"
    payload = json.loads(res.stdout)
    assert [r["action"] for r in payload["repos"]] == ["skipped_existing"] * 5


def test_f2_lerobot_editable_installed_in_venv(workspace_root: Path) -> None:
    """T1-F02-05: Active Python environment resolves `lerobot` from repos/lerobot."""
    probe = run_cmd(
        [sys.executable, "-c", "import lerobot; print(lerobot.__file__)"],
        cwd=workspace_root,
    )
    assert probe.returncode == 0, f"Failed importing lerobot: {probe.stderr}"
    lerobot_path = Path(probe.stdout.strip()).resolve()
    expected_parent = (workspace_root / "repos" / "lerobot").resolve()
    assert lerobot_path.is_relative_to(expected_parent), (
        f"lerobot imported from {lerobot_path}, expected inside {expected_parent}"
    )


# ---------------------------------------------------------------------------
# Feature 3: Extensible Directory Layout & Root README.md — 5 tests
# ---------------------------------------------------------------------------


def test_f3_top_level_directories_present(workspace_root: Path) -> None:
    """T1-F03-01: All standardized top-level directories exist."""
    expected_dirs = (
        "docs",
        "scripts",
        "configs",
        "app",
        "repos",
        "envs",
        "data",
        "models",
        "logs",
        "tests",
    )
    for d in expected_dirs:
        assert (workspace_root / d).is_dir(), f"Missing top-level directory: {d}/"


def test_f3_data_models_logs_subfolders_and_readmes_present(workspace_root: Path) -> None:
    """T1-F03-02: data/, models/, logs/ subfolders and README.md files exist."""
    for sub in (
        "data/datasets",
        "data/calibration",
        "data/dry_run",
        "models/pretrained",
        "models/checkpoints",
    ):
        assert (workspace_root / sub).is_dir(), f"Missing subdirectory: {sub}"
    for readme_rel in ("data/README.md", "models/README.md", "logs/README.md"):
        p = workspace_root / readme_rel
        assert p.is_file(), f"Missing {readme_rel}"
        assert contains_chinese(p.read_text(encoding="utf-8")), f"{readme_rel} should be Chinese"


def test_f3_root_readme_documents_layout_and_extension_rules(workspace_root: Path) -> None:
    """T1-F03-03: Root README.md documents all top-level dirs and extension conventions."""
    readme = workspace_root / "README.md"
    assert readme.is_file(), "Missing root README.md"
    text = readme.read_text(encoding="utf-8")
    assert contains_chinese(text)
    for token in (
        "TUST-iMaker/lerobot",
        "docs/",
        "scripts/",
        "configs/",
        "app/",
        "repos/",
        "envs/",
        "data/",
        "models/",
        "logs/",
        "tests/",
        "configs/repos.yaml",
    ):
        assert token in text, f"Root README.md missing directory/extension reference: {token}"


def test_f3_root_readme_documents_dual_os_synergy(workspace_root: Path) -> None:
    """T1-F03-04: Root README.md explains Windows vs Ubuntu/WSL2 division of labor and venvs."""
    text = (workspace_root / "README.md").read_text(encoding="utf-8")
    for token in (
        "Windows",
        "Ubuntu",
        "envs/win/lerobot",
        "~/.venvs/tust-lerobot",
        "~/.venvs/tust-openpi",
        "COM*",
        "/dev/ttyACM*",
        "99-lerobot-arms.rules",
    ):
        assert token in text, f"Root README.md missing dual-OS guidance: {token}"


def test_f3_root_readme_documents_quickstart_commands(workspace_root: Path) -> None:
    """T1-F03-05: Root README.md documents bootstrap, setup, check_env, and Studio entrypoints."""
    text = (workspace_root / "README.md").read_text(encoding="utf-8")
    for cmd_token in (
        "bootstrap_repos.py",
        "setup_win.ps1",
        "find_ports.ps1",
        "setup_ubuntu.sh",
        "check_env.py",
        "app.backend.main",
        "run_studio.py",
    ):
        assert cmd_token in text, f"Root README.md missing quickstart command: {cmd_token}"
