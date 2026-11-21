"""Tier 1 E2E Tests: Dual-OS Setup Scripts, Env Checker & Config Templates (F4–F6).

Total: 15 tests (5 per feature). Strictly opaque-box against M1 deliverables.
"""

from __future__ import annotations

import json
import py_compile
import shutil
import sys
from pathlib import Path

import pytest

from tests.e2e._helpers import (
    load_yaml,
    run_cmd,
    run_ubuntu_script,
    validate_powershell_ast,
    validate_shell_script_lf_and_syntax,
)

pytestmark = pytest.mark.tier1


# ---------------------------------------------------------------------------
# Feature 4: Windows Setup & Port Helper Scripts — 5 tests
# ---------------------------------------------------------------------------


def test_f4_windows_scripts_exist(workspace_root: Path) -> None:
    """T1-F04-01: Windows setup and port discovery PowerShell scripts exist."""
    assert (workspace_root / "scripts" / "windows" / "setup_win.ps1").is_file()
    assert (workspace_root / "scripts" / "windows" / "find_ports.ps1").is_file()


def test_f4_powershell_scripts_ast_syntax_valid(workspace_root: Path) -> None:
    """T1-F04-02: All scripts/windows/*.ps1 pass PowerShell AST syntax validation."""
    ps1_files = sorted((workspace_root / "scripts" / "windows").glob("*.ps1"))
    assert len(ps1_files) >= 2
    for ps1 in ps1_files:
        validate_powershell_ast(ps1)


def test_f4_setup_win_check_only_and_dry_run_succeed(workspace_root: Path) -> None:
    """T1-F04-03: setup_win.ps1 -CheckOnly and -DryRun succeed non-destructively."""
    setup_ps1 = workspace_root / "scripts" / "windows" / "setup_win.ps1"
    ps_exe = shutil.which("powershell") or shutil.which("pwsh")
    if ps_exe is None:
        text = setup_ps1.read_text(encoding="utf-8")
        assert "[switch]$CheckOnly" in text and "[switch]$DryRun" in text
        return

    res_check = run_cmd(
        [ps_exe, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(setup_ps1), "-CheckOnly"],
        cwd=workspace_root,
    )
    assert res_check.returncode == 0, f"setup_win.ps1 -CheckOnly failed: {res_check.stderr}"
    assert "Workspace Root" in res_check.stdout

    res_dry = run_cmd(
        [ps_exe, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(setup_ps1), "-DryRun"],
        cwd=workspace_root,
    )
    assert res_dry.returncode == 0, f"setup_win.ps1 -DryRun failed: {res_dry.stderr}"
    assert "[DRY-RUN]" in res_dry.stdout


def test_f4_win_venv_python_is_312(workspace_root: Path) -> None:
    """T1-F04-04: Windows uv venv uses Python 3.12.x (not 3.14)."""
    win_py = workspace_root / "envs" / "win" / "lerobot" / "Scripts" / "python.exe"
    if sys.platform == "win32":
        assert win_py.is_file(), f"Missing Windows venv Python at {win_py}"
        res = run_cmd([str(win_py), "--version"])
        assert res.returncode == 0
        assert res.stdout.strip().startswith("Python 3.12."), (
            f"Expected Python 3.12.x, got {res.stdout.strip()!r}"
        )
    else:
        text = (workspace_root / "scripts" / "windows" / "setup_win.ps1").read_text(
            encoding="utf-8"
        )
        assert '"3.12"' in text


def test_f4_find_ports_json_executes_cleanly(workspace_root: Path) -> None:
    """T1-F04-05: find_ports.ps1 -Json executes cleanly and outputs valid hardware JSON."""
    find_ps1 = workspace_root / "scripts" / "windows" / "find_ports.ps1"
    ps_exe = shutil.which("powershell") or shutil.which("pwsh")
    if ps_exe is None:
        text = find_ps1.read_text(encoding="utf-8")
        assert "active_com_names" in text and "serial_ports" in text and "cameras" in text
        return

    res = run_cmd(
        [ps_exe, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(find_ps1), "-Json"],
        cwd=workspace_root,
    )
    assert res.returncode == 0, f"find_ports.ps1 -Json failed: {res.stderr}"
    payload = json.loads(res.stdout)
    assert set(("active_com_names", "serial_ports", "cameras")).issubset(payload.keys())
    assert isinstance(payload["active_com_names"], list)
    assert isinstance(payload["serial_ports"], list)
    assert isinstance(payload["cameras"], list)


# ---------------------------------------------------------------------------
# Feature 5: Ubuntu Setup & Cross-OS Env Check — 5 tests
# ---------------------------------------------------------------------------


def test_f5_ubuntu_setup_script_exists_with_bash_syntax(workspace_root: Path) -> None:
    """T1-F05-01: setup_ubuntu.sh exists, is strictly LF, and passes bash syntax check."""
    sh_path = workspace_root / "scripts" / "ubuntu" / "setup_ubuntu.sh"
    assert sh_path.is_file(), "Missing scripts/ubuntu/setup_ubuntu.sh"
    validate_shell_script_lf_and_syntax(sh_path)


def test_f5_ubuntu_setup_supports_check_only_dry_run_help(workspace_root: Path) -> None:
    """T1-F05-02: setup_ubuntu.sh --help, --check-only, and --dry-run exit 0."""
    sh_path = workspace_root / "scripts" / "ubuntu" / "setup_ubuntu.sh"
    res_help = run_ubuntu_script(sh_path, ["--help"])
    if res_help is not None:
        assert res_help.returncode == 0, f"--help failed: {res_help.stderr}"
        for flag in ("--check-only", "--dry-run", "--with-openpi", "--setup-udev"):
            assert flag in res_help.stdout

        res_check = run_ubuntu_script(sh_path, ["--check-only"])
        assert res_check is not None and res_check.returncode == 0, (
            f"--check-only failed: {res_check.stderr}"
        )
        assert "=== TUST-iMaker LeRobot Ubuntu / WSL2 Check ===" in res_check.stdout

        res_dry = run_ubuntu_script(
            sh_path,
            ["--dry-run", "--with-openpi", "--with-core-scripts", "--with-training", "--setup-udev"],
        )
        assert res_dry is not None and res_dry.returncode == 0, (
            f"--dry-run failed: {res_dry.stderr}"
        )
        assert "[DRY-RUN]" in res_dry.stdout
    else:
        text = sh_path.read_text(encoding="utf-8")
        for flag in ("--check-only", "--dry-run", "--with-openpi", "--setup-udev"):
            assert flag in text


def test_f5_ubuntu_setup_specifies_dual_venvs_and_udev(workspace_root: Path) -> None:
    """T1-F05-03: setup_ubuntu.sh configures separate Py3.12/Py3.11 ext4 venvs and udev rules."""
    text = (workspace_root / "scripts" / "ubuntu" / "setup_ubuntu.sh").read_text(encoding="utf-8")
    assert "$HOME/.venvs/tust-lerobot" in text
    assert "$HOME/.venvs/tust-openpi" in text
    assert "--python 3.12" in text
    assert "--python 3.11" in text
    assert "/etc/udev/rules.d/99-lerobot-arms.rules" in text


def test_f5_check_env_runs_cleanly_json(workspace_root: Path) -> None:
    """T1-F05-04: scripts/common/check_env.py --json exits 0 with all 6 diagnostic sections."""
    script = workspace_root / "scripts" / "common" / "check_env.py"
    res = run_cmd([sys.executable, str(script), "--json"], cwd=workspace_root)
    assert res.returncode == 0, f"check_env.py --json failed: {res.stderr}"
    report = json.loads(res.stdout)
    assert report["overall_status"] == "OK"
    expected_checks = {
        "os_runtime",
        "cli_tools",
        "git_and_repos",
        "shell_line_endings",
        "venvs",
        "hardware_snapshot",
    }
    assert expected_checks.issubset(report["checks"].keys())
    assert report["checks"]["shell_line_endings"]["status"] == "PASS"
    assert report["checks"]["git_and_repos"]["status"] == "PASS"


def test_f5_check_env_py_passes_ast_and_strict_mode(workspace_root: Path) -> None:
    """T1-F05-05: check_env.py compiles cleanly and passes --strict mode with exit 0."""
    script = workspace_root / "scripts" / "common" / "check_env.py"
    py_compile.compile(str(script), doraise=True)
    res = run_cmd([sys.executable, str(script), "--strict"], cwd=workspace_root)
    assert res.returncode == 0, f"check_env.py --strict failed: {res.stdout}\n{res.stderr}"


# ---------------------------------------------------------------------------
# Feature 6: Versioned Robot & Training Configs — 5 tests
# ---------------------------------------------------------------------------


def test_f6_arm101_win_config_valid_schema(workspace_root: Path) -> None:
    """T1-F06-01: configs/robots/arm101_win.example.yaml uses SO-101 types & COM ports."""
    cfg = load_yaml(workspace_root / "configs" / "robots" / "arm101_win.example.yaml")
    assert cfg["robot"]["type"] == "so101_follower"
    assert str(cfg["robot"]["port"]).startswith("COM")
    assert cfg["teleop"]["type"] == "so101_leader"
    assert str(cfg["teleop"]["port"]).startswith("COM")
    assert cfg["robot"]["calibration_dir"].startswith("data/calibration/")
    assert set(cfg["robot"]["cameras"].keys()) == {"front", "wrist"}
    assert cfg["display_mode"] in ("rerun", "foxglove")


def test_f6_arm101_ubuntu_config_valid_schema(workspace_root: Path) -> None:
    """T1-F06-02: configs/robots/arm101_ubuntu.example.yaml uses SO-101 & /dev/ttyACM*."""
    cfg = load_yaml(workspace_root / "configs" / "robots" / "arm101_ubuntu.example.yaml")
    assert cfg["robot"]["type"] == "so101_follower"
    assert str(cfg["robot"]["port"]).startswith("/dev/tty")
    assert cfg["teleop"]["type"] == "so101_leader"
    assert str(cfg["teleop"]["port"]).startswith("/dev/tty")
    assert cfg["robot"]["calibration_dir"].startswith("data/calibration/")
    assert set(cfg["robot"]["cameras"].keys()) == {"front", "wrist"}


def test_f6_train_act_and_smolvla_configs_valid(workspace_root: Path) -> None:
    """T1-F06-03: act_so101.yaml and smolvla_so101_cloud.yaml conform to their respective schemas."""
    act_cfg = load_yaml(workspace_root / "configs" / "train" / "act_so101.yaml")
    assert act_cfg["policy"]["type"] == "act"
    assert act_cfg["dataset"]["repo_id"]
    assert act_cfg["output_dir"].startswith("models/checkpoints/")

    smol_cfg = load_yaml(workspace_root / "configs" / "train" / "smolvla_so101_cloud.yaml")
    # SmolVLA loads via policy.path and intentionally omits policy.type
    assert smol_cfg["policy"]["path"] == "lerobot/smolvla_base"
    assert "type" not in smol_cfg["policy"]
    assert "rename_map" in smol_cfg
    assert smol_cfg["output_dir"].startswith("models/checkpoints/")


def test_f6_train_pi0_config_valid(workspace_root: Path) -> None:
    """T1-F06-04: pi0_so101_cloud.yaml specifies pi0 policy, pretrained_path, and bfloat16."""
    pi0_cfg = load_yaml(workspace_root / "configs" / "train" / "pi0_so101_cloud.yaml")
    assert pi0_cfg["policy"]["type"] == "pi0"
    assert pi0_cfg["policy"]["pretrained_path"] == "lerobot/pi0_base"
    assert pi0_cfg["policy"]["dtype"] == "bfloat16"
    assert pi0_cfg["dataset"]["repo_id"]
    assert pi0_cfg["output_dir"].startswith("models/checkpoints/")


def test_f6_train_pi05_openpi_config_valid(workspace_root: Path) -> None:
    """T1-F06-05: pi05_openpi_cloud.yaml provides OpenPI TrainConfig + lerobot-train pi05 ref."""
    path = workspace_root / "configs" / "train" / "pi05_openpi_cloud.yaml"
    raw = path.read_text(encoding="utf-8")
    cfg = load_yaml(path)
    openpi_cfg = cfg["openpi_train_config"]
    assert openpi_cfg["model"]["pi05"] is True
    assert openpi_cfg["data"]["base_config"]["use_quantile_norm"] is True
    assert openpi_cfg["weight_loader"]["params_path"].startswith("gs://openpi-assets/")
    assert "lerobot-train" in raw and "--policy.type=pi05" in raw
