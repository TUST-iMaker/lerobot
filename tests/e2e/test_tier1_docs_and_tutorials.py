"""Tier 1 E2E Tests: Chinese Documentation, Tutorials & Frontend Survey (F7–F15).

Total: 45 tests (5 per feature). Strictly opaque-box against PROJECT.md & TEST_INFRA.md.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.e2e._helpers import contains_chinese

pytestmark = pytest.mark.tier1


def _read_doc(workspace_root: Path, rel_path: str) -> str:
    path = workspace_root / "docs" / rel_path
    assert path.is_file(), f"Missing documentation file: docs/{rel_path}"
    text = path.read_text(encoding="utf-8")
    assert len(text.strip()) >= 200, f"Documentation file too short / stubbed: docs/{rel_path}"
    return text


# ---------------------------------------------------------------------------
# Feature 7: Tutorial Source & Citation Registry (docs/sources/README.md) — 5 tests
# ---------------------------------------------------------------------------


def test_f7_sources_readme_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F07-01: docs/sources/README.md exists and is written in UTF-8 Chinese."""
    text = _read_doc(workspace_root, "sources/README.md")
    assert contains_chinese(text)


def test_f7_sources_cites_core_github_repos(workspace_root: Path) -> None:
    """T1-F07-02: docs/sources/README.md cites TUST-iMaker/lerobot, lerobot, openpi, SO-ARM100."""
    text = _read_doc(workspace_root, "sources/README.md")
    for ref in (
        "TUST-iMaker/lerobot",
        "huggingface/lerobot",
        "Physical-Intelligence/openpi",
        "TheRobotStudio/SO-ARM100",
    ):
        assert ref in text, f"docs/sources/README.md missing GitHub citation: {ref}"


def test_f7_sources_cites_hf_docs_and_seeed_wiki(workspace_root: Path) -> None:
    """T1-F07-03: docs/sources/README.md cites Hugging Face LeRobot docs and Seeed Studio Wiki."""
    text = _read_doc(workspace_root, "sources/README.md")
    assert "huggingface.co/docs/lerobot" in text or "huggingface.co" in text
    assert "wiki.seeedstudio.com" in text or "Seeed" in text


def test_f7_sources_cites_arxiv_pi_papers(workspace_root: Path) -> None:
    """T1-F07-04: docs/sources/README.md cites arXiv / Physical Intelligence papers (incl. pi0.7)."""
    text = _read_doc(workspace_root, "sources/README.md")
    assert "arxiv.org" in text.lower()
    assert "pi0.7" in text.lower() or "2604.15483" in text


def test_f7_sources_records_feishu_wiki_status(workspace_root: Path) -> None:
    """T1-F07-05: docs/sources/README.md records Feishu wiki URL and actual access status."""
    text = _read_doc(workspace_root, "sources/README.md")
    assert "TS6swApHbinx01kHDi5cf5n5n8c" in text
    assert "feishu.cn" in text
    # Must explicitly document retrieval status (e.g., 302 / login / access state)
    assert any(
        kw in text.lower()
        for kw in ("302", "登录", "login", "passport", "无法直接", "playwright", "访问")
    )


# ---------------------------------------------------------------------------
# Feature 8: Dual-OS Environment Setup Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f8_dual_os_setup_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F08-01: docs/01_environment/dual_os_setup.md exists in UTF-8 Chinese."""
    text = _read_doc(workspace_root, "01_environment/dual_os_setup.md")
    assert contains_chinese(text)


def test_f8_dual_os_covers_windows_uv_and_scripts(workspace_root: Path) -> None:
    """T1-F08-02: Dual-OS guide covers Windows uv setup, envs/win/lerobot, and setup_win.ps1."""
    text = _read_doc(workspace_root, "01_environment/dual_os_setup.md")
    for tok in ("uv", "envs/win/lerobot", "setup_win.ps1"):
        assert tok in text, f"dual_os_setup.md missing Windows setup token: {tok}"


def test_f8_dual_os_covers_ubuntu_ext4_venvs_and_script(workspace_root: Path) -> None:
    """T1-F08-03: Dual-OS guide covers Ubuntu/WSL2 ext4 ~/.venvs/ and setup_ubuntu.sh."""
    text = _read_doc(workspace_root, "01_environment/dual_os_setup.md")
    for tok in ("setup_ubuntu.sh", "tust-lerobot", "tust-openpi", "ext4"):
        assert tok in text, f"dual_os_setup.md missing Ubuntu setup token: {tok}"


def test_f8_dual_os_covers_serial_ports_udev_and_dialout(workspace_root: Path) -> None:
    """T1-F08-04: Dual-OS guide explains COM* vs /dev/ttyACM*, udev rules, and dialout group."""
    text = _read_doc(workspace_root, "01_environment/dual_os_setup.md")
    for tok in ("COM", "/dev/ttyACM", "udev", "dialout"):
        assert tok in text, f"dual_os_setup.md missing serial/udev token: {tok}"


def test_f8_dual_os_covers_camera_discovery_both_oses(workspace_root: Path) -> None:
    """T1-F08-05: Dual-OS guide documents DirectShow vs V4L2 camera probing."""
    text = _read_doc(workspace_root, "01_environment/dual_os_setup.md")
    assert "find_ports.ps1" in text
    assert "lerobot-find-cameras" in text
    assert "DirectShow" in text or "DSHOW" in text
    assert "V4L2" in text or "/dev/video" in text


# ---------------------------------------------------------------------------
# Feature 9: arm101 Brief Prep & Calibration Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f9_calibration_doc_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F09-01: docs/02_arm101_workflow/01_calibration_and_ports.md exists in Chinese."""
    text = _read_doc(workspace_root, "02_arm101_workflow/01_calibration_and_ports.md")
    assert contains_chinese(text)


def test_f9_calibration_notes_hardware_assembly_briefly(workspace_root: Path) -> None:
    """T1-F09-02: Calibration guide notes hardware assembly as brief prerequisite, cites SO-ARM100."""
    text = _read_doc(workspace_root, "02_arm101_workflow/01_calibration_and_ports.md")
    assert "SO-ARM100" in text or "arm101" in text.lower()
    assert any(kw in text for kw in ("组装", "装配", "硬件安装", "已完成"))


def test_f9_calibration_covers_port_discovery_tools(workspace_root: Path) -> None:
    """T1-F09-03: Calibration guide documents lerobot-find-port and find_ports.ps1."""
    text = _read_doc(workspace_root, "02_arm101_workflow/01_calibration_and_ports.md")
    assert "lerobot-find-port" in text
    assert "find_ports.ps1" in text


def test_f9_calibration_covers_setup_motors(workspace_root: Path) -> None:
    """T1-F09-04: Calibration guide documents lerobot-setup-motors for STS3215 motor IDs."""
    text = _read_doc(workspace_root, "02_arm101_workflow/01_calibration_and_ports.md")
    assert "lerobot-setup-motors" in text


def test_f9_calibration_covers_so101_calibrate_commands(workspace_root: Path) -> None:
    """T1-F09-05: Calibration guide documents lerobot-calibrate for follower & leader."""
    text = _read_doc(workspace_root, "02_arm101_workflow/01_calibration_and_ports.md")
    assert "lerobot-calibrate" in text
    assert "so101_follower" in text
    assert "so101_leader" in text


# ---------------------------------------------------------------------------
# Feature 10: arm101 Teleoperation Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f10_teleop_doc_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F10-01: docs/02_arm101_workflow/02_teleoperation.md exists in Chinese."""
    text = _read_doc(workspace_root, "02_arm101_workflow/02_teleoperation.md")
    assert contains_chinese(text)


def test_f10_teleop_covers_camera_discovery(workspace_root: Path) -> None:
    """T1-F10-02: Teleoperation guide documents lerobot-find-cameras."""
    text = _read_doc(workspace_root, "02_arm101_workflow/02_teleoperation.md")
    assert "lerobot-find-cameras" in text


def test_f10_teleop_covers_lerobot_teleoperate_so101(workspace_root: Path) -> None:
    """T1-F10-03: Teleoperation guide documents lerobot-teleoperate with so101_follower/leader."""
    text = _read_doc(workspace_root, "02_arm101_workflow/02_teleoperation.md")
    assert "lerobot-teleoperate" in text
    assert "so101_follower" in text
    assert "so101_leader" in text


def test_f10_teleop_covers_rerun_live_visualization(workspace_root: Path) -> None:
    """T1-F10-04: Teleoperation guide documents Rerun live visualization flags."""
    text = _read_doc(workspace_root, "02_arm101_workflow/02_teleoperation.md")
    assert "rerun" in text.lower()
    assert "display_data" in text


def test_f10_teleop_covers_win_ubuntu_config_parity(workspace_root: Path) -> None:
    """T1-F10-05: Teleoperation guide references both arm101_win and arm101_ubuntu templates."""
    text = _read_doc(workspace_root, "02_arm101_workflow/02_teleoperation.md")
    assert "arm101_win.example.yaml" in text
    assert "arm101_ubuntu.example.yaml" in text


# ---------------------------------------------------------------------------
# Feature 11: arm101 Teaching / Demo Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f11_teaching_doc_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F11-01: docs/02_arm101_workflow/03_teaching_and_demo.md exists in Chinese."""
    text = _read_doc(workspace_root, "02_arm101_workflow/03_teaching_and_demo.md")
    assert contains_chinese(text)


def test_f11_teaching_explains_demonstration_workflow(workspace_root: Path) -> None:
    """T1-F11-02: Teaching guide explains leader-follower demonstration and teaching workflow."""
    text = _read_doc(workspace_root, "02_arm101_workflow/03_teaching_and_demo.md")
    assert any(kw in text for kw in ("示教", "演示"))
    assert "so101_follower" in text


def test_f11_teaching_documents_lerobot_replay(workspace_root: Path) -> None:
    """T1-F11-03: Teaching guide documents open-loop trajectory verification via lerobot-replay."""
    text = _read_doc(workspace_root, "02_arm101_workflow/03_teaching_and_demo.md")
    assert "lerobot-replay" in text


def test_f11_teaching_documents_safety_precautions(workspace_root: Path) -> None:
    """T1-F11-04: Teaching guide covers safety limits such as max_relative_target or torque stop."""
    text = _read_doc(workspace_root, "02_arm101_workflow/03_teaching_and_demo.md")
    assert "max_relative_target" in text or "disable_torque_on_disconnect" in text
    assert any(kw in text for kw in ("安全", "急停", "碰撞", "限位"))


def test_f11_teaching_links_studio_or_hil_rollout(workspace_root: Path) -> None:
    """T1-F11-05: Teaching guide connects to Arm101 Studio waypoint replay or rollout."""
    text = _read_doc(workspace_root, "02_arm101_workflow/03_teaching_and_demo.md")
    assert any(kw in text.lower() for kw in ("studio", "waypoint", "轨迹点", "lerobot-rollout"))


# ---------------------------------------------------------------------------
# Feature 12: arm101 Data Collection & Editing Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f12_data_collection_doc_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F12-01: docs/02_arm101_workflow/04_data_collection.md exists in Chinese."""
    text = _read_doc(workspace_root, "02_arm101_workflow/04_data_collection.md")
    assert contains_chinese(text)


def test_f12_data_collection_covers_lerobot_record_v3(workspace_root: Path) -> None:
    """T1-F12-02: Data collection guide documents lerobot-record and LeRobot v3.0 dataset format."""
    text = _read_doc(workspace_root, "02_arm101_workflow/04_data_collection.md")
    assert "lerobot-record" in text
    assert "v3" in text.lower()


def test_f12_data_collection_covers_no_stamp_and_resume(workspace_root: Path) -> None:
    """T1-F12-03: Data collection guide covers --dataset.no_stamp / no_stamp and --resume."""
    text = _read_doc(workspace_root, "02_arm101_workflow/04_data_collection.md")
    assert "no_stamp" in text
    assert "resume" in text


def test_f12_data_collection_covers_dataset_viz(workspace_root: Path) -> None:
    """T1-F12-04: Data collection guide documents lerobot-dataset-viz."""
    text = _read_doc(workspace_root, "02_arm101_workflow/04_data_collection.md")
    assert "lerobot-dataset-viz" in text


def test_f12_data_collection_covers_edit_dataset_recompute_stats(workspace_root: Path) -> None:
    """T1-F12-05: Data collection guide documents lerobot-edit-dataset and recompute_stats."""
    text = _read_doc(workspace_root, "02_arm101_workflow/04_data_collection.md")
    assert "lerobot-edit-dataset" in text
    assert "recompute_stats" in text


# ---------------------------------------------------------------------------
# Feature 13: VLA Data Prep & Cloud GPU Training Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f13_vla_cloud_doc_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F13-01: docs/03_vla_and_pi/01_vla_data_and_cloud_training.md exists in Chinese."""
    text = _read_doc(workspace_root, "03_vla_and_pi/01_vla_data_and_cloud_training.md")
    assert contains_chinese(text)


def test_f13_vla_cloud_covers_dataset_prep_and_normalization(workspace_root: Path) -> None:
    """T1-F13-02: VLA training guide covers dataset prep, stats/quantiles, rename_map, HF Hub."""
    text = _read_doc(workspace_root, "03_vla_and_pi/01_vla_data_and_cloud_training.md")
    assert "rename_map" in text
    assert "stats.json" in text or "q01" in text or "quantile" in text.lower()
    assert "huggingface" in text.lower() or "hf" in text.lower()


def test_f13_vla_cloud_covers_cloud_gpu_rental_platforms(workspace_root: Path) -> None:
    """T1-F13-03: VLA training guide covers cloud GPU platforms (HF Jobs, Featurize, AutoDL)."""
    text = _read_doc(workspace_root, "03_vla_and_pi/01_vla_data_and_cloud_training.md")
    assert "AutoDL" in text
    assert "Featurize" in text
    assert "Hugging Face Jobs" in text or "HF Jobs" in text or "--job.target" in text


def test_f13_vla_cloud_covers_lerobot_train_and_multi_gpu(workspace_root: Path) -> None:
    """T1-F13-04: VLA training guide documents lerobot-train and multi-GPU / accelerate launch."""
    text = _read_doc(workspace_root, "03_vla_and_pi/01_vla_data_and_cloud_training.md")
    assert "lerobot-train" in text
    assert "accelerate" in text.lower() or "multi" in text.lower() or "fsdp" in text.lower()


def test_f13_vla_cloud_covers_checkpoints_and_eval(workspace_root: Path) -> None:
    """T1-F13-05: VLA training guide documents checkpoints and evaluation/rollout."""
    text = _read_doc(workspace_root, "03_vla_and_pi/01_vla_data_and_cloud_training.md")
    assert "models/checkpoints" in text or "checkpoint" in text.lower()
    assert "lerobot-eval" in text or "lerobot-rollout" in text or "lerobot-policy-server" in text


# ---------------------------------------------------------------------------
# Feature 14: pi0.7 Fact-Check & pi0/pi0.5/openpi Guide — 5 tests
# ---------------------------------------------------------------------------


def test_f14_pi_series_doc_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F14-01: docs/03_vla_and_pi/02_pi_series_and_pi07.md exists in Chinese."""
    text = _read_doc(workspace_root, "03_vla_and_pi/02_pi_series_and_pi07.md")
    assert contains_chinese(text)


def test_f14_pi_series_documents_pi07_arxiv_factually(workspace_root: Path) -> None:
    """T1-F14-02: pi0.7 section cites arXiv:2604.15483v2 / Physical Intelligence accurately."""
    text = _read_doc(workspace_root, "03_vla_and_pi/02_pi_series_and_pi07.md")
    assert "pi0.7" in text.lower() or "π0.7" in text
    assert "2604.15483" in text or "Physical Intelligence" in text


def test_f14_pi_series_states_pi07_is_closed_weight(workspace_root: Path) -> None:
    """T1-F14-03: pi0.7 section explicitly states weights/code are NOT publicly open-sourced."""
    text = _read_doc(workspace_root, "03_vla_and_pi/02_pi_series_and_pi07.md")
    assert any(
        kw in text
        for kw in ("未开源", "未公开发布", "闭源", "暂未开放", "没有公开权重", "未发布权重")
    )


def test_f14_pi_series_covers_runnable_lerobot_vla_pipelines(workspace_root: Path) -> None:
    """T1-F14-04: Guide covers runnable pi0, pi0_fast, pi05, and smolvla pipelines."""
    text = _read_doc(workspace_root, "03_vla_and_pi/02_pi_series_and_pi07.md")
    for policy in ("pi0", "pi0_fast", "pi05", "smolvla"):
        assert policy in text.lower(), f"Missing runnable VLA pipeline: {policy}"


def test_f14_pi_series_covers_openpi_ubuntu_workflow(workspace_root: Path) -> None:
    """T1-F14-05: Guide documents Physical-Intelligence/openpi Ubuntu scripts and VRAM limits."""
    text = _read_doc(workspace_root, "03_vla_and_pi/02_pi_series_and_pi07.md")
    for tok in ("serve_policy.py", "compute_norm_stats.py", "openpi"):
        assert tok in text, f"Missing openpi workflow detail: {tok}"
    assert "Ubuntu" in text or "Linux" in text


# ---------------------------------------------------------------------------
# Feature 15: Open-Source Frontend Survey & Integration Doc — 5 tests
# ---------------------------------------------------------------------------


def test_f15_frontend_survey_exists_and_chinese(workspace_root: Path) -> None:
    """T1-F15-01: docs/04_frontend_and_tools/open_source_frontend_survey.md exists in Chinese."""
    text = _read_doc(workspace_root, "04_frontend_and_tools/open_source_frontend_survey.md")
    assert contains_chinese(text)


def test_f15_frontend_survey_evaluates_core_visualizers_and_studios(workspace_root: Path) -> None:
    """T1-F15-02: Survey compares leLab, rerun, and lerobot-studio."""
    text = _read_doc(workspace_root, "04_frontend_and_tools/open_source_frontend_survey.md")
    low = text.lower()
    for tool in ("lelab", "rerun", "lerobot-studio"):
        assert tool in low, f"Frontend survey missing evaluation of {tool}"


def test_f15_frontend_survey_evaluates_servo_and_ecosystem_tools(workspace_root: Path) -> None:
    """T1-F15-03: Survey evaluates feetech-motorlab, sts-studio, phosphobot, and solo-cli."""
    text = _read_doc(workspace_root, "04_frontend_and_tools/open_source_frontend_survey.md")
    low = text.lower()
    for tool in ("feetech-motorlab", "sts-studio", "phosphobot", "solo-cli"):
        assert tool in low, f"Frontend survey missing evaluation of {tool}"


def test_f15_frontend_survey_defines_adapter_and_isolation_rules(workspace_root: Path) -> None:
    """T1-F15-04: Survey defines clean ToolAdapter / dependency-isolation integration rules."""
    text = _read_doc(workspace_root, "04_frontend_and_tools/open_source_frontend_survey.md")
    assert "ToolAdapter" in text or "适配器" in text
    assert any(kw in text for kw in ("隔离", "子进程", "repos/"))


def test_f15_frontend_survey_rejects_legacy_nonstandard_structure(workspace_root: Path) -> None:
    """T1-F15-05: Survey explicitly rejects copying unstandardized legacy tool layouts."""
    text = _read_doc(workspace_root, "04_frontend_and_tools/open_source_frontend_survey.md")
    assert any(kw in text for kw in ("不沿用", "旧结构", "重构", "规范结构", "禁止"))
