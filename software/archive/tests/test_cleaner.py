"""Unit tests for EpisodeCleaner and EpisodeData."""

from __future__ import annotations

import json
import shutil
import tempfile
import unittest
from pathlib import Path

from core.data_types import SO101_JOINT_NAMES
from recording.cleaner import EpisodeCleaner, EpisodeData, TrajectoryStats


class TestEpisodeCleaner(unittest.TestCase):
    """Test suite for trajectory data analysis and cleaning."""

    def setUp(self) -> None:
        self.temp_dir = tempfile.mkdtemp()
        self.sample_jsonl = Path(self.temp_dir) / "sample_episode.jsonl"
        self._create_mock_episode_file(self.sample_jsonl, total_frames=60)

    def tearDown(self) -> None:
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def _create_mock_episode_file(self, filepath: Path, total_frames: int = 60) -> None:
        """Helper to create a realistic test episode JSONL with idle deadzones."""
        metadata = {
            "type": "episode_metadata",
            "task_name": "test_cleaner_task",
            "episode_id": 1,
            "robot_type": "so_arm101",
            "env_type": "sim",
            "is_sim": True,
            "created_at": "2026-10-26T20:00:00Z",
        }

        frames = []
        # Frames 0..9: stationary (idle head)
        # Frames 10..49: moving shoulder_pan from 0.0 to 1.0 rad
        # Frames 50..59: stationary (idle tail)
        for i in range(total_frames):
            rel_t = round(i * 0.033, 3)
            if i < 10:
                pan_val = 0.0
            elif i < 50:
                pan_val = ((i - 10) / 40.0) * 1.0
            else:
                pan_val = 1.0

            # inject one intentional anomaly at frame 35 on elbow_flex
            elbow_val = 0.8 if i == 35 else 0.0

            q_pos = {j: 0.0 for j in SO101_JOINT_NAMES}
            q_pos["shoulder_pan"] = pan_val
            q_pos["elbow_flex"] = elbow_val

            frame = {
                "type": "frame",
                "frame_index": i,
                "relative_time_s": rel_t,
                "timestamp": 1000.0 + rel_t,
                "env_type": "sim",
                "is_sim": True,
                "observation": {
                    "joint_state": {
                        "positions": q_pos,
                        "unit": "rad",
                    },
                    "tcp_state": {
                        "position_m": [0.24, 0.0, 0.18],
                        "valid": True,
                    },
                },
                "action": {
                    "target_joint_positions": q_pos,
                    "source": "teleop",
                },
            }
            frames.append(frame)

        summary = {
            "type": "episode_summary",
            "task_name": "test_cleaner_task",
            "episode_id": 1,
            "env_type": "sim",
            "is_sim": True,
            "total_frames": total_frames,
            "duration_seconds": round((total_frames - 1) * 0.033, 3),
            "average_fps": 30.0,
        }

        with open(filepath, "w", encoding="utf-8") as f:
            f.write(json.dumps(metadata) + "\n")
            for frame in frames:
                f.write(json.dumps(frame) + "\n")
            f.write(json.dumps(summary) + "\n")

    def test_load_episode_from_file(self) -> None:
        """测试正确反序列化标准示教文件."""
        ep = EpisodeData.from_file(self.sample_jsonl)
        self.assertEqual(ep.frame_count, 60)
        self.assertEqual(ep.task_name, "test_cleaner_task")
        self.assertEqual(ep.episode_id, 1)
        self.assertTrue(ep.is_sim)
        self.assertEqual(ep.env_type, "sim")
        self.assertAlmostEqual(ep.duration_s, 59 * 0.033, places=2)
        self.assertGreater(ep.average_fps, 25.0)

        # Check joint values
        q0 = ep.get_joint_positions(0)
        self.assertEqual(q0["shoulder_pan"], 0.0)
        q59 = ep.get_joint_positions(59)
        self.assertEqual(q59["shoulder_pan"], 1.0)

        # Check TCP positions
        tcp = ep.get_tcp_position(0)
        self.assertEqual(len(tcp), 3)

    def test_analyze_diagnostics(self) -> None:
        """测试轨迹全维度诊断 (静止死区识别、角速度统计、突变检测)."""
        ep = EpisodeData.from_file(self.sample_jsonl)
        stats = ep.analyze(idle_velocity_threshold=0.025, anomaly_jump_threshold=0.35)

        self.assertIsInstance(stats, TrajectoryStats)
        self.assertEqual(stats.total_frames, 60)

        # Should identify idle head (~9-10 frames) and idle tail (~9-10 frames)
        self.assertGreaterEqual(stats.idle_start_frames, 8)
        self.assertGreaterEqual(stats.idle_end_frames, 8)
        self.assertLess(stats.effective_frames, stats.total_frames)

        # Max velocity on shoulder_pan should be > 0
        self.assertGreater(stats.max_joint_velocities["shoulder_pan"], 0.0)

        # TCP bounds
        bounds = stats.tcp_bounds
        self.assertIn("min_x", bounds)
        self.assertIn("max_z", bounds)

        # Anomalies: we injected a 0.8 rad jump on elbow_flex at frame 35
        self.assertGreaterEqual(stats.anomalies_count, 1)
        detail_joints = [a["joint"] for a in stats.anomalies_detail]
        self.assertIn("elbow_flex", detail_joints)

        # to_dict conversion
        d = stats.to_dict()
        self.assertEqual(d["total_frames"], 60)
        self.assertIn("effective_duration_s", d)

    def test_trim_range(self) -> None:
        """测试指定区间裁剪与时间戳重整."""
        ep = EpisodeData.from_file(self.sample_jsonl)
        trimmed = EpisodeCleaner.trim_range(ep, start_frame=15, end_frame=45)

        expected_count = 45 - 15 + 1
        self.assertEqual(trimmed.frame_count, expected_count)

        # Check frame indices are zero-based sequential
        self.assertEqual(trimmed.frames[0]["frame_index"], 0)
        self.assertEqual(trimmed.frames[-1]["frame_index"], expected_count - 1)

        # Check relative time starts at 0.0
        self.assertEqual(trimmed.frames[0]["relative_time_s"], 0.0)
        self.assertGreater(trimmed.frames[-1]["relative_time_s"], 0.0)

        # Check metadata recording
        self.assertIn("trim_source", trimmed.metadata)
        self.assertEqual(trimmed.metadata["trim_source"]["cut_start_frame"], 15)
        self.assertEqual(trimmed.metadata["trim_source"]["cut_end_frame"], 45)

    def test_auto_trim_idle(self) -> None:
        """测试自动掐头去尾去除静止死区."""
        ep = EpisodeData.from_file(self.sample_jsonl)
        cleaned, cut_s, cut_e = EpisodeCleaner.auto_trim_idle(ep, idle_velocity_threshold=0.025)

        self.assertGreater(cut_s, 5)
        self.assertLess(cut_e, 58)
        self.assertLess(cleaned.frame_count, ep.frame_count)
        self.assertEqual(cleaned.frames[0]["frame_index"], 0)
        self.assertEqual(cleaned.frames[0]["relative_time_s"], 0.0)

    def test_smooth_trajectory(self) -> None:
        """测试滑动窗口平滑滤波."""
        ep = EpisodeData.from_file(self.sample_jsonl)
        smoothed = EpisodeCleaner.smooth_trajectory(ep, window_size=5)

        self.assertEqual(smoothed.frame_count, ep.frame_count)
        self.assertTrue(smoothed.metadata.get("smoothed"))
        self.assertEqual(smoothed.metadata.get("smooth_window"), 5)

        # Spike at frame 35 on elbow_flex should be dampened
        raw_spike = ep.get_joint_positions(35)["elbow_flex"]
        smoothed_spike = smoothed.get_joint_positions(35)["elbow_flex"]
        self.assertEqual(raw_spike, 0.8)
        self.assertLess(smoothed_spike, 0.8)

    def test_save_to_file_and_reload(self) -> None:
        """测试清洗后文件保存与完整回读验证."""
        ep = EpisodeData.from_file(self.sample_jsonl)
        cleaned, _, _ = EpisodeCleaner.auto_trim_idle(ep)
        cleaned = EpisodeCleaner.smooth_trajectory(cleaned, window_size=3)

        out_path = Path(self.temp_dir) / "cleaned_output.jsonl"
        saved = EpisodeCleaner.save_to_file(cleaned, out_path)
        self.assertTrue(saved.is_file())

        # Re-read
        reloaded = EpisodeData.from_file(saved)
        self.assertEqual(reloaded.frame_count, cleaned.frame_count)
        self.assertEqual(reloaded.task_name, cleaned.task_name)
        self.assertTrue(reloaded.metadata.get("smoothed"))


if __name__ == "__main__":
    unittest.main()

