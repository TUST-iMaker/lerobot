"""Unit tests for EpisodeRecorder using unified data types."""

from __future__ import annotations

import json
import shutil
import tempfile
import time
import unittest
from pathlib import Path

from core.data_types import (
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    Pose3D,
    RobotAction,
)
from recording.recorder import EpisodeRecorder


class TestEpisodeRecorder(unittest.TestCase):
    """Test suite for EpisodeRecorder."""

    def setUp(self) -> None:
        self.temp_dir = tempfile.mkdtemp()
        self.recorder = EpisodeRecorder(base_dir=self.temp_dir)

    def tearDown(self) -> None:
        if self.recorder.is_recording:
            self.recorder.stop_episode()
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_start_and_stop_episode(self) -> None:
        self.assertFalse(self.recorder.is_recording)
        fpath = self.recorder.start_episode(task_name="test_pick")
        self.assertTrue(self.recorder.is_recording)
        self.assertTrue(fpath.is_file())

        # Record 2 steps
        st = JointState(timestamp=time.monotonic(), positions={"shoulder_pan": 0.1, "elbow_flex": -0.5})
        act = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.KEYBOARD,
            timestamp=time.monotonic(),
            target_joint_positions={"shoulder_pan": 0.12},
        )
        tcp = Pose3D(timestamp=time.monotonic(), position_m=(0.24, 0.0, 0.18), valid=True)

        self.recorder.record_step(st, act, tcp)
        self.recorder.record_step(st, act, tcp)
        self.assertEqual(self.recorder.current_frame_count, 2)

        summary = self.recorder.stop_episode()
        self.assertTrue(summary["success"])
        self.assertEqual(summary["total_frames"], 2)
        self.assertFalse(self.recorder.is_recording)

        # Verify JSONL lines
        lines = fpath.read_text(encoding="utf-8").strip().split("\n")
        self.assertEqual(len(lines), 4)  # metadata + 2 frames + summary
        meta = json.loads(lines[0])
        self.assertEqual(meta["type"], "episode_metadata")
        self.assertEqual(meta["task_name"], "test_pick")
        self.assertEqual(meta["env_type"], "real")
        self.assertFalse(meta["is_sim"])

        f1 = json.loads(lines[1])
        self.assertEqual(f1["type"], "frame")
        self.assertEqual(f1["env_type"], "real")
        self.assertFalse(f1["is_sim"])
        self.assertIn("observation", f1)
        self.assertIn("action", f1)
        self.assertEqual(f1["observation"]["joint_state"]["positions"]["shoulder_pan"], 0.1)

    def test_sim_vs_real_differentiation(self) -> None:
        """测试真机与仿真示教数据自动分流与元数据标记."""
        rec_real = EpisodeRecorder(base_dir=self.temp_dir, is_sim=False)
        rec_sim = EpisodeRecorder(base_dir=self.temp_dir, is_sim=True)

        self.assertEqual(rec_real.env_type, "real")
        self.assertEqual(rec_sim.env_type, "sim")
        self.assertTrue(str(rec_real.base_dir).endswith("real"))
        self.assertTrue(str(rec_sim.base_dir).endswith("sim"))

        f_real = rec_real.start_episode(task_name="task_demo")
        f_sim = rec_sim.start_episode(task_name="task_demo")

        self.assertIn("/real/task_demo/", str(f_real))
        self.assertIn("/sim/task_demo/", str(f_sim))

        rec_real.stop_episode()
        rec_sim.stop_episode()


if __name__ == "__main__":
    unittest.main()

