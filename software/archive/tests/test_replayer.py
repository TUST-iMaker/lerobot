"""Unit tests for EpisodeReplayer."""

from __future__ import annotations

import json
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from core.action_api import ActionAPI
from core.data_types import (
    SO101_JOINT_NAMES,
    ControlSource,
    JointState,
    JointUnit,
    SystemState,
)
from core.safety_controller import SafetyController
from core.state_machine import RobotStateMachine
from hardware.system_adapter import RobotSystemAdapter
from recording.cleaner import EpisodeData
from recording.replayer import EpisodeReplayer


class TestEpisodeReplayer(unittest.TestCase):
    """Test suite for offline episode replayer."""

    def setUp(self) -> None:
        self.temp_dir = tempfile.mkdtemp()
        self.sample_jsonl = Path(self.temp_dir) / "test_replay.jsonl"
        self._create_mock_episode_file(self.sample_jsonl, total_frames=20)

        # Mock Hardware & API
        self.mock_hw = MagicMock(spec=RobotSystemAdapter)
        self.mock_hw.is_follower_connected = True
        self.mock_hw.is_leader_connected = False
        self.mock_hw.leader = None
        self.mock_hw.read_follower_state.return_value = JointState(
            timestamp=time.monotonic(),
            positions={j: 0.0 for j in SO101_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        self.mock_hw.send_action.return_value = True

        self.sm = RobotStateMachine(initial_state=SystemState.IDLE)
        self.safety = SafetyController(self.sm)
        self.api = ActionAPI(
            hardware=self.mock_hw,
            state_machine=self.sm,
            safety_controller=self.safety,
        )

        self.replayer = EpisodeReplayer(action_api=self.api)

    def tearDown(self) -> None:
        self.replayer.stop()
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def _create_mock_episode_file(self, filepath: Path, total_frames: int = 20) -> None:
        """Create sample trajectory file."""
        metadata = {
            "type": "episode_metadata",
            "task_name": "replay_task",
            "episode_id": 42,
            "robot_type": "so_arm101",
            "env_type": "sim",
            "is_sim": True,
        }

        frames = []
        for i in range(total_frames):
            rel_t = round(i * 0.033, 3)
            q_pos = {j: 0.0 for j in SO101_JOINT_NAMES}
            q_pos["shoulder_pan"] = round(i * 0.05, 3)
            frames.append({
                "type": "frame",
                "frame_index": i,
                "relative_time_s": rel_t,
                "timestamp": 100.0 + rel_t,
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
                    "source": "replay",
                },
            })

        summary = {
            "type": "episode_summary",
            "task_name": "replay_task",
            "episode_id": 42,
            "total_frames": total_frames,
            "duration_seconds": round((total_frames - 1) * 0.033, 3),
        }

        with open(filepath, "w", encoding="utf-8") as f:
            f.write(json.dumps(metadata) + "\n")
            for frame in frames:
                f.write(json.dumps(frame) + "\n")
            f.write(json.dumps(summary) + "\n")

    def test_load_episode(self) -> None:
        """测试轨迹载入与状态初始化."""
        self.assertFalse(self.replayer.is_loaded)
        res = self.replayer.load_episode(self.sample_jsonl)

        self.assertTrue(res["success"])
        self.assertEqual(res["total_frames"], 20)
        self.assertEqual(res["task_name"], "replay_task")
        self.assertTrue(self.replayer.is_loaded)
        self.assertEqual(self.replayer.total_frames, 20)
        self.assertEqual(self.replayer.current_frame, 0)

    def test_seek_and_step(self) -> None:
        """测试帧精确跳转与单步步进."""
        self.replayer.load_episode(self.sample_jsonl)

        # Seek to frame 10
        res = self.replayer.seek(10)
        self.assertTrue(res["success"])
        self.assertEqual(self.replayer.current_frame, 10)
        self.assertAlmostEqual(res["joints_rad"]["shoulder_pan"], 0.5, places=2)

        # Step forward (+1)
        res_step = self.replayer.step(1)
        self.assertTrue(res_step["success"])
        self.assertEqual(self.replayer.current_frame, 11)

        # Step backward (-1)
        res_back = self.replayer.step(-1)
        self.assertTrue(res_back["success"])
        self.assertEqual(self.replayer.current_frame, 10)

        # Seek beyond bounds clamp test
        self.replayer.seek(999)
        self.assertEqual(self.replayer.current_frame, 19)

    def test_playback_lifecycle(self) -> None:
        """测试回放启动、暂停、恢复与停止的完整生命周期与状态机转换."""
        self.replayer.load_episode(self.sample_jsonl)

        # Play at 10x speed to run fast
        played = self.replayer.play(speed=5.0, loop=False, start_frame=0)
        self.assertTrue(played)
        self.assertTrue(self.replayer.is_playing)
        self.assertEqual(self.sm.current_state, SystemState.PLAYBACK)
        self.assertEqual(self.sm.active_source, ControlSource.REPLAY)

        # Pause
        self.replayer.pause()
        self.assertTrue(self.replayer.is_paused)
        self.assertFalse(self.replayer.is_playing)

        # Resume
        self.replayer.resume()
        self.assertTrue(self.replayer.is_playing)
        self.assertFalse(self.replayer.is_paused)

        # Stop
        self.replayer.stop()
        self.assertFalse(self.replayer.is_playing)
        self.assertFalse(self.replayer.is_paused)
        self.assertEqual(self.sm.current_state, SystemState.IDLE)
        self.assertIsNone(self.sm.active_source)

    def test_speed_and_loop_settings(self) -> None:
        """测试动态倍速调节与循环设置."""
        self.replayer.load_episode(self.sample_jsonl)
        self.replayer.set_speed(2.5)
        self.replayer.set_loop(True)

        st = self.replayer.get_status()
        self.assertEqual(st["speed"], 2.5)
        self.assertTrue(st["loop"])
        self.assertEqual(st["total_frames"], 20)

    def test_frame_callback(self) -> None:
        """测试每帧回调函数正常触发."""
        self.replayer.load_episode(self.sample_jsonl)
        called_frames = []

        def on_frame(idx: int, frame_dict: dict) -> None:
            called_frames.append(idx)

        self.replayer.on_frame_callback = on_frame
        self.replayer.play(speed=5.0, loop=False, start_frame=0)

        # Wait a short moment for a few frames to play
        time.sleep(0.08)
        self.replayer.stop()

        self.assertGreater(len(called_frames), 0)


if __name__ == "__main__":
    unittest.main()

