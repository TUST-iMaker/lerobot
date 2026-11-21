"""测试全局配置加载与解析 (test_config.py)."""

import unittest
from pathlib import Path
from core.config_loader import (
    DEFAULT_CONFIG_PATH,
    FollowerConfig,
    LeaderConfig,
    get_config,
    load_config,
)


class TestConfigLoader(unittest.TestCase):
    """测试配置读取与各模块参数映射."""

    def test_default_config_exists(self):
        """测试 config.yaml 默认配置文件存在."""
        self.assertTrue(DEFAULT_CONFIG_PATH.is_file(), "software/config.yaml 应存在")

    def test_load_config_values(self):
        """测试加载出的配置属性及数据类型."""
        cfg = get_config(reload=True)
        self.assertIsInstance(cfg.follower, FollowerConfig)
        self.assertIsInstance(cfg.leader, LeaderConfig)
        self.assertTrue(bool(cfg.follower.port))
        self.assertTrue(bool(cfg.leader.port))
        self.assertTrue(bool(cfg.follower.id))
        self.assertTrue(bool(cfg.leader.id))
        self.assertEqual(cfg.camera.width, 640)
        self.assertEqual(cfg.camera.height, 480)
        self.assertEqual(cfg.camera.fps, 30)
        self.assertTrue(cfg.safety.auto_rest_on_exit)
        self.assertEqual(cfg.operation.fps, 30)
        self.assertEqual(cfg.operation.default_replay_episode, 0)

    def test_camera_index_or_path(self):
        """测试相机设备路径与索引属性解析."""
        cfg = get_config()
        self.assertIsInstance(cfg.camera.index_or_path, (int, str))

    def test_safety_rest_pose_path(self):
        """测试蜷缩姿态绝对路径解析."""
        cfg = get_config()
        self.assertEqual(cfg.safety.rest_pose_path.name, "rest_pose.json")

    def test_safety_home_pose_path(self):
        """测试中位姿态绝对路径解析与预设绑定."""
        from core.presets import get_preset_pose
        cfg = get_config()
        self.assertEqual(cfg.safety.home_pose_path.name, "upright_pose.json")
        home_pose = get_preset_pose("home")
        self.assertIn("wrist_flex", home_pose)
        self.assertIn("wrist_roll", home_pose)


if __name__ == "__main__":
    unittest.main()

