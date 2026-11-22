"""Unit tests for core/kinematics.py."""

from __future__ import annotations

import math
import unittest

from core.kinematics import KinematicsConfig, SO101Kinematics


class TestSO101Kinematics(unittest.TestCase):
    def setUp(self) -> None:
        self.kin = SO101Kinematics(KinematicsConfig())
        self.L1 = self.kin.config.base_height_m
        self.L2 = self.kin.config.upper_arm_m
        self.L3 = self.kin.config.forearm_m
        self.L4 = self.kin.config.gripper_offset_m

    def test_neutral_fk(self) -> None:
        # 对齐官方 so101_new_calib.urdf: 零位 gripper_frame_link TCP 应为 (0.39136, -0.00001, 0.22647) m
        joints = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        (x, y, z), pitch, roll = self.kin.forward_kinematics(joints)
        self.assertAlmostEqual(x, 0.39136, places=4)
        self.assertAlmostEqual(y, -0.00001, places=4)
        self.assertAlmostEqual(z, 0.22647, places=4)
        self.assertAlmostEqual(pitch, 0.0, places=4)

        # 验证 shoulder_pan 世界轴为 [0, 0, -1]: +0.5 rad 向 -Y 旋转
        (x2, y2, _), _, _ = self.kin.forward_kinematics({"shoulder_pan": 0.5})
        self.assertGreater(x2, 0.30)
        self.assertLess(y2, -0.15)

    def test_fk_ik_roundtrip(self) -> None:
        # 挑选典型工作姿态
        test_cases = [
            (0.0, 0.0, 0.0, 0.0),
            (0.3, 0.2, -0.3, 0.1),
            (-0.4, -0.1, 0.2, -0.1),
            (0.5, 0.3, -0.4, 0.1),
        ]

        for pan, lift, elbow, wrist_p in test_cases:
            original_joints = {
                "shoulder_pan": pan,
                "shoulder_lift": lift,
                "elbow_flex": elbow,
                "wrist_flex": wrist_p,
                "wrist_roll": 0.1,
                "gripper": 0.0,
            }

            # 1. FK 计算末端空间点
            (x, y, z), pitch, roll = self.kin.forward_kinematics(original_joints)

            # 2. IK 反解关节角度
            ok, solved_joints, msg = self.kin.inverse_kinematics(
                target_xyz=(x, y, z),
                desired_pitch_rad=pitch,
                preserve_wrist_roll=roll,
            )
            self.assertTrue(ok, f"IK 求解失败: {msg}")
            self.assertIsNotNone(solved_joints)

            # 3. 再用反解结果做 FK，验证空间点闭环一致性
            (x2, y2, z2), pitch2, roll2 = self.kin.forward_kinematics(solved_joints)
            self.assertAlmostEqual(x, x2, places=4)
            self.assertAlmostEqual(y, y2, places=4)
            self.assertAlmostEqual(z, z2, places=4)
            self.assertAlmostEqual(pitch, pitch2, places=4)

    def test_unreachable_target(self) -> None:
        # 请求超出总臂展的极限目标 (1.5 米)
        ok, joints, msg = self.kin.inverse_kinematics((1.5, 0.0, 0.1))
        self.assertFalse(ok)
        self.assertIsNone(joints)
        self.assertIn("超出最大臂展", msg)


if __name__ == "__main__":
    unittest.main()

