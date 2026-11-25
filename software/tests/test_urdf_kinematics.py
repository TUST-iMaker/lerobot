"""Direct URDF verification tests against repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf."""

from __future__ import annotations

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from core.data_types import SO101_ARM_JOINT_NAMES, JointState, JointUnit
from core.kinematics import DEFAULT_SO101_URDF_PATH, SO101Kinematics, solve_horizontal_planar_reach
from core.safety_controller import DEFAULT_JOINT_LIMITS_RAD


class TestURDFKinematicsAndLimits(unittest.TestCase):
    """Cross-verify SO101Kinematics & SafetyController directly against official URDF XML."""

    def test_urdf_file_exists_and_limits_match_safety_controller(self) -> None:
        self.assertTrue(DEFAULT_SO101_URDF_PATH.is_file(), f"URDF missing: {DEFAULT_SO101_URDF_PATH}")
        tree = ET.parse(str(DEFAULT_SO101_URDF_PATH))
        root = tree.getroot()
        urdf_limits: dict[str, tuple[float, float]] = {}
        for j in root.findall("joint"):
            name = j.attrib["name"]
            lim = j.find("limit")
            if name in SO101_ARM_JOINT_NAMES and lim is not None:
                urdf_limits[name] = (float(lim.attrib["lower"]), float(lim.attrib["upper"]))

        self.assertEqual(set(urdf_limits.keys()), set(SO101_ARM_JOINT_NAMES))
        for jname, (lo, hi) in urdf_limits.items():
            s_lo, s_hi = DEFAULT_JOINT_LIMITS_RAD[jname]
            self.assertAlmostEqual(s_lo, lo, places=5)
            self.assertAlmostEqual(s_hi, hi, places=5)

    def test_zero_pose_tcp_and_shoulder_pan_negative_z_axis(self) -> None:
        kin = SO101Kinematics()
        (x0, y0, z0), pitch0, roll0 = kin.forward_kinematics(
            {j: 0.0 for j in SO101_ARM_JOINT_NAMES},
            unit=JointUnit.RADIAN,
        )
        # Official so101_new_calib.urdf zero-pose gripper_frame_link: (391.36, -0.01, 226.47) mm
        self.assertAlmostEqual(x0 * 1000.0, 391.36, delta=0.1)
        self.assertAlmostEqual(y0 * 1000.0, -0.01, delta=0.1)
        self.assertAlmostEqual(z0 * 1000.0, 226.47, delta=0.1)
        self.assertAlmostEqual(pitch0, 0.0, places=4)
        self.assertAlmostEqual(roll0, 0.0, places=5)

        # +0.5 rad on shoulder_pan rotates around world [0, 0, -1] -> -Y
        (x1, y1, z1), _, _ = kin.forward_kinematics({"shoulder_pan": 0.5})
        self.assertLess(y1, -0.15)
        self.assertAlmostEqual(z1, z0, places=4)

    def test_studio_targets_and_planar_reach(self) -> None:
        kin = SO101Kinematics()
        for tgt in [(0.180, 0.020, 0.210), (0.208, 0.0, 0.245)]:
            ok, sol, msg = kin.inverse_kinematics(tgt, desired_pitch_rad=0.0)
            self.assertTrue(ok, msg)
            self.assertIsNotNone(sol)
            (fx, fy, fz), _, _ = kin.forward_kinematics(sol)
            err_mm = math.dist(tgt, (fx, fy, fz)) * 1000.0
            self.assertLess(err_mm, 1.0)

        ok_far, _, _ = kin.inverse_kinematics((0.999, 0.0, 0.999))
        self.assertFalse(ok_far)

        ok_p, lift, elbow, wrist, msg_p = solve_horizontal_planar_reach(0.32, 0.22647)
        self.assertTrue(ok_p, msg_p)
        pose3d = kin.compute_tcp_pose(
            JointState(
                timestamp=1.0,
                positions={
                    "shoulder_pan": 0.0,
                    "shoulder_lift": lift,
                    "elbow_flex": elbow,
                    "wrist_flex": wrist,
                    "wrist_roll": 0.0,
                    "gripper": 50.0,
                },
                unit=JointUnit.RADIAN,
            )
        )
        self.assertTrue(pose3d.valid)
        self.assertAlmostEqual(pose3d.position_m[0], 0.32, places=3)
        self.assertAlmostEqual(pose3d.position_m[2], 0.22647, places=3)


if __name__ == "__main__":
    unittest.main()
