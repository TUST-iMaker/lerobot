"""Hardware package for SO-ARM101 robot system.

Provides modular hardware adapters for Follower and Leader arms, as well as unified system-level teleoperation management.
"""

from __future__ import annotations

from hardware.base import ArmHealth, BaseArmAdapter
from hardware.config import ArmConfig, ArmRole, RobotSystemConfig
from hardware.follower_arm import SO101FollowerArm
from hardware.leader_arm import SO101LeaderArm
from hardware.mock_arm import MockArmAdapter
from hardware.system_adapter import RobotSystemAdapter

# Backward compatibility alias
SO101FollowerAdapter = SO101FollowerArm
BaseRobotAdapter = BaseArmAdapter

__all__ = [
    "ArmRole",
    "ArmConfig",
    "RobotSystemConfig",
    "BaseArmAdapter",
    "BaseRobotAdapter",
    "ArmHealth",
    "SO101FollowerArm",
    "SO101FollowerAdapter",
    "SO101LeaderArm",
    "MockArmAdapter",
    "RobotSystemAdapter",
]

