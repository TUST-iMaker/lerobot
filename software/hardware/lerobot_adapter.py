"""Compatibility shim for lerobot_adapter."""

from hardware.follower_arm import SO101FollowerArm
from hardware.system_adapter import RobotSystemAdapter

SO101FollowerAdapter = SO101FollowerArm

__all__ = ["SO101FollowerArm", "SO101FollowerAdapter", "RobotSystemAdapter"]

