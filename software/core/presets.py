"""SO-ARM101 机械臂预设姿态与宏动作库.

为现场演示、自动巡检、安全复位与抓取示教提供标准姿态预设及插值运动序列.
"""

from __future__ import annotations

from typing import Any

from core.data_types import (
    SO101_JOINT_NAMES,
    JointUnit,
    deg_to_rad,
    rad_to_deg,
)

# 标准中位校准基准 (全 0.0 弧度: 大臂垂直、小臂水平、夹爪水平向正前)
POSE_HOME_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

# 折叠休眠位 (下垂低重心姿态)
POSE_REST_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.10,
    "elbow_flex": 1.35,
    "wrist_flex": 0.20,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

# 桌面俯视观察位 (用于相机取景与目标识别)
POSE_INSPECT_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.35,
    "elbow_flex": 0.25,
    "wrist_flex": -0.60,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

# 抓取悬停预备位 (末端居中前伸, 夹爪张开)
POSE_PICK_HOVER_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.50,
    "elbow_flex": 0.40,
    "wrist_flex": -0.90,
    "wrist_roll": 0.0,
    "gripper": -0.85,
}

# 挥手致意位 (大臂微抬, 手腕摆动预备)
POSE_WAVE_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.65,
    "elbow_flex": -0.30,
    "wrist_flex": 0.10,
    "wrist_roll": 0.70,
    "gripper": 0.0,
}

PRESET_POSES: dict[str, dict[str, float]] = {
    "home": POSE_HOME_RAD,
    "rest": POSE_REST_RAD,
    "inspect": POSE_INSPECT_RAD,
    "pick_hover": POSE_PICK_HOVER_RAD,
    "wave": POSE_WAVE_RAD,
}


def get_preset_pose(name: str, unit: JointUnit = JointUnit.RADIAN) -> dict[str, float]:
    """获取指定预设姿态字典，支持弧度或角度单位转换.
    
    对于 "home" (中位)，优先读取 software/upright_pose.json 定制姿态文件.
    对于 "rest" (蜷缩位)，优先读取 software/rest_pose.json 定制姿态文件.
    """
    import json
    from pathlib import Path

    name_key = name.lower().strip()
    pose_rad: dict[str, float] | None = None

    if name_key == "home":
        try:
            from core.config_loader import get_config
            upright_path = get_config().safety.home_pose_path
        except Exception:
            upright_path = Path(__file__).resolve().parent.parent / "upright_pose.json"

        if upright_path.is_file():
            try:
                with open(upright_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                joints_data = data.get("joints", {})
                is_deg = data.get("unit") == "degree"
                custom_home: dict[str, float] = {}
                for k, v in joints_data.items():
                    jname = k[:-4] if k.endswith(".pos") else k
                    custom_home[jname] = deg_to_rad(float(v)) if is_deg else float(v)
                if all(j in custom_home for j in SO101_JOINT_NAMES):
                    pose_rad = custom_home
            except Exception:
                pose_rad = None

    elif name_key == "rest":
        try:
            from core.config_loader import get_config
            rest_path = get_config().safety.rest_pose_path
        except Exception:
            rest_path = Path(__file__).resolve().parent.parent / "rest_pose.json"

        if rest_path.is_file():
            try:
                with open(rest_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                joints_data = data.get("joints", {})
                is_deg = data.get("unit") == "degree"
                custom_rest: dict[str, float] = {}
                for k, v in joints_data.items():
                    jname = k[:-4] if k.endswith(".pos") else k
                    custom_rest[jname] = deg_to_rad(float(v)) if is_deg else float(v)
                if all(j in custom_rest for j in SO101_JOINT_NAMES):
                    pose_rad = custom_rest
            except Exception:
                pose_rad = None

    if pose_rad is None:
        if name_key not in PRESET_POSES:
            raise KeyError(f"未知的预设姿态名称: {name} (可用预设: {list(PRESET_POSES.keys())})")
        pose_rad = dict(PRESET_POSES[name_key])

    if unit == JointUnit.DEGREE:
        return {k: round(rad_to_deg(v), 2) for k, v in pose_rad.items()}
    return dict(pose_rad)


def list_preset_poses() -> list[str]:
    """返回所有可用预设姿态名称列表."""
    return list(PRESET_POSES.keys())
