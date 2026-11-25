"""SO-ARM101 机械臂预设姿态与宏动作库.

约定:
- 5 个臂关节 (shoulder_pan..wrist_roll): 内部存储弧度 (rad), 导出 DEGREE 时转为角度;
- gripper: 统一遵循官方 MotorNormMode.RANGE_0_100 ([0.0, 100.0] %, 0=闭合, 100=全开),
  不随 DEGREE/RADIAN 转换而缩放.
"""

from __future__ import annotations

import json
from pathlib import Path

from core.data_types import (
    SO101_GRIPPER_JOINT_NAME,
    SO101_JOINT_NAMES,
    JointUnit,
    clamp_gripper_pct,
    deg_to_rad,
    rad_to_deg,
)

# 标准中位校准基准 (5 臂关节全 0.0 rad: 上臂竖直、小臂与夹爪水平前伸; 夹爪 0.0% 闭合)
POSE_HOME_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

# 折叠休眠位 (下垂低重心姿态, 严格在 URDF 限位内)
POSE_REST_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.10,
    "elbow_flex": 1.35,
    "wrist_flex": 0.20,
    "wrist_roll": 0.0,
    "gripper": 5.0,
}

# 桌面俯视观察位 (用于相机取景与目标识别)
POSE_INSPECT_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.35,
    "elbow_flex": 0.25,
    "wrist_flex": -0.60,
    "wrist_roll": 0.0,
    "gripper": 20.0,
}

# 抓取悬停预备位 (末端居中前伸, 夹爪张开 75%)
POSE_PICK_HOVER_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.50,
    "elbow_flex": 0.40,
    "wrist_flex": -0.90,
    "wrist_roll": 0.0,
    "gripper": 75.0,
}

# 挥手致意位 (大臂微抬, 手腕摆动预备)
POSE_WAVE_RAD: dict[str, float] = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.65,
    "elbow_flex": -0.30,
    "wrist_flex": 0.10,
    "wrist_roll": 0.70,
    "gripper": 30.0,
}

PRESET_POSES: dict[str, dict[str, float]] = {
    "home": POSE_HOME_RAD,
    "rest": POSE_REST_RAD,
    "inspect": POSE_INSPECT_RAD,
    "pick_hover": POSE_PICK_HOVER_RAD,
    "wave": POSE_WAVE_RAD,
}


def _parse_pose_json(path: Path) -> dict[str, float] | None:
    """解析姿态 JSON 文件 (臂关节按 unit 转换, gripper 始终保持 RANGE_0_100 [0, 100])."""
    if not path.is_file():
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        joints_data = data.get("joints", {})
        is_deg = data.get("unit") == "degree"
        parsed: dict[str, float] = {}
        for k, v in joints_data.items():
            jname = k[:-4] if k.endswith(".pos") else k
            if jname == SO101_GRIPPER_JOINT_NAME:
                parsed[jname] = clamp_gripper_pct(float(v))
            else:
                parsed[jname] = deg_to_rad(float(v)) if is_deg else float(v)
        if all(j in parsed for j in SO101_JOINT_NAMES):
            return parsed
    except Exception:
        return None
    return None


def get_preset_pose(name: str, unit: JointUnit = JointUnit.RADIAN) -> dict[str, float]:
    """获取指定预设姿态字典，支持弧度或角度单位转换 (gripper 始终保持 0..100%).

    对于 "home" (中位)，优先读取 software/upright_pose.json 定制姿态文件.
    对于 "rest" (蜷缩位)，优先读取 software/rest_pose.json 定制姿态文件.
    """
    name_key = name.lower().strip()
    pose_rad: dict[str, float] | None = None

    if name_key == "home":
        try:
            from core.config_loader import get_config
            upright_path = get_config().safety.home_pose_path
        except Exception:
            upright_path = Path(__file__).resolve().parent.parent / "upright_pose.json"
        pose_rad = _parse_pose_json(upright_path)

    elif name_key == "rest":
        try:
            from core.config_loader import get_config
            rest_path = get_config().safety.rest_pose_path
        except Exception:
            rest_path = Path(__file__).resolve().parent.parent / "rest_pose.json"
        pose_rad = _parse_pose_json(rest_path)

    if pose_rad is None:
        if name_key not in PRESET_POSES:
            raise KeyError(f"未知的预设姿态名称: {name} (可用预设: {list(PRESET_POSES.keys())})")
        pose_rad = dict(PRESET_POSES[name_key])

    if unit == JointUnit.DEGREE:
        return {
            k: round(float(v), 2) if k == SO101_GRIPPER_JOINT_NAME else round(rad_to_deg(v), 2)
            for k, v in pose_rad.items()
        }
    return dict(pose_rad)


def list_preset_poses() -> list[str]:
    """返回所有可用预设姿态名称列表."""
    return list(PRESET_POSES.keys())
