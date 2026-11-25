"""SO-ARM101 具身控制统一数据协议与核心数据类型定义.

设计原则:
1. 强类型与显式单位: 拒绝歧义的原始字典, 明确标注 radian / degree / raw 等单位;
2. 防御性校验: 提供有效性检查 (NaN/Inf 过滤、关节缺失检查、时间戳单调性);
3. 观测与动作解耦: 严格区分实际传感器反馈 (JointState) 与控制器下发指令 (RobotAction);
4. 序列化支持: 提供标准 JSON 序列化与反序列化方法, 为 JSONL 示教记录与 LeRobotDataset 转换奠定基础.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


# ==============================================================================
# 1. 硬件关节常量
# ==============================================================================

SO101_JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

SO101_ARM_JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
SO101_GRIPPER_JOINT_NAME: str = "gripper"

# 中位回零与使能安全顺序 (自外向内或按物理承重顺序)
MIDDLE_ORDER: tuple[str, ...] = (
    "elbow_flex",
    "shoulder_lift",
    "wrist_flex",
    "wrist_roll",
    "shoulder_pan",
    "gripper",
)


# ==============================================================================
# 2. 枚举定义
# ==============================================================================

class JointUnit(str, Enum):
    """关节数值单位定义."""
    RADIAN = "radian"               # 弧度 ([-pi, pi] 国际标准单位，运动学内部推荐)
    DEGREE = "degree"               # 角度 ([-180, 180] 直观易读)
    RANGE_0_100 = "range_0_100"     # 百分比 [0.0, 100.0] (官方 gripper 归一化单位)
    NORMALIZED = "normalized"       # 归一化 ([-100, 100] 或 [-1, 1]，LeRobot 常见格式)
    RAW_STEPS = "raw_steps"         # 舵机底层原始计数值 (如 0~4096)


class ActionType(str, Enum):
    """动作意图类型."""
    JOINT_POSITION = "JOINT_POSITION"       # 绝对关节目标
    JOINT_DELTA = "JOINT_DELTA"             # 关节角度增量
    CARTESIAN_POSE = "CARTESIAN_POSE"       # 末端绝对位姿 (x,y,z,qx,qy,qz,qw)
    CARTESIAN_DELTA = "CARTESIAN_DELTA"     # 末端增量 (dx,dy,dz,droll,dpitch,dyaw)


class ControlSource(str, Enum):
    """动作输入来源."""
    KEYBOARD = "keyboard"
    LEADER = "leader"
    REPLAY = "replay"
    POLICY = "policy"
    UI = "ui"
    MOUSE = "mouse"
    SCRIPT = "script"


class EnvironmentMode(str, Enum):
    """示教与采集环境类型 (真机 vs 仿真)."""
    REAL = "real"           # 真实物理机械臂实机数据
    SIM = "sim"             # 虚拟仿真环境数据


class SystemState(str, Enum):
    """机器人系统运行状态机状态."""
    DISCONNECTED = "DISCONNECTED"       # 硬件未连接
    IDLE = "IDLE"                       # 已连接，伺服未使能或待机
    TELEOP = "TELEOP"                   # 遥操作/手动控制中
    RECORDING = "RECORDING"             # 正在记录轨迹数据
    RECORD_PAUSED = "RECORD_PAUSED"     # 轨迹记录暂停
    PLAYBACK_READY = "PLAYBACK_READY"   # 轨迹已加载校验通过，等待执行
    PLAYBACK = "PLAYBACK"               # 正在执行轨迹回放
    PLAYBACK_PAUSED = "PLAYBACK_PAUSED" # 轨迹回放暂停
    STOPPING = "STOPPING"               # 减速停止中
    ESTOP_LATCHED = "ESTOP_LATCHED"     # 急停闭锁 (拒绝新指令，保持力矩)
    FAULT = "FAULT"                     # 故障状态 (通信异常/过载/硬件报警)


# ==============================================================================
# 3. 辅助转换函数
# ==============================================================================

def deg_to_rad(val: float) -> float:
    """角度转弧度."""
    return val * math.pi / 180.0


def rad_to_deg(val: float) -> float:
    """弧度转角度."""
    return val * 180.0 / math.pi


def clamp_gripper_pct(val: float) -> float:
    """将夹爪开合度钳位至官方 RANGE_0_100 范围 [0.0, 100.0]."""
    return max(0.0, min(100.0, float(val)))


def convert_joint_positions(
    positions: dict[str, float],
    from_unit: JointUnit,
    to_unit: JointUnit,
    gripper_unit: JointUnit = JointUnit.RANGE_0_100,
) -> dict[str, float]:
    """在 RADIAN 与 DEGREE 之间转换 5 个臂关节，同时保持 RANGE_0_100 夹爪百分比不变."""
    if from_unit == to_unit:
        return {k: float(v) for k, v in positions.items()}
    out: dict[str, float] = {}
    for k, v in positions.items():
        fval = float(v)
        if k == SO101_GRIPPER_JOINT_NAME and gripper_unit == JointUnit.RANGE_0_100:
            out[k] = fval
        elif from_unit == JointUnit.DEGREE and to_unit == JointUnit.RADIAN:
            out[k] = deg_to_rad(fval)
        elif from_unit == JointUnit.RADIAN and to_unit == JointUnit.DEGREE:
            out[k] = rad_to_deg(fval)
        else:
            raise ValueError(f"暂不支持从 {from_unit} 转换为 {to_unit}")
    return out


# ==============================================================================
# 4. 核心数据类
# ==============================================================================

@dataclass(frozen=True)
class JointState:
    """某一时刻的机械臂关节状态观测.
    
    Attributes:
        timestamp: 单调时钟时间戳 (秒, 来自 time.monotonic())
        positions: 关节位置字典 {joint_name: value}
        unit: 位置数值的单位
        velocities: 可选的关节速度反馈
        efforts: 可选的电流/力矩反馈
        torques_enabled: 各关节扭矩使能状态 {joint_name: bool}
    """
    timestamp: float
    positions: dict[str, float]
    unit: JointUnit = JointUnit.RADIAN
    velocities: dict[str, float] | None = None
    efforts: dict[str, float] | None = None
    torques_enabled: dict[str, bool] | None = None
    gripper_unit: JointUnit = JointUnit.RANGE_0_100

    def validate(self, require_all_joints: bool = True) -> tuple[bool, str | None]:
        """校验关节状态数据的完备性与合法性."""
        if not math.isfinite(self.timestamp) or self.timestamp < 0:
            return False, f"非法的时间戳: {self.timestamp}"

        if require_all_joints:
            missing = set(SO101_JOINT_NAMES) - set(self.positions.keys())
            if missing:
                return False, f"缺少关键关节数据: {sorted(missing)}"

        for name, val in self.positions.items():
            if not isinstance(val, (int, float)):
                return False, f"关节 {name} 数值类型无效: {type(val)}"
            if not math.isfinite(val):
                return False, f"关节 {name} 包含非有限数值: {val}"

        return True, None

    def to_radians(self) -> JointState:
        """转换臂关节位置为弧度表示 (保持 RANGE_0_100 夹爪原值不变)."""
        if self.unit == JointUnit.RADIAN:
            return self
        if self.unit == JointUnit.DEGREE:
            new_pos = convert_joint_positions(
                self.positions,
                self.unit,
                JointUnit.RADIAN,
                gripper_unit=self.gripper_unit,
            )
            return JointState(
                timestamp=self.timestamp,
                positions=new_pos,
                unit=JointUnit.RADIAN,
                velocities=self.velocities,
                efforts=self.efforts,
                torques_enabled=self.torques_enabled,
                gripper_unit=self.gripper_unit,
            )
        raise ValueError(f"暂不支持从 {self.unit} 转换为 RADIAN")

    def to_degrees(self) -> JointState:
        """转换臂关节位置为角度表示 (保持 RANGE_0_100 夹爪原值不变)."""
        if self.unit == JointUnit.DEGREE:
            return self
        if self.unit == JointUnit.RADIAN:
            new_pos = convert_joint_positions(
                self.positions,
                self.unit,
                JointUnit.DEGREE,
                gripper_unit=self.gripper_unit,
            )
            return JointState(
                timestamp=self.timestamp,
                positions=new_pos,
                unit=JointUnit.DEGREE,
                velocities=self.velocities,
                efforts=self.efforts,
                torques_enabled=self.torques_enabled,
                gripper_unit=self.gripper_unit,
            )
        raise ValueError(f"暂不支持从 {self.unit} 转换为 DEGREE")

    def to_dict(self) -> dict[str, Any]:
        """序列化为标准字典."""
        d = asdict(self)
        d["unit"] = self.unit.value
        d["gripper_unit"] = self.gripper_unit.value
        return d

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> JointState:
        """从标准字典反序列化."""
        unit_str = data.get("unit", JointUnit.RADIAN.value)
        gripper_unit_str = data.get("gripper_unit", JointUnit.RANGE_0_100.value)
        return cls(
            timestamp=float(data["timestamp"]),
            positions={k: float(v) for k, v in data["positions"].items()},
            unit=JointUnit(unit_str),
            velocities={k: float(v) for k, v in data["velocities"].items()} if data.get("velocities") else None,
            efforts={k: float(v) for k, v in data["efforts"].items()} if data.get("efforts") else None,
            torques_enabled={k: bool(v) for k, v in data["torques_enabled"].items()} if data.get("torques_enabled") else None,
            gripper_unit=JointUnit(gripper_unit_str),
        )


@dataclass(frozen=True)
class Pose3D:
    """机械臂末端 (TCP) 或空间位姿.
    
    Attributes:
        timestamp: 单调时钟时间戳
        frame_id: 坐标系标识 (例如 'robot_base')
        position_m: 三维位置 (x, y, z)，单位米
        orientation_xyzw: 四元数 (qx, qy, qz, qw)
        valid: 运动学校验标记 (False 代表运动学未标定或正解计算无效)
        kinematics_version: 运动学模型版本号
    """
    timestamp: float
    frame_id: str = "robot_base"
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    valid: bool = False
    kinematics_version: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Pose3D:
        return cls(
            timestamp=float(data["timestamp"]),
            frame_id=str(data.get("frame_id", "robot_base")),
            position_m=tuple(float(x) for x in data["position_m"]),  # type: ignore[arg-type]
            orientation_xyzw=tuple(float(x) for x in data["orientation_xyzw"]),  # type: ignore[arg-type]
            valid=bool(data.get("valid", False)),
            kinematics_version=data.get("kinematics_version"),
        )


@dataclass(frozen=True)
class CameraFrameRef:
    """相机帧元数据与存储引用.
    
    Attributes:
        camera_id: 相机逻辑名 (如 'wrist', 'global')
        frame_id: 相机独立自增帧序号
        timestamp: 图像采集时的单调时钟时间戳
        width: 宽度像素
        height: 高度像素
        color_format: 编码或通道格式 (如 'RGB', 'BGR', 'JPEG')
        relative_path: 相对数据集根目录的保存路径 (如 'images/wrist/frame_000010.jpg')
    """
    camera_id: str
    frame_id: int
    timestamp: float
    width: int
    height: int
    color_format: str = "RGB"
    relative_path: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> CameraFrameRef:
        return cls(
            camera_id=str(data["camera_id"]),
            frame_id=int(data["frame_id"]),
            timestamp=float(data["timestamp"]),
            width=int(data["width"]),
            height=int(data["height"]),
            color_format=str(data.get("color_format", "RGB")),
            relative_path=data.get("relative_path"),
        )


@dataclass(frozen=True)
class RobotAction:
    """统一动作意图命令. 所有输入源 (键盘、Leader、回放、模型) 均生成此对象.
    
    Attributes:
        action_type: 动作类型 (绝对位置 / 增量 / 末端位姿)
        source: 输入来源
        timestamp: 发出时间戳 (单调时钟)
        target_joint_positions: 目标关节位置 (JOINT_POSITION 模式使用)
        target_joint_deltas: 目标关节增量 (JOINT_DELTA 模式使用)
        target_cartesian_delta: 末端空间增量 (dx, dy, dz, droll, dpitch, dyaw)
        target_cartesian_pose: 末端绝对位姿 (CARTESIAN_POSE 模式使用)
        unit: 关节相关目标值的单位
    """
    action_type: ActionType
    source: ControlSource
    timestamp: float
    target_joint_positions: dict[str, float] | None = None
    target_joint_deltas: dict[str, float] | None = None
    target_cartesian_delta: tuple[float, ...] | None = None
    target_cartesian_pose: Pose3D | None = None
    unit: JointUnit = JointUnit.RADIAN
    gripper_unit: JointUnit = JointUnit.RANGE_0_100

    def validate(self) -> tuple[bool, str | None]:
        """校验动作意图是否符合对应的动作类型定义."""
        if not math.isfinite(self.timestamp) or self.timestamp < 0:
            return False, f"非法的时间戳: {self.timestamp}"

        if self.action_type == ActionType.JOINT_POSITION:
            if not self.target_joint_positions:
                return False, "JOINT_POSITION 模式缺少 target_joint_positions"
            for k, v in self.target_joint_positions.items():
                if not math.isfinite(v):
                    return False, f"关节 {k} 目标非有限值: {v}"

        elif self.action_type == ActionType.JOINT_DELTA:
            if not self.target_joint_deltas:
                return False, "JOINT_DELTA 模式缺少 target_joint_deltas"
            for k, v in self.target_joint_deltas.items():
                if not math.isfinite(v):
                    return False, f"关节增量 {k} 非有限值: {v}"

        elif self.action_type == ActionType.CARTESIAN_DELTA:
            if not self.target_cartesian_delta:
                return False, "CARTESIAN_DELTA 模式缺少 target_cartesian_delta"
            if len(self.target_cartesian_delta) not in (3, 6):
                return False, f"末端增量维度必须为 3 或 6, 当前为 {len(self.target_cartesian_delta)}"
            for i, v in enumerate(self.target_cartesian_delta):
                if not math.isfinite(v):
                    return False, f"末端增量第 {i} 项非有限值: {v}"

        elif self.action_type == ActionType.CARTESIAN_POSE:
            if self.target_cartesian_pose is None:
                return False, "CARTESIAN_POSE 模式缺少 target_cartesian_pose"

        return True, None

    def to_radians(self) -> RobotAction:
        """将关节目标/增量转换为弧度表示 (保持 RANGE_0_100 夹爪不变)."""
        if self.unit == JointUnit.RADIAN:
            return self
        new_pos = (
            convert_joint_positions(
                self.target_joint_positions,
                self.unit,
                JointUnit.RADIAN,
                gripper_unit=self.gripper_unit,
            )
            if self.target_joint_positions is not None
            else None
        )
        new_deltas = (
            convert_joint_positions(
                self.target_joint_deltas,
                self.unit,
                JointUnit.RADIAN,
                gripper_unit=self.gripper_unit,
            )
            if self.target_joint_deltas is not None
            else None
        )
        return RobotAction(
            action_type=self.action_type,
            source=self.source,
            timestamp=self.timestamp,
            target_joint_positions=new_pos,
            target_joint_deltas=new_deltas,
            target_cartesian_delta=self.target_cartesian_delta,
            target_cartesian_pose=self.target_cartesian_pose,
            unit=JointUnit.RADIAN,
            gripper_unit=self.gripper_unit,
        )

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["action_type"] = self.action_type.value
        d["source"] = self.source.value
        d["unit"] = self.unit.value
        d["gripper_unit"] = self.gripper_unit.value
        if self.target_cartesian_pose:
            d["target_cartesian_pose"] = self.target_cartesian_pose.to_dict()
        return d

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotAction:
        pose_data = data.get("target_cartesian_pose")
        target_pose = Pose3D.from_dict(pose_data) if pose_data else None
        return cls(
            action_type=ActionType(data["action_type"]),
            source=ControlSource(data["source"]),
            timestamp=float(data["timestamp"]),
            target_joint_positions=data.get("target_joint_positions"),
            target_joint_deltas=data.get("target_joint_deltas"),
            target_cartesian_delta=tuple(data["target_cartesian_delta"]) if data.get("target_cartesian_delta") else None,
            target_cartesian_pose=target_pose,
            unit=JointUnit(data.get("unit", JointUnit.RADIAN.value)),
            gripper_unit=JointUnit(data.get("gripper_unit", JointUnit.RANGE_0_100.value)),
        )


@dataclass(frozen=True)
class RobotObservation:
    """多模态对齐观测状态 (某一时间切片的统一视图).
    
    Attributes:
        timestamp: 观测聚合基准时间戳
        joint_state: 关节传感器反馈
        tcp_state: 末端空间位姿 (可选)
        camera_frames: 多相机帧元数据 {camera_id: CameraFrameRef}
    """
    timestamp: float
    joint_state: JointState
    tcp_state: Pose3D | None = None
    camera_frames: dict[str, CameraFrameRef] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "joint_state": self.joint_state.to_dict(),
            "tcp_state": self.tcp_state.to_dict() if self.tcp_state else None,
            "camera_frames": {k: v.to_dict() for k, v in self.camera_frames.items()},
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotObservation:
        tcp_data = data.get("tcp_state")
        cameras_data = data.get("camera_frames", {})
        return cls(
            timestamp=float(data["timestamp"]),
            joint_state=JointState.from_dict(data["joint_state"]),
            tcp_state=Pose3D.from_dict(tcp_data) if tcp_data else None,
            camera_frames={k: CameraFrameRef.from_dict(v) for k, v in cameras_data.items()},
        )


@dataclass(frozen=True)
class SafetyReport:
    """安全与健康状态报告.
    
    Attributes:
        timestamp: 检查时间戳
        system_state: 当前系统状态机状态
        estop_active: 是否处于急停锁定
        limit_triggered: 是否触发限位保护
        tracking_error_exceeded: 跟随误差是否过大 (失步/堵转)
        active_source: 当前持有控制权的来源
        message: 状态描述或异常报警文本
    """
    timestamp: float
    system_state: SystemState
    estop_active: bool = False
    limit_triggered: bool = False
    tracking_error_exceeded: bool = False
    active_source: ControlSource | None = None
    message: str = ""

    def is_safe_for_motion(self) -> bool:
        """是否允许执行运动指令."""
        return (
            not self.estop_active
            and not self.limit_triggered
            and not self.tracking_error_exceeded
            and self.system_state in (SystemState.TELEOP, SystemState.PLAYBACK, SystemState.RECORDING)
        )

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["system_state"] = self.system_state.value
        d["active_source"] = self.active_source.value if self.active_source else None
        return d

