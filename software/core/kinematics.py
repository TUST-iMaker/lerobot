"""SO-ARM101 机械臂正逆运动学计算模块 (对齐官方 so101_new_calib.urdf 与 RobotKinematics).

设计原则 (决策点 A):
1. 当运行环境可导入官方 `lerobot.model.kinematics.RobotKinematics` (`placo`) 时 (如 Ubuntu),
   优先委托官方 `RobotKinematics` 对 `so101_new_calib.urdf` 执行计算;
2. 当运行环境缺少 `placo` (如 Windows) 时, 使用直接解析官方
   `repos/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` XML 的纯 NumPy 3D SE(3) 连杆链
   与多初值阻尼最小二乘 (DLS) Jacobian IK 求解器, 确保零位 TCP、旋转轴向 (含 shoulder_pan [0,0,-1])
   与关节软限位 100% 与官方 URDF 保持一致.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from core.data_types import (
    SO101_ARM_JOINT_NAMES,
    JointState,
    JointUnit,
    Pose3D,
    deg_to_rad,
)

logger = logging.getLogger(__name__)

DEFAULT_SO101_URDF_PATH: Path = (
    Path(__file__).resolve().parents[2]
    / "repos"
    / "SO-ARM100"
    / "Simulation"
    / "SO101"
    / "so101_new_calib.urdf"
)


@dataclass(frozen=True)
class KinematicsConfig:
    """SO-ARM101 机械臂 URDF 运动学配置与兼容参数."""

    urdf_path: Path | str = DEFAULT_SO101_URDF_PATH
    target_frame_name: str = "gripper_frame_link"
    joint_names: tuple[str, ...] = SO101_ARM_JOINT_NAMES
    use_placo_if_available: bool = True
    position_weight: float = 1.0
    orientation_weight: float = 0.01
    ik_max_iters: int = 100
    ik_damping: float = 1e-3
    ik_pos_tolerance_m: float = 0.005

    # 兼容字段 (对齐 so101_new_calib.urdf 等效几何尺寸, 单位: 米)
    base_height_m: float = 0.1166
    upper_arm_m: float = 0.1160
    forearm_m: float = 0.1350
    gripper_offset_m: float = 0.1605

    # 关节传感器读数 -> 几何坐标系的符号对齐系数
    sign_shoulder_pan: float = 1.0
    sign_shoulder_lift: float = 1.0
    sign_elbow_flex: float = 1.0
    sign_wrist_flex: float = 1.0
    sign_wrist_roll: float = 1.0


@dataclass(frozen=True)
class _URDFJointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF 标准 RPY 旋转矩阵: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    """绕单位向量 axis 旋转 angle (rad) 的 3x3 旋转矩阵."""
    if abs(angle) < 1e-15:
        return np.eye(3, dtype=np.float64)
    k = axis / np.linalg.norm(axis)
    kx, ky, kz = float(k[0]), float(k[1]), float(k[2])
    K = np.array(
        [
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def _rotation_matrix_to_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """Shepperd 方法将 3x3 旋转矩阵无奇异地转换为四元数 (qx, qy, qz, qw)."""
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (float(R[2, 1]) - float(R[1, 2])) / S
        qy = (float(R[0, 2]) - float(R[2, 0])) / S
        qz = (float(R[1, 0]) - float(R[0, 1])) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + float(R[0, 0]) - float(R[1, 1]) - float(R[2, 2])) * 2.0
        qw = (float(R[2, 1]) - float(R[1, 2])) / S
        qx = 0.25 * S
        qy = (float(R[0, 1]) + float(R[1, 0])) / S
        qz = (float(R[0, 2]) + float(R[2, 0])) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + float(R[1, 1]) - float(R[0, 0]) - float(R[2, 2])) * 2.0
        qw = (float(R[0, 2]) - float(R[2, 0])) / S
        qx = (float(R[0, 1]) + float(R[1, 0])) / S
        qy = 0.25 * S
        qz = (float(R[1, 2]) + float(R[2, 1])) / S
    else:
        S = math.sqrt(1.0 + float(R[2, 2]) - float(R[0, 0]) - float(R[1, 1])) * 2.0
        qw = (float(R[1, 0]) - float(R[0, 1])) / S
        qx = (float(R[0, 2]) + float(R[2, 0])) / S
        qy = (float(R[1, 2]) + float(R[2, 1])) / S
        qz = 0.25 * S
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (qx, qy, qz, qw)


class _NumpyURDFKinematics:
    """直接解析 URDF XML 的纯 NumPy SE(3) 正逆运动学引擎."""

    def __init__(
        self,
        urdf_path: Path | str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: tuple[str, ...] = SO101_ARM_JOINT_NAMES,
    ) -> None:
        self.urdf_path = Path(urdf_path)
        self.target_frame_name = target_frame_name
        self.joint_names = tuple(joint_names)
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"未找到官方 SO-101 URDF 文件: {self.urdf_path}")

        self.chain = self._parse_chain(self.urdf_path, target_frame_name)
        self._joint_index: dict[str, int] = {name: i for i, name in enumerate(self.joint_names)}

        self.joint_limits_rad: dict[str, tuple[float, float]] = {}
        for spec in self.chain:
            if spec.name in self._joint_index:
                self.joint_limits_rad[spec.name] = (spec.lower, spec.upper)

        self.lower_bounds = np.array(
            [self.joint_limits_rad.get(n, (-math.pi, math.pi))[0] for n in self.joint_names],
            dtype=np.float64,
        )
        self.upper_bounds = np.array(
            [self.joint_limits_rad.get(n, (-math.pi, math.pi))[1] for n in self.joint_names],
            dtype=np.float64,
        )

        # 预计算零位末端旋转矩阵，用于从 yaw/pitch 构造参考姿态
        T_zero, _, _ = self.forward_kinematics_details(np.zeros(len(self.joint_names), dtype=np.float64))
        self.R_ee_zero = T_zero[:3, :3].copy()

    @staticmethod
    def _parse_chain(urdf_path: Path, target_frame_name: str) -> list[_URDFJointSpec]:
        tree = ET.parse(str(urdf_path))
        root = tree.getroot()
        by_child: dict[str, _URDFJointSpec] = {}
        parents: set[str] = set()
        children: set[str] = set()

        for elem in root.findall("joint"):
            jname = elem.attrib["name"]
            jtype = elem.attrib.get("type", "fixed")
            parent_el = elem.find("parent")
            child_el = elem.find("child")
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.attrib["link"]
            child = child_el.attrib["link"]
            parents.add(parent)
            children.add(child)

            origin_el = elem.find("origin")
            xyz = np.zeros(3, dtype=np.float64)
            rpy = np.zeros(3, dtype=np.float64)
            if origin_el is not None:
                if "xyz" in origin_el.attrib:
                    xyz = np.array([float(x) for x in origin_el.attrib["xyz"].split()], dtype=np.float64)
                if "rpy" in origin_el.attrib:
                    rpy = np.array([float(x) for x in origin_el.attrib["rpy"].split()], dtype=np.float64)

            axis_el = elem.find("axis")
            axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            if axis_el is not None and "xyz" in axis_el.attrib:
                raw_axis = np.array([float(x) for x in axis_el.attrib["xyz"].split()], dtype=np.float64)
                norm = np.linalg.norm(raw_axis)
                if norm > 0.0:
                    axis = raw_axis / norm

            limit_el = elem.find("limit")
            lower, upper = -math.pi, math.pi
            if limit_el is not None:
                lower = float(limit_el.attrib.get("lower", -math.pi))
                upper = float(limit_el.attrib.get("upper", math.pi))

            by_child[child] = _URDFJointSpec(
                name=jname,
                joint_type=jtype,
                parent=parent,
                child=child,
                xyz=xyz,
                rpy=rpy,
                axis=axis,
                lower=lower,
                upper=upper,
            )

        if target_frame_name not in by_child:
            raise ValueError(f"URDF {urdf_path} 中未找到目标帧 {target_frame_name}")

        root_links = parents - children
        root_link = "base_link" if "base_link" in root_links else next(iter(root_links))

        chain: list[_URDFJointSpec] = []
        cur = target_frame_name
        visited: set[str] = set()
        while cur != root_link:
            if cur in visited or cur not in by_child:
                raise ValueError(f"URDF 运动学链无法从 {root_link} 到达 {target_frame_name}")
            visited.add(cur)
            spec = by_child[cur]
            chain.append(spec)
            cur = spec.parent
        chain.reverse()
        return chain

    def forward_kinematics_details(
        self, q_rad: np.ndarray
    ) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray]:
        """计算 4x4 末端变换矩阵, 各关节世界坐标, 与 6xN 几何雅可比矩阵."""
        T = np.eye(4, dtype=np.float64)
        joint_positions: dict[str, np.ndarray] = {"base": np.zeros(3, dtype=np.float64)}
        n_joints = len(self.joint_names)
        joint_origins: list[np.ndarray | None] = [None] * n_joints
        joint_axes: list[np.ndarray | None] = [None] * n_joints

        for spec in self.chain:
            T_origin = np.eye(4, dtype=np.float64)
            T_origin[:3, :3] = _rpy_matrix(float(spec.rpy[0]), float(spec.rpy[1]), float(spec.rpy[2]))
            T_origin[:3, 3] = spec.xyz
            T = T @ T_origin

            idx = self._joint_index.get(spec.name)
            if idx is not None:
                p_j = T[:3, 3].copy()
                w_j = (T[:3, :3] @ spec.axis).copy()
                joint_positions[spec.name] = p_j
                joint_origins[idx] = p_j
                joint_axes[idx] = w_j

                if spec.joint_type in ("revolute", "continuous"):
                    T_rot = np.eye(4, dtype=np.float64)
                    T_rot[:3, :3] = _rodrigues(spec.axis, float(q_rad[idx]))
                    T = T @ T_rot

        p_ee = T[:3, 3].copy()
        joint_positions["tcp"] = p_ee

        J = np.zeros((6, n_joints), dtype=np.float64)
        for i in range(n_joints):
            p_j = joint_origins[i]
            w_j = joint_axes[i]
            if p_j is not None and w_j is not None:
                J[:3, i] = np.cross(w_j, p_ee - p_j)
                J[3:, i] = w_j

        return T, joint_positions, J

    def inverse_kinematics_dls(
        self,
        target_xyz: np.ndarray,
        q_init: np.ndarray,
        R_des: np.ndarray | None = None,
        pos_w: float = 1.0,
        ori_w: float = 0.01,
        max_iters: int = 100,
        damping: float = 1e-3,
        active_mask: np.ndarray | None = None,
        desired_pitch_rad: float | None = None,
    ) -> tuple[np.ndarray, float]:
        """阻尼最小二乘 (DLS) Jacobian 逆解."""
        q = np.clip(np.asarray(q_init, dtype=np.float64).copy(), self.lower_bounds, self.upper_bounds)
        n_joints = len(self.joint_names)
        mask = np.ones(n_joints, dtype=bool) if active_mask is None else np.asarray(active_mask, dtype=bool)

        best_q = q.copy()
        best_err = float("inf")

        for _ in range(max_iters):
            T_cur, _, J = self.forward_kinematics_details(q)
            p_cur = T_cur[:3, 3]
            pos_err_vec = target_xyz - p_cur
            pos_err = float(np.linalg.norm(pos_err_vec))

            if pos_err < best_err:
                best_err = pos_err
                best_q = q.copy()

            if pos_err < 1e-5 and ori_w <= 0.0:
                break

            if ori_w > 0.0 and desired_pitch_rad is not None:
                # 仅约束末端俯仰角 pitch (f_z = -sin(pitch)), 释放侧向偏置耦合
                f_cur = T_cur[:3, 2]
                cur_pitch = math.atan2(-float(f_cur[2]), math.hypot(float(f_cur[0]), float(f_cur[1])))
                pitch_err = float(desired_pitch_rad) - cur_pitch
                if pos_err < 1e-6 and abs(pitch_err) < 1e-5:
                    break
                fz_err = -math.sin(float(desired_pitch_rad)) - float(f_cur[2])
                # d(f_z)/dq_i = (w_i x f_cur)_z, 注意 sin(pitch) = -f_z
                J_fz = np.array(
                    [float(np.cross(J[3:, i], f_cur)[2]) for i in range(n_joints)],
                    dtype=np.float64,
                ).reshape(1, n_joints)
                err_vec = np.concatenate([pos_w * pos_err_vec, np.array([ori_w * fz_err], dtype=np.float64)])
                Jw = np.vstack([pos_w * J[:3, :], ori_w * J_fz])
            elif ori_w > 0.0 and R_des is not None:
                R_cur = T_cur[:3, :3]
                ori_err_vec = 0.5 * (
                    np.cross(R_cur[:, 0], R_des[:, 0])
                    + np.cross(R_cur[:, 1], R_des[:, 1])
                    + np.cross(R_cur[:, 2], R_des[:, 2])
                )
                ori_norm = float(np.linalg.norm(ori_err_vec))
                if pos_err < 1e-5 and ori_norm < 1e-4:
                    break
                err_vec = np.concatenate([pos_w * pos_err_vec, ori_w * ori_err_vec])
                Jw = np.vstack([pos_w * J[:3, :], ori_w * J[3:, :]])
            else:
                err_vec = pos_w * pos_err_vec
                Jw = pos_w * J[:3, :]

            Jw_act = Jw[:, mask]
            n_act = int(np.sum(mask))
            H = Jw_act.T @ Jw_act + (damping * damping) * np.eye(n_act, dtype=np.float64)
            g = Jw_act.T @ err_vec
            dq_act = np.linalg.solve(H, g)

            max_step = float(np.max(np.abs(dq_act))) if n_act > 0 else 0.0
            if max_step > 0.35:
                dq_act = dq_act * (0.35 / max_step)

            dq = np.zeros(n_joints, dtype=np.float64)
            dq[mask] = dq_act
            q_next = np.clip(q + dq, self.lower_bounds, self.upper_bounds)
            if float(np.linalg.norm(q_next - q)) < 1e-8 and pos_err < 1e-4:
                q = q_next
                break
            q = q_next

        T_fin, _, _ = self.forward_kinematics_details(q)
        fin_err = float(np.linalg.norm(target_xyz - T_fin[:3, 3]))
        if fin_err <= best_err:
            return q, fin_err
        return best_q, best_err


class SO101Kinematics:
    """SO-ARM101 运动学求解器 (对齐 so101_new_calib.urdf 与官方 RobotKinematics)."""

    def __init__(self, config: KinematicsConfig | None = None) -> None:
        self.config = config or KinematicsConfig()
        self._numpy_solver = _NumpyURDFKinematics(
            urdf_path=self.config.urdf_path,
            target_frame_name=self.config.target_frame_name,
            joint_names=self.config.joint_names,
        )
        self._placo_solver: Any = None
        if self.config.use_placo_if_available:
            try:
                from lerobot.model.kinematics import RobotKinematics

                self._placo_solver = RobotKinematics(
                    urdf_path=str(self.config.urdf_path),
                    target_frame_name=self.config.target_frame_name,
                    joint_names=list(self.config.joint_names),
                )
            except ImportError:
                self._placo_solver = None
            except Exception as exc:
                logger.warning("初始化 RobotKinematics (placo) 失败，回退纯 NumPy URDF 求解器: %s", exc)
                self._placo_solver = None

    @property
    def backend_name(self) -> str:
        return "placo" if self._placo_solver is not None else "numpy_urdf"

    @property
    def joint_limits_rad(self) -> dict[str, tuple[float, float]]:
        return dict(self._numpy_solver.joint_limits_rad)

    def _extract_q_rad(
        self,
        joint_positions: dict[str, float],
        unit: JointUnit = JointUnit.RADIAN,
    ) -> np.ndarray:
        signs = (
            self.config.sign_shoulder_pan,
            self.config.sign_shoulder_lift,
            self.config.sign_elbow_flex,
            self.config.sign_wrist_flex,
            self.config.sign_wrist_roll,
        )
        vals = []
        for idx, name in enumerate(self.config.joint_names):
            raw = float(joint_positions.get(name, 0.0))
            rad = deg_to_rad(raw) if unit == JointUnit.DEGREE else raw
            sign = signs[idx] if idx < len(signs) else 1.0
            vals.append(sign * rad)
        return np.array(vals, dtype=np.float64)

    def _apply_inverse_signs(self, q_rad: np.ndarray) -> dict[str, float]:
        signs = (
            self.config.sign_shoulder_pan,
            self.config.sign_shoulder_lift,
            self.config.sign_elbow_flex,
            self.config.sign_wrist_flex,
            self.config.sign_wrist_roll,
        )
        out: dict[str, float] = {}
        for idx, name in enumerate(self.config.joint_names):
            sign = signs[idx] if idx < len(signs) else 1.0
            out[name] = float(q_rad[idx]) / sign
        return out

    def forward_kinematics_matrix(
        self,
        joint_positions: dict[str, float],
        unit: JointUnit = JointUnit.RADIAN,
    ) -> np.ndarray:
        """返回从 base_link 到 gripper_frame_link 的 4x4 齐次变换矩阵."""
        q_rad = self._extract_q_rad(joint_positions, unit=unit)
        if self._placo_solver is not None:
            return np.asarray(self._placo_solver.forward_kinematics(np.rad2deg(q_rad)), dtype=np.float64)
        T, _, _ = self._numpy_solver.forward_kinematics_details(q_rad)
        return T

    def forward_kinematics_links(
        self,
        joint_positions: dict[str, float],
        unit: JointUnit = JointUnit.RADIAN,
    ) -> dict[str, tuple[float, float, float]]:
        """返回各关节与 TCP 在底座坐标系下的 3D 坐标 (米)."""
        q_rad = self._extract_q_rad(joint_positions, unit=unit)
        _, joint_positions_m, _ = self._numpy_solver.forward_kinematics_details(q_rad)
        return {
            name: (float(vec[0]), float(vec[1]), float(vec[2]))
            for name, vec in joint_positions_m.items()
        }

    def forward_kinematics(
        self,
        joint_positions: dict[str, float],
        unit: JointUnit = JointUnit.RADIAN,
    ) -> tuple[tuple[float, float, float], float, float]:
        """正运动学计算 (FK): 返回 ((x, y, z), pitch_rad, roll_rad)."""
        q_rad = self._extract_q_rad(joint_positions, unit=unit)
        if self._placo_solver is not None:
            T = np.asarray(self._placo_solver.forward_kinematics(np.rad2deg(q_rad)), dtype=np.float64)
        else:
            T, _, _ = self._numpy_solver.forward_kinematics_details(q_rad)

        x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
        # gripper_frame_link 的 +Z 轴在零位指向世界 +X 前方
        f = T[:3, 2]
        pitch_rad = math.atan2(-float(f[2]), math.hypot(float(f[0]), float(f[1])))
        roll_rad = float(q_rad[4]) if len(q_rad) > 4 else 0.0
        return (x, y, z), pitch_rad, roll_rad

    def compute_tcp_pose(self, state: JointState) -> Pose3D:
        """根据当前 JointState 生成标准 Pose3D 对象 (基于 3D SE(3) 矩阵转四元数)."""
        T = self.forward_kinematics_matrix(state.positions, unit=state.unit)
        qx, qy, qz, qw = _rotation_matrix_to_xyzw(T[:3, :3])
        return Pose3D(
            timestamp=state.timestamp,
            frame_id="robot_base",
            position_m=(round(float(T[0, 3]), 5), round(float(T[1, 3]), 5), round(float(T[2, 3]), 5)),
            orientation_xyzw=(round(qx, 5), round(qy, 5), round(qz, 5), round(qw, 5)),
            valid=True,
            kinematics_version=f"so101_urdf_{self.backend_name}_v1",
        )

    def inverse_kinematics(
        self,
        target_xyz: tuple[float, float, float],
        current_joint_positions: dict[str, float] | None = None,
        desired_pitch_rad: float | None = None,
        preserve_wrist_roll: float = 0.0,
        preserve_gripper: float = 0.0,
        desired_ee_pose: np.ndarray | None = None,
        position_weight: float | None = None,
        orientation_weight: float | None = None,
    ) -> tuple[bool, dict[str, float] | None, str]:
        """逆运动学计算 (IK): 优先使用 placo (若可用) + 纯 NumPy URDF 多初值 DLS 求解."""
        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
        target_vec = np.array([x, y, z], dtype=np.float64)

        if current_joint_positions is not None:
            q_init = self._extract_q_rad(current_joint_positions, unit=JointUnit.RADIAN)
        else:
            q_init = np.zeros(len(self.config.joint_names), dtype=np.float64)

        pos_w = self.config.position_weight if position_weight is None else float(position_weight)
        ori_w_default = self.config.orientation_weight if orientation_weight is None else float(orientation_weight)

        # 构造目标姿态
        if desired_ee_pose is not None:
            T_des = np.asarray(desired_ee_pose, dtype=np.float64)
            target_vec = T_des[:3, 3].copy()
            R_des = T_des[:3, :3].copy()
            pitch_des: float | None = None
            ori_w = ori_w_default
        else:
            dist_xy = math.hypot(x, y)
            yaw = math.atan2(y, x) if dist_xy >= 0.01 else -float(q_init[0])
            if desired_pitch_rad is not None:
                pitch = float(desired_pitch_rad)
                pitch_des = pitch
                ori_w = ori_w_default
            elif current_joint_positions is not None:
                _, pitch, _ = self.forward_kinematics(current_joint_positions, unit=JointUnit.RADIAN)
                pitch_des = pitch
                ori_w = ori_w_default
            else:
                pitch = 0.0
                pitch_des = None
                ori_w = 0.0

            R_des = _rpy_matrix(0.0, pitch, yaw) @ self._numpy_solver.R_ee_zero
            T_des = np.eye(4, dtype=np.float64)
            T_des[:3, :3] = R_des
            T_des[:3, 3] = target_vec

        best_q: np.ndarray | None = None
        best_err = float("inf")

        # 1. 若可用且为全 6D 姿态目标, 优先尝试官方 RobotKinematics (placo)
        #    注意: 纯 XYZ 位置 IK (desired_ee_pose is None) 必须锁定 #5 wrist_roll, 防止解算器利用 7.9mm 偏心乱转腕部
        if self._placo_solver is not None and desired_ee_pose is not None:
            try:
                q_seed_deg = np.rad2deg(q_init)
                q_sol_deg = np.asarray(
                    self._placo_solver.inverse_kinematics(
                        q_seed_deg,
                        T_des,
                        position_weight=pos_w,
                        orientation_weight=ori_w,
                        max_iters=8,
                    ),
                    dtype=np.float64,
                )
                q_sol_rad = np.clip(
                    np.deg2rad(q_sol_deg),
                    self._numpy_solver.lower_bounds,
                    self._numpy_solver.upper_bounds,
                )
                T_check = np.asarray(
                    self._placo_solver.forward_kinematics(np.rad2deg(q_sol_rad)),
                    dtype=np.float64,
                )
                err = float(np.linalg.norm(target_vec - T_check[:3, 3]))
                if err < best_err:
                    best_err = err
                    best_q = q_sol_rad
            except Exception as exc:
                logger.debug("placo IK 异常, 回退 NumPy DLS: %s", exc)

        # 2. 纯 NumPy URDF DLS 求解与多初值回退
        tol = self.config.ik_pos_tolerance_m
        if best_err > tol:
            if desired_ee_pose is None:
                # 锁定 wrist_roll 为 preserve_wrist_roll, 用前 4 关节精确求解 (x, y, z, pitch)
                roll_idx = 4
                lo_r, hi_r = self._numpy_solver.lower_bounds[roll_idx], self._numpy_solver.upper_bounds[roll_idx]
                q_seed = q_init.copy()
                q_seed[roll_idx] = max(lo_r, min(hi_r, float(preserve_wrist_roll) * self.config.sign_wrist_roll))
                active_4 = np.array([True, True, True, True, False], dtype=bool)
            else:
                q_seed = q_init.copy()
                active_4 = np.ones(len(self.config.joint_names), dtype=bool)

            # 2a. 首选带姿态权重求解
            if ori_w > 0.0:
                q_try, err_try = self._numpy_solver.inverse_kinematics_dls(
                    target_xyz=target_vec,
                    q_init=q_seed,
                    R_des=R_des if desired_ee_pose is not None else None,
                    pos_w=pos_w,
                    ori_w=max(ori_w, 0.1) if desired_ee_pose is None else ori_w,
                    max_iters=self.config.ik_max_iters,
                    damping=self.config.ik_damping,
                    active_mask=active_4,
                    desired_pitch_rad=pitch_des,
                )
                if err_try < best_err:
                    best_err = err_try
                    best_q = q_try

            # 2b. 若姿态约束导致超出限位或未收敛, 降级为仅位置约束 (adaptive pitch)
            if best_err > tol:
                q_try, err_try = self._numpy_solver.inverse_kinematics_dls(
                    target_xyz=target_vec,
                    q_init=q_seed,
                    pos_w=pos_w,
                    ori_w=0.0,
                    max_iters=self.config.ik_max_iters,
                    damping=self.config.ik_damping,
                    active_mask=active_4,
                )
                if err_try < best_err:
                    best_err = err_try
                    best_q = q_try

            # 2c. 多初值回退 (注意 shoulder_pan 世界轴为 [0, 0, -1], 故 pan_g = -atan2(y, x))
            if best_err > tol:
                pan_g = -math.atan2(y, x) if math.hypot(x, y) >= 1e-4 else float(q_seed[0])
                pan_opp = (pan_g + math.pi) % (2.0 * math.pi) - math.pi
                roll_val = q_seed[4] if len(q_seed) > 4 else 0.0
                seeds = [
                    np.array([pan_g, 0.3, 0.3, 0.3, roll_val], dtype=np.float64),
                    np.array([pan_g, 0.6, -0.5, 0.5, roll_val], dtype=np.float64),
                    np.array([pan_g, -0.5, 0.8, 0.4, roll_val], dtype=np.float64),
                    np.array([pan_opp, -0.6, -0.5, -0.5, roll_val], dtype=np.float64),
                    np.array([0.0, 0.0, 0.0, 0.0, roll_val], dtype=np.float64),
                ]
                for seed in seeds:
                    if ori_w > 0.0 and pitch_des is not None:
                        q_try, err_try = self._numpy_solver.inverse_kinematics_dls(
                            target_xyz=target_vec,
                            q_init=seed,
                            pos_w=pos_w,
                            ori_w=max(ori_w, 0.1),
                            max_iters=self.config.ik_max_iters,
                            damping=self.config.ik_damping,
                            active_mask=active_4,
                            desired_pitch_rad=pitch_des,
                        )
                        if err_try < best_err:
                            best_err = err_try
                            best_q = q_try
                        if best_err <= 1e-5:
                            break
                    q_try, err_try = self._numpy_solver.inverse_kinematics_dls(
                        target_xyz=target_vec,
                        q_init=seed,
                        pos_w=pos_w,
                        ori_w=0.0,
                        max_iters=self.config.ik_max_iters,
                        damping=self.config.ik_damping,
                        active_mask=active_4,
                    )
                    if err_try < best_err:
                        best_err = err_try
                        best_q = q_try
                    if best_err <= 1e-4:
                        break

        if best_q is None or best_err > tol:
            return (
                False,
                None,
                f"目标超出最大臂展或不可达 (位置误差 {best_err * 1000.0:.1f}mm > 阈值 {tol * 1000.0:.1f}mm)",
            )

        joint_targets = self._apply_inverse_signs(best_q)
        if desired_ee_pose is None:
            lo_r, hi_r = self._numpy_solver.lower_bounds[4], self._numpy_solver.upper_bounds[4]
            clamped_roll = max(lo_r, min(hi_r, float(preserve_wrist_roll) * self.config.sign_wrist_roll))
            joint_targets["wrist_roll"] = clamped_roll / self.config.sign_wrist_roll
        joint_targets["gripper"] = float(preserve_gripper)
        return True, joint_targets, "逆运动学 URDF 求解成功"

    def inverse_kinematics_clamped(
        self,
        target_xyz: tuple[float, float, float],
        current_joint_positions: dict[str, float] | None = None,
        desired_pitch_rad: float | None = None,
        preserve_wrist_roll: float = 0.0,
        preserve_gripper: float = 0.0,
        orientation_weight: float | None = None,
    ) -> tuple[dict[str, float], bool, str]:
        """纯 XYZ 位置逆解 (锁定 #5 wrist_roll): 超出物理工作空间时自动钳制到最近可达边界."""
        ok, joints, msg = self.inverse_kinematics(
            target_xyz=target_xyz,
            current_joint_positions=current_joint_positions,
            desired_pitch_rad=desired_pitch_rad,
            preserve_wrist_roll=preserve_wrist_roll,
            preserve_gripper=preserve_gripper,
            desired_ee_pose=None,
            orientation_weight=orientation_weight,
        )
        if ok and joints is not None:
            return joints, False, msg

        # 超出物理工作空间: 锁定 wrist_roll, 沿最优接近路径投影到 URDF 可达边界
        x, y, z = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
        target_vec = np.array([x, y, z], dtype=np.float64)
        q_seed = (
            self._extract_q_rad(current_joint_positions, unit=JointUnit.RADIAN)
            if current_joint_positions is not None
            else np.zeros(len(self.config.joint_names), dtype=np.float64)
        )
        roll_idx = 4
        lo_r, hi_r = self._numpy_solver.lower_bounds[roll_idx], self._numpy_solver.upper_bounds[roll_idx]
        locked_roll = max(lo_r, min(hi_r, float(preserve_wrist_roll) * self.config.sign_wrist_roll))
        q_seed[roll_idx] = locked_roll
        active_4 = np.array([True, True, True, True, False], dtype=bool)

        best_q, best_err = self._numpy_solver.inverse_kinematics_dls(
            target_xyz=target_vec,
            q_init=q_seed,
            pos_w=self.config.position_weight,
            ori_w=0.0,
            max_iters=self.config.ik_max_iters,
            damping=max(self.config.ik_damping, 5e-3),
            active_mask=active_4,
        )
        pan_g = -math.atan2(y, x) if math.hypot(x, y) >= 1e-4 else float(q_seed[0])
        pan_seed = np.array([pan_g, 0.0, 0.0, 0.0, locked_roll], dtype=np.float64)
        q_try, err_try = self._numpy_solver.inverse_kinematics_dls(
            target_xyz=target_vec,
            q_init=pan_seed,
            pos_w=self.config.position_weight,
            ori_w=0.0,
            max_iters=self.config.ik_max_iters,
            damping=max(self.config.ik_damping, 5e-3),
            active_mask=active_4,
        )
        if err_try < best_err:
            best_q, best_err = q_try, err_try

        joint_targets = self._apply_inverse_signs(best_q)
        joint_targets["wrist_roll"] = locked_roll / self.config.sign_wrist_roll
        joint_targets["gripper"] = float(preserve_gripper)
        return (
            joint_targets,
            True,
            f"已触达 SO-101 物理工作空间边界 (截断误差 {best_err * 1000.0:.1f}mm)",
        )


def solve_horizontal_planar_reach(
    target_R_gripper: float,
    locked_Z_gripper: float,
    kin_cfg: KinematicsConfig | None = None,
    sign_lift: float = 1.0,
    sign_elbow: float = 1.0,
    sign_wrist: float = 1.0,
) -> tuple[bool, float, float, float, str]:
    """保持夹爪指尖高度 locked_Z_gripper 恒定与水平前向姿态的平推逆解 (对齐官方 URDF)."""
    cfg = kin_cfg or KinematicsConfig(
        sign_shoulder_lift=sign_lift,
        sign_elbow_flex=sign_elbow,
        sign_wrist_flex=sign_wrist,
    )
    kin = SO101Kinematics(cfg)
    ok, joints, msg = kin.inverse_kinematics(
        target_xyz=(float(target_R_gripper), 0.0, float(locked_Z_gripper)),
        desired_pitch_rad=0.0,
    )
    if not ok or joints is None:
        return False, 0.0, 0.0, 0.0, msg
    return (
        True,
        float(joints["shoulder_lift"]),
        float(joints["elbow_flex"]),
        float(joints["wrist_flex"]),
        "求解成功",
    )
