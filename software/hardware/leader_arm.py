"""SO-ARM101 Leader 示教主臂硬件驱动适配器.

职责:
1. 封装 Leader 示教臂的连接与状态读取;
2. 零扭矩保证: 连接后严格禁用舵机扭矩，允许操作员轻快拖动示教;
3. 提供 read_action() 接口，将主臂当前关节位姿直接封装为 RobotAction (source=LEADER);
4. 实时监测采样延迟与数据有效性.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from core.data_types import (
    SO101_GRIPPER_JOINT_NAME,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    deg_to_rad,
)
from hardware.base import BaseArmAdapter
from hardware.config import ArmConfig

logger = logging.getLogger(__name__)


class SO101LeaderArm(BaseArmAdapter):
    """SO-ARM101 主臂 (示教器) 硬件适配器."""

    def __init__(self, config: ArmConfig) -> None:
        super().__init__(config)
        self._leader: Any = None
        self._bus_lock = threading.RLock()
        self._torque_enabled: bool = False

    def connect(self) -> None:
        """连接示教主臂. 强制释放扭矩以便人工顺应拖拽."""
        if self._connected:
            logger.info("[%s] 主臂已连接", self.config.robot_id)
            return

        self.load_calibration()

        cal_dir = self.config.calibration_dir or self.config.calibration_fpath.parent

        try:
            from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
            driver_cfg = SO101LeaderConfig(
                port=self.config.port,
                id=self.config.robot_id,
                calibration_dir=cal_dir,
                use_degrees=self.config.use_degrees,
            )
            leader = SO101Leader(driver_cfg)
            leader.connect(calibrate=False)
            try:
                hw_cal = bool(leader.is_calibrated)
            except Exception as cal_exc:
                hw_cal = False
                logger.warning("[%s] 检查主臂硬件校准状态异常: %s", self.config.robot_id, cal_exc)
            self._hw_calibrated = hw_cal
            if not hw_cal:
                logger.warning(
                    "[%s] 主臂舵机 EEPROM 与校准文件 %s 不一致或未校准!",
                    self.config.robot_id,
                    getattr(leader, "calibration_fpath", self.config.calibration_fpath),
                )
            # 确保主臂扭矩处于禁用状态
            try:
                leader.bus.disable_torque()
            except Exception:
                pass
            self._leader = leader
        except ImportError:
            # 兼容：如果当前环境未安装特定 teleoperator 模块，尝试使用通用 Follower 类并强制释放力矩
            logger.info("[%s] 未找到 SO101Leader 驱动，尝试通过通用驱动以只读/零力矩模式连接...", self.config.robot_id)
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            driver_cfg = SO101FollowerConfig(
                port=self.config.port,
                id=self.config.robot_id,
                calibration_dir=cal_dir,
                disable_torque_on_disconnect=True,
                use_degrees=self.config.use_degrees,
            )
            leader = SO101Follower(driver_cfg)
            leader.bus.connect()
            try:
                hw_cal = bool(leader.is_calibrated)
            except Exception:
                hw_cal = False
            self._hw_calibrated = hw_cal
            leader.bus.disable_torque()
            try:
                leader.configure()
            finally:
                # SOFollower.configure() 退出 with torque_disabled() 时会恢复使能，主臂必须再次泄力
                leader.bus.disable_torque()
            self._leader = leader

        self._connected = True
        logger.info("[%s] 主臂连接成功! 扭矩已关闭，可自由手动拖动示教", self.config.robot_id)

    def disconnect(self, keep_torque: bool = False) -> None:
        """断开主臂连接."""
        if not self._connected or self._leader is None:
            return

        logger.info("[%s] 正在断开主臂...", self.config.robot_id)
        try:
            if hasattr(self._leader, "bus"):
                self._leader.bus.disable_torque()
            self._leader.disconnect()
        except Exception as exc:
            logger.error("[%s] 断开主臂异常: %s", self.config.robot_id, exc)
        finally:
            self._leader = None
            self._connected = False

    def read_state(self) -> JointState:
        """读取主臂当前关节观测数据."""
        if not self.is_connected or self._leader is None:
            raise RuntimeError(f"[{self.config.robot_id}] 主臂未连接，无法读取")

        t0 = time.perf_counter()
        try:
            # 尝试调用 teleoperator 的 get_action 或 robot 的 get_observation
            with self._bus_lock:
                if hasattr(self._leader, "get_action"):
                    obs = self._leader.get_action()
                else:
                    obs = self._leader.get_observation()
            latency_ms = (time.perf_counter() - t0) * 1000.0
            self._record_read_latency(latency_ms)
        except Exception as exc:
            self._health.consecutive_read_errors += 1
            self._health.last_error_message = f"读取主臂异常: {exc}"
            logger.error("[%s] 读取主臂失败: %s", self.config.robot_id, exc)
            raise

        timestamp = time.monotonic()
        raw_positions: dict[str, float] = {}
        for key, val in obs.items():
            if not key.endswith(".pos"):
                continue
            name = key[:-4]
            try:
                raw_positions[name] = float(val)
            except (TypeError, ValueError):
                continue

        driver_unit = JointUnit.DEGREE if self.config.use_degrees else JointUnit.NORMALIZED
        converted_pos, final_unit = self._convert_driver_to_internal_unit(raw_positions, driver_unit)

        # 应用主从遥操作偏置补偿 (如主从五号舵机安装/标定 90° 偏差校正, 跳过 gripper)
        if hasattr(self.config, "joint_offsets_deg") and self.config.joint_offsets_deg:
            for j, offset_deg in self.config.joint_offsets_deg.items():
                if j == SO101_GRIPPER_JOINT_NAME:
                    continue
                if j in converted_pos:
                    if final_unit == JointUnit.DEGREE:
                        converted_pos[j] += offset_deg
                    elif final_unit == JointUnit.RADIAN:
                        converted_pos[j] += deg_to_rad(offset_deg)

        state = JointState(
            timestamp=timestamp,
            positions=converted_pos,
            unit=final_unit,
            torques_enabled={k: getattr(self, "_torque_enabled", False) for k in converted_pos},
            gripper_unit=JointUnit.RANGE_0_100,
        )
        return state

    def enable_torque(self) -> bool:
        """平稳使能主臂全部舵机扭矩（配置与从臂一致的 PID 与夹爪限流保护 + 零跳变锁位）."""
        if not self.is_connected or self._leader is None or not hasattr(self._leader, "bus"):
            return False
        with self._bus_lock:
            try:
                bus = self._leader.bus
                try:
                    for motor in bus.motors:
                        bus.write("P_Coefficient", motor, 16)
                        bus.write("I_Coefficient", motor, 0)
                        bus.write("D_Coefficient", motor, 32)
                        if motor == SO101_GRIPPER_JOINT_NAME:
                            bus.write("Max_Torque_Limit", motor, 500)
                            bus.write("Protection_Current", motor, 250)
                            bus.write("Overload_Torque", motor, 25)
                except Exception as cfg_exc:
                    logger.debug("[%s] 配置主臂 PID/限流寄存器跳过: %s", self.config.robot_id, cfg_exc)
                present = bus.sync_read("Present_Position", normalize=False)
                bus.sync_write("Goal_Position", present, normalize=False)
                bus.enable_torque()
                self._torque_enabled = True
                logger.info("[%s] 主臂舵机扭矩已零跳变平稳使能", self.config.robot_id)
                return True
            except Exception as exc:
                logger.warning("[%s] 使能主臂舵机扭矩失败: %s", self.config.robot_id, exc)
                return False

    def disable_torque(self) -> bool:
        """释放主臂全部舵机扭矩，恢复手掰遥操作模式."""
        if not self.is_connected or self._leader is None or not hasattr(self._leader, "bus"):
            self._torque_enabled = False
            return False
        with self._bus_lock:
            try:
                self._leader.bus.disable_torque()
                self._torque_enabled = False
                logger.info("[%s] 主臂舵机扭矩已释放（恢复手掰模式）", self.config.robot_id)
                return True
            except Exception as exc:
                logger.warning("[%s] 释放主臂扭矩失败: %s", self.config.robot_id, exc)
                return False

    def send_target_deg(self, targets_deg: dict[str, float]) -> bool:
        """像驱动从臂一样直接向主臂 6 轴下发角度/夹爪目标 (度 / 0..100%)."""
        if not self.is_connected or self._leader is None or not hasattr(self._leader, "bus"):
            return False
        offsets = getattr(self.config, "joint_offsets_deg", None) or {}
        goals: dict[str, float] = {}
        for k, val in targets_deg.items():
            fv = float(val)
            if k == SO101_GRIPPER_JOINT_NAME:
                goals[k] = max(0.0, min(100.0, fv))
            else:
                goals[k] = fv - float(offsets.get(k, 0.0))
        if not goals:
            return False
        with self._bus_lock:
            try:
                if not getattr(self, "_torque_enabled", False):
                    if not self.enable_torque():
                        return False
                self._leader.bus.sync_write("Goal_Position", goals)
                return True
            except Exception as exc:
                logger.error("[%s] 主臂目标位置下发失败: %s", self.config.robot_id, exc)
                return False

    def read_action(self) -> RobotAction:
        """从主臂生成当前的示教动作意图 (可以直接下发给从臂)."""
        state = self.read_state()
        return RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.LEADER,
            timestamp=state.timestamp,
            target_joint_positions=dict(state.positions),
            unit=state.unit,
            gripper_unit=state.gripper_unit,
        )

