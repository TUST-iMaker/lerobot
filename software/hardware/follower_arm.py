"""SO-ARM101 Follower 执行从臂硬件驱动适配器.

职责:
1. 封装 Follower 从臂全部硬件通信与伺服控制;
2. 严格实现上电防跳变 (Present -> Goal -> Configure 使能) 与断开保护;
3. 执行动作下发、当前位置急停保持 (Hold & Estop)、分步平滑回中位;
4. 实时监测舵机扭矩状态与读取延迟.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from core.data_types import (
    MIDDLE_ORDER,
    SO101_ARM_JOINT_NAMES,
    SO101_GRIPPER_JOINT_NAME,
    SO101_JOINT_NAMES,
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
    clamp_gripper_pct,
    deg_to_rad,
    rad_to_deg,
)
from hardware.base import BaseArmAdapter
from hardware.config import ArmConfig

logger = logging.getLogger(__name__)


class SO101FollowerArm(BaseArmAdapter):
    """SO-ARM101 从臂 (执行器) 硬件适配器."""

    def __init__(self, config: ArmConfig) -> None:
        super().__init__(config)
        self._follower: Any = None
        self._bus_lock = threading.RLock()
        self._last_state: JointState | None = None
        self._last_torque_states: dict[str, bool] = {}
        self._last_torque_read_time: float = 0.0

    def connect(self) -> None:
        """连接从臂硬件. 严格执行防跳变安全上电流程."""
        if self._connected:
            logger.info("[%s] 从臂已处于连接状态", self.config.robot_id)
            return

        self.load_calibration()

        try:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        except ImportError as exc:
            msg = f"缺少 LeRobot 机器人依赖库: {exc}"
            self._health.last_error_message = msg
            raise RuntimeError(msg) from exc

        logger.info("[%s] 正在连接从臂 (端口: %s)...", self.config.robot_id, self.config.port)
        cal_dir = self.config.calibration_dir or self.config.calibration_fpath.parent
        driver_cfg = SO101FollowerConfig(
            port=self.config.port,
            id=self.config.robot_id,
            calibration_dir=cal_dir,
            disable_torque_on_disconnect=False,
            use_degrees=self.config.use_degrees,
            max_relative_target=self.config.max_relative_target,
        )
        follower = SO101Follower(driver_cfg)

        try:
            # 1. 物理连接总线、非阻塞校准一致性检查与挂载相机
            follower.bus.connect()
            try:
                hw_cal = bool(follower.is_calibrated)
            except Exception as cal_exc:
                hw_cal = False
                logger.warning("[%s] 检查硬件校准状态异常: %s", self.config.robot_id, cal_exc)
            self._hw_calibrated = hw_cal
            if not hw_cal:
                logger.warning(
                    "[%s] 舵机 EEPROM 与校准文件 %s 不一致或未校准，请运行校准流程!",
                    self.config.robot_id,
                    getattr(follower, "calibration_fpath", self.config.calibration_fpath),
                )

            for cam in follower.cameras.values():
                cam.connect()

            # 2. 绝对防跳变时序:
            # 预同步: 避免 configure() 内部 with torque_disabled(): 退出自动恢复扭矩时向旧 Goal_Position 跳变
            follower.bus.disable_torque()
            present_pre = follower.bus.sync_read("Present_Position", normalize=False)
            follower.bus.sync_write("Goal_Position", present_pre, normalize=False)

            # 第一阶段: 在零力矩状态下完成所有电机基础参数与工作模式配置
            follower.configure()
            # 确保处于禁用扭矩状态
            follower.bus.disable_torque()
            time.sleep(0.05)

            # 第二阶段: 配置稳定后，读取此时真实物理编码器位置 (Present_Position)
            present = follower.bus.sync_read("Present_Position", normalize=False)

            # 第三阶段: 将当前的绝对真实物理位置写入目标寄存器 (Goal_Position)
            follower.bus.sync_write("Goal_Position", present, normalize=False)
            time.sleep(0.02)

            # 第四阶段: 平稳使能扭矩，目标位置与物理位置 100% 重合，彻底杜绝开机手腕翻转与跳变
            follower.bus.enable_torque()

            self._follower = follower
            self._connected = True

            # 校验并更新使能状态
            torques = self.read_torque_states(force_refresh=True)
            self._health.torque_enabled_count = sum(1 for v in torques.values() if v)
            logger.info(
                "[%s] 从臂连接成功! 扭矩已在当前位姿平稳使能 (%d/6 个关节使能)",
                self.config.robot_id,
                self._health.torque_enabled_count,
            )
        except Exception as exc:
            self._cleanup_on_failure(follower)
            msg = f"从臂连接初始化失败: {exc}"
            self._health.last_error_message = msg
            logger.error("[%s] %s", self.config.robot_id, msg)
            raise RuntimeError(msg) from exc

    def _cleanup_on_failure(self, follower: Any) -> None:
        self._connected = False
        self._follower = None
        if follower is not None:
            try:
                follower.bus.disable_torque()
            except Exception:
                pass
            try:
                follower.disconnect()
            except Exception:
                pass

    def disconnect(self, keep_torque: bool = False) -> None:
        """安全断开从臂."""
        if not self._connected or self._follower is None:
            return

        logger.info("[%s] 正在断开从臂 (keep_torque=%s)...", self.config.robot_id, keep_torque)
        try:
            if not keep_torque:
                self._follower.bus.disable_torque()
                logger.info("[%s] 舵机扭矩已释放 (手臂可能受重力下垂)", self.config.robot_id)
            else:
                logger.warning("[%s] 扭矩保持开启，请注意电机发热", self.config.robot_id)
            self._follower.disconnect()
        except Exception as exc:
            logger.error("[%s] 断开时发生异常: %s", self.config.robot_id, exc)
        finally:
            self._follower = None
            self._connected = False
            self._health.torque_enabled_count = 0

    def disable_torque(self) -> bool:
        """释放全部舵机扭矩 (进入松弛休眠状态，避免持续发热和蜂鸣)."""
        if not self.is_connected or self._follower is None:
            return False
        with self._bus_lock:
            try:
                self._follower.bus.disable_torque()
                self._health.torque_enabled_count = 0
                self._last_torque_states = {k: False for k in self._last_torque_states}
                self._last_torque_read_time = time.monotonic()
                logger.info("[%s] 全部舵机扭矩已释放进入休眠", self.config.robot_id)
                return True
            except Exception as exc:
                logger.warning("[%s] 释放舵机扭矩失败: %s", self.config.robot_id, exc)
                return False

    def enable_torque(self) -> bool:
        """平稳恢复全部舵机扭矩 (先同步当前物理位置，防止跳变)."""
        if not self.is_connected or self._follower is None:
            return False
        with self._bus_lock:
            try:
                present = self._follower.bus.sync_read("Present_Position", normalize=False)
                self._follower.bus.sync_write("Goal_Position", present, normalize=False)
                self._follower.bus.enable_torque()
                torques = self.read_torque_states(force_refresh=True)
                self._health.torque_enabled_count = sum(1 for v in torques.values() if v)
                logger.info("[%s] 舵机扭矩已平稳恢复使能 (%d/6)", self.config.robot_id, self._health.torque_enabled_count)
                return True
            except Exception as exc:
                logger.warning("[%s] 恢复舵机扭矩失败: %s", self.config.robot_id, exc)
                return False

    def read_torque_states(self, force_refresh: bool = False) -> dict[str, bool]:
        """读取全部舵机 Torque_Enable 寄存器状态 (带高频调用缓存，避免阻碍 30Hz~60Hz 控制流)."""
        if not self.is_connected or self._follower is None:
            return {}
        now = time.monotonic()
        if not force_refresh and self._last_torque_states and (now - self._last_torque_read_time < 1.0):
            return dict(self._last_torque_states)
        with self._bus_lock:
            try:
                raw = self._follower.bus.sync_read("Torque_Enable", normalize=False)
                res = {str(k): bool(int(v) == 1) for k, v in raw.items()}
                self._last_torque_states = res
                self._last_torque_read_time = now
                return res
            except Exception as exc:
                logger.warning("[%s] 读取扭矩状态失败: %s", self.config.robot_id, exc)
                return dict(self._last_torque_states)

    def read_diagnostics(self) -> dict[str, Any]:
        """读取舵机物理诊断信息 (温度、输入电压等)."""
        if not self.is_connected or self._follower is None:
            return {}
        diag: dict[str, Any] = {
            "temperatures": {},
            "voltages": {},
            "max_temp": 0.0,
            "min_voltage": 0.0,
        }
        with self._bus_lock:
            try:
                raw_temps = self._follower.bus.sync_read("Present_Temperature", normalize=False)
                temps = {str(k): float(v) for k, v in raw_temps.items()}
                diag["temperatures"] = temps
                if temps:
                    diag["max_temp"] = round(max(temps.values()), 1)
            except Exception:
                pass

            try:
                raw_volts = self._follower.bus.sync_read("Present_Voltage", normalize=False)
                volts = {str(k): round(float(v) / 10.0, 2) for k, v in raw_volts.items()}
                diag["voltages"] = volts
                if volts:
                    diag["min_voltage"] = round(min(volts.values()), 2)
            except Exception:
                pass

            return diag

    def read_state(self) -> JointState:
        """从机械臂读取关节状态，封装为标准 JointState."""
        if not self.is_connected or self._follower is None:
            raise RuntimeError(f"[{self.config.robot_id}] 机械臂未连接，无法读取状态")

        with self._bus_lock:
            t0 = time.perf_counter()
            try:
                obs = self._follower.get_observation()
                latency_ms = (time.perf_counter() - t0) * 1000.0
                self._record_read_latency(latency_ms)
            except Exception as exc:
                self._health.consecutive_read_errors += 1
                self._health.last_error_message = f"读取观测异常: {exc}"
                logger.error("[%s] 读取观测失败: %s", self.config.robot_id, exc)
                raise

            timestamp = time.monotonic()
            driver_positions: dict[str, float] = {}
            for key, val in obs.items():
                if not key.endswith(".pos"):
                    continue
                name = key[:-4]
                try:
                    driver_positions[name] = float(val)
                except (TypeError, ValueError):
                    continue

            driver_unit = JointUnit.DEGREE if self.config.use_degrees else JointUnit.NORMALIZED
            converted_pos, final_unit = self._convert_driver_to_internal_unit(driver_positions, driver_unit)

            torques = self.read_torque_states()
            state = JointState(
                timestamp=timestamp,
                positions=converted_pos,
                unit=final_unit,
                torques_enabled=torques if torques else None,
                gripper_unit=JointUnit.RANGE_0_100,
            )

            is_valid, err = state.validate(require_all_joints=False)
            if not is_valid:
                logger.warning("[%s] 读取到的 JointState 校验告警: %s", self.config.robot_id, err)

            self._last_state = state
            return state

    def send_action(self, action: RobotAction) -> bool:
        """向从臂下发运动指令."""
        if not self.is_connected or self._follower is None:
            raise RuntimeError(f"[{self.config.robot_id}] 机械臂未连接，无法下发动作")

        is_valid, err = action.validate()
        if not is_valid:
            logger.error("[%s] 拒绝执行非法动作: %s", self.config.robot_id, err)
            return False

        if action.action_type == ActionType.JOINT_POSITION:
            if not action.target_joint_positions:
                return False
            targets = action.target_joint_positions

            # 转换为驱动单位 (arm 转 degree, gripper 保持 RANGE_0_100 [0, 100])
            targets_driver = self._convert_internal_to_driver_unit(targets, action.unit)

        elif action.action_type == ActionType.JOINT_DELTA:
            if not action.target_joint_deltas:
                return False
            cur_state = self._last_state or self.read_state()
            targets_driver = {}
            for j, delta in action.target_joint_deltas.items():
                if j not in cur_state.positions:
                    continue
                cur_val = float(cur_state.positions[j])
                if j == SO101_GRIPPER_JOINT_NAME:
                    targets_driver[j] = clamp_gripper_pct(cur_val + float(delta))
                else:
                    cur_driver = (
                        rad_to_deg(cur_val)
                        if (cur_state.unit == JointUnit.RADIAN and self.config.use_degrees)
                        else cur_val
                    )
                    delta_driver = (
                        rad_to_deg(float(delta))
                        if (action.unit == JointUnit.RADIAN and self.config.use_degrees)
                        else float(delta)
                    )
                    targets_driver[j] = cur_driver + delta_driver
        else:
            logger.error("[%s] 暂不支持直接下发 %s 动作，需先经运动学解算", self.config.robot_id, action.action_type)
            return False

        with self._bus_lock:
            try:
                if self._health.torque_enabled_count == 0:
                    self.enable_torque()
                payload = {f"{k}.pos": float(v) for k, v in targets_driver.items()}
                self._follower.send_action(payload)
                return True
            except Exception as exc:
                logger.error("[%s] 动作发送失败: %s", self.config.robot_id, exc)
                return False

    def stop_motion(self) -> None:
        """急停/停止运动: 读取当前物理位置并立即设为目标位置，锁紧当前姿态."""
        if not self.is_connected or self._follower is None:
            return

        logger.info("[%s] 触发原位保持: 正在锁定目标位置...", self.config.robot_id)
        with self._bus_lock:
            try:
                present = self._follower.bus.sync_read("Present_Position", normalize=False)
                self._follower.bus.sync_write("Goal_Position", present, normalize=False)
                logger.info("[%s] 已成功锁定当前位姿并保持扭矩", self.config.robot_id)
            except Exception as exc:
                logger.error("[%s] 锁定目标位置失败: %s", self.config.robot_id, exc)

    def move_to_middle(
        self,
        step_deg: float = 3.0,
        tolerance_deg: float = 1.0,
        timeout_s: float = 60.0,
        gripper_target: float | None = None,
    ) -> bool:
        """按安全次序平稳移动回物理校准零位 (5 臂关节归 0.0°; gripper 默认保持当前开合度，不强行闭合)."""
        if not self.is_connected or self._follower is None:
            raise RuntimeError(f"[{self.config.robot_id}] 机械臂未连接")

        logger.info("[%s] 开始安全回标准中位 (0.0°)...", self.config.robot_id)
        deadline = time.monotonic() + timeout_s
        arm_motors = [m for m in MIDDLE_ORDER if m in SO101_ARM_JOINT_NAMES]

        while True:
            cur_state = self.read_state()
            action_deg: dict[str, float] = {}
            for m in arm_motors:
                cur_val = float(cur_state.positions.get(m, 0.0))
                cur_deg = rad_to_deg(cur_val) if cur_state.unit == JointUnit.RADIAN else cur_val
                err = 0.0 - cur_deg
                if abs(err) <= tolerance_deg:
                    continue
                step = max(-step_deg, min(step_deg, err))
                action_deg[m] = cur_deg + step

            gripper_next: float | None = None
            if gripper_target is not None and SO101_GRIPPER_JOINT_NAME in cur_state.positions:
                cur_g = float(cur_state.positions[SO101_GRIPPER_JOINT_NAME])
                g_err = clamp_gripper_pct(gripper_target) - cur_g
                if abs(g_err) > tolerance_deg:
                    g_step = max(-step_deg, min(step_deg, g_err))
                    gripper_next = clamp_gripper_pct(cur_g + g_step)

            if not action_deg and gripper_next is None:
                logger.info("[%s] 从臂全部臂关节已安全归位至标准中位 (0.0°)", self.config.robot_id)
                return True

            if time.monotonic() > deadline:
                logger.error("[%s] 回中位动作超时 (%.1f s)", self.config.robot_id, timeout_s)
                self.stop_motion()
                return False

            targets: dict[str, float] = {}
            for j in SO101_ARM_JOINT_NAMES:
                if j in action_deg:
                    targets[j] = (
                        deg_to_rad(action_deg[j])
                        if cur_state.unit == JointUnit.RADIAN
                        else action_deg[j]
                    )
                elif j in cur_state.positions:
                    targets[j] = float(cur_state.positions[j])

            if SO101_GRIPPER_JOINT_NAME in cur_state.positions:
                targets[SO101_GRIPPER_JOINT_NAME] = (
                    gripper_next
                    if gripper_next is not None
                    else float(cur_state.positions[SO101_GRIPPER_JOINT_NAME])
                )

            self.send_action(
                RobotAction(
                    action_type=ActionType.JOINT_POSITION,
                    source=ControlSource.UI,
                    timestamp=time.monotonic(),
                    target_joint_positions=targets,
                    unit=cur_state.unit,
                    gripper_unit=JointUnit.RANGE_0_100,
                )
            )
            time.sleep(0.04)

