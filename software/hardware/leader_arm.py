"""SO-ARM101 Leader 示教主臂硬件驱动适配器.

职责:
1. 封装 Leader 示教臂的连接与状态读取;
2. 零扭矩保证: 连接后严格禁用舵机扭矩，允许操作员轻快拖动示教;
3. 提供 read_action() 接口，将主臂当前关节位姿直接封装为 RobotAction (source=LEADER);
4. 实时监测采样延迟与数据有效性.
"""

from __future__ import annotations

import logging
import time
from typing import Any

from core.data_types import (
    ActionType,
    ControlSource,
    JointState,
    JointUnit,
    RobotAction,
)
from hardware.base import BaseArmAdapter
from hardware.config import ArmConfig

logger = logging.getLogger(__name__)


class SO101LeaderArm(BaseArmAdapter):
    """SO-ARM101 主臂 (示教器) 硬件适配器."""

    def __init__(self, config: ArmConfig) -> None:
        super().__init__(config)
        self._leader: Any = None

    def connect(self) -> None:
        """连接示教主臂. 强制释放扭矩以便人工顺应拖拽."""
        if self._connected:
            logger.info("[%s] 主臂已连接", self.config.robot_id)
            return

        self.load_calibration()

        try:
            from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
            driver_cfg = SO101LeaderConfig(
                port=self.config.port,
                id=self.config.robot_id,
                use_degrees=self.config.use_degrees,
            )
            leader = SO101Leader(driver_cfg)
            leader.connect()
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
                disable_torque_on_disconnect=True,
                use_degrees=self.config.use_degrees,
            )
            leader = SO101Follower(driver_cfg)
            leader.bus.connect()
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

        # 应用主从遥操作偏置补偿 (如主从五号舵机安装/标定 90° 偏差校正)
        if hasattr(self.config, "joint_offsets_deg") and self.config.joint_offsets_deg:
            for j, offset_deg in self.config.joint_offsets_deg.items():
                if j in converted_pos:
                    if final_unit == JointUnit.DEGREE:
                        converted_pos[j] += offset_deg
                    elif final_unit == JointUnit.RADIAN:
                        from core.data_types import deg_to_rad
                        converted_pos[j] += deg_to_rad(offset_deg)

        state = JointState(
            timestamp=timestamp,
            positions=converted_pos,
            unit=final_unit,
            torques_enabled={k: False for k in converted_pos},
        )
        return state

    def read_action(self) -> RobotAction:
        """从主臂生成当前的示教动作意图 (可以直接下发给从臂)."""
        state = self.read_state()
        return RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.LEADER,
            timestamp=state.timestamp,
            target_joint_positions=dict(state.positions),
            unit=state.unit,
        )

