"""机械臂硬件抽象基类与通用健康状态定义.

规范所有机械臂适配器必须履行的接口契约，提供生命周期管理、校准数据解析与通信延迟健康监控.
"""

from __future__ import annotations

import abc
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from core.data_types import (
    SO101_GRIPPER_JOINT_NAME,
    SO101_JOINT_NAMES,
    JointState,
    JointUnit,
    deg_to_rad,
    rad_to_deg,
)
from hardware.config import ArmConfig

logger = logging.getLogger(__name__)


@dataclass
class ArmHealth:
    """机械臂硬件与通信健康诊断信息."""
    is_connected: bool = False
    is_calibrated: bool = False
    last_read_latency_ms: float = 0.0
    avg_read_latency_ms: float = 0.0
    consecutive_read_errors: int = 0
    total_samples: int = 0
    torque_enabled_count: int = 0
    total_motors: int = 6
    last_error_message: str | None = None

    @property
    def is_healthy(self) -> bool:
        """通信正常且连续错误小于 3 次."""
        return self.is_connected and self.consecutive_read_errors < 3


class BaseArmAdapter(abc.ABC):
    """单条机械臂硬件适配器抽象基类."""

    def __init__(self, config: ArmConfig) -> None:
        self.config = config
        self._connected: bool = False
        self._calibration_ranges: dict[str, tuple[int, int]] = {}
        self._hw_calibrated: bool | None = None
        self._health = ArmHealth()
        self._latencies: list[float] = []

    @property
    def is_connected(self) -> bool:
        """硬件连接状态."""
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        """是否已成功加载校准配置且硬件 EEPROM 未校验失败."""
        return bool(self._calibration_ranges) and (self._hw_calibrated is not False)

    @property
    def calibration_ranges(self) -> dict[str, tuple[int, int]]:
        """获取各关节的物理 raw 范围 (range_min, range_max)."""
        return self._calibration_ranges

    @property
    def health(self) -> ArmHealth:
        """获取当前硬件健康遥测数据."""
        self._health.is_connected = self.is_connected
        self._health.is_calibrated = self.is_calibrated
        return self._health

    def load_calibration(self) -> dict[str, tuple[int, int]]:
        """从指定路径加载并解析校准 JSON 文件."""
        path = self.config.calibration_fpath
        if not path.is_file():
            logger.warning("[%s] 校准文件不存在: %s", self.config.robot_id, path)
            self._calibration_ranges = {}
            return {}

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            ranges: dict[str, tuple[int, int]] = {}
            for motor, item in data.items():
                if isinstance(item, dict) and "range_min" in item and "range_max" in item:
                    min_v = int(item["range_min"])
                    max_v = int(item["range_max"])
                    if min_v >= max_v:
                        raise ValueError(f"关节 {motor} range_min ({min_v}) >= range_max ({max_v})")
                    ranges[motor] = (min_v, max_v)

            self._calibration_ranges = ranges
            logger.info("[%s] 成功解析 %d 个关节的校准范围", self.config.robot_id, len(ranges))
            return ranges
        except Exception as exc:
            msg = f"解析校准文件失败: {exc}"
            logger.error("[%s] %s", self.config.robot_id, msg)
            self._health.last_error_message = msg
            self._calibration_ranges = {}
            return {}

    def _convert_driver_to_internal_unit(
        self,
        positions: dict[str, float],
        driver_unit: JointUnit,
    ) -> tuple[dict[str, float], JointUnit]:
        """将驱动输出的单位转换为内部基准单位 (arm 转换, gripper 始终保持 RANGE_0_100)."""
        if self.config.internal_unit == driver_unit:
            return {k: float(v) for k, v in positions.items()}, driver_unit

        if self.config.internal_unit == JointUnit.RADIAN and driver_unit == JointUnit.DEGREE:
            return {
                k: (float(v) if k == SO101_GRIPPER_JOINT_NAME else deg_to_rad(float(v)))
                for k, v in positions.items()
            }, JointUnit.RADIAN

        if self.config.internal_unit == JointUnit.DEGREE and driver_unit == JointUnit.RADIAN:
            return {
                k: (float(v) if k == SO101_GRIPPER_JOINT_NAME else rad_to_deg(float(v)))
                for k, v in positions.items()
            }, JointUnit.DEGREE

        # 归一化等其他情况保持原状返回
        return {k: float(v) for k, v in positions.items()}, driver_unit

    def _convert_internal_to_driver_unit(
        self,
        positions: dict[str, float],
        source_unit: JointUnit,
    ) -> dict[str, float]:
        """将内部动作单位转换为驱动目标单位 (arm 转 DEGREE, gripper 始终保持 RANGE_0_100)."""
        if source_unit == JointUnit.RADIAN and self.config.use_degrees:
            return {
                k: (float(v) if k == SO101_GRIPPER_JOINT_NAME else rad_to_deg(float(v)))
                for k, v in positions.items()
            }
        return {k: float(v) for k, v in positions.items()}

    def _record_read_latency(self, latency_ms: float) -> None:
        """记录状态读取延迟并更新健康统计."""
        self._latencies.append(latency_ms)
        if len(self._latencies) > 100:
            self._latencies.pop(0)

        self._health.last_read_latency_ms = round(latency_ms, 2)
        self._health.avg_read_latency_ms = round(sum(self._latencies) / len(self._latencies), 2)
        self._health.total_samples += 1
        self._health.consecutive_read_errors = 0

    def read_diagnostics(self) -> dict[str, Any]:
        """读取舵机物理诊断信息 (温度、输入电压、负载，默认空字典)."""
        return {}

    @abc.abstractmethod
    def connect(self) -> None:
        """建立串口与硬件通信, 完成初始化与上电校验."""

    @abc.abstractmethod
    def disconnect(self, keep_torque: bool = False) -> None:
        """安全断开与机械臂硬件的连接."""

    @abc.abstractmethod
    def read_state(self) -> JointState:
        """读取当前各关节状态观测反馈."""

