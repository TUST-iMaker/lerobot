"""Bidirectional real-hardware bridge uniting Arm101 Studio with software/core safety kernel."""

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Literal

from app.backend.config import StudioConfig
from app.backend.simulation import JOINT_NAMES, Arm101Simulator

_SOFTWARE_DIR = str(Path(__file__).resolve().parents[2] / "software")
if _SOFTWARE_DIR not in sys.path:
    sys.path.append(_SOFTWARE_DIR)

SyncDirection = Literal["off", "real_to_sim", "sim_to_real"]


class SO101RealBridge:
    """Safely bridges real SO-101 hardware with Arm101Simulator via software/core ActionAPI.

    - `real_to_sim`: Read-only polling of Present_Position (torque NEVER enabled).
    - `sim_to_real`: 4-stage zero-jump power-on + SafetyController (15 deg/step clamp,
      URDF soft limits, RANGE_0_100 gripper, stall watchdog, E-Stop latching) driving follower.
    - Automatically releases COM ports before any lerobot-* subprocess starts.
    """

    def __init__(self, config: StudioConfig, simulator: Arm101Simulator) -> None:
        self.config = config
        self.simulator = simulator
        self._lock = threading.RLock()
        self.direction: SyncDirection = "off"
        self._api: Any = None
        self._poll_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self.last_error: str | None = None
        self.connected_follower: bool = False
        self.connected_leader: bool = False

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "direction": self.direction,
                "connected_follower": self.connected_follower,
                "connected_leader": self.connected_leader,
                "last_error": self.last_error,
            }

    def disconnect(self, *, return_to_rest: bool = False, keep_torque: bool = False) -> None:
        with self._lock:
            self._stop_event.set()
            poll_thread = self._poll_thread
            self._poll_thread = None

        # Join polling thread OUTSIDE self._lock so _poll_real_loop error writes never deadlock
        if poll_thread is not None and poll_thread.is_alive():
            poll_thread.join(timeout=1.5)

        with self._lock:
            api = self._api
            self._api = None
            self.direction = "off"
            self.connected_follower = False
            self.connected_leader = False

        if api is not None:
            try:
                api.disconnect(keep_torque=keep_torque, return_to_rest=return_to_rest)
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)

    def _follower_cal_dir(self) -> Path:
        return self.config.calibration_dir / "robots" / "so_follower"

    def _leader_cal_dir(self) -> Path:
        return self.config.calibration_dir / "teleoperators" / "so_leader"

    def set_sync_mode(
        self,
        direction: SyncDirection,
        *,
        is_real_mode: bool,
        safety_confirmed: bool = False,
    ) -> dict[str, Any]:
        if direction == "off":
            self.disconnect(return_to_rest=False, keep_torque=False)
            return self.status()

        if direction == "sim_to_real" and not safety_confirmed:
            raise ValueError(
                "开启「动画动 → 现实动」前必须勾选安全确认（确认桌面无遮挡、允许限幅驱动从臂）。"
            )

        if not is_real_mode:
            with self._lock:
                self.direction = direction
                self.last_error = None
            return {
                **self.status(),
                "note": "当前为 dry_run 模拟模式，仅模拟同步状态，未打开真实串口。",
            }

        self.disconnect(return_to_rest=False, keep_torque=False)
        try:
            from core.action_api import ActionAPI
            from core.data_types import ControlSource
            from hardware.config import ArmConfig, RobotSystemConfig

            f_cal_dir = self._follower_cal_dir()
            l_cal_dir = self._leader_cal_dir()
            sys_cfg = RobotSystemConfig(
                follower=ArmConfig.default_follower(
                    port=self.config.follower_port,
                    robot_id=self.config.follower_id,
                    calibration_dir=f_cal_dir,
                    calibration_fpath=f_cal_dir / f"{self.config.follower_id}.json",
                ),
                leader=ArmConfig.default_leader(
                    port=self.config.leader_port,
                    robot_id=self.config.leader_id,
                    calibration_dir=l_cal_dir,
                    calibration_fpath=l_cal_dir / f"{self.config.leader_id}.json",
                ),
            )
            api = ActionAPI(config=sys_cfg)
            connect_leader = direction == "real_to_sim"
            ok = api.connect(connect_leader=connect_leader)
            if not ok:
                raise RuntimeError(
                    f"无法连接真机串口 (follower={self.config.follower_port}, leader={self.config.leader_port})。"
                )

            if direction == "real_to_sim":
                api.disable_torque()
                self.simulator.torque_enabled = False
            else:
                api.claim_control(ControlSource.UI)
                self.simulator.torque_enabled = True

            hw_adapter = getattr(api, "hardware", None)
            leader_arm = getattr(hw_adapter, "leader", None)
            with self._lock:
                self._api = api
                self.direction = direction
                self.connected_follower = True
                self.connected_leader = bool(
                    connect_leader and leader_arm is not None and leader_arm.is_connected
                )
                self.last_error = None
                self._stop_event.clear()

            if direction == "real_to_sim":
                self._poll_once(api)
                self._poll_thread = threading.Thread(
                    target=self._poll_real_loop, name="so101-real-sync", daemon=True
                )
                self._poll_thread.start()
            return self.status()
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
            self.disconnect(return_to_rest=False, keep_torque=False)
            raise RuntimeError(str(exc)) from exc

    @staticmethod
    def _positions_to_studio_deg(positions: dict[str, float], unit_val: Any) -> dict[str, float]:
        """Convert body joints from RADIAN to degrees while preserving gripper in RANGE_0_100 [0..100]."""
        is_rad = str(getattr(unit_val, "value", unit_val)).lower() == "radian"
        out: dict[str, float] = {}
        for k, v in positions.items():
            if k not in JOINT_NAMES:
                continue
            fv = float(v)
            if k == "gripper":
                out[k] = round(max(0.0, min(100.0, fv)), 2)
            elif is_rad:
                out[k] = round(math.degrees(fv), 2)
            else:
                out[k] = round(fv, 2)
        return out

    def _poll_once(self, api: Any) -> None:
        l_action = api.get_leader_action() if hasattr(api, "get_leader_action") else None
        if l_action is not None and getattr(l_action, "target_joint_positions", None):
            l_deg = self._positions_to_studio_deg(
                l_action.target_joint_positions, l_action.unit
            )
            self.simulator.set_leader_joints(l_deg, step_immediately=False)

        obs = api.get_observation()
        f_state = getattr(obs, "joint_state", None) if obs is not None else None
        if f_state is not None and getattr(f_state, "positions", None):
            f_deg = self._positions_to_studio_deg(f_state.positions, f_state.unit)
            self.simulator.set_follower_joints(f_deg)

    def _poll_real_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                api = self._api
            if api is None:
                break
            try:
                self._poll_once(api)
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)
                time.sleep(0.1)
                continue
            time.sleep(0.05)

    def send_follower_target_deg(self, joints_deg: dict[str, float]) -> bool:
        """Send safety-clamped follower target through ActionAPI when in sim_to_real mode."""
        with self._lock:
            if self.direction != "sim_to_real" or self._api is None:
                return False
            api = self._api

        from core.data_types import ActionType, ControlSource, JointUnit, RobotAction

        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.UI,
            timestamp=time.monotonic(),
            target_joint_positions=dict(joints_deg),
            unit=JointUnit.DEGREE,
            gripper_unit=JointUnit.RANGE_0_100,
        )
        res = api.submit_action(action)
        if isinstance(res, tuple):
            ok, msg = bool(res[0]), str(res[1])
            if not ok:
                with self._lock:
                    self.last_error = msg
            return ok
        return bool(getattr(res, "passed", res))

    def trigger_estop(self, reason: str = "Web Studio E-Stop") -> None:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                api.request_estop(reason=reason)
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)

    def reset_estop(self) -> None:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                api.request_reset()
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)

    def set_torque(self, enabled: bool) -> None:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                if enabled:
                    api.enable_torque()
                else:
                    api.disable_torque()
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)

    def safe_park_to_rest(self, duration_s: float = 2.0) -> bool:
        """Invoke teleop.official_wrapper.safe_park_to_rest with Studio follower ID & calibration dir."""
        self.disconnect(return_to_rest=False, keep_torque=False)
        try:
            from teleop.official_wrapper import safe_park_to_rest

            return bool(
                safe_park_to_rest(
                    port=self.config.follower_port,
                    duration_s=duration_s,
                    robot_id=self.config.follower_id,
                    calibration_dir=self._follower_cal_dir(),
                )
            )
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
            return False
