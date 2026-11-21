"""Bidirectional real-hardware bridge uniting Arm101 Studio with aguiya's software/core safety kernel."""

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
      soft limits, stall watchdog, E-Stop latching) driving the follower arm only.
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
            if self._poll_thread is not None and self._poll_thread.is_alive():
                self._poll_thread.join(timeout=1.5)
            self._poll_thread = None

            if self._api is not None:
                try:
                    self._api.disconnect(keep_torque=keep_torque, return_to_rest=return_to_rest)
                except Exception:
                    pass
                self._api = None

            self.direction = "off"
            self.connected_follower = False
            self.connected_leader = False

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
            # In dry_run mode, record direction cleanly without opening serial ports
            with self._lock:
                self.direction = direction
                self.last_error = None
            return {
                **self.status(),
                "note": "当前为 dry_run 模拟模式，仅模拟同步状态，未打开真实串口。",
            }

        # Real mode: connect via aguiya's ActionAPI + 4-stage zero-jump safety
        self.disconnect(return_to_rest=False, keep_torque=False)
        try:
            from core.action_api import ActionAPI
            from core.data_types import ControlSource
            from hardware.config import ArmConfig, RobotSystemConfig

            sys_cfg = RobotSystemConfig(
                follower=ArmConfig.default_follower(
                    port=self.config.follower_port,
                    calibration_id=self.config.follower_id,
                ),
                leader=ArmConfig.default_leader(
                    port=self.config.leader_port,
                    calibration_id=self.config.leader_id,
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
                # Guarantee zero torque in read-only observation mode
                try:
                    api.set_torque(False)
                except Exception:
                    pass
                self.simulator.torque_enabled = False
            else:
                api.claim_control(ControlSource.TELEOP)
                self.simulator.torque_enabled = True

            with self._lock:
                self._api = api
                self.direction = direction
                self.connected_follower = True
                self.connected_leader = connect_leader and bool(
                    getattr(api.adapter, "leader", None) and api.adapter.leader.is_connected
                )
                self.last_error = None
                self._stop_event.clear()

            if direction == "real_to_sim":
                self._poll_thread = threading.Thread(
                    target=self._poll_real_loop, name="so101-real-sync", daemon=True
                )
                self._poll_thread.start()
            return self.status()
        except Exception as exc:
            self.last_error = str(exc)
            self.disconnect(return_to_rest=False, keep_torque=False)
            raise RuntimeError(str(exc)) from exc

    def _poll_real_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                api = self._api
            if api is None:
                break
            try:
                obs = api.get_observation()
                if obs and obs.follower_joints:
                    f_deg = {
                        k: round(math.degrees(float(v)), 2)
                        for k, v in obs.follower_joints.positions.items()
                        if k in JOINT_NAMES
                    }
                    self.simulator.set_follower_joints(f_deg)
                if obs and obs.leader_joints:
                    l_deg = {
                        k: round(math.degrees(float(v)), 2)
                        for k, v in obs.leader_joints.positions.items()
                        if k in JOINT_NAMES
                    }
                    self.simulator.set_leader_joints(l_deg, step_immediately=False)
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.3)
            time.sleep(0.1)

    def send_follower_target_deg(self, joints_deg: dict[str, float]) -> bool:
        """Send safety-clamped follower target through ActionAPI when in sim_to_real mode."""
        with self._lock:
            if self.direction != "sim_to_real" or self._api is None:
                return False
            api = self._api

        from core.data_types import ActionType, ControlSource, JointUnit, RobotAction

        action = RobotAction(
            action_type=ActionType.JOINT_POSITION,
            source=ControlSource.TELEOP,
            target_joints=dict(joints_deg),
            unit=JointUnit.DEGREE,
        )
        report = api.submit_action(action)
        return bool(getattr(report, "passed", False))

    def trigger_estop(self, reason: str = "Web Studio E-Stop") -> None:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                api.emergency_stop(reason=reason)
            except Exception:
                pass

    def reset_estop(self) -> None:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                api.request_reset()
            except Exception:
                pass

    def set_torque(self, enabled: bool) -> None:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                api.set_torque(enabled)
            except Exception:
                pass

    def safe_park_to_rest(self, duration_s: float = 2.0) -> bool:
        """Invoke aguiya's official_wrapper.safe_park_to_rest when hardware is connected."""
        self.disconnect(return_to_rest=False, keep_torque=False)
        try:
            from teleop.official_wrapper import safe_park_to_rest

            return bool(safe_park_to_rest(port=self.config.follower_port, duration_s=duration_s))
        except Exception as exc:
            self.last_error = str(exc)
            return False
