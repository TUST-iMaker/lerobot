"""Bidirectional real-hardware bridge uniting Arm101 Studio with software/core safety kernel."""

from __future__ import annotations

import math
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, Literal

from app.backend.config import StudioConfig
from app.backend.simulation import JOINT_NAMES, Arm101Simulator

_SOFTWARE_DIR = str(Path(__file__).resolve().parents[2] / "software")
if _SOFTWARE_DIR not in sys.path:
    sys.path.append(_SOFTWARE_DIR)

SyncDirection = Literal["off", "real_to_sim", "sim_to_real", "teleop_bidirectional"]


class SO101RealBridge:
    """Safely bridges real SO-101 hardware with Arm101Simulator via software/core ActionAPI.

    - `real_to_sim`: Read-only polling of Present_Position (torque NEVER enabled).
    - `sim_to_real`: 4-stage zero-jump power-on + SafetyController driving follower from UI/IK.
    - `teleop_bidirectional`: Real-time bidirectional sync — physical Leader/Follower motion
      streams into the 3D Studio at ~30Hz AND UI sliders/IK/WASD/presets drive the real Follower.
    - Automatically releases COM ports before any lerobot-* subprocess starts.
    """

    def __init__(
        self,
        config: StudioConfig,
        simulator: Arm101Simulator,
        *,
        on_motion: Callable[[], None] | None = None,
    ) -> None:
        self.config = config
        self.simulator = simulator
        self._on_motion = on_motion
        self._lock = threading.RLock()
        self.direction: SyncDirection = "off"
        self._api: Any = None
        self._poll_thread: threading.Thread | None = None
        self._poll_interval_s: float = 0.033
        self._stop_event = threading.Event()
        self._ui_control_latched: bool = False
        self._leader_torque_enabled: bool = False
        self._last_leader_deg: dict[str, float] | None = None
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
            self.simulator.real_leader_active = False
            self.simulator.clear_follower_cmd_target()

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
            f_port = (self.config.follower_port or "").strip()
            l_port = (self.config.leader_port or "").strip()
            connect_leader = bool(
                l_port and l_port.lower() != "none" and l_port != f_port
            )
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
                self._leader_torque_enabled = False
                self.last_error = None
                self._stop_event.clear()

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

    def start_bidirectional_teleop(
        self,
        *,
        fps: int = 60,
        connect_leader: bool = True,
    ) -> dict[str, Any]:
        """Start real-time bidirectional Real <-> Sim teleop bridge (supports single or dual arm)."""
        self.disconnect(return_to_rest=False, keep_torque=False)
        try:
            from core.action_api import ActionAPI
            from core.data_types import ControlSource
            from hardware.config import ArmConfig, RobotSystemConfig

            f_cal_dir = self._follower_cal_dir()
            l_cal_dir = self._leader_cal_dir()
            f_port = (self.config.follower_port or "").strip()
            l_port = (self.config.leader_port or "").strip()
            want_leader = bool(
                connect_leader
                and l_port
                and l_port.lower() != "none"
                and l_port != f_port
            )

            sys_cfg = RobotSystemConfig(
                follower=ArmConfig.default_follower(
                    port=f_port,
                    robot_id=self.config.follower_id,
                    calibration_dir=f_cal_dir,
                    calibration_fpath=f_cal_dir / f"{self.config.follower_id}.json",
                ),
                leader=ArmConfig.default_leader(
                    port=l_port if want_leader else f_port,
                    robot_id=self.config.leader_id,
                    calibration_dir=l_cal_dir,
                    calibration_fpath=l_cal_dir / f"{self.config.leader_id}.json",
                ),
            )
            api = ActionAPI(config=sys_cfg)
            ok = api.connect(connect_leader=want_leader)
            if not ok:
                raise RuntimeError(
                    f"无法连接真机串口 (follower={f_port}, leader={l_port})。"
                )

            api.claim_control(ControlSource.UI)
            self.simulator.torque_enabled = True

            hw_adapter = getattr(api, "hardware", None)
            leader_arm = getattr(hw_adapter, "leader", None)
            with self._lock:
                self._api = api
                self.direction = "teleop_bidirectional"
                self._poll_interval_s = max(0.02, min(0.05, 1.0 / max(1, fps)))
                self._ui_control_latched = False
                self._leader_torque_enabled = False
                self._last_leader_deg = None
                self.connected_follower = True
                self.connected_leader = bool(
                    want_leader and leader_arm is not None and leader_arm.is_connected
                )
                self.simulator.real_leader_active = self.connected_leader
                self.last_error = None
                self._stop_event.clear()

            self._poll_once(api)
            self._poll_thread = threading.Thread(
                target=self._poll_real_loop, name="so101-bidir-teleop", daemon=True
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
        moved = False
        now = time.monotonic()
        l_action = api.get_leader_action() if hasattr(api, "get_leader_action") else None
        if l_action is not None and getattr(l_action, "target_joint_positions", None):
            l_deg = self._positions_to_studio_deg(
                l_action.target_joint_positions, l_action.unit
            )
            prev_l = self._last_leader_deg
            leader_delta = (
                max((abs(l_deg[k] - prev_l.get(k, l_deg[k])) for k in l_deg), default=0.0)
                if prev_l
                else 0.0
            )
            self._last_leader_deg = dict(l_deg)
            self.simulator.set_leader_joints(l_deg, step_immediately=False)
            if leader_delta > 0.25:
                moved = True

            # In bidirectional teleop: physical leader drives physical follower unless UI took temporary override
            if (
                self.direction == "teleop_bidirectional"
                and self.simulator.linked_arms
                and not self.simulator.estop_latched
                and self.simulator.torque_enabled
            ):
                from core.data_types import SystemState

                cur_state = getattr(getattr(api, "state_machine", None), "current_state", None)
                if cur_state not in (SystemState.IDLE, SystemState.ESTOP_LATCHED, SystemState.FAULT, SystemState.DISCONNECTED):
                    if leader_delta > 2.5:
                        self._ui_control_latched = False  # Human actually moved real leader -> reclaim control
                        self.simulator.clear_follower_cmd_target()
                    if not self._ui_control_latched:
                        self._submit_follower_deg_unlocked(api, l_deg)

        obs = api.get_observation()
        f_state = getattr(obs, "joint_state", None) if obs is not None else None
        if f_state is not None and getattr(f_state, "positions", None):
            f_deg = self._positions_to_studio_deg(f_state.positions, f_state.unit)
            self.simulator.set_follower_joints(f_deg, from_real_feedback=True)
            moved = True

        if moved and self._on_motion is not None:
            try:
                self._on_motion()
            except Exception:
                pass

    def _poll_real_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                api = self._api
                interval = self._poll_interval_s
            if api is None:
                break
            try:
                self._poll_once(api)
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)
                time.sleep(0.1)
                continue
            time.sleep(interval)

    def _submit_follower_deg_unlocked(self, api: Any, joints_deg: dict[str, float]) -> bool:
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

    def send_follower_target_deg(self, joints_deg: dict[str, float]) -> bool:
        """Send safety-clamped follower target through ActionAPI when in sim_to_real or teleop_bidirectional."""
        with self._lock:
            if self.direction not in ("sim_to_real", "teleop_bidirectional") or self._api is None:
                return False
            api = self._api
            self._ui_control_latched = True
        return self._submit_follower_deg_unlocked(api, joints_deg)

    def disable_leader_torque(self) -> None:
        """Safely release physical Leader servo torque (returning Leader to passive hand-guided mode)."""
        with self._lock:
            api = self._api
            self._leader_torque_enabled = False
        if api is None:
            return
        leader_arm = getattr(getattr(api, "hardware", None), "leader", None)
        if leader_arm is not None and getattr(leader_arm, "is_connected", False):
            try:
                leader_arm.disable_torque()
            except Exception:
                pass

    def send_leader_target_deg(self, joints_deg: dict[str, float]) -> bool:
        """Drive physical Leader STS3215 servos directly (reuses official SO101Leader.send_feedback)."""
        with self._lock:
            if (
                self.direction not in ("sim_to_real", "teleop_bidirectional")
                or self._api is None
                or not self.connected_leader
                or self.simulator.estop_latched
                or not self.simulator.torque_enabled
            ):
                return False
            api = self._api
            need_enable = not self._leader_torque_enabled
        leader_arm = getattr(getattr(api, "hardware", None), "leader", None)
        if leader_arm is None or not getattr(leader_arm, "is_connected", False):
            return False
        try:
            if need_enable:
                # Zero-jump torque enable: hold current encoder positions first, then enable torque
                cur_action = leader_arm.get_action()
                if cur_action:
                    leader_arm.send_feedback(cur_action)
                leader_arm.enable_torque()
                with self._lock:
                    self._leader_torque_enabled = True
            feedback = {
                f"{k}.pos": float(joints_deg[k])
                for k in JOINT_NAMES
                if k in joints_deg
            }
            if feedback:
                leader_arm.send_feedback(feedback)
            return True
        except Exception as exc:
            with self._lock:
                self.last_error = f"主臂舵机写入失败: {exc}"
            return False

    def trigger_estop(self, reason: str = "Web Studio E-Stop") -> None:
        self.disable_leader_torque()
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
            direction = self.direction
        if api is not None:
            try:
                api.request_reset()
                if direction in ("sim_to_real", "teleop_bidirectional"):
                    from core.data_types import ControlSource

                    api.claim_control(ControlSource.UI)
                with self._lock:
                    self.last_error = None
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)

    @staticmethod
    def _write_direct_bus_torque(port: str, enabled: bool) -> tuple[bool, int, str]:
        """Directly write STS3215 Torque_Enable (addr 40) on motor IDs 1..6 and close the port immediately."""
        if not port or port.lower() == "none":
            return False, 0, "未配置有效串口"
        try:
            import scservo_sdk as scs
        except Exception as exc:
            return False, 0, f"缺少 scservo_sdk: {exc}"

        port_handler = scs.PortHandler(port)
        packet_handler = scs.PacketHandler(0)  # Feetech STS/SMS protocol 0
        opened = False
        try:
            if not port_handler.openPort():
                return False, 0, f"无法打开串口 {port}（可能未连接或被其他进程占用）"
            opened = True
            if not port_handler.setBaudRate(1_000_000):
                return False, 0, f"串口 {port} 设置 1Mbps 波特率失败"
            val = 1 if enabled else 0
            acked = 0
            for motor_id in range(1, 7):
                comm_res, _ = packet_handler.write1ByteTxRx(port_handler, motor_id, 40, val)
                if comm_res == scs.COMM_SUCCESS:
                    acked += 1
            if acked == 0:
                return False, 0, f"串口 {port} 无舵机响应（请检查 12V 供电与总线连接）"
            return True, acked, f"{port} 已对 {acked}/6 颗舵机{'上力锁定' if enabled else '释放扭矩'}"
        except Exception as exc:
            return False, 0, f"串口 {port} 写入扭矩失败: {exc}"
        finally:
            if opened:
                try:
                    port_handler.closePort()
                except Exception:
                    pass

    def set_torque(self, enabled: bool, *, is_real_mode: bool = False) -> tuple[bool, str]:
        with self._lock:
            api = self._api
        if api is not None:
            try:
                if enabled:
                    api.enable_torque()
                else:
                    self.disable_leader_torque()
                    api.disable_torque()
                return True, f"已通过实时桥接{'使能' if enabled else '释放'}真机扭矩"
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)
                return False, f"实时桥接切换扭矩失败: {exc}"

        if not is_real_mode:
            return False, f"模拟模式扭矩已切换为{'使能' if enabled else '释放'}"

        # Direct short-lived STS3215 bus write when bridge is idle in real mode
        target_ports: list[str] = []
        f_port = (self.config.follower_port or "").strip()
        l_port = (self.config.leader_port or "").strip()
        if f_port and f_port.lower() != "none":
            target_ports.append(f_port)
        # When releasing torque, also release leader arm if connected on a separate port;
        # when enabling torque, only energize follower arm so leader remains free for hand-guiding.
        if not enabled and l_port and l_port.lower() != "none" and l_port not in target_ports:
            target_ports.append(l_port)

        if not target_ports:
            return False, "未配置有效真机串口，无法下发扭矩指令"

        msgs: list[str] = []
        any_ok = False
        for p in target_ports:
            ok, _, msg = self._write_direct_bus_torque(p, enabled)
            if ok:
                any_ok = True
            msgs.append(msg)
        summary = "；".join(msgs)
        if not any_ok:
            with self._lock:
                self.last_error = summary
        return any_ok, summary

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
