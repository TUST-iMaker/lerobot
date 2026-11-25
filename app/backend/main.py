"""FastAPI entrypoint and WebSocket telemetry server for TUST-iMaker Arm101 Studio."""

from __future__ import annotations

import asyncio
import math
import platform
import re
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Literal

import uvicorn
from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from app.backend.config import ModeType, StudioConfig, load_config
from app.backend.hardware import HardwareManager
from app.backend.plugins import (
    PluginRegistry,
    PluginUnavailableError,
    get_dataset_preview,
    get_servo_bench_diagnostics,
)
from app.backend.process_manager import ProcessRegistry
from app.backend.simulation import Arm101Simulator, validate_joint_dict
from app.backend.workflows import WorkflowConflictError, WorkflowManager


# ----------------------------------------------------------------------
# Request Schemas
# ----------------------------------------------------------------------
class ModeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mode: ModeType


class CalibrateStartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: Literal["follower", "leader"] = "follower"
    role: Literal["follower", "leader"] | None = None
    port: str | None = None
    arm_id: str | None = None
    calibration_dir: str | None = None

    @model_validator(mode="after")
    def _apply_role_alias(self) -> CalibrateStartRequest:
        if self.role is not None:
            self.target = self.role
        return self


class CalibrateInputRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    value: str = ""


class HardwareConfigRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    follower_port: str | None = None
    leader_port: str | None = None
    follower_id: str | None = None
    leader_id: str | None = None
    front_camera_id: str | None = None
    wrist_camera_id: str | None = None
    persist_env: bool = True
    auto_detect: bool = False


class TeleopStartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    fps: int = Field(default=60, ge=1, le=120)
    follower_port: str | None = None
    leader_port: str | None = None
    display_data: bool = False
    use_cameras: bool = False


class TeleopJointsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    joints: dict[str, Any] | list[Any]

    @field_validator("joints")
    @classmethod
    def _validate_joints(cls, value: dict[str, Any] | list[Any]) -> dict[str, float]:
        return validate_joint_dict(value, require_non_empty=True)


class WaypointRecordRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    label: str | None = None
    joints: dict[str, Any] | list[Any] | None = None

    @field_validator("joints")
    @classmethod
    def _validate_optional_joints(
        cls, value: dict[str, Any] | list[Any] | None
    ) -> dict[str, float] | None:
        if value is None:
            return None
        return validate_joint_dict(value, require_non_empty=True)


class ReplayRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    speed: float = Field(default=1.0, ge=0.1, le=5.0)
    loop: bool = False


class LinearTeachStartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(default="demo_linear_01", min_length=1, max_length=64)
    duration_s: float = Field(default=30.0, ge=2.0, le=300.0)
    fps: int = Field(default=30, ge=10, le=60)


class LinearReplayRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(min_length=1, max_length=64)
    episode: int = Field(default=0, ge=0)
    fps: int = Field(default=30, ge=10, le=60)


class CollectStartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    dataset_name: str = Field(default="arm101_dryrun_demo", min_length=1, max_length=80)
    repo_id: str | None = None
    single_task: str = Field(default="Pick and place cube", min_length=1, max_length=240)
    task: str | None = None
    num_episodes: int = Field(default=2, ge=1, le=200)
    episode_time_s: float = Field(default=5.0, gt=0.1, le=600.0)
    reset_time_s: float = Field(default=1.0, ge=0.0, le=120.0)
    fps: int = Field(default=30, ge=1, le=120)
    push_to_hub: bool = False

    @model_validator(mode="before")
    @classmethod
    def _resolve_aliases(cls, data: Any) -> Any:
        if isinstance(data, dict):
            data = dict(data)
            if "dataset_name" not in data and data.get("repo_id"):
                raw_repo = str(data["repo_id"]).strip()
                if ".." in raw_repo or "\\" in raw_repo:
                    raise ValueError("Invalid repo_id.")
                data["dataset_name"] = raw_repo.split("/")[-1]
            if "single_task" not in data and data.get("task"):
                data["single_task"] = str(data["task"])
        return data

    @field_validator("dataset_name")
    @classmethod
    def _validate_dataset_name(cls, val: str) -> str:
        clean = val.strip()
        if not clean:
            raise ValueError("dataset_name cannot be empty.")
        if clean.startswith("eval_"):
            raise ValueError("dataset_name must not start with 'eval_' (reserved by lerobot-record).")
        if ".." in clean or "/" in clean or "\\" in clean:
            raise ValueError("dataset_name must be a safe folder name without path separators.")
        return clean


class PluginLaunchRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    args: dict[str, Any] = Field(default_factory=dict)


class VLAMockPredictRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    endpoint: str = "ws://127.0.0.1:8000"
    policy_name: str = "pi05_so101_custom"
    task_prompt: str = "Pick up the cube and place it in the bin"


class EstopRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    reason: str = "Web Studio E-Stop"


class TorqueRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enabled: bool


class SimJointsRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    arm: Literal["leader", "follower", "both"] = "follower"
    joints: dict[str, Any] | list[Any]
    linked: bool | None = None

    @field_validator("joints")
    @classmethod
    def _validate_sim_joints(cls, value: dict[str, Any] | list[Any]) -> dict[str, float]:
        return validate_joint_dict(value, require_non_empty=True)


class SimLinkedRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    linked: bool


class SimIKRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    arm: Literal["leader", "follower", "both"] = "follower"
    x_mm: float
    y_mm: float
    z_mm: float
    pitch_deg: float | None = None
    gripper: float | None = Field(default=None, ge=0.0, le=100.0)
    clamp_to_workspace: bool = False


class SimSyncModeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    direction: Literal["off", "real_to_sim", "sim_to_real"] = "off"
    safety_confirmed: bool = False


class ControlKeyRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    key: str = Field(min_length=1, max_length=8)
    step_mm: float = Field(default=8.0, ge=0.5, le=40.0)


class PresetPoseRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    pose: Literal["home", "rest", "inspect", "pick_hover", "wave"]
    duration_s: float = Field(default=1.2, ge=0.1, le=10.0)


class DemoRoutineRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    routine: Literal["wave", "pick"]


class SavePoseRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: Literal["home", "rest"]


class StudioRuntime:
    """Container holding all runtime managers on app.state.runtime."""

    def __init__(self, config: StudioConfig | None = None) -> None:
        self.config = config or load_config()
        self.config.ensure_directories()
        self.started_at = time.monotonic()
        self.shutting_down = False
        self.process_registry = ProcessRegistry(logs_dir=self.config.logs_dir)
        self.hardware = HardwareManager(self.config)
        self.simulator = Arm101Simulator()
        self.workflows = WorkflowManager(
            self.config, self.hardware, self.simulator, self.process_registry
        )
        self.plugins = PluginRegistry(self.config, self.hardware, self.process_registry)


def create_app(config: StudioConfig | None = None) -> FastAPI:
    """Create and configure the Arm101 Studio FastAPI application."""

    @asynccontextmanager
    async def lifespan(fastapi_app: FastAPI):
        runtime = StudioRuntime(config=config)
        fastapi_app.state.runtime = runtime
        probe_data = runtime.hardware.probe()
        runtime.workflows.log(
            f"Arm101 Studio started in '{probe_data['active_mode']}' mode "
            f"(requested='{probe_data['requested_mode']}')."
        )
        # Always-on Joint State Broadcaster: in real mode, automatically start passive real_to_sim
        # monitoring on boot so the 3D viewport stays live-synced without manual activation.
        if probe_data.get("active_mode") == "real":
            async def _auto_start_broadcaster() -> None:
                try:
                    await asyncio.to_thread(
                        runtime.workflows.bridge.set_sync_mode,
                        "real_to_sim",
                        is_real_mode=True,
                        safety_confirmed=False,
                    )
                    runtime.workflows.simulator.torque_enabled = False
                    runtime.workflows.log(
                        "Auto-started always-on real_to_sim joint state broadcaster (Freedrive monitor)."
                    )
                except Exception as exc:
                    runtime.workflows.log(
                        f"Auto-start real_to_sim monitor skipped ({exc}); click readiness pill to retry."
                    )

            asyncio.create_task(_auto_start_broadcaster())
        try:
            yield
        finally:
            runtime.shutting_down = True
            await runtime.workflows.shutdown()
            runtime.plugins.stop_all()
            runtime.process_registry.terminate_all()

    fastapi_app = FastAPI(
        title="TUST-iMaker Arm101 Studio",
        version="0.1.0",
        lifespan=lifespan,
    )

    cfg_for_static = config or load_config()
    if cfg_for_static.frontend_dir.exists():
        fastapi_app.mount(
            "/static",
            StaticFiles(directory=str(cfg_for_static.frontend_dir)),
            name="static",
        )

    def _rt() -> StudioRuntime:
        return fastapi_app.state.runtime  # type: ignore[no-any-return]

    def _asset_version(frontend_dir: Path) -> str:
        try:
            mtimes = [int(p.stat().st_mtime * 1000) for p in frontend_dir.glob("*") if p.is_file()]
            return str(max(mtimes)) if mtimes else str(int(time.time() * 1000))
        except Exception:
            return str(int(time.time() * 1000))

    @fastapi_app.middleware("http")
    async def disable_static_cache(request: Request, call_next: Any) -> Response:
        path = request.url.path
        if path in ("/", "/sim-viewport") or path.startswith("/static/"):
            # Suppress conditional If-None-Match / If-Modified-Since so browsers never reuse stale 304 cache
            request.scope["headers"] = [
                (k, v)
                for (k, v) in request.scope.get("headers", [])
                if k.lower() not in (b"if-none-match", b"if-modified-since")
            ]
        response: Response = await call_next(request)
        if path in ("/", "/sim-viewport") or path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
            response.headers["Pragma"] = "no-cache"
            response.headers["Expires"] = "0"
            for hdr in ("etag", "last-modified"):
                if hdr in response.headers:
                    del response.headers[hdr]
        return response

    def _render_versioned_html(filename: str) -> HTMLResponse:
        frontend_dir = _rt().config.frontend_dir
        html_file = frontend_dir / filename
        if not html_file.exists():
            raise HTTPException(status_code=404, detail=f"Frontend {filename} not found.")
        ver = _asset_version(frontend_dir)
        raw_html = html_file.read_text(encoding="utf-8")
        versioned_html = re.sub(
            r'(/static/(?:styles\.css|app\.js|stick_sim\.js|studio_bus\.js|sim_viewport\.js))(?:\?v=[^"\'\s>]*)?',
            rf"\1?v={ver}",
            raw_html,
        )
        return HTMLResponse(
            content=versioned_html,
            media_type="text/html; charset=utf-8",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "Pragma": "no-cache",
                "Expires": "0",
            },
        )

    # ------------------------------------------------------------------
    # Frontend Root & Standalone Sim Viewport
    # ------------------------------------------------------------------
    @fastapi_app.get("/", include_in_schema=False)
    async def index() -> HTMLResponse:
        return _render_versioned_html("index.html")

    @fastapi_app.get("/sim-viewport", include_in_schema=False)
    async def sim_viewport() -> HTMLResponse:
        return _render_versioned_html("sim_viewport.html")

    # ------------------------------------------------------------------
    # System, Hardware & Mode
    # ------------------------------------------------------------------
    @fastapi_app.get("/api/health")
    async def health() -> dict[str, Any]:
        rt = _rt()
        return {
            "status": "ok",
            "os": platform.system(),
            "python_version": platform.python_version(),
            "mode": rt.hardware.active_mode,
            "requested_mode": rt.hardware.requested_mode,
            "workspace_root": str(rt.config.workspace_root),
            "uptime_s": round(time.monotonic() - rt.started_at, 2),
        }

    @fastapi_app.get("/api/hardware/probe")
    async def hardware_probe() -> dict[str, Any]:
        rt = _rt()
        await asyncio.to_thread(rt.workflows.release_cameras)
        data = await asyncio.to_thread(rt.hardware.probe)
        data["robot_profile"] = rt.config.load_robot_profile()
        return data

    @fastapi_app.post("/api/hardware/config")
    async def hardware_config(req: HardwareConfigRequest) -> dict[str, Any]:
        rt = _rt()
        if req.auto_detect:
            serial_ports = await asyncio.to_thread(rt.hardware.probe_serial_ports)
            cameras = await asyncio.to_thread(rt.hardware.probe_cameras)
            rt.hardware.auto_match_devices(serial_ports, cameras, force=True)
        else:
            f_port = (
                req.follower_port.strip()
                if req.follower_port is not None
                else rt.config.follower_port
            )
            l_port = (
                req.leader_port.strip()
                if req.leader_port is not None
                else rt.config.leader_port
            )
            if (
                f_port
                and l_port
                and f_port.lower() != "none"
                and l_port.lower() != "none"
                and f_port == l_port
            ):
                raise HTTPException(
                    status_code=422,
                    detail="从臂串口与主臂串口不能选择同一个端口。",
                )

            front_cam = (
                req.front_camera_id.strip()
                if req.front_camera_id is not None
                else rt.config.front_camera_id
            )
            wrist_cam = (
                req.wrist_camera_id.strip()
                if req.wrist_camera_id is not None
                else rt.config.wrist_camera_id
            )
            if (
                front_cam
                and wrist_cam
                and front_cam.lower() != "none"
                and wrist_cam.lower() != "none"
                and front_cam == wrist_cam
            ):
                raise HTTPException(
                    status_code=422,
                    detail="前置相机与腕部相机不能选择同一个摄像头设备。",
                )

            if f_port:
                rt.config.follower_port = f_port
            if l_port:
                rt.config.leader_port = l_port
            if req.follower_id is not None and req.follower_id.strip():
                rt.config.follower_id = req.follower_id.strip()
            if req.leader_id is not None and req.leader_id.strip():
                rt.config.leader_id = req.leader_id.strip()
            if req.front_camera_id is not None:
                rt.config.front_camera_id = front_cam or "none"
            if req.wrist_camera_id is not None:
                rt.config.wrist_camera_id = wrist_cam or "none"

        saved_env = rt.config.save_device_overrides(persist_env=req.persist_env)
        probe_data = await asyncio.to_thread(rt.hardware.probe, auto_match=bool(req.auto_detect))
        probe_data["robot_profile"] = rt.config.load_robot_profile()
        probe_data["saved_env"] = saved_env
        probe_data["persisted_to_env"] = req.persist_env
        rt.workflows.log(
            f"Updated device mapping: follower={rt.config.follower_port}, "
            f"leader={rt.config.leader_port}, front_cam={rt.config.front_camera_id}, "
            f"wrist_cam={rt.config.wrist_camera_id}."
        )
        return probe_data

    @fastapi_app.post("/api/mode")
    async def set_mode(req: ModeRequest) -> JSONResponse:
        rt = _rt()
        ok, payload = await asyncio.to_thread(rt.hardware.set_mode, req.mode)
        if not ok:
            rt.workflows.log(f"Mode switch to '{req.mode}' rejected: hardware not ready.")
            return JSONResponse(status_code=409, content=payload)
        rt.workflows.log(f"Operating mode set to '{payload['mode']}' (requested='{req.mode}').")
        return JSONResponse(status_code=200, content=payload)

    # ------------------------------------------------------------------
    # Workflows
    # ------------------------------------------------------------------
    @fastapi_app.get("/api/workflows/status")
    async def workflow_status() -> dict[str, Any]:
        return _rt().workflows.get_status()

    @fastapi_app.post("/api/workflows/calibrate/start")
    async def calibrate_start(req: CalibrateStartRequest | None = None) -> dict[str, Any]:
        body = req or CalibrateStartRequest()
        try:
            return await _rt().workflows.start_calibrate(
                target=body.target,
                port=body.port,
                arm_id=body.arm_id,
                calibration_dir=body.calibration_dir,
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/calibrate/input")
    async def calibrate_input(req: CalibrateInputRequest | None = None) -> dict[str, Any]:
        body = req or CalibrateInputRequest()
        try:
            return await _rt().workflows.send_calibrate_input(body.value)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/calibrate/stop")
    async def calibrate_stop() -> dict[str, Any]:
        return await _rt().workflows.stop_calibrate()

    @fastapi_app.post("/api/workflows/teleop/start")
    async def teleop_start(req: TeleopStartRequest | None = None) -> dict[str, Any]:
        body = req or TeleopStartRequest()
        try:
            return await _rt().workflows.start_teleop(
                fps=body.fps,
                follower_port=body.follower_port,
                leader_port=body.leader_port,
                display_data=body.display_data,
                use_cameras=body.use_cameras,
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/teleop/stop")
    async def teleop_stop() -> dict[str, Any]:
        return await _rt().workflows.stop_teleop()

    @fastapi_app.post("/api/workflows/teleop/joints")
    async def teleop_joints(req: TeleopJointsRequest) -> dict[str, Any]:
        try:
            return _rt().workflows.update_teleop_joints(req.joints)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/teaching/record_waypoint")
    async def teaching_record_waypoint(
        req: WaypointRecordRequest | None = None,
    ) -> dict[str, Any]:
        body = req or WaypointRecordRequest()
        return _rt().workflows.record_waypoint(label=body.label, joints=body.joints)

    @fastapi_app.get("/api/workflows/teaching/waypoints")
    async def teaching_waypoints() -> dict[str, Any]:
        return _rt().workflows.list_waypoints()

    @fastapi_app.post("/api/workflows/teaching/replay")
    async def teaching_replay(req: ReplayRequest | None = None) -> dict[str, Any]:
        body = req or ReplayRequest()
        try:
            return await _rt().workflows.replay_waypoints(speed=body.speed, loop=body.loop)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/teaching/clear")
    async def teaching_clear() -> dict[str, Any]:
        return await _rt().workflows.clear_waypoints()

    @fastapi_app.get("/api/workflows/teaching/linear/list")
    async def teaching_linear_list() -> dict[str, Any]:
        return _rt().workflows.list_linear_trajectories()

    @fastapi_app.post("/api/workflows/teaching/linear/start")
    async def teaching_linear_start(
        req: LinearTeachStartRequest | None = None,
    ) -> dict[str, Any]:
        body = req or LinearTeachStartRequest()
        try:
            return await _rt().workflows.start_linear_teach(
                name=body.name, duration_s=body.duration_s, fps=body.fps
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/teaching/linear/stop")
    async def teaching_linear_stop() -> dict[str, Any]:
        return await _rt().workflows.stop_linear_teach()

    @fastapi_app.post("/api/workflows/teaching/linear/replay")
    async def teaching_linear_replay(req: LinearReplayRequest) -> dict[str, Any]:
        try:
            return await _rt().workflows.replay_linear_teach(
                name=req.name, episode=req.episode, fps=req.fps
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/teaching/linear/replay/stop")
    async def teaching_linear_replay_stop() -> dict[str, Any]:
        return await _rt().workflows.stop_linear_replay()

    @fastapi_app.post("/api/workflows/teaching/linear/delete")
    async def teaching_linear_delete(req: LinearReplayRequest) -> dict[str, Any]:
        return _rt().workflows.delete_linear_trajectory(name=req.name, episode=req.episode)

    @fastapi_app.post("/api/workflows/collect/start")
    async def collect_start(req: CollectStartRequest | None = None) -> dict[str, Any]:
        body = req or CollectStartRequest()
        try:
            return await _rt().workflows.start_collect(
                dataset_name=body.dataset_name,
                single_task=body.single_task,
                num_episodes=body.num_episodes,
                episode_time_s=body.episode_time_s,
                reset_time_s=body.reset_time_s,
                fps=body.fps,
                push_to_hub=body.push_to_hub,
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/collect/stop")
    async def collect_stop() -> dict[str, Any]:
        return await _rt().workflows.stop_collect()

    @fastapi_app.get("/api/workflows/collect/episodes")
    async def collect_episodes() -> dict[str, Any]:
        return _rt().workflows.list_episodes()

    @fastapi_app.get("/api/workflows/collect/episodes/{dataset_name}/{episode_index}")
    async def collect_episode_detail(dataset_name: str, episode_index: int) -> dict[str, Any]:
        try:
            return _rt().workflows.get_episode_frames(dataset_name, episode_index)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc))
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail=str(exc))

    # ------------------------------------------------------------------
    # Safety Kernel, Independent Dual-Arm Sim, IK & aguiya Presets
    # ------------------------------------------------------------------
    @fastapi_app.post("/api/safety/estop")
    async def safety_estop(req: EstopRequest | None = None) -> dict[str, Any]:
        body = req or EstopRequest()
        return await _rt().workflows.trigger_estop(reason=body.reason)

    @fastapi_app.post("/api/safety/reset")
    async def safety_reset() -> dict[str, Any]:
        return await _rt().workflows.reset_estop()

    @fastapi_app.post("/api/safety/torque")
    async def safety_torque(req: TorqueRequest) -> dict[str, Any]:
        try:
            return await _rt().workflows.set_torque(req.enabled)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/sim/joints")
    async def sim_joints(req: SimJointsRequest) -> dict[str, Any]:
        try:
            return _rt().workflows.set_sim_joints(
                arm=req.arm, raw_joints=req.joints, linked=req.linked
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/sim/linked")
    async def sim_linked(req: SimLinkedRequest) -> dict[str, Any]:
        return _rt().workflows.set_linked_arms(req.linked)

    @fastapi_app.post("/api/sim/ik")
    async def sim_ik(req: SimIKRequest) -> dict[str, Any]:
        try:
            return _rt().workflows.apply_ik_target(
                arm=req.arm,
                x_mm=req.x_mm,
                y_mm=req.y_mm,
                z_mm=req.z_mm,
                pitch_deg=req.pitch_deg,
                gripper=req.gripper,
                clamp_to_workspace=req.clamp_to_workspace,
            )
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc))

    @fastapi_app.post("/api/sim/sync_mode")
    async def sim_sync_mode(req: SimSyncModeRequest) -> dict[str, Any]:
        try:
            return await asyncio.to_thread(
                _rt().workflows.set_sync_mode,
                direction=req.direction,
                safety_confirmed=req.safety_confirmed,
            )
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc))
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc))

    @fastapi_app.post("/api/workflows/control/key")
    async def workflow_control_key(req: ControlKeyRequest) -> dict[str, Any]:
        try:
            return _rt().workflows.send_control_key(req.key, step_mm=req.step_mm)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc))

    @fastapi_app.post("/api/workflows/teaching/pose")
    async def teaching_pose(req: PresetPoseRequest) -> dict[str, Any]:
        try:
            return await _rt().workflows.run_preset_pose(req.pose, duration_s=req.duration_s)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc))

    @fastapi_app.post("/api/workflows/teaching/routine")
    async def teaching_routine(req: DemoRoutineRequest) -> dict[str, Any]:
        try:
            return await _rt().workflows.run_demo_routine(req.routine)
        except WorkflowConflictError as exc:
            raise HTTPException(status_code=409, detail={"detail": exc.detail, "code": exc.code})

    @fastapi_app.post("/api/workflows/teaching/save_pose")
    async def teaching_save_pose(req: SavePoseRequest) -> dict[str, Any]:
        return _rt().workflows.save_custom_pose(req.target)

    # ------------------------------------------------------------------
    # Cameras
    # ------------------------------------------------------------------
    @fastapi_app.get("/api/cameras/{cam_id}/frame")
    async def camera_frame(cam_id: str, theme: str = "light") -> Response:
        try:
            jpg = await asyncio.to_thread(_rt().workflows.get_camera_jpeg, cam_id, theme)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        return Response(
            content=jpg,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    @fastapi_app.get("/api/cameras/{cam_id}/stream")
    async def camera_stream(cam_id: str, request: Request, theme: str = "light") -> StreamingResponse:
        wf = _rt().workflows
        if cam_id not in wf._valid_camera_ids():
            raise HTTPException(status_code=404, detail=f"Camera '{cam_id}' not found.")

        async def _mjpeg_generator():
            rt = _rt()
            while not rt.shutting_down:
                if await request.is_disconnected():
                    break
                try:
                    jpg = await asyncio.to_thread(wf.get_camera_jpeg, cam_id, theme)
                except Exception:
                    break
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                    + jpg
                    + b"\r\n"
                )
                interval = 0.04 if wf.hardware.active_mode == "real" else 0.066
                await asyncio.sleep(interval)

        return StreamingResponse(
            _mjpeg_generator(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )

    # ------------------------------------------------------------------
    # Open-Source Tool Plugins & VLA Config Viewer
    # ------------------------------------------------------------------
    @fastapi_app.get("/api/plugins")
    async def list_plugins() -> dict[str, Any]:
        return {"plugins": _rt().plugins.list_plugins()}

    @fastapi_app.get("/api/plugins/open_source_datasets/preview")
    async def plugin_dataset_preview(repo_id: str | None = None) -> dict[str, Any]:
        try:
            return get_dataset_preview(repo_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc))

    @fastapi_app.get("/api/plugins/feetech_servo_bench/diagnostics")
    async def plugin_servo_diagnostics() -> dict[str, Any]:
        rt = _rt()
        return get_servo_bench_diagnostics(rt.simulator, rt.hardware.active_mode)

    @fastapi_app.post("/api/plugins/{plugin_id}/launch")
    async def launch_plugin(
        plugin_id: str, req: PluginLaunchRequest | None = None
    ) -> JSONResponse:
        body = req or PluginLaunchRequest()
        rt = _rt()
        try:
            res = rt.plugins.launch(plugin_id, body.args)
            rt.workflows.log(f"Launched plugin '{plugin_id}' (pid={res.get('pid')}).")
            return JSONResponse(status_code=200, content=res)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc))
        except PluginUnavailableError as exc:
            rt.workflows.log(f"Plugin '{plugin_id}' unavailable: {exc.detail}")
            return JSONResponse(
                status_code=409,
                content={
                    "detail": exc.detail,
                    "code": exc.code,
                    "plugin_id": plugin_id,
                    "install_hint": exc.install_hint,
                },
            )

    @fastapi_app.post("/api/plugins/{plugin_id}/stop")
    async def stop_plugin(plugin_id: str) -> dict[str, Any]:
        rt = _rt()
        try:
            res = rt.plugins.stop(plugin_id)
            rt.workflows.log(f"Stopped plugin '{plugin_id}'.")
            return res
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc))

    @fastapi_app.get("/api/vla/configs")
    async def vla_configs() -> dict[str, Any]:
        rt = _rt()
        return {
            "configs": rt.config.load_vla_configs(),
            "pi07_notice": {
                "title": "关于 pi0.7 (π0.7) 的公开权重说明",
                "paper": "arXiv:2604.15483v2",
                "status": "闭源/未公开权重（截至当前开源状态）",
                "recommendation": (
                    "目前 Physical-Intelligence/openpi 与 Hugging Face LeRobot 中实际可下载权重并复现训练的是 "
                    "pi0 (π0)、pi0_fast、pi05 (π0.5) 与 SmolVLA。工作区 configs/train/ 已提供对应即用模板。"
                ),
            },
        }

    @fastapi_app.post("/api/vla/mock_predict")
    async def vla_mock_predict(req: VLAMockPredictRequest) -> dict[str, Any]:
        rt = _rt()
        _, follower, _ = rt.simulator.snapshot()
        base_pan = float(follower.get("shoulder_pan", 0.0))
        base_lift = float(follower.get("shoulder_lift", 0.0))
        base_elbow = float(follower.get("elbow_flex", 0.0))
        base_wrist = float(follower.get("wrist_flex", 0.0))
        base_roll = float(follower.get("wrist_roll", 0.0))
        waypoints_5 = [
            (-18.0, 12.0, 10.0, -16.0, 0.0, 75.0),
            (-6.0, 24.0, 20.0, -32.0, 8.0, 75.0),
            (4.0, 34.0, 28.0, -48.0, 12.0, 18.0),
            (18.0, 18.0, 14.0, -24.0, 6.0, 18.0),
            (32.0, 28.0, 22.0, -38.0, 0.0, 70.0),
        ]
        chunk = []
        for step, (d_pan, d_lift, d_elbow, d_wrist, d_roll, grip) in enumerate(waypoints_5, start=1):
            chunk.append(
                {
                    "step": step,
                    "shoulder_pan": round(max(-110.0, min(110.0, base_pan * 0.2 + d_pan)), 2),
                    "shoulder_lift": round(max(-100.0, min(100.0, base_lift * 0.2 + d_lift)), 2),
                    "elbow_flex": round(max(-97.0, min(97.0, base_elbow * 0.2 + d_elbow)), 2),
                    "wrist_flex": round(max(-95.0, min(95.0, base_wrist * 0.2 + d_wrist)), 2),
                    "wrist_roll": round(max(-157.0, min(163.0, base_roll * 0.2 + d_roll)), 2),
                    "gripper": round(grip, 2),
                }
            )
        rt.workflows.set_vla_chunk(chunk, badge=f"VLA 5 步预测 ({req.policy_name})")
        rt.workflows.log(
            f"VLA preview chunk generated for policy '{req.policy_name}' ({req.endpoint})."
        )
        return {
            "status": "ok",
            "mode": rt.hardware.active_mode,
            "endpoint": req.endpoint,
            "policy_name": req.policy_name,
            "task_prompt": req.task_prompt,
            "action_chunk": chunk,
        }

    # ------------------------------------------------------------------
    # WebSocket Telemetry (25~30Hz during motion + Dual-Frame Split)
    # ------------------------------------------------------------------
    @fastapi_app.websocket("/ws/telemetry")
    async def ws_telemetry(websocket: WebSocket) -> None:
        await websocket.accept()
        wf = _rt().workflows
        last_overlay_seq = -1
        try:
            # First frame: telemetry (satisfies single-frame WS health tests)
            await websocket.send_json(wf.get_telemetry_frame())
            # Second frame: initial overlay_update snapshot
            ov = wf.get_overlay_frame()
            last_overlay_seq = ov["overlay_seq"]
            await websocket.send_json(ov)

            while not _rt().shutting_down:
                now = time.monotonic()
                is_active = wf.active_workflow != "idle" or (now - wf.last_motion_ts) < 1.5
                timeout_s = 0.033 if is_active else 0.10
                try:
                    await asyncio.wait_for(wf.motion_event.wait(), timeout=timeout_s)
                    wf.motion_event.clear()
                except asyncio.TimeoutError:
                    pass

                await websocket.send_json(wf.get_telemetry_frame())
                if wf.overlay_seq != last_overlay_seq:
                    ov = wf.get_overlay_frame()
                    last_overlay_seq = ov["overlay_seq"]
                    await websocket.send_json(ov)
        except WebSocketDisconnect:
            return
        except Exception:
            return

    return fastapi_app


app = create_app()


if __name__ == "__main__":
    cfg = load_config()
    uvicorn.run(
        "app.backend.main:app",
        host=cfg.host,
        port=cfg.port,
        log_level="info",
    )
