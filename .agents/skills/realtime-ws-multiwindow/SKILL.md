---
name: realtime-ws-multiwindow
description: Trigger whenever modifying /ws/telemetry, state broadcasting, studio_bus.js, or /sim-viewport multi-window synchronization and mutual exclusion.
---

# Realtime WebSocket & Multi-Window Synchronization Contract

## 1. Dual-Frame WebSocket Protocol (`/ws/telemetry`)
1. **High-Frequency `telemetry` Frame (`25~30Hz` during active motion/workflows, immediate push on manual write, `10Hz` when idle)**:
   - Carries `type: "telemetry"`, `overlay_seq`, `overlay_badge`, `mode`, `active_workflow`, `leader_joints`, `follower_joints`, `leader_fk`, `follower_fk`, `linked_arms`, `safety`, `connection_status`, `calibration`, `workflow_status`, `log_tail`.
   - NEVER embed large static `waypoints` or `vla_chunk` arrays in high-frequency `telemetry` frames.
2. **Low-Frequency `overlay_update` Frame**:
   - Sent immediately after initial connection and whenever `overlay_seq` increments.
   - Carries `type: "overlay_update"`, `overlay_seq`, `waypoints`, `vla_chunk`, `overlay_badge`.

## 2. Control Write Arbitration & Slider Anti-Jitter Lock
- **Backend Arbitration**:
  - When `active_workflow in ("calibrate", "collect")`, reject manual slider/IK/WASD writes with `HTTP 409 WORKFLOW_BUSY`.
  - When `active_workflow in ("teaching_replay", "demo_routine", "preview_replay")`, manual slider/IK/WASD writes automatically preempt and cancel the background replay task so manual control never fights automated replay.
- **Frontend Slider Lock**:
  - Track `activeSliderId` on `pointerdown`/`input` plus a `250ms` post-input cooldown timestamp per joint slider. Skip telemetry writeback to `slider.value` while locked.

## 3. `/sim-viewport` Web Locks + BroadcastChannel Mutual Exclusion
- `/sim-viewport` holds an exclusive browser lock via `navigator.locks.request('arm101_sim_viewport', () => new Promise(() => {}))` for its entire lifetime and posts `{ type: 'OPENED' }` / `{ type: 'CLOSED' }` on `BroadcastChannel('arm101_stage_bus')`.
- Because `navigator.locks` is managed by the browser kernel, background tab timer throttling NEVER causes false-positive timeouts, and closing/crashing `/sim-viewport` releases the lock instantaneously, waking the main window's built-in 3D stage.
