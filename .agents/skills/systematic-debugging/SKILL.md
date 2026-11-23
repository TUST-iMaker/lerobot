---
name: systematic-debugging
description: Trigger when diagnosing WebSocket sync jitter, Canvas coordinate offset, IK drag drift, or cross-window communication bugs.
---

# Root-Cause-First Debugging Protocol

Never guess-and-patch when fixing WebSocket synchronization jitter or 3D Canvas coordinate offsets:

1. **Capture Raw Telemetry & Overlay Frames First**:
   - Connect to `/ws/telemetry` and inspect one complete `telemetry` JSON payload and one `overlay_update` JSON payload before touching frontend state handlers.
2. **Log Intermediate Projection & Scale Values**:
   - For Canvas or IK drag issues, inspect `(logicalW, logicalH, dpr, camZoom, camPanX, camPanY, effectiveScale)` and compare `_project(pts.tcp)` against `_lastTcpScreen` and pointer logical coordinates.
3. **Fix at the Single Source of Truth**:
   - Apply the minimal fix at the root producer or shared bus (`workflows.py`, `stick_sim.js`, or `studio_bus.js`) and verify with a reproducible Playwright check.
