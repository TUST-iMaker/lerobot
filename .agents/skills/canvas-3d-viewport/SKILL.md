---
name: canvas-3d-viewport
description: Trigger whenever modifying stick_sim.js, 3D kinematic rendering, camera orbit/zoom/pan controls, red-ring IK dragging, or canvas ResizeObserver logic.
---

# 3D Kinematic Canvas & Camera Projection Contract

## 1. Orthographic-Isometric Projection & Camera Transform
The 3D canvas projects world coordinates $(x, y, z)$ in millimeters to logical CSS coordinates $(x_s, y_s)$ using:
- Yaw/Pitch rotation around $Z$ and $X$:
  $$r_x = x \cos(\text{yaw}) - y \sin(\text{yaw}), \quad r_y = x \sin(\text{yaw}) + y \cos(\text{yaw}), \quad s_y = z \cos(\text{pitch}) - r_y \sin(\text{pitch})$$
- Zoom & Pan mapping:
  $$\text{baseScale} = \frac{\min(0.90 w, 1.35 h)}{460}, \quad \text{effectiveScale} = \text{baseScale} \cdot \text{camZoom}$$
  $$x_s = 0.34 w + \text{camPanX} + r_x \cdot \text{effectiveScale}, \quad y_s = 0.79 h + \text{camPanY} - s_y \cdot \text{effectiveScale}$$
- Clamping:
  - `camZoom` clamped to `[0.45, 2.80]`.
  - `camPanX` clamped to `[-0.45 * w, +0.45 * w]`; `camPanY` clamped to `[-0.45 * h, +0.45 * h]`.

## 2. Hit-Testing & Zoom-Compensated IK Inverse Mapping
- Hit-testing for the red TCP IK target ring MUST compare pointer logical coordinates directly against `this._lastTcpScreen` (the projected screen coordinates $(x_s, y_s)$, which already include `camZoom` and `camPanX/Y`).
- When dragging the IK ring, convert screen pixel deltas $(\Delta x_s, \Delta y_s)$ back to millimeter workspace deltas by dividing by `effectiveScale`:
  $$\Delta r = \frac{\Delta x_s}{\text{effectiveScale}}, \quad \Delta z = \frac{-\Delta y_s}{\text{effectiveScale}}$$

## 3. DPR Scaling, Event Interception & ResizeObserver Safety
- Always draw in logical CSS coordinates via `ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0)` while sizing backing store `canvas.width = Math.round(logicalW * dpr)`.
- Bind `wheel` with `{ passive: false }` and call `ev.preventDefault()`; intercept `contextmenu` (`ev.preventDefault()`) for right-click panning.
- In `ResizeObserver`, strictly ignore hidden/collapsed states (`contentRect.width < 20 || contentRect.height < 20`) so the canvas never locks to `0x0`. Force `refreshSize()` when unpausing or expanding the stage dock.
- Daylight Theme Consistency: Canvas backdrop must use clean architectural daylight paper (`#F8F9FA`) with dark slate grid lines and high-contrast cobalt/cyan links—never a black hole on a white page.
