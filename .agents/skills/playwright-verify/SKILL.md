---
name: playwright-verify
description: Mandatory verification skill triggered after any frontend HTML, CSS, JS, or 3D Canvas change.
---

# Browser Visual & DOM Automated Acceptance Protocol

Never claim a frontend or UI change is complete by reading code alone. After modifying any frontend file, start the live server and execute Playwright verification at **both `1440x900` and `1280x720` viewports** with these 4 hard assertions:

1. **Zero Horizontal Overflow & Full-Height Fit**:
   - Assert `document.documentElement.scrollWidth <= document.documentElement.clientWidth` across all 7 modules and `/sim-viewport`.
2. **Zero Console or Page Errors**:
   - Capture `page.on("console")` (`type == "error"`) and `page.on("pageerror")`; assert both lists are empty (`[]`).
3. **Non-Uniform / Non-Blank Canvas Pixel Assertion**:
   - Sample the 2D canvas pixel buffer (`ctx.getImageData`) on active 3D viewports and assert that distinct non-background pixels (robotic arm links and grid lines) are rendered.
4. **Zero `NaN` / `undefined` in Telemetry DOM**:
   - Assert that the visible text of `#workspace-main` and `#global-stage-dock` contains no `NaN` or `undefined`.
