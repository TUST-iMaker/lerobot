---
name: robotics-ide-layout
description: Trigger whenever modifying index.html, sim_viewport.html, styles.css, or adding/removing UI panels and controls.
---

# Utilitarian Daylight Robotics IDE Layout Contract

## 1. Visual & Typographic Discipline (`minimalist-ui` + `ponytail`)
- **All-White / Daylight Palette (`color-scheme: light`)**:
  - App background `#F7F6F3`, Panel/Card/Popup surface `#FFFFFF`, 3D & Camera viewport surface `#F8F9FA`, 1px structural borders `#E5E5E0`, primary text `#18181B`, secondary text `#64645E`.
  - Every popup, `<select>` dropdown, floating drawer, synthetic camera JPEG, and `/sim-viewport` window MUST use this daylight palette—no dark/black mismatches.
- **Zero Emojis & Zero Explanatory Bloat**:
  - Do not use emojis anywhere in HTML, CSS, or JS strings.
  - Omit unnecessary secondary subtitle paragraphs and bilingual parenthetical labels. Let the layout hierarchy explain itself.
- **Tabular-Nums Anti-Jitter**:
  - Joint slider rows use `grid-template-columns: 140px minmax(0, 1fr) 124px` with `font-family: var(--font-mono); font-variant-numeric: tabular-nums; white-space: nowrap;` on the `124px` readout column.

## 2. Full-Viewport 100vh Grid & Anti-Overflow Rules
- Outer shell: Left collapsible sidebar (`176px` / `52px`) + Global Persistent 3D Stage Dock (`#global-stage-dock`, `340~520px` resizable, **Leader and Follower stacked vertically**, shared compact camera bar) + Right Main Column (`46px` topbar, `minmax(0, 1fr)` workspace, `26px` statusbar).
- Every panel inside `#workspace-main` uses `min-height: 100%; display: flex; flex-direction: column; flex: 1` so cards stretch cleanly to the bottom without empty whitespace.
- Enable `container-type: inline-size` on `#workspace-main`; at `@container (max-width: 680px)`, dual-column slider/card grids collapse to a single column so `1280x720` has zero horizontal overflow.
