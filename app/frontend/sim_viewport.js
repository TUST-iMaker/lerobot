/**
 * Standalone Virtual Hardware Twin Window (/sim-viewport)
 * Synchronizes bidirectionally with the Main Studio via /ws/telemetry & Web Locks mutex.
 */
import {
  StickArmCanvas,
  renderStickTelemetryHtml,
  computeStickFK,
  fmtZero,
} from "/static/stick_sim.js?v=2026112305";
import {
  JOINTS,
  apiFetch,
  createStudioSocket,
  StagePopupMutex,
  getStoredTheme,
  applyThemeToDocument,
} from "/static/studio_bus.js?v=2026112305";

const $ = (id) => document.getElementById(id);

const twinState = {
  theme: getStoredTheme(),
  layoutMode: "dual", // "dual" | "quad-follower" | "quad-leader"
  linkedArms: true,
  leaderJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  followerJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  waypoints: [],
  vlaChunk: [],
  overlayBadge: "",
  activeWorkflow: "idle",
  estopLatched: false,
};

let leaderView = null;
let followerView = null;
const quadViews = {}; // { iso, front, side, top }
let ikInFlight = false;
let ikPending = null;

function isQuadMode() {
  return twinState.layoutMode !== "dual";
}

function getQuadArmRole() {
  return twinState.layoutMode === "quad-leader" ? "leader" : "follower";
}

function syncTwinIkMutex() {
  const inCal = twinState.activeWorkflow === "calibrate";
  const linked = Boolean(twinState.linkedArms);
  if (inCal) {
    leaderView?.setIkEnabled(false);
    followerView?.setIkEnabled(false);
  } else if (linked) {
    leaderView?.setIkEnabled(true);
    followerView?.setIkEnabled(false);
  } else {
    leaderView?.setIkEnabled(true);
    followerView?.setIkEnabled(true);
  }

  // Quad-view IK: enabled for the displayed arm unless calibrating (or follower while linked -> allow leader IK or auto-route)
  const quadRole = getQuadArmRole();
  const quadIkOk = !inCal && (quadRole === "leader" || !linked || quadRole === "follower");
  Object.values(quadViews).forEach((v) => {
    if (!v) return;
    v.role = quadRole;
    v.setIkEnabled(quadIkOk);
  });
}

function isAnyViewDraggingIk() {
  if (leaderView?.isDraggingIk() || followerView?.isDraggingIk()) return true;
  return Object.values(quadViews).some((v) => v?.isDraggingIk?.());
}

function setTwinLayoutMode(mode) {
  twinState.layoutMode = mode;
  const dualEl = $("sim-twin-dual");
  const quadEl = $("sim-twin-quad");
  const quad = mode !== "dual";
  if (dualEl) dualEl.hidden = quad;
  if (quadEl) quadEl.hidden = !quad;

  document.querySelectorAll(".btn-twin-layout").forEach((btn) => {
    const active = btn.dataset.layout === mode;
    btn.classList.toggle("btn-primary", active);
    btn.classList.toggle("btn-secondary", !active);
  });

  if (quad) {
    leaderView?.setPaused(true);
    followerView?.setPaused(true);
    ensureQuadViews();
    const role = getQuadArmRole();
    const viewLabels = {
      iso: `${role === "leader" ? "Leader 主臂" : "Follower 从臂"} · 轴测`,
      front: `${role === "leader" ? "Leader 主臂" : "Follower 从臂"} · 正视`,
      side: `${role === "leader" ? "Leader 主臂" : "Follower 从臂"} · 侧视`,
      top: `${role === "leader" ? "Leader 主臂" : "Follower 从臂"} · 俯视`,
    };
    Object.entries(quadViews).forEach(([k, v]) => {
      v.role = role;
      v.label = viewLabels[k];
      v.setPaused(false);
      v.setPresetView(k);
    });
    if ($("quad-arm-title")) {
      $("quad-arm-title").textContent =
        role === "leader" ? "Leader 主臂 · 四视图同步" : "Follower 从臂 · 四视图同步";
    }
  } else {
    Object.values(quadViews).forEach((v) => v?.setPaused(true));
    leaderView?.setPaused(false);
    followerView?.setPaused(false);
  }
  syncTwinIkMutex();
  renderTwinViews();
}

function ensureQuadViews() {
  if (quadViews.iso) return;
  const specs = [
    { key: "iso", id: "canvas-quad-iso", label: "轴测 Iso" },
    { key: "front", id: "canvas-quad-front", label: "正视 Front" },
    { key: "side", id: "canvas-quad-side", label: "侧视 Side" },
    { key: "top", id: "canvas-quad-top", label: "俯视 Top" },
  ];
  for (const s of specs) {
    const el = $(s.id);
    if (!el) continue;
    const v = new StickArmCanvas(el, {
      role: getQuadArmRole(),
      label: s.label,
      enableIkDrag: true,
      onIkDrag: (req) => {
        // When linked & dragging follower in quad-follower, route via leader so both move cleanly
        const arm = twinState.linkedArms ? "leader" : getQuadArmRole();
        ikPending = { ...req, arm };
        drainIkQueue();
      },
    });
    v.setPresetView(s.key);
    quadViews[s.key] = v;
  }
}

async function drainIkQueue() {
  if (ikInFlight || !ikPending) return;
  ikInFlight = true;
  while (ikPending) {
    const req = ikPending;
    ikPending = null;
    const res = await apiFetch("/api/sim/ik", {
      method: "POST",
      body: JSON.stringify(req),
    });
    if (res.ok && res.data) {
      twinState.leaderJoints = res.data.leader_joints;
      twinState.followerJoints = res.data.follower_joints;
      if (res.data.clamped) {
        const fkObj = req.arm === "leader" ? res.data.leader_fk : res.data.follower_fk;
        const tcp = fkObj?.tcp_pose;
        if (tcp) {
          if (isQuadMode()) {
            Object.values(quadViews).forEach((v) => v?.syncClampedDragAnchor(tcp));
          } else {
            const activeView = req.arm === "leader" ? leaderView : followerView;
            activeView?.syncClampedDragAnchor(tcp);
          }
        }
      }
      renderTwinViews();
    }
  }
  ikInFlight = false;
}

function renderTwinViews() {
  const l = twinState.leaderJoints;
  const f = twinState.followerJoints;
  const showHoming = twinState.activeWorkflow === "calibrate";

  if (isQuadMode()) {
    const role = getQuadArmRole();
    const joints = role === "leader" ? l : f;
    const isFollower = role === "follower";
    Object.values(quadViews).forEach((v) => {
      v?.setState({
        joints,
        waypoints: isFollower ? twinState.waypoints : [],
        vlaChunk: isFollower ? twinState.vlaChunk : [],
        overlayBadge: twinState.overlayBadge,
        recActive: isFollower && twinState.activeWorkflow === "collect",
        showHomingRef: showHoming,
      });
    });
    const tcp = computeStickFK(joints).tcp;
    if ($("quad-tcp-readout")) {
      $("quad-tcp-readout").textContent = `TCP ${fmtZero(tcp.x)}, ${fmtZero(tcp.y)}, ${fmtZero(tcp.z)} mm`;
    }
    if ($("quad-telemetry-strip")) {
      $("quad-telemetry-strip").innerHTML = renderStickTelemetryHtml(
        joints,
        isFollower ? "从臂 6-DOF" : "主臂 6-DOF"
      );
    }
    return;
  }

  if (leaderView) {
    leaderView.setState({
      joints: l,
      showHomingRef: showHoming,
      overlayBadge: twinState.overlayBadge,
    });
    const lTcp = computeStickFK(l).tcp;
    if ($("twin-leader-tcp")) {
      $("twin-leader-tcp").textContent = `TCP ${fmtZero(lTcp.x)}, ${fmtZero(lTcp.y)}, ${fmtZero(lTcp.z)} mm`;
    }
    if ($("twin-leader-strip")) {
      $("twin-leader-strip").innerHTML = renderStickTelemetryHtml(l, "主臂 6-DOF");
    }
  }

  if (followerView) {
    followerView.setState({
      joints: f,
      waypoints: twinState.waypoints,
      vlaChunk: twinState.vlaChunk,
      overlayBadge: twinState.overlayBadge,
      recActive: twinState.activeWorkflow === "collect",
      showHomingRef: showHoming,
    });
    const fTcp = computeStickFK(f).tcp;
    if ($("twin-follower-tcp")) {
      $("twin-follower-tcp").textContent = `TCP ${fmtZero(fTcp.x)}, ${fmtZero(fTcp.y)}, ${fmtZero(fTcp.z)} mm`;
    }
    if ($("twin-follower-strip")) {
      $("twin-follower-strip").innerHTML = renderStickTelemetryHtml(f, "从臂 6-DOF");
    }
  }
}

function updateTwinThemeUI(theme) {
  twinState.theme = applyThemeToDocument(theme);
  const btn = $("btn-twin-toggle-theme");
  if (btn) {
    btn.textContent = twinState.theme === "dark" ? "深色 · 切浅色" : "浅色 · 切深色";
  }
  leaderView?.render?.();
  followerView?.render?.();
  Object.values(quadViews).forEach((v) => v?.render?.());
}

function bootstrapTwin() {
  updateTwinThemeUI(twinState.theme);

  // 1. Acquire exclusive Web Lock & notify Main Studio window + Sync Theme
  const mutex = new StagePopupMutex({
    role: "popup",
    onThemeChange: (nextTheme) => {
      updateTwinThemeUI(nextTheme);
    },
  });

  $("btn-twin-toggle-theme")?.addEventListener("click", () => {
    const nextTheme = twinState.theme === "dark" ? "light" : "dark";
    mutex.broadcastTheme(nextTheme);
  });

  // 2. Initialize Dual-Arm 3D Canvases
  leaderView = new StickArmCanvas($("canvas-twin-leader"), {
    role: "leader",
    label: "Leader 主臂 (红圈IK · 联动)",
    enableIkDrag: true,
    onIkDrag: (req) => {
      ikPending = req;
      drainIkQueue();
    },
  });
  followerView = new StickArmCanvas($("canvas-twin-follower"), {
    role: "follower",
    label: "Follower 从臂",
    enableIkDrag: false,
    onIkDrag: (req) => {
      ikPending = req;
      drainIkQueue();
    },
  });
  syncTwinIkMutex();

  // 3. Layout Switcher & Camera View Controls
  document.querySelectorAll(".btn-twin-layout").forEach((btn) => {
    btn.addEventListener("click", () => {
      setTwinLayoutMode(btn.dataset.layout || "dual");
    });
  });

  document.querySelectorAll(".btn-cam-preset").forEach((btn) => {
    btn.addEventListener("click", () => {
      const view = btn.dataset.view;
      if (isQuadMode()) {
        if (view === "reset") {
          Object.entries(quadViews).forEach(([k, v]) => v?.setPresetView(k));
        } else {
          Object.values(quadViews).forEach((v) => v?.setPresetView(view));
        }
      } else {
        leaderView?.setPresetView(view);
        followerView?.setPresetView(view);
      }
    });
  });

  $("btn-twin-zoom-in")?.addEventListener("click", () => {
    if (isQuadMode()) {
      Object.values(quadViews).forEach((v) => v?.zoomBy(1.15));
    } else {
      leaderView?.zoomBy(1.15);
      followerView?.zoomBy(1.15);
    }
  });

  $("btn-twin-zoom-out")?.addEventListener("click", () => {
    if (isQuadMode()) {
      Object.values(quadViews).forEach((v) => v?.zoomBy(0.87));
    } else {
      leaderView?.zoomBy(0.87);
      followerView?.zoomBy(0.87);
    }
  });

  // 4. Quick Hardware Actions
  $("chk-twin-linked")?.addEventListener("change", async (ev) => {
    const res = await apiFetch("/api/sim/linked", {
      method: "POST",
      body: JSON.stringify({ linked: ev.target.checked }),
    });
    if (res.ok && res.data) {
      twinState.linkedArms = Boolean(res.data.linked_arms);
      syncTwinIkMutex();
    }
  });

  $("btn-twin-home")?.addEventListener("click", async () => {
    const home = Object.fromEntries(JOINTS.map((j) => [j.key, j.def]));
    await apiFetch("/api/sim/joints", {
      method: "POST",
      body: JSON.stringify({ arm: "both", joints: home }),
    });
  });

  $("btn-twin-rest")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teaching/pose", {
      method: "POST",
      body: JSON.stringify({ pose: "rest", duration_s: 1.2 }),
    });
  });

  $("btn-twin-estop")?.addEventListener("click", async () => {
    await apiFetch("/api/safety/estop", {
      method: "POST",
      body: JSON.stringify({ reason: "Sim Viewport E-Stop" }),
    });
  });

  $("btn-twin-reset-estop")?.addEventListener("click", async () => {
    await apiFetch("/api/safety/reset", { method: "POST" });
  });

  $("btn-recall-to-main")?.addEventListener("click", () => {
    mutex.recallPopupWindow();
    window.close();
  });

  // 5. Connect Realtime Telemetry Socket
  createStudioSocket({
    onOpen: () => {
      const pill = $("twin-ws-pill");
      if (pill) {
        pill.innerHTML = '<span class="status-dot dot-ok"></span>WS Connected';
        pill.className = "sb-item";
      }
    },
    onClose: () => {
      const pill = $("twin-ws-pill");
      if (pill) {
        pill.innerHTML = '<span class="status-dot dot-warn"></span>Reconnecting...';
        pill.className = "sb-item sb-item-warn";
      }
    },
    onOverlay: (ov) => {
      if (Array.isArray(ov.waypoints)) twinState.waypoints = ov.waypoints;
      if (Array.isArray(ov.vla_chunk)) twinState.vlaChunk = ov.vla_chunk;
      if (ov.overlay_badge !== undefined) twinState.overlayBadge = ov.overlay_badge || "";
      renderTwinViews();
    },
    onTelemetry: (msg) => {
      twinState.activeWorkflow = msg.active_workflow || "idle";
      if (msg.overlay_badge !== undefined) twinState.overlayBadge = msg.overlay_badge || "";
      if ($("twin-mode-pill") && msg.mode) {
        $("twin-mode-pill").textContent = `MODE: ${String(msg.mode).toUpperCase().replace("_", "-")}`;
      }
      if ($("twin-wf-pill")) {
        $("twin-wf-pill").textContent = `WORKFLOW: ${String(twinState.activeWorkflow).toUpperCase()}`;
      }
      if ($("chk-twin-linked") && typeof msg.linked_arms === "boolean") {
        $("chk-twin-linked").checked = msg.linked_arms;
        if (twinState.linkedArms !== msg.linked_arms) {
          twinState.linkedArms = msg.linked_arms;
          syncTwinIkMutex();
        }
      }
      if (msg.safety) {
        twinState.estopLatched = Boolean(msg.safety.estop_latched);
        const sPill = $("twin-safety-pill");
        const btnReset = $("btn-twin-reset-estop");
        if (sPill) {
          sPill.innerHTML = twinState.estopLatched
            ? '<span class="status-dot dot-danger"></span>SAFETY: ESTOP'
            : '<span class="status-dot dot-ok"></span>SAFETY: NORMAL';
          sPill.className = twinState.estopLatched ? "sb-item sb-item-alert" : "sb-item";
        }
        if (btnReset) btnReset.hidden = !twinState.estopLatched;
      }
      if (!isAnyViewDraggingIk()) {
        if (msg.leader_joints) twinState.leaderJoints = msg.leader_joints;
        if (msg.follower_joints) twinState.followerJoints = msg.follower_joints;
        renderTwinViews();
      }
    },
  });
}

document.addEventListener("DOMContentLoaded", bootstrapTwin);
