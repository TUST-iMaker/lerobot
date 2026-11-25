import {
  StickArmCanvas,
  renderStickTelemetryHtml,
  computeStickFK,
  fmtZero,
} from "/static/stick_sim.js?v=2026112304";
import {
  JOINTS,
  formatJointReadout,
  apiFetch as busApiFetch,
  createStudioSocket,
  StagePopupMutex,
  getStoredTheme,
  applyThemeToDocument,
} from "/static/studio_bus.js?v=2026112304";

const state = {
  theme: getStoredTheme(),
  mode: "dry_run",
  activeTab: "dashboard",
  activeWorkflow: "idle",
  lastCalStage: "idle",
  linkedArms: true,
  torqueEnabled: true,
  estopLatched: false,
  stageViewMode: "dual", // "dual" | "follower" | "leader"
  stageCollapsed: false,
  userStageCollapsed: null, // null = auto (1..4 open, 5..7 collapsed); boolean = user override
  popupActive: false,
  activeSliderId: null,
  sliderCooldownTs: {},
  filledCollectConfig: null,
  leaderJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  followerJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  waypoints: [],
  vlaChunk: [],
  overlayBadge: "",
  vlaAnimTimer: null,
  epReplayTimer: null,
  pluginReplayTimer: null,
  pluginsCache: [],
  localLogs: [],
  pollTimer: null,
  camTimer: null,
  toastTimer: null,
  jointSendTimer: null,
  pendingJoints: {},
  simSendTimers: { leader: null, follower: null },
  simPending: { leader: {}, follower: {} },
};

const stageViews = {
  leader: null,
  follower: null,
};

let popupMutex = null;

const $ = (id) => document.getElementById(id);

function appendLocalLog(msg) {
  const ts = new Date().toTimeString().slice(0, 8);
  state.localLogs.push(`[${ts}] [UI] ${msg}`);
  if (state.localLogs.length > 60) state.localLogs.shift();
  renderLogs([]);
}

function renderLogs(backendLogs) {
  const el = $("log-console");
  if (!el) return;
  const combined = [...(backendLogs || []), ...state.localLogs].slice(-120);
  el.textContent = combined.join("\n");
  el.scrollTop = el.scrollHeight;
}

async function apiFetch(url, options = {}) {
  return busApiFetch(url, options, appendLocalLog);
}

let _lastBoundaryToastTs = 0;

function showAnomalyToast(message, { sticky = false, level = "danger", durationMs = 3600 } = {}) {
  const strip = $("anomaly-toast-strip");
  const textEl = $("anomaly-toast-text");
  const dotEl = $("anomaly-toast-dot");
  if (!strip || !textEl) return;
  textEl.textContent = message;
  strip.className = `anomaly-toast toast-${level}`;
  if (dotEl) {
    dotEl.className = `status-dot dot-${level === "ok" ? "ok" : level === "warn" ? "warn" : level === "info" ? "info" : "danger"}`;
  }
  strip.hidden = false;
  if (state.toastTimer) {
    clearTimeout(state.toastTimer);
    state.toastTimer = null;
  }
  if (!sticky && !state.estopLatched) {
    state.toastTimer = setTimeout(() => {
      if (!state.estopLatched) strip.hidden = true;
    }, durationMs);
  }
}

function showFeedbackToast(message, level = "info") {
  if (state.estopLatched) return;
  showAnomalyToast(message, { sticky: false, level, durationMs: 2400 });
}

function showBoundaryToast(message) {
  const now = performance.now();
  if (now - _lastBoundaryToastTs < 1500) return;
  _lastBoundaryToastTs = now;
  showFeedbackToast(message || "已触达 SO-101 物理工作空间边界", "warn");
}

function clearAnomalyToast() {
  const strip = $("anomaly-toast-strip");
  if (strip && !state.estopLatched) {
    strip.hidden = true;
  }
}

/* ==========================================================================
   Global Single Persistent 3D Stage Dock, 4 Structural Resizers & Control Mutex
   ========================================================================== */
function refreshStageSizes() {
  requestAnimationFrame(() => {
    stageViews.leader?.refreshSize?.();
    stageViews.follower?.refreshSize?.();
  });
}

function setStageViewMode(mode) {
  state.stageViewMode = mode;
  ["dual", "follower", "leader"].forEach((m) => {
    const btn = $(`btn-stage-mode-${m}`);
    if (btn) btn.classList.toggle("active", m === mode);
  });
  const slotL = $("stage-slot-leader");
  const slotF = $("stage-slot-follower");
  const armResizer = $("stage-arms-resizer");
  if (slotL) {
    slotL.hidden = mode === "follower";
    if (mode !== "dual") slotL.style.flex = "1";
  }
  if (slotF) {
    slotF.hidden = mode === "leader";
    if (mode !== "dual") slotF.style.flex = "1";
  }
  if (armResizer) armResizer.hidden = mode !== "dual";
  refreshStageSizes();
}

function syncArmControlMutexUI() {
  const linked = Boolean(state.linkedArms);
  const inCal = state.activeWorkflow === "calibrate";
  const ikAllowed = Boolean($("chk-sim-ik-drag")?.checked ?? true) && !inCal;

  const chk = $("chk-sim-linked");
  const pill = $("sim-linked-pill");
  const txt = $("sim-linked-text");
  const teleopLinkBtn = $("btn-teleop-link-toggle");

  if (chk) chk.checked = linked;
  if (pill) pill.classList.toggle("active", linked);
  if (txt) txt.textContent = linked ? "Link Leader-Follower: 开" : "Link Leader-Follower: 关";
  if (teleopLinkBtn) {
    teleopLinkBtn.classList.toggle("active", linked);
    teleopLinkBtn.textContent = linked ? "主从联动: 开" : "主从联动: 关";
  }

  // Mutual exclusion: when Link Leader-Follower is ON, lock & grey out Follower manual controls + Follower IK ring
  const fCard = $("sim-follower-card");
  const fBadge = $("sim-follower-lock-badge");
  const fHomeBtn = $("btn-sim-follower-home");
  if (fCard) fCard.classList.toggle("card-locked", linked);
  if (fBadge) fBadge.hidden = !linked;
  if (fHomeBtn) fHomeBtn.disabled = linked;
  document.querySelectorAll("#sim-follower-sliders .joint-slider").forEach((sl) => {
    sl.disabled = linked;
  });

  const ikLabel = $("ik-target-arm-label");
  if (ikLabel) {
    ikLabel.textContent = linked
      ? "主臂末端 IK 坐标 (联动从臂, mm)"
      : "从臂末端 IK 坐标 (独立控制, mm)";
  }

  const authBadge = $("control-authority-badge");
  if (authBadge) {
    if (linked) {
      authBadge.textContent = state.mode === "real" ? "真实主臂操控从臂" : "主臂联动操控从臂";
      authBadge.className = "status-tag tag-ok";
    } else {
      authBadge.textContent = "电脑操控从臂";
      authBadge.className = "status-tag tag-warn";
    }
  }

  // Calibration is strictly joint-space (no red-ring IK on either arm)
  if (inCal) {
    stageViews.leader?.setIkEnabled(false);
    stageViews.follower?.setIkEnabled(false);
    stageViews.leader?.setState({ label: "Leader 主臂 (标定中)" });
    stageViews.follower?.setState({ label: "Follower 从臂 (标定中)" });
  } else if (linked) {
    stageViews.leader?.setIkEnabled(ikAllowed);
    stageViews.follower?.setIkEnabled(false);
    stageViews.leader?.setState({ label: ikAllowed ? "Leader 主臂 (红圈IK · 联动)" : "Leader 主臂 (联动)" });
    stageViews.follower?.setState({ label: "Follower 从臂 (跟随主臂)" });
  } else {
    stageViews.leader?.setIkEnabled(ikAllowed);
    stageViews.follower?.setIkEnabled(ikAllowed);
    stageViews.leader?.setState({ label: ikAllowed ? "Leader 主臂 (红圈IK · 真机可驱动)" : "Leader 主臂 (真机可驱动)" });
    stageViews.follower?.setState({ label: ikAllowed ? "Follower 从臂 (红圈IK · 独立)" : "Follower 从臂 (独立)" });
  }
  const ikArmSel = $("sim-ik-arm-select");
  if (ikArmSel) {
    ikArmSel.disabled = linked;
    if (linked) ikArmSel.value = "leader";
  }
}

function updateTeleopButtonsUI() {
  const running = state.activeWorkflow === "teleop";
  const btnStart = $("btn-start-teleop");
  const btnStop = $("btn-stop-teleop");
  const camCard = $("teleop-cam-card");

  if (btnStart) {
    btnStart.disabled = running;
    btnStart.classList.toggle("btn-teleop-live", running);
    btnStart.textContent = running ? "遥操作运行中" : "启动遥操作";
  }
  if (btnStop) {
    btnStop.disabled = !running;
    btnStop.className = running ? "btn btn-xs btn-estop" : "btn btn-xs btn-secondary";
  }
  const btnTopTeleop = $("btn-topbar-teleop");
  if (btnTopTeleop) {
    btnTopTeleop.classList.toggle("btn-teleop-live", running);
    btnTopTeleop.textContent = running ? "遥操作: 开" : "遥操作: 关";
  }
  if (camCard) {
    camCard.classList.toggle("teleop-running-card", running);
  }
  const collecting = state.activeWorkflow === "collect";
  const btnColStart = $("btn-start-collect");
  const btnColStop = $("btn-stop-collect");
  if (btnColStart) {
    btnColStart.disabled = collecting;
    btnColStart.textContent = collecting ? "采集中..." : "开始采集";
  }
  if (btnColStop) {
    btnColStop.disabled = !collecting;
  }
}

function initStructuralResizers() {
  // Resizer 1: Global 3D Stage Dock Width (330px .. 520px)
  const stageHandle = $("stage-resize-handle");
  const dock = $("global-stage-dock");
  if (stageHandle && dock) {
    let dragging = false;
    stageHandle.addEventListener("pointerdown", (ev) => {
      dragging = true;
      stageHandle.setPointerCapture(ev.pointerId);
    });
    stageHandle.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const dockRect = dock.getBoundingClientRect();
      const nextW = Math.max(330, Math.min(520, Math.round(ev.clientX - dockRect.left)));
      document.documentElement.style.setProperty("--stage-dock-w", `${nextW}px`);
      refreshStageSizes();
    });
    const end = () => { dragging = false; };
    stageHandle.addEventListener("pointerup", end);
    stageHandle.addEventListener("pointercancel", end);
  }

  // Resizer 2: Inside 3D Stage — Leader / Follower Vertical Split
  const armResizer = $("stage-arms-resizer");
  const stack = $("stage-stack");
  const slotL = $("stage-slot-leader");
  const slotF = $("stage-slot-follower");
  if (armResizer && stack && slotL && slotF) {
    let dragging = false;
    armResizer.addEventListener("pointerdown", (ev) => {
      dragging = true;
      armResizer.classList.add("dragging");
      armResizer.setPointerCapture(ev.pointerId);
    });
    armResizer.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const rect = stack.getBoundingClientRect();
      const usableH = Math.max(280, rect.height - 16);
      const topH = Math.max(135, Math.min(usableH - 135, ev.clientY - rect.top - 8));
      const ratio = topH / usableH;
      slotL.style.flex = `${ratio.toFixed(3)} 1 0px`;
      slotF.style.flex = `${(1 - ratio).toFixed(3)} 1 0px`;
      refreshStageSizes();
    });
    const end = () => {
      dragging = false;
      armResizer.classList.remove("dragging");
    };
    armResizer.addEventListener("pointerup", end);
    armResizer.addEventListener("pointercancel", end);
  }

  // Resizer 3: Tab 2 Teleop — Dual Cameras / Bottom Controls Vertical Split
  const camResizer = $("teleop-cam-resizer");
  if (camResizer) {
    let dragging = false;
    let startY = 0;
    let startH = 168;
    camResizer.addEventListener("pointerdown", (ev) => {
      const firstFrame = document.querySelector("#teleop-cam-card .cam-frame");
      if (!firstFrame) return;
      dragging = true;
      startY = ev.clientY;
      startH = firstFrame.getBoundingClientRect().height || 168;
      camResizer.classList.add("dragging");
      camResizer.setPointerCapture(ev.pointerId);
    });
    camResizer.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const nextH = Math.max(110, Math.min(420, Math.round(startH + (ev.clientY - startY))));
      document.querySelectorAll("#teleop-cam-card .cam-frame").forEach((el) => {
        el.style.height = `${nextH}px`;
      });
    });
    const end = () => {
      dragging = false;
      camResizer.classList.remove("dragging");
    };
    camResizer.addEventListener("pointerup", end);
    camResizer.addEventListener("pointercancel", end);
  }

  // Resizer 4: Bottom Console Drawer Top-Edge Height Resizer
  const consoleHandle = $("console-resize-handle");
  const logDock = $("log-dock");
  if (consoleHandle && logDock) {
    let dragging = false;
    let startY = 0;
    let startH = 175;
    consoleHandle.addEventListener("pointerdown", (ev) => {
      dragging = true;
      startY = ev.clientY;
      startH = logDock.getBoundingClientRect().height || 175;
      consoleHandle.classList.add("dragging");
      consoleHandle.setPointerCapture(ev.pointerId);
    });
    consoleHandle.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const nextH = Math.max(110, Math.min(460, Math.round(startH + (startY - ev.clientY))));
      logDock.style.height = `${nextH}px`;
    });
    const end = () => {
      dragging = false;
      consoleHandle.classList.remove("dragging");
    };
    consoleHandle.addEventListener("pointerup", end);
    consoleHandle.addEventListener("pointercancel", end);
  }

  // Resizer 5: Tab 4 Continuous Trajectory Card vs Waypoints Queue Vertical Split
  const teachResizer = $("teaching-split-resizer");
  const linearCard = $("linear-teach-card");
  const wpCard = $("waypoints-card");
  if (teachResizer && linearCard && wpCard) {
    let dragging = false;
    let startY = 0;
    let startTopH = 260;
    let totalH = 520;
    teachResizer.addEventListener("pointerdown", (ev) => {
      dragging = true;
      startY = ev.clientY;
      const topRect = linearCard.getBoundingClientRect();
      const botRect = wpCard.getBoundingClientRect();
      startTopH = topRect.height;
      totalH = Math.max(320, topRect.height + botRect.height);
      teachResizer.classList.add("dragging");
      teachResizer.setPointerCapture(ev.pointerId);
    });
    teachResizer.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const nextTop = Math.max(150, Math.min(totalH - 140, startTopH + (ev.clientY - startY)));
      const ratio = nextTop / totalH;
      linearCard.style.flex = `${ratio.toFixed(3)} 1 0px`;
      wpCard.style.flex = `${(1 - ratio).toFixed(3)} 1 0px`;
    });
    const end = () => {
      dragging = false;
      teachResizer.classList.remove("dragging");
    };
    teachResizer.addEventListener("pointerup", end);
    teachResizer.addEventListener("pointercancel", end);
  }
}

function applyStageVisibility(collapsed) {
  state.stageCollapsed = Boolean(collapsed);
  const shell = $("studio-shell");
  const dock = $("global-stage-dock");
  const popBar = $("stage-popout-bar");
  const toggleBtn = $("btn-toggle-stage-dock");

  if (shell) {
    shell.classList.toggle("stage-popped-out", Boolean(state.popupActive));
    shell.classList.toggle("stage-collapsed", Boolean(state.stageCollapsed || state.popupActive));
  }

  if (state.popupActive) {
    if (dock) dock.classList.add("collapsed");
    if (popBar) popBar.hidden = false;
    stageViews.leader?.setPaused(true);
    stageViews.follower?.setPaused(true);
    if (toggleBtn) toggleBtn.textContent = "已在独立窗运行";
    return;
  }

  if (popBar) popBar.hidden = true;
  if (dock) dock.classList.toggle("collapsed", state.stageCollapsed);
  stageViews.leader?.setPaused(state.stageCollapsed);
  stageViews.follower?.setPaused(state.stageCollapsed);
  if (toggleBtn) {
    toggleBtn.textContent = state.stageCollapsed ? "展开 3D 舞台" : "收起 3D 舞台";
  }
  if (!state.stageCollapsed) {
    refreshStageSizes();
  }
}

function ensureStageVisibleForAction() {
  if (!state.popupActive && state.stageCollapsed) {
    applyStageVisibility(false);
  }
}

function initGlobalStageDock() {
  let ikInFlight = false;
  let ikPendingReq = null;
  const drainIkQueue = async () => {
    if (ikInFlight || !ikPendingReq) return;
    ikInFlight = true;
    while (ikPendingReq) {
      const req = ikPendingReq;
      ikPendingReq = null;
      const res = await apiFetch("/api/sim/ik", {
        method: "POST",
        body: JSON.stringify(req),
      });
      if (res.ok && res.data) {
        updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
        const fkObj = req.arm === "leader" ? res.data.leader_fk : res.data.follower_fk;
        const tcp = fkObj?.tcp_pose;
        if (tcp) {
          if ($("ik-x")) $("ik-x").value = String(Math.round(tcp.x_mm));
          if ($("ik-y")) $("ik-y").value = String(Math.round(tcp.y_mm));
          if ($("ik-z")) $("ik-z").value = String(Math.round(tcp.z_mm));
        }
        if (res.data.clamped && tcp) {
          const activeView = req.arm === "leader" ? stageViews.leader : stageViews.follower;
          activeView?.syncClampedDragAnchor(tcp);
          showBoundaryToast(res.data.boundary_message);
        }
        if (isTeleopPanelVisible()) refreshCameraImagesOnce();
      }
    }
    ikInFlight = false;
  };

  stageViews.leader = new StickArmCanvas($("canvas-sim-leader"), {
    role: "leader",
    label: "Leader 主臂 (红圈IK · 联动)",
    enableIkDrag: true,
    onIkDrag: (req) => {
      ikPendingReq = req;
      drainIkQueue();
    },
  });

  stageViews.follower = new StickArmCanvas($("canvas-sim-follower"), {
    role: "follower",
    label: "Follower 从臂 (跟随主臂)",
    enableIkDrag: false,
    onIkDrag: (req) => {
      ikPendingReq = req;
      drainIkQueue();
    },
  });

  // View mode buttons (Dual / Follower / Leader)
  document.querySelectorAll("[data-stage-mode]").forEach((btn) => {
    btn.addEventListener("click", () => setStageViewMode(btn.dataset.stageMode));
  });

  // Camera Preset & Zoom Controls
  document.querySelectorAll("[data-cam-view]").forEach((btn) => {
    btn.addEventListener("click", () => {
      const view = btn.dataset.camView;
      stageViews.leader?.setPresetView(view);
      stageViews.follower?.setPresetView(view);
    });
  });

  $("btn-stage-zoom-in")?.addEventListener("click", () => {
    stageViews.leader?.zoomBy(1.15);
    stageViews.follower?.zoomBy(1.15);
  });

  $("btn-stage-zoom-out")?.addEventListener("click", () => {
    stageViews.leader?.zoomBy(0.87);
    stageViews.follower?.zoomBy(0.87);
  });

  initStructuralResizers();

  // Initialize Web Locks + BroadcastChannel Pop-out Mutex & Theme Sync
  popupMutex = new StagePopupMutex({
    role: "host",
    onStateChange: (active) => {
      state.popupActive = active;
      applyStageVisibility(state.stageCollapsed);
      updateAllStickViews();
    },
    onThemeChange: (nextTheme) => {
      updateThemeUI(nextTheme);
    },
  });
}

function updateThemeUI(theme) {
  state.theme = applyThemeToDocument(theme);
  const labelEl = $("theme-switch-label");
  if (labelEl) {
    labelEl.textContent = state.theme === "dark" ? "深色模式 · 切浅色" : "浅色模式 · 切深色";
  }
  stageViews.leader?.render?.();
  stageViews.follower?.render?.();
  refreshCameraImagesOnce();
}

function updateAllStickViews() {
  const l = state.leaderJoints;
  const f = state.followerJoints;
  const calTarget = $("calibrate-target")?.value || "follower";
  const calJoints = calTarget === "leader" ? l : f;
  const isCalTab = state.activeTab === "dashboard";
  const isReplayRunning = Boolean(state.epReplayTimer || state.pluginReplayTimer || state.vlaAnimTimer);

  if (stageViews.leader && !state.popupActive && !state.stageCollapsed) {
    stageViews.leader.setState({
      joints: l,
      showHomingRef: isCalTab && calTarget === "leader",
      overlayBadge: state.overlayBadge,
    });
  }

  if (stageViews.follower && !state.popupActive && !state.stageCollapsed && !isReplayRunning) {
    stageViews.follower.setState({
      joints: f,
      waypoints: state.waypoints,
      vlaChunk: state.vlaChunk,
      overlayBadge: state.overlayBadge,
      recActive: state.activeWorkflow === "collect",
      showHomingRef: isCalTab && calTarget === "follower",
    });
  }

  const tcp = computeStickFK(f).tcp;
  if ($("teleop-tcp-badge")) {
    $("teleop-tcp-badge").textContent = `TCP: X=${fmtZero(tcp.x)} Y=${fmtZero(tcp.y)} Z=${fmtZero(tcp.z)} mm`;
  }

  if ($("cal-stick-data")) {
    $("cal-stick-data").innerHTML = renderStickTelemetryHtml(
      isCalTab ? calJoints : f,
      isCalTab ? `标定臂 (${calTarget})` : "从臂 6-DOF"
    );
  }
  if ($("teach-stick-data") && state.activeTab === "teaching") {
    $("teach-stick-data").innerHTML = renderStickTelemetryHtml(f, "示教臂实时姿态");
  }
  if ($("collect-stick-data") && state.activeTab === "collect" && !state.epReplayTimer) {
    $("collect-stick-data").innerHTML = renderStickTelemetryHtml(f, "采集帧 (observation.state)");
  }
  if ($("plugin-stick-data") && state.activeTab === "plugins" && !state.pluginReplayTimer) {
    $("plugin-stick-data").innerHTML = renderStickTelemetryHtml(f, "工作台 6-DOF 姿态");
  }
  if ($("vla-stick-data") && state.activeTab === "vla" && !state.vlaAnimTimer) {
    $("vla-stick-data").innerHTML = renderStickTelemetryHtml(f, "VLA 预测姿态");
  }
}

/* ==========================================================================
   Mode, Safety & Hardware Probe Rendering
   ========================================================================== */
function setModeUI(mode) {
  state.mode = mode;
  const isReal = mode === "real";
  const badge = $("mode-badge");
  if (badge) {
    badge.textContent = isReal ? "MODE: REAL (真机)" : "MODE: DRY-RUN (仿真)";
    badge.className = "sb-item";
  }
  const topbar = document.querySelector(".topbar");
  if (topbar) {
    topbar.classList.toggle("mode-real", isReal);
    topbar.classList.toggle("mode-dry", !isReal);
  }
  const cap = $("mode-capsule");
  if (cap) {
    cap.classList.toggle("mode-real", isReal);
    cap.classList.toggle("mode-dry", !isReal);
  }
  const capDot = $("mode-capsule-dot");
  if (capDot) {
    capDot.className = isReal ? "status-dot dot-ok" : "status-dot dot-warn";
  }
  const liveTag = $("mode-live-tag");
  if (liveTag) {
    liveTag.textContent = isReal ? "真机 REAL" : "仿真 SIM";
  }
  renderReadinessState();
}

function renderReadinessState() {
  const pill = $("readiness-pill");
  const pillDot = $("readiness-pill-dot");
  const pillText = $("readiness-pill-text");
  const card = $("readiness-card");
  const cardDot = $("readiness-card-dot");
  const cardTitle = $("readiness-card-title");
  const cardDesc = $("readiness-card-desc");
  const btn = $("btn-readiness-action");

  const isReal = state.mode === "real";
  const wf = state.activeWorkflow || "idle";
  const dir = state.syncDirection || "off";

  let level = "ready"; // ready | info | warn | danger
  let pillLabel = "仿真就绪";
  let title = "系统已准备就绪 · 仿真可直接操控";
  let desc = "当前为 dry_run 仿真模式：拖动左侧 3D 红圈、右侧滑块或 WASD/QE 均会立即生效，不会驱动真机。";
  let actionLabel = "前往 3. 仿真控臂";
  let actionType = "tab_sim";

  if (state.estopLatched) {
    level = "danger";
    pillLabel = "急停锁定";
    title = "急停已锁定 · 当前不可操控";
    desc = "所有真机与仿真运动均已切断。请确认桌面安全后点击「解除急停」恢复。";
    actionLabel = "解除急停";
    actionType = "reset_estop";
  } else if (wf === "calibrate") {
    level = "warn";
    pillLabel = "标定进行中";
    title = "正在执行零点标定 · 红圈 IK 已临时禁用";
    desc = isReal
      ? "真机舵机已释放扭矩，请按下方提示摆正大臂竖直、小臂水平向前（中位）并逐步完成标定。"
      : "模拟标定期间请使用标定卡片内的 6 轴滑块或「一键对齐中位」，完成后再切回正常控臂。";
    actionLabel = "查看标定步骤";
    actionType = "tab_dashboard";
  } else if (isReal) {
    if (dir === "teleop_bidirectional") {
      level = "ready";
      pillLabel = state.linkedArms ? "真实主臂操控从臂" : "电脑操控从臂";
      title = state.linkedArms
        ? "真实主臂操控从臂 · 遥操作双向同步中"
        : "电脑操控从臂 · 主从已解绑（防主臂干扰）";
      desc = state.linkedArms
        ? "当前主从联动开启：用手移动物理主臂会实时带动从臂；若用滑块/键控微调从臂，松手后将保持在新位置，直到再次掰动主臂。"
        : "当前主从联动关闭：由电脑滑块、红圈 IK 或 WASD/QE 单独控制真机从臂，碰主臂不会干扰从臂。";
      actionLabel = "前往 2. 遥操作";
      actionType = "tab_teleop";
    } else if (dir === "sim_to_real") {
      level = "ready";
      pillLabel = "电脑操控从臂";
      title = "电脑操控从臂 · 界面操作将实时带动真机";
      desc = "拖动 3D 红圈、滑块、WASD/QE 或执行示教回放都会实时驱动真机从臂，并回读编码器更新画面。";
      actionLabel = "前往 3. 仿真控臂";
      actionType = "tab_sim";
    } else if (dir === "real_to_sim") {
      level = "info";
      pillLabel = "真机→仿真监听中";
      title = "自由示教 / 只读同步中 · 掰动真机→3D 跟随";
      desc = "从臂扭矩已释放，用手掰动真机会实时带动 3D 仿真，可直接记录示教点；点「点击上力」或回放即可切回驱动。";
      actionLabel = "前往 4. 示教记录";
      actionType = "tab_teaching";
    } else {
      level = "warn";
      pillLabel = "真机待命（未同步）";
      title = "真机已连接但未开启同步 · 掰动真机暂不会带动 3D";
      desc = "当前处于待命：点击右侧按钮可立即开启「真机→仿真」实时读回（掰真机即动 3D），或进入「2. 遥操作」开启双向联动。";
      actionLabel = "开启真机→仿真实时同步";
      actionType = "enable_real_to_sim";
    }
  } else if (wf === "teleop") {
    level = "ready";
    pillLabel = "仿真遥操作中";
    title = "仿真遥操作运行中 · 主从臂已就绪";
    desc = "拖动主臂红圈或滑块会同步带动从臂；如需单独调节从臂，取消勾选「主从联动」即可。";
    actionLabel = "前往 2. 遥操作";
    actionType = "tab_teleop";
  } else if (state.linkedArms) {
    desc = "当前「主从联动」已开启：请拖动主臂（Leader）红圈或滑块控臂，从臂会自动跟随；取消联动可单独控从臂。";
  }

  const dotClass =
    level === "danger"
      ? "status-dot dot-danger"
      : level === "warn"
      ? "status-dot dot-warn"
      : level === "info"
      ? "status-dot dot-ok"
      : "status-dot dot-ok";

  if (pill) pill.className = `readiness-pill pill-${level}`;
  if (pillDot) pillDot.className = dotClass;
  if (pillText) pillText.textContent = pillLabel;
  if (card) card.className = `card readiness-card state-${level}`;
  if (cardDot) cardDot.className = dotClass;
  if (cardTitle) cardTitle.textContent = title;
  if (cardDesc) cardDesc.textContent = desc;
  if (btn) {
    btn.textContent = actionLabel;
    btn.dataset.actionType = actionType;
  }
}

function updateSafetyUI(safety) {
  if (!safety) return;
  state.torqueEnabled = Boolean(safety.torque_enabled);
  state.estopLatched = Boolean(safety.estop_latched);
  if (safety.sync_direction !== undefined) {
    state.syncDirection = safety.sync_direction || "off";
    const syncSt = $("sim-sync-status");
    if (syncSt) syncSt.textContent = `同步: ${state.syncDirection}`;
  }

  const tBadge = $("torque-badge");
  if (tBadge) {
    tBadge.innerHTML = state.torqueEnabled
      ? '<span class="status-dot dot-ok"></span>扭矩: 已使能'
      : '<span class="status-dot dot-warn"></span>扭矩: 已释放';
  }
  const btnTorque = $("btn-toggle-torque");
  if (btnTorque) {
    btnTorque.textContent = state.torqueEnabled ? "点击卸力" : "点击上力";
  }

  const eBadge = $("estop-badge");
  const btnReset = $("btn-global-reset");
  if (eBadge) {
    if (state.estopLatched) {
      eBadge.innerHTML = '<span class="status-dot dot-danger"></span>SAFETY: ESTOP LATCHED';
      eBadge.className = "sb-item sb-item-alert";
      showAnomalyToast("急停 (E-STOP) 已触发锁定，所有关节驱动已切断！点击顶栏「解除急停」恢复。", { sticky: true });
    } else {
      eBadge.innerHTML = '<span class="status-dot dot-ok"></span>SAFETY: NORMAL';
      eBadge.className = "sb-item";
      clearAnomalyToast();
    }
  }
  if (btnReset) {
    btnReset.hidden = !state.estopLatched;
  }
  renderReadinessState();
}

function updateConnectionBadge(conn) {
  const badge = $("teleop-connection-badge");
  if (!badge) return;
  if (state.mode === "real" && conn && conn.hardware_ready) {
    badge.textContent = `真机已连接 (${conn.follower_port} <- ${conn.leader_port})`;
    badge.className = "status-tag tag-ok";
  } else {
    badge.textContent = "模拟演练模式（主从联动就绪）";
    badge.className = "status-tag tag-warn";
  }
}

function renderHardwareProbe(probe) {
  if (!probe) return;
  const isLinux = String(probe.os || "").toLowerCase().includes("linux");
  const hwTag = $("nav-os-hw-tag");
  const trainTag = $("nav-os-train-tag");
  if (hwTag) hwTag.textContent = isLinux ? "LINUX" : "WIN";
  if (trainTag) trainTag.textContent = isLinux ? "本机 GPU" : "UBUNTU";
  setModeUI(probe.active_mode || "dry_run");
  const modeSelect = $("mode-select");
  if (modeSelect && probe.requested_mode) {
    modeSelect.value = probe.requested_mode;
  }

  const title = $("hw-banner-title");
  const meta = $("hw-banner-meta");
  const chips = $("hw-missing-chips");
  const guidance = $("hw-guidance-list");
  const sbHw = $("hardware-status-banner");

  const candPorts = (probe.serial_ports || []).filter((p) => p.is_robot_candidate);
  const cams = probe.cameras || [];
  if (sbHw) {
    sbHw.textContent = `HW: ${candPorts.length} Ports / ${cams.length} Cam`;
  }
  if (title) {
    title.textContent = probe.dual_arm_ready
      ? "SO-101 双臂硬件已就绪（支持真实控制与模拟演练）"
      : probe.hardware_ready
        ? "SO-101 单臂硬件已就绪（真机模式可用，双臂遥操作需接入第 2 条串口）"
        : "未检测到 SO-101 硬件 — 已自动降级至 DRY-RUN 模拟模式";
  }
  if (meta) {
    const cap = probe.os_capabilities || {};
    meta.textContent = `OS: ${cap.os || "-"} | Python ${cap.python_version || "-"} | 串口候选: ${candPorts.length} | 相机: ${cams.length}`;
  }
  if (chips) {
    const items = (probe.missing_hardware || []).map(
      (m) => `<span class="chip chip-warn">缺少: ${m}</span>`
    );
    items.push(
      probe.lerobot_available
        ? '<span class="chip chip-ok">lerobot: OK</span>'
        : '<span class="chip chip-warn">lerobot: missing</span>'
    );
    chips.innerHTML = items.join("");
  }
  if (guidance) {
    guidance.innerHTML = (probe.guidance || []).map((g) => `<li>${g}</li>`).join("");
  }

  const portsBox = $("serial-ports-list");
  if (portsBox) {
    const ports = probe.serial_ports || [];
    portsBox.innerHTML = ports.length
      ? ports
          .map(
            (p) =>
              `<div class="kv-item">${p.device} — ${p.tag} (${p.is_robot_candidate ? "机械臂候选" : "已忽略"})</div>`
          )
          .join("")
      : '<div class="kv-item">未检测到串口设备</div>';
  }

  const camsBox = $("cameras-list");
  if (camsBox) {
    camsBox.innerHTML = cams.length
      ? cams
          .map((c) => `<div class="kv-item">[${c.id}] ${c.name} (${c.width}x${c.height} ${c.backend})</div>`)
          .join("")
      : '<div class="kv-item">未检测到物理相机（已启用合成 2D 视图）</div>';
  }

  const capsBox = $("capabilities-list");
  if (capsBox) {
    const cap = probe.os_capabilities || {};
    const pkgs = cap.packages || {};
    capsBox.innerHTML = [
      `<div class="kv-item">串口后端: ${cap.serial_backend || "-"} | 相机后端: ${cap.camera_backend || "-"}</div>`,
      `<div class="kv-item">OpenPI 本地运行: ${cap.can_run_openpi_local ? "支持 (Linux)" : "需 Ubuntu/云端"}</div>`,
      `<div class="kv-item">包状态: lerobot=${Boolean(pkgs.lerobot)}, datasets=${Boolean(pkgs.datasets)}, rerun=${Boolean(pkgs.rerun)}</div>`,
    ].join("");
  }

  renderDeviceSelectors(probe);
  renderCalibrationFileBadges(probe.calibration_status);
  updateConnectionBadge({
    hardware_ready: probe.hardware_ready,
    follower_port: probe.device_selection?.follower_port,
    leader_port: probe.device_selection?.leader_port,
  });
}

function populatePortSelect(selectEl, ports, currentValue) {
  if (!selectEl) return;
  const options = [];
  const seen = new Set();
  for (const p of ports || []) {
    if (!p || !p.device || p.is_bluetooth) continue;
    seen.add(p.device);
    const label = `${p.device} — ${p.tag}${p.is_robot_candidate ? "" : " (非USB候选)"}`;
    options.push(`<option value="${p.device}">${label}</option>`);
  }
  if (currentValue && !seen.has(currentValue)) {
    options.unshift(`<option value="${currentValue}">${currentValue} (当前配置 / 未连接)</option>`);
  }
  if (!options.length) {
    options.push('<option value="">未检测到可用串口</option>');
  }
  selectEl.innerHTML = options.join("");
  if (currentValue) selectEl.value = currentValue;
}

function populateCameraSelect(selectEl, cameras, currentValue) {
  if (!selectEl) return;
  const options = ['<option value="none">不使用摄像头 (none)</option>'];
  const seen = new Set(["none"]);
  for (const c of cameras || []) {
    const raw = String(c.raw_id ?? c.index_or_path ?? "");
    if (!raw || seen.has(raw)) continue;
    seen.add(raw);
    options.push(`<option value="${raw}">[${raw}] ${c.name} (${c.width}x${c.height} ${c.backend})</option>`);
  }
  if (currentValue && !seen.has(currentValue)) {
    options.push(`<option value="${currentValue}">[${currentValue}] 当前配置 (未连接)</option>`);
  }
  selectEl.innerHTML = options.join("");
  selectEl.value = currentValue || "none";
}

function renderDeviceSelectors(probe) {
  const sel = probe.device_selection || {};
  const ports = probe.serial_ports || [];
  const cams = probe.cameras || [];
  populatePortSelect($("select-follower-port"), ports, sel.follower_port || "");
  populatePortSelect($("select-leader-port"), ports, sel.leader_port || "");
  populateCameraSelect($("select-front-camera"), cams, sel.front_camera_id || "none");
  populateCameraSelect($("select-wrist-camera"), cams, sel.wrist_camera_id || "none");

  const statusEl = $("device-config-status");
  if (statusEl) {
    statusEl.textContent = `当前配置: 从臂=${sel.follower_port || "-"} | 主臂=${sel.leader_port || "-"} | 前置=${sel.front_camera_id || "none"} | 腕部=${sel.wrist_camera_id || "none"}`;
  }
}

function renderCalibrationFileBadges(calStatus) {
  const box = $("calibration-file-badges");
  if (!box || !calStatus) return;
  const fChip = calStatus.follower_calibrated
    ? '<span class="chip chip-ok">从臂标定: 已就绪</span>'
    : '<span class="chip chip-warn">从臂标定: 未标定</span>';
  const lChip = calStatus.leader_calibrated
    ? '<span class="chip chip-ok">主臂标定: 已就绪</span>'
    : '<span class="chip chip-warn">主臂标定: 未标定</span>';
  box.innerHTML = fChip + lChip;
}

function renderCalibrationState(cal) {
  if (!cal) return;
  const panel = $("calibration-interactive-panel");
  const statusEl = $("calibrate-status");
  const promptEl = $("calibrate-stage-prompt");
  const btnEnter = $("btn-calibrate-enter");
  const btnRecal = $("btn-calibrate-recal");
  const tableWrap = $("calibrate-live-table-wrap");
  const tbody = $("calibrate-live-tbody");

  const stage = cal.stage || "idle";
  const prevStage = state.lastCalStage;
  state.lastCalStage = stage;

  if (stage === "completed" && prevStage !== "completed") {
    apiFetch("/api/hardware/probe").then((r) => {
      if (r.ok) renderHardwareProbe(r.data);
    });
  }

  if (statusEl && (cal.active || stage !== "idle")) {
    statusEl.textContent = `标定目标: ${cal.target} (${cal.port || "-"}) | 模式: ${cal.mode} | 阶段: ${stage}`;
  }

  if (!panel) return;
  if (!cal.active && (stage === "idle" || stage === "stopped")) {
    panel.hidden = true;
    return;
  }

  const btnAutoHome = $("btn-cal-auto-home");
  const btnAutoSweep = $("btn-cal-auto-sweep");
  const calSlidersBox = $("cal-sim-sliders-box");
  const isDryCalActive = Boolean(
    cal.active && cal.mode === "dry_run" && (stage === "waiting_middle" || stage === "recording_range")
  );

  panel.hidden = false;
  if (promptEl) promptEl.textContent = cal.prompt || `标定阶段: ${stage}`;
  if (btnEnter) {
    btnEnter.disabled = !cal.waiting_for_input;
    btnEnter.hidden = !cal.active;
  }
  if (btnRecal) {
    btnRecal.hidden = !(cal.active && cal.allow_recalibrate);
    btnRecal.disabled = !cal.waiting_for_input;
  }
  if (btnAutoHome) btnAutoHome.hidden = !(isDryCalActive && stage === "waiting_middle");
  if (btnAutoSweep) btnAutoSweep.hidden = !(isDryCalActive && stage === "recording_range");
  if (calSlidersBox) calSlidersBox.hidden = !isDryCalActive;
  if (isDryCalActive) {
    const calJoints = cal.target === "leader" ? state.leaderJoints : state.followerJoints;
    for (const j of JOINTS) {
      const sId = `cal-slider-${j.key}`;
      const val = Number(calJoints?.[j.key] ?? j.def);
      if (!isSliderLocked(sId)) {
        const sEl = $(sId);
        const vEl = $(`cal-val-${j.key}`);
        if (sEl) sEl.value = String(val);
        if (vEl) vEl.textContent = formatJointReadout(j.key, val, j.unit);
      }
    }
  }

  const rows = cal.live_table || [];
  if (tableWrap && tbody) {
    if (rows.length > 0) {
      tableWrap.hidden = false;
      tbody.innerHTML = rows
        .map(
          (r) => `<tr>
            <td class="joint-values">${r.name}</td>
            <td class="joint-values">${r.min}</td>
            <td class="joint-values"><strong>${r.pos}</strong></td>
            <td class="joint-values">${r.max}</td>
          </tr>`
        )
        .join("");
    } else {
      tableWrap.hidden = true;
    }
  }
}

/* ==========================================================================
   6-DOF Tabular-Nums Sliders with Anti-Jitter Drag Lock
   ========================================================================== */
function buildSlidersHtml(prefix) {
  return JOINTS.map((j) => {
    return `
      <div class="joint-row">
        <div class="joint-row-main">
          <span class="joint-label" title="${j.title || j.label}">${j.label}</span>
          <input
            class="joint-slider"
            type="range"
            min="${j.min}"
            max="${j.max}"
            step="1"
            value="${j.def}"
            data-joint="${j.key}"
            id="${prefix}-slider-${j.key}"
          />
          <span class="joint-values" id="${prefix}-val-${j.key}">${formatJointReadout(j.key, j.def, j.unit)}</span>
        </div>
      </div>
    `;
  }).join("");
}

function bindSliderDragLock(slider) {
  slider.addEventListener("pointerdown", () => {
    state.activeSliderId = slider.id;
    state.sliderCooldownTs[slider.id] = performance.now() + 300;
  });
}

window.addEventListener("pointerup", () => {
  if (state.activeSliderId) {
    state.sliderCooldownTs[state.activeSliderId] = performance.now() + 260;
    state.activeSliderId = null;
  }
});

function isSliderLocked(sliderId) {
  if (state.activeSliderId === sliderId) return true;
  const until = state.sliderCooldownTs[sliderId] || 0;
  return performance.now() < until;
}

function initJointSliders() {
  const teleopBox = $("joints-container");
  if (teleopBox) {
    teleopBox.innerHTML = buildSlidersHtml("teleop");
    teleopBox.querySelectorAll(".joint-slider").forEach((slider) => {
      bindSliderDragLock(slider);
      slider.addEventListener("input", (ev) => {
        const key = ev.target.dataset.joint;
        const val = Number(ev.target.value);
        state.sliderCooldownTs[slider.id] = performance.now() + 350;
        state.followerJoints[key] = val;
        state.pendingJoints[key] = val;
        const jMeta = JOINTS.find((x) => x.key === key);
        const valEl = $(`teleop-val-${key}`);
        if (valEl && jMeta) valEl.textContent = formatJointReadout(key, val, jMeta.unit);
        updateAllStickViews();
        if (!state.jointSendTimer) {
          state.jointSendTimer = setTimeout(flushPendingJoints, 45);
        }
      });
    });
  }

  for (const arm of ["leader", "follower"]) {
    const box = $(`sim-${arm}-sliders`);
    if (!box) continue;
    box.innerHTML = buildSlidersHtml(`sim-${arm}`);
    box.querySelectorAll(".joint-slider").forEach((slider) => {
      bindSliderDragLock(slider);
      slider.addEventListener("input", (ev) => {
        if (arm === "follower" && state.linkedArms) return;
        const key = ev.target.dataset.joint;
        const val = Number(ev.target.value);
        state.sliderCooldownTs[slider.id] = performance.now() + 280;
        if (arm === "leader") state.leaderJoints[key] = val;
        else state.followerJoints[key] = val;
        state.simPending[arm][key] = val;
        const jMeta = JOINTS.find((x) => x.key === key);
        const valEl = $(`sim-${arm}-val-${key}`);
        if (valEl && jMeta) valEl.textContent = formatJointReadout(key, val, jMeta.unit);
        updateAllStickViews();
        if (!state.simSendTimers[arm]) {
          state.simSendTimers[arm] = setTimeout(() => flushSimJoints(arm), 45);
        }
      });
    });
  }

  // Calibration Joint-Space Temporary Sliders (dry_run only)
  const calBox = $("cal-joints-container");
  if (calBox) {
    calBox.innerHTML = buildSlidersHtml("cal");
    calBox.querySelectorAll(".joint-slider").forEach((slider) => {
      bindSliderDragLock(slider);
      slider.addEventListener("input", (ev) => {
        const key = ev.target.dataset.joint;
        const val = Number(ev.target.value);
        const calTarget = $("calibrate-target")?.value || "follower";
        state.sliderCooldownTs[slider.id] = performance.now() + 280;
        if (calTarget === "leader") state.leaderJoints[key] = val;
        else state.followerJoints[key] = val;
        state.simPending[calTarget][key] = val;
        const jMeta = JOINTS.find((x) => x.key === key);
        const valEl = $(`cal-val-${key}`);
        if (valEl && jMeta) valEl.textContent = formatJointReadout(key, val, jMeta.unit);
        updateAllStickViews();
        if (!state.simSendTimers[calTarget]) {
          state.simSendTimers[calTarget] = setTimeout(() => flushSimJoints(calTarget), 40);
        }
      });
    });
  }
}

async function flushPendingJoints() {
  state.jointSendTimer = null;
  const payload = { ...state.pendingJoints };
  state.pendingJoints = {};
  if (Object.keys(payload).length === 0) return;
  const res = await apiFetch("/api/workflows/teleop/joints", {
    method: "POST",
    body: JSON.stringify({ joints: payload }),
  });
  if (res.ok && res.data) {
    updateJointGauges(res.data.leader_joints, res.data.follower_joints, false);
    if (isTeleopPanelVisible()) refreshCameraImagesOnce();
    if (state.mode === "real" && res.data.real_dispatched === false && !state.torqueEnabled) {
      const now = performance.now();
      if (!state._lastTorqueWarnTs || now - state._lastTorqueWarnTs > 4000) {
        state._lastTorqueWarnTs = now;
        showFeedbackToast("当前从臂已卸力（仅更新仿真），点顶栏「点击上力」即可驱动真机", "warn");
      }
    }
  }
}

async function flushSimJoints(arm) {
  state.simSendTimers[arm] = null;
  const payload = { ...state.simPending[arm] };
  state.simPending[arm] = {};
  if (Object.keys(payload).length === 0) return;
  const linked = Boolean($("chk-sim-linked")?.checked);
  const res = await apiFetch("/api/sim/joints", {
    method: "POST",
    body: JSON.stringify({ arm, joints: payload, linked }),
  });
  if (res.ok && res.data) {
    updateJointGauges(res.data.leader_joints, res.data.follower_joints, false);
    if (state.mode === "real" && res.data.real_dispatched === false && !state.torqueEnabled) {
      const now = performance.now();
      if (!state._lastTorqueWarnTs || now - state._lastTorqueWarnTs > 4000) {
        state._lastTorqueWarnTs = now;
        showFeedbackToast("当前真机已卸力（仅动 3D 仿真），点顶栏「点击上力」即可驱动真机主/从臂", "warn");
      }
    }
  }
}

function updateJointGauges(leader, follower, syncSliders = false) {
  if (!leader || !follower) return;
  state.leaderJoints = leader;
  state.followerJoints = follower;

  for (const j of JOINTS) {
    const lv = Number(leader[j.key] ?? j.def);
    const fv = Number(follower[j.key] ?? j.def);

    const tSliderId = `teleop-slider-${j.key}`;
    if (!isSliderLocked(tSliderId)) {
      const tVal = $(`teleop-val-${j.key}`);
      if (tVal) tVal.textContent = formatJointReadout(j.key, fv, j.unit);
      if (syncSliders) {
        const sl = $(tSliderId);
        if (sl) sl.value = String(fv);
      }
    }

    for (const [arm, val] of [["leader", lv], ["follower", fv]]) {
      const sId = `sim-${arm}-slider-${j.key}`;
      if (!isSliderLocked(sId)) {
        const vEl = $(`sim-${arm}-val-${j.key}`);
        if (vEl) vEl.textContent = formatJointReadout(j.key, val, j.unit);
        if (syncSliders) {
          const sEl = $(sId);
          if (sEl) sEl.value = String(val);
        }
      }
    }

    const calTarget = $("calibrate-target")?.value || "follower";
    const calVal = calTarget === "leader" ? lv : fv;
    const calId = `cal-slider-${j.key}`;
    if (!isSliderLocked(calId)) {
      const cValEl = $(`cal-val-${j.key}`);
      if (cValEl) cValEl.textContent = formatJointReadout(j.key, calVal, j.unit);
      if (syncSliders) {
        const cEl = $(calId);
        if (cEl) cEl.value = String(calVal);
      }
    }
  }
  // Sync Tab 3 live IK sliders & inputs when user is not actively dragging/editing them
  (() => {
    const activeTag = document.activeElement?.id || "";
    if (activeTag.startsWith("ik-")) return;
    if (
      isSliderLocked("ik-slider-x") ||
      isSliderLocked("ik-slider-y") ||
      isSliderLocked("ik-slider-z") ||
      isSliderLocked("ik-slider-grip")
    ) {
      return;
    }
    const targetArm = state.linkedArms ? "leader" : ($("sim-ik-arm-select")?.value || "follower");
    const jSrc = targetArm === "leader" ? state.leaderJoints : state.followerJoints;
    const fk = computeStickFK(jSrc);
    if (fk?.tcp) {
      const rx = Math.round(fk.tcp.x);
      const ry = Math.round(fk.tcp.y);
      const rz = Math.round(fk.tcp.z);
      const rg = Math.round(Number(jSrc?.gripper ?? 20));
      if ($("ik-x")) $("ik-x").value = String(rx);
      if ($("ik-y")) $("ik-y").value = String(ry);
      if ($("ik-z")) $("ik-z").value = String(rz);
      if ($("ik-grip")) $("ik-grip").value = String(rg);
      if ($("ik-slider-x")) $("ik-slider-x").value = String(rx);
      if ($("ik-slider-y")) $("ik-slider-y").value = String(ry);
      if ($("ik-slider-z")) $("ik-slider-z").value = String(rz);
      if ($("ik-slider-grip")) $("ik-slider-grip").value = String(rg);
    }
  })();

  updateAllStickViews();
}

function isTeleopPanelVisible() {
  const panel = $("panel-teleop");
  return Boolean(panel && !panel.hidden);
}

function isCollectPanelVisible() {
  const panel = $("panel-collect");
  return Boolean(panel && !panel.hidden);
}

let _activeTeleopCamTheme = null;
let _activeCollectCamTheme = null;

function refreshCameraImagesOnce(forceRestart = false) {
  const theme = state.theme || "light";
  const ts = Date.now();

  // 1. Tab 2 Teleop Cameras
  const tFront = $("camera-preview-front");
  const tWrist = $("camera-preview-wrist");
  if (tFront || tWrist) {
    if (!isTeleopPanelVisible()) {
      if (_activeTeleopCamTheme !== null) {
        _activeTeleopCamTheme = null;
        if (tFront) tFront.removeAttribute("src");
        if (tWrist) tWrist.removeAttribute("src");
      }
    } else if (forceRestart || _activeTeleopCamTheme !== theme) {
      _activeTeleopCamTheme = theme;
      if (tFront) tFront.src = `/api/cameras/front/stream?theme=${theme}&t=${ts}`;
      if (tWrist) tWrist.src = `/api/cameras/wrist/stream?theme=${theme}&t=${ts}`;
    }
  }

  // 2. Tab 5 Data Collection 1:1 Permanent Dual Cameras
  const cFront = $("collect-cam-front");
  const cWrist = $("collect-cam-wrist");
  if (cFront || cWrist) {
    if (!isCollectPanelVisible()) {
      if (_activeCollectCamTheme !== null) {
        _activeCollectCamTheme = null;
        if (cFront) cFront.removeAttribute("src");
        if (cWrist) cWrist.removeAttribute("src");
      }
    } else if (forceRestart || _activeCollectCamTheme !== theme) {
      _activeCollectCamTheme = theme;
      if (cFront) cFront.src = `/api/cameras/front/stream?theme=${theme}&t=${ts}`;
      if (cWrist) cWrist.src = `/api/cameras/wrist/stream?theme=${theme}&t=${ts}`;
    }
  }
}

function scheduleCameraLoop() {
  // Real-time MJPEG stream handles continuous 25 FPS updates natively
  refreshCameraImagesOnce(false);
}

function applyWorkflowStatus(statusObj) {
  if (!statusObj) return;
  const wf = statusObj.active_workflow || "idle";
  state.activeWorkflow = wf;
  const wfPill = $("workflow-pill");
  if (wfPill) {
    wfPill.textContent = `WORKFLOW: ${String(wf).toUpperCase()}`;
    wfPill.className = "sb-item";
  }
  if (statusObj.mode) setModeUI(statusObj.mode);
  const colProg = $("collect-progress-text");
  if (colProg) {
    if (wf === "collect") {
      const stageTxt = statusObj.collect_stage_text ? ` · ${statusObj.collect_stage_text}` : "";
      colProg.textContent = `采集中: ${statusObj.current_episode || 0}/${statusObj.target_episodes || 0} 回合 (${statusObj.current_step || 0} 帧)${stageTxt}`;
    } else if (statusObj.collect_stage === "completed" && statusObj.collect_stage_text) {
      colProg.textContent = statusObj.collect_stage_text;
    } else {
      colProg.textContent = "待机（未在采集）";
    }
  }
  const colLive = $("collect-live-actions");
  if (colLive) colLive.hidden = wf !== "collect";
  if (Array.isArray(statusObj.logs)) renderLogs(statusObj.logs);
  if (statusObj.calibration) renderCalibrationState(statusObj.calibration);
  if (wf !== "idle") scheduleCameraLoop();
  updateAllStickViews();
}

function validateCollectSpecUI() {
  if (state.filledCollectConfig) {
    if ($("collect-dataset") && !$("collect-dataset").value) {
      $("collect-dataset").value = state.filledCollectConfig.datasetName;
    }
    if ($("collect-task") && !$("collect-task").value) {
      $("collect-task").value = state.filledCollectConfig.taskPrompt;
    }
  }
  const ds = ($("collect-dataset")?.value || "").trim();
  const task = ($("collect-task")?.value || "").trim();
  const fps = Number($("collect-fps")?.value || 30);
  const badge = $("spec-check-badge");
  if (!badge) return;

  const issues = [];
  if (!ds || ds.startsWith("eval_") || /[\s\u4e00-\u9fff/\\]/.test(ds)) {
    issues.push("数据集名需为英文下划线且禁用 eval_");
  }
  if (!task) issues.push("缺少英文任务指令");
  if (fps !== 30) issues.push("建议保持 30 FPS");

  if (issues.length === 0) {
    badge.textContent = "规范校验: 符合标准";
    badge.className = "status-tag tag-ok";
  } else {
    badge.textContent = `规范提示: ${issues.join("；")}`;
    badge.className = "status-tag tag-warn";
  }
}

async function refreshWaypoints() {
  const res = await apiFetch("/api/workflows/teaching/waypoints");
  const tbody = $("waypoints-tbody");
  if (!res.ok || !tbody) return;
  const list = res.data.waypoints || [];
  state.waypoints = list;
  updateAllStickViews();
  if (!list.length) {
    tbody.innerHTML = '<tr><td colspan="5" class="muted">暂无示教轨迹点。</td></tr>';
    return;
  }
  tbody.innerHTML = list
    .map((wp) => {
      const j = wp.joints || {};
      const summary = JOINTS.map((s) => `${s.key.slice(0, 4)}:${Number(j[s.key] ?? 0).toFixed(1)}`).join(" ");
      return `<tr>
        <td>${wp.index + 1}</td>
        <td>${wp.label}</td>
        <td>${(wp.timestamp || "").slice(11, 19)}</td>
        <td class="joint-values">${summary}</td>
        <td>${wp.mode || "dry_run"}</td>
      </tr>`;
    })
    .join("");
}

async function refreshLinearTrajectories() {
  const res = await apiFetch("/api/workflows/teaching/linear/list");
  const sel = $("linear-traj-select");
  const tbody = $("linear-traj-tbody");
  if (!res.ok) return;
  const items = res.data.trajectories || [];
  if (sel) {
    const prev = sel.value;
    if (!items.length) {
      sel.innerHTML = '<option value="">暂无已录制连续轨迹</option>';
    } else {
      sel.innerHTML = items
        .map((t) => {
          const val = JSON.stringify({ name: t.name, episode: t.episode, fps: t.fps });
          const tag = t.mode === "real" ? "真机" : "仿真";
          return `<option value='${val.replace(/'/g, "&apos;")}'>[${tag}] ${t.name} (ep${t.episode} · ${t.frames}帧 @ ${t.fps}Hz)</option>`;
        })
        .join("");
      if (prev && Array.from(sel.options).some((o) => o.value === prev)) {
        sel.value = prev;
      }
    }
  }
  if (tbody) {
    if (!items.length) {
      tbody.innerHTML = '<tr><td colspan="6" class="muted">暂无已录制连续轨迹。</td></tr>';
    } else {
      tbody.innerHTML = items
        .map((t, idx) => {
          const dur = t.fps ? (t.frames / t.fps).toFixed(1) : "0.0";
          const modeLabel = t.mode === "real" ? "真机" : "仿真";
          const safeName = String(t.name).replace(/"/g, "&quot;");
          return `<tr>
            <td>${idx + 1}</td>
            <td><strong>${safeName}</strong></td>
            <td>ep${t.episode}</td>
            <td class="joint-values">${t.frames} 帧 / ${t.fps}Hz (${dur}s)</td>
            <td><span class="status-tag ${t.mode === "real" ? "tag-ok" : "tag-warn"}">${modeLabel}</span></td>
            <td>
              <button type="button" class="btn btn-xs btn-primary btn-row-linear-replay" data-name="${safeName}" data-ep="${t.episode}" data-fps="${t.fps}">回放</button>
              <button type="button" class="btn btn-xs btn-secondary btn-row-linear-del" data-name="${safeName}" data-ep="${t.episode}">删除</button>
            </td>
          </tr>`;
        })
        .join("");
      tbody.querySelectorAll(".btn-row-linear-replay").forEach((b) => {
        b.addEventListener("click", async () => {
          ensureStageVisibleForAction();
          const name = b.dataset.name;
          const episode = Number(b.dataset.ep || 0);
          const fps = Number(b.dataset.fps || 30);
          const r = await apiFetch("/api/workflows/teaching/linear/replay", {
            method: "POST",
            body: JSON.stringify({ name, episode, fps }),
          });
          if (r.ok) {
            state.activeWorkflow = "linear_replay";
            if ($("linear-teach-status")) {
              $("linear-teach-status").textContent = `线性回放中 [${r.data.mode}]: ${name} (ep${episode})`;
            }
            showFeedbackToast(`开始线性回放: ${name}`, "ok");
          } else {
            showAnomalyToast(r.error || "启动线性回放失败");
          }
        });
      });
      tbody.querySelectorAll(".btn-row-linear-del").forEach((b) => {
        b.addEventListener("click", async () => {
          const name = b.dataset.name;
          const episode = Number(b.dataset.ep || 0);
          await apiFetch("/api/workflows/teaching/linear/delete", {
            method: "POST",
            body: JSON.stringify({ name, episode }),
          });
          await refreshLinearTrajectories();
          showFeedbackToast(`已删除轨迹 ${name} (ep${episode})`, "info");
        });
      });
    }
  }
}

async function replayEpisodeOnStick(dataset, episodeIndex) {
  if (state.epReplayTimer) {
    clearInterval(state.epReplayTimer);
    state.epReplayTimer = null;
  }
  ensureStageVisibleForAction();
  const res = await apiFetch(`/api/workflows/collect/episodes/${encodeURIComponent(dataset)}/${episodeIndex}`);
  if (!res.ok || !Array.isArray(res.data?.frames) || res.data.frames.length === 0) {
    appendLocalLog("该 Episode 暂无可回放帧数据。");
    return;
  }
  const frames = res.data.frames;
  let idx = 0;
  appendLocalLog(`开始回放 ${dataset} #${episodeIndex} (${frames.length} 帧)...`);
  state.epReplayTimer = setInterval(() => {
    if (idx >= frames.length) {
      clearInterval(state.epReplayTimer);
      state.epReplayTimer = null;
      state.overlayBadge = "";
      updateAllStickViews();
      return;
    }
    const f = frames[idx];
    const st = f["observation.state"] || [];
    const jObj = {};
    JOINTS.forEach((jm, i) => {
      if (i < st.length) jObj[jm.key] = Number(st[i]);
    });
    state.followerJoints = { ...state.followerJoints, ...jObj };
    state.overlayBadge = `回放 #${episodeIndex} (${idx + 1}/${frames.length})`;
    stageViews.follower?.setState({
      joints: state.followerJoints,
      overlayBadge: state.overlayBadge,
    });
    if ($("collect-stick-data")) {
      $("collect-stick-data").innerHTML = renderStickTelemetryHtml(
        state.followerJoints,
        `回放 ${dataset} #${episodeIndex} (${idx + 1}/${frames.length})`
      );
    }
    idx += 1;
  }, 60);
}

const WAVEFORM_COLORS = ["#2563EB", "#16A34A", "#EA580C", "#9333EA", "#0891B2", "#DC2626"];

function drawEpisodeWaveform(canvas, frames) {
  if (!canvas || !Array.isArray(frames) || frames.length === 0) return;
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const cssW = Math.max(260, canvas.clientWidth || 520);
  const cssH = 175;
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const dark = state.theme === "dark";
  ctx.fillStyle = dark ? "#14181F" : "#FAFBFD";
  ctx.fillRect(0, 0, cssW, cssH);

  const padL = 34;
  const padR = 10;
  const padT = 22;
  const padB = 20;
  const plotW = cssW - padL - padR;
  const plotH = cssH - padT - padB;

  let minV = -90;
  let maxV = 100;
  for (const f of frames) {
    for (const arr of [f["observation.state"], f.action]) {
      if (!Array.isArray(arr)) continue;
      for (const v of arr) {
        const n = Number(v);
        if (Number.isFinite(n)) {
          if (n < minV) minV = n;
          if (n > maxV) maxV = n;
        }
      }
    }
  }
  const span = Math.max(10, maxV - minV);
  const yOf = (v) => padT + plotH - ((v - minV) / span) * plotH;
  const xOf = (i) => padL + (frames.length <= 1 ? 0 : (i / (frames.length - 1)) * plotW);

  // Grid lines
  ctx.strokeStyle = dark ? "rgba(255,255,255,0.08)" : "rgba(15,23,42,0.08)";
  ctx.lineWidth = 1;
  ctx.fillStyle = dark ? "#94A3B8" : "#64748B";
  ctx.font = "10px monospace";
  for (const val of [maxV, 0, minV]) {
    const y = yOf(val);
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(cssW - padR, y);
    ctx.stroke();
    ctx.fillText(Math.round(val).toString(), 4, y + 3);
  }

  // Draw 6-DOF observation.state (solid) & action (dashed)
  const step = Math.max(1, Math.floor(frames.length / 320));
  for (let j = 0; j < 6; j++) {
    const color = WAVEFORM_COLORS[j % WAVEFORM_COLORS.length];
    // Action (dashed)
    ctx.save();
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.42;
    ctx.setLineDash([4, 3]);
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    for (let i = 0; i < frames.length; i += step) {
      const v = Number(frames[i].action?.[j] ?? 0);
      const x = xOf(i);
      const y = yOf(v);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.restore();

    // State (solid)
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < frames.length; i += step) {
      const v = Number(frames[i]["observation.state"]?.[j] ?? 0);
      const x = xOf(i);
      const y = yOf(v);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  // Compact top legend (never clips at narrow inspector widths)
  const labels = ["J1", "J2", "J3", "J4", "J5", "Grip"];
  let lx = padL;
  ctx.font = "10px sans-serif";
  for (let j = 0; j < 6; j++) {
    ctx.fillStyle = WAVEFORM_COLORS[j];
    ctx.fillRect(lx, 7, 7, 6);
    ctx.fillStyle = dark ? "#CBD5E1" : "#334155";
    ctx.fillText(labels[j], lx + 9, 13);
    lx += j === 5 ? 36 : 28;
  }
  ctx.fillStyle = dark ? "#94A3B8" : "#64748B";
  ctx.textAlign = "right";
  ctx.fillText("实线=State 虚线=Action", cssW - padR, cssH - 5);
  ctx.textAlign = "left";
}

function switchInspectorToLive() {
  state.inspectorMode = "live";
  state.selectedEpisode = null;
  document.querySelectorAll("#episodes-tbody tr.ep-row-clickable").forEach((tr) => {
    tr.classList.remove("ep-row-selected");
  });
  const btnLive = $("btn-inspector-live");
  if (btnLive) btnLive.hidden = true;
  const acts = $("ep-inspector-actions");
  if (acts) acts.hidden = true;
  const videoBox = $("ep-video-box");
  if (videoBox) {
    videoBox.hidden = true;
    videoBox.innerHTML = "";
  }
  renderLiveCollectWaveform();
}

function renderLiveCollectWaveform() {
  if (state.inspectorMode && state.inspectorMode !== "live") return;
  const frames = state.liveCollectFrames || [];
  const metaEl = $("ep-inspector-meta");
  const isCol = state.activeWorkflow === "collect";
  if (metaEl) {
    const modeLabel = isCol ? "采集中 · 实时动态曲线" : "待机 · 实时 6-DOF 波形";
    metaEl.innerHTML = `
      <span class="chip ${isCol ? "chip-ok" : ""}">${modeLabel}</span>
      <span class="chip">缓冲 ${frames.length} 帧 (实线=从臂State / 虚线=主臂Action)</span>
    `;
  }
  const canvas = $("ep-waveform-canvas");
  if (canvas && frames.length > 0 && isCollectPanelVisible()) {
    drawEpisodeWaveform(canvas, frames);
  }
}

function pushLiveCollectFrame(leader, follower, wfStatus) {
  if (!leader && !follower) return;
  if (!Array.isArray(state.liveCollectFrames)) state.liveCollectFrames = [];
  const curEp = Number(wfStatus?.current_episode || 0);
  if (state.activeWorkflow === "collect" && curEp !== state._lastLiveEpNum) {
    state._lastLiveEpNum = curEp;
    state.liveCollectFrames = [];
  }
  const obs = JOINTS.map((j) => Number(follower?.[j.key] ?? 0));
  const act = JOINTS.map((j) => Number(leader?.[j.key] ?? obs[JOINTS.indexOf(j)] ?? 0));
  state.liveCollectFrames.push({
    frame_index: state.liveCollectFrames.length,
    "observation.state": obs,
    action: act,
  });
  if (state.liveCollectFrames.length > 240) {
    state.liveCollectFrames.shift();
  }
  if ((state.inspectorMode || "live") === "live" && isCollectPanelVisible()) {
    const now = performance.now();
    if (!state._lastWaveDrawTs || now - state._lastWaveDrawTs >= 55) {
      state._lastWaveDrawTs = now;
      renderLiveCollectWaveform();
    }
  }
}

async function inspectEpisode(dataset, episodeIndex) {
  state.inspectorMode = "history";
  state.selectedEpisode = { dataset, episodeIndex };
  document.querySelectorAll("#episodes-tbody tr.ep-row-clickable").forEach((tr) => {
    const match = tr.dataset.dataset === dataset && Number(tr.dataset.ep) === episodeIndex;
    tr.classList.toggle("ep-row-selected", match);
  });

  const bodyEl = $("ep-inspector-body");
  const metaEl = $("ep-inspector-meta");
  const videoBox = $("ep-video-box");
  const btnLive = $("btn-inspector-live");
  const acts = $("ep-inspector-actions");
  if (!bodyEl || !metaEl) return;

  const res = await apiFetch(
    `/api/workflows/collect/episodes/${encodeURIComponent(dataset)}/${episodeIndex}`
  );
  if (!res.ok || !res.data) {
    showAnomalyToast(res.error || `无法加载 ${dataset} #${episodeIndex}`);
    return;
  }
  const ep = res.data;
  const frames = ep.frames || [];
  const fps = Number(ep.fps || 30);
  const durS = (frames.length / Math.max(1, fps)).toFixed(1);
  if (btnLive) btnLive.hidden = false;
  if (acts) acts.hidden = false;

  metaEl.innerHTML = `
    <span class="chip chip-ok">${ep.dataset} #${ep.episode_index}</span>
    <span class="chip">${ep.source === "real" ? "真机 Parquet" : "Dry-Run JSON"}</span>
    <span class="chip">${frames.length} 帧 · ${fps}Hz · ${durS}s</span>
    <span class="chip" title="${ep.task || ""}">${(ep.task || "未标注任务").slice(0, 42)}</span>
  `;

  drawEpisodeWaveform($("ep-waveform-canvas"), frames);

  if (videoBox) {
    const vUrls = ep.video_urls || {};
    const cams = Object.keys(vUrls);
    if (cams.length > 0) {
      videoBox.hidden = false;
      videoBox.innerHTML = cams
        .map(
          (cam) => `<div>
            <div class="mono-meta" style="font-size:10.5px;margin-bottom:2px">cam: ${cam}</div>
            <video src="${vUrls[cam]}" controls muted playsinline preload="metadata"></video>
          </div>`
        )
        .join("");
    } else {
      videoBox.hidden = true;
      videoBox.innerHTML = "";
    }
  }
}

async function refreshEpisodes() {
  const res = await apiFetch("/api/workflows/collect/episodes");
  const tbody = $("episodes-tbody");
  if (!res.ok || !tbody) return;
  const eps = res.data.episodes || [];
  state.episodesList = eps;
  if (typeof syncVLADatasetSelect === "function") syncVLADatasetSelect(eps);
  if (!eps.length) {
    tbody.innerHTML = '<tr><td colspan="6" class="muted">暂无已保存的 Episode。</td></tr>';
    return;
  }
  tbody.innerHTML = eps
    .map((ep) => {
      const isSelected =
        state.selectedEpisode &&
        state.selectedEpisode.dataset === ep.dataset &&
        state.selectedEpisode.episodeIndex === ep.episode_index;
      const srcTag =
        ep.source === "real"
          ? `<span class="status-tag tag-ok">真机${ep.has_video ? "+MP4" : ""}</span>`
          : `<span class="status-tag tag-mono">仿真</span>`;
      return `<tr class="ep-row-clickable${isSelected ? " ep-row-selected" : ""}" data-dataset="${ep.dataset}" data-ep="${ep.episode_index}">
        <td class="ep-cell-nowrap"><strong>${ep.dataset}</strong></td>
        <td class="ep-cell-nowrap">#${ep.episode_index}</td>
        <td class="ep-cell-task" title="${ep.task || ""}">${ep.task || "-"}</td>
        <td class="joint-values ep-cell-nowrap">${ep.num_frames}/${ep.fps}Hz</td>
        <td class="ep-cell-nowrap">${srcTag}</td>
        <td class="ep-cell-nowrap"><button type="button" class="btn btn-xs btn-primary btn-ep-stick-replay" data-dataset="${ep.dataset}" data-ep="${ep.episode_index}">回放</button></td>
      </tr>`;
    })
    .join("");

  tbody.querySelectorAll("tr.ep-row-clickable").forEach((tr) => {
    tr.addEventListener("click", () => {
      inspectEpisode(tr.dataset.dataset, Number(tr.dataset.ep));
    });
  });

  tbody.querySelectorAll(".btn-ep-stick-replay").forEach((btn) => {
    btn.addEventListener("click", (ev) => {
      ev.stopPropagation();
      inspectEpisode(btn.dataset.dataset, Number(btn.dataset.ep));
      replayEpisodeOnStick(btn.dataset.dataset, Number(btn.dataset.ep));
    });
  });
}

/* ==========================================================================
   Tab 6: Cleanly Decoupled Open-Source Datasets & Honest External Tools
   ========================================================================== */
function stopPluginReplayTimer() {
  if (state.pluginReplayTimer) {
    clearInterval(state.pluginReplayTimer);
    state.pluginReplayTimer = null;
  }
  state.overlayBadge = "";
  const stageBadge = $("plugin-stage-badge");
  if (stageBadge) {
    stageBadge.textContent = "就绪";
    stageBadge.className = "status-tag tag-mono";
  }
  updateAllStickViews();
}

function fillDatasetIntoConfigs(repoId, taskPrompt) {
  const rawSlug = String(repoId || "").split("/").pop() || "so101_dataset";
  const cleanName = rawSlug.replace(/[^a-zA-Z0-9_]/g, "_").replace(/^eval_/, "train_");
  const prompt = taskPrompt || "Open the drawer, put the block inside, then close the drawer";

  state.filledCollectConfig = { datasetName: cleanName, taskPrompt: prompt };
  if ($("collect-dataset")) $("collect-dataset").value = cleanName;
  if ($("collect-task")) $("collect-task").value = prompt;
  if ($("vla-prompt")) $("vla-prompt").value = prompt;
  validateCollectSpecUI();

  appendLocalLog(`已将开源数据集 [${repoId}] 填入采集与 VLA 配置 (${cleanName})`);
  const out = $("plugin-workbench-output");
  if (out) {
    out.innerHTML = `
      <div class="card-head-inline">
        <strong>已同步至「5. 数据采集」与「7. VLA 云端训练」配置</strong>
        <span class="chip chip-ok">dataset_name=${cleanName}</span>
      </div>
      <div class="muted">英文指令 (single_task): <code>${prompt}</code></div>
    `;
  }
}

async function playDatasetTrajectoryOnPluginStage(repoId) {
  stopPluginReplayTimer();
  ensureStageVisibleForAction();
  const url = repoId
    ? `/api/plugins/open_source_datasets/preview?repo_id=${encodeURIComponent(repoId)}`
    : "/api/plugins/open_source_datasets/preview";
  const res = await apiFetch(url);
  if (!res.ok || !res.data?.dataset) {
    appendLocalLog(`开源数据集预览加载失败: ${repoId || "default"}`);
    return;
  }

  const ds = res.data.dataset;
  const traj =
    Array.isArray(ds.sample_trajectory) && ds.sample_trajectory.length > 0
      ? ds.sample_trajectory
      : [{ phase: "1. 初始姿态", shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 0, wrist_flex: 0, wrist_roll: 0, gripper: 20 }];

  const stageBadge = $("plugin-stage-badge");
  if (stageBadge) {
    stageBadge.textContent = `正在演练: ${ds.repo_id}`;
    stageBadge.className = "status-tag tag-ok";
  }

  const interpFrames = [];
  const subSteps = 6;
  for (let i = 0; i < traj.length; i++) {
    const cur = traj[i];
    const next = traj[(i + 1) % traj.length];
    for (let s = 0; s < subSteps; s++) {
      const alpha = s / subSteps;
      const joints = {};
      for (const j of JOINTS) {
        const a = Number(cur[j.key] ?? j.def);
        const b = Number(next[j.key] ?? a);
        joints[j.key] = Number((a + (b - a) * alpha).toFixed(1));
      }
      interpFrames.push({
        phaseIndex: i,
        phaseLabel: cur.phase || `阶段 ${i + 1}`,
        joints,
      });
    }
  }

  const renderWorkbenchCard = (activePhaseIdx, currentJoints) => {
    const out = $("plugin-workbench-output");
    if (!out) return;
    const phaseChips = traj
      .map((step, idx) => {
        const cls = idx === activePhaseIdx ? "chip chip-ok" : "chip";
        return `<span class="${cls}">${step.phase || `Step ${idx + 1}`}</span>`;
      })
      .join("");
    const activePhase = traj[activePhaseIdx] || traj[0];
    const tcp = computeStickFK(currentJoints).tcp;

    out.innerHTML = `
      <div class="card-head-inline">
        <div class="btn-group">
          <strong>${ds.repo_id}</strong>
          <span class="chip chip-ok">${ds.format}</span>
          <span class="chip">${ds.stats}</span>
        </div>
        <div class="btn-group">
          <button type="button" class="btn btn-xs btn-primary" id="btn-wb-fill-active">填入采集与 VLA 配置</button>
          <a class="btn btn-xs btn-secondary" href="${ds.visualizer_url}" target="_blank" rel="noopener">官方在线可视化</a>
          <a class="btn btn-xs btn-secondary" href="${ds.hf_url}" target="_blank" rel="noopener">HF 仓库</a>
        </div>
      </div>
      <div class="muted">任务: <strong>${ds.task}</strong> | 英文指令: <code>${ds.recommended_task_prompt || ds.task}</code></div>
      <div class="chip-row mt-6">${phaseChips}</div>
      <div class="kv-item mt-6">
        当前阶段: <strong>${activePhase.phase || "-"}</strong> |
        末端 TCP: <code>X=${fmtZero(tcp.x, 1)} Y=${fmtZero(tcp.y, 1)} Z=${fmtZero(tcp.z, 1)} mm</code> |
        夹爪: <code>${Number(currentJoints.gripper ?? 20).toFixed(0)}%</code>
      </div>
    `;
    $("btn-wb-fill-active")?.addEventListener("click", () => {
      fillDatasetIntoConfigs(ds.repo_id, ds.recommended_task_prompt || ds.task);
    });
  };

  let frameIdx = 0;
  const applyFrame = () => {
    const frame = interpFrames[frameIdx];
    state.followerJoints = { ...state.followerJoints, ...frame.joints };
    state.overlayBadge = frame.phaseLabel;
    stageViews.follower?.setState({
      joints: frame.joints,
      waypoints: traj.map((t, idx) => ({ index: idx, label: t.phase || `P${idx + 1}`, joints: t })),
      overlayBadge: frame.phaseLabel,
    });
    // Also sync to backend simulator when on Step 0 of each phase so /sim-viewport popup moves in sync
    if (frameIdx % 3 === 0) {
      apiFetch("/api/sim/joints", {
        method: "POST",
        body: JSON.stringify({ arm: "follower", joints: frame.joints }),
      });
    }
    if ($("plugin-stick-data")) {
      $("plugin-stick-data").innerHTML = renderStickTelemetryHtml(
        frame.joints,
        `${ds.task} — ${frame.phaseLabel}`
      );
    }
    renderWorkbenchCard(frame.phaseIndex, frame.joints);
  };

  applyFrame();
  appendLocalLog(`已在 3D 舞台启动开源数据集轨迹演练: ${ds.repo_id}`);

  state.pluginReplayTimer = setInterval(() => {
    if ($("panel-plugins")?.hidden) return;
    frameIdx = (frameIdx + 1) % interpFrames.length;
    applyFrame();
  }, 180);
}

async function showServoDiagnosticsOnPluginStage() {
  stopPluginReplayTimer();
  const res = await apiFetch("/api/plugins/feetech_servo_bench/diagnostics");
  if (!res.ok || !res.data) {
    appendLocalLog("飞特舵机总线诊断获取失败。");
    return;
  }
  const diag = res.data;
  const servos = diag.servos || [];
  const isSim = Boolean(diag.simulated);

  const stageBadge = $("plugin-stage-badge");
  if (stageBadge) {
    stageBadge.textContent = isSim ? "飞特总线诊断 (模拟态)" : "飞特总线诊断 (真机在线)";
    stageBadge.className = isSim ? "status-tag tag-warn" : "status-tag tag-ok";
  }

  if ($("plugin-stick-data")) {
    $("plugin-stick-data").innerHTML = renderStickTelemetryHtml(
      state.followerJoints,
      "飞特 STS3215 6 轴当前角度与编码器换算"
    );
  }

  const out = $("plugin-workbench-output");
  if (out) {
    const rowsHtml = servos
      .map(
        (s) => `
        <tr>
          <td class="joint-values">#${s.servo_id}</td>
          <td class="joint-values"><strong>${s.joint_name}</strong></td>
          <td class="joint-values">${s.angle_val}${s.unit} <span class="muted">[${s.min_limit}..${s.max_limit}]</span></td>
          <td class="joint-values"><strong>${s.Present_Position}</strong> / 4095</td>
          <td class="joint-values">${s.Homing_Offset}</td>
          <td class="joint-values">${s.Present_Voltage_V}</td>
          <td class="joint-values">${s.Present_Temperature_C}</td>
          <td><span class="chip ${isSim ? "chip-warn" : "chip-ok"}">${isSim ? "模拟态 (Lock=--)" : `Lock=${s.Lock} / Torque=${s.Torque_Enable}`}</span></td>
        </tr>`
      )
      .join("");

    out.innerHTML = `
      <div class="card-head-inline">
        <div class="btn-group">
          <strong>飞特 STS3215 6 轴总线寄存器诊断</strong>
          <span class="chip ${isSim ? "chip-warn" : "chip-ok"}">${diag.bus_protocol}</span>
        </div>
        <button type="button" class="btn btn-xs btn-secondary" id="btn-wb-refresh-servo">刷新寄存器快照</button>
      </div>
      <div class="table-wrap">
        <table>
          <thead>
            <tr>
              <th>ID</th>
              <th>关节名称</th>
              <th>当前值 / URDF 限位</th>
              <th>编码器步数 (Present_Position)</th>
              <th>Homing_Offset</th>
              <th>总线电压</th>
              <th>内部温度</th>
              <th>EEPROM 锁与状态</th>
            </tr>
          </thead>
          <tbody>${rowsHtml}</tbody>
        </table>
      </div>
    `;
    $("btn-wb-refresh-servo")?.addEventListener("click", showServoDiagnosticsOnPluginStage);
  }
  appendLocalLog("已加载飞特 STS3215 6 轴寄存器诊断表。");
}

async function refreshPlugins() {
  const res = await apiFetch("/api/plugins");
  const container = $("plugin-list");
  const dsTbody = $("open-datasets-tbody");
  if (!res.ok || !container) return;
  const plugins = res.data.plugins || [];
  state.pluginsCache = plugins;

  // 1. Render Open-Source Datasets into dedicated Section A table (#open-datasets-tbody)
  const dsPlugin = plugins.find((p) => p.id === "open_source_datasets");
  const datasets = dsPlugin?.datasets || [];
  if (dsTbody) {
    dsTbody.innerHTML = datasets
      .map(
        (ds) => `
        <tr>
          <td class="joint-values" style="word-break:break-all;"><code>${ds.repo_id}</code></td>
          <td>${ds.task}</td>
          <td>${ds.stats}</td>
          <td><span class="chip">${ds.format}</span></td>
          <td>
            <div class="btn-group">
              <button type="button" class="btn btn-xs btn-primary btn-dataset-replay" data-repo="${ds.repo_id}">3D 演练</button>
              <button type="button" class="btn btn-xs btn-secondary btn-dataset-fill" data-repo="${ds.repo_id}" data-prompt="${ds.recommended_task_prompt || ds.task}">填入配置</button>
              <a class="btn btn-xs btn-secondary" href="${ds.visualizer_url}" target="_blank" rel="noopener">HF 在线检视</a>
            </div>
          </td>
        </tr>`
      )
      .join("");

    dsTbody.querySelectorAll(".btn-dataset-replay").forEach((btn) => {
      btn.addEventListener("click", () => {
        playDatasetTrajectoryOnPluginStage(btn.dataset.repo);
      });
    });

    dsTbody.querySelectorAll(".btn-dataset-fill").forEach((btn) => {
      btn.addEventListener("click", () => {
        fillDatasetIntoConfigs(btn.dataset.repo, btn.dataset.prompt);
      });
    });
  }

  // 2. Render External CLI Tool Adapters into Section B (#plugin-list) with honest availability & disabled launch when missing deps
  container.innerHTML = plugins
    .map((p) => {
      const canLaunch = Boolean(p.available && p.os_supported);
      const isCatalog = p.id === "open_source_datasets";
      let statusChip = "";
      if (p.status === "running") {
        statusChip = `<span class="chip chip-ok">运行中 (PID ${p.pid})</span>`;
      } else if (isCatalog) {
        statusChip = `<span class="chip chip-ok">内置目录 (${datasets.length} 个数据集)</span>`;
      } else if (!p.os_supported) {
        statusChip = '<span class="chip chip-warn">仅限 Ubuntu / 云端 Linux</span>';
      } else if (!p.available) {
        statusChip = '<span class="chip chip-warn">未安装可选依赖</span>';
      } else {
        statusChip = '<span class="chip chip-ok">可启动</span>';
      }

      const hint = p.install_hint ? `<div class="muted mt-6">${p.install_hint}</div>` : "";
      let actionBtns = "";
      if (isCatalog) {
        actionBtns = `<button type="button" class="btn btn-xs btn-secondary btn-plugin-workbench" data-id="${p.id}">在上方演练中心回放</button>`;
      } else if (p.id === "feetech_servo_bench") {
        actionBtns = `
          <button type="button" class="btn btn-xs btn-secondary btn-plugin-workbench" data-id="${p.id}">内置寄存器诊断</button>
          <button type="button" class="btn btn-xs btn-primary btn-plugin-launch" data-id="${p.id}" ${canLaunch ? "" : 'disabled title="缺少可选外部依赖"'}>${canLaunch ? "启动外部 MotorLab" : "未安装外部进程"}</button>
          ${p.status === "running" ? `<button type="button" class="btn btn-xs btn-danger btn-plugin-stop" data-id="${p.id}">停止</button>` : ""}
        `;
      } else {
        actionBtns = `
          <button type="button" class="btn btn-xs btn-primary btn-plugin-launch" data-id="${p.id}" ${canLaunch ? "" : 'disabled title="当前环境未安装该可选依赖或不受此 OS 支持"'}>${
            !p.os_supported ? "仅支持 Linux" : canLaunch ? "启动外部进程" : "缺少可选依赖"
          }</button>
          ${p.status === "running" ? `<button type="button" class="btn btn-xs btn-danger btn-plugin-stop" data-id="${p.id}">停止</button>` : ""}
        `;
      }

      return `
        <div class="card" data-plugin-id="${p.id}">
          <div class="card-head-inline">
            <h3>${p.name}</h3>
            <div class="chip-row">${statusChip}</div>
          </div>
          <div class="muted">${p.description}</div>
          ${hint}
          <div class="btn-group mt-8">${actionBtns}</div>
        </div>
      `;
    })
    .join("");

  container.querySelectorAll(".btn-plugin-workbench").forEach((btn) => {
    btn.addEventListener("click", async () => {
      if (btn.dataset.id === "feetech_servo_bench") {
        await showServoDiagnosticsOnPluginStage();
      } else {
        await playDatasetTrajectoryOnPluginStage();
      }
    });
  });

  container.querySelectorAll(".btn-plugin-launch").forEach((btn) => {
    btn.addEventListener("click", async () => {
      if (btn.disabled) return;
      const id = btn.dataset.id;
      const r = await apiFetch(`/api/plugins/${id}/launch`, {
        method: "POST",
        body: JSON.stringify({ args: {} }),
      });
      if (r.ok) {
        appendLocalLog(`外部插件进程 ${id} 启动成功。`);
        showFeedbackToast(`外部工具 ${id} 已启动`, "ok");
      } else {
        const msg = r.error || r.data?.detail || r.data?.install_hint || `插件 ${id} 启动失败`;
        showAnomalyToast(msg);
      }
      await refreshPlugins();
    });
  });

  container.querySelectorAll(".btn-plugin-stop").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const id = btn.dataset.id;
      await apiFetch(`/api/plugins/${id}/stop`, { method: "POST" });
      await refreshPlugins();
    });
  });
}

function syncVLADatasetSelect(episodes) {
  const sel = $("vla-dataset-select");
  if (!sel) return;
  const prev = sel.value;
  const seen = new Map();
  for (const ep of episodes || []) {
    if (!ep.dataset) continue;
    const prevRec = seen.get(ep.dataset) || { count: 0, source: ep.source || "dry_run", task: ep.task || "" };
    prevRec.count += 1;
    if (ep.source === "real") prevRec.source = "real";
    if (!prevRec.task && ep.task) prevRec.task = ep.task;
    seen.set(ep.dataset, prevRec);
  }
  const collectVal = ($("collect-dataset")?.value || "").trim();
  if (collectVal && !seen.has(collectVal)) {
    seen.set(collectVal, { count: 0, source: "config", task: $("collect-task")?.value || "" });
  }
  if (seen.size === 0) {
    seen.set("yyzmiao_pick_cube_demo", { count: 0, source: "default", task: "Pick up the cube" });
  }
  sel.innerHTML = Array.from(seen.entries())
    .map(([ds, info]) => {
      const tag =
        info.source === "real"
          ? `真机 ${info.count}条`
          : info.source === "dry_run"
          ? `仿真 ${info.count}条`
          : "当前配置";
      return `<option value="${ds}" data-task="${(info.task || "").replace(/"/g, "&quot;")}">${ds} (${tag})</option>`;
    })
    .join("");
  if (prev && seen.has(prev)) sel.value = prev;
  renderVLAConfigCards();
}

function renderVLAConfigCards() {
  const grid = $("vla-configs-grid");
  const cfgs = state.vlaConfigs || [];
  if (!grid || !cfgs.length) return;
  const ds = ($("vla-dataset-select")?.value || "yyzmiao_pick_cube_demo").trim();
  const taskPrompt = ($("vla-prompt")?.value || "Pick up the cube and place it in the bin").trim();
  const repoArg = ds.includes("/") ? ds : `local/${ds}`;

  grid.innerHTML = cfgs
    .map((c, idx) => {
      const rawCmd = String(c.command || "");
      let trainCmd = rawCmd
        .replace(/--dataset\.repo_id=\S+/g, `--dataset.repo_id=${repoArg}`)
        .replace(/--dataset\.root=\S+/g, `--dataset.root=./data/datasets/${ds}`);
      if (!trainCmd.includes("--dataset.repo_id=")) {
        trainCmd = `${trainCmd} --dataset.repo_id=${repoArg} --dataset.root=./data/datasets/${ds}`;
      }
      const policySlug = String(c.id || c.title || "policy")
        .toLowerCase()
        .replace(/[^a-z0-9]+/g, "_");
      const evalCmd =
        `lerobot-record --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=so101_follower ` +
        `--dataset.repo_id=local/eval_${ds} --dataset.single_task="${taskPrompt}" ` +
        `--policy.path=./outputs/train/${policySlug}_${ds}/checkpoints/last/pretrained_model`;

      return `
        <div class="card">
          <div class="panel-header">
            <h3>${c.title}</h3>
            <div class="btn-group">
              <button type="button" class="btn btn-xs btn-primary btn-copy-vla" data-kind="train" data-idx="${idx}">复制训练命令</button>
              <button type="button" class="btn btn-xs btn-secondary btn-copy-vla" data-kind="eval" data-idx="${idx}">复制真机推理命令</button>
            </div>
          </div>
          <div class="chip-row mt-6">
            <span class="chip">${c.path}</span>
            <span class="chip chip-ok">显存: ${c.vram}</span>
            <span class="chip">数据集: ${ds}</span>
          </div>
          <p class="muted mt-6">平台: ${c.platform}</p>
          <pre class="code-box mt-6" id="vla-train-cmd-${idx}">${trainCmd}</pre>
          <pre class="code-box mt-6" id="vla-eval-cmd-${idx}" hidden>${evalCmd}</pre>
        </div>
      `;
    })
    .join("");

  grid.querySelectorAll(".btn-copy-vla").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const idx = btn.dataset.idx;
      const kind = btn.dataset.kind;
      const el = $(kind === "eval" ? `vla-eval-cmd-${idx}` : `vla-train-cmd-${idx}`);
      const text = el?.textContent || "";
      try {
        await navigator.clipboard.writeText(text);
        showFeedbackToast(
          kind === "eval" ? "已复制真机推理评估命令到剪贴板" : "已复制 Ubuntu 训练命令到剪贴板",
          "ok"
        );
      } catch {
        showFeedbackToast("已选中命令（浏览器未授权剪贴板，请手动复制）", "info");
      }
    });
  });
}

async function loadVLAConfigs() {
  const res = await apiFetch("/api/vla/configs");
  if (!res.ok) return;
  const notice = res.data.pi07_notice || {};
  const noticeEl = $("pi07-notice-card");
  if (noticeEl) {
    noticeEl.innerHTML = `
      <h3>${notice.title || "pi0.7 状态说明"}</h3>
      <p class="mt-6"><strong>论文</strong>: ${notice.paper || "arXiv:2604.15483v2"} | <strong>状态</strong>: ${notice.status || "未公开权重"}</p>
      <p class="muted mt-6">${notice.recommendation || ""}</p>
    `;
  }
  state.vlaConfigs = res.data.configs || [];
  renderVLAConfigCards();
}

/* ==========================================================================
   Navigation Tabs & Stage Collapse Memory (1..4 Expanded / 5..7 Collapsed Default)
   ========================================================================== */
function initTabs() {
  const buttons = document.querySelectorAll(".tab-btn");
  const panels = document.querySelectorAll(".tab-panel");
  const titleEl = $("active-module-title");
  const defaultExpandedTabs = new Set(["dashboard", "teleop", "sim", "teaching"]);

  buttons.forEach((btn) => {
    btn.addEventListener("click", () => {
      const target = btn.dataset.tab;
      state.activeTab = target;
      buttons.forEach((b) => {
        const active = b === btn;
        b.classList.toggle("active", active);
        b.setAttribute("aria-selected", active ? "true" : "false");
      });
      if (titleEl && btn.dataset.title) {
        titleEl.textContent = btn.dataset.title;
      }
      panels.forEach((p) => {
        p.hidden = p.id !== `panel-${target}`;
      });

      // Respect user manual stage collapse override; otherwise apply 1..4 open / 5..7 collapsed default
      if (state.userStageCollapsed === null) {
        applyStageVisibility(!defaultExpandedTabs.has(target));
      }

      // Restore protected cross-tab filled config if present
      if (state.filledCollectConfig) {
        if ($("collect-dataset")) $("collect-dataset").value = state.filledCollectConfig.datasetName;
        if ($("collect-task")) $("collect-task").value = state.filledCollectConfig.taskPrompt;
        if ($("vla-prompt")) $("vla-prompt").value = state.filledCollectConfig.taskPrompt;
      }

      refreshCameraImagesOnce(false);
      if (target === "teaching") {
        refreshWaypoints();
        refreshLinearTrajectories();
      }
      if (target === "collect") {
        validateCollectSpecUI();
        refreshEpisodes();
        setTimeout(() => renderLiveCollectWaveform(), 40);
      }
      if (target === "plugins") {
        refreshPlugins();
      }
      if (target === "vla") loadVLAConfigs();
      updateAllStickViews();
      refreshStageSizes();
    });
  });
}

function startPollingFallback() {
  if (state.pollTimer) return;
  state.pollTimer = setInterval(async () => {
    const res = await apiFetch("/api/workflows/status");
    if (res.ok) applyWorkflowStatus(res.data);
  }, 1500);
}

function stopPollingFallback() {
  if (state.pollTimer) {
    clearInterval(state.pollTimer);
    state.pollTimer = null;
  }
}

function connectTelemetryBus() {
  const pill = $("ws-pill");
  createStudioSocket({
    onOpen: () => {
      stopPollingFallback();
      if (pill) {
        pill.innerHTML = '<span class="status-dot dot-ok"></span>WS Connected';
        pill.className = "sb-item";
      }
      clearAnomalyToast();
    },
    onClose: () => {
      if (pill) {
        pill.innerHTML = '<span class="status-dot dot-warn"></span>WS Fallback';
        pill.className = "sb-item sb-item-warn";
      }
      startPollingFallback();
    },
    onOverlay: (ov) => {
      if (Array.isArray(ov.waypoints)) state.waypoints = ov.waypoints;
      if (Array.isArray(ov.vla_chunk)) state.vlaChunk = ov.vla_chunk;
      if (ov.overlay_badge !== undefined) state.overlayBadge = ov.overlay_badge || "";
      updateAllStickViews();
    },
    onTelemetry: (msg) => {
      if (msg.mode) setModeUI(msg.mode);
      const prevWf = state.activeWorkflow;
      const wf = msg.active_workflow || "idle";
      state.activeWorkflow = wf;
      if ((prevWf === "collect" || prevWf === "linear_teach") && wf === "idle") {
        refreshEpisodes();
        refreshLinearTrajectories();
      }
      if (prevWf !== wf) {
        updateTeleopButtonsUI();
        syncArmControlMutexUI();
        renderReadinessState();
      }
      const wfPill = $("workflow-pill");
      if (wfPill) {
        wfPill.textContent = `WORKFLOW: ${String(wf).toUpperCase()}`;
        wfPill.className = "sb-item";
      }
      if (typeof msg.linked_arms === "boolean" && msg.linked_arms !== state.linkedArms) {
        state.linkedArms = msg.linked_arms;
        syncArmControlMutexUI();
        renderReadinessState();
      }
      if (msg.safety) updateSafetyUI(msg.safety);
      if (msg.connection_status) updateConnectionBadge(msg.connection_status);

      const isDraggingIk = Boolean(
        stageViews.leader?.isDraggingIk?.() || stageViews.follower?.isDraggingIk?.()
      );
      const shouldSyncSliders =
        !isDraggingIk &&
        (wf === "teaching_replay" ||
          wf === "demo_routine" ||
          wf === "collect" ||
          wf === "calibrate" ||
          (wf === "teleop" && msg.mode === "real") ||
          msg.safety?.sync_direction === "real_to_sim" ||
          msg.safety?.sync_direction === "teleop_bidirectional");
      if (!isDraggingIk) {
        updateJointGauges(msg.leader_joints, msg.follower_joints, shouldSyncSliders);
      }
      pushLiveCollectFrame(msg.leader_joints, msg.follower_joints, msg.workflow_status);
      if (msg.workflow_status) {
        const colProg = $("collect-progress-text");
        if (colProg) {
          if (wf === "collect") {
            const stTxt = msg.workflow_status.collect_stage_text
              ? ` · ${msg.workflow_status.collect_stage_text}`
              : "";
            colProg.textContent = `采集中: ${msg.workflow_status.current_episode || 0}/${msg.workflow_status.target_episodes || 0} 回合 (${msg.workflow_status.current_step || 0} 帧)${stTxt}`;
          } else if (
            msg.workflow_status.collect_stage === "completed" &&
            msg.workflow_status.collect_stage_text
          ) {
            colProg.textContent = msg.workflow_status.collect_stage_text;
          } else {
            colProg.textContent = "待机（未在采集）";
          }
        }
        const colLive = $("collect-live-actions");
        if (colLive) colLive.hidden = wf !== "collect";
      }
      if (Array.isArray(msg.log_tail)) renderLogs(msg.log_tail);
      if (msg.calibration) renderCalibrationState(msg.calibration);
      if (wf !== "idle") scheduleCameraLoop();
    },
  });
}

function bindActions() {
  // Left Sidebar Collapse
  $("btn-toggle-sidebar")?.addEventListener("click", () => {
    const shell = $("studio-shell");
    if (!shell) return;
    shell.classList.toggle("sidebar-collapsed");
    setTimeout(refreshStageSizes, 160);
  });

  // Bottom-Left Sidebar Light / Dark Theme Switcher (Synchronized with /sim-viewport)
  $("btn-toggle-theme")?.addEventListener("click", () => {
    const nextTheme = state.theme === "dark" ? "light" : "dark";
    if (popupMutex) {
      popupMutex.broadcastTheme(nextTheme);
    } else {
      updateThemeUI(nextTheme);
    }
  });

  // Global 3D Stage Dock Manual Collapse/Expand (with User Memory)
  $("btn-toggle-stage-dock")?.addEventListener("click", () => {
    if (state.popupActive) {
      popupMutex?.openPopupWindow();
      return;
    }
    const nextCollapsed = !state.stageCollapsed;
    state.userStageCollapsed = nextCollapsed;
    applyStageVisibility(nextCollapsed);
  });

  // Pop-out Standalone Virtual Hardware Window (/sim-viewport)
  $("btn-popout-stage")?.addEventListener("click", () => {
    popupMutex?.openPopupWindow();
  });

  $("btn-recall-stage")?.addEventListener("click", () => {
    popupMutex?.recallPopupWindow();
  });

  // Toast Dismiss
  $("btn-dismiss-toast")?.addEventListener("click", () => {
    const strip = $("anomaly-toast-strip");
    if (strip) strip.hidden = true;
  });

  // Hardware Guidance Toggle
  $("btn-toggle-hw-guidance")?.addEventListener("click", () => {
    const list = $("hw-guidance-list");
    const btn = $("btn-toggle-hw-guidance");
    if (!list) return;
    list.hidden = !list.hidden;
    if (btn) btn.textContent = list.hidden ? "展开提示" : "收起提示";
  });

  // Floating Console Drawer
  const toggleLogDrawer = (forceOpen) => {
    const dock = $("log-dock");
    const btn = $("btn-toggle-log-dock");
    if (!dock) return;
    const expanded =
      typeof forceOpen === "boolean"
        ? (dock.classList.toggle("expanded", forceOpen), forceOpen)
        : dock.classList.toggle("expanded");
    if (btn) btn.textContent = expanded ? "Console ▼" : "Console ▲";
  };

  $("btn-toggle-log-dock")?.addEventListener("click", () => toggleLogDrawer());
  $("btn-close-log-dock")?.addEventListener("click", () => toggleLogDrawer(false));

  // Plugin & Open-Source Dataset Workbench Actions
  $("btn-plugin-demo-drawer")?.addEventListener("click", () => {
    playDatasetTrajectoryOnPluginStage(
      "huanianxia/b601rs_open_the_drawer_put_the_block_inside_then_close_the_drawer_20261116_001"
    );
  });

  $("btn-stop-dataset-replay")?.addEventListener("click", () => {
    stopPluginReplayTimer();
  });

  $("btn-plugin-servo-diag")?.addEventListener("click", () => {
    showServoDiagnosticsOnPluginStage();
  });

  // Global Safety Controls
  $("btn-global-estop")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/safety/estop", {
      method: "POST",
      body: JSON.stringify({ reason: "Topbar E-Stop Button" }),
    });
    if (res.ok) updateSafetyUI(res.data);
  });

  $("btn-global-reset")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/safety/reset", { method: "POST" });
    if (res.ok) updateSafetyUI(res.data);
  });

  $("btn-toggle-torque")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/safety/torque", {
      method: "POST",
      body: JSON.stringify({ enabled: !state.torqueEnabled }),
    });
    if (res.ok && res.data) {
      updateSafetyUI(res.data);
      if (res.data.message) {
        showFeedbackToast(res.data.message, res.data.hardware_applied || state.mode !== "real" ? "ok" : "warn");
      }
    } else if (res.detail) {
      showFeedbackToast(res.detail, "warn");
    }
  });

  $("readiness-pill")?.addEventListener("click", async () => {
    if (state.estopLatched) {
      const res = await apiFetch("/api/safety/reset", { method: "POST" });
      if (res.ok) showFeedbackToast("急停已解除", "ok");
      return;
    }
    if (state.mode !== "real") {
      showFeedbackToast("当前为 dry_run 仿真模式；连接真机后切到 real/auto 可切换实时监听。", "info");
      return;
    }
    const nextDir = state.syncDirection === "off" ? "real_to_sim" : "off";
    const res = await apiFetch("/api/sim/sync_mode", {
      method: "POST",
      body: JSON.stringify({ direction: nextDir, safety_confirmed: false }),
    });
    if (res.ok) {
      state.syncDirection = nextDir;
      renderReadinessState();
      showFeedbackToast(
        nextDir === "real_to_sim"
          ? "已开启真机 30Hz 实时监听（手掰从臂 → 3D 实时跟随）"
          : "已暂停真机监听并释放串口（可供外部工具使用）",
        nextDir === "real_to_sim" ? "ok" : "info"
      );
    } else {
      showAnomalyToast(res.error || "切换真机监听状态失败");
    }
  });

  $("btn-readiness-action")?.addEventListener("click", async () => {
    const action = $("btn-readiness-action")?.dataset.actionType || "tab_sim";
    if (action === "reset_estop") {
      await apiFetch("/api/safety/reset", { method: "POST" });
      showFeedbackToast("急停已解除", "ok");
    } else if (action === "enable_real_to_sim") {
      const res = await apiFetch("/api/sim/sync_mode", {
        method: "POST",
        body: JSON.stringify({ direction: "real_to_sim", safety_confirmed: false }),
      });
      if (res.ok) {
        state.syncDirection = "real_to_sim";
        renderReadinessState();
        showFeedbackToast("已开启真机→仿真实时同步，现在掰动真机会实时带动 3D 画面", "ok");
      } else {
        showAnomalyToast(res.error || "开启实时同步失败");
      }
    } else if (action === "tab_teleop") {
      document.querySelector('.tab-btn[data-tab="teleop"]')?.click();
    } else if (action === "tab_teaching") {
      document.querySelector('.tab-btn[data-tab="teaching"]')?.click();
    } else if (action === "tab_dashboard") {
      document.querySelector('.tab-btn[data-tab="dashboard"]')?.click();
    } else {
      document.querySelector('.tab-btn[data-tab="sim"]')?.click();
    }
  });

  $("btn-topbar-teleop")?.addEventListener("click", async () => {
    if (state.activeWorkflow === "teleop") {
      const res = await apiFetch("/api/workflows/teleop/stop", { method: "POST" });
      if (res.ok) {
        state.activeWorkflow = "idle";
        updateTeleopButtonsUI();
        showFeedbackToast("已停止遥操作（从臂已卸力，3D 监听保持在线）", "info");
      } else {
        showAnomalyToast(res.error || "停止遥操作失败");
      }
      return;
    }
    const fps = Number($("teleop-fps")?.value || 60);
    const res = await apiFetch("/api/workflows/teleop/start", {
      method: "POST",
      body: JSON.stringify({ fps, display_data: false, use_cameras: false }),
    });
    if (res.ok) {
      state.activeWorkflow = "teleop";
      updateTeleopButtonsUI();
      showFeedbackToast("全局遥操作已开启：手掰主臂即可驱动从臂", "ok");
    } else {
      showAnomalyToast(res.error || res.data?.detail || "启动遥操作失败");
    }
  });

  $("btn-quick-rest")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/teaching/pose", {
      method: "POST",
      body: JSON.stringify({ pose: "rest", duration_s: 1.5 }),
    });
    if (res.ok) {
      showFeedbackToast("正在缓慢回到蜷缩休眠位并自动卸力", "ok");
    } else {
      showAnomalyToast(res.error || res.data?.detail || "无法执行休眠动作");
    }
  });

  $("mode-select")?.addEventListener("change", async (ev) => {
    const mode = ev.target.value;
    const res = await apiFetch("/api/mode", {
      method: "POST",
      body: JSON.stringify({ mode }),
    });
    if (res.ok) {
      setModeUI(res.data.mode);
      clearAnomalyToast();
      appendLocalLog(`模式已切换为 ${res.data.mode} (请求模式: ${mode})`);
    } else {
      ev.target.value = state.mode;
      showAnomalyToast(
        res.data?.detail || "未检测到双臂 SO-101 硬件，无法切入 REAL 真机模式（已保持 DRY-RUN 仿真）。"
      );
    }
  });

  $("btn-probe-hardware")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/hardware/probe");
    if (res.ok) {
      renderHardwareProbe(res.data);
      refreshCameraImagesOnce();
      appendLocalLog("已完成硬件重新探测。");
    }
  });

  $("btn-auto-detect-devices")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/hardware/config", {
      method: "POST",
      body: JSON.stringify({ auto_detect: true, persist_env: true }),
    });
    if (res.ok) {
      renderHardwareProbe(res.data);
      refreshCameraImagesOnce();
      appendLocalLog("已自动识别并保存主从臂串口与摄像头配置。");
    }
  });

  $("btn-save-device-config")?.addEventListener("click", async () => {
    const follower_port = $("select-follower-port")?.value || "";
    const leader_port = $("select-leader-port")?.value || "";
    const front_camera_id = $("select-front-camera")?.value || "none";
    const wrist_camera_id = $("select-wrist-camera")?.value || "none";
    const res = await apiFetch("/api/hardware/config", {
      method: "POST",
      body: JSON.stringify({
        follower_port,
        leader_port,
        front_camera_id,
        wrist_camera_id,
        persist_env: true,
      }),
    });
    if (res.ok) {
      renderHardwareProbe(res.data);
      refreshCameraImagesOnce();
      if ($("device-config-status")) {
        $("device-config-status").textContent = "已应用并保存到 .env";
      }
      appendLocalLog("已保存主从臂及摄像头配置到 .env。");
    } else if ($("device-config-status")) {
      $("device-config-status").textContent = `保存失败: ${res.data?.detail || "参数冲突"}`;
    }
  });

  $("btn-swap-arms")?.addEventListener("click", () => {
    const fSel = $("select-follower-port");
    const lSel = $("select-leader-port");
    if (!fSel || !lSel) return;
    [fSel.value, lSel.value] = [lSel.value, fSel.value];
  });

  $("btn-swap-cameras")?.addEventListener("click", () => {
    const fCam = $("select-front-camera");
    const wCam = $("select-wrist-camera");
    if (!fCam || !wCam) return;
    [fCam.value, wCam.value] = [wCam.value, fCam.value];
  });

  // Calibration Controls
  $("calibrate-target")?.addEventListener("change", () => updateAllStickViews());

  $("btn-start-calibrate")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const target = $("calibrate-target")?.value || "follower";
    const port =
      target === "follower"
        ? $("select-follower-port")?.value || null
        : $("select-leader-port")?.value || null;
    const res = await apiFetch("/api/workflows/calibrate/start", {
      method: "POST",
      body: JSON.stringify({ target, port }),
    });
    if (res.ok) {
      const st = await apiFetch("/api/workflows/status");
      if (st.ok) applyWorkflowStatus(st.data);
      showFeedbackToast("已启动标定向导，请按下方提示操作", "ok");
    } else {
      showAnomalyToast(res.error || res.data?.detail || "启动标定失败");
    }
  });

  $("btn-calibrate-enter")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/calibrate/input", {
      method: "POST",
      body: JSON.stringify({ value: "" }),
    });
    if (res.ok) {
      const st = await apiFetch("/api/workflows/status");
      if (st.ok) applyWorkflowStatus(st.data);
    } else {
      showAnomalyToast(res.error || res.data?.detail || "标定确认失败");
    }
  });

  $("btn-calibrate-recal")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/calibrate/input", {
      method: "POST",
      body: JSON.stringify({ value: "c" }),
    });
    if (res.ok) {
      const st = await apiFetch("/api/workflows/status");
      if (st.ok) applyWorkflowStatus(st.data);
      showFeedbackToast("已选择重新标定", "info");
    } else {
      showAnomalyToast(res.error || res.data?.detail || "重新标定指令失败");
    }
  });

  $("btn-stop-calibrate")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/calibrate/stop", { method: "POST" });
    if ($("calibrate-status")) $("calibrate-status").textContent = "标定已停止。";
    const st = await apiFetch("/api/workflows/status");
    if (st.ok) applyWorkflowStatus(st.data);
  });

  $("btn-save-rest-pose")?.addEventListener("click", async () => {
    const r = await apiFetch("/api/workflows/teaching/save_pose", {
      method: "POST",
      body: JSON.stringify({ target: "rest" }),
    });
    if (r.ok) appendLocalLog("已保存当前姿态至 rest_pose.json");
  });

  $("btn-cal-auto-home")?.addEventListener("click", async () => {
    const calTarget = $("calibrate-target")?.value || "follower";
    const home = Object.fromEntries(JOINTS.map((j) => [j.key, j.def]));
    const res = await apiFetch("/api/sim/joints", {
      method: "POST",
      body: JSON.stringify({ arm: calTarget, joints: home }),
    });
    if (res.ok && res.data) {
      updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
      showFeedbackToast("已自动对齐官方 URDF 中位 (0°)", "ok");
    }
  });

  $("btn-cal-auto-sweep")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/calibrate/input", {
      method: "POST",
      body: JSON.stringify({ value: "sweep" }),
    });
    if (res.ok) {
      showFeedbackToast("正在自动扫查 6 轴限位范围...", "info");
    }
  });

  async function toggleLinkedArms(nextLinked) {
    const res = await apiFetch("/api/sim/linked", {
      method: "POST",
      body: JSON.stringify({ linked: Boolean(nextLinked) }),
    });
    if (res.ok && res.data) {
      state.linkedArms = Boolean(res.data.linked_arms);
      syncArmControlMutexUI();
      showFeedbackToast(
        state.linkedArms
          ? "主从联动已开启：请操控主臂，从臂滑块与红圈已互斥锁定"
          : "主从联动已断开：主臂与从臂现可独立控制",
        state.linkedArms ? "ok" : "info"
      );
    }
  }

  $("chk-sim-linked")?.addEventListener("change", (ev) => {
    toggleLinkedArms(ev.target.checked);
  });

  $("btn-teleop-link-toggle")?.addEventListener("click", () => {
    toggleLinkedArms(!state.linkedArms);
  });

  // Teleop & WASD + Q/E
  $("btn-start-teleop")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const fps = Number($("teleop-fps")?.value || 60);
    const res = await apiFetch("/api/workflows/teleop/start", {
      method: "POST",
      body: JSON.stringify({ fps }),
    });
    if (res.ok) {
      state.activeWorkflow = "teleop";
      state.linkedArms = true;
      updateTeleopButtonsUI();
      syncArmControlMutexUI();
      scheduleCameraLoop();
      showFeedbackToast(`遥操作已启动 (${fps} FPS · 主从联动)`, "ok");
    }
  });

  $("btn-stop-teleop")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teleop/stop", { method: "POST" });
    state.activeWorkflow = "idle";
    updateTeleopButtonsUI();
    syncArmControlMutexUI();
    showFeedbackToast("遥操作已停止", "info");
  });

  $("btn-reset-joints")?.addEventListener("click", async () => {
    const home = Object.fromEntries(JOINTS.map((j) => [j.key, j.def]));
    const res = await apiFetch("/api/sim/joints", {
      method: "POST",
      body: JSON.stringify({ arm: "both", joints: home }),
    });
    if (res.ok) {
      updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
      refreshCameraImagesOnce();
      showFeedbackToast("主从双臂已归中位", "info");
    }
  });

  document.querySelectorAll(".btn-dpad").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const key = btn.dataset.key;
      const res = await apiFetch("/api/workflows/control/key", {
        method: "POST",
        body: JSON.stringify({ key, step_mm: 10.0 }),
      });
      if (res.ok && res.data?.follower_joints) {
        updateJointGauges(
          res.data.leader_joints || state.leaderJoints,
          res.data.follower_joints,
          true
        );
        if (res.data.clamped) showBoundaryToast(res.data.boundary_message);
        if (isTeleopPanelVisible()) refreshCameraImagesOnce();
      }
    });
  });

  $("chk-wasd-enable")?.addEventListener("change", (ev) => {
    // Blur checkbox immediately so space/WASD/QE work right away on every tab
    ev.target.blur();
  });

  document.addEventListener("keydown", async (ev) => {
    if (!$("chk-wasd-enable")?.checked) return;
    const activeEl = document.activeElement;
    const isTextInput =
      activeEl &&
      (activeEl.tagName === "TEXTAREA" ||
        activeEl.tagName === "SELECT" ||
        activeEl.isContentEditable ||
        (activeEl.tagName === "INPUT" &&
          !["checkbox", "radio", "range", "button", "submit"].includes(
            (activeEl.type || "").toLowerCase()
          )));
    if (isTextInput) {
      return;
    }
    const k = ev.key.toLowerCase();
    if (!["w", "a", "s", "d", "q", "e", "r", "f", "j", "k", "u", "i", "c", "v", "m", " "].includes(k)) {
      return;
    }
    ev.preventDefault();
    const stepMm = ["q", "e", "r", "f"].includes(k) ? 15.0 : 12.0;
    const res = await apiFetch("/api/workflows/control/key", {
      method: "POST",
      body: JSON.stringify({ key: k, step_mm: stepMm }),
    });
    if (res.ok && res.data?.follower_joints) {
      updateJointGauges(
        res.data.leader_joints || state.leaderJoints,
        res.data.follower_joints,
        true
      );
      if (res.data.clamped) showBoundaryToast(res.data.boundary_message);
      if (isTeleopPanelVisible()) refreshCameraImagesOnce();
    }
  });

  // Simulation Page Controls
  $("btn-apply-sync-mode")?.addEventListener("click", async () => {
    const direction = $("sim-sync-direction")?.value || "off";
    const safety_confirmed = Boolean($("chk-sim-safety")?.checked);
    const res = await apiFetch("/api/sim/sync_mode", {
      method: "POST",
      body: JSON.stringify({ direction, safety_confirmed }),
    });
    const el = $("sim-sync-status");
    if (res.ok && el) {
      el.textContent = `同步: ${res.data.direction} ${res.data.note || ""}`;
      showFeedbackToast(`真机同步模式已切换为: ${res.data.direction}`, "ok");
    } else if (el) {
      el.textContent = `切换失败: ${res.data?.detail || "请检查安全确认与硬件连接"}`;
      showFeedbackToast(res.data?.detail || "同步切换失败", "warn");
    }
  });

  $("chk-sim-ik-drag")?.addEventListener("change", () => {
    syncArmControlMutexUI();
    showFeedbackToast(
      $("chk-sim-ik-drag")?.checked ? "已启用 3D 红圈 IK 拖拽" : "已关闭 3D 红圈 IK 拖拽",
      "info"
    );
  });

  $("btn-apply-ik")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const x_mm = Number($("ik-x")?.value || 391);
    const y_mm = Number($("ik-y")?.value || 0);
    const z_mm = Number($("ik-z")?.value || 226);
    const gripper = Number($("ik-grip")?.value || 20);
    const targetArm = state.linkedArms ? "leader" : ($("sim-ik-arm-select")?.value || "follower");
    const res = await apiFetch("/api/sim/ik", {
      method: "POST",
      body: JSON.stringify({ arm: targetArm, x_mm, y_mm, z_mm, gripper }),
    });
    if (res.ok && res.data) {
      updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
      showFeedbackToast(`IK 逆解完成 (${targetArm === "leader" ? "主臂" : "从臂"})`, "ok");
    } else {
      showFeedbackToast(res.error || res.data?.detail || "目标坐标超出物理工作空间", "warn");
    }
  });

  $("btn-sim-leader-home")?.addEventListener("click", async () => {
    const home = Object.fromEntries(JOINTS.map((j) => [j.key, j.def]));
    const res = await apiFetch("/api/sim/joints", {
      method: "POST",
      body: JSON.stringify({ arm: "leader", joints: home, linked: Boolean($("chk-sim-linked")?.checked) }),
    });
    if (res.ok) updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
  });

  $("btn-sim-follower-home")?.addEventListener("click", async () => {
    const home = Object.fromEntries(JOINTS.map((j) => [j.key, j.def]));
    const res = await apiFetch("/api/sim/joints", {
      method: "POST",
      body: JSON.stringify({ arm: "follower", joints: home, linked: Boolean($("chk-sim-linked")?.checked) }),
    });
    if (res.ok) updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
  });

  // Teaching, Presets & Demo Routines
  $("btn-teach-freedrag")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const res = await apiFetch("/api/safety/torque", {
      method: "POST",
      body: JSON.stringify({ enabled: false }),
    });
    if (res.ok && res.data) {
      updateSafetyUI(res.data);
      showFeedbackToast(
        res.data.message || "已卸力并开启真机→3D实时同步，请手掰机械臂并点击记录示教点",
        "ok"
      );
    } else if (res.detail) {
      showFeedbackToast(res.detail, "warn");
    }
  });

  $("btn-linear-teach-start")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const name = ($("linear-teach-name")?.value || "demo_linear_01").trim();
    const duration_s = Math.max(2, Number($("linear-teach-duration")?.value || 30));
    const statusEl = $("linear-teach-status");
    const res = await apiFetch("/api/workflows/teaching/linear/start", {
      method: "POST",
      body: JSON.stringify({ name, duration_s, fps: 30 }),
    });
    if (res.ok) {
      state.activeWorkflow = "linear_teach";
      if (statusEl) {
        statusEl.textContent = `录制中 [${res.data.mode}]: ${res.data.name} (${duration_s}s) — 请操作主臂，完成后点「停止并保存」`;
      }
      showFeedbackToast(`已开始连续示教录制: ${res.data.name}`, "ok");
    } else {
      showAnomalyToast(res.error || "启动连续示教失败");
    }
  });

  $("btn-linear-teach-stop")?.addEventListener("click", async () => {
    const statusEl = $("linear-teach-status");
    const res = await apiFetch("/api/workflows/teaching/linear/stop", { method: "POST" });
    await refreshLinearTrajectories();
    if (res.ok) {
      const saved = res.data?.saved;
      if (statusEl) {
        statusEl.textContent = saved
          ? `已保存轨迹 ${saved.name} (${saved.frames}帧)，可点击「线性回放选中轨迹」。`
          : "已停止连续示教并保存，可点击「线性回放选中轨迹」。";
      }
      showFeedbackToast("已停止并保存连续轨迹", "ok");
    }
  });

  $("btn-linear-replay-start")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const sel = $("linear-traj-select");
    const statusEl = $("linear-teach-status");
    if (!sel || !sel.value) {
      showAnomalyToast("请先录制并选择一条连续轨迹");
      return;
    }
    let meta;
    try {
      meta = JSON.parse(sel.value);
    } catch {
      showAnomalyToast("轨迹选项解析失败");
      return;
    }
    const res = await apiFetch("/api/workflows/teaching/linear/replay", {
      method: "POST",
      body: JSON.stringify({ name: meta.name, episode: meta.episode || 0, fps: meta.fps || 30 }),
    });
    if (res.ok) {
      state.activeWorkflow = "linear_replay";
      if (statusEl) {
        statusEl.textContent = `线性回放中 [${res.data.mode}]: ${meta.name} (ep${meta.episode || 0})`;
      }
      showFeedbackToast(`开始线性回放: ${meta.name}`, "ok");
    } else {
      showAnomalyToast(res.error || "启动线性回放失败");
    }
  });

  $("btn-linear-replay-stop")?.addEventListener("click", async () => {
    const statusEl = $("linear-teach-status");
    await apiFetch("/api/workflows/teaching/linear/replay/stop", { method: "POST" });
    if (statusEl) statusEl.textContent = "已停止线性回放。";
    showFeedbackToast("已停止线性回放", "info");
  });

  $("btn-linear-refresh")?.addEventListener("click", async () => {
    await refreshLinearTrajectories();
    showFeedbackToast("已刷新连续轨迹列表", "info");
  });

  $("btn-teach-waypoint")?.addEventListener("click", async () => {
    const label = $("waypoint-label")?.value || "";
    const res = await apiFetch("/api/workflows/teaching/record_waypoint", {
      method: "POST",
      body: JSON.stringify({ label: label || null }),
    });
    if (res.ok) {
      await refreshWaypoints();
      showFeedbackToast(`已记录示教点 #${res.data?.count || ""}`, "ok");
    }
  });

  $("btn-replay-waypoints")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const res = await apiFetch("/api/workflows/teaching/replay", {
      method: "POST",
      body: JSON.stringify({ speed: 1.5, loop: false }),
    });
    if (res.ok) {
      state.activeWorkflow = "teaching_replay";
      showFeedbackToast("开始回放示教航点队列", "ok");
    } else {
      showAnomalyToast(res.error || res.data?.detail || "无法回放（请先记录至少 1 个示教点）");
    }
  });

  $("btn-clear-waypoints")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teaching/clear", { method: "POST" });
    await refreshWaypoints();
  });

  document.querySelectorAll(".btn-preset").forEach((btn) => {
    btn.addEventListener("click", async () => {
      ensureStageVisibleForAction();
      const res = await apiFetch("/api/workflows/teaching/pose", {
        method: "POST",
        body: JSON.stringify({ pose: btn.dataset.pose, duration_s: 1.2 }),
      });
      if (res.ok) {
        showFeedbackToast(`正在平滑移动到预设姿态: ${btn.dataset.pose}`, "info");
      } else {
        showAnomalyToast(res.error || res.data?.detail || "无法执行预设姿态");
      }
    });
  });

  document.querySelectorAll(".btn-routine").forEach((btn) => {
    btn.addEventListener("click", async () => {
      ensureStageVisibleForAction();
      const res = await apiFetch("/api/workflows/teaching/routine", {
        method: "POST",
        body: JSON.stringify({ routine: btn.dataset.routine }),
      });
      if (res.ok) {
        showFeedbackToast(`开始执行示范动作: ${btn.dataset.routine}`, "ok");
      } else {
        showAnomalyToast(res.error || res.data?.detail || "无法执行示范动作");
      }
    });
  });

  // Data Collection
  for (const id of ["collect-dataset", "collect-task", "collect-fps"]) {
    $(id)?.addEventListener("input", () => {
      state.filledCollectConfig = {
        datasetName: $("collect-dataset")?.value || "",
        taskPrompt: $("collect-task")?.value || "",
      };
      validateCollectSpecUI();
    });
  }

  $("btn-start-collect")?.addEventListener("click", async () => {
    if (state.activeWorkflow === "collect") return;
    const dataset_name = ($("collect-dataset")?.value || "").trim() || "yyzmiao_pick_cube_demo";
    if (dataset_name.startsWith("eval_") || /[\s\u4e00-\u9fff/\\]/.test(dataset_name)) {
      showAnomalyToast("数据集名称不能以 eval_ 开头，且只能包含英文字母、数字和下划线。");
      return;
    }
    ensureStageVisibleForAction();
    const single_task = ($("collect-task")?.value || "").trim() || "Pick up the red cube and place it into the tray";
    const num_episodes = Number($("collect-episodes")?.value || 5);
    const episode_time_s = Number($("collect-ep-time")?.value ?? 0);
    const reset_time_s = Number($("collect-reset-time")?.value ?? 3);
    const fps = Number($("collect-fps")?.value || 30);
    state.liveCollectFrames = [];
    switchInspectorToLive();
    const colProg = $("collect-progress-text");
    if (colProg) colProg.textContent = "正在初始化数据采集与双臂遥操作...";
    const res = await apiFetch("/api/workflows/collect/start", {
      method: "POST",
      body: JSON.stringify({ dataset_name, single_task, num_episodes, episode_time_s, reset_time_s, fps }),
    });
    if (res.ok) {
      state.activeWorkflow = "collect";
      updateTeleopButtonsUI();
      const colLive = $("collect-live-actions");
      if (colLive) colLive.hidden = false;
      updateAllStickViews();
      refreshCameraImagesOnce();
      showFeedbackToast(`已启动 LeRobotDataset v3.0 采集: ${dataset_name}`, "ok");
    } else {
      if (colProg) colProg.textContent = "待机（启动失败）";
      showAnomalyToast(res.error || res.data?.detail || "启动数据采集失败");
    }
  });

  $("btn-inspector-live")?.addEventListener("click", switchInspectorToLive);

  $("btn-stop-collect")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/collect/stop", { method: "POST" });
    state.activeWorkflow = "idle";
    const colLive = $("collect-live-actions");
    if (colLive) colLive.hidden = true;
    await refreshEpisodes();
    updateAllStickViews();
    showFeedbackToast("已停止采集并落盘数据集", "info");
  });

  $("btn-collect-next")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/collect/episode_action", {
      method: "POST",
      body: JSON.stringify({ action: "next" }),
    });
    if (res.ok) showFeedbackToast("已触发「完成本条」保存", "ok");
  });

  $("btn-collect-rerecord")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/collect/episode_action", {
      method: "POST",
      body: JSON.stringify({ action: "rerecord" }),
    });
    if (res.ok) showFeedbackToast("已丢弃本条，准备重录", "warn");
  });

  document.addEventListener("keydown", (ev) => {
    if (state.activeWorkflow !== "collect") return;
    const tag = (document.activeElement?.tagName || "").toUpperCase();
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (ev.key === "ArrowRight") {
      ev.preventDefault();
      $("btn-collect-next")?.click();
    } else if (ev.key === "ArrowLeft") {
      ev.preventDefault();
      $("btn-collect-rerecord")?.click();
    }
  });

  $("btn-inspector-replay")?.addEventListener("click", () => {
    if (!state.selectedEpisode) return;
    replayEpisodeOnStick(state.selectedEpisode.dataset, state.selectedEpisode.episodeIndex);
  });

  $("btn-inspector-delete")?.addEventListener("click", async () => {
    if (!state.selectedEpisode) return;
    const { dataset, episodeIndex } = state.selectedEpisode;
    const res = await apiFetch("/api/workflows/collect/episodes/delete", {
      method: "POST",
      body: JSON.stringify({ dataset_name: dataset, episode_index: episodeIndex }),
    });
    if (res.ok) {
      switchInspectorToLive();
      await refreshEpisodes();
      showFeedbackToast(`已删除 ${dataset} #${episodeIndex}`, "info");
    } else {
      showAnomalyToast(res.error || "删除失败");
    }
  });

  // Tab 5 Vertical Camera Strip Resizer (#collect-cam-resizer)
  (() => {
    const resizer = $("collect-cam-resizer");
    const strip = $("collect-cameras-strip");
    if (!resizer || !strip) return;
    let dragging = false;
    let startY = 0;
    let startH = 230;
    resizer.addEventListener("pointerdown", (ev) => {
      dragging = true;
      startY = ev.clientY;
      startH = strip.getBoundingClientRect().height || 230;
      resizer.classList.add("dragging");
      resizer.setPointerCapture?.(ev.pointerId);
      ev.preventDefault();
    });
    window.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const nextH = Math.max(150, Math.min(620, startH + (ev.clientY - startY)));
      strip.style.height = `${Math.round(nextH)}px`;
    });
    window.addEventListener("pointerup", () => {
      if (!dragging) return;
      dragging = false;
      resizer.classList.remove("dragging");
    });
  })();

  // Per-Camera Zoom (+/- / Wheel / Pan) & Full-View Lightbox Modal
  (() => {
    const zoomState = new Map();
    const getZ = (id) => {
      if (!zoomState.has(id)) zoomState.set(id, { scale: 1, tx: 0, ty: 0 });
      return zoomState.get(id);
    };
    const applyZ = (id) => {
      const img = $(id);
      if (!img) return;
      const z = getZ(id);
      if (z.scale <= 1.01) {
        z.scale = 1;
        z.tx = 0;
        z.ty = 0;
      }
      img.style.transform = `translate(${z.tx}px, ${z.ty}px) scale(${z.scale.toFixed(2)})`;
      img.parentElement?.classList.toggle("is-zoomed", z.scale > 1.01);
      document
        .querySelectorAll(`.cam-zoom-tools[data-cam-target="${id}"] .btn-cam-zoom-reset`)
        .forEach((b) => {
          b.textContent = `${Math.round(z.scale * 100)}%`;
        });
    };

    const modal = $("cam-lightbox-modal");
    const modalImg = $("cam-lightbox-img");
    const modalTitle = $("cam-lightbox-title");
    const openLightbox = (targetId, title) => {
      const srcImg = $(targetId);
      if (!modal || !modalImg || !srcImg) return;
      if (modalTitle) modalTitle.textContent = `${title || "相机实时视图"} (滚轮缩放 · 拖拽平移)`;
      modalImg.src = srcImg.src || "";
      const z = getZ("cam-lightbox-img");
      z.scale = 1;
      z.tx = 0;
      z.ty = 0;
      applyZ("cam-lightbox-img");
      modal.hidden = false;
    };
    const closeLightbox = () => {
      if (!modal || modal.hidden) return;
      modal.hidden = true;
      if (modalImg) modalImg.removeAttribute("src");
    };

    $("btn-cam-lightbox-close")?.addEventListener("click", closeLightbox);
    modal?.addEventListener("click", (ev) => {
      if (ev.target === modal) closeLightbox();
    });
    document.addEventListener("keydown", (ev) => {
      if (ev.key === "Escape" && modal && !modal.hidden) closeLightbox();
    });

    document.querySelectorAll(".cam-zoom-tools").forEach((bar) => {
      const targetId = bar.dataset.camTarget;
      const title = bar.dataset.camTitle || "";
      bar.querySelector(".btn-cam-zoom-in")?.addEventListener("click", () => {
        const z = getZ(targetId);
        z.scale = Math.min(4, z.scale + 0.25);
        applyZ(targetId);
      });
      bar.querySelector(".btn-cam-zoom-out")?.addEventListener("click", () => {
        const z = getZ(targetId);
        z.scale = Math.max(1, z.scale - 0.25);
        applyZ(targetId);
      });
      bar.querySelector(".btn-cam-zoom-reset")?.addEventListener("click", () => {
        const z = getZ(targetId);
        z.scale = 1;
        z.tx = 0;
        z.ty = 0;
        applyZ(targetId);
      });
      bar.querySelector(".btn-cam-expand")?.addEventListener("click", () => {
        openLightbox(targetId, title);
      });
    });

    document.querySelectorAll(".cam-zoom-viewport").forEach((vp) => {
      const targetId = vp.dataset.camTarget;
      const title = vp.dataset.camTitle || "";
      vp.addEventListener(
        "wheel",
        (ev) => {
          ev.preventDefault();
          const z = getZ(targetId);
          const delta = ev.deltaY < 0 ? 0.2 : -0.2;
          z.scale = Math.max(1, Math.min(4, z.scale + delta));
          applyZ(targetId);
        },
        { passive: false }
      );
      if (targetId !== "cam-lightbox-img") {
        vp.addEventListener("dblclick", () => openLightbox(targetId, title));
      }
      let panStart = null;
      vp.addEventListener("pointerdown", (ev) => {
        const z = getZ(targetId);
        if (z.scale <= 1.01) return;
        panStart = { x: ev.clientX - z.tx, y: ev.clientY - z.ty };
        vp.setPointerCapture?.(ev.pointerId);
        ev.preventDefault();
      });
      vp.addEventListener("pointermove", (ev) => {
        if (!panStart) return;
        const z = getZ(targetId);
        const maxOff = 220 * (z.scale - 1);
        z.tx = Math.max(-maxOff, Math.min(maxOff, ev.clientX - panStart.x));
        z.ty = Math.max(-maxOff, Math.min(maxOff, ev.clientY - panStart.y));
        applyZ(targetId);
      });
      const endPan = () => {
        panStart = null;
      };
      vp.addEventListener("pointerup", endPan);
      vp.addEventListener("pointercancel", endPan);
    });
  })();

  // Live Draggable XYZ + Gripper IK Sliders in Tab 3 (#sim-ik-card)
  (() => {
    let ikDragTimer = null;
    const flushLiveIk = async () => {
      ikDragTimer = null;
      const x_mm = Number($("ik-x")?.value || 391);
      const y_mm = Number($("ik-y")?.value || 0);
      const z_mm = Number($("ik-z")?.value || 226);
      const gripper = Number($("ik-grip")?.value || 20);
      const targetArm = state.linkedArms ? "leader" : ($("sim-ik-arm-select")?.value || "follower");
      const res = await apiFetch("/api/sim/ik", {
        method: "POST",
        body: JSON.stringify({ arm: targetArm, x_mm, y_mm, z_mm, gripper, clamp_to_workspace: true }),
      });
      if (res.ok && res.data) {
        updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
        if (state.mode === "real" && res.data.real_dispatched === false && !state.torqueEnabled) {
          const now = performance.now();
          if (!state._lastTorqueWarnTs || now - state._lastTorqueWarnTs > 4000) {
            state._lastTorqueWarnTs = now;
            showFeedbackToast("当前真机已卸力（仅动 3D 仿真），点顶栏「点击上力」即可驱动真机主/从臂", "warn");
          }
        }
      }
    };

    document.querySelectorAll(".ik-live-slider").forEach((slider) => {
      bindSliderDragLock(slider);
      slider.addEventListener("input", () => {
        state.sliderCooldownTs[slider.id] = performance.now() + 350;
        const axis = slider.dataset.axis;
        const numInput = $(`ik-${axis}`);
        if (numInput) numInput.value = slider.value;
        if (!ikDragTimer) {
          ikDragTimer = setTimeout(flushLiveIk, 45);
        }
      });
    });

    ["x", "y", "z", "grip"].forEach((axis) => {
      $(`ik-${axis}`)?.addEventListener("input", (ev) => {
        const sl = $(`ik-slider-${axis}`);
        if (sl) sl.value = ev.target.value;
      });
    });
  })();

  // Reusable Panel Split Resizers Across All Tabs (Col & Row)
  (() => {
    const bindColSplit = (resizerId, wrapId, leftId, rightId, minPct = 26, maxPct = 74) => {
      const resizer = $(resizerId);
      const wrap = $(wrapId);
      const leftEl = $(leftId);
      const rightEl = $(rightId);
      if (!resizer || !wrap || !leftEl || !rightEl) return;
      let dragging = false;
      resizer.addEventListener("pointerdown", (ev) => {
        dragging = true;
        resizer.classList.add("dragging");
        resizer.setPointerCapture?.(ev.pointerId);
        ev.preventDefault();
      });
      window.addEventListener("pointermove", (ev) => {
        if (!dragging) return;
        const rect = wrap.getBoundingClientRect();
        if (rect.width < 360) return;
        const pct = Math.max(minPct, Math.min(maxPct, ((ev.clientX - rect.left) / rect.width) * 100));
        leftEl.style.flex = `0 0 ${pct.toFixed(1)}%`;
        rightEl.style.flex = "1 1 0%";
      });
      window.addEventListener("pointerup", () => {
        if (!dragging) return;
        dragging = false;
        resizer.classList.remove("dragging");
      });
    };

    const bindRowHeightSplit = (resizerId, topId, minH = 110, maxH = 540) => {
      const resizer = $(resizerId);
      const topEl = $(topId);
      if (!resizer || !topEl) return;
      let dragging = false;
      let startY = 0;
      let startH = 0;
      resizer.addEventListener("pointerdown", (ev) => {
        dragging = true;
        startY = ev.clientY;
        startH = topEl.getBoundingClientRect().height || minH;
        resizer.classList.add("dragging");
        resizer.setPointerCapture?.(ev.pointerId);
        ev.preventDefault();
      });
      window.addEventListener("pointermove", (ev) => {
        if (!dragging) return;
        const nextH = Math.max(minH, Math.min(maxH, startH + (ev.clientY - startY)));
        topEl.style.height = `${Math.round(nextH)}px`;
        topEl.style.flex = `0 0 ${Math.round(nextH)}px`;
      });
      window.addEventListener("pointerup", () => {
        if (!dragging) return;
        dragging = false;
        resizer.classList.remove("dragging");
      });
    };

    bindColSplit("dashboard-split-resizer", "dashboard-split-wrap", "dashboard-left-card", "dashboard-right-card");
    bindColSplit("teleop-bottom-resizer", "teleop-bottom-wrap", "teleop-sliders-card", "teleop-wasd-card");
    bindColSplit("sim-arms-resizer", "sim-arms-wrap", "sim-leader-card", "sim-follower-card");
    bindRowHeightSplit("sim-ik-resizer", "sim-ik-card", 115, 360);
    bindRowHeightSplit("plugins-split-resizer", "plugins-top-card", 180, 620);
  })();

  // Tab 5 Horizontal Split Resizer (#collect-split-resizer)
  (() => {
    const resizer = $("collect-split-resizer");
    const wrap = $("collect-split-wrap");
    const leftCard = $("collect-episodes-card");
    const rightCard = $("data-spec-card");
    if (!resizer || !wrap || !leftCard || !rightCard) return;
    let dragging = false;
    resizer.addEventListener("pointerdown", (ev) => {
      dragging = true;
      resizer.classList.add("dragging");
      resizer.setPointerCapture?.(ev.pointerId);
      ev.preventDefault();
    });
    window.addEventListener("pointermove", (ev) => {
      if (!dragging) return;
      const rect = wrap.getBoundingClientRect();
      if (rect.width < 400) return;
      const pct = Math.max(30, Math.min(72, ((ev.clientX - rect.left) / rect.width) * 100));
      leftCard.style.flex = `0 0 ${pct.toFixed(1)}%`;
      rightCard.style.flex = "1 1 0%";
    });
    window.addEventListener("pointerup", () => {
      if (!dragging) return;
      dragging = false;
      resizer.classList.remove("dragging");
      if (state.selectedEpisode) {
        inspectEpisode(state.selectedEpisode.dataset, state.selectedEpisode.episodeIndex);
      } else {
        renderLiveCollectWaveform();
      }
    });
  })();

  $("btn-refresh-episodes")?.addEventListener("click", refreshEpisodes);
  $("btn-refresh-plugins")?.addEventListener("click", refreshPlugins);

  // VLA Dataset Linkage & 5-Step Prediction
  $("vla-dataset-select")?.addEventListener("change", (ev) => {
    const opt = ev.target.selectedOptions?.[0];
    const optTask = opt?.dataset?.task || "";
    if (optTask && $("vla-prompt")) {
      $("vla-prompt").value = optTask;
    }
    renderVLAConfigCards();
  });

  $("vla-prompt")?.addEventListener("input", renderVLAConfigCards);

  $("btn-vla-preview")?.addEventListener("click", async () => {
    ensureStageVisibleForAction();
    const endpoint = $("vla-endpoint")?.value || "ws://127.0.0.1:8000";
    const policy_name = $("vla-policy-name")?.value || "pi05_so101_custom";
    const task_prompt = $("vla-prompt")?.value || "Pick up the cube";
    const res = await apiFetch("/api/vla/mock_predict", {
      method: "POST",
      body: JSON.stringify({ endpoint, policy_name, task_prompt }),
    });
    if (res.ok && res.data) {
      if ($("vla-preview-output")) {
        $("vla-preview-output").textContent = JSON.stringify(res.data, null, 2);
      }
      const chunk = res.data.action_chunk || [];
      state.vlaChunk = chunk;
      if (state.vlaAnimTimer) clearInterval(state.vlaAnimTimer);
      if (chunk.length > 0) {
        let stepIdx = 0;
        state.vlaAnimTimer = setInterval(() => {
          if ($("panel-vla")?.hidden) return;
          const j = chunk[stepIdx % chunk.length];
          state.followerJoints = { ...state.followerJoints, ...j };
          state.overlayBadge = `预测步 ${j.step}/${chunk.length}`;
          stageViews.follower?.setState({
            joints: j,
            vlaChunk: chunk,
            overlayBadge: state.overlayBadge,
          });
          if ($("vla-stick-data")) {
            $("vla-stick-data").innerHTML = renderStickTelemetryHtml(j, `VLA 预测第 ${j.step} 步`);
          }
          stepIdx += 1;
        }, 420);
      }
    }
  });

  $("btn-clear-log-view")?.addEventListener("click", () => {
    state.localLogs = [];
    if ($("log-console")) $("log-console").textContent = "";
  });
}

async function bootstrap() {
  updateThemeUI(state.theme);
  initGlobalStageDock();
  initTabs();
  initJointSliders();
  bindActions();
  syncArmControlMutexUI();
  updateTeleopButtonsUI();
  validateCollectSpecUI();

  const [probeRes, statusRes] = await Promise.all([
    apiFetch("/api/hardware/probe"),
    apiFetch("/api/workflows/status"),
    refreshWaypoints(),
    refreshLinearTrajectories(),
    refreshEpisodes(),
    refreshPlugins(),
    loadVLAConfigs(),
  ]);

  if (probeRes.ok) renderHardwareProbe(probeRes.data);
  if (statusRes.ok) applyWorkflowStatus(statusRes.data);

  updateAllStickViews();
  refreshCameraImagesOnce();
  connectTelemetryBus();
}

document.addEventListener("DOMContentLoaded", bootstrap);
