import { StickArmCanvas, renderStickTelemetryHtml, computeStickFK } from "/static/stick_sim.js";

const JOINTS = [
  { key: "shoulder_pan", label: "#1 底座旋转 (shoulder_pan)", min: -110, max: 110, unit: "°", def: 0 },
  { key: "shoulder_lift", label: "#2 大臂俯仰 (shoulder_lift)", min: -100, max: 100, unit: "°", def: 0 },
  { key: "elbow_flex", label: "#3 小臂肘弯 (elbow_flex)", min: -97, max: 97, unit: "°", def: 0 },
  { key: "wrist_flex", label: "#4 手腕俯仰 (wrist_flex)", min: -95, max: 95, unit: "°", def: 0 },
  { key: "wrist_roll", label: "#5 手腕滚转 (wrist_roll)", min: -157, max: 163, unit: "°", def: 0 },
  { key: "gripper", label: "#6 末端夹爪 (gripper)", min: 0, max: 100, unit: "%", def: 20 },
];

const state = {
  mode: "dry_run",
  activeWorkflow: "idle",
  lastCalStage: "idle",
  linkedArms: false,
  torqueEnabled: true,
  estopLatched: false,
  leaderJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  followerJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  waypoints: [],
  vlaChunk: [],
  vlaAnimTimer: null,
  epReplayTimer: null,
  localLogs: [],
  ws: null,
  wsRetryMs: 1000,
  pollTimer: null,
  camTimer: null,
  jointSendTimer: null,
  pendingJoints: {},
  simSendTimers: { leader: null, follower: null },
  simPending: { leader: {}, follower: {} },
};

const stickViews = {
  cal: null,
  teleopLeader: null,
  teleopFollower: null,
  simLeader: null,
  simFollower: null,
  teach: null,
  collect: null,
  vla: null,
};

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
  try {
    const resp = await fetch(url, {
      headers: { "Content-Type": "application/json", ...(options.headers || {}) },
      ...options,
    });
    const text = await resp.text();
    let data = {};
    try {
      data = text ? JSON.parse(text) : {};
    } catch {
      data = { detail: text };
    }
    if (!resp.ok) {
      const detail =
        (data && data.detail && (typeof data.detail === "string" ? data.detail : data.detail.detail)) ||
        data.detail ||
        `HTTP ${resp.status}`;
      appendLocalLog(`请求提示 (${resp.status}): ${detail}`);
      return { ok: false, status: resp.status, data };
    }
    return { ok: true, status: resp.status, data };
  } catch (err) {
    appendLocalLog(`网络异常: ${err.message || err}`);
    return { ok: false, status: 0, data: { detail: String(err) } };
  }
}

function setModeUI(mode) {
  state.mode = mode;
  const badge = $("mode-badge");
  const banner = $("hardware-status-banner");
  if (badge) {
    if (mode === "real") {
      badge.textContent = "REAL 硬件模式";
      badge.className = "mode-badge mode-real";
    } else {
      badge.textContent = "DRY-RUN 模拟模式";
      badge.className = "mode-badge mode-dry";
    }
  }
  if (banner) {
    banner.className = mode === "real" ? "hw-banner hw-real" : "hw-banner hw-dry";
  }
}

function updateSafetyUI(safety) {
  if (!safety) return;
  state.torqueEnabled = Boolean(safety.torque_enabled);
  state.estopLatched = Boolean(safety.estop_latched);

  const tBadge = $("torque-badge");
  if (tBadge) {
    tBadge.textContent = state.torqueEnabled ? "⚡ 扭矩: 使能" : "💤 扭矩: 已释放";
    tBadge.className = state.torqueEnabled ? "pill pill-ok" : "pill pill-warn";
  }
  const eBadge = $("estop-badge");
  const btnReset = $("btn-global-reset");
  if (eBadge) {
    eBadge.textContent = state.estopLatched ? "🛑 急停已锁定 (ESTOP)" : "安全状态: 正常";
    eBadge.className = state.estopLatched ? "pill pill-warn" : "pill pill-info";
  }
  if (btnReset) {
    btnReset.hidden = !state.estopLatched;
  }
}

function updateConnectionBadge(conn) {
  const badge = $("teleop-connection-badge");
  if (!badge) return;
  if (state.mode === "real" && conn && conn.hardware_ready) {
    badge.textContent = `✓ 主从臂连接成功 (从臂 ${conn.follower_port} ← 主臂 ${conn.leader_port})`;
    badge.className = "conn-badge conn-ok";
  } else {
    badge.textContent = "模拟演练模式（主从联动就绪）";
    badge.className = "conn-badge conn-sim";
  }
}

function initStickCanvases() {
  stickViews.cal = new StickArmCanvas($("canvas-cal-stick"), {
    role: "follower",
    title: "标定中位比对 (虚影=官方前伸中位)",
    accent: "#06b6d4",
    showHomingRef: true,
  });
  stickViews.teleopLeader = new StickArmCanvas($("canvas-teleop-leader"), {
    role: "leader",
    title: "主臂 Leader (操控端)",
    accent: "#3b82f6",
  });
  stickViews.teleopFollower = new StickArmCanvas($("canvas-teleop-follower"), {
    role: "follower",
    title: "从臂 Follower (跟随端)",
    accent: "#06b6d4",
  });
  stickViews.simLeader = new StickArmCanvas($("canvas-sim-leader"), {
    role: "leader",
    title: "主臂 Leader 6-DOF (独立控制)",
    accent: "#3b82f6",
  });
  stickViews.simFollower = new StickArmCanvas($("canvas-sim-follower"), {
    role: "follower",
    title: "从臂 Follower 6-DOF (拖拽红圈=IK)",
    accent: "#10b981",
    enableIkDrag: true,
    onIkDrag: async (req) => {
      const res = await apiFetch("/api/sim/ik", {
        method: "POST",
        body: JSON.stringify(req),
      });
      if (res.ok && res.data) {
        updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
        if ($("ik-x")) $("ik-x").value = String(req.x_mm);
        if ($("ik-y")) $("ik-y").value = String(req.y_mm);
        if ($("ik-z")) $("ik-z").value = String(req.z_mm);
      }
    },
  });
  stickViews.teach = new StickArmCanvas($("canvas-teach-stick"), {
    role: "follower",
    title: "示教与表演轨迹 6-DOF 火柴棍",
    accent: "#06b6d4",
  });
  stickViews.collect = new StickArmCanvas($("canvas-collect-stick"), {
    role: "follower",
    title: "数据采集 / Episode 回放 6-DOF 火柴棍",
    accent: "#10b981",
  });
  stickViews.vla = new StickArmCanvas($("canvas-vla-stick"), {
    role: "follower",
    title: "VLA 5 步 Action Chunk 预测动画",
    accent: "#a855f7",
  });
}

function updateAllStickViews() {
  const l = state.leaderJoints;
  const f = state.followerJoints;
  const calTarget = $("calibrate-target")?.value || "follower";
  const calJoints = calTarget === "leader" ? l : f;

  if (stickViews.cal) {
    stickViews.cal.setState({ joints: calJoints, showHomingRef: true });
    if ($("cal-stick-data")) {
      $("cal-stick-data").innerHTML = renderStickTelemetryHtml(
        calJoints,
        `标定臂 (${calTarget}) 舵机角度与臂长`
      );
    }
  }
  if (stickViews.teleopLeader && !$("panel-teleop")?.hidden) {
    stickViews.teleopLeader.setState({ joints: l });
    if ($("teleop-leader-data")) {
      $("teleop-leader-data").innerHTML = renderStickTelemetryHtml(l, "主臂 Leader 数据");
    }
  }
  if (stickViews.teleopFollower && !$("panel-teleop")?.hidden) {
    stickViews.teleopFollower.setState({ joints: f });
    if ($("teleop-follower-data")) {
      $("teleop-follower-data").innerHTML = renderStickTelemetryHtml(f, "从臂 Follower 数据");
    }
    const tcp = computeStickFK(f).tcp;
    if ($("teleop-tcp-badge")) {
      $("teleop-tcp-badge").textContent = `TCP: X=${tcp.x.toFixed(0)} Y=${tcp.y.toFixed(0)} Z=${tcp.z.toFixed(0)} mm | Pitch=${tcp.pitchDeg.toFixed(1)}°`;
    }
  }
  if (stickViews.simLeader && !$("panel-sim")?.hidden) {
    stickViews.simLeader.setState({ joints: l });
    if ($("sim-leader-data")) {
      $("sim-leader-data").innerHTML = renderStickTelemetryHtml(l, "主臂 Leader 完整数据");
    }
  }
  if (stickViews.simFollower && !$("panel-sim")?.hidden) {
    stickViews.simFollower.setState({ joints: f });
    if ($("sim-follower-data")) {
      $("sim-follower-data").innerHTML = renderStickTelemetryHtml(f, "从臂 Follower 完整数据");
    }
  }
  if (stickViews.teach && !$("panel-teaching")?.hidden) {
    stickViews.teach.setState({ joints: f, waypoints: state.waypoints });
    if ($("teach-stick-data")) {
      $("teach-stick-data").innerHTML = renderStickTelemetryHtml(f, "示教实时关节与臂长数据");
    }
  }
  if (stickViews.collect && !$("panel-collect")?.hidden) {
    stickViews.collect.setState({
      joints: f,
      recActive: state.activeWorkflow === "collect",
    });
    if ($("collect-stick-data")) {
      $("collect-stick-data").innerHTML = renderStickTelemetryHtml(f, "采集状态 (observation.state) 数据");
    }
  }
  if (stickViews.vla && !$("panel-vla")?.hidden) {
    stickViews.vla.setState({ joints: f, vlaChunk: state.vlaChunk });
    if ($("vla-stick-data")) {
      $("vla-stick-data").innerHTML = renderStickTelemetryHtml(f, "VLA 预测关节与臂长数据");
    }
  }
}

function renderHardwareProbe(probe) {
  if (!probe) return;
  setModeUI(probe.active_mode || "dry_run");
  const modeSelect = $("mode-select");
  if (modeSelect && probe.requested_mode) {
    modeSelect.value = probe.requested_mode;
  }

  const title = $("hw-banner-title");
  const meta = $("hw-banner-meta");
  const chips = $("hw-missing-chips");
  const guidance = $("hw-guidance-list");

  const candPorts = (probe.serial_ports || []).filter((p) => p.is_robot_candidate);
  const cams = probe.cameras || [];
  if (title) {
    title.textContent = probe.hardware_ready
      ? "SO-101 硬件已就绪（支持真实控制与模拟演练）"
      : "未检测到双臂 SO-101 硬件 — 已自动降级至 DRY-RUN 模拟模式";
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
          .map(
            (c) =>
              `<div class="kv-item">[${c.id}] ${c.name} (${c.width}x${c.height} ${c.backend})</div>`
          )
          .join("")
      : '<div class="kv-item">未检测到物理相机（已启用合成 2D HUD）</div>';
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
    const autoNote = sel.auto_matched ? "（已自动匹配当前探测到的设备）" : "";
    statusEl.textContent = `当前配置: 从臂=${sel.follower_port || "-"} | 主臂=${sel.leader_port || "-"} | 前置相机=${sel.front_camera_id || "none"} | 腕部相机=${sel.wrist_camera_id || "none"} ${autoNote}`;
  }
}

function renderCalibrationFileBadges(calStatus) {
  const box = $("calibration-file-badges");
  if (!box || !calStatus) return;
  const fChip = calStatus.follower_calibrated
    ? '<span class="chip chip-ok">从臂标定文件: 已就绪</span>'
    : '<span class="chip chip-warn">从臂标定文件: 未标定</span>';
  const lChip = calStatus.leader_calibrated
    ? '<span class="chip chip-ok">主臂标定文件: 已就绪</span>'
    : '<span class="chip chip-warn">主臂标定文件: 未标定</span>';
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

function buildSlidersHtml(prefix) {
  return JOINTS.map((j) => {
    const pct = ((j.def - j.min) / (j.max - j.min)) * 100;
    return `
      <div class="joint-row">
        <div class="joint-head">
          <span>${j.label}</span>
          <span class="joint-values" id="${prefix}-val-${j.key}">${j.def}${j.unit}</span>
        </div>
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
        <div class="gauge-bar-track">
          <div class="gauge-bar-fill" id="${prefix}-bar-${j.key}" style="width:${pct}%"></div>
        </div>
      </div>
    `;
  }).join("");
}

function initJointSliders() {
  // 1. Teleop sliders
  const teleopBox = $("joints-container");
  if (teleopBox) {
    teleopBox.innerHTML = buildSlidersHtml("teleop");
    teleopBox.querySelectorAll(".joint-slider").forEach((slider) => {
      slider.addEventListener("input", (ev) => {
        const key = ev.target.dataset.joint;
        const val = Number(ev.target.value);
        state.leaderJoints[key] = val;
        state.pendingJoints[key] = val;
        updateAllStickViews();
        if (!state.jointSendTimer) {
          state.jointSendTimer = setTimeout(flushPendingJoints, 55);
        }
      });
    });
  }

  // 2. Independent Simulation page sliders (Leader & Follower separately)
  for (const arm of ["leader", "follower"]) {
    const box = $(`sim-${arm}-sliders`);
    if (!box) continue;
    box.innerHTML = buildSlidersHtml(`sim-${arm}`);
    box.querySelectorAll(".joint-slider").forEach((slider) => {
      slider.addEventListener("input", (ev) => {
        const key = ev.target.dataset.joint;
        const val = Number(ev.target.value);
        if (arm === "leader") state.leaderJoints[key] = val;
        else state.followerJoints[key] = val;
        state.simPending[arm][key] = val;
        updateAllStickViews();
        if (!state.simSendTimers[arm]) {
          state.simSendTimers[arm] = setTimeout(() => flushSimJoints(arm), 55);
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
  }
}

function updateJointGauges(leader, follower, syncSliders = false) {
  if (!leader || !follower) return;
  state.leaderJoints = leader;
  state.followerJoints = follower;

  for (const j of JOINTS) {
    const lv = Number(leader[j.key] ?? j.def);
    const fv = Number(follower[j.key] ?? j.def);

    // Teleop panel
    const tVal = $(`teleop-val-${j.key}`);
    if (tVal) tVal.textContent = `L:${lv.toFixed(1)}${j.unit} / F:${fv.toFixed(1)}${j.unit}`;
    const tBar = $(`teleop-bar-${j.key}`);
    if (tBar) {
      const pct = Math.max(0, Math.min(100, ((fv - j.min) / (j.max - j.min)) * 100));
      tBar.style.width = `${pct.toFixed(1)}%`;
    }
    if (syncSliders) {
      const sl = $(`teleop-slider-${j.key}`);
      if (sl && document.activeElement !== sl) sl.value = String(lv);
    }

    // Independent Simulation sliders
    for (const [arm, val] of [["leader", lv], ["follower", fv]]) {
      const vEl = $(`sim-${arm}-val-${j.key}`);
      if (vEl) vEl.textContent = `${val.toFixed(1)}${j.unit}`;
      const bEl = $(`sim-${arm}-bar-${j.key}`);
      if (bEl) {
        const pct = Math.max(0, Math.min(100, ((val - j.min) / (j.max - j.min)) * 100));
        bEl.style.width = `${pct.toFixed(1)}%`;
      }
      if (syncSliders) {
        const sEl = $(`sim-${arm}-slider-${j.key}`);
        if (sEl && document.activeElement !== sEl) sEl.value = String(val);
      }
    }
  }
  updateAllStickViews();
}

function isTeleopPanelVisible() {
  const panel = $("panel-teleop");
  return Boolean(panel && !panel.hidden);
}

function refreshCameraImagesOnce() {
  const ts = Date.now();
  const front = $("camera-preview-front");
  const wrist = $("camera-preview-wrist");
  if (front) front.src = `/api/cameras/front/frame?t=${ts}`;
  if (wrist) wrist.src = `/api/cameras/wrist/frame?t=${ts}`;
}

function scheduleCameraLoop() {
  if (state.camTimer) return;
  if (!isTeleopPanelVisible()) return;
  const intervalMs = state.mode === "real" ? 1200 : 450;
  const tick = () => {
    state.camTimer = null;
    if (state.activeWorkflow === "teleop" && isTeleopPanelVisible()) {
      refreshCameraImagesOnce();
      state.camTimer = setTimeout(tick, intervalMs);
    }
  };
  state.camTimer = setTimeout(tick, intervalMs);
}

function applyWorkflowStatus(statusObj) {
  if (!statusObj) return;
  const wf = statusObj.active_workflow || "idle";
  state.activeWorkflow = wf;
  const wfPill = $("workflow-pill");
  if (wfPill) {
    wfPill.textContent = `状态: ${wf}`;
    wfPill.className = wf === "idle" ? "pill pill-info" : "pill pill-ok";
  }
  if (statusObj.mode) setModeUI(statusObj.mode);
  const colProg = $("collect-progress-text");
  if (colProg) {
    colProg.textContent = `当前进度: ${statusObj.current_episode || 0} / ${statusObj.target_episodes || 0} 回合 (步数: ${statusObj.current_step || 0})`;
  }
  if (Array.isArray(statusObj.logs)) renderLogs(statusObj.logs);
  if (statusObj.calibration) renderCalibrationState(statusObj.calibration);
  if (wf !== "idle") scheduleCameraLoop();
  updateAllStickViews();
}

function validateCollectSpecUI() {
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
    badge.textContent = "规范校验: ✓ 符合团队标准";
    badge.className = "pill pill-ok";
  } else {
    badge.textContent = `规范提示: ${issues.join("；")}`;
    badge.className = "pill pill-warn";
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
    tbody.innerHTML = '<tr><td colspan="5" class="muted">暂无示教轨迹点，请点击「记录示教点」。</td></tr>';
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

async function replayEpisodeOnStick(dataset, episodeIndex) {
  if (state.epReplayTimer) {
    clearInterval(state.epReplayTimer);
    state.epReplayTimer = null;
  }
  const res = await apiFetch(`/api/workflows/collect/episodes/${encodeURIComponent(dataset)}/${episodeIndex}`);
  if (!res.ok || !Array.isArray(res.data?.frames) || res.data.frames.length === 0) {
    appendLocalLog("该 Episode 暂无可回放帧数据。");
    return;
  }
  const frames = res.data.frames;
  let idx = 0;
  appendLocalLog(`开始用火柴棍回放 ${dataset} #${episodeIndex} (${frames.length} 帧)...`);
  state.epReplayTimer = setInterval(() => {
    if (idx >= frames.length) {
      clearInterval(state.epReplayTimer);
      state.epReplayTimer = null;
      if (stickViews.collect) stickViews.collect.setState({ overlayBadge: "" });
      return;
    }
    const f = frames[idx];
    const st = f["observation.state"] || [];
    const jObj = {};
    JOINTS.forEach((jm, i) => {
      if (i < st.length) jObj[jm.key] = Number(st[i]);
    });
    state.followerJoints = { ...state.followerJoints, ...jObj };
    if (stickViews.collect) {
      stickViews.collect.setState({
        joints: state.followerJoints,
        overlayBadge: `▶ 回放 #${episodeIndex} (${idx + 1}/${frames.length})`,
      });
    }
    if ($("collect-stick-data")) {
      $("collect-stick-data").innerHTML = renderStickTelemetryHtml(
        state.followerJoints,
        `回放 ${dataset} #${episodeIndex} 帧 ${idx + 1}/${frames.length}`
      );
    }
    idx += 1;
  }, 60);
}

async function refreshEpisodes() {
  const res = await apiFetch("/api/workflows/collect/episodes");
  const tbody = $("episodes-tbody");
  if (!res.ok || !tbody) return;
  const eps = res.data.episodes || [];
  if (!eps.length) {
    tbody.innerHTML = '<tr><td colspan="6" class="muted">暂无已保存的 Episode。</td></tr>';
    return;
  }
  tbody.innerHTML = eps
    .map(
      (ep) => `<tr>
        <td>${ep.dataset}</td>
        <td>#${ep.episode_index}</td>
        <td>${ep.task}</td>
        <td>${ep.num_frames} / ${ep.fps}Hz</td>
        <td class="joint-values">${ep.path}</td>
        <td><button type="button" class="btn btn-xs btn-primary btn-ep-stick-replay" data-dataset="${ep.dataset}" data-ep="${ep.episode_index}">火柴棍回放</button></td>
      </tr>`
    )
    .join("");

  tbody.querySelectorAll(".btn-ep-stick-replay").forEach((btn) => {
    btn.addEventListener("click", () => {
      replayEpisodeOnStick(btn.dataset.dataset, Number(btn.dataset.ep));
    });
  });
}

async function refreshPlugins() {
  const res = await apiFetch("/api/plugins");
  const container = $("plugin-list");
  if (!res.ok || !container) return;
  const plugins = res.data.plugins || [];
  container.innerHTML = plugins
    .map((p) => {
      const availBadge = p.available
        ? '<span class="chip chip-ok">可用</span>'
        : '<span class="chip chip-warn">依赖未就绪</span>';
      const osBadge = p.os_supported
        ? '<span class="chip chip-ok">OS 支持</span>'
        : '<span class="chip chip-warn">需 Ubuntu/云端</span>';
      const statusBadge =
        p.status === "running"
          ? `<span class="chip chip-ok">运行中 (PID ${p.pid})</span>`
          : `<span class="chip">${p.status}</span>`;
      const hint = p.install_hint ? `<p class="muted mt-12">${p.install_hint}</p>` : "";
      return `
        <div class="card" data-plugin-id="${p.id}">
          <h3>${p.name}</h3>
          <div class="chip-row">${availBadge}${osBadge}${statusBadge}</div>
          <p class="muted mt-12">${p.description}</p>
          ${hint}
          <div class="btn-group mt-12">
            <button type="button" class="btn btn-primary btn-plugin-launch" data-id="${p.id}">启动插件</button>
            <button type="button" class="btn btn-danger btn-plugin-stop" data-id="${p.id}">停止</button>
          </div>
        </div>
      `;
    })
    .join("");

  container.querySelectorAll(".btn-plugin-launch").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const id = btn.dataset.id;
      const r = await apiFetch(`/api/plugins/${id}/launch`, {
        method: "POST",
        body: JSON.stringify({ args: {} }),
      });
      if (r.ok) appendLocalLog(`插件 ${id} 启动成功。`);
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

async function loadVLAConfigs() {
  const res = await apiFetch("/api/vla/configs");
  if (!res.ok) return;
  const notice = res.data.pi07_notice || {};
  const noticeEl = $("pi07-notice-card");
  if (noticeEl) {
    noticeEl.innerHTML = `
      <h3>${notice.title || "pi0.7 状态说明"}</h3>
      <p><strong>论文</strong>: ${notice.paper || "arXiv:2604.15483v2"} | <strong>权重状态</strong>: ${notice.status || "闭源未公开"}</p>
      <p class="muted">${notice.recommendation || ""}</p>
    `;
  }
  const grid = $("vla-configs-grid");
  if (grid) {
    const cfgs = res.data.configs || [];
    grid.innerHTML = cfgs
      .map(
        (c) => `
        <div class="card">
          <h3>${c.title}</h3>
          <div class="chip-row">
            <span class="chip">${c.path}</span>
            <span class="chip chip-ok">显存: ${c.vram}</span>
          </div>
          <p class="muted mt-12">适用平台: ${c.platform}</p>
          <pre class="code-box mt-12">${c.command}</pre>
        </div>
      `
      )
      .join("");
  }
}

function initTabs() {
  const buttons = document.querySelectorAll(".tab-btn");
  const panels = document.querySelectorAll(".tab-panel");
  buttons.forEach((btn) => {
    btn.addEventListener("click", () => {
      const target = btn.dataset.tab;
      buttons.forEach((b) => {
        const active = b === btn;
        b.classList.toggle("active", active);
        b.setAttribute("aria-selected", active ? "true" : "false");
      });
      panels.forEach((p) => {
        p.hidden = p.id !== `panel-${target}`;
      });
      if (target === "teleop") refreshCameraImagesOnce();
      if (target === "teaching") refreshWaypoints();
      if (target === "collect") {
        validateCollectSpecUI();
        refreshEpisodes();
      }
      if (target === "plugins") refreshPlugins();
      if (target === "vla") loadVLAConfigs();
      updateAllStickViews();
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

function connectWebSocket() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const url = `${proto}//${location.host}/ws/telemetry`;
  const pill = $("ws-pill");
  try {
    const ws = new WebSocket(url);
    state.ws = ws;

    ws.onopen = () => {
      state.wsRetryMs = 1000;
      stopPollingFallback();
      if (pill) {
        pill.textContent = "WS 实时连接";
        pill.className = "pill pill-ok";
      }
    };

    ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.mode) setModeUI(msg.mode);
        const wf = msg.active_workflow || "idle";
        state.activeWorkflow = wf;
        const wfPill = $("workflow-pill");
        if (wfPill) {
          wfPill.textContent = `状态: ${wf}`;
          wfPill.className = wf === "idle" ? "pill pill-info" : "pill pill-ok";
        }
        if (msg.safety) updateSafetyUI(msg.safety);
        if (msg.connection_status) updateConnectionBadge(msg.connection_status);
        const shouldSyncSliders =
          wf === "teaching_replay" ||
          wf === "demo_routine" ||
          msg.safety?.sync_direction === "real_to_sim";
        updateJointGauges(msg.leader_joints, msg.follower_joints, shouldSyncSliders);
        if (msg.workflow_status) {
          const colProg = $("collect-progress-text");
          if (colProg) {
            colProg.textContent = `当前进度: ${msg.workflow_status.current_episode || 0} / ${msg.workflow_status.target_episodes || 0} 回合 (步数: ${msg.workflow_status.current_step || 0})`;
          }
        }
        if (Array.isArray(msg.log_tail)) renderLogs(msg.log_tail);
        if (msg.calibration) renderCalibrationState(msg.calibration);
        if (wf !== "idle") scheduleCameraLoop();
      } catch {
        // Ignore malformed frame
      }
    };

    ws.onclose = () => {
      if (pill) {
        pill.textContent = "HTTP 轮询兜底";
        pill.className = "pill pill-warn";
      }
      startPollingFallback();
      const delay = state.wsRetryMs;
      state.wsRetryMs = Math.min(10000, state.wsRetryMs * 2);
      setTimeout(connectWebSocket, delay);
    };
    ws.onerror = () => {};
  } catch {
    startPollingFallback();
  }
}

function bindActions() {
  // Global safety bar
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
    if (res.ok) updateSafetyUI(res.data);
  });

  $("btn-quick-rest")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teaching/pose", {
      method: "POST",
      body: JSON.stringify({ pose: "rest", duration_s: 1.5 }),
    });
  });

  $("mode-select")?.addEventListener("change", async (ev) => {
    const mode = ev.target.value;
    const res = await apiFetch("/api/mode", {
      method: "POST",
      body: JSON.stringify({ mode }),
    });
    if (res.ok) {
      setModeUI(res.data.mode);
      appendLocalLog(`模式已切换为 ${res.data.mode}`);
    } else {
      ev.target.value = state.mode;
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
        $("device-config-status").textContent = "已应用并保存到 .env！";
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

  // Calibration controls
  $("calibrate-target")?.addEventListener("change", () => updateAllStickViews());

  $("btn-start-calibrate")?.addEventListener("click", async () => {
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
    }
  });

  $("btn-stop-calibrate")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/calibrate/stop", { method: "POST" });
    if ($("calibrate-status")) $("calibrate-status").textContent = "标定已停止。";
    const st = await apiFetch("/api/workflows/status");
    if (st.ok) applyWorkflowStatus(st.data);
  });

  $("btn-save-home-pose")?.addEventListener("click", async () => {
    const r = await apiFetch("/api/workflows/teaching/save_pose", {
      method: "POST",
      body: JSON.stringify({ target: "home" }),
    });
    if (r.ok) appendLocalLog("已保存当前姿态至 software/upright_pose.json");
  });

  $("btn-save-rest-pose")?.addEventListener("click", async () => {
    const r = await apiFetch("/api/workflows/teaching/save_pose", {
      method: "POST",
      body: JSON.stringify({ target: "rest" }),
    });
    if (r.ok) appendLocalLog("已保存当前姿态至 software/rest_pose.json");
  });

  // Teleop & WASD
  $("btn-start-teleop")?.addEventListener("click", async () => {
    const fps = Number($("teleop-fps")?.value || 60);
    const res = await apiFetch("/api/workflows/teleop/start", {
      method: "POST",
      body: JSON.stringify({ fps }),
    });
    if (res.ok) {
      state.activeWorkflow = "teleop";
      scheduleCameraLoop();
    }
  });

  $("btn-stop-teleop")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teleop/stop", { method: "POST" });
    state.activeWorkflow = "idle";
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
      }
    });
  });

  document.addEventListener("keydown", async (ev) => {
    if (!$("chk-wasd-enable")?.checked) return;
    if (["INPUT", "TEXTAREA", "SELECT"].includes(document.activeElement?.tagName || "")) return;
    const k = ev.key.toLowerCase();
    if (!["w", "a", "s", "d", "r", "f", "j", "k", "u", "i", "c", "v", "m", " "].includes(k)) return;
    ev.preventDefault();
    const res = await apiFetch("/api/workflows/control/key", {
      method: "POST",
      body: JSON.stringify({ key: k, step_mm: 10.0 }),
    });
    if (res.ok && res.data?.follower_joints) {
      updateJointGauges(
        res.data.leader_joints || state.leaderJoints,
        res.data.follower_joints,
        true
      );
    }
  });

  // Dedicated Simulation page controls
  $("btn-apply-sync-mode")?.addEventListener("click", async () => {
    const direction = $("sim-sync-direction")?.value || "off";
    const safety_confirmed = Boolean($("chk-sim-safety")?.checked);
    const res = await apiFetch("/api/sim/sync_mode", {
      method: "POST",
      body: JSON.stringify({ direction, safety_confirmed }),
    });
    const el = $("sim-sync-status");
    if (res.ok && el) {
      el.textContent = `当前同步状态: ${res.data.direction} ${res.data.note || ""}`;
    } else if (el) {
      el.textContent = `同步切换失败: ${res.data?.detail || "请检查安全确认与硬件连接"}`;
    }
  });

  $("chk-sim-ik-drag")?.addEventListener("change", (ev) => {
    if (stickViews.simFollower) stickViews.simFollower.setIkEnabled(ev.target.checked);
  });

  $("btn-apply-ik")?.addEventListener("click", async () => {
    const x_mm = Number($("ik-x")?.value || 196);
    const y_mm = Number($("ik-y")?.value || 0);
    const z_mm = Number($("ik-z")?.value || 245);
    const gripper = Number($("ik-grip")?.value || 20);
    const res = await apiFetch("/api/sim/ik", {
      method: "POST",
      body: JSON.stringify({ arm: "follower", x_mm, y_mm, z_mm, gripper }),
    });
    if (res.ok && res.data) {
      updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
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
  $("btn-teach-waypoint")?.addEventListener("click", async () => {
    const label = $("waypoint-label")?.value || "";
    const res = await apiFetch("/api/workflows/teaching/record_waypoint", {
      method: "POST",
      body: JSON.stringify({ label: label || null }),
    });
    if (res.ok) await refreshWaypoints();
  });

  $("btn-replay-waypoints")?.addEventListener("click", async () => {
    const res = await apiFetch("/api/workflows/teaching/replay", {
      method: "POST",
      body: JSON.stringify({ speed: 1.5, loop: false }),
    });
    if (res.ok) {
      state.activeWorkflow = "teaching_replay";
    }
  });

  $("btn-clear-waypoints")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teaching/clear", { method: "POST" });
    await refreshWaypoints();
  });

  document.querySelectorAll(".btn-preset").forEach((btn) => {
    btn.addEventListener("click", async () => {
      await apiFetch("/api/workflows/teaching/pose", {
        method: "POST",
        body: JSON.stringify({ pose: btn.dataset.pose, duration_s: 1.2 }),
      });
    });
  });

  document.querySelectorAll(".btn-routine").forEach((btn) => {
    btn.addEventListener("click", async () => {
      await apiFetch("/api/workflows/teaching/routine", {
        method: "POST",
        body: JSON.stringify({ routine: btn.dataset.routine }),
      });
    });
  });

  // Data Collection & Spec Validator
  for (const id of ["collect-dataset", "collect-task", "collect-fps"]) {
    $(id)?.addEventListener("input", validateCollectSpecUI);
  }

  $("btn-start-collect")?.addEventListener("click", async () => {
    const dataset_name = $("collect-dataset")?.value || "yyzmiao_pick_cube_demo";
    const single_task = $("collect-task")?.value || "Pick up the red cube and place it into the tray";
    const num_episodes = Number($("collect-episodes")?.value || 2);
    const fps = Number($("collect-fps")?.value || 30);
    const res = await apiFetch("/api/workflows/collect/start", {
      method: "POST",
      body: JSON.stringify({ dataset_name, single_task, num_episodes, fps }),
    });
    if (res.ok) {
      state.activeWorkflow = "collect";
      updateAllStickViews();
    }
  });

  $("btn-stop-collect")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/collect/stop", { method: "POST" });
    state.activeWorkflow = "idle";
    await refreshEpisodes();
    updateAllStickViews();
  });

  $("btn-refresh-episodes")?.addEventListener("click", refreshEpisodes);
  $("btn-refresh-plugins")?.addEventListener("click", refreshPlugins);

  // VLA Preview + Stick Animation
  $("btn-vla-preview")?.addEventListener("click", async () => {
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
          if (stickViews.vla) {
            stickViews.vla.setState({
              joints: j,
              vlaChunk: chunk,
              overlayBadge: `预测步 ${j.step}/${chunk.length}`,
            });
          }
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
  initTabs();
  initStickCanvases();
  initJointSliders();
  bindActions();
  validateCollectSpecUI();

  const [probeRes, statusRes] = await Promise.all([
    apiFetch("/api/hardware/probe"),
    apiFetch("/api/workflows/status"),
    refreshWaypoints(),
    refreshEpisodes(),
    refreshPlugins(),
    loadVLAConfigs(),
  ]);

  if (probeRes.ok) renderHardwareProbe(probeRes.data);
  if (statusRes.ok) applyWorkflowStatus(statusRes.data);

  updateAllStickViews();
  refreshCameraImagesOnce();
  connectWebSocket();
}

document.addEventListener("DOMContentLoaded", bootstrap);
