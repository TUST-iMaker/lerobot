const JOINTS = [
  { key: "shoulder_pan", label: "肩部水平 (shoulder_pan)", min: -180, max: 180, unit: "°", def: 0 },
  { key: "shoulder_lift", label: "肩部俯仰 (shoulder_lift)", min: -180, max: 180, unit: "°", def: -25 },
  { key: "elbow_flex", label: "肘部屈伸 (elbow_flex)", min: -180, max: 180, unit: "°", def: 45 },
  { key: "wrist_flex", label: "腕部俯仰 (wrist_flex)", min: -180, max: 180, unit: "°", def: 30 },
  { key: "wrist_roll", label: "腕部旋转 (wrist_roll)", min: -180, max: 180, unit: "°", def: 0 },
  { key: "gripper", label: "夹爪开合 (gripper)", min: 0, max: 100, unit: "%", def: 20 },
];

const state = {
  mode: "dry_run",
  activeWorkflow: "idle",
  leaderJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  followerJoints: Object.fromEntries(JOINTS.map((j) => [j.key, j.def])),
  localLogs: [],
  ws: null,
  wsRetryMs: 1000,
  pollTimer: null,
  camTimer: null,
  jointSendTimer: null,
  pendingJoints: {},
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

  // Serial ports card
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

  // Cameras card
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

  // Capabilities card
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
}

function initJointSliders() {
  const container = $("joints-container");
  if (!container) return;
  container.innerHTML = JOINTS.map((j) => {
    const pct = ((j.def - j.min) / (j.max - j.min)) * 100;
    return `
      <div class="joint-row">
        <div class="joint-head">
          <span>${j.label}</span>
          <span class="joint-values" id="val-${j.key}">L:${j.def}${j.unit} / F:${j.def}${j.unit}</span>
        </div>
        <input
          class="joint-slider"
          type="range"
          min="${j.min}"
          max="${j.max}"
          step="1"
          value="${j.def}"
          data-joint="${j.key}"
          id="slider-${j.key}"
        />
        <div class="gauge-bar-track">
          <div class="gauge-bar-fill" id="bar-${j.key}" style="width:${pct}%"></div>
        </div>
      </div>
    `;
  }).join("");

  container.querySelectorAll(".joint-slider").forEach((slider) => {
    slider.addEventListener("input", (ev) => {
      const target = ev.target;
      const key = target.dataset.joint;
      const val = Number(target.value);
      state.leaderJoints[key] = val;
      state.pendingJoints[key] = val;
      if (!state.jointSendTimer) {
        state.jointSendTimer = setTimeout(flushPendingJoints, 55);
      }
    });
  });
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

function updateJointGauges(leader, follower, syncSliders = false) {
  if (!leader || !follower) return;
  state.leaderJoints = leader;
  state.followerJoints = follower;
  for (const j of JOINTS) {
    const lv = Number(leader[j.key] ?? j.def);
    const fv = Number(follower[j.key] ?? j.def);
    const valEl = $(`val-${j.key}`);
    if (valEl) {
      valEl.textContent = `L:${lv.toFixed(1)}${j.unit} / F:${fv.toFixed(1)}${j.unit}`;
    }
    const barEl = $(`bar-${j.key}`);
    if (barEl) {
      const pct = Math.max(0, Math.min(100, ((fv - j.min) / (j.max - j.min)) * 100));
      barEl.style.width = `${pct.toFixed(1)}%`;
    }
    if (syncSliders) {
      const slider = $(`slider-${j.key}`);
      if (slider && document.activeElement !== slider) {
        slider.value = String(lv);
      }
    }
  }
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
  const tick = () => {
    state.camTimer = null;
    if (state.activeWorkflow !== "idle") {
      refreshCameraImagesOnce();
      state.camTimer = setTimeout(tick, 450);
    }
  };
  state.camTimer = setTimeout(tick, 450);
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
  if (Array.isArray(statusObj.logs)) {
    renderLogs(statusObj.logs);
  }
  if (wf !== "idle") {
    scheduleCameraLoop();
  }
}

async function refreshWaypoints() {
  const res = await apiFetch("/api/workflows/teaching/waypoints");
  const tbody = $("waypoints-tbody");
  if (!res.ok || !tbody) return;
  const list = res.data.waypoints || [];
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

async function refreshEpisodes() {
  const res = await apiFetch("/api/workflows/collect/episodes");
  const tbody = $("episodes-tbody");
  if (!res.ok || !tbody) return;
  const eps = res.data.episodes || [];
  if (!eps.length) {
    tbody.innerHTML = '<tr><td colspan="5" class="muted">暂无已保存的 Episode。</td></tr>';
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
      </tr>`
    )
    .join("");
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
      if (target === "teaching") refreshWaypoints();
      if (target === "collect") refreshEpisodes();
      if (target === "plugins") refreshPlugins();
      if (target === "vla") loadVLAConfigs();
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
        updateJointGauges(msg.leader_joints, msg.follower_joints, wf === "teaching_replay");
        if (msg.workflow_status) {
          const colProg = $("collect-progress-text");
          if (colProg) {
            colProg.textContent = `当前进度: ${msg.workflow_status.current_episode || 0} / ${msg.workflow_status.target_episodes || 0} 回合 (步数: ${msg.workflow_status.current_step || 0})`;
          }
        }
        if (Array.isArray(msg.log_tail)) renderLogs(msg.log_tail);
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

    ws.onerror = () => {
      // Handled by onclose
    };
  } catch {
    startPollingFallback();
  }
}

function bindActions() {
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
      appendLocalLog("已完成硬件重新探测。");
    }
  });

  $("btn-start-calibrate")?.addEventListener("click", async () => {
    const target = $("calibrate-target")?.value || "follower";
    const res = await apiFetch("/api/workflows/calibrate/start", {
      method: "POST",
      body: JSON.stringify({ target }),
    });
    if (res.ok && $("calibrate-status")) {
      $("calibrate-status").textContent = `正在执行 ${target} 标定 (${res.data.mode})...`;
    }
  });

  $("btn-stop-calibrate")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/calibrate/stop", { method: "POST" });
    if ($("calibrate-status")) $("calibrate-status").textContent = "标定已停止。";
  });

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
    const res = await apiFetch("/api/workflows/teleop/joints", {
      method: "POST",
      body: JSON.stringify({ joints: home }),
    });
    if (res.ok) {
      updateJointGauges(res.data.leader_joints, res.data.follower_joints, true);
      refreshCameraImagesOnce();
    }
  });

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
      scheduleCameraLoop();
    }
  });

  $("btn-clear-waypoints")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/teaching/clear", { method: "POST" });
    await refreshWaypoints();
  });

  $("btn-start-collect")?.addEventListener("click", async () => {
    const dataset_name = $("collect-dataset")?.value || "arm101_dryrun_demo";
    const single_task = $("collect-task")?.value || "Pick and place cube";
    const num_episodes = Number($("collect-episodes")?.value || 2);
    const fps = Number($("collect-fps")?.value || 30);
    const res = await apiFetch("/api/workflows/collect/start", {
      method: "POST",
      body: JSON.stringify({ dataset_name, single_task, num_episodes, fps }),
    });
    if (res.ok) {
      state.activeWorkflow = "collect";
      scheduleCameraLoop();
    }
  });

  $("btn-stop-collect")?.addEventListener("click", async () => {
    await apiFetch("/api/workflows/collect/stop", { method: "POST" });
    state.activeWorkflow = "idle";
    await refreshEpisodes();
  });

  $("btn-refresh-episodes")?.addEventListener("click", refreshEpisodes);
  $("btn-refresh-plugins")?.addEventListener("click", refreshPlugins);

  $("btn-vla-preview")?.addEventListener("click", async () => {
    const endpoint = $("vla-endpoint")?.value || "ws://127.0.0.1:8000";
    const policy_name = $("vla-policy-name")?.value || "pi05_so101_custom";
    const task_prompt = $("vla-prompt")?.value || "Pick up the cube";
    const res = await apiFetch("/api/vla/mock_predict", {
      method: "POST",
      body: JSON.stringify({ endpoint, policy_name, task_prompt }),
    });
    if (res.ok && $("vla-preview-output")) {
      $("vla-preview-output").textContent = JSON.stringify(res.data, null, 2);
    }
  });

  $("btn-clear-log-view")?.addEventListener("click", () => {
    state.localLogs = [];
    if ($("log-console")) $("log-console").textContent = "";
  });
}

async function bootstrap() {
  initTabs();
  initJointSliders();
  bindActions();

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

  refreshCameraImagesOnce();
  connectWebSocket();
}

document.addEventListener("DOMContentLoaded", bootstrap);
