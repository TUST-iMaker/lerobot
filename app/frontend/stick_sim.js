/**
 * Official SO-ARM101 6-DOF Stick-Figure 3D Kinematic Canvas Engine
 * Geometry aligned with software/core/kinematics.py & repos/SO-ARM100/Simulation/SO101:
 *   L1 = 110 mm (base -> shoulder_lift)
 *   L2 = 135 mm (upper arm: shoulder_lift -> elbow_flex)
 *   L3 = 125 mm (forearm: elbow_flex -> wrist_flex)
 *   L4 =  71 mm (wrist -> gripper TCP)
 */

export const LINK_MM = {
  base_height: 110,
  upper_arm: 135,
  forearm: 125,
  gripper_tcp: 71,
};

export const JOINT_META = [
  { id: 1, key: "shoulder_pan", label: "底座旋转", unit: "°", linkMm: "L1=110", min: -110, max: 110 },
  { id: 2, key: "shoulder_lift", label: "大臂俯仰", unit: "°", linkMm: "L2=135", min: -100, max: 100 },
  { id: 3, key: "elbow_flex", label: "小臂肘弯", unit: "°", linkMm: "L3=125", min: -97, max: 97 },
  { id: 4, key: "wrist_flex", label: "手腕俯仰", unit: "°", linkMm: "L4=71", min: -95, max: 95 },
  { id: 5, key: "wrist_roll", label: "手腕滚转", unit: "°", linkMm: "Roll", min: -157, max: 163 },
  { id: 6, key: "gripper", label: "末端夹爪", unit: "%", linkMm: "Claw", min: 0, max: 100 },
];

export function estimateEncoder(key, val) {
  const n = Number(val ?? 0);
  if (key === "gripper") {
    return Math.max(0, Math.min(4095, Math.round(2048 + (n - 50) * 20)));
  }
  return Math.max(0, Math.min(4095, Math.round(2048 + n * (4096 / 360))));
}

export function computeStickFK(joints) {
  const pan = ((Number(joints?.shoulder_pan ?? 0)) * Math.PI) / 180;
  const lift = ((Number(joints?.shoulder_lift ?? 0)) * Math.PI) / 180;
  const elbow = ((Number(joints?.elbow_flex ?? 0)) * Math.PI) / 180;
  const wristP = ((Number(joints?.wrist_flex ?? 0)) * Math.PI) / 180;
  const wristR = Number(joints?.wrist_roll ?? 0);
  const grip = Number(joints?.gripper ?? 20);

  const { base_height: L1, upper_arm: L2, forearm: L3, gripper_tcp: L4 } = LINK_MM;
  const phi2 = lift;
  const phi3 = phi2 + Math.PI / 2 + elbow;
  const phi4 = phi3 + wristP;

  const r1 = 0;
  const z1 = L1;
  const r2 = L2 * Math.sin(phi2);
  const z2 = L1 + L2 * Math.cos(phi2);
  const r3 = r2 + L3 * Math.sin(phi3);
  const z3 = z2 + L3 * Math.cos(phi3);
  const r4 = r3 + L4 * Math.sin(phi4);
  const z4 = z3 + L4 * Math.cos(phi4);

  const cp = Math.cos(pan);
  const sp = Math.sin(pan);

  const pts = {
    base: [0, 0, 0],
    shoulder: [r1 * cp, r1 * sp, z1],
    elbow: [r2 * cp, r2 * sp, z2],
    wrist: [r3 * cp, r3 * sp, z3],
    tcp: [r4 * cp, r4 * sp, z4],
  };

  const bodyErr = Math.max(
    Math.abs(Number(joints?.shoulder_pan ?? 0)),
    Math.abs(Number(joints?.shoulder_lift ?? 0)),
    Math.abs(Number(joints?.elbow_flex ?? 0)),
    Math.abs(Number(joints?.wrist_flex ?? 0)),
    Math.abs(Number(joints?.wrist_roll ?? 0))
  );

  return {
    pts,
    phi4,
    pan,
    tcp: {
      x: pts.tcp[0],
      y: pts.tcp[1],
      z: pts.tcp[2],
      r: r4,
      pitchDeg: ((phi4 - Math.PI / 2) * 180) / Math.PI,
      rollDeg: wristR,
      gripperPct: grip,
    },
    homingMaxErr: bodyErr,
    homingAligned: bodyErr <= 8.0,
  };
}

export function renderStickTelemetryHtml(joints, title = "6-DOF 舵机与臂长数据") {
  const fk = computeStickFK(joints);
  const t = fk.tcp;
  const rows = JOINT_META.map((m) => {
    const val = Number(joints?.[m.key] ?? 0);
    const enc = estimateEncoder(m.key, val);
    return `<tr>
      <td>#${m.id} ${m.label}</td>
      <td class="joint-values">${val.toFixed(1)}${m.unit}</td>
      <td class="joint-values">${enc}</td>
      <td class="joint-values">${m.linkMm}</td>
    </tr>`;
  }).join("");

  return `
    <div class="stick-data-head">
      <strong>${title}</strong>
      <span class="joint-values">TCP: X=${t.x.toFixed(0)} Y=${t.y.toFixed(0)} Z=${t.z.toFixed(0)}mm | R=${t.r.toFixed(0)}mm | Pitch=${t.pitchDeg.toFixed(1)}°</span>
    </div>
    <table class="stick-data-table">
      <thead>
        <tr><th>舵机</th><th>角度/开合</th><th>编码器</th><th>臂长(mm)</th></tr>
      </thead>
      <tbody>${rows}</tbody>
    </table>
  `;
}

export class StickArmCanvas {
  constructor(canvasEl, options = {}) {
    this.canvas = canvasEl;
    this.ctx = canvasEl ? canvasEl.getContext("2d") : null;
    this.role = options.role || "follower";
    this.title = options.title || "SO-101 6-DOF";
    this.accent = options.accent || (this.role === "leader" ? "#3b82f6" : "#06b6d4");
    this.showHomingRef = Boolean(options.showHomingRef);
    this.enableIkDrag = Boolean(options.enableIkDrag);
    this.onIkDrag = options.onIkDrag || null;

    // Default 3/4 side view matching official _HOMING_POSITION_DIAGRAM
    this.camYaw = -0.35;
    this.camPitch = 0.24;
    this.joints = {
      shoulder_pan: 0,
      shoulder_lift: 0,
      elbow_flex: 0,
      wrist_flex: 0,
      wrist_roll: 0,
      gripper: 20,
    };
    this.waypoints = [];
    this.vlaChunk = [];
    this.overlayBadge = "";
    this.recActive = false;
    this._lastTcpScreen = null;
    this._dragMode = null; // "orbit" | "ik"
    this._lastMouse = { x: 0, y: 0 };
    this._dragStartMouse = null;
    this._dragStartTcp = null;
    this._ikThrottleTs = 0;

    if (this.canvas) {
      this._bindPointerEvents();
      this.render();
    }
  }

  isDraggingIk() {
    return this._dragMode === "ik";
  }

  setIkEnabled(enabled) {
    this.enableIkDrag = Boolean(enabled);
    this.render();
  }

  resetView() {
    this.camYaw = -0.35;
    this.camPitch = 0.24;
    this.render();
  }

  setState({ joints, waypoints, vlaChunk, overlayBadge, recActive, showHomingRef } = {}) {
    if (joints) this.joints = { ...this.joints, ...joints };
    if (waypoints !== undefined) this.waypoints = waypoints || [];
    if (vlaChunk !== undefined) this.vlaChunk = vlaChunk || [];
    if (overlayBadge !== undefined) this.overlayBadge = overlayBadge || "";
    if (recActive !== undefined) this.recActive = Boolean(recActive);
    if (showHomingRef !== undefined) this.showHomingRef = Boolean(showHomingRef);
    this.render();
  }

  _project(pt3, w, h) {
    const [x, y, z] = pt3;
    const cy = Math.cos(this.camYaw);
    const sy = Math.sin(this.camYaw);
    const cp = Math.cos(this.camPitch);
    const sp = Math.sin(this.camPitch);

    // Rotate around Z (yaw), then tilt by pitch
    const rx = x * cy - y * sy;
    const ry = x * sy + y * cy;
    const rz = z;

    const sx = rx;
    const syScreen = rz * cp - ry * sp;
    const scale = Math.min(w, h) / 460;
    return {
      x: w * 0.44 + sx * scale,
      y: h * 0.80 - syScreen * scale,
      scale,
    };
  }

  _bindPointerEvents() {
    const getPos = (ev) => {
      const rect = this.canvas.getBoundingClientRect();
      return {
        x: ((ev.clientX - rect.left) / Math.max(1, rect.width)) * this.canvas.width,
        y: ((ev.clientY - rect.top) / Math.max(1, rect.height)) * this.canvas.height,
      };
    };

    this.canvas.addEventListener("pointerdown", (ev) => {
      const pos = getPos(ev);
      this._lastMouse = pos;
      if (this.enableIkDrag && this._lastTcpScreen) {
        const dx = pos.x - this._lastTcpScreen.x;
        const dy = pos.y - this._lastTcpScreen.y;
        if (Math.hypot(dx, dy) <= 28) {
          const fk = computeStickFK(this.joints);
          this._dragMode = "ik";
          this._dragStartMouse = pos;
          this._dragStartTcp = {
            r: fk.tcp.r,
            z: fk.tcp.z,
            pan: fk.pan,
            pitchDeg: fk.tcp.pitchDeg,
          };
          this.canvas.setPointerCapture(ev.pointerId);
          return;
        }
      }
      this._dragMode = "orbit";
      this.canvas.setPointerCapture(ev.pointerId);
    });

    this.canvas.addEventListener("pointermove", (ev) => {
      if (!this._dragMode) return;
      const pos = getPos(ev);
      const dx = pos.x - this._lastMouse.x;
      const dy = pos.y - this._lastMouse.y;
      this._lastMouse = pos;

      if (this._dragMode === "orbit") {
        this.camYaw += dx * 0.01;
        this.camPitch = Math.max(-0.5, Math.min(1.2, this.camPitch + dy * 0.008));
        this.render();
      } else if (this._dragMode === "ik" && this.onIkDrag && this._dragStartMouse && this._dragStartTcp) {
        const now = performance.now();
        if (now - this._ikThrottleTs < 45) return;
        this._ikThrottleTs = now;
        const scale = Math.min(this.canvas.width, this.canvas.height) / 460;
        const totalDx = pos.x - this._dragStartMouse.x;
        const totalDy = pos.y - this._dragStartMouse.y;
        const dForward = totalDx / Math.max(0.2, scale);
        const dUp = -totalDy / Math.max(0.2, scale);
        const pan = this._dragStartTcp.pan;
        const newR = Math.max(60, Math.min(315, this._dragStartTcp.r + dForward));
        const newZ = Math.max(30, Math.min(360, this._dragStartTcp.z + dUp));
        this.onIkDrag({
          arm: this.role,
          x_mm: Number((newR * Math.cos(pan)).toFixed(1)),
          y_mm: Number((newR * Math.sin(pan)).toFixed(1)),
          z_mm: Number(newZ.toFixed(1)),
          pitch_deg: Number(this._dragStartTcp.pitchDeg.toFixed(1)),
          gripper: this.joints.gripper,
        });
      }
    });

    const endDrag = () => {
      this._dragMode = null;
      this._dragStartMouse = null;
      this._dragStartTcp = null;
    };
    this.canvas.addEventListener("pointerup", endDrag);
    this.canvas.addEventListener("pointercancel", endDrag);
    this.canvas.addEventListener("dblclick", () => this.resetView());
  }

  _drawArmSkeleton(fk, w, h, { ghost = false, color = "#06b6d4" } = {}) {
    const ctx = this.ctx;
    const pBase = this._project(fk.pts.base, w, h);
    const pSh = this._project(fk.pts.shoulder, w, h);
    const pEl = this._project(fk.pts.elbow, w, h);
    const pWr = this._project(fk.pts.wrist, w, h);
    const pTcp = this._project(fk.pts.tcp, w, h);

    ctx.save();
    if (ghost) {
      ctx.globalAlpha = 0.35;
      ctx.setLineDash([6, 5]);
    }

    // Table C-clamp & base
    if (!ghost) {
      ctx.fillStyle = "#334155";
      ctx.fillRect(pBase.x - 26, pBase.y - 4, 52, 14);
      ctx.fillStyle = "#1e293b";
      ctx.fillRect(pBase.x - 18, pBase.y + 10, 36, 10);
    }

    // Links with distinct segments
    const segments = [
      [pBase, pSh, 8, ghost ? "#f59e0b" : "#64748b", "L1:110"],
      [pSh, pEl, 9, ghost ? "#f59e0b" : color, "L2:135"],
      [pEl, pWr, 7, ghost ? "#f59e0b" : "#38bdf8", "L3:125"],
      [pWr, pTcp, 5, ghost ? "#f59e0b" : "#10b981", "L4:71"],
    ];

    for (const [a, b, lw, stroke, label] of segments) {
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.strokeStyle = stroke;
      ctx.lineWidth = lw;
      ctx.lineCap = "round";
      ctx.stroke();

      if (!ghost) {
        const mx = (a.x + b.x) * 0.5;
        const my = (a.y + b.y) * 0.5;
        ctx.fillStyle = "rgba(226,232,240,0.75)";
        ctx.font = "10px Consolas, monospace";
        ctx.fillText(label, mx + 6, my - 4);
      }
    }

    // Gripper jaws (movable upper claw + fixed lower claw)
    const dirX = pTcp.x - pWr.x;
    const dirY = pTcp.y - pWr.y;
    const dLen = Math.hypot(dirX, dirY) || 1;
    const ux = dirX / dLen;
    const uy = dirY / dLen;
    const nx = -uy;
    const ny = ux;
    const openPx = 4 + (fk.tcp.gripperPct / 100) * 16;

    ctx.strokeStyle = ghost ? "#fbbf24" : "#f8fafc";
    ctx.lineWidth = 3;
    // Lower fixed jaw
    ctx.beginPath();
    ctx.moveTo(pTcp.x - nx * 4, pTcp.y - ny * 4);
    ctx.lineTo(pTcp.x + ux * 18 - nx * 4, pTcp.y + uy * 18 - ny * 4);
    ctx.stroke();
    // Upper movable jaw
    ctx.beginPath();
    ctx.moveTo(pTcp.x + nx * 4, pTcp.y + ny * 4);
    ctx.lineTo(pTcp.x + ux * 18 + nx * openPx, pTcp.y + uy * 18 + ny * openPx);
    ctx.stroke();

    // Joint circles
    if (!ghost) {
      const jointsPts = [
        [pBase, "J1", 6],
        [pSh, "J2", 6],
        [pEl, "J3", 5.5],
        [pWr, "J4/5", 5],
      ];
      for (const [pt, tag, r] of jointsPts) {
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, r, 0, Math.PI * 2);
        ctx.fillStyle = "#f59e0b";
        ctx.fill();
        ctx.fillStyle = "#0f172a";
        ctx.font = "bold 9px Consolas, monospace";
        ctx.fillText(tag, pt.x - 7, pt.y - 9);
      }

      // Draggable IK target ring at TCP
      this._lastTcpScreen = pTcp;
      if (this.enableIkDrag) {
        ctx.beginPath();
        ctx.arc(pTcp.x, pTcp.y, 13, 0, Math.PI * 2);
        ctx.strokeStyle = "#f43f5e";
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = "rgba(244, 63, 94, 0.2)";
        ctx.fill();
      }
    }
    ctx.restore();
  }

  render() {
    if (!this.ctx || !this.canvas) return;
    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;

    // Dark engineering backdrop
    ctx.fillStyle = "#0b1220";
    ctx.fillRect(0, 0, w, h);

    // 3D floor grid (z=0 plane)
    ctx.strokeStyle = "rgba(51, 65, 85, 0.45)";
    ctx.lineWidth = 1;
    for (let g = -200; g <= 280; g += 60) {
      const a = this._project([g, -180, 0], w, h);
      const b = this._project([g, 180, 0], w, h);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
    for (let g = -180; g <= 180; g += 60) {
      const a = this._project([-200, g, 0], w, h);
      const b = this._project([280, g, 0], w, h);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }

    // Optional official homing reference ghost (Calibration page)
    if (this.showHomingRef) {
      const refFk = computeStickFK({
        shoulder_pan: 0,
        shoulder_lift: 0,
        elbow_flex: 0,
        wrist_flex: 0,
        wrist_roll: 0,
        gripper: 20,
      });
      this._drawArmSkeleton(refFk, w, h, { ghost: true });
    }

    // Optional waypoint trail (Teaching page)
    if (this.waypoints.length > 0) {
      ctx.save();
      ctx.strokeStyle = "rgba(245, 158, 11, 0.7)";
      ctx.setLineDash([4, 4]);
      ctx.lineWidth = 2;
      ctx.beginPath();
      this.waypoints.forEach((wp, i) => {
        const p = this._project(computeStickFK(wp.joints || wp).pts.tcp, w, h);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
      ctx.setLineDash([]);
      this.waypoints.forEach((wp, i) => {
        const p = this._project(computeStickFK(wp.joints || wp).pts.tcp, w, h);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
        ctx.fillStyle = "#fbbf24";
        ctx.fill();
        ctx.fillStyle = "#fef3c7";
        ctx.font = "10px Consolas, monospace";
        ctx.fillText(`W${i + 1}`, p.x + 6, p.y - 4);
      });
      ctx.restore();
    }

    // Optional VLA 5-step prediction curve
    if (this.vlaChunk.length > 0) {
      ctx.save();
      ctx.strokeStyle = "#a855f7";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      this.vlaChunk.forEach((stepJ, i) => {
        const p = this._project(computeStickFK(stepJ).pts.tcp, w, h);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
      ctx.restore();
    }

    // Live arm skeleton
    const fk = computeStickFK(this.joints);
    this._drawArmSkeleton(fk, w, h, { ghost: false, color: this.accent });

    // Top HUD header
    ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
    ctx.fillRect(0, 0, w, 28);
    ctx.fillStyle = "#e2e8f0";
    ctx.font = "bold 12px Microsoft YaHei UI, sans-serif";
    ctx.fillText(this.title, 10, 18);

    if (this.showHomingRef) {
      const ok = fk.homingAligned;
      ctx.fillStyle = ok ? "#10b981" : "#f59e0b";
      ctx.font = "bold 11px Microsoft YaHei UI, sans-serif";
      ctx.fillText(
        ok
          ? `✓ 中位重合成功 (误差 ${fk.homingMaxErr.toFixed(1)}°)`
          : `对齐中位中 (最大偏差 ${fk.homingMaxErr.toFixed(1)}°)`,
        w - 195,
        18
      );
    } else if (this.recActive) {
      ctx.fillStyle = "#ef4444";
      ctx.font = "bold 11px Consolas, monospace";
      ctx.fillText("● REC 采集中", w - 105, 18);
    } else if (this.overlayBadge) {
      ctx.fillStyle = "#38bdf8";
      ctx.font = "bold 11px Microsoft YaHei UI, sans-serif";
      ctx.fillText(this.overlayBadge, w - 150, 18);
    }

    // Bottom hint
    ctx.fillStyle = "rgba(148, 163, 184, 0.85)";
    ctx.font = "10px Microsoft YaHei UI, sans-serif";
    const hint = this.enableIkDrag
      ? "拖红圈=末端夹爪 IK 控臂 | 拖空白=旋转视角 | 双击复位"
      : "拖拽画布旋转 3D 视角 | 双击复位";
    ctx.fillText(hint, 10, h - 8);
  }
}
