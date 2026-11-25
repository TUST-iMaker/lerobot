/**
 * Official SO-ARM101 6-DOF Stick-Figure 3D Kinematic Canvas Engine
 * Daylight Utilitarian Minimalist Edition
 * - Exact URDF Forward Kinematics (so101_new_calib.urdf)
 * - Wheel Zoom (0.45x..2.80x, passive:false), Right/Shift Pan (clamped), Left Orbit, 4 View Presets
 * - Screen-space hit-testing & zoom-compensated IK inverse mapping
 * - ResizeObserver + DPR adaptive full-height rendering (ignores 0x0 hidden states, supports pause/resume)
 */

export const LINK_MM = {
  base_height: 116.6,
  upper_arm: 116.0,
  forearm: 135.0,
  gripper_tcp: 160.5,
};

export const STATIC_LINK_TOOLTIP =
  "SO-ARM101 URDF 固定臂长: L1=116.6mm | L2=116.0mm | L3=135.0mm | L4=160.5mm | 左键旋转 / 右键或Shift平移 / 滚轮缩放 / 双击复位";

export const JOINT_META = [
  { id: 1, key: "shoulder_pan", short: "#1 shoulder_pan", label: "底座旋转", unit: "°", min: -110, max: 110 },
  { id: 2, key: "shoulder_lift", short: "#2 shoulder_lift", label: "大臂俯仰", unit: "°", min: -100, max: 100 },
  { id: 3, key: "elbow_flex", short: "#3 elbow_flex", label: "小臂肘弯", unit: "°", min: -97, max: 97 },
  { id: 4, key: "wrist_flex", short: "#4 wrist_flex", label: "手腕俯仰", unit: "°", min: -95, max: 95 },
  { id: 5, key: "wrist_roll", short: "#5 wrist_roll", label: "手腕滚转", unit: "°", min: -157, max: 163 },
  { id: 6, key: "gripper", short: "#6 gripper", label: "末端夹爪", unit: "%", min: 0, max: 100 },
];

const URDF_CHAIN = [
  { name: "shoulder_pan", rev: true, xyz: [0.0388353, -8.97657e-9, 0.0624], rpy: [3.14159, 4.18253e-17, -3.14159] },
  { name: "shoulder_lift", rev: true, xyz: [-0.0303992, -0.0182778, -0.0542], rpy: [-1.5708, -1.5708, 0.0] },
  { name: "elbow_flex", rev: true, xyz: [-0.11257, -0.028, 1.73763e-16], rpy: [-3.63608e-16, 8.74301e-16, 1.5708] },
  { name: "wrist_flex", rev: true, xyz: [-0.1349, 0.0052, 3.62355e-17], rpy: [4.02456e-15, 8.67362e-16, -1.5708] },
  { name: "wrist_roll", rev: true, xyz: [5.55112e-17, -0.0611, 0.0181], rpy: [1.5708, 0.0486795, 3.14159] },
  { name: "gripper_frame_joint", rev: false, xyz: [-0.0079, -0.000218121, -0.0981274], rpy: [0.0, 3.14159, 0.0] },
];

function _rpyMat(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function _rotZMat(angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ];
}

function _mulMat3(A, B) {
  const C = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    }
  }
  return C;
}

function _mulMatVec3(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

export function fmtZero(n, digits = 0) {
  const val = Number(n ?? 0);
  const threshold = digits === 0 ? 0.5 : Math.pow(10, -(digits + 1)) * 5;
  return Math.abs(val) < threshold ? (0).toFixed(digits) : val.toFixed(digits);
}

export function estimateEncoder(key, val, opts = {}) {
  const n = Number(val ?? 0);
  const rMin = opts.rangeMin ?? 0;
  const rMax = opts.rangeMax ?? 4095;
  const driveMode = opts.driveMode ?? 0;
  const span = Math.max(1, rMax - rMin);
  if (key === "gripper") {
    const pct = Math.max(0, Math.min(100, n));
    const norm = driveMode ? 100 - pct : pct;
    return Math.max(rMin, Math.min(rMax, Math.round(rMin + (norm / 100) * span)));
  }
  const mid = opts.rangeMin !== undefined && opts.rangeMax !== undefined ? (rMin + rMax) / 2 : 2047;
  const sign = driveMode ? -1 : 1;
  return Math.max(rMin, Math.min(rMax, Math.round(mid + sign * n * (4095 / 360))));
}

export function computeStickFK(joints) {
  const wristR = Number(joints?.wrist_roll ?? 0);
  const grip = Number(joints?.gripper ?? 20);

  let R = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  let p = [0, 0, 0];
  const nodePosMm = { base: [0, 0, 0] };

  for (const spec of URDF_CHAIN) {
    const dp = _mulMatVec3(R, spec.xyz);
    p = [p[0] + dp[0], p[1] + dp[1], p[2] + dp[2]];
    R = _mulMat3(R, _rpyMat(spec.rpy[0], spec.rpy[1], spec.rpy[2]));
    nodePosMm[spec.name] = [p[0] * 1000, p[1] * 1000, p[2] * 1000];
    if (spec.rev) {
      const deg = Number(joints?.[spec.name] ?? 0);
      const rad = (deg * Math.PI) / 180;
      R = _mulMat3(R, _rotZMat(rad));
    }
  }

  const pts = {
    base: [0, 0, 0],
    shoulder: nodePosMm.shoulder_lift || [0, 0, 116.6],
    elbow: nodePosMm.elbow_flex || [0, 0, 229.2],
    wrist: nodePosMm.wrist_flex || [171.3, 0, 229.2],
    wristRoll: nodePosMm.wrist_roll || [232.4, 0, 229.2],
    tcp: nodePosMm.gripper_frame_joint,
  };

  const tcpX = pts.tcp[0];
  const tcpY = pts.tcp[1];
  const tcpZ = pts.tcp[2];
  const r4 = Math.hypot(tcpX, tcpY);
  // gripper_frame_link local basis in world frame:
  // +Z (R[:,2]) = forward along gripper axis
  // +X (R[:,0]) = jaw opening/closing plane direction (rotates with #5 wrist_roll!)
  // +Y (R[:,1]) = jaw hinge pin axis
  const forwardVec = [R[0][2], R[1][2], R[2][2]];
  const jawOpenVec = [R[0][0], R[1][0], R[2][0]];
  const jawNormalVec = [R[0][1], R[1][1], R[2][1]];
  const pitchRad = Math.atan2(-forwardVec[2], Math.hypot(forwardVec[0], forwardVec[1]));
  const panWorld = Math.atan2(tcpY, tcpX);
  const phi4 = pitchRad + Math.PI / 2;

  const bodyErr = Math.max(
    Math.abs(Number(joints?.shoulder_pan ?? 0)),
    Math.abs(Number(joints?.shoulder_lift ?? 0)),
    Math.abs(Number(joints?.elbow_flex ?? 0)),
    Math.abs(Number(joints?.wrist_flex ?? 0)),
    Math.abs(Number(joints?.wrist_roll ?? 0))
  );

  return {
    pts,
    R_tcp: R,
    forwardVec,
    jawOpenVec,
    jawNormalVec,
    phi4,
    pan: panWorld,
    tcp: {
      x: tcpX,
      y: tcpY,
      z: tcpZ,
      r: r4,
      pitchDeg: (pitchRad * 180) / Math.PI,
      rollDeg: wristR,
      gripperPct: grip,
    },
    homingMaxErr: bodyErr,
    homingAligned: bodyErr <= 8.0,
  };
}

export function renderStickTelemetryHtml(joints, title = "6-DOF") {
  const fk = computeStickFK(joints);
  const t = fk.tcp;
  const cells = JOINT_META.map((m) => {
    const val = Number(joints?.[m.key] ?? 0);
    const enc = estimateEncoder(m.key, val);
    return `<div class="telemetry-chip" title="${m.label} (${enc})">
      <span class="telemetry-chip-key">#${m.id}</span>
      <span class="joint-values">${fmtZero(val, 1)}${m.unit}</span>
    </div>`;
  }).join("");

  return `
    <div class="telemetry-strip-bar" title="${STATIC_LINK_TOOLTIP}">
      <span class="telemetry-strip-title">${title}</span>
      <div class="telemetry-chip-group">${cells}</div>
      <span class="joint-values telemetry-tcp-inline">TCP ${fmtZero(t.x)}, ${fmtZero(t.y)}, ${fmtZero(t.z)} mm</span>
    </div>
  `;
}

export class StickArmCanvas {
  constructor(canvasEl, options = {}) {
    this.canvas = canvasEl;
    this.ctx = canvasEl ? canvasEl.getContext("2d") : null;
    this.role = options.role || "follower";
    this.label = options.label || (this.role === "leader" ? "Leader 主臂" : "Follower 从臂");
    this.showHomingRef = Boolean(options.showHomingRef);
    this.enableIkDrag = Boolean(options.enableIkDrag);
    this.onIkDrag = options.onIkDrag || null;
    this.onCameraChange = options.onCameraChange || null;
    this.paused = false;

    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.logicalW = Number(canvasEl?.getAttribute("width") || 380);
    this.logicalH = Number(canvasEl?.getAttribute("height") || 220);

    // Camera parameters: Yaw, Pitch, Zoom (0.45..2.80), PanX, PanY
    this.camYaw = -0.42;
    this.camPitch = 0.26;
    this.camZoom = 1.0;
    this.camPanX = 0;
    this.camPanY = 0;

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
    this._dragMode = null; // "orbit" | "pan" | "ik"
    this._lastMouse = { x: 0, y: 0 };
    this._dragStartMouse = null;
    this._dragStartTcp = null;
    this._ikThrottleTs = 0;
    this._resizeObserver = null;

    if (this.canvas) {
      this.canvas.title = STATIC_LINK_TOOLTIP;
      this._bindPointerEvents();
      this._initResizeObserver();
      this._initThemeObserver();
      this.render();
    }
  }

  _isDarkTheme() {
    return document.documentElement?.dataset?.theme === "dark";
  }

  _initThemeObserver() {
    if (typeof MutationObserver === "undefined" || !document.documentElement) return;
    this._themeObserver = new MutationObserver(() => {
      if (!this.paused) this.render();
    });
    this._themeObserver.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ["data-theme"],
    });
  }

  _initResizeObserver() {
    if (!this.canvas || typeof ResizeObserver === "undefined") return;
    const target = this.canvas.parentElement || this.canvas;
    this._resizeObserver = new ResizeObserver((entries) => {
      if (this.paused) return;
      for (const entry of entries) {
        const cr = entry.contentRect;
        if (!cr || cr.width < 20 || cr.height < 20) {
          continue;
        }
        const nextW = Math.max(180, Math.round(cr.width));
        const nextH = Math.max(140, Math.min(920, Math.round(cr.height)));
        const nextDpr = Math.min(window.devicePixelRatio || 1, 2);
        if (
          Math.abs(nextW - this.logicalW) > 1 ||
          Math.abs(nextH - this.logicalH) > 1 ||
          nextDpr !== this.dpr
        ) {
          this.logicalW = nextW;
          this.logicalH = nextH;
          this.dpr = nextDpr;
          this.canvas.width = Math.round(this.logicalW * this.dpr);
          this.canvas.height = Math.round(this.logicalH * this.dpr);
          this._clampPan();
          this.render();
        }
      }
    });
    this._resizeObserver.observe(target);
  }

  _clampPan() {
    const maxX = this.logicalW * 0.45;
    const maxY = this.logicalH * 0.45;
    this.camPanX = Math.max(-maxX, Math.min(maxX, this.camPanX));
    this.camPanY = Math.max(-maxY, Math.min(maxY, this.camPanY));
  }

  setPaused(paused) {
    const next = Boolean(paused);
    if (this.paused === next) return;
    this.paused = next;
    if (!next) {
      this.refreshSize();
    }
  }

  refreshSize() {
    if (!this.canvas || this.paused) return;
    const parent = this.canvas.parentElement || this.canvas;
    const rect = parent.getBoundingClientRect();
    if (rect.width < 20 || rect.height < 20) return;
    this.logicalW = Math.max(180, Math.round(rect.width));
    this.logicalH = Math.max(140, Math.min(920, Math.round(rect.height)));
    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.canvas.width = Math.round(this.logicalW * this.dpr);
    this.canvas.height = Math.round(this.logicalH * this.dpr);
    this._clampPan();
    this.render();
  }

  isDraggingIk() {
    return this._dragMode === "ik";
  }

  setIkEnabled(enabled) {
    this.enableIkDrag = Boolean(enabled);
    this.render();
  }

  zoomBy(factor) {
    this.camZoom = Math.max(0.45, Math.min(2.8, Number((this.camZoom * factor).toFixed(2))));
    this._clampPan();
    this.render();
    if (this.onCameraChange) this.onCameraChange(this.getCameraState());
  }

  setPresetView(preset) {
    if (preset === "side") {
      // Side view: exact X/Z plane (horizontal = +X, vertical = +Z)
      this.camYaw = 0.0;
      this.camPitch = 0.0;
    } else if (preset === "front") {
      // Front view: exact Y/Z plane (horizontal = +Y, vertical = +Z)
      this.camYaw = -Math.PI / 2;
      this.camPitch = 0.0;
    } else if (preset === "top") {
      // Top view: looking down onto X/Y horizontal plane
      this.camYaw = 0.0;
      this.camPitch = 1.35;
    } else {
      this.camYaw = -0.42;
      this.camPitch = 0.26;
      if (preset === "reset") {
        this.camZoom = 1.0;
        this.camPanX = 0;
        this.camPanY = 0;
      }
    }
    this._clampPan();
    this.render();
    if (this.onCameraChange) this.onCameraChange(this.getCameraState());
  }

  /**
   * Truncate internal drag target anchor when backend hits URDF workspace boundary
   * (prevents target coordinate from drifting outside boundary and causing pull-back lag).
   */
  syncClampedDragAnchor(actualTcp) {
    if (this._dragMode !== "ik" || !actualTcp || !this._dragStartTcp) return;
    this._dragStartTcp.x = Number(actualTcp.x_mm ?? actualTcp.x ?? this._dragStartTcp.x);
    this._dragStartTcp.y = Number(actualTcp.y_mm ?? actualTcp.y ?? this._dragStartTcp.y);
    this._dragStartTcp.z = Number(actualTcp.z_mm ?? actualTcp.z ?? this._dragStartTcp.z);
    if (this._lastMouse) {
      this._dragStartMouse = { x: this._lastMouse.x, y: this._lastMouse.y };
    }
  }

  resetView() {
    this.setPresetView("reset");
  }

  getCameraState() {
    return {
      yaw: this.camYaw,
      pitch: this.camPitch,
      zoom: this.camZoom,
      panX: this.camPanX,
      panY: this.camPanY,
    };
  }

  setCameraState(cam = {}) {
    if (cam.yaw !== undefined) this.camYaw = Number(cam.yaw);
    if (cam.pitch !== undefined) this.camPitch = Number(cam.pitch);
    if (cam.zoom !== undefined) this.camZoom = Math.max(0.45, Math.min(2.8, Number(cam.zoom)));
    if (cam.panX !== undefined) this.camPanX = Number(cam.panX);
    if (cam.panY !== undefined) this.camPanY = Number(cam.panY);
    this._clampPan();
    this.render();
  }

  setState({ joints, waypoints, vlaChunk, overlayBadge, recActive, showHomingRef, label } = {}) {
    if (joints) this.joints = { ...this.joints, ...joints };
    if (waypoints !== undefined) this.waypoints = waypoints || [];
    if (vlaChunk !== undefined) this.vlaChunk = vlaChunk || [];
    if (overlayBadge !== undefined) this.overlayBadge = overlayBadge || "";
    if (recActive !== undefined) this.recActive = Boolean(recActive);
    if (showHomingRef !== undefined) this.showHomingRef = Boolean(showHomingRef);
    if (label !== undefined) this.label = label;
    if (!this.paused) this.render();
  }

  _baseScale(w, h) {
    return Math.min(w * 0.76, h * 1.16) / 485;
  }

  _effectiveScale(w, h) {
    return this._baseScale(w, h) * this.camZoom;
  }

  _project(pt3, w, h) {
    const [x, y, z] = pt3;
    const cy = Math.cos(this.camYaw);
    const sy = Math.sin(this.camYaw);
    const cp = Math.cos(this.camPitch);
    const sp = Math.sin(this.camPitch);

    const rx = x * cy - y * sy;
    const ry = x * sy + y * cy;
    const rz = z;

    const sx = rx;
    const syScreen = rz * cp - ry * sp;
    const scale = this._effectiveScale(w, h);
    return {
      x: w * 0.28 + this.camPanX + sx * scale,
      y: h * 0.80 + this.camPanY - syScreen * scale,
      scale,
    };
  }

  _bindPointerEvents() {
    const getPos = (ev) => {
      const rect = this.canvas.getBoundingClientRect();
      return {
        x: ((ev.clientX - rect.left) / Math.max(1, rect.width)) * this.logicalW,
        y: ((ev.clientY - rect.top) / Math.max(1, rect.height)) * this.logicalH,
      };
    };

    this.canvas.addEventListener("contextmenu", (ev) => {
      ev.preventDefault();
    });

    this.canvas.addEventListener(
      "wheel",
      (ev) => {
        ev.preventDefault();
        const factor = ev.deltaY < 0 ? 1.1 : 0.91;
        this.zoomBy(factor);
      },
      { passive: false }
    );

    this.canvas.addEventListener("pointerdown", (ev) => {
      const pos = getPos(ev);
      this._lastMouse = pos;

      // 1. Left-click (with or without Shift) on projected TCP red ring -> Plane-Projected IK Drag
      //    (Shift + Red-Ring Drag controls vertical Z axis)
      if (ev.button === 0 && this.enableIkDrag && this._lastTcpScreen) {
        const dx = pos.x - this._lastTcpScreen.x;
        const dy = pos.y - this._lastTcpScreen.y;
        if (Math.hypot(dx, dy) <= 26) {
          const fk = computeStickFK(this.joints);
          this._dragMode = "ik";
          this._dragStartMouse = pos;
          this._dragStartTcp = {
            x: fk.tcp.x,
            y: fk.tcp.y,
            z: fk.tcp.z,
            pitchDeg: fk.tcp.pitchDeg,
          };
          this.canvas.setPointerCapture(ev.pointerId);
          return;
        }
      }

      // 2. Right-click, Middle-click, or Shift+Left-click on canvas background -> 2D Camera Pan
      if (ev.button === 2 || ev.button === 1 || ev.shiftKey) {
        this._dragMode = "pan";
        this.canvas.setPointerCapture(ev.pointerId);
        return;
      }

      // 3. Left-click on canvas background -> 3D Camera Orbit
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
        this.camPitch = Math.max(-0.45, Math.min(1.38, this.camPitch + dy * 0.008));
        this.render();
        if (this.onCameraChange) this.onCameraChange(this.getCameraState());
      } else if (this._dragMode === "pan") {
        this.camPanX += dx;
        this.camPanY += dy;
        this._clampPan();
        this.render();
        if (this.onCameraChange) this.onCameraChange(this.getCameraState());
      } else if (this._dragMode === "ik" && this.onIkDrag && this._dragStartMouse && this._dragStartTcp) {
        const now = performance.now();
        if (now - this._ikThrottleTs < 34) return;
        this._ikThrottleTs = now;

        const effScale = Math.max(0.15, this._effectiveScale(this.logicalW, this.logicalH));
        const dScreenX = (pos.x - this._dragStartMouse.x) / effScale;
        const dScreenY = -(pos.y - this._dragStartMouse.y) / effScale; // Screen Up = +

        const cy = Math.cos(this.camYaw);
        const sy = Math.sin(this.camYaw);
        const cp = Math.cos(this.camPitch);
        const sp = Math.sin(this.camPitch);

        let targetX = this._dragStartTcp.x;
        let targetY = this._dragStartTcp.y;
        let targetZ = this._dragStartTcp.z;

        if (ev.shiftKey) {
          // Shift + Red-Ring Drag -> vertical Z axis control
          const denomZ = Math.abs(cp) < 0.25 ? 0.25 : cp;
          targetZ = Math.max(20, Math.min(440, this._dragStartTcp.z + dScreenY / denomZ));
        } else if (Math.abs(sy) < 0.24 && Math.abs(this.camPitch) < 0.24) {
          // Side View -> exact X / Z plane (Y locked)
          const denomX = Math.abs(cy) < 0.25 ? Math.sign(cy || 1) * 0.25 : cy;
          const denomZ = Math.abs(cp) < 0.25 ? 0.25 : cp;
          targetX = this._dragStartTcp.x + dScreenX / denomX;
          targetZ = Math.max(20, Math.min(440, this._dragStartTcp.z + dScreenY / denomZ));
        } else if (Math.abs(cy) < 0.24 && Math.abs(this.camPitch) < 0.24) {
          // Front View -> exact Y / Z plane (X locked)
          const denomY = Math.abs(sy) < 0.25 ? Math.sign(sy || -1) * 0.25 : sy;
          const denomZ = Math.abs(cp) < 0.25 ? 0.25 : cp;
          targetY = this._dragStartTcp.y - dScreenX / denomY;
          targetZ = Math.max(20, Math.min(440, this._dragStartTcp.z + dScreenY / denomZ));
        } else {
          // Isometric / Top / Orbit View -> ray intersection with horizontal plane Z = current_tcp.z (controls X / Y)
          const denomSp = Math.abs(sp) < 0.18 ? Math.sign(sp || 1) * 0.18 : sp;
          const drx = dScreenX;
          const dry = -dScreenY / denomSp;
          targetX = this._dragStartTcp.x + (drx * cy + dry * sy);
          targetY = this._dragStartTcp.y + (-drx * sy + dry * cy);
        }

        this.onIkDrag({
          arm: this.role,
          x_mm: Number(targetX.toFixed(1)),
          y_mm: Number(targetY.toFixed(1)),
          z_mm: Number(targetZ.toFixed(1)),
          pitch_deg: Number(this._dragStartTcp.pitchDeg.toFixed(1)),
          gripper: this.joints.gripper,
          clamp_to_workspace: true,
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

  _drawCylinderLink(a, b, width, mainColor, highlightColor, shadowColor, ghost = false) {
    const ctx = this.ctx;
    ctx.save();
    ctx.lineCap = "round";
    if (ghost) {
      ctx.strokeStyle = "rgba(100, 116, 139, 0.38)";
      ctx.lineWidth = width * 0.72;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      ctx.restore();
      return;
    }

    ctx.strokeStyle = shadowColor;
    ctx.lineWidth = width + 2;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();

    ctx.strokeStyle = mainColor;
    ctx.lineWidth = width;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();

    ctx.strokeStyle = highlightColor;
    ctx.lineWidth = Math.max(1.5, width * 0.32);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y - 0.8);
    ctx.lineTo(b.x, b.y - 0.8);
    ctx.stroke();
    ctx.restore();
  }

  _drawArmSkeleton(fk, w, h, { ghost = false } = {}) {
    const ctx = this.ctx;
    const isDark = this._isDarkTheme();
    const pBase = this._project(fk.pts.base, w, h);
    const pSh = this._project(fk.pts.shoulder, w, h);
    const pEl = this._project(fk.pts.elbow, w, h);
    const pWr = this._project(fk.pts.wrist, w, h);
    const pTcp = this._project(fk.pts.tcp, w, h);
    const zScale = Math.max(0.75, Math.min(1.45, this.camZoom));

    ctx.save();

    if (!ghost) {
      ctx.fillStyle = isDark ? "#1E293B" : "#E2E8F0";
      ctx.strokeStyle = isDark ? "#475569" : "#94A3B8";
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.ellipse(pBase.x, pBase.y + 4, 22 * zScale, 8 * zScale, 0, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = isDark ? "#64748B" : "#334155";
      ctx.beginPath();
      ctx.ellipse(pBase.x, pBase.y, 12 * zScale, 4.5 * zScale, 0, 0, Math.PI * 2);
      ctx.fill();
    }

    const isLeader = this.role === "leader";
    this._drawCylinderLink(pBase, pSh, 9.5 * zScale, "#475569", "#94A3B8", "#1E293B", ghost);
    this._drawCylinderLink(
      pSh,
      pEl,
      9 * zScale,
      isLeader ? "#2563EB" : "#0284C7",
      isLeader ? "#93C5FD" : "#7DD3FC",
      isLeader ? "#1E3A8A" : "#0369A1",
      ghost
    );
    this._drawCylinderLink(
      pEl,
      pWr,
      7.5 * zScale,
      isLeader ? "#3B82F6" : "#0EA5E9",
      isLeader ? "#BFDBFE" : "#BAE6FD",
      isLeader ? "#1D4ED8" : "#0284C7",
      ghost
    );
    this._drawCylinderLink(pWr, pTcp, 5.8 * zScale, "#06B6D4", "#A5F3FC", "#155E75", ghost);

    // True 3D Parallel Gripper Jaws using URDF R_tcp (accurately displays #5 wrist_roll axial rotation!)
    const tcp3 = fk.pts.tcp;
    const fV = fk.forwardVec || [1, 0, 0];
    const oV = fk.jawOpenVec || [0, 1, 0];
    const add3 = (a, b, s = 1) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];

    // Gripper wrist base collar 22mm behind TCP tip along -forwardVec
    const collar3 = add3(tcp3, fV, -22);
    const fixedOffMm = -10;
    const movingOffMm = 10 + (fk.tcp.gripperPct / 100) * 32;
    const jawLenMm = 28;

    const pColFixed = this._project(add3(collar3, oV, fixedOffMm), w, h);
    const pColMov = this._project(add3(collar3, oV, movingOffMm), w, h);
    const pTipFixed = this._project(add3(add3(collar3, oV, fixedOffMm), fV, jawLenMm), w, h);
    const pTipMov = this._project(add3(add3(collar3, oV, movingOffMm), fV, jawLenMm), w, h);

    ctx.lineCap = "round";
    // Wrist-roll crossbar connecting fixed jaw and moving jaw (shows 360° roll plane clearly)
    ctx.strokeStyle = ghost ? "rgba(100, 116, 139, 0.3)" : isDark ? "#94A3B8" : "#475569";
    ctx.lineWidth = 2.6 * zScale;
    ctx.beginPath();
    ctx.moveTo(pColFixed.x, pColFixed.y);
    ctx.lineTo(pColMov.x, pColMov.y);
    ctx.stroke();

    // Fixed Jaw (Dark Steel)
    ctx.strokeStyle = ghost ? "rgba(100, 116, 139, 0.35)" : isDark ? "#E2E8F0" : "#18181B";
    ctx.lineWidth = 2.6 * zScale;
    ctx.beginPath();
    ctx.moveTo(pColFixed.x, pColFixed.y);
    ctx.lineTo(pTipFixed.x, pTipFixed.y);
    ctx.stroke();

    // Moving Jaw (High-contrast Accent Amber/Orange to distinguish roll orientation at a glance)
    ctx.strokeStyle = ghost ? "rgba(100, 116, 139, 0.35)" : isDark ? "#FB923C" : "#EA580C";
    ctx.lineWidth = 2.6 * zScale;
    ctx.beginPath();
    ctx.moveTo(pColMov.x, pColMov.y);
    ctx.lineTo(pTipMov.x, pTipMov.y);
    ctx.stroke();

    if (!ghost) {
      const jointNodes = [
        [pBase, 5.5 * zScale],
        [pSh, 5.5 * zScale],
        [pEl, 5 * zScale],
        [pWr, 4.5 * zScale],
      ];
      for (const [pt, r] of jointNodes) {
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, r, 0, Math.PI * 2);
        ctx.fillStyle = isDark ? "#0F172A" : "#FFFFFF";
        ctx.fill();
        ctx.lineWidth = 1.8;
        ctx.strokeStyle = isDark ? "#E2E8F0" : "#18181B";
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 1.8, 0, Math.PI * 2);
        ctx.fillStyle = isLeader ? "#3B82F6" : "#38BDF8";
        ctx.fill();
      }

      const targetCenter = { x: pTcp.x, y: pTcp.y };
      this._lastTcpScreen = targetCenter;
      if (this.enableIkDrag) {
        ctx.beginPath();
        ctx.arc(targetCenter.x, targetCenter.y, 13 * zScale, 0, Math.PI * 2);
        ctx.fillStyle = isDark ? "rgba(239, 68, 68, 0.20)" : "rgba(220, 38, 38, 0.12)";
        ctx.fill();
        ctx.lineWidth = 1.75;
        ctx.strokeStyle = isDark ? "#EF4444" : "#DC2626";
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(targetCenter.x, targetCenter.y, 3, 0, Math.PI * 2);
        ctx.fillStyle = isDark ? "#EF4444" : "#DC2626";
        ctx.fill();
      }
    }
    ctx.restore();
  }

  render() {
    if (!this.ctx || !this.canvas || this.paused) return;
    const ctx = this.ctx;
    const w = this.logicalW;
    const h = this.logicalH;
    const isDark = this._isDarkTheme();

    ctx.save();
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);

    // 1. Theme-Adaptive Studio Backdrop (#F8F9FA in Light, #0B0E14 in Dark)
    ctx.fillStyle = isDark ? "#0B0E14" : "#F8F9FA";
    ctx.fillRect(0, 0, w, h);

    // 2. Perspective CAD Floor Grid (z = 0 plane)
    ctx.strokeStyle = isDark ? "rgba(148, 163, 184, 0.12)" : "rgba(15, 23, 42, 0.075)";
    ctx.lineWidth = 1;
    for (let g = -220; g <= 300; g += 55) {
      const a = this._project([g, -200, 0], w, h);
      const b = this._project([g, 200, 0], w, h);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
    for (let g = -200; g <= 200; g += 55) {
      const a = this._project([-220, g, 0], w, h);
      const b = this._project([300, g, 0], w, h);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }

    // 3. Optional Homing Ghost
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

    // 4. Optional Waypoint Trajectory Curve
    if (this.waypoints.length > 0) {
      ctx.save();
      ctx.strokeStyle = isDark ? "#38BDF8" : "#0284C7";
      ctx.setLineDash([4, 4]);
      ctx.lineWidth = 1.8;
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
        ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = isDark ? "#38BDF8" : "#0284C7";
        ctx.fill();
        ctx.fillStyle = isDark ? "#F1F5F9" : "#18181B";
        ctx.font = "600 10px 'JetBrains Mono', Consolas, monospace";
        ctx.fillText(`W${i + 1}`, p.x + 5, p.y - 4);
      });
      ctx.restore();
    }

    // 5. Optional VLA 5-Step Prediction Curve
    if (this.vlaChunk.length > 0) {
      ctx.save();
      ctx.strokeStyle = isDark ? "#60A5FA" : "#1D4ED8";
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      this.vlaChunk.forEach((stepJ, i) => {
        const p = this._project(computeStickFK(stepJ).pts.tcp, w, h);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
      ctx.restore();
    }

    // 6. Live Robotic Arm Skeleton
    const fk = computeStickFK(this.joints);
    this._drawArmSkeleton(fk, w, h, { ghost: false });

    const pillBg = isDark ? "rgba(22, 27, 36, 0.90)" : "rgba(255, 255, 255, 0.92)";
    const pillBorder = isDark ? "#262E3D" : "#E5E5E0";
    const pillText = isDark ? "#F1F5F9" : "#18181B";

    // 7. Top-Right Monospace TCP Coordinate + Zoom HUD Pill
    const t = fk.tcp;
    const zoomStr = this.camZoom !== 1 ? ` | ${this.camZoom.toFixed(1)}x` : "";
    const tcpText = `TCP ${fmtZero(t.x)}, ${fmtZero(t.y)}, ${fmtZero(t.z)} mm${zoomStr}`;
    ctx.font = "500 10.5px 'JetBrains Mono', 'Cascadia Mono', Consolas, monospace";
    const tm = ctx.measureText(tcpText);
    const pillW = tm.width + 14;
    const pillH = 20;
    const pillX = w - pillW - 8;
    const pillY = 8;

    ctx.fillStyle = pillBg;
    ctx.strokeStyle = pillBorder;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(pillX, pillY, pillW, pillH, 4);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = pillText;
    ctx.fillText(tcpText, pillX + 7, pillY + 14);

    // 8. Top-Left Arm Role Pill & Bottom-Left Active Workflow Status Pill (Zero Collision)
    if (this.label) {
      ctx.font = "600 11px 'SF Pro Display', 'Microsoft YaHei UI', sans-serif";
      const ltm = ctx.measureText(this.label);
      const lw = ltm.width + 14;
      ctx.fillStyle = pillBg;
      ctx.strokeStyle = pillBorder;
      ctx.beginPath();
      ctx.roundRect(8, 8, lw, 20, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = pillText;
      ctx.fillText(this.label, 15, 22);
    }

    let statusText = "";
    let statusColor = pillText;
    if (this.showHomingRef) {
      statusText = fk.homingAligned
        ? `中位已对齐 (${fk.homingMaxErr.toFixed(1)}°)`
        : `中位偏差 ${fk.homingMaxErr.toFixed(1)}°`;
      statusColor = fk.homingAligned
        ? isDark
          ? "#4ADE80"
          : "#285A2C"
        : isDark
          ? "#FBBF24"
          : "#8F5E00";
    } else if (this.recActive) {
      statusText = "REC 采集中";
      statusColor = isDark ? "#F87171" : "#9F2F2D";
    } else if (this.overlayBadge) {
      statusText = this.overlayBadge;
      statusColor = isDark ? "#38BDF8" : "#1B5E8A";
    }

    if (statusText) {
      ctx.font = "600 10.5px 'SF Pro Display', 'Microsoft YaHei UI', sans-serif";
      const stm = ctx.measureText(statusText);
      const sw = stm.width + 14;
      const sy = h - 26;
      ctx.fillStyle = pillBg;
      ctx.strokeStyle = pillBorder;
      ctx.beginPath();
      ctx.roundRect(8, sy, sw, 19, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = statusColor;
      ctx.fillText(statusText, 15, sy + 13.5);
    }

    ctx.restore();
  }
}
