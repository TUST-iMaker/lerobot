/**
 * Shared Communication, WebSocket Telemetry, Web Locks Popup Mutex & Control Bus
 * Reused across Main Studio (index.html) and Standalone Virtual Twin (/sim-viewport)
 */
import { estimateEncoder, fmtZero } from "/static/stick_sim.js?v=2026112304";

export const JOINTS = [
  { key: "shoulder_pan", label: "#1 shoulder_pan", title: "#1 底座旋转 (shoulder_pan)", min: -110, max: 110, unit: "°", def: 0 },
  { key: "shoulder_lift", label: "#2 shoulder_lift", title: "#2 大臂俯仰 (shoulder_lift)", min: -100, max: 100, unit: "°", def: 0 },
  { key: "elbow_flex", label: "#3 elbow_flex", title: "#3 小臂肘弯 (elbow_flex)", min: -97, max: 97, unit: "°", def: 0 },
  { key: "wrist_flex", label: "#4 wrist_flex", title: "#4 手腕俯仰 (wrist_flex)", min: -95, max: 95, unit: "°", def: 0 },
  { key: "wrist_roll", label: "#5 wrist_roll", title: "#5 手腕滚转 (wrist_roll)", min: -157, max: 163, unit: "°", def: 0 },
  { key: "gripper", label: "#6 gripper", title: "#6 末端夹爪 (gripper)", min: 0, max: 100, unit: "%", def: 20 },
];

export function formatJointReadout(key, val, unit) {
  const num = Number(val ?? 0);
  const enc = String(estimateEncoder(key, num)).padStart(4, "0");
  return `${fmtZero(num, 1)}${unit} | ${enc}`;
}

export async function apiFetch(url, options = {}, onLog = null) {
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
      if (onLog) onLog(`请求提示 (${resp.status}): ${detail}`);
      return { ok: false, status: resp.status, data, detail };
    }
    return { ok: true, status: resp.status, data };
  } catch (err) {
    const msg = err?.message || String(err);
    if (onLog) onLog(`网络异常: ${msg}`);
    return { ok: false, status: 0, data: { detail: msg }, detail: msg };
  }
}

/**
 * Dual-Frame WebSocket Client (25~30Hz `telemetry` + event-driven `overlay_update`)
 */
export function createStudioSocket({ onTelemetry, onOverlay, onOpen, onClose } = {}) {
  let ws = null;
  let retryMs = 1000;
  let closedManually = false;

  const connect = () => {
    if (closedManually) return;
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const url = `${proto}//${location.host}/ws/telemetry`;
    try {
      ws = new WebSocket(url);
      ws.onopen = () => {
        retryMs = 1000;
        if (onOpen) onOpen();
      };
      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data);
          if (msg.type === "overlay_update") {
            if (onOverlay) onOverlay(msg);
          } else {
            if (onTelemetry) onTelemetry(msg);
          }
        } catch {
          // Ignore malformed frame
        }
      };
      ws.onclose = () => {
        if (onClose) onClose();
        if (!closedManually) {
          const delay = retryMs;
          retryMs = Math.min(8000, retryMs * 2);
          setTimeout(connect, delay);
        }
      };
      ws.onerror = () => {};
    } catch {
      if (onClose) onClose();
    }
  };

  connect();
  return {
    close() {
      closedManually = true;
      try {
        ws?.close();
      } catch {}
    },
  };
}

/**
 * Kernel-level Web Locks (`navigator.locks`) + BroadcastChannel Mutual Exclusion for `/sim-viewport`.
 * Immune to Chrome background tab timer throttling.
 */
const LOCK_NAME = "arm101_sim_viewport";
const CHANNEL_NAME = "arm101_stage_bus";
const POPUP_WIN_NAME = "Arm101SimViewport";
const THEME_STORAGE_KEY = "arm101_theme";

export function getStoredTheme() {
  try {
    const saved = localStorage.getItem(THEME_STORAGE_KEY);
    return saved === "dark" ? "dark" : "light";
  } catch {
    return "light";
  }
}

export function applyThemeToDocument(theme) {
  const normalized = theme === "dark" ? "dark" : "light";
  document.documentElement.dataset.theme = normalized;
  try {
    localStorage.setItem(THEME_STORAGE_KEY, normalized);
  } catch {}
  return normalized;
}

export class StagePopupMutex {
  constructor({
    role = "host",
    onStateChange = null,
    onRemoteCloseRequest = null,
    onThemeChange = null,
  } = {}) {
    this.role = role; // "host" | "popup"
    this.onStateChange = onStateChange;
    this.onRemoteCloseRequest = onRemoteCloseRequest;
    this.onThemeChange = onThemeChange;
    this.isPopupActive = false;
    this.popupWinRef = null;
    this._releaseLock = null;
    this._waitingOnLock = false;
    this.bc = typeof BroadcastChannel !== "undefined" ? new BroadcastChannel(CHANNEL_NAME) : null;

    if (this.bc) {
      this.bc.onmessage = (ev) => this._handleMessage(ev.data || {});
    }

    window.addEventListener("storage", (ev) => {
      if (ev.key === THEME_STORAGE_KEY && ev.newValue) {
        const nextTheme = ev.newValue === "dark" ? "dark" : "light";
        document.documentElement.dataset.theme = nextTheme;
        if (this.onThemeChange) this.onThemeChange(nextTheme);
      }
    });

    if (this.role === "popup") {
      this._initPopupHolder();
    } else {
      this._initHostWatcher();
    }
  }

  broadcastTheme(theme) {
    const normalized = applyThemeToDocument(theme);
    try {
      this.bc?.postMessage({ type: "THEME_CHANGED", theme: normalized, ts: Date.now() });
    } catch {}
    if (this.onThemeChange) this.onThemeChange(normalized);
    return normalized;
  }

  _setPopupActive(active) {
    const next = Boolean(active);
    if (this.isPopupActive === next) return;
    this.isPopupActive = next;
    if (this.onStateChange) this.onStateChange(next);
  }

  _initPopupHolder() {
    const announceOpen = () => {
      try {
        this.bc?.postMessage({ type: "POPUP_OPENED", ts: Date.now() });
      } catch {}
    };

    if (navigator.locks && navigator.locks.request) {
      navigator.locks.request(LOCK_NAME, { mode: "exclusive" }, () => {
        announceOpen();
        return new Promise((resolve) => {
          this._releaseLock = resolve;
        });
      });
    } else {
      announceOpen();
    }

    const cleanup = () => {
      if (this._releaseLock) {
        this._releaseLock();
        this._releaseLock = null;
      }
      try {
        this.bc?.postMessage({ type: "POPUP_CLOSED", ts: Date.now() });
      } catch {}
    };
    window.addEventListener("pagehide", cleanup);
    window.addEventListener("beforeunload", cleanup);
  }

  _initHostWatcher() {
    this.probePopupAlive();
    try {
      this.bc?.postMessage({ type: "PING", ts: Date.now() });
    } catch {}
  }

  async probePopupAlive() {
    if (this.role !== "host") return false;
    if (navigator.locks && navigator.locks.request) {
      let heldByPopup = false;
      await navigator.locks.request(LOCK_NAME, { ifAvailable: true }, async (lock) => {
        if (lock === null) {
          heldByPopup = true;
        }
      });
      this._setPopupActive(heldByPopup);
      if (heldByPopup && !this._waitingOnLock) {
        this._waitingOnLock = true;
        navigator.locks.request(LOCK_NAME, { mode: "shared" }, async () => {
          this._waitingOnLock = false;
          this._setPopupActive(false);
        });
      }
      return heldByPopup;
    }
    const open = Boolean(this.popupWinRef && !this.popupWinRef.closed);
    this._setPopupActive(open);
    return open;
  }

  _handleMessage(msg) {
    if (msg.type === "THEME_CHANGED" && msg.theme) {
      const nextTheme = msg.theme === "dark" ? "dark" : "light";
      document.documentElement.dataset.theme = nextTheme;
      if (this.onThemeChange) this.onThemeChange(nextTheme);
      return;
    }
    if (this.role === "host") {
      if (msg.type === "POPUP_OPENED") {
        this._setPopupActive(true);
        this.probePopupAlive();
      } else if (msg.type === "POPUP_CLOSED") {
        this._setPopupActive(false);
      }
    } else if (this.role === "popup") {
      if (msg.type === "PING") {
        try {
          this.bc?.postMessage({ type: "POPUP_OPENED", ts: Date.now() });
        } catch {}
      } else if (msg.type === "CLOSE_POPUP") {
        if (this._releaseLock) {
          this._releaseLock();
          this._releaseLock = null;
        }
        try {
          this.bc?.postMessage({ type: "POPUP_CLOSED", ts: Date.now() });
        } catch {}
        if (this.onRemoteCloseRequest) this.onRemoteCloseRequest();
        window.close();
      }
    }
  }

  openPopupWindow() {
    if (this.role !== "host") return null;
    if (this.popupWinRef && !this.popupWinRef.closed) {
      this.popupWinRef.focus();
      this._setPopupActive(true);
      return this.popupWinRef;
    }
    const win = window.open(
      "/sim-viewport",
      POPUP_WIN_NAME,
      "width=1120,height=760,menubar=no,toolbar=no,location=no,status=no"
    );
    if (win) {
      this.popupWinRef = win;
      this._setPopupActive(true);
      setTimeout(() => this.probePopupAlive(), 250);
    }
    return win;
  }

  recallPopupWindow() {
    try {
      this.bc?.postMessage({ type: "CLOSE_POPUP", ts: Date.now() });
    } catch {}
    if (this.popupWinRef && !this.popupWinRef.closed) {
      try {
        this.popupWinRef.close();
      } catch {}
    }
    this.popupWinRef = null;
    this._setPopupActive(false);
  }
}
