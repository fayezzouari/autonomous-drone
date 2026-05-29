// Single source of truth for live sim data.
//
// The bridge pushes state at ~50 Hz. Re-rendering the React tree that often
// would be wasteful, so this store keeps the hot data in plain mutable fields
// that the 3D scene and charts read every animation frame (via useFrame / rAF),
// and only notifies React listeners for *low-frequency* changes (connection
// state, meta, status string). That keeps the UI smooth.

import type { HistorySample, MetaMsg, StateMsg } from "./types";

const BRIDGE_URL =
  (import.meta.env.VITE_BRIDGE_URL as string | undefined) ??
  `ws://${location.hostname || "localhost"}:8765`;

const HISTORY_LIMIT = 1200; // ~24 s at 50 Hz

type Listener = () => void;

class SimStore {
  // ── hot, read every frame (no React notify) ───────────────────────────────
  latest: StateMsg | null = null;
  history: HistorySample[] = [];

  // ── cold, drives React re-renders when changed ────────────────────────────
  meta: MetaMsg | null = null;
  connected = false;
  status = "connecting…";
  source: "demo" | "mqtt" | "—" = "—";
  hasPid = false;

  private listeners = new Set<Listener>();
  private ws: WebSocket | null = null;
  private reconnectTimer: number | null = null;
  // snapshot object reused so useSyncExternalStore sees a stable ref until change
  private snap = this.buildSnap();

  url = BRIDGE_URL;

  connect() {
    if (this.ws && (this.ws.readyState === 0 || this.ws.readyState === 1)) return;
    try {
      this.ws = new WebSocket(this.url);
    } catch {
      this.scheduleReconnect();
      return;
    }
    this.ws.onopen = () => {
      this.connected = true;
      this.status = "connected";
      this.bump();
    };
    this.ws.onclose = () => {
      this.connected = false;
      this.status = "disconnected — retrying";
      this.bump();
      this.scheduleReconnect();
    };
    this.ws.onerror = () => this.ws?.close();
    this.ws.onmessage = (ev) => this.onMessage(ev.data as string);
  }

  private scheduleReconnect() {
    if (this.reconnectTimer != null) return;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, 1200);
  }

  private onMessage(raw: string) {
    let msg: MetaMsg | StateMsg;
    try {
      msg = JSON.parse(raw);
    } catch {
      return;
    }
    if (msg.type === "meta") {
      this.meta = msg;
      this.source = msg.source;
      this.bump();
      return;
    }
    // state — hot path, no React notify unless status/pid-availability changes.
    this.latest = msg;
    this.pushHistory(msg);
    const hadPid = this.hasPid;
    this.hasPid = msg.pid != null;
    if (msg.status !== this.status || hadPid !== this.hasPid) {
      this.status = msg.status;
      this.bump();
    }
  }

  private pushHistory(s: StateMsg) {
    const t = s.telemetry;
    const c = s.command;
    const pid = s.pid;
    const sp = s.setpoint;
    const speed = Math.hypot(t.vx, t.vy, t.vz);
    const sample: HistorySample = {
      t: t.t,
      x: t.x, y: t.y, z: t.z,
      vx: t.vx, vy: t.vy, vz: t.vz,
      speed,
      yaw: t.yaw,
      rpm: (t.prop_speed / 360) * 60,
      throttle: c.throttle,
      pitchDeg: (c.pitch * 180) / Math.PI,
      rollDeg: (c.roll * 180) / Math.PI,
      px: pid?.vx.p ?? NaN, ix: pid?.vx.i ?? NaN, dx: pid?.vx.d ?? NaN, ox: pid?.vx.out ?? NaN, spx: sp?.vx ?? NaN,
      py: pid?.vy.p ?? NaN, iy: pid?.vy.i ?? NaN, dy: pid?.vy.d ?? NaN, oy: pid?.vy.out ?? NaN, spy: sp?.vy ?? NaN,
      pz: pid?.vz.p ?? NaN, iz: pid?.vz.i ?? NaN, dz: pid?.vz.d ?? NaN, oz: pid?.vz.out ?? NaN, spz: sp?.vz ?? NaN,
    };
    const h = this.history;
    h.push(sample);
    if (h.length > HISTORY_LIMIT) h.splice(0, h.length - HISTORY_LIMIT);
  }

  // ── React glue (useSyncExternalStore) ─────────────────────────────────────
  subscribe = (l: Listener) => {
    this.listeners.add(l);
    return () => this.listeners.delete(l);
  };
  getSnapshot = () => this.snap;

  private buildSnap() {
    return {
      connected: this.connected,
      status: this.status,
      source: this.source,
      meta: this.meta,
      hasPid: this.hasPid,
    };
  }
  private bump() {
    this.snap = this.buildSnap();
    this.listeners.forEach((l) => l());
  }
}

export const store = new SimStore();
</content>
