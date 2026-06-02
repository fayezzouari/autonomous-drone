// Single source of truth for live sim data.
//
// The bridge pushes state at ~50 Hz. Re-rendering the React tree that often
// would be wasteful, so this store keeps the hot data in plain mutable fields
// that the 3D scene and charts read every animation frame (via useFrame / rAF),
// and only notifies React listeners for *low-frequency* changes (connection
// state, meta, status string). That keeps the UI smooth.

import type { HistorySample, MetaMsg, Obstacle, ObstaclesMsg, StateMsg } from "./types";

const BRIDGE_URL =
  (import.meta.env.VITE_BRIDGE_URL as string | undefined) ??
  `ws://${location.hostname || "localhost"}:8765`;

const HISTORY_LIMIT = 6000; // ~120 s at 50 Hz

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
  hasImu = false;
  // Obstacle boxes the navigator plans around. They change rarely, so a counter
  // bumped on each new set lets React (and useSyncExternalStore) re-render only
  // when the world actually changes.
  obstacles: Obstacle[] = [];
  obstaclesVersion = 0;

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
    let msg: MetaMsg | StateMsg | ObstaclesMsg;
    try {
      msg = JSON.parse(raw);
    } catch {
      return;
    }
    if (msg.type === "meta") {
      this.meta = msg;
      this.source = msg.source;
      this.setObstacles(msg.obstacles ?? []);
      this.bump();
      return;
    }
    if (msg.type === "obstacles") {
      this.setObstacles(msg.obstacles);
      this.bump();
      return;
    }
    // state — hot path, no React notify unless status/pid-availability changes.
    this.latest = msg;
    this.pushHistory(msg);
    const hadPid = this.hasPid;
    const hadImu = this.hasImu;
    this.hasPid = msg.pid != null;
    this.hasImu = msg.imu != null;
    if (msg.status !== this.status || hadPid !== this.hasPid || hadImu !== this.hasImu) {
      this.status = msg.status;
      this.bump();
    }
  }

  private setObstacles(obs: Obstacle[]) {
    this.obstacles = obs;
    this.obstaclesVersion++;
  }

  private pushHistory(s: StateMsg) {
    const t = s.telemetry;
    const c = s.command;
    const alt = s.pid?.alt;
    const deg = 180 / Math.PI;
    const sample: HistorySample = {
      t: t.t,
      // Wall-clock stamp (monotonic, seconds) used as the chart x-axis. Sim
      // time (t.t) is 0 on real hardware (no telemetry source), so keying the
      // plots off it stalls the trace — this advances regardless of source.
      clock: performance.now() / 1000,
      x: t.x, y: t.y, z: t.z,
      vx: t.vx, vy: t.vy, vz: t.vz,
      speed: Math.hypot(t.vx, t.vy, t.vz),
      yaw: t.yaw,
      rpm: (t.prop_speed / 360) * 60,
      throttle: c.throttle,
      v1: c.vane1 * deg, v2: c.vane2 * deg, v3: c.vane3 * deg, v4: c.vane4 * deg,
      altP: alt?.p ?? NaN,
      altI: alt?.i ?? NaN,
      altD: alt?.d ?? NaN,
      altOut: alt?.out ?? NaN,
      altSp: alt?.setpoint ?? NaN,
      imuYaw: s.imu?.yaw ?? NaN,
      imuPitch: s.imu?.pitch ?? NaN,
      imuRoll: s.imu?.roll ?? NaN,
      gz: s.imu?.gz ?? NaN,
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
      hasImu: this.hasImu,
      obstaclesVersion: this.obstaclesVersion,
    };
  }
  private bump() {
    this.snap = this.buildSnap();
    this.listeners.forEach((l) => l());
  }
}

export const store = new SimStore();
