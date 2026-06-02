// Offline flight-trail profiling.
//
// The Blender sim dumps the autonomous run as a trail file:
//   { trail_pts: [[x,y,z], …], ground_z, coll_offset }
// That's *just* the flown path — no timestamps, no controller internals. This
// module reconstructs the rest so we can profile how the autonomous A→B command
// behaved: it differentiates the path into velocity/accel, splits it into legs
// against the mission waypoints, and *replays the goto controller's setpoint
// logic* over the recorded states so we can see how well the inner velocity
// loop (the PID) tracked what it was being asked to do.
//
// Everything here is pure + framework-free so it can be unit-reasoned about and
// reused by any view.

export type Vec3 = [number, number, number];

// Raw on-disk shape. `waypoints`/`hz`/`label` are optional forward-compatible
// extras — current sim dumps omit them, so we fall back to the config mission.
export interface RawTrail {
  trail_pts: Vec3[];
  ground_z: number;
  coll_offset: number;
  waypoints?: Vec3[];
  hz?: number;
  label?: string;
}

// ── controller constants, mirrored from config/config.yaml `goto:` ───────────
// These are what the autonomous command actually used, so replaying them
// reproduces the velocity setpoints the real PID chased.
export const GOTO = {
  posXyP: 1.2, // 1/s   horizontal position error → velocity setpoint
  posZP: 1.5, //  1/s   altitude error → climb-rate setpoint
  vMaxXy: 4.0, // m/s    horizontal speed cap
  vzMax: 2.5, //  m/s    climb/descent cap
  arrivalRadius: 0.5, // m   waypoint capture radius
  arrivalSpeed: 0.4, //  m/s waypoint capture speed
} as const;

// Default A→B mission (config/config.yaml `mission.waypoints`). Used when the
// trail file doesn't carry its own waypoint list.
export const DEFAULT_MISSION: Vec3[] = [
  [0.0, 0.0, 3.0],
  [12.0, 0.0, 7.0],
  [12.0, 12.0, 4.0],
  [-4.0, 8.0, 9.0],
];

export const DEFAULT_HZ = 50; // controller loop_rate_hz — our time assumption

export interface Sample {
  i: number;
  t: number; // s (assuming uniform sampling at `hz`)
  pos: Vec3;
  agl: number; // m above ground floor (ground_z + coll_offset)
  // velocity (m/s), central finite difference, lightly smoothed
  vx: number;
  vy: number;
  vz: number;
  speed: number; // 3-D speed
  speedH: number; // horizontal speed
  accel: number; // |dv/dt|
  course: number; // horizontal heading of travel, rad (NaN when ~stationary)
  // active autonomous target at this instant + reconstructed setpoints
  wpIndex: number; // which leg/waypoint is active
  posErr: number; // distance to active target
  vzSp: number; // commanded climb-rate setpoint
  speedHSp: number; // commanded horizontal speed setpoint
  trackErrH: number; // |horizontal velocity setpoint − actual|  (inner-loop error)
  trackErrZ: number; // climb-rate setpoint − actual
}

export interface Leg {
  wpIndex: number;
  target: Vec3;
  targetAgl: number;
  startIdx: number;
  endIdx: number;
  startT: number;
  endT: number;
  duration: number;
  pathLen: number; // distance actually flown
  straightLen: number; // start → target straight line
  efficiency: number; // straight / path  (1 = perfectly direct)
  meanSpeed: number;
  maxSpeed: number;
  maxClimb: number; // signed peak |vz| in leg direction
  altStart: number;
  altEnd: number;
  arrivalErr: number; // closest approach to the waypoint (m)
  // altitude step-response metrics (the throttle/climb PID, treated as SISO)
  riseTime: number; // 10→90 % of the altitude step (s, NaN if no step)
  overshoot: number; // % beyond the target altitude step
  settleTime: number; // s until altitude stays within ±settleBand of target
  steadyErr: number; // |altEnd − targetAgl|
  captured: boolean; // got inside arrival radius
}

export interface Profile {
  label: string;
  hz: number;
  dt: number;
  floor: number; // ground_z + coll_offset (world Z of the ground surface)
  groundZ: number;
  samples: Sample[];
  legs: Leg[];
  waypoints: Vec3[];
  // bounds for plotting / framing
  duration: number;
  bbox: { min: Vec3; max: Vec3 };
  // headline numbers
  summary: {
    totalPath: number;
    netDisplacement: number;
    routeEfficiency: number;
    maxSpeed: number;
    maxClimb: number; // signed extreme of vz
    maxAlt: number;
    maxAccel: number;
    waypointsCaptured: number;
    waypointsTotal: number;
    rmsTrackH: number; // RMS horizontal velocity-tracking error
    rmsTrackZ: number; // RMS climb-rate tracking error
    peakPosErr: number;
  };
}

const SETTLE_BAND = 0.15; // m — altitude "settled" tolerance

const dist = (a: Vec3, b: Vec3) =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

const clamp = (v: number, lo: number, hi: number) =>
  v < lo ? lo : v > hi ? hi : v;

// 3-tap moving average over a scalar series, NaN-safe at the ends.
function smooth(xs: number[]): number[] {
  const n = xs.length;
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    const a = xs[i - 1] ?? xs[i];
    const b = xs[i];
    const c = xs[i + 1] ?? xs[i];
    out[i] = (a + b + c) / 3;
  }
  return out;
}

export class ProfileError extends Error {}

/** Validate + normalize a parsed JSON blob into a RawTrail. Throws on garbage. */
export function coerceRawTrail(data: unknown): RawTrail {
  if (!data || typeof data !== "object")
    throw new ProfileError("File is not a JSON object.");
  const d = data as Record<string, unknown>;
  const pts = d.trail_pts;
  if (!Array.isArray(pts) || pts.length < 2)
    throw new ProfileError("Missing or too-short `trail_pts` array (need ≥ 2 points).");
  const trail_pts: Vec3[] = [];
  for (const p of pts) {
    if (
      !Array.isArray(p) || p.length < 3 ||
      !p.slice(0, 3).every((c) => typeof c === "number" && Number.isFinite(c))
    )
      throw new ProfileError("`trail_pts` must be an array of [x, y, z] numbers.");
    trail_pts.push([p[0], p[1], p[2]]);
  }
  const num = (v: unknown, dflt: number) => (typeof v === "number" && Number.isFinite(v) ? v : dflt);
  const wp =
    Array.isArray(d.waypoints) && d.waypoints.every((w) => Array.isArray(w) && w.length >= 3)
      ? (d.waypoints as number[][]).map((w) => [w[0], w[1], w[2]] as Vec3)
      : undefined;
  return {
    trail_pts,
    ground_z: num(d.ground_z, 0),
    coll_offset: num(d.coll_offset, 0),
    waypoints: wp,
    hz: typeof d.hz === "number" && d.hz > 0 ? d.hz : undefined,
    label: typeof d.label === "string" ? d.label : undefined,
  };
}

/**
 * Find each waypoint's closest-approach index, scanning forward so legs stay
 * ordered. Returns one [startIdx, endIdx] span per waypoint.
 */
function segment(pts: Vec3[], waypoints: Vec3[]): { startIdx: number; endIdx: number; arrivalErr: number }[] {
  const out: { startIdx: number; endIdx: number; arrivalErr: number }[] = [];
  const n = pts.length;
  let from = 0;
  let prevStart = 0;
  waypoints.forEach((w, wi) => {
    // last waypoint: scan to the end; otherwise leave room for the rest.
    const limit = wi === waypoints.length - 1 ? n : n;
    let best = from;
    let bestD = Infinity;
    for (let i = from; i < limit; i++) {
      const dd = dist(pts[i], w);
      if (dd < bestD) {
        bestD = dd;
        best = i;
      }
    }
    out.push({ startIdx: prevStart, endIdx: Math.max(best, prevStart), arrivalErr: bestD });
    prevStart = best;
    from = Math.min(best + 1, n - 1);
  });
  return out;
}

/** Altitude step-response metrics for one leg (treat altitude as a SISO step). */
function altMetrics(
  agl: number[],
  startIdx: number,
  endIdx: number,
  targetAgl: number,
  dt: number,
): Pick<Leg, "riseTime" | "overshoot" | "settleTime" | "steadyErr"> {
  const a0 = agl[startIdx];
  const step = targetAgl - a0;
  const steadyErr = Math.abs(agl[endIdx] - targetAgl);
  if (Math.abs(step) < 0.2) {
    // no meaningful altitude command on this leg
    return { riseTime: NaN, overshoot: 0, settleTime: NaN, steadyErr };
  }
  const dir = Math.sign(step);
  // rise time: 10 % → 90 % of the step
  let t10 = NaN;
  let t90 = NaN;
  for (let i = startIdx; i <= endIdx; i++) {
    const frac = (agl[i] - a0) / step;
    if (Number.isNaN(t10) && frac >= 0.1) t10 = i;
    if (Number.isNaN(t90) && frac >= 0.9) {
      t90 = i;
      break;
    }
  }
  const riseTime = !Number.isNaN(t10) && !Number.isNaN(t90) ? (t90 - t10) * dt : NaN;
  // overshoot: furthest the altitude went past the target, as % of the step
  let peakBeyond = 0;
  for (let i = startIdx; i <= endIdx; i++) {
    const beyond = dir * (agl[i] - targetAgl);
    if (beyond > peakBeyond) peakBeyond = beyond;
  }
  const overshoot = (peakBeyond / Math.abs(step)) * 100;
  // settling time: last time it left the ±band, measured from leg start
  let settleIdx = endIdx;
  for (let i = endIdx; i >= startIdx; i--) {
    if (Math.abs(agl[i] - targetAgl) > SETTLE_BAND) {
      settleIdx = Math.min(i + 1, endIdx);
      break;
    }
    if (i === startIdx) settleIdx = startIdx;
  }
  const settleTime = (settleIdx - startIdx) * dt;
  return { riseTime, overshoot, settleTime, steadyErr };
}

/** Full analysis pipeline: raw trail → rich Profile. */
export function analyze(raw: RawTrail): Profile {
  const pts = raw.trail_pts;
  const n = pts.length;
  const hz = raw.hz ?? DEFAULT_HZ;
  const dt = 1 / hz;
  const floor = raw.ground_z + raw.coll_offset;
  const waypoints = raw.waypoints && raw.waypoints.length ? raw.waypoints : DEFAULT_MISSION;

  // ── velocity via central differences, then lightly smoothed ────────────────
  const vxR = new Array<number>(n);
  const vyR = new Array<number>(n);
  const vzR = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(n - 1, i + 1)];
    const span = (Math.min(n - 1, i + 1) - Math.max(0, i - 1)) * dt || dt;
    vxR[i] = (b[0] - a[0]) / span;
    vyR[i] = (b[1] - a[1]) / span;
    vzR[i] = (b[2] - a[2]) / span;
  }
  const vx = smooth(vxR);
  const vy = smooth(vyR);
  const vz = smooth(vzR);

  const aglArr = pts.map((p) => p[2] - floor);

  // ── leg segmentation against the mission ───────────────────────────────────
  const spans = segment(pts, waypoints);

  // active-waypoint index per sample (for setpoint reconstruction + charts)
  const wpOf = new Array<number>(n).fill(waypoints.length - 1);
  spans.forEach((s, wi) => {
    for (let i = s.startIdx; i <= s.endIdx; i++) wpOf[i] = wi;
  });

  // ── per-sample reconstruction of the goto controller's setpoints ───────────
  const samples: Sample[] = new Array(n);
  let prevSpeed = 0;
  for (let i = 0; i < n; i++) {
    const p = pts[i];
    const tgt = waypoints[wpOf[i]];
    const ex = tgt[0] - p[0];
    const ey = tgt[1] - p[1];
    const ez = tgt[2] - p[2];
    const errH = Math.hypot(ex, ey);
    const speedHSp = Math.min(GOTO.posXyP * errH, GOTO.vMaxXy);
    const vzSp = clamp(GOTO.posZP * ez, -GOTO.vzMax, GOTO.vzMax);
    // horizontal velocity setpoint vector (toward target)
    const ux = errH > 1e-6 ? ex / errH : 0;
    const uy = errH > 1e-6 ? ey / errH : 0;
    const vxSp = speedHSp * ux;
    const vySp = speedHSp * uy;
    const speedH = Math.hypot(vx[i], vy[i]);
    const speed = Math.hypot(vx[i], vy[i], vz[i]);
    const accel = i === 0 ? 0 : Math.abs(speed - prevSpeed) / dt;
    prevSpeed = speed;
    samples[i] = {
      i,
      t: i * dt,
      pos: p,
      agl: aglArr[i],
      vx: vx[i],
      vy: vy[i],
      vz: vz[i],
      speed,
      speedH,
      accel,
      course: speedH > 0.1 ? Math.atan2(vy[i], vx[i]) : NaN,
      wpIndex: wpOf[i],
      posErr: Math.hypot(ex, ey, ez),
      vzSp,
      speedHSp,
      trackErrH: Math.hypot(vxSp - vx[i], vySp - vy[i]),
      trackErrZ: vzSp - vz[i],
    };
  }

  // ── per-leg rollups + step-response metrics ────────────────────────────────
  const legs: Leg[] = spans.map((s, wi) => {
    const target = waypoints[wi];
    const targetAgl = target[2] - floor;
    let pathLen = 0;
    let maxSpeed = 0;
    let maxClimb = 0;
    let sumSpeed = 0;
    let cnt = 0;
    for (let i = s.startIdx; i <= s.endIdx; i++) {
      if (i > s.startIdx) pathLen += dist(pts[i - 1], pts[i]);
      const sp = samples[i].speed;
      if (sp > maxSpeed) maxSpeed = sp;
      if (Math.abs(samples[i].vz) > Math.abs(maxClimb)) maxClimb = samples[i].vz;
      sumSpeed += sp;
      cnt++;
    }
    const straightLen = dist(pts[s.startIdx], target);
    const duration = (s.endIdx - s.startIdx) * dt;
    const am = altMetrics(aglArr, s.startIdx, s.endIdx, targetAgl, dt);
    return {
      wpIndex: wi,
      target,
      targetAgl,
      startIdx: s.startIdx,
      endIdx: s.endIdx,
      startT: s.startIdx * dt,
      endT: s.endIdx * dt,
      duration,
      pathLen,
      straightLen,
      efficiency: pathLen > 1e-6 ? straightLen / pathLen : 0,
      meanSpeed: cnt ? sumSpeed / cnt : 0,
      maxSpeed,
      maxClimb,
      altStart: aglArr[s.startIdx],
      altEnd: aglArr[s.endIdx],
      arrivalErr: s.arrivalErr,
      captured: s.arrivalErr <= GOTO.arrivalRadius,
      ...am,
    };
  });

  // ── bounds + headline summary ──────────────────────────────────────────────
  const min: Vec3 = [Infinity, Infinity, Infinity];
  const max: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (const p of pts) {
    for (let k = 0; k < 3; k++) {
      if (p[k] < min[k]) min[k] = p[k];
      if (p[k] > max[k]) max[k] = p[k];
    }
  }
  let totalPath = 0;
  for (let i = 1; i < n; i++) totalPath += dist(pts[i - 1], pts[i]);
  const netDisplacement = dist(pts[0], pts[n - 1]);
  let maxSpeed = 0;
  let maxClimb = 0;
  let maxAccel = 0;
  let maxAlt = -Infinity;
  let sumH2 = 0;
  let sumZ2 = 0;
  let peakPosErr = 0;
  for (const s of samples) {
    if (s.speed > maxSpeed) maxSpeed = s.speed;
    if (Math.abs(s.vz) > Math.abs(maxClimb)) maxClimb = s.vz;
    if (s.accel > maxAccel) maxAccel = s.accel;
    if (s.agl > maxAlt) maxAlt = s.agl;
    sumH2 += s.trackErrH * s.trackErrH;
    sumZ2 += s.trackErrZ * s.trackErrZ;
    if (s.posErr > peakPosErr) peakPosErr = s.posErr;
  }

  return {
    label: raw.label ?? "",
    hz,
    dt,
    floor,
    groundZ: raw.ground_z,
    samples,
    legs,
    waypoints,
    duration: (n - 1) * dt,
    bbox: { min, max },
    summary: {
      totalPath,
      netDisplacement,
      routeEfficiency: totalPath > 1e-6 ? netDisplacement / totalPath : 0,
      maxSpeed,
      maxClimb,
      maxAlt,
      maxAccel,
      waypointsCaptured: legs.filter((l) => l.captured).length,
      waypointsTotal: waypoints.length,
      rmsTrackH: Math.sqrt(sumH2 / n),
      rmsTrackZ: Math.sqrt(sumZ2 / n),
      peakPosErr,
    },
  };
}

// ── small shared helpers for the view ────────────────────────────────────────

/** blue→cyan→green→amber→red ramp, t in [0,1]; used to colour by speed. */
export function speedColor(t: number): [number, number, number] {
  const stops: [number, [number, number, number]][] = [
    [0.0, [0.0, 0.44, 0.95]],
    [0.35, [0.16, 0.83, 0.83]],
    [0.6, [0.12, 0.87, 0.54]],
    [0.8, [0.96, 0.65, 0.14]],
    [1.0, [1.0, 0.36, 0.36]],
  ];
  const c = clamp(t, 0, 1);
  for (let i = 1; i < stops.length; i++) {
    if (c <= stops[i][0]) {
      const [p0, a] = stops[i - 1];
      const [p1, b] = stops[i];
      const f = (c - p0) / (p1 - p0 || 1);
      return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
    }
  }
  return stops[stops.length - 1][1];
}

export const fmt = (v: number, digits = 2) => (Number.isFinite(v) ? v.toFixed(digits) : "—");
