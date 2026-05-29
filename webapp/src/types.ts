// Wire protocol shared with sim_bridge/web_bridge.py

export interface Telemetry {
  t: number;
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
  yaw: number; // radians
  prop_speed: number; // deg/s
}

// Four independent vane angles (radians). Vanes 1 & 3 → body-X (fore/aft),
// vanes 2 & 4 → body-Y (lateral).
export interface Command {
  throttle: number; // [0,1]
  vane1: number;
  vane2: number;
  vane3: number;
  vane4: number;
}

export interface PidTerm {
  p: number;
  i: number;
  d: number;
  out: number;
  setpoint: number;
  measurement: number;
}

// Only the vertical (altitude → climb-rate → accel) loop exists now.
export interface PidBlock {
  alt: PidTerm;
}

export interface StateMsg {
  type: "state";
  telemetry: Telemetry;
  command: Command;
  status: string;
  pid: PidBlock | null;
}

export interface MetaMsg {
  type: "meta";
  source: "demo" | "mqtt";
  drone: {
    mass: number;
    gravity: number;
    thrust_max: number;
    prop_max_speed: number;
    max_vane_deg: number;
    rotor_radius: number;
  };
  hover_throttle: number;
  ground_z: number;
  target_altitude: number;
}

// One flattened sample kept in the rolling history (for charts).
export interface HistorySample {
  t: number; // seconds (sim time)
  // telemetry
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
  speed: number;
  yaw: number;
  rpm: number;
  // command
  throttle: number;
  v1: number; v2: number; v3: number; v4: number; // degrees
  // altitude PID (NaN when unavailable)
  altP: number;
  altI: number;
  altD: number;
  altOut: number;
  altSp: number; // climb-rate setpoint (m/s)
}
