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

export interface Command {
  throttle: number; // [0,1]
  pitch: number; // rad, Vanes 1-3
  roll: number; // rad, Vanes 2-4
}

export interface PidTerm {
  p: number;
  i: number;
  d: number;
  out: number;
  error: number;
  setpoint: number;
  measurement: number;
}

export interface PidBlock {
  vx: PidTerm;
  vy: PidTerm;
  vz: PidTerm;
}

export interface StateMsg {
  type: "state";
  telemetry: Telemetry;
  command: Command;
  status: string;
  target_index: number | null;
  pid: PidBlock | null;
  setpoint: { vx: number; vy: number; vz: number } | null;
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
  waypoints: [number, number, number][];
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
  pitchDeg: number;
  rollDeg: number;
  // pid (NaN when unavailable)
  px: number; ix: number; dx: number; ox: number; spx: number;
  py: number; iy: number; dy: number; oy: number; spy: number;
  pz: number; iz: number; dz: number; oz: number; spz: number;
}
</content>
