// Mirror of the sim's visual/physics constants (blender-navigatio.py) so the
// browser twin animates exactly like the Blender viewport.
export const PROP_MAX_SPEED = 720; // deg/s
export const PROP_VISUAL_MULT = 8.0; // prop spins this much faster visually
export const MAX_DEG = 28;
export const ROTOR_RADIUS = 0.15; // m
export const TILT_FACTOR = 0.025; // rad of lean per m/s body speed
export const TILT_MAX = (18 * Math.PI) / 180;
export const TILT_SMOOTH = 10.0;

// Palette (kept in sync with theme.css accents) for parts / vectors.
export const COLOR = {
  prop: "#29d4d4",
  vanePitch: "#8a63ff",
  vaneRoll: "#f5a623",
  thrust: "#1edd8a",
  velocity: "#1edd8a",
  force: "#f5a623",
  downwash: "#0070f3",
  body: "#3a3a3a",
  accent: "#0070f3",
  // named palette (kept in sync with theme.css)
  green: "#1edd8a",
  amber: "#f5a623",
  violet: "#8a63ff",
  cyan: "#29d4d4",
  red: "#ff5c5c",
};
