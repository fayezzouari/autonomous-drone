import { useMemo, useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Grid, Html, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import Singlecopter, { CopterRefs } from "./Singlecopter";
import { store } from "../store";
import {
  COLOR, MAX_DEG, PROP_MAX_SPEED, PROP_VISUAL_MULT, TILT_FACTOR, TILT_MAX, TILT_SMOOTH,
} from "../consts";
import Hud from "./Hud";
import { useSimSnapshot } from "../hooks";
import CanvasBoundary from "./CanvasBoundary";

const MAX_RAD = (MAX_DEG * Math.PI) / 180;
const DEG = Math.PI / 180;
// Where to float the twin when there is no position source (real-hardware manual
// flight has no GPS/VICON), so the live IMU attitude stays clearly in view.
const DISPLAY_HOVER_Z = 1.5;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

function FlyingDrone() {
  const root = useRef<THREE.Group>(null); // translation only
  const body = useRef<THREE.Group>(null); // yaw + lean
  const copter = useRef<CopterRefs>(null);
  const tilt = useRef({ x: 0, y: 0 });

  // attitude quaternion scratch (used when a real IMU drives the orientation)
  const Z = useMemo(() => new THREE.Vector3(0, 0, 1), []);
  const Y = useMemo(() => new THREE.Vector3(0, 1, 0), []);
  const X = useMemo(() => new THREE.Vector3(1, 0, 0), []);
  const qy = useMemo(() => new THREE.Quaternion(), []);
  const qp = useMemo(() => new THREE.Quaternion(), []);
  const qr = useMemo(() => new THREE.Quaternion(), []);
  const qTarget = useMemo(() => new THREE.Quaternion(), []);

  // vector overlays (created once)
  const vel = useMemo(() => new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.001, COLOR.velocity, 0.12, 0.07), []);
  const force = useMemo(() => new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.001, COLOR.force, 0.12, 0.07), []);
  const downMat = useMemo(() => new THREE.MeshBasicMaterial({ color: COLOR.downwash, transparent: true, opacity: 0, side: THREE.DoubleSide }), []);

  useFrame((_, dtRaw) => {
    const s = store.latest;
    const c = copter.current;
    if (!s || !root.current || !body.current || !c) return;
    const dt = Math.min(dtRaw, 0.05);
    const t = s.telemetry, cmd = s.command, imu = s.imu;
    // A sim / position source is publishing real telemetry (sim time advances);
    // on real-hardware manual flight there is none, only IMU + commands.
    const hasTelem = t.t > 0;

    // ── position ──────────────────────────────────────────────────────────────
    if (!hasTelem && (imu || s.hw)) {
      root.current.position.set(0, 0, DISPLAY_HOVER_Z); // hardware: no position sensor
    } else {
      root.current.position.set(t.x, t.y, t.z);
    }

    // ── attitude ──────────────────────────────────────────────────────────────
    const cy = Math.cos(t.yaw), sy = Math.sin(t.yaw);
    if (imu) {
      // real flight-controller attitude (aerospace ZYX): world = Rz·Ry·Rx
      qy.setFromAxisAngle(Z, imu.yaw * DEG);
      qp.setFromAxisAngle(Y, imu.pitch * DEG);
      qr.setFromAxisAngle(X, imu.roll * DEG);
      qTarget.copy(qy).multiply(qp).multiply(qr);
      body.current.quaternion.slerp(qTarget, 1 - Math.exp(-dt * 18));
    } else {
      // sim: velocity lean in body frame + telemetry yaw
      const vxb = t.vx * cy + t.vy * sy;
      const vyb = -t.vx * sy + t.vy * cy;
      const tgtX = clamp(-vyb * TILT_FACTOR, -TILT_MAX, TILT_MAX);
      const tgtY = clamp(vxb * TILT_FACTOR, -TILT_MAX, TILT_MAX);
      const a = Math.min(1, TILT_SMOOTH * dt);
      tilt.current.x += (tgtX - tilt.current.x) * a;
      tilt.current.y += (tgtY - tilt.current.y) * a;
      body.current.rotation.set(tilt.current.x, tilt.current.y, t.yaw);
    }

    // prop spin + glow — hardware sends no prop_speed, so drive the rotor from
    // throttle (drone/hw, else the command) so the disc still reacts to the stick.
    const thr = s.hw?.throttle ?? cmd.throttle;
    let propSpeed = t.prop_speed; // deg/s
    if (propSpeed < 1 && !hasTelem) propSpeed = clamp(thr, 0, 1) * PROP_MAX_SPEED;
    const propFrac = propSpeed / PROP_MAX_SPEED;
    const thrustFrac = propFrac * propFrac;
    if (c.prop) c.prop.rotation.z += (propSpeed * PROP_VISUAL_MULT * dt * Math.PI) / 180;
    c.matProp.emissiveIntensity = 0.08 + 0.6 * propFrac;
    c.discMat.opacity = 0.04 + 0.16 * thrustFrac;

    // four independent vanes: v1/v3 hinge about Y (fore/aft), v2/v4 about X (lateral)
    const angles = [cmd.vane1, cmd.vane2, cmd.vane3, cmd.vane4];
    if (c.vanes[0]) c.vanes[0].rotation.y = -cmd.vane1;
    if (c.vanes[2]) c.vanes[2].rotation.y = -cmd.vane3;
    if (c.vanes[1]) c.vanes[1].rotation.x = cmd.vane2;
    if (c.vanes[3]) c.vanes[3].rotation.x = cmd.vane4;
    c.vaneMats.forEach((m, i) => { m.emissiveIntensity = clamp(Math.abs(angles[i]) / MAX_RAD, 0, 1) * 0.9; });

    // velocity vector (green)
    const speed = Math.hypot(t.vx, t.vy, t.vz);
    if (speed > 0.05) {
      vel.setDirection(new THREE.Vector3(t.vx, t.vy, t.vz).normalize());
      vel.setLength((Math.min(speed, 5) / 5) * 1.6 + 0.1, 0.12, 0.07);
      vel.visible = true;
    } else vel.visible = false;

    // lateral aerodynamic force (orange), body→world like the sim
    const fxb = -0.5 * (Math.sin(cmd.vane1) + Math.sin(cmd.vane3));
    const fyb = 0.5 * (Math.sin(cmd.vane2) + Math.sin(cmd.vane4));
    const fx = fxb * cy - fyb * sy;
    const fy = fxb * sy + fyb * cy;
    const fmag = Math.hypot(fx, fy);
    if (fmag > 0.02 && thrustFrac > 0.02) {
      force.setDirection(new THREE.Vector3(fx, fy, 0).normalize());
      force.setLength(fmag * 1.4 + 0.1, 0.12, 0.07);
      force.visible = true;
    } else force.visible = false;

    // downwash cone opacity
    downMat.opacity = 0.03 + 0.22 * thrustFrac;
  });

  return (
    <group ref={root}>
      <group ref={body}>
        <Singlecopter ref={copter} />
        {/* downwash cone under the prop (points down) */}
        <mesh position={[0, 0, -0.05]} rotation={[Math.PI / 2, 0, 0]} material={downMat}>
          <coneGeometry args={[0.26, 0.42, 28, 1, true]} />
        </mesh>
      </group>
      <primitive object={vel} />
      <primitive object={force} />
    </group>
  );
}

function Trail() {
  const MAX = 260;
  const acc = useRef(0);
  const count = useRef(0);
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const arr = new Float32Array(MAX * 3);
    g.setAttribute("position", new THREE.BufferAttribute(arr, 3));
    g.setDrawRange(0, 0);
    return g;
  }, []);
  const mat = useMemo(() => new THREE.LineBasicMaterial({ color: COLOR.accent, transparent: true, opacity: 0.6 }), []);

  useFrame((_, dt) => {
    const s = store.latest;
    if (!s || s.telemetry.t <= 0) return; // only trace when a position source is live
    acc.current += dt;
    if (acc.current < 0.04) return;
    acc.current = 0;
    const pos = geom.getAttribute("position") as THREE.BufferAttribute;
    const arr = pos.array as Float32Array;
    if (count.current < MAX) {
      const i = count.current * 3;
      arr[i] = s.telemetry.x; arr[i + 1] = s.telemetry.y; arr[i + 2] = s.telemetry.z;
      count.current++;
    } else {
      arr.copyWithin(0, 3);
      const i = (MAX - 1) * 3;
      arr[i] = s.telemetry.x; arr[i + 1] = s.telemetry.y; arr[i + 2] = s.telemetry.z;
    }
    geom.setDrawRange(0, count.current);
    pos.needsUpdate = true;
  });

  return <line>{/* @ts-ignore drei line */}<primitive object={geom} attach="geometry" /><primitive object={mat} attach="material" /></line>;
}

// The controller holds a target altitude; show it as a translucent disc the
// drone settles onto, with a live label.
function AltitudeTarget() {
  const snap = useSimSnapshot();
  const z = snap.meta?.target_altitude;
  if (z == null) return null;
  return (
    <group position={[0, 0, z]}>
      <mesh rotation={[0, 0, 0]}>
        <ringGeometry args={[3.6, 3.7, 64]} />
        <meshBasicMaterial color={COLOR.accent} transparent opacity={0.6} side={THREE.DoubleSide} />
      </mesh>
      <mesh>
        <circleGeometry args={[3.7, 64]} />
        <meshBasicMaterial color={COLOR.accent} transparent opacity={0.05} side={THREE.DoubleSide} />
      </mesh>
      <Html center distanceFactor={12} style={{ pointerEvents: "none" }} position={[3.7, 0, 0]}>
        <div style={{ font: "11px ui-monospace, monospace", color: COLOR.accent, background: "rgba(0,0,0,0.5)", padding: "1px 6px", borderRadius: 5, whiteSpace: "nowrap" }}>
          hold {z.toFixed(1)} m
        </div>
      </Html>
    </group>
  );
}

export default function SimulationViewport() {
  return (
    <div className="card canvas-card">
      <div className="card-head">
        <span className="card-title">Simulation Viewport</span>
        <span className="card-sub">live digital twin · orbit to look around</span>
      </div>
      <div className="canvas-wrap">
        <CanvasBoundary label="Viewport">
        <Canvas
          shadows
          dpr={[1, 2]}
          camera={{ position: [6, -7, 4.5], fov: 50, near: 0.05, far: 500 }}
          onCreated={({ camera }) => {
            camera.up.set(0, 0, 1);
            camera.lookAt(0, 0, 1.5);
          }}
        >
          <color attach="background" args={["#050505"]} />
          <fog attach="fog" args={["#050505", 18, 45]} />
          <hemisphereLight args={["#bcd4ff", "#1a1a22", 0.7]} />
          <directionalLight position={[6, -4, 10]} intensity={1.4} castShadow shadow-mapSize={[1024, 1024]} />
          <ambientLight intensity={0.25} />

          <Grid
            position={[0, 0, 0]}
            rotation={[Math.PI / 2, 0, 0]}
            args={[40, 40]}
            cellSize={1}
            cellThickness={0.6}
            cellColor="#1b1b1b"
            sectionSize={5}
            sectionThickness={1}
            sectionColor="#2e2e38"
            fadeDistance={40}
            fadeStrength={1.5}
            infiniteGrid
          />

          <AltitudeTarget />
          <Trail />
          <FlyingDrone />

          <OrbitControls makeDefault target={[0, 0, 1.5]} enableDamping dampingFactor={0.1} maxPolarAngle={Math.PI / 2 - 0.02} />
        </Canvas>
        </CanvasBoundary>
        <Hud />
      </div>
    </div>
  );
}
