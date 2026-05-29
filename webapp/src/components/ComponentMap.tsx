import { useMemo, useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Html, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import Singlecopter, { CopterRefs } from "./Singlecopter";
import { store } from "../store";
import { COLOR, MAX_DEG, PROP_MAX_SPEED, PROP_VISUAL_MULT } from "../consts";
import { useThrottledState } from "../hooks";
import CanvasBoundary from "./CanvasBoundary";

const MAX_RAD = (MAX_DEG * Math.PI) / 180;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

function label(text: string, pos: [number, number, number], color: string) {
  return (
    <Html position={pos} center distanceFactor={2.4} style={{ pointerEvents: "none" }} zIndexRange={[10, 0]}>
      <div style={{ font: "10px ui-monospace, monospace", color, background: "rgba(0,0,0,0.6)", border: "1px solid rgba(255,255,255,0.12)", padding: "1px 5px", borderRadius: 5, whiteSpace: "nowrap" }}>{text}</div>
    </Html>
  );
}

function ExplodedDrone() {
  const copter = useRef<CopterRefs>(null);
  const spinRing = useRef<THREE.Group>(null);

  // body-frame force arrows: X from vanes 1&3, Y from vanes 2&4, plus vertical thrust
  const xArrow = useMemo(() => new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.02), 0.001, COLOR.vanePitch, 0.06, 0.035), []);
  const yArrow = useMemo(() => new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0.02), 0.001, COLOR.vaneRoll, 0.06, 0.035), []);
  const thrustArrow = useMemo(() => new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, 0, 0.16), 0.001, COLOR.thrust, 0.06, 0.035), []);

  useFrame((_, dtRaw) => {
    const s = store.latest;
    const c = copter.current;
    if (!s || !c) return;
    const dt = Math.min(dtRaw, 0.05);
    const cmd = s.command, t = s.telemetry;
    const propFrac = t.prop_speed / PROP_MAX_SPEED;
    const thrustFrac = propFrac * propFrac;

    // spin + deflect (no yaw/lean — schematic stays still while the camera orbits)
    if (c.prop) c.prop.rotation.z += (t.prop_speed * PROP_VISUAL_MULT * dt * Math.PI) / 180;
    const angles = [cmd.vane1, cmd.vane2, cmd.vane3, cmd.vane4];
    if (c.vanes[0]) c.vanes[0].rotation.y = -cmd.vane1;
    if (c.vanes[2]) c.vanes[2].rotation.y = -cmd.vane3;
    if (c.vanes[1]) c.vanes[1].rotation.x = cmd.vane2;
    if (c.vanes[3]) c.vanes[3].rotation.x = cmd.vane4;

    // highlight whichever parts are active
    c.matProp.emissiveIntensity = 0.1 + 0.7 * propFrac;
    c.discMat.opacity = 0.04 + 0.16 * thrustFrac;
    c.vaneMats.forEach((m, i) => { m.emissiveIntensity = clamp(Math.abs(angles[i]) / MAX_RAD, 0, 1); });

    // prop spin indicator ring
    if (spinRing.current) {
      spinRing.current.visible = propFrac > 0.02;
      spinRing.current.rotation.z += (t.prop_speed * dt * Math.PI) / 180;
    }

    // body-frame force arrows (sim convention)
    const fxb = -0.5 * (Math.sin(cmd.vane1) + Math.sin(cmd.vane3));
    const fyb = 0.5 * (Math.sin(cmd.vane2) + Math.sin(cmd.vane4));
    if (Math.abs(fxb) > 0.01 && thrustFrac > 0.02) {
      xArrow.visible = true;
      xArrow.setDirection(new THREE.Vector3(Math.sign(fxb), 0, 0));
      xArrow.setLength(0.12 + Math.abs(fxb) * 1.6, 0.06, 0.035);
    } else xArrow.visible = false;
    if (Math.abs(fyb) > 0.01 && thrustFrac > 0.02) {
      yArrow.visible = true;
      yArrow.setDirection(new THREE.Vector3(0, Math.sign(fyb), 0));
      yArrow.setLength(0.12 + Math.abs(fyb) * 1.6, 0.06, 0.035);
    } else yArrow.visible = false;
    thrustArrow.visible = thrustFrac > 0.02;
    thrustArrow.setLength(0.12 + cmd.throttle * 0.6, 0.06, 0.035);
  });

  return (
    <group scale={1.9} position={[0, 0, -0.15]}>
      <Singlecopter ref={copter} />
      <primitive object={xArrow} />
      <primitive object={yArrow} />
      <primitive object={thrustArrow} />

      {/* prop spin indicator */}
      <group ref={spinRing} position={[0, 0, 0.2]}>
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <torusGeometry args={[0.2, 0.006, 8, 40]} />
          <meshBasicMaterial color={COLOR.prop} />
        </mesh>
        <mesh position={[0.2, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <coneGeometry args={[0.022, 0.05, 12]} />
          <meshBasicMaterial color={COLOR.prop} />
        </mesh>
      </group>

      {label("Propeller", [0, 0, 0.44], COLOR.prop)}
      {label("Vanes 1·3 — fore/aft", [0.34, 0, -0.06], COLOR.vanePitch)}
      {label("Vanes 2·4 — lateral", [0, 0.34, -0.06], COLOR.vaneRoll)}
    </group>
  );
}

function Legend() {
  const s = useThrottledState(12);
  const cmd = s?.command;
  const t = s?.telemetry;
  const propFrac = t ? t.prop_speed / PROP_MAX_SPEED : 0;
  const rpm = t ? (t.prop_speed / 360) * 60 : 0;
  const deg = (r: number) => (r * 180) / Math.PI;
  const fmtDeg = (r: number) => `${deg(r) >= 0 ? "+" : ""}${deg(r).toFixed(1)}°`;

  const rows = [
    { color: COLOR.prop, name: "Propeller", desc: "lift · spins about +Z", val: `${rpm.toFixed(0)} rpm`, on: propFrac > 0.02 },
    { color: COLOR.vanePitch, name: "Vane 1", desc: "+X arm · fore/aft", val: fmtDeg(cmd?.vane1 ?? 0), on: Math.abs(deg(cmd?.vane1 ?? 0)) > 0.5 },
    { color: COLOR.vaneRoll, name: "Vane 2", desc: "+Y arm · lateral", val: fmtDeg(cmd?.vane2 ?? 0), on: Math.abs(deg(cmd?.vane2 ?? 0)) > 0.5 },
    { color: COLOR.vanePitch, name: "Vane 3", desc: "−X arm · fore/aft", val: fmtDeg(cmd?.vane3 ?? 0), on: Math.abs(deg(cmd?.vane3 ?? 0)) > 0.5 },
    { color: COLOR.vaneRoll, name: "Vane 4", desc: "−Y arm · lateral", val: fmtDeg(cmd?.vane4 ?? 0), on: Math.abs(deg(cmd?.vane4 ?? 0)) > 0.5 },
    { color: COLOR.thrust, name: "Thrust", desc: "vertical · throttle", val: `${((cmd?.throttle ?? 0) * 100).toFixed(0)}%`, on: (cmd?.throttle ?? 0) > 0.02 },
  ];

  return (
    <div className="legend">
      {rows.map((r) => (
        <div className={"legend-row" + (r.on ? " active" : "")} key={r.name}>
          <span className="swatch" style={{ background: r.color }} />
          <span className="name">{r.name}<small>{r.desc}</small></span>
          <span className="val">{r.val}<span className={"state-dot" + (r.on ? " on" : "")} /></span>
        </div>
      ))}
    </div>
  );
}

export default function ComponentMap() {
  return (
    <div className="card canvas-card">
      <div className="card-head">
        <span className="card-title">Component Map</span>
        <span className="card-sub">which parts move · and which way</span>
      </div>
      <div className="canvas-wrap" style={{ height: 340 }}>
        <CanvasBoundary label="Component map">
        <Canvas
          dpr={[1, 2]}
          camera={{ position: [1.4, -1.6, 1.0], fov: 45, near: 0.05, far: 50 }}
          onCreated={({ camera }) => { camera.up.set(0, 0, 1); camera.lookAt(0, 0, 0.1); }}
        >
          <color attach="background" args={["#070707"]} />
          <hemisphereLight args={["#cfe0ff", "#15151a", 0.8]} />
          <directionalLight position={[3, -2, 5]} intensity={1.2} />
          <ambientLight intensity={0.35} />
          <ExplodedDrone />
          <OrbitControls makeDefault enablePan={false} autoRotate autoRotateSpeed={0.9} target={[0, 0, 0.1]} minDistance={1.2} maxDistance={4} />
        </Canvas>
        </CanvasBoundary>
      </div>
      <Legend />
    </div>
  );
}
