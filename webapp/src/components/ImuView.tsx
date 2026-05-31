import { useEffect, useMemo, useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import Singlecopter, { CopterRefs } from "./Singlecopter";
import CanvasBoundary from "./CanvasBoundary";
import LineChart from "./LineChart";
import { store } from "../store";
import { useSimSnapshot } from "../hooks";
import { COLOR } from "../consts";

const DEG = Math.PI / 180;

// The drone, rotated live by the real IMU attitude (yaw·pitch·roll) and with its
// vanes set from the hardware servo angles (drone/hw).
function AttitudeDrone() {
  const grp = useRef<THREE.Group>(null);
  const copter = useRef<CopterRefs>(null);

  const Z = useMemo(() => new THREE.Vector3(0, 0, 1), []);
  const Y = useMemo(() => new THREE.Vector3(0, 1, 0), []);
  const X = useMemo(() => new THREE.Vector3(1, 0, 0), []);
  const qy = useMemo(() => new THREE.Quaternion(), []);
  const qp = useMemo(() => new THREE.Quaternion(), []);
  const qr = useMemo(() => new THREE.Quaternion(), []);
  const target = useMemo(() => new THREE.Quaternion(), []);

  useFrame((_, dt) => {
    const s = store.latest;
    if (!grp.current || !s?.imu) return;
    const { yaw, pitch, roll } = s.imu;
    // aerospace ZYX: world = Rz(yaw)·Ry(pitch)·Rx(roll)
    qy.setFromAxisAngle(Z, yaw * DEG);
    qp.setFromAxisAngle(Y, pitch * DEG);
    qr.setFromAxisAngle(X, roll * DEG);
    target.copy(qy).multiply(qp).multiply(qr);
    grp.current.quaternion.slerp(target, 1 - Math.exp(-dt * 18));

    // vanes from servo angles (90° = neutral), same hinge convention as the twin
    const c = copter.current;
    const hw = s.hw;
    if (c && hw) {
      const a1 = (hw.s1 - 90) * DEG, a2 = (hw.s2 - 90) * DEG;
      const a3 = (hw.s3 - 90) * DEG, a4 = (hw.s4 - 90) * DEG;
      if (c.vanes[0]) c.vanes[0].rotation.y = -a1;
      if (c.vanes[2]) c.vanes[2].rotation.y = -a3;
      if (c.vanes[1]) c.vanes[1].rotation.x = a2;
      if (c.vanes[3]) c.vanes[3].rotation.x = a4;
      const f = Math.min(1, Math.max(0, hw.throttle));
      c.matProp.emissiveIntensity = 0.1 + 0.6 * f;
      if (c.prop) c.prop.rotation.z += (f * 720 * 8 * dt * Math.PI) / 180;
    }
  });

  return (
    <group ref={grp}>
      <Singlecopter ref={copter} scale={2.2} />
      {/* body axes so the orientation is unambiguous: X fwd, Y left, Z up */}
      <arrowHelper args={[new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.7, COLOR.red, 0.12, 0.07]} />
      <arrowHelper args={[new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.7, COLOR.green, 0.12, 0.07]} />
      <arrowHelper args={[new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.7, COLOR.accent, 0.12, 0.07]} />
    </group>
  );
}

function AttitudeScene() {
  return (
    <CanvasBoundary label="Attitude view">
      <Canvas
        dpr={[1, 2]}
        camera={{ position: [2.2, -2.6, 1.3], fov: 45, near: 0.05, far: 50 }}
        onCreated={({ camera }) => { camera.up.set(0, 0, 1); camera.lookAt(0, 0, 0); }}
      >
        <color attach="background" args={["#050505"]} />
        <hemisphereLight args={["#bcd4ff", "#15151a", 0.8]} />
        <directionalLight position={[4, -3, 6]} intensity={1.3} />
        <ambientLight intensity={0.3} />
        {/* fixed world reference grid (the drone rotates against it) */}
        <Grid position={[0, 0, -0.9]} rotation={[Math.PI / 2, 0, 0]} args={[12, 12]}
          cellSize={0.5} cellColor="#1b1b1b" sectionSize={2} sectionColor="#2e2e38"
          fadeDistance={14} infiniteGrid />
        <AttitudeDrone />
        <OrbitControls makeDefault enablePan={false} target={[0, 0, 0]} minDistance={1.5} maxDistance={6} />
      </Canvas>
    </CanvasBoundary>
  );
}

// Classic artificial horizon, driven imperatively (rAF) for smoothness.
function AttitudeIndicator() {
  const ball = useRef<HTMLDivElement>(null);
  const bankRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    let raf = 0;
    const tick = () => {
      raf = requestAnimationFrame(tick);
      const imu = store.latest?.imu;
      if (!imu) return;
      const pxPerDeg = 2.6;
      if (ball.current)
        ball.current.style.transform =
          `translate(-50%, calc(-50% + ${imu.pitch * pxPerDeg}px)) rotate(${-imu.roll}deg)`;
      if (bankRef.current)
        bankRef.current.style.transform = `rotate(${-imu.roll}deg)`;
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);

  return (
    <div className="adi">
      <div className="adi-ball" ref={ball}>
        <div className="adi-sky" />
        <div className="adi-ground" />
        <div className="adi-horizon" />
      </div>
      {/* fixed aircraft reference */}
      <div className="adi-fixed">
        <span className="adi-wing left" />
        <span className="adi-dot" />
        <span className="adi-wing right" />
      </div>
      <div className="adi-bezel" />
      <div className="adi-bank" ref={bankRef}><span className="adi-bank-mark" /></div>
    </div>
  );
}

function Readouts() {
  const refs = {
    yaw: useRef<HTMLDivElement>(null), pitch: useRef<HTMLDivElement>(null),
    roll: useRef<HTMLDivElement>(null), gz: useRef<HTMLDivElement>(null),
    thr: useRef<HTMLDivElement>(null),
    s: [useRef<HTMLDivElement>(null), useRef<HTMLDivElement>(null), useRef<HTMLDivElement>(null), useRef<HTMLDivElement>(null)],
  };
  useEffect(() => {
    let raf = 0;
    const set = (el: HTMLDivElement | null, v: string) => { if (el && el.firstChild) el.firstChild.textContent = v; };
    const tick = () => {
      raf = requestAnimationFrame(tick);
      const s = store.latest;
      const imu = s?.imu, hw = s?.hw;
      if (imu) {
        set(refs.yaw.current, imu.yaw.toFixed(1));
        set(refs.pitch.current, imu.pitch.toFixed(1));
        set(refs.roll.current, imu.roll.toFixed(1));
        set(refs.gz.current, imu.gz.toFixed(2));
      }
      if (hw) {
        set(refs.thr.current, (hw.throttle * 100).toFixed(0));
        [hw.s1, hw.s2, hw.s3, hw.s4].forEach((v, i) => set(refs.s[i].current, v.toFixed(1)));
      }
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);

  const mk = (label: string, ref: React.RefObject<HTMLDivElement>, unit?: string, color?: string) => (
    <div className="readout">
      <div className="readout-k">{label}</div>
      <div className="readout-v" ref={ref} style={color ? { color } : undefined}><span>—</span><small>{unit}</small></div>
    </div>
  );

  return (
    <div className="readout-grid">
      {mk("Yaw", refs.yaw, "°", COLOR.accent)}
      {mk("Pitch", refs.pitch, "°", COLOR.green)}
      {mk("Roll", refs.roll, "°", COLOR.amber)}
      {mk("Gyro Z", refs.gz, "°/s", COLOR.violet)}
      {mk("Throttle", refs.thr, "%")}
      {mk("Servo 1", refs.s[0], "°")}
      {mk("Servo 2", refs.s[1], "°")}
      {mk("Servo 3", refs.s[2], "°")}
      {mk("Servo 4", refs.s[3], "°")}
    </div>
  );
}

export default function ImuView() {
  const snap = useSimSnapshot();
  return (
    <>
      {!snap.hasImu && (
        <div className="notice">
          No IMU data on the wire yet. Point the bridge at the broker carrying{" "}
          <code>drone/imu</code> — e.g. <code>uv run web-bridge --mqtt --host 10.243.245.93</code>{" "}
          (or add <code>--imu-host &lt;ip&gt;</code> to read it from a separate flight controller
          while the main twin uses another broker).
        </div>
      )}
      <div className="stage">
        <div className="card canvas-card">
          <div className="card-head">
            <span className="card-title">Attitude — live IMU</span>
            <span className="card-sub">yaw · pitch · roll from <code>drone/imu</code></span>
          </div>
          <div className="canvas-wrap"><AttitudeScene /></div>
        </div>

        <div className="card">
          <div className="card-head">
            <span className="card-title">Instruments</span>
            <span className="card-sub">artificial horizon + live values</span>
          </div>
          <div className="adi-wrap"><AttitudeIndicator /></div>
          <Readouts />
        </div>
      </div>

      <div className="section-label">IMU history</div>
      <div className="viz-grid">
        <div className="card chart-card">
          <div className="card-head"><span className="card-title">Attitude</span><span className="card-sub">degrees</span></div>
          <LineChart symmetric series={[
            { key: "imuYaw", color: COLOR.accent, label: "yaw", unit: "°" },
            { key: "imuPitch", color: COLOR.green, label: "pitch", unit: "°" },
            { key: "imuRoll", color: COLOR.amber, label: "roll", unit: "°" },
          ]} emptyHint="waiting for drone/imu" />
        </div>
        <div className="card chart-card">
          <div className="card-head"><span className="card-title">Gyro Z</span><span className="card-sub">yaw rate · °/s</span></div>
          <LineChart symmetric series={[{ key: "gz", color: COLOR.violet, label: "gz", unit: "°/s" }]} emptyHint="waiting for drone/imu" />
        </div>
        <div className="card chart-card">
          <div className="card-head"><span className="card-title">Yaw</span><span className="card-sub">heading · degrees</span></div>
          <LineChart zeroLine={false} series={[{ key: "imuYaw", color: COLOR.accent, label: "yaw", unit: "°" }]} emptyHint="waiting for drone/imu" />
        </div>
      </div>
    </>
  );
}
