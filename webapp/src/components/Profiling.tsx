import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, Html, Line, OrbitControls } from "@react-three/drei";
import CanvasBoundary from "./CanvasBoundary";
import StaticChart, { ChartMarker } from "./StaticChart";
import { COLOR } from "../consts";
import {
  analyze,
  coerceRawTrail,
  DEFAULT_HZ,
  fmt,
  GOTO,
  ProfileError,
  speedColor,
  type Leg,
  type Profile,
  type Vec3,
} from "../profiling";

const wpLabel = (i: number) => (i < 26 ? String.fromCharCode(65 + i) : "W" + (i + 1));

// ── 3-D flight path ───────────────────────────────────────────────────────────
function FlightScene({
  profile,
  cursorIdx,
  selectedLeg,
}: {
  profile: Profile;
  cursorIdx: number;
  selectedLeg: number | null;
}) {
  const { samples, waypoints, floor, bbox, summary } = profile;

  const points = useMemo<Vec3[]>(() => samples.map((s) => s.pos), [samples]);
  const colors = useMemo<[number, number, number][]>(
    () => samples.map((s) => speedColor(summary.maxSpeed ? s.speed / summary.maxSpeed : 0)),
    [samples, summary.maxSpeed],
  );
  // faint projection of the path onto the ground plane (depth cue)
  const shadow = useMemo<Vec3[]>(
    () => samples.map((s) => [s.pos[0], s.pos[1], floor] as Vec3),
    [samples, floor],
  );

  const cx = (bbox.min[0] + bbox.max[0]) / 2;
  const cy = (bbox.min[1] + bbox.max[1]) / 2;
  const cz = (bbox.min[2] + bbox.max[2]) / 2;
  const span = Math.max(
    bbox.max[0] - bbox.min[0],
    bbox.max[1] - bbox.min[1],
    bbox.max[2] - bbox.min[2],
    2,
  );
  const d = span * 1.15 + 4;
  const cur = samples[Math.min(cursorIdx, samples.length - 1)];

  const sel = selectedLeg != null ? profile.legs[selectedLeg] : null;
  const selPoints = sel
    ? samples.slice(sel.startIdx, sel.endIdx + 1).map((s) => s.pos)
    : null;

  return (
    <CanvasBoundary label="Flight path">
      <Canvas
        dpr={[1, 2]}
        camera={{ position: [cx + d, cy - d, cz + d * 0.7], fov: 45, near: 0.05, far: span * 12 + 80 }}
        onCreated={({ camera }) => {
          camera.up.set(0, 0, 1);
          camera.lookAt(cx, cy, cz);
        }}
      >
        <color attach="background" args={["#050505"]} />
        <hemisphereLight args={["#bcd4ff", "#0c0c10", 0.9]} />
        <directionalLight position={[span, -span, span * 2]} intensity={1.1} />
        <ambientLight intensity={0.35} />

        <Grid
          position={[cx, cy, floor]}
          rotation={[Math.PI / 2, 0, 0]}
          args={[span * 2.4, span * 2.4]}
          cellSize={1}
          cellColor="#161616"
          sectionSize={5}
          sectionColor="#2a2a32"
          fadeDistance={span * 4}
          infiniteGrid
        />

        {/* ground-projected shadow of the path */}
        <Line points={shadow} color="#1a1a1a" lineWidth={1} transparent opacity={0.7} />

        {/* the flight path, coloured by speed */}
        <Line points={points} vertexColors={colors} lineWidth={2.4} />

        {/* highlighted leg (selected in the timeline) */}
        {selPoints && selPoints.length > 1 && (
          <Line points={selPoints} color="#ffffff" lineWidth={4} transparent opacity={0.9} />
        )}

        {/* waypoint markers + labels */}
        {waypoints.map((w, i) => {
          const captured = profile.legs[i]?.captured;
          const col = captured ? COLOR.green : COLOR.amber;
          return (
            <group key={i} position={w}>
              <mesh>
                <sphereGeometry args={[0.22, 16, 16]} />
                <meshStandardMaterial color={col} emissive={col} emissiveIntensity={0.5} />
              </mesh>
              <Html center distanceFactor={span * 1.4} zIndexRange={[10, 0]}>
                <div className="wp-tag" style={{ borderColor: col, color: col }}>
                  {wpLabel(i)}
                </div>
              </Html>
            </group>
          );
        })}

        {/* start / end markers */}
        <mesh position={points[0]}>
          <sphereGeometry args={[0.18, 16, 16]} />
          <meshStandardMaterial color={COLOR.cyan} emissive={COLOR.cyan} emissiveIntensity={0.6} />
        </mesh>
        <mesh position={points[points.length - 1]}>
          <sphereGeometry args={[0.18, 16, 16]} />
          <meshStandardMaterial color={COLOR.red} emissive={COLOR.red} emissiveIntensity={0.6} />
        </mesh>

        {/* live scrub marker */}
        <group position={cur.pos}>
          <mesh>
            <sphereGeometry args={[0.16, 20, 20]} />
            <meshStandardMaterial color="#ffffff" emissive="#ffffff" emissiveIntensity={0.9} />
          </mesh>
          {/* drop line to the ground for altitude readability */}
          <Line points={[cur.pos, [cur.pos[0], cur.pos[1], floor]]} color="#666" lineWidth={1} />
        </group>

        <OrbitControls makeDefault target={[cx, cy, cz]} minDistance={2} maxDistance={span * 8 + 40} />
      </Canvas>
    </CanvasBoundary>
  );
}

// ── top-down ground track (SVG, equal aspect) ──────────────────────────────────
function GroundTrack({
  profile,
  cursorIdx,
  height = 300,
}: {
  profile: Profile;
  cursorIdx: number;
  height?: number;
}) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const [w, setW] = useState(360);
  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver((e) => {
      const cw = e[0]?.contentRect.width;
      if (cw) setW(Math.round(cw));
    });
    ro.observe(el);
    setW(el.clientWidth || 360);
    return () => ro.disconnect();
  }, []);

  const { samples, waypoints } = profile;
  const pad = 24;
  const xs = samples.map((s) => s.pos[0]);
  const ys = samples.map((s) => s.pos[1]);
  const allX = xs.concat(waypoints.map((w) => w[0]));
  const allY = ys.concat(waypoints.map((w) => w[1]));
  const minX = Math.min(...allX);
  const maxX = Math.max(...allX);
  const minY = Math.min(...allY);
  const maxY = Math.max(...allY);
  const spanX = maxX - minX || 1;
  const spanY = maxY - minY || 1;
  const scale = Math.min((w - 2 * pad) / spanX, (height - 2 * pad) / spanY);
  // world +X → right, world +Y → up (flip svg y)
  const px = (x: number) => pad + (x - minX) * scale;
  const py = (y: number) => height - pad - (y - minY) * scale;

  const path = samples.map((s, i) => (i ? "L" : "M") + px(s.pos[0]).toFixed(1) + " " + py(s.pos[1]).toFixed(1)).join(" ");
  const cur = samples[Math.min(cursorIdx, samples.length - 1)];

  return (
    <div ref={wrapRef} style={{ width: "100%" }}>
      <svg width={w} height={height} style={{ display: "block" }}>
        <path d={path} fill="none" stroke={COLOR.accent} strokeWidth={1.6} opacity={0.85} />
        {/* waypoints + connecting ideal route */}
        <path
          d={waypoints.map((wp, i) => (i ? "L" : "M") + px(wp[0]) + " " + py(wp[1])).join(" ")}
          fill="none"
          stroke="rgba(255,255,255,0.25)"
          strokeDasharray="4 4"
        />
        {waypoints.map((wp, i) => {
          const col = profile.legs[i]?.captured ? COLOR.green : COLOR.amber;
          return (
            <g key={i}>
              <circle cx={px(wp[0])} cy={py(wp[1])} r={5} fill="none" stroke={col} strokeWidth={1.5} />
              <text x={px(wp[0]) + 8} y={py(wp[1]) + 4} className="sc-mark" fill={col}>
                {wpLabel(i)}
              </text>
            </g>
          );
        })}
        <circle cx={px(samples[0].pos[0])} cy={py(samples[0].pos[1])} r={4} fill={COLOR.cyan} />
        <circle cx={cur ? px(cur.pos[0]) : 0} cy={cur ? py(cur.pos[1]) : 0} r={4} fill="#fff" stroke="#000" />
      </svg>
    </div>
  );
}

// ── headline stat grid ──────────────────────────────────────────────────────────
function StatGrid({ profile }: { profile: Profile }) {
  const s = profile.summary;
  const cells: [string, string, string?][] = [
    ["Duration", fmt(profile.duration, 1) + " s"],
    ["Path flown", fmt(s.totalPath, 1) + " m"],
    ["Net displacement", fmt(s.netDisplacement, 1) + " m"],
    ["Route efficiency", fmt(s.routeEfficiency * 100, 0) + " %"],
    ["Max speed", fmt(s.maxSpeed) + " m/s"],
    ["Peak climb", fmt(s.maxClimb) + " m/s"],
    ["Max altitude", fmt(s.maxAlt) + " m"],
    ["Peak accel", fmt(s.maxAccel) + " m/s²"],
    ["Waypoints hit", `${s.waypointsCaptured}/${s.waypointsTotal}`],
    ["RMS track err — H", fmt(s.rmsTrackH) + " m/s"],
    ["RMS track err — V", fmt(s.rmsTrackZ) + " m/s"],
    ["Peak pos error", fmt(s.peakPosErr) + " m"],
  ];
  return (
    <div className="stat-grid">
      {cells.map(([k, v]) => (
        <div className="stat" key={k}>
          <div className="stat-k">{k}</div>
          <div className="stat-v">{v}</div>
        </div>
      ))}
    </div>
  );
}

// ── autonomous command sequence (per-leg) ──────────────────────────────────────
function legNarrative(leg: Leg, prev: Leg | null): string {
  const t = leg.target;
  const from = prev ? prev.target : null;
  const dx = from ? t[0] - from[0] : t[0];
  const dy = from ? t[1] - from[1] : t[1];
  const dAlt = leg.targetAgl - leg.altStart;
  const horiz: string[] = [];
  if (Math.abs(dx) > 0.5) horiz.push(`${dx > 0 ? "+" : "−"}X ${Math.abs(dx).toFixed(0)} m`);
  if (Math.abs(dy) > 0.5) horiz.push(`${dy > 0 ? "+" : "−"}Y ${Math.abs(dy).toFixed(0)} m`);
  const vert =
    Math.abs(dAlt) < 0.3
      ? "hold altitude"
      : `${dAlt > 0 ? "climb" : "descend"} to ${leg.targetAgl.toFixed(1)} m`;
  const move = horiz.length ? `transit ${horiz.join(" / ")}, ${vert}` : vert;
  return `→ ${wpLabel(leg.wpIndex)}: ${move}`;
}

function LegTimeline({
  profile,
  selected,
  onSelect,
  cursorIdx,
}: {
  profile: Profile;
  selected: number | null;
  onSelect: (i: number | null) => void;
  cursorIdx: number;
}) {
  const total = profile.duration || 1;
  const activeLeg = profile.samples[cursorIdx]?.wpIndex;
  return (
    <div className="legs">
      {/* gantt strip */}
      <div className="gantt">
        {profile.legs.map((l, i) => (
          <div
            key={i}
            className={"gantt-seg" + (selected === i ? " sel" : "") + (activeLeg === i ? " active" : "")}
            style={{ width: `${(l.duration / total) * 100}%` }}
            onClick={() => onSelect(selected === i ? null : i)}
            title={`Leg ${wpLabel(i)} · ${fmt(l.duration, 1)} s`}
          >
            {wpLabel(i)}
          </div>
        ))}
      </div>

      {profile.legs.map((l, i) => (
        <div
          key={i}
          className={"leg-row" + (selected === i ? " sel" : "")}
          onClick={() => onSelect(selected === i ? null : i)}
        >
          <div className="leg-head">
            <span className="leg-title">{legNarrative(l, i > 0 ? profile.legs[i - 1] : null)}</span>
            <span className={"leg-badge " + (l.captured ? "ok" : "miss")}>
              {l.captured ? "captured" : `missed ${fmt(l.arrivalErr)} m`}
            </span>
          </div>
          <div className="leg-metrics">
            <span><i>t</i>{fmt(l.startT, 1)}–{fmt(l.endT, 1)}s</span>
            <span><i>dur</i>{fmt(l.duration, 1)}s</span>
            <span><i>path</i>{fmt(l.pathLen, 1)}m</span>
            <span><i>direct</i>{fmt(l.efficiency * 100, 0)}%</span>
            <span><i>v̄</i>{fmt(l.meanSpeed)}m/s</span>
            <span><i>v↑</i>{fmt(l.maxClimb)}m/s</span>
            <span><i>rise</i>{fmt(l.riseTime, 1)}s</span>
            <span><i>OS</i>{fmt(l.overshoot, 0)}%</span>
            <span><i>settle</i>{fmt(l.settleTime, 1)}s</span>
            <span><i>ss-err</i>{fmt(l.steadyErr)}m</span>
          </div>
        </div>
      ))}
    </div>
  );
}

// ── upload zone ────────────────────────────────────────────────────────────────
function Dropzone({
  onFile,
  error,
}: {
  onFile: (file: File) => void;
  error: string | null;
}) {
  const [over, setOver] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);
  return (
    <div
      className={"dropzone" + (over ? " over" : "") + (error ? " err" : "")}
      onDragOver={(e) => {
        e.preventDefault();
        setOver(true);
      }}
      onDragLeave={() => setOver(false)}
      onDrop={(e) => {
        e.preventDefault();
        setOver(false);
        const f = e.dataTransfer.files?.[0];
        if (f) onFile(f);
      }}
      onClick={() => inputRef.current?.click()}
    >
      <input
        ref={inputRef}
        type="file"
        accept="application/json,.json"
        style={{ display: "none" }}
        onChange={(e) => {
          const f = e.target.files?.[0];
          if (f) onFile(f);
          e.target.value = "";
        }}
      />
      <div className="dz-mark" />
      <div className="dz-title">Drop a trail <code>.json</code> here, or click to browse</div>
      <div className="dz-sub">
        e.g. <code>samples/trail_20260601_142659.json</code> — a Blender sim dump with{" "}
        <code>trail_pts</code>, <code>ground_z</code>, <code>coll_offset</code>
      </div>
      {error && <div className="dz-err">{error}</div>}
    </div>
  );
}

// ── page ────────────────────────────────────────────────────────────────────────
export default function Profiling() {
  const [profile, setProfile] = useState<Profile | null>(null);
  const [fileName, setFileName] = useState("");
  const [error, setError] = useState<string | null>(null);
  const [cursorIdx, setCursorIdx] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [selectedLeg, setSelectedLeg] = useState<number | null>(null);
  const accRef = useRef(0);

  const loadFile = useCallback((file: File) => {
    setFileName(file.name);
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const data = JSON.parse(String(reader.result));
        const raw = coerceRawTrail(data);
        const p = analyze(raw);
        setProfile(p);
        setError(null);
        setCursorIdx(0);
        setSelectedLeg(null);
        setPlaying(false);
        accRef.current = 0;
      } catch (e) {
        setProfile(null);
        setError(e instanceof ProfileError ? e.message : "Could not parse JSON: " + (e as Error).message);
      }
    };
    reader.onerror = () => setError("Failed to read file.");
    reader.readAsText(file);
  }, []);

  // playback loop — advance the cursor in real (scaled) flight time
  useEffect(() => {
    if (!playing || !profile) return;
    const n = profile.samples.length;
    let raf = 0;
    let last: number | null = null;
    const step = (ts: number) => {
      if (last == null) last = ts;
      accRef.current += ((ts - last) / 1000) * speed;
      last = ts;
      const idx = Math.round(accRef.current / profile.dt);
      if (idx >= n - 1) {
        setCursorIdx(n - 1);
        setPlaying(false);
        return;
      }
      setCursorIdx(idx);
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
    return () => cancelAnimationFrame(raf);
  }, [playing, profile, speed]);

  const onScrub = useCallback(
    (t: number | null) => {
      if (t == null || !profile) return;
      setPlaying(false);
      const idx = Math.max(0, Math.min(profile.samples.length - 1, Math.round(t / profile.dt)));
      setCursorIdx(idx);
      accRef.current = idx * profile.dt;
    },
    [profile],
  );

  const togglePlay = () => {
    if (!profile) return;
    if (!playing && cursorIdx >= profile.samples.length - 1) {
      setCursorIdx(0);
      accRef.current = 0;
    } else {
      accRef.current = cursorIdx * profile.dt;
    }
    setPlaying((p) => !p);
  };

  // chart inputs (memoized — independent of the cursor)
  const charts = useMemo(() => {
    if (!profile) return null;
    const ss = profile.samples;
    const t = ss.map((s) => s.t);
    const markers: ChartMarker[] = profile.legs.map((l) => ({
      t: l.startT,
      label: wpLabel(l.wpIndex),
    }));
    return {
      t,
      markers,
      alt: ss.map((s) => s.agl),
      altTarget: ss.map((s) => profile.legs[s.wpIndex]?.targetAgl ?? NaN),
      vz: ss.map((s) => s.vz),
      vzSp: ss.map((s) => s.vzSp),
      speedH: ss.map((s) => s.speedH),
      speedHSp: ss.map((s) => s.speedHSp),
      trackH: ss.map((s) => s.trackErrH),
      trackZ: ss.map((s) => s.trackErrZ),
      posErr: ss.map((s) => s.posErr),
      accel: ss.map((s) => s.accel),
    };
  }, [profile]);

  const cursorT = profile ? cursorIdx * profile.dt : null;
  const cur = profile ? profile.samples[Math.min(cursorIdx, profile.samples.length - 1)] : null;

  if (!profile) {
    return (
      <>
        <div className="section-label">Autonomous run profiling</div>
        <p className="page-intro">
          Upload a flight-trail dump from the autonomous A→B controller. The profiler reconstructs
          velocity, segments the run against the mission waypoints, and replays the{" "}
          <code>goto</code> controller's setpoint logic to show how the command sequenced and how
          tightly the inner PID tracked it.
        </p>
        <Dropzone onFile={loadFile} error={error} />
      </>
    );
  }

  const n = profile.samples.length;

  return (
    <>
      <div className="prof-bar">
        <div className="prof-file">
          <span className="card-title">Run profile</span>
          <code>{fileName}</code>
          <span className="pill">
            {n} samples · {fmt(profile.duration, 1)}s · assumed {profile.hz} Hz
            {profile.hz === DEFAULT_HZ ? " (loop rate)" : ""}
          </span>
        </div>
        <Dropzone onFile={loadFile} error={error} />
      </div>

      <StatGrid profile={profile} />

      <div className="stage">
        <div className="card canvas-card">
          <div className="card-head">
            <span className="card-title">Flight path</span>
            <span className="card-sub">coloured by speed · ◯ waypoints · ● scrub marker</span>
          </div>
          <div className="canvas-wrap">
            <FlightScene profile={profile} cursorIdx={cursorIdx} selectedLeg={selectedLeg} />
            {cur && (
              <div className="hud">
                <span className="k">t</span>
                <span className="v">{fmt(cur.t, 2)} s</span>
                <span className="k">pos</span>
                <span className="v">
                  {fmt(cur.pos[0], 1)}, {fmt(cur.pos[1], 1)}, {fmt(cur.pos[2], 1)}
                </span>
                <span className="k">alt agl</span>
                <span className="v">{fmt(cur.agl)} m</span>
                <span className="k">speed</span>
                <span className="v">{fmt(cur.speed)} m/s</span>
                <span className="k">climb</span>
                <span className="v">{fmt(cur.vz)} m/s</span>
                <span className="k">leg</span>
                <span className="v">→ {wpLabel(cur.wpIndex)}</span>
                <span className="k">pos err</span>
                <span className="v">{fmt(cur.posErr)} m</span>
                <span className="k">track err</span>
                <span className={"v " + (cur.trackErrH > 1 ? "warn" : "good")}>
                  {fmt(cur.trackErrH)} m/s
                </span>
              </div>
            )}
          </div>
          <div className="playback">
            <button className="play-btn" onClick={togglePlay}>
              {playing ? "❚❚ Pause" : cursorIdx >= n - 1 ? "↻ Replay" : "▶ Play"}
            </button>
            <input
              className="scrub"
              type="range"
              min={0}
              max={n - 1}
              value={cursorIdx}
              onChange={(e) => {
                setPlaying(false);
                const idx = Number(e.target.value);
                setCursorIdx(idx);
                accRef.current = idx * profile.dt;
              }}
            />
            <span className="play-t">
              {fmt(cursorT ?? 0, 1)} / {fmt(profile.duration, 1)} s
            </span>
            <select
              className="speed-sel"
              value={speed}
              onChange={(e) => setSpeed(Number(e.target.value))}
            >
              {[0.25, 0.5, 1, 2, 4].map((v) => (
                <option key={v} value={v}>
                  {v}×
                </option>
              ))}
            </select>
          </div>
        </div>

        <div className="card">
          <div className="card-head">
            <span className="card-title">Ground track</span>
            <span className="card-sub">top-down · world X→right, Y→up</span>
          </div>
          <div style={{ padding: 12 }}>
            <GroundTrack profile={profile} cursorIdx={cursorIdx} />
          </div>
        </div>
      </div>

      <div className="section-label">Autonomous command sequence</div>
      <div className="card" style={{ padding: 14 }}>
        <LegTimeline
          profile={profile}
          selected={selectedLeg}
          onSelect={setSelectedLeg}
          cursorIdx={cursorIdx}
        />
      </div>

      <div className="section-label">
        PID / tracking profile{" "}
        <span style={{ textTransform: "none", letterSpacing: 0, color: "var(--fg-muted)" }}>
          — dashed = controller setpoint (goto P-gains: xy {GOTO.posXyP}, z {GOTO.posZP}), solid =
          flown
        </span>
      </div>
      {charts && (
        <div className="viz-grid">
          <div className="card chart-card">
            <div className="card-head">
              <span className="card-title">Altitude tracking</span>
              <span className="card-sub">agl vs commanded · m</span>
            </div>
            <div className="chart-body">
              <StaticChart
                t={charts.t}
                zeroLine={false}
                markers={charts.markers}
                cursorT={cursorT}
                onCursor={onScrub}
                series={[
                  { data: charts.alt, color: COLOR.accent, label: "altitude", unit: " m" },
                  { data: charts.altTarget, color: COLOR.amber, label: "target", unit: " m", dashed: true },
                ]}
              />
            </div>
          </div>

          <div className="card chart-card">
            <div className="card-head">
              <span className="card-title">Climb-rate loop</span>
              <span className="card-sub">vz vs setpoint · m/s</span>
            </div>
            <div className="chart-body">
              <StaticChart
                t={charts.t}
                symmetric
                markers={charts.markers}
                cursorT={cursorT}
                onCursor={onScrub}
                series={[
                  { data: charts.vz, color: COLOR.green, label: "vz", unit: " m/s" },
                  { data: charts.vzSp, color: COLOR.amber, label: "vz setpoint", unit: " m/s", dashed: true },
                ]}
              />
            </div>
          </div>

          <div className="card chart-card">
            <div className="card-head">
              <span className="card-title">Horizontal speed loop</span>
              <span className="card-sub">‖v‖ₕ vs setpoint · m/s</span>
            </div>
            <div className="chart-body">
              <StaticChart
                t={charts.t}
                zeroLine={false}
                markers={charts.markers}
                cursorT={cursorT}
                onCursor={onScrub}
                series={[
                  { data: charts.speedH, color: COLOR.cyan, label: "speed", unit: " m/s" },
                  { data: charts.speedHSp, color: COLOR.amber, label: "setpoint", unit: " m/s", dashed: true },
                ]}
              />
            </div>
          </div>

          <div className="card chart-card">
            <div className="card-head">
              <span className="card-title">Velocity tracking error</span>
              <span className="card-sub">setpoint − flown · inner loop · m/s</span>
            </div>
            <div className="chart-body">
              <StaticChart
                t={charts.t}
                symmetric
                markers={charts.markers}
                cursorT={cursorT}
                onCursor={onScrub}
                series={[
                  { data: charts.trackH, color: COLOR.violet, label: "horizontal", unit: " m/s" },
                  { data: charts.trackZ, color: COLOR.green, label: "vertical", unit: " m/s" },
                ]}
              />
            </div>
          </div>

          <div className="card chart-card">
            <div className="card-head">
              <span className="card-title">Position error</span>
              <span className="card-sub">distance to active waypoint · m</span>
            </div>
            <div className="chart-body">
              <StaticChart
                t={charts.t}
                zeroLine={false}
                markers={charts.markers}
                cursorT={cursorT}
                onCursor={onScrub}
                series={[{ data: charts.posErr, color: COLOR.red, label: "‖error‖", unit: " m" }]}
              />
            </div>
          </div>

          <div className="card chart-card">
            <div className="card-head">
              <span className="card-title">Acceleration</span>
              <span className="card-sub">|dv/dt| · m/s²</span>
            </div>
            <div className="chart-body">
              <StaticChart
                t={charts.t}
                zeroLine={false}
                markers={charts.markers}
                cursorT={cursorT}
                onCursor={onScrub}
                series={[{ data: charts.accel, color: COLOR.amber, label: "accel", unit: " m/s²" }]}
              />
            </div>
          </div>
        </div>
      )}
    </>
  );
}
