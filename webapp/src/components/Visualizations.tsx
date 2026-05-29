import LineChart, { Series } from "./LineChart";
import { useSimSnapshot } from "../hooks";
import { COLOR } from "../consts";

function VizCard({ title, sub, children }: { title: string; sub?: string; children: React.ReactNode }) {
  return (
    <div className="card chart-card">
      <div className="card-head">
        <span className="card-title">{title}</span>
        {sub && <span className="card-sub">{sub}</span>}
      </div>
      {children}
    </div>
  );
}

const C = COLOR.accent, V = COLOR.violet, G = COLOR.green, A = COLOR.amber, Y = COLOR.cyan, R = COLOR.red;

const telemetryCharts: { title: string; sub: string; series: Series[]; symmetric?: boolean }[] = [
  { title: "Altitude", sub: "z · metres", series: [{ key: "z", color: G, label: "z", unit: " m" }] },
  {
    title: "Velocity", sub: "world-frame · m/s", symmetric: true, series: [
      { key: "vx", color: C, label: "vx", unit: " m/s" },
      { key: "vy", color: V, label: "vy", unit: " m/s" },
      { key: "vz", color: G, label: "vz", unit: " m/s" },
    ],
  },
  { title: "Speed", sub: "magnitude · m/s", series: [{ key: "speed", color: Y, label: "|v|", unit: " m/s" }] },
  { title: "Propeller", sub: "rotor speed · rpm", series: [{ key: "rpm", color: Y, label: "rpm" }] },
  { title: "Throttle", sub: "command · 0–1", series: [{ key: "throttle", color: G, label: "thr" }] },
  {
    title: "Vane angles", sub: "4 independent vanes · degrees", symmetric: true, series: [
      { key: "v1", color: C, label: "v1", unit: "°" },
      { key: "v2", color: A, label: "v2", unit: "°" },
      { key: "v3", color: V, label: "v3", unit: "°" },
      { key: "v4", color: R, label: "v4", unit: "°" },
    ],
  },
  {
    title: "Position", sub: "x / y · metres", series: [
      { key: "x", color: C, label: "x", unit: " m" },
      { key: "y", color: V, label: "y", unit: " m" },
    ],
  },
];

// The controller is now an altitude loop: climb-rate setpoint → vertical accel,
// broken into its P / I / D terms + summed output (each in its own graph), plus
// the tracking view. Populated only in --demo mode.
const HINT = "PID internals stream in --demo mode";
const pidCharts: { title: string; sub: string; series: Series[]; symmetric?: boolean; zero?: boolean }[] = [
  { title: "Altitude PID · P", sub: "proportional term", symmetric: true, series: [{ key: "altP", color: C, label: "P" }] },
  { title: "Altitude PID · I", sub: "integral term", symmetric: true, series: [{ key: "altI", color: V, label: "I" }] },
  { title: "Altitude PID · D", sub: "derivative term", symmetric: true, series: [{ key: "altD", color: A, label: "D" }] },
  { title: "Altitude PID · Output", sub: "vertical accel cmd · m/s²", symmetric: true, series: [{ key: "altOut", color: G, label: "output" }] },
  {
    title: "Climb-rate tracking", sub: "setpoint vs measured · m/s", symmetric: true, series: [
      { key: "altSp", color: C, label: "setpoint", dashed: true },
      { key: "vz", color: G, label: "vz (measured)", unit: " m/s" },
    ],
  },
];

export default function Visualizations() {
  const snap = useSimSnapshot();

  return (
    <>
      <div className="section-label">Telemetry</div>
      <div className="viz-grid">
        {telemetryCharts.map((c) => (
          <VizCard key={c.title} title={c.title} sub={c.sub}>
            <LineChart series={c.series} symmetric={c.symmetric} />
          </VizCard>
        ))}
      </div>

      <div className="section-label" style={{ marginTop: 8 }}>PID Profiling — altitude-hold loop</div>
      {!snap.hasPid && (
        <div className="notice">
          PID component traces (P / I / D / output) are produced by the altitude
          controller running inside the bridge. Launch with{" "}
          <code>uv run web-bridge --demo</code> to see them live — in{" "}
          <code>--mqtt</code> mode the navigator runs as a separate process, so only
          telemetry &amp; commands are on the wire.
        </div>
      )}
      <div className="viz-grid">
        {pidCharts.map((c) => (
          <VizCard key={c.title} title={c.title} sub={c.sub}>
            <LineChart series={c.series} symmetric={c.symmetric} zeroLine emptyHint={HINT} />
          </VizCard>
        ))}
      </div>
    </>
  );
}
