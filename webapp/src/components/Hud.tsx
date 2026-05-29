import { useSimSnapshot, useThrottledState } from "../hooks";
import { COLOR } from "../consts";

const f = (v: number, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : "—");

export default function Hud() {
  const s = useThrottledState(12);
  const snap = useSimSnapshot();
  const t = s?.telemetry;
  const c = s?.command;
  const ground = snap.meta?.ground_z ?? 0;
  const alt = t ? t.z - ground : 0;
  const speed = t ? Math.hypot(t.vx, t.vy, t.vz) : 0;
  const rpm = t ? (t.prop_speed / 360) * 60 : 0;
  const thr = c?.throttle ?? 0;
  const hover = snap.meta?.hover_throttle ?? 0.68;

  return (
    <>
      <div className="hud">
        <span className="k">Alt</span>
        <span className={"v " + (alt < 0.1 ? "warn" : "good")}>{f(alt)} m</span>
        <span className="k">Speed</span>
        <span className="v">{f(speed)} m/s</span>
        <span className="k">Pos</span>
        <span className="v">{t ? `${f(t.x, 1)}, ${f(t.y, 1)}, ${f(t.z, 1)}` : "—"}</span>
        <span className="k">Yaw</span>
        <span className="v">{t ? f((t.yaw * 180) / Math.PI, 0) : "—"}°</span>
        <span className="k">RPM</span>
        <span className="v">{f(rpm, 0)}</span>
      </div>

      <div className="bar-wrap">
        <div className="bar-label"><span>Throttle</span><b>{(thr * 100).toFixed(0)}%</b></div>
        <div className="bar">
          <span style={{ width: `${Math.min(100, thr * 100)}%`, background: thr > hover ? COLOR.amber : COLOR.green }} />
        </div>
        <div className="bar-label" style={{ marginTop: 8 }}><span>Prop</span><b>{((rpm))|0} rpm</b></div>
        <div className="bar">
          <span style={{ width: `${Math.min(100, (rpm / ((720 / 360) * 60)) * 100)}%`, background: COLOR.prop }} />
        </div>
      </div>

      <div className="hud-bottom">
        <span className="tag">{snap.status}</span>
      </div>
    </>
  );
}
