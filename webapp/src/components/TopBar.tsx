import { NavLink } from "react-router-dom";
import { useSimSnapshot } from "../hooks";
import { store } from "../store";

export default function TopBar() {
  const snap = useSimSnapshot();
  const live = snap.connected;
  return (
    <header className="topbar">
      <div className="brand">
        <div className="brand-mark" />
        <div>
          <h1>Singlecopter · Live Sim</h1>
          <div className="sub">3D digital twin · component map · telemetry &amp; PID profiling</div>
        </div>
      </div>
      <nav className="nav">
        <NavLink to="/" end className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}>
          Dashboard
        </NavLink>
        <NavLink to="/position" className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}>
          Position
          <span className={"nav-dot" + (snap.hasImu ? " on" : "")} />
        </NavLink>
        <NavLink to="/profiling" className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}>
          Profiling
        </NavLink>
      </nav>
      <div className="topbar-right">
        <span className="pill">
          source <code>{snap.source}</code>
        </span>
        <span className="pill" title={store.url}>
          <span className={"dot " + (live ? "live" : "off")} />
          {live ? "connected" : "offline"}
        </span>
      </div>
    </header>
  );
}
