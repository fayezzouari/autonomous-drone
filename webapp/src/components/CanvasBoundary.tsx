import { Component, ReactNode } from "react";

// Keeps a WebGL/Three failure in one canvas from blanking the whole page —
// the telemetry/PID charts and the other panel keep working.
export default class CanvasBoundary extends Component<
  { children: ReactNode; label: string },
  { failed: boolean }
> {
  state = { failed: false };
  static getDerivedStateFromError() {
    return { failed: true };
  }
  render() {
    if (this.state.failed) {
      return (
        <div style={{ display: "grid", placeItems: "center", height: "100%", padding: 24, textAlign: "center" }}>
          <div>
            <div style={{ fontSize: 13, color: "var(--fg-dim)" }}>{this.props.label} unavailable</div>
            <div style={{ fontSize: 11, color: "var(--fg-muted)", marginTop: 6 }}>
              WebGL could not start in this browser/GPU.
            </div>
          </div>
        </div>
      );
    }
    return this.props.children;
  }
}
