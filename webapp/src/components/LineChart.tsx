import { useEffect, useRef } from "react";
import { store } from "../store";
import type { HistorySample } from "../types";

export interface Series {
  key: keyof HistorySample;
  color: string;
  label: string;
  unit?: string;
  dashed?: boolean;
}

interface Props {
  series: Series[];
  height?: number;
  windowSec?: number;
  /** draw a dashed baseline at y = 0 when 0 is within range */
  zeroLine?: boolean;
  /** force symmetric range around 0 (nice for PID terms) */
  symmetric?: boolean;
  /** show "needs --demo" style empty hint when every series is all-NaN */
  emptyHint?: string;
}

const num = (v: number, unit?: string) =>
  (Number.isFinite(v) ? v.toFixed(2) : "—") + (unit ?? "");

// A compact, dependency-free time-series chart. It reads the rolling history
// straight from the store every animation frame and draws to a canvas, so it
// stays smooth at the bridge's 50 Hz without re-rendering React.
export default function LineChart({
  series,
  height = 130,
  windowSec = 60,
  zeroLine = true,
  symmetric = false,
  emptyHint,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const valRefs = useRef<(HTMLSpanElement | null)[]>([]);

  useEffect(() => {
    const canvas = canvasRef.current!;
    const ctx = canvas.getContext("2d")!;
    let raf = 0;

    const css = getComputedStyle(document.documentElement);
    const cMuted = css.getPropertyValue("--fg-muted").trim() || "#6b6b6b";
    const cBorder = "rgba(255,255,255,0.06)";

    const draw = () => {
      raf = requestAnimationFrame(draw);
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);

      const hist = store.history;
      const n = hist.length;
      const padL = 6, padR = 6, padT = 10, padB = 8;
      const plotW = w - padL - padR;
      const plotH = h - padT - padB;

      if (n === 0) {
        ctx.fillStyle = cMuted;
        ctx.font = "11px ui-monospace, monospace";
        ctx.fillText("waiting for data…", padL, h / 2);
        return;
      }

      const tNow = hist[n - 1].clock;
      const tMin = tNow - windowSec;

      // find first index within window
      let start = 0;
      for (let i = n - 1; i >= 0; i--) {
        if (hist[i].clock < tMin) { start = i + 1; break; }
      }

      // y range across all series in window
      let lo = Infinity, hi = -Infinity;
      let anyFinite = false;
      for (let i = start; i < n; i++) {
        for (const s of series) {
          const v = hist[i][s.key] as number;
          if (Number.isFinite(v)) {
            anyFinite = true;
            if (v < lo) lo = v;
            if (v > hi) hi = v;
          }
        }
      }

      if (!anyFinite) {
        ctx.fillStyle = cMuted;
        ctx.font = "11px ui-monospace, monospace";
        ctx.fillText(emptyHint ?? "no data", padL, h / 2);
        // clear live values
        valRefs.current.forEach((el) => el && (el.textContent = "—"));
        return;
      }

      if (symmetric) {
        const m = Math.max(Math.abs(lo), Math.abs(hi), 1e-6);
        lo = -m; hi = m;
      }
      if (lo === hi) { lo -= 1; hi += 1; }
      const pad = (hi - lo) * 0.12;
      lo -= pad; hi += pad;

      const xOf = (t: number) => padL + ((t - tMin) / windowSec) * plotW;
      const yOf = (v: number) => padT + (1 - (v - lo) / (hi - lo)) * plotH;

      // gridlines
      ctx.strokeStyle = cBorder;
      ctx.lineWidth = 1;
      for (let g = 0; g <= 2; g++) {
        const yy = padT + (g / 2) * plotH;
        ctx.beginPath();
        ctx.moveTo(padL, yy);
        ctx.lineTo(w - padR, yy);
        ctx.stroke();
      }

      // zero baseline
      if (zeroLine && lo < 0 && hi > 0) {
        ctx.strokeStyle = "rgba(255,255,255,0.22)";
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(padL, yOf(0));
        ctx.lineTo(w - padR, yOf(0));
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // series
      series.forEach((s) => {
        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.6;
        if (s.dashed) ctx.setLineDash([4, 3]);
        ctx.beginPath();
        let pen = false;
        for (let i = start; i < n; i++) {
          const v = hist[i][s.key] as number;
          if (!Number.isFinite(v)) { pen = false; continue; }
          const x = xOf(hist[i].clock);
          const y = yOf(v);
          if (!pen) { ctx.moveTo(x, y); pen = true; }
          else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      });

      // live values
      const last = hist[n - 1];
      series.forEach((s, idx) => {
        const el = valRefs.current[idx];
        if (el) el.textContent = num(last[s.key] as number, s.unit);
      });
    };

    raf = requestAnimationFrame(draw);
    return () => cancelAnimationFrame(raf);
  }, [series, windowSec, zeroLine, symmetric, emptyHint]);

  return (
    <>
      <div className="chart-body">
        <canvas ref={canvasRef} style={{ width: "100%", height }} />
      </div>
      <div className="chart-legend">
        {series.map((s, idx) => (
          <span className="item" key={s.label}>
            <span className="line" style={{ background: s.color, opacity: s.dashed ? 0.7 : 1 }} />
            {s.label}
            <span className="v" ref={(el) => (valRefs.current[idx] = el)}>—</span>
          </span>
        ))}
      </div>
    </>
  );
}
