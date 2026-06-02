import { useLayoutEffect, useMemo, useRef, useState } from "react";

export interface StaticSeries {
  data: number[]; // aligned to `t`
  color: string;
  label: string;
  unit?: string;
  dashed?: boolean; // setpoint / reference style
}

export interface ChartMarker {
  t: number;
  label?: string;
  color?: string;
}

interface Props {
  t: number[]; // x values (seconds), shared by every series
  series: StaticSeries[];
  height?: number;
  symmetric?: boolean; // force range symmetric about 0
  zeroLine?: boolean;
  unit?: string;
  /** vertical leg/phase boundaries */
  markers?: ChartMarker[];
  /** controlled playback cursor (seconds); null/undefined hides it */
  cursorT?: number | null;
  /** hover/scrub — receives a time in seconds, or null on leave */
  onCursor?: (t: number | null) => void;
}

const fmt = (v: number, unit?: string) =>
  (Number.isFinite(v) ? v.toFixed(2) : "—") + (unit ?? "");

// Dependency-free static SVG time-series chart. Unlike the live LineChart it
// plots fixed arrays (a loaded trail), supports dashed reference/setpoint
// series, phase markers, and a shared playback cursor so every chart on the
// page scrubs together.
export default function StaticChart({
  t,
  series,
  height = 150,
  symmetric = false,
  zeroLine = true,
  unit,
  markers = [],
  cursorT,
  onCursor,
}: Props) {
  const wrapRef = useRef<HTMLDivElement>(null);
  const [w, setW] = useState(480);

  useLayoutEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      const cw = entries[0]?.contentRect.width;
      if (cw && cw > 0) setW(Math.round(cw));
    });
    ro.observe(el);
    setW(Math.round(el.clientWidth) || 480);
    return () => ro.disconnect();
  }, []);

  const padL = 40;
  const padR = 10;
  const padT = 10;
  const padB = 20;
  const h = height;
  const plotW = Math.max(1, w - padL - padR);
  const plotH = Math.max(1, h - padT - padB);

  const n = t.length;
  const tMin = n ? t[0] : 0;
  const tMax = n ? t[n - 1] : 1;
  const tSpan = tMax - tMin || 1;

  // Scale + path strings only depend on the (static) data and dimensions, never
  // on the cursor — memoize so scrubbing/playback just moves cheap overlays.
  const { lo, hi, yTicks, paths, xOf, yOf } = useMemo(() => {
    let lo = Infinity;
    let hi = -Infinity;
    for (const s of series)
      for (const v of s.data)
        if (Number.isFinite(v)) {
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
    if (!Number.isFinite(lo)) {
      lo = -1;
      hi = 1;
    }
    if (symmetric) {
      const m = Math.max(Math.abs(lo), Math.abs(hi), 1e-6);
      lo = -m;
      hi = m;
    }
    if (lo === hi) {
      lo -= 1;
      hi += 1;
    }
    const yPad = (hi - lo) * 0.1;
    lo -= yPad;
    hi += yPad;
    const xOf = (tt: number) => padL + ((tt - tMin) / tSpan) * plotW;
    const yOf = (v: number) => padT + (1 - (v - lo) / (hi - lo)) * plotH;
    const paths = series.map((s) => {
      let d = "";
      let pen = false;
      for (let i = 0; i < n; i++) {
        const v = s.data[i];
        if (!Number.isFinite(v)) {
          pen = false;
          continue;
        }
        d += (pen ? "L" : "M") + xOf(t[i]).toFixed(1) + " " + yOf(v).toFixed(1) + " ";
        pen = true;
      }
      return d;
    });
    return { lo, hi, yPad, yTicks: [hi - yPad, (lo + hi) / 2, lo + yPad], paths, xOf, yOf };
  }, [series, t, n, tMin, tSpan, plotW, plotH, symmetric, padL, padT]);

  // nearest index to a given time (for cursor readouts)
  const idxOf = (tt: number) => {
    if (n === 0) return 0;
    const f = ((tt - tMin) / tSpan) * (n - 1);
    return Math.max(0, Math.min(n - 1, Math.round(f)));
  };

  const onMove = (e: React.MouseEvent<SVGSVGElement>) => {
    if (!onCursor) return;
    const rect = e.currentTarget.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const frac = (px - padL) / plotW;
    onCursor(tMin + Math.max(0, Math.min(1, frac)) * tSpan);
  };

  const curIdx = cursorT != null ? idxOf(cursorT) : -1;
  const cursorX = cursorT != null ? xOf(cursorT) : 0;

  return (
    <div ref={wrapRef} style={{ width: "100%" }}>
      <svg
        width={w}
        height={h}
        style={{ display: "block", cursor: onCursor ? "crosshair" : "default" }}
        onMouseMove={onMove}
        onMouseLeave={() => onCursor?.(null)}
      >
        {/* gridlines + y labels */}
        {yTicks.map((v, i) => (
          <g key={i}>
            <line
              x1={padL}
              x2={w - padR}
              y1={yOf(v)}
              y2={yOf(v)}
              stroke="rgba(255,255,255,0.06)"
            />
            <text x={padL - 6} y={yOf(v) + 3} textAnchor="end" className="sc-axis">
              {Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1)}
            </text>
          </g>
        ))}

        {/* zero baseline */}
        {zeroLine && lo < 0 && hi > 0 && (
          <line
            x1={padL}
            x2={w - padR}
            y1={yOf(0)}
            y2={yOf(0)}
            stroke="rgba(255,255,255,0.22)"
            strokeDasharray="3 3"
          />
        )}

        {/* phase / leg markers */}
        {markers.map((m, i) => (
          <g key={"m" + i}>
            <line
              x1={xOf(m.t)}
              x2={xOf(m.t)}
              y1={padT}
              y2={h - padB}
              stroke={m.color ?? "rgba(255,255,255,0.16)"}
              strokeDasharray="2 3"
            />
            {m.label && (
              <text x={xOf(m.t) + 3} y={padT + 9} className="sc-mark">
                {m.label}
              </text>
            )}
          </g>
        ))}

        {/* series */}
        {series.map((s, si) => (
          <path
            key={s.label}
            d={paths[si]}
            fill="none"
            stroke={s.color}
            strokeWidth={1.6}
            strokeLinejoin="round"
            strokeDasharray={s.dashed ? "5 3" : undefined}
            opacity={s.dashed ? 0.85 : 1}
          />
        ))}

        {/* playback cursor */}
        {curIdx >= 0 && (
          <>
            <line
              x1={cursorX}
              x2={cursorX}
              y1={padT}
              y2={h - padB}
              stroke="rgba(255,255,255,0.5)"
            />
            {series.map((s, i) =>
              Number.isFinite(s.data[curIdx]) ? (
                <circle
                  key={"c" + i}
                  cx={cursorX}
                  cy={yOf(s.data[curIdx])}
                  r={2.6}
                  fill={s.color}
                />
              ) : null,
            )}
          </>
        )}

        {/* x label */}
        <text x={w - padR} y={h - 6} textAnchor="end" className="sc-axis">
          {tMax.toFixed(1)}s
        </text>
        <text x={padL} y={h - 6} textAnchor="start" className="sc-axis">
          {tMin.toFixed(1)}s
        </text>
      </svg>

      <div className="chart-legend">
        {series.map((s) => (
          <span className="item" key={s.label}>
            <span
              className="line"
              style={{ background: s.color, opacity: s.dashed ? 0.6 : 1 }}
            />
            {s.label}
            <span className="v">
              {curIdx >= 0 ? fmt(s.data[curIdx], s.unit ?? unit) : ""}
            </span>
          </span>
        ))}
      </div>
    </div>
  );
}
