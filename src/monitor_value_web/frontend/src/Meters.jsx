import { colorFor, fmt, level, ratio } from "./format.js";

const TICK_PS = [0, 0.25, 0.5, 0.75, 1];

function tickValue(spec, p) {
  const min = spec?.min ?? 0;
  const max = spec?.max ?? 1;
  return min + p * (max - min);
}

function tickDigits(_spec, _digits) {
  return 0;
}

export function Dial({ label, unit, value, spec, digits = 1 }) {
  const lvl = level(value, spec);
  const col = value === null || value === undefined ? "#8b9bb0" : colorFor(lvl);
  const t = ratio(value, spec);
  const a0 = Math.PI * 0.75;
  const a1 = Math.PI * 2.25;
  const a = a0 + t * (a1 - a0);
  const cx = 80;
  const cy = 68;
  const r = 48;
  const needle = polar(cx, cy, r - 8, a);
  const arc = describeArc(cx, cy, r, a0, a1);
  const d = tickDigits(spec, digits);
  const ticks = TICK_PS.map((p) => {
    const ang = a0 + p * (a1 - a0);
    const outer = polar(cx, cy, r + 2, ang);
    const inner = polar(cx, cy, r - 9, ang);
    const labelPos = polar(cx, cy, r + 15, ang);
    return {
      outer,
      inner,
      labelPos,
      major: p === 0 || p === 0.5 || p === 1,
      text: fmt(tickValue(spec, p), d),
    };
  });
  return (
    <div className="meter">
      <svg viewBox="0 0 160 140" aria-label={label}>
        <path d={arc} fill="none" stroke="#0a0e13" strokeWidth="12" strokeLinecap="round" />
        <path d={arc} fill="none" stroke={col} strokeWidth="12" strokeLinecap="round"
          strokeDasharray={`${t * arcLen(r, a0, a1)} ${arcLen(r, a0, a1)}`} />
        {ticks.map((tick, i) => (
          <g key={i}>
            <line
              x1={tick.inner.x}
              y1={tick.inner.y}
              x2={tick.outer.x}
              y2={tick.outer.y}
              stroke="#5a6a7a"
              strokeWidth={tick.major ? 1.6 : 1}
            />
            {tick.major && (
              <text
                x={tick.labelPos.x}
                y={tick.labelPos.y}
                textAnchor="middle"
                dominantBaseline="middle"
                className="tick-label"
                fill="#8b9bb0"
                fontSize="11"
              >
                {tick.text}
              </text>
            )}
          </g>
        ))}
        <line x1={cx} y1={cy} x2={needle.x} y2={needle.y} stroke="#f4f7fb" strokeWidth="2.5" />
        <circle cx={cx} cy={cy} r="4" fill="#f4f7fb" />
      </svg>
      <div className="val" style={{ color: col }}>
        {fmt(value, digits)}<span className="unit">{unit}</span>
      </div>
      <div className="lbl">{label}</div>
    </div>
  );
}

export function Bar({ label, unit, value, spec, digits = 1 }) {
  const lvl = level(value, spec);
  const col = value === null || value === undefined ? "#8b9bb0" : colorFor(lvl);
  const w = `${ratio(value, spec) * 100}%`;
  const d = tickDigits(spec, digits);
  const labels = TICK_PS.map((p) => ({
    p,
    text: fmt(tickValue(spec, p), d),
    major: p === 0 || p === 0.5 || p === 1,
  }));
  return (
    <div className="bar">
      <div>{label}</div>
      <div className="bar-scale">
        <div className="track" aria-label={label}>
          <div className="bar-ticks" aria-hidden="true">
            <i /><i /><i /><i /><i />
          </div>
          <div className="fill" style={{ width: w, background: col }} />
        </div>
        <div className="bar-tick-labels" aria-hidden="true">
          {labels.map((tick, i) => (
            <span
              key={i}
              className={tick.major ? "major" : ""}
              style={{ left: `${tick.p * 100}%` }}
            >
              {tick.major ? tick.text : ""}
            </span>
          ))}
        </div>
      </div>
      <div style={{ color: col, textAlign: "right" }}>{fmt(value, digits)} {unit}</div>
    </div>
  );
}

export function Thermo({ nickname, value, spec }) {
  const lvl = level(value, spec);
  const col = value === null || value === undefined ? "#8b9bb0" : colorFor(lvl);
  const h = `${ratio(value, spec) * 100}%`;
  const d = 0;
  const labels = [0, 0.5, 1].map((p) => ({
    p,
    text: fmt(tickValue(spec, p), d),
  }));
  return (
    <div className="thermo">
      <div className="thermo-body">
        <div className="thermo-tick-labels" aria-hidden="true">
          {labels.map((tick) => (
            <span key={tick.p} style={{ bottom: `${tick.p * 100}%` }}>{tick.text}</span>
          ))}
        </div>
        <div className="well" aria-label={nickname}>
          <div className="thermo-ticks" aria-hidden="true">
            <i /><i /><i /><i /><i />
          </div>
          <div className="fill" style={{ height: h, background: col }} />
        </div>
      </div>
      <div className="thermo-caption">
        <div className="name">{nickname}</div>
        <div className="val" style={{ color: col }}>{fmt(value, 1)} °C</div>
      </div>
    </div>
  );
}

function polar(cx, cy, r, a) {
  return { x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) };
}

function describeArc(cx, cy, r, a0, a1) {
  const s = polar(cx, cy, r, a0);
  const e = polar(cx, cy, r, a1);
  const large = a1 - a0 > Math.PI ? 1 : 0;
  return `M ${s.x} ${s.y} A ${r} ${r} 0 ${large} 1 ${e.x} ${e.y}`;
}

function arcLen(r, a0, a1) {
  return r * (a1 - a0);
}
