import { colorFor, fmt, level, ratio } from "./format.js";

export function Dial({ label, unit, value, spec, digits = 1 }) {
  const lvl = level(value, spec);
  const col = value === null || value === undefined ? "#8b9bb0" : colorFor(lvl);
  const t = ratio(value, spec);
  const a0 = Math.PI * 0.75;
  const a1 = Math.PI * 2.25;
  const a = a0 + t * (a1 - a0);
  const cx = 80;
  const cy = 78;
  const r = 54;
  const needle = polar(cx, cy, r - 8, a);
  const arc = describeArc(cx, cy, r, a0, a1);
  return (
    <div className="meter">
      <svg viewBox="0 0 160 118" aria-label={label}>
        <path d={arc} fill="none" stroke="#0a0e13" strokeWidth="12" strokeLinecap="round" />
        <path d={arc} fill="none" stroke={col} strokeWidth="12" strokeLinecap="round"
          strokeDasharray={`${t * arcLen(r, a0, a1)} ${arcLen(r, a0, a1)}`} />
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
  return (
    <div className="bar">
      <div>{label}</div>
      <div className="track" aria-label={label}>
        <div className="fill" style={{ width: w, background: col }} />
      </div>
      <div style={{ color: col, textAlign: "right" }}>{fmt(value, digits)} {unit}</div>
    </div>
  );
}

export function Thermo({ nickname, value, spec }) {
  const lvl = level(value, spec);
  const col = value === null || value === undefined ? "#8b9bb0" : colorFor(lvl);
  const h = `${ratio(value, spec) * 100}%`;
  return (
    <div className="thermo">
      <div className="well" aria-label={nickname}>
        <div className="fill" style={{ height: h, background: col }} />
      </div>
      <div className="name">{nickname}</div>
      <div className="val" style={{ color: col }}>{fmt(value, 1)} °C</div>
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
