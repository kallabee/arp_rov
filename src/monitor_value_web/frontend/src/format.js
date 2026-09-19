export function fmt(v, digits = 1) {
  if (v === null || v === undefined || Number.isNaN(v)) return "---";
  return Number(v).toFixed(digits);
}

export function clamp01(v) {
  if (v === null || v === undefined || Number.isNaN(v)) return 0;
  return Math.max(0, Math.min(1, v));
}

export function level(value, spec) {
  if (value === null || value === undefined || Number.isNaN(value) || !spec) return "ok";
  const lowBad = Boolean(spec.low_bad);
  if (lowBad) {
    if (spec.alarm !== undefined && value <= spec.alarm) return "alarm";
    if (spec.warn !== undefined && value <= spec.warn) return "warn";
    return "ok";
  }
  if (spec.alarm !== undefined && value >= spec.alarm) return "alarm";
  if (spec.warn !== undefined && value >= spec.warn) return "warn";
  return "ok";
}

export function colorFor(lvl) {
  if (lvl === "alarm") return "#ff4d4f";
  if (lvl === "warn") return "#f5c542";
  return "#3dd68c";
}

export function ratio(value, spec) {
  if (!spec || value === null || value === undefined || Number.isNaN(value)) return 0;
  const min = spec.min ?? 0;
  const max = spec.max ?? 1;
  if (max === min) return 0;
  return clamp01((value - min) / (max - min));
}
