import { useEffect, useMemo, useState } from "react";
import { loadUiPrefs, saveUiPrefs } from "./prefs.js";

const AF_DIV = 9;
const AF_CELLS = Array.from({ length: AF_DIV * AF_DIV }, (_, i) => [
  i % AF_DIV,
  Math.floor(i / AF_DIV),
]);

/** UI distance range (m). Max = infinity (LensPosition 0). */
const FOCUS_DIST_MIN_M = 0.1;
const FOCUS_DIST_MAX_M = 10;

/** LensPosition is dioptres (1/m). 0 = infinity. */
function lensToDistanceM(lensPosition) {
  const lp = Number(lensPosition) || 0;
  if (lp <= 0.05) return FOCUS_DIST_MAX_M;
  return Math.min(FOCUS_DIST_MAX_M, Math.max(FOCUS_DIST_MIN_M, 1 / lp));
}

function distanceToLens(distanceM) {
  const d = Number(distanceM);
  if (!Number.isFinite(d) || d >= FOCUS_DIST_MAX_M - 1e-6) return 0;
  return 1 / Math.max(FOCUS_DIST_MIN_M, d);
}

function formatFocusDistance(distanceM, lensPosition) {
  const lp = Number(lensPosition) || 0;
  if (lp <= 0.05 || distanceM >= FOCUS_DIST_MAX_M - 1e-6) return "∞";
  return Number(distanceM).toFixed(2);
}

function windowForCell(c, r) {
  const w = 1 / AF_DIV;
  const h = 1 / AF_DIV;
  return [c * w, r * h, w, h].map((v) => Number(v.toFixed(4)));
}

function parseWindow(s) {
  if (!s) return null;
  const p = String(s).split(",").map(Number);
  if (p.length !== 4 || p.some((n) => Number.isNaN(n))) return null;
  return p;
}

function sameWindow(a, b) {
  if (!a || !b) return false;
  const tol = 0.5 / AF_DIV;
  return a.every((v, i) => Math.abs(v - b[i]) < tol);
}

/** Full-frame AF: empty / missing / whole sensor rectangle. */
function isFullFrame(win) {
  if (!win) return true;
  const [x, y, w, h] = win;
  return x <= 0.02 && y <= 0.02 && x + w >= 0.98 && y + h >= 0.98;
}

async function postCamera(id, body) {
  const res = await fetch(`/api/cameras/${encodeURIComponent(id)}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `camera ${res.status}`);
  return data;
}

/** Build MediaMTX WebRTC player URL for a path (e.g. http://host:8889/cam1/). */
function streamPageUrl(webrtcBase, path) {
  const name = String(path || "").replace(/^\/+|\/+$/g, "");
  try {
    const u = new URL(webrtcBase || "http://127.0.0.1:8889");
    u.hostname = window.location.hostname || u.hostname;
    u.pathname = `/${name}/`;
    u.search = "";
    u.hash = "";
    return u.toString();
  } catch (_) {
    return `http://${window.location.hostname || "127.0.0.1"}:8889/${name}/`;
  }
}

function formatBytes(n) {
  const x = Number(n);
  if (!Number.isFinite(x) || x < 0) return "—";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let v = x;
  let i = 0;
  while (v >= 1000 && i < units.length - 1) {
    v /= 1000;
    i += 1;
  }
  const digits = v >= 10 || i === 0 ? 0 : 1;
  return `${v.toFixed(digits)}${units[i]}`;
}

function recActive(bundle, id) {
  return Boolean((bundle?.state || []).find((c) => c.id === id)?.recording?.active);
}

export function CameraPanel({ config }) {
  const meta = config?.cameras || {};
  const items = Array.isArray(meta.items) && meta.items.length
    ? meta.items
    : [
        { id: "cam0", path: "cam0", nickname: "Ceiling" },
        { id: "cam1", path: "cam1", nickname: "Canopy" },
      ];
  const zooms = Array.isArray(meta.zoom) && meta.zoom.length
    ? meta.zoom
    : [
        { id: "binned", label: "1x Wide" },
        { id: "full", label: "1x Full res" },
        { id: "crop", label: "1.5x Crop" },
      ];
  const webrtcBase = meta.webrtc || "http://127.0.0.1:8889";
  const [camId, setCamId] = useState(() => loadUiPrefs().camId || "cam0");
  const [bundle, setBundle] = useState(null);
  const [busy, setBusy] = useState(false);
  const [note, setNote] = useState("");

  const selectCam = (id) => {
    setCamId(id);
    saveUiPrefs({ camId: id });
  };

  useEffect(() => {
    if (!items.some((x) => x.id === camId)) {
      const fallback = items[0]?.id || "cam0";
      setCamId(fallback);
    }
  }, [items, camId]);

  const refresh = async () => {
    try {
      const res = await fetch("/api/cameras");
      const data = await res.json();
      setBundle(data);
      if (data.error) setNote(data.error);
    } catch (exc) {
      setNote(String(exc.message || exc));
    }
  };

  useEffect(() => {
    refresh();
    const t = setInterval(refresh, 2500);
    return () => clearInterval(t);
  }, []);

  const state = useMemo(
    () => (bundle?.state || []).find((c) => c.id === camId) || {},
    [bundle, camId]
  );
  const live = new Set(Array.isArray(meta.live) ? meta.live : ["exposure", "wb"]);
  const restartOf = (group) => !live.has(group);
  const tag = (group) => (restartOf(group) ? "restart" : "live");
  const backend = bundle?.backend || meta.backend || "mediamtx";
  const streamBase = bundle?.webrtc || webrtcBase;
  const afWin = parseWindow(state.focus?.window);
  const afFull = isFullFrame(afWin);
  const afMode = state.focus?.mode === "continuous" || state.focus?.mode === "auto";
  const exposureMode = state.exposure?.mode === "me" ? "me" : "ae";
  const isAe = exposureMode === "ae";
  const isMe = exposureMode === "me";
  const isWbManual = state.wb?.mode === "manual";
  const isWbAuto = !isWbManual;

  const [meHold, setMeHold] = useState({ shutter_us: 8000, gain: 2 });
  const [wbHold, setWbHold] = useState([2, 1.5]);
  useEffect(() => {
    const shutter = Number(state.exposure?.seed_shutter_us ?? state.exposure?.shutter_us);
    const gain = Number(state.exposure?.seed_gain ?? state.exposure?.gain);
    if (Number.isFinite(shutter) && shutter > 0 && Number.isFinite(gain) && gain > 0) {
      setMeHold({ shutter_us: shutter, gain });
    }
  }, [
    state.exposure?.mode,
    state.exposure?.shutter_us,
    state.exposure?.gain,
    state.exposure?.seed_shutter_us,
    state.exposure?.seed_gain,
  ]);
  useEffect(() => {
    const src = state.wb?.seed_gains?.[0] ? state.wb.seed_gains : state.wb?.gains;
    const r = Number(src?.[0]);
    const b = Number(src?.[1]);
    if (Number.isFinite(r) && r > 0 && Number.isFinite(b) && b > 0) {
      setWbHold([r, b]);
    }
  }, [state.wb?.mode, state.wb?.gains, state.wb?.seed_gains]);

  const send = async (body, restartHint) => {
    setBusy(true);
    setNote(restartHint ? "Restarting stream…" : "Applying…");
    try {
      const result = await postCamera(camId, body);
      setNote(result.restart ? "Stream restarted" : "Applied live");
      await refresh();
    } catch (exc) {
      setNote(String(exc.message || exc));
    } finally {
      setBusy(false);
    }
  };

  const openStream = (item) => {
    const path = item.path || item.id;
    window.open(streamPageUrl(streamBase, path), "_blank", "noopener,noreferrer");
  };

  return (
    <div className="card control-card cam-panel">
      <h2>Cameras</h2>
      <div className="cam-notebook" role="tablist" aria-label="Cameras">
        {items.map((item) => {
          const nick = (item.nickname || item.id || "").toLowerCase();
          const tone = nick.includes("canopy") ? "canopy"
            : nick.includes("ceil") ? "ceiling"
              : item.id === "cam1" ? "canopy" : "ceiling";
          const rec = recActive(bundle, item.id);
          return (
            <button
              key={item.id}
              type="button"
              role="tab"
              aria-selected={item.id === camId}
              className={`cam-notebook-tab ${tone}${item.id === camId ? " on" : ""}`}
              onClick={() => selectCam(item.id)}
            >
              {item.nickname || item.id}
              {rec ? <span className="cam-rec-dot" title="Recording" aria-label="Recording" /> : null}
            </button>
          );
        })}
      </div>

      <div className="cam-page" role="tabpanel">
        <div className="cam-page-bar">
          <span className={`lamp ${state.ready ? "live" : "stale"}`}>
            <i /> {state.ready ? "ready" : "idle"}
          </span>
          {state.recording?.active ? (
            <span
              className={`lamp rec on${bundle?.storage && !bundle.storage.ok ? " alarm" : ""}`}
              title="Recording to disk"
            >
              <i /> REC
            </span>
          ) : null}
          {bundle?.storage ? (
            <span
              className={`cam-free${bundle.storage.ok ? "" : " low"}`}
              title={`Keep at least ${formatBytes(bundle.storage.min_free_bytes)} free`}
            >
              {formatBytes(bundle.storage.free_bytes)} free
            </span>
          ) : null}
          <span className="cam-note">{backend}</span>
          <a
            className="btn cam-stream"
            href={streamPageUrl(streamBase, state.path || camId)}
            target="_blank"
            rel="noopener noreferrer"
            title={`Open WebRTC stream (${state.path || camId})`}
            onClick={(ev) => {
              ev.preventDefault();
              const item = items.find((x) => x.id === camId) || { id: camId, path: state.path || camId };
              openStream(item);
            }}
          >
            Stream
          </a>
        </div>

      <div className="cam-section">
        <div className="cam-label">Zoom (sensor mode · {tag("zoom")})</div>
        <div className="cam-seg">
          {zooms.map((z) => (
            <button
              key={z.id}
              type="button"
              disabled={busy}
              className={state.zoom === z.id ? "btn primary" : "btn"}
              title={
                z.hint
                || (z.width && z.height
                  ? `Sensor ${z.sensor_mode || "—"} → stream ${z.width}x${z.height} @ ${z.fps || "—"} fps`
                  : undefined)
              }
              onClick={() => send({ zoom: z.id }, restartOf("zoom"))}
            >
              {z.label}
            </button>
          ))}
        </div>
      </div>

      <div className="cam-grid">
        <div className="cam-col cam-col-focus">
          <div className="cam-label">Focus ({tag("focus")})</div>
          <div className="cam-seg">
            <button type="button" disabled={busy} className={state.focus?.mode === "continuous" ? "btn primary" : "btn"} onClick={() => send({ focus: { mode: "continuous" } }, restartOf("focus"))}>AF cont.</button>
            <button type="button" disabled={busy} className={state.focus?.mode === "auto" ? "btn primary" : "btn"} onClick={() => send({ focus: { mode: "auto" } }, restartOf("focus"))}>AF once</button>
            <button type="button" disabled={busy} className={state.focus?.mode === "manual" ? "btn primary" : "btn"} onClick={() => send({ focus: { mode: "manual", lens_position: state.focus?.lens_position || 1 } }, restartOf("focus"))} title="Manual focus — set focus distance">MF</button>
          </div>
          <div className="cam-label">AF window{afMode ? (afFull ? " · full frame" : " · region") : ""}</div>
          <div className={`af-grid${afMode ? " af-mode" : ""}${afFull ? " af-full" : ""}`}>
            {AF_CELLS.map(([c, r]) => {
              const win = windowForCell(c, r);
              const on = !afFull && sameWindow(afWin, win);
              return (
                <button
                  key={`${c}-${r}`}
                  type="button"
                  disabled={busy}
                  className={on ? "af-cell on" : "af-cell"}
                  aria-pressed={on}
                  title={`AF cell ${c + 1},${r + 1}`}
                  onClick={() => send({ focus: { window: win } }, restartOf("focus"))}
                />
              );
            })}
          </div>
          <button
            type="button"
            className={`btn cam-wide${afFull ? " primary" : ""}`}
            disabled={busy}
            aria-pressed={afFull}
            onClick={() => send({ focus: { window: "" } }, restartOf("focus"))}
          >
            Full frame
          </button>
          {state.focus?.mode === "manual" && (
            <RangeCommit
              label="Focus"
              unit="m"
              display={formatFocusDistance(
                lensToDistanceM(state.focus?.lens_position),
                state.focus?.lens_position,
              )}
              digits={2}
              min={FOCUS_DIST_MIN_M}
              max={FOCUS_DIST_MAX_M}
              step={0.05}
              disabled={busy}
              value={lensToDistanceM(state.focus?.lens_position)}
              onCommit={(v) => send({ focus: { mode: "manual", lens_position: distanceToLens(v) } }, restartOf("focus"))}
            />
          )}
        </div>

        <div className="cam-col cam-col-exp">
          <div className="cam-label">Exposure ({isMe ? "manual" : "auto"} · {tag("exposure")})</div>
          <div className="cam-seg">
            <button type="button" disabled={busy} className={isAe ? "btn primary" : "btn"} onClick={() => send({ exposure: { mode: "ae" } }, true)}>Auto</button>
            <button
              type="button"
              disabled={busy}
              className={isMe ? "btn primary" : "btn"}
              onClick={() => send({ exposure: { mode: "me" } }, restartOf("exposure"))}
            >
              Manual
            </button>
          </div>
          <RangeCommit
            label="EV"
            digits={1}
            min={-4}
            max={4}
            step={0.5}
            disabled={busy || !isAe}
            value={Number(state.exposure?.ev ?? 0)}
            onCommit={(v) => send({ exposure: { mode: "ae", ev: v } }, restartOf("exposure"))}
          />
          <RangeCommit
            label="Shutter"
            unit="µs"
            digits={0}
            min={100}
            max={33000}
            step={100}
            disabled={busy || !isMe}
            value={Number(
              isMe
                ? (state.exposure?.shutter_us ?? state.exposure?.seed_shutter_us ?? meHold.shutter_us)
                : (state.exposure?.seed_shutter_us ?? state.exposure?.shutter_us ?? meHold.shutter_us)
            )}
            onCommit={(v) => send({ exposure: { mode: "me", shutter_us: v, gain: meHold.gain } }, restartOf("exposure"))}
          />
          <RangeCommit
            label="Gain"
            digits={1}
            min={1}
            max={12}
            step={0.1}
            disabled={busy || !isMe}
            value={Number(
              isMe
                ? (state.exposure?.gain ?? state.exposure?.seed_gain ?? meHold.gain)
                : (state.exposure?.seed_gain ?? state.exposure?.gain ?? meHold.gain)
            )}
            onCommit={(v) => send({ exposure: { mode: "me", shutter_us: meHold.shutter_us, gain: v } }, restartOf("exposure"))}
          />

          <div className="cam-label">White balance ({isWbManual ? "manual" : "auto"} · {tag("wb")})</div>
          <div className="cam-seg">
            <button type="button" disabled={busy} className={isWbAuto ? "btn primary" : "btn"} onClick={() => send({ wb: { mode: "auto" } }, restartOf("wb"))}>Auto</button>
            <button
              type="button"
              disabled={busy}
              className={isWbManual ? "btn primary" : "btn"}
              onClick={() => send({ wb: { mode: "manual" } }, restartOf("wb"))}
            >
              Manual
            </button>
          </div>
          <RangeCommit
            label="R"
            digits={2}
            min={0.5}
            max={8}
            step={0.05}
            disabled={busy || !isWbManual}
            value={Number(
              isWbManual
                ? (state.wb?.gains?.[0] ?? state.wb?.seed_gains?.[0] ?? wbHold[0])
                : (state.wb?.seed_gains?.[0] ?? state.wb?.gains?.[0] ?? wbHold[0])
            )}
            onCommit={(v) => send({ wb: { mode: "manual", gains: [v, wbHold[1]] } }, restartOf("wb"))}
          />
          <RangeCommit
            label="B"
            digits={2}
            min={0.5}
            max={8}
            step={0.05}
            disabled={busy || !isWbManual}
            value={Number(
              isWbManual
                ? (state.wb?.gains?.[1] ?? state.wb?.seed_gains?.[1] ?? wbHold[1])
                : (state.wb?.seed_gains?.[1] ?? state.wb?.gains?.[1] ?? wbHold[1])
            )}
            onCommit={(v) => send({ wb: { mode: "manual", gains: [wbHold[0], v] } }, restartOf("wb"))}
          />
        </div>
      </div>
      <div className="cam-note">{note || `${state.width || "—"}x${state.height || "—"} @ ${state.fps || "—"} fps`}</div>
      </div>
    </div>
  );
}

function RangeCommit({ label, value, min, max, step, digits, unit = "", display, disabled, onCommit }) {
  const [draft, setDraft] = useState(value);
  useEffect(() => { setDraft(value); }, [value]);
  const shown = display != null
    ? (draft === value ? display : (
      Number(draft) >= max - 1e-6 ? "∞" : Number(draft).toFixed(digits)
    ))
    : Number(draft).toFixed(digits);
  return (
    <label className={`cam-slider${disabled ? " disabled" : ""}`}>
      {label} {shown}{unit && shown !== "∞" ? ` ${unit}` : shown === "∞" ? "" : ""}
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        disabled={disabled}
        value={draft}
        onChange={(ev) => {
          if (disabled) return;
          setDraft(Number(ev.target.value));
        }}
        onPointerUp={(ev) => {
          if (disabled) return;
          onCommit(Number(ev.currentTarget.value));
        }}
        onKeyUp={(ev) => {
          if (disabled) return;
          if (ev.key === "Enter" || ev.key === "ArrowLeft" || ev.key === "ArrowRight") {
            onCommit(Number(ev.currentTarget.value));
          }
        }}
      />
    </label>
  );
}
