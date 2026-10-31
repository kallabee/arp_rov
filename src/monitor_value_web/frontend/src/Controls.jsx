import { useCallback, useEffect, useRef, useState } from "react";
import { loadUiPrefs, saveUiPrefs } from "./prefs.js";

const TWIST_AXES = [
  { key: "x", label: "X" },
  { key: "y", label: "Y" },
  { key: "z", label: "Z" },
  { key: "yaw", label: "Yaw" },
  { key: "pitch", label: "Pitch" },
  { key: "roll", label: "Roll" },
];

const HAND_AXES = [
  { key: "grab", label: "Grip" },
  { key: "roll", label: "Roll" },
];

const ZERO_TWIST = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
const ZERO_HAND = { grab: 0, roll: 0 };
const ALL_TWIST_ON = Object.fromEntries(TWIST_AXES.map((ax) => [ax.key, true]));
const ALL_TWIST_OFF = Object.fromEntries(TWIST_AXES.map((ax) => [ax.key, false]));
const ALL_HAND_ON = Object.fromEntries(HAND_AXES.map((ax) => [ax.key, true]));
const ALL_HAND_OFF = Object.fromEntries(HAND_AXES.map((ax) => [ax.key, false]));
// thruster_controller decays cmd_vel / hand after 50ms without a new message.
const TWIST_KEEPALIVE_MS = 40;

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function toUnit(percent) {
  return clamp(percent, -100, 100) / 100;
}

function toPercent(unit, min = -100, max = 100) {
  return clamp((Number(unit) || 0) * 100, min, max);
}

function roundPct(v) {
  return Math.round(v);
}

function twistIsActive(t) {
  return TWIST_AXES.some((ax) => Math.abs(Number(t?.[ax.key]) || 0) > 1e-4);
}

function handIsActive(t) {
  return HAND_AXES.some((ax) => Math.abs(Number(t?.[ax.key]) || 0) > 1e-4);
}

function loadHandDefault() {
  const prefs = loadUiPrefs();
  const h = prefs.hand;
  if (!h || typeof h !== "object") return ZERO_HAND;
  return {
    grab: clamp(Number(h.grab) || 0, -1, 1),
    roll: clamp(Number(h.roll) || 0, -1, 1),
  };
}

async function postCommand(body) {
  const res = await fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`command ${res.status}`);
  return res.json();
}

export function ControlDock({ command, lights, stepPercent = 5, children }) {
  const lightNames = Array.isArray(lights) && lights.length ? lights : [];
  const [twist, setTwist] = useState(ZERO_TWIST);
  const [twistEnabled, setTwistEnabled] = useState(ALL_TWIST_ON);
  const [hand, setHand] = useState(loadHandDefault);
  const [handEnabled, setHandEnabled] = useState(ALL_HAND_ON);
  const [lightDuty, setLightDuty] = useState(() => Object.fromEntries(lightNames.map((n) => [n, 0])));
  const [lightEnabled, setLightEnabled] = useState(() => Object.fromEntries(lightNames.map((n) => [n, false])));
  const interacting = useRef({ twist: false, hand: false, lights: false });
  const pointerDown = useRef({ twist: false, hand: false, lights: false });
  const sending = useRef({ twist: false, hand: false, lights: false });
  const queued = useRef({ twist: null, hand: null, lights: null });
  const lightEnabledRef = useRef(lightEnabled);
  lightEnabledRef.current = lightEnabled;
  const twistEnabledRef = useRef(twistEnabled);
  twistEnabledRef.current = twistEnabled;
  const handEnabledRef = useRef(handEnabled);
  handEnabledRef.current = handEnabled;
  const latest = useRef({ twist, hand, lights: lightDuty });
  latest.current = { twist, hand, lights: lightDuty };

  const releaseGroup = (group) => {
    if (!pointerDown.current[group] && !sending.current[group] && queued.current[group] == null) {
      interacting.current[group] = false;
    }
  };

  const buildLightOutput = (duty, enabled) => {
    const out = {};
    for (const name of lightNames) {
      out[name] = enabled[name] ? (duty[name] ?? 0) : 0;
    }
    return out;
  };

  const buildTwistOutput = (duty, enabled) => {
    const out = {};
    for (const ax of TWIST_AXES) {
      out[ax.key] = enabled[ax.key] ? (duty[ax.key] ?? 0) : 0;
    }
    return out;
  };

  const buildHandOutput = (duty, enabled) => {
    const out = {};
    for (const ax of HAND_AXES) {
      out[ax.key] = enabled[ax.key] ? (duty[ax.key] ?? 0) : 0;
    }
    return out;
  };

  useEffect(() => {
    setLightDuty((prev) => {
      const next = {};
      for (const name of lightNames) next[name] = prev[name] ?? 0;
      return next;
    });
    setLightEnabled((prev) => {
      const next = {};
      for (const name of lightNames) next[name] = Boolean(prev[name]);
      return next;
    });
  }, [lights]);

  useEffect(() => {
    if (!command) return;
    if (!interacting.current.twist && command.twist) {
      setTwist((prev) => {
        const next = { ...prev };
        let changed = false;
        for (const ax of TWIST_AXES) {
          if (command.twist[ax.key] == null) continue;
          const v = clamp(Number(command.twist[ax.key]) || 0, -1, 1);
          // Keep slider when output is off (0); only adopt remote non-zero levels.
          if (Math.abs(v) > 1e-4 && next[ax.key] !== v) {
            next[ax.key] = v;
            changed = true;
          }
        }
        return changed ? next : prev;
      });
    }
    if (!interacting.current.hand && command.hand) {
      setHand((prev) => {
        const next = { ...prev };
        let changed = false;
        for (const ax of HAND_AXES) {
          if (command.hand[ax.key] == null) continue;
          const v = clamp(Number(command.hand[ax.key]) || 0, -1, 1);
          if (Math.abs(v) > 1e-4 && next[ax.key] !== v) {
            next[ax.key] = v;
            changed = true;
          }
        }
        return changed ? next : prev;
      });
    }
    if (!interacting.current.lights && command.lights) {
      setLightDuty((prev) => {
        const next = { ...prev };
        let changed = false;
        for (const name of lightNames) {
          if (command.lights[name] == null) continue;
          const v = clamp(Number(command.lights[name]) || 0, 0, 1);
          // Keep slider when output is off (0); only adopt remote non-zero levels.
          if (v > 0 && next[name] !== v) {
            next[name] = v;
            changed = true;
          }
        }
        return changed ? next : prev;
      });
      setLightEnabled((prev) => {
        const next = { ...prev };
        let changed = false;
        for (const name of lightNames) {
          if (command.lights[name] == null) continue;
          const on = (Number(command.lights[name]) || 0) > 0.001;
          if (Boolean(next[name]) !== on) {
            next[name] = on;
            changed = true;
          }
        }
        return changed ? next : prev;
      });
    }
  }, [command, lights, lightNames]);

  const sendGroup = useCallback((group, payload, opts = {}) => {
    if (!opts.keepalive) interacting.current[group] = true;
    queued.current[group] = payload;
    if (sending.current[group]) return;
    sending.current[group] = true;
    const pump = async () => {
      while (queued.current[group] != null) {
        const next = queued.current[group];
        queued.current[group] = null;
        try {
          if (group === "twist") await postCommand({ twist: next });
          else if (group === "hand") await postCommand({ hand: next });
          else await postCommand({ lights: next });
        } catch (_) { /* keep local UI even if publish fails */ }
      }
      sending.current[group] = false;
      releaseGroup(group);
    };
    pump();
  }, []);

  useEffect(() => {
    const id = setInterval(() => {
      if (typeof document !== "undefined" && document.hidden) return;
      const twistOut = buildTwistOutput(latest.current.twist, twistEnabledRef.current);
      if (twistIsActive(twistOut)) sendGroup("twist", twistOut, { keepalive: true });
      const handOut = buildHandOutput(latest.current.hand, handEnabledRef.current);
      if (handIsActive(handOut)) sendGroup("hand", handOut, { keepalive: true });
    }, TWIST_KEEPALIVE_MS);
    const stopActuators = () => {
      const body = JSON.stringify({ twist: ZERO_TWIST, hand: ZERO_HAND });
      try {
        navigator.sendBeacon?.("/api/command", new Blob([body], { type: "application/json" }));
      } catch (_) { /* ignore */ }
    };
    const onHide = () => {
      if (typeof document !== "undefined" && document.hidden) stopActuators();
    };
    window.addEventListener("pagehide", stopActuators);
    document.addEventListener("visibilitychange", onHide);
    return () => {
      clearInterval(id);
      window.removeEventListener("pagehide", stopActuators);
      document.removeEventListener("visibilitychange", onHide);
    };
  }, [sendGroup]);

  const changeTwist = (key, percent) => {
    const next = { ...latest.current.twist, [key]: toUnit(percent) };
    latest.current.twist = next;
    setTwist(next);
    sendGroup("twist", buildTwistOutput(next, twistEnabledRef.current));
  };

  const twistOff = (key) => {
    const enabled = { ...twistEnabledRef.current, [key]: false };
    twistEnabledRef.current = enabled;
    setTwistEnabled(enabled);
    sendGroup("twist", buildTwistOutput(latest.current.twist, enabled));
  };

  const twistOn = (key) => {
    const enabled = { ...twistEnabledRef.current, [key]: true };
    twistEnabledRef.current = enabled;
    setTwistEnabled(enabled);
    sendGroup("twist", buildTwistOutput(latest.current.twist, enabled));
  };

  const twistAllOff = () => {
    twistEnabledRef.current = ALL_TWIST_OFF;
    setTwistEnabled(ALL_TWIST_OFF);
    pointerDown.current.twist = false;
    queued.current.twist = null;
    sendGroup("twist", { ...ZERO_TWIST });
  };

  const twistAllOn = () => {
    twistEnabledRef.current = ALL_TWIST_ON;
    setTwistEnabled(ALL_TWIST_ON);
    pointerDown.current.twist = false;
    queued.current.twist = null;
    sendGroup("twist", buildTwistOutput(latest.current.twist, ALL_TWIST_ON));
  };

  const twistAllReset = () => {
    const next = { ...ZERO_TWIST };
    latest.current.twist = next;
    setTwist(next);
    sendGroup("twist", buildTwistOutput(next, twistEnabledRef.current));
  };

  const changeHand = (key, percent) => {
    const next = { ...latest.current.hand, [key]: toUnit(percent) };
    latest.current.hand = next;
    setHand(next);
    saveUiPrefs({ hand: next });
    sendGroup("hand", buildHandOutput(next, handEnabledRef.current));
  };

  const handOff = (key) => {
    const enabled = { ...handEnabledRef.current, [key]: false };
    handEnabledRef.current = enabled;
    setHandEnabled(enabled);
    sendGroup("hand", buildHandOutput(latest.current.hand, enabled));
  };

  const handOn = (key) => {
    const enabled = { ...handEnabledRef.current, [key]: true };
    handEnabledRef.current = enabled;
    setHandEnabled(enabled);
    sendGroup("hand", buildHandOutput(latest.current.hand, enabled));
  };

  const handAllOff = () => {
    handEnabledRef.current = ALL_HAND_OFF;
    setHandEnabled(ALL_HAND_OFF);
    pointerDown.current.hand = false;
    queued.current.hand = null;
    sendGroup("hand", { ...ZERO_HAND });
  };

  const handAllOn = () => {
    handEnabledRef.current = ALL_HAND_ON;
    setHandEnabled(ALL_HAND_ON);
    pointerDown.current.hand = false;
    queued.current.hand = null;
    sendGroup("hand", buildHandOutput(latest.current.hand, ALL_HAND_ON));
  };

  const handAllReset = () => {
    const next = { ...ZERO_HAND };
    latest.current.hand = next;
    setHand(next);
    saveUiPrefs({ hand: next });
    sendGroup("hand", buildHandOutput(next, handEnabledRef.current));
  };

  const changeLight = (name, percent) => {
    const unit = toUnit(percent);
    const nextDuty = { ...latest.current.lights, [name]: unit };
    setLightDuty(nextDuty);
    latest.current.lights = nextDuty;
    const enabled = { ...lightEnabledRef.current };
    sendGroup("lights", buildLightOutput(nextDuty, enabled));
  };

  const lightOff = (name) => {
    const enabled = { ...lightEnabledRef.current, [name]: false };
    lightEnabledRef.current = enabled;
    setLightEnabled(enabled);
    sendGroup("lights", buildLightOutput(latest.current.lights, enabled));
  };

  const lightOn = (name) => {
    const duty = { ...latest.current.lights };
    if (!(duty[name] > 0)) duty[name] = 1;
    const enabled = { ...lightEnabledRef.current, [name]: true };
    lightEnabledRef.current = enabled;
    setLightDuty(duty);
    latest.current.lights = duty;
    setLightEnabled(enabled);
    sendGroup("lights", buildLightOutput(duty, enabled));
  };

  const estop = () => {
    interacting.current.twist = true;
    interacting.current.hand = true;
    pointerDown.current.twist = false;
    pointerDown.current.hand = false;
    queued.current.twist = null;
    queued.current.hand = null;
    setTwist(ZERO_TWIST);
    latest.current.twist = { ...ZERO_TWIST };
    twistEnabledRef.current = ALL_TWIST_OFF;
    setTwistEnabled(ALL_TWIST_OFF);
    setHand(ZERO_HAND);
    latest.current.hand = { ...ZERO_HAND };
    handEnabledRef.current = ALL_HAND_OFF;
    setHandEnabled(ALL_HAND_OFF);
    saveUiPrefs({ hand: ZERO_HAND });
    sending.current.twist = true;
    sending.current.hand = true;
    postCommand({ estop: true }).finally(() => {
      sending.current.twist = false;
      sending.current.hand = false;
      releaseGroup("twist");
      releaseGroup("hand");
    });
  };

  const lightsAllOff = () => {
    const enabled = Object.fromEntries(lightNames.map((name) => [name, false]));
    lightEnabledRef.current = enabled;
    setLightEnabled(enabled);
    pointerDown.current.lights = false;
    queued.current.lights = null;
    sendGroup("lights", buildLightOutput(latest.current.lights, enabled));
  };

  const lightsAllOn = () => {
    const duty = Object.fromEntries(lightNames.map((name) => [name, 1]));
    const enabled = Object.fromEntries(lightNames.map((name) => [name, true]));
    lightEnabledRef.current = enabled;
    setLightDuty(duty);
    latest.current.lights = duty;
    setLightEnabled(enabled);
    pointerDown.current.lights = false;
    queued.current.lights = null;
    sendGroup("lights", buildLightOutput(duty, enabled));
  };

  const lightsAllReset = () => {
    const duty = Object.fromEntries(lightNames.map((name) => [name, 0]));
    setLightDuty(duty);
    latest.current.lights = duty;
    sendGroup("lights", buildLightOutput(duty, lightEnabledRef.current));
  };

  const onFaderActive = (group, active) => {
    pointerDown.current[group] = active;
    if (active) interacting.current[group] = true;
    else releaseGroup(group);
  };

  const handSeeded = useRef(false);
  useEffect(() => {
    if (handSeeded.current) return;
    handSeeded.current = true;
    const h = loadHandDefault();
    latest.current.hand = h;
    if (handIsActive(h)) {
      sendGroup("hand", buildHandOutput(h, handEnabledRef.current));
    }
  }, [sendGroup]);

  return (
    <div className="control-dock">
      <div className="control-top">
        <div className="control-lights-wrap">
          <div className="card control-card">
            <h2>Thrusters</h2>
            <div className="fader-row">
              {TWIST_AXES.map((ax) => (
                <Fader
                  key={ax.key}
                  label={ax.label}
                  value={toPercent(twist[ax.key])}
                  min={-100}
                  max={100}
                  step={stepPercent}
                  bipolar
                  outputSwitch
                  lit={Boolean(twistEnabled[ax.key])}
                  onChange={(v) => changeTwist(ax.key, v)}
                  onOff={() => twistOff(ax.key)}
                  onOn={() => twistOn(ax.key)}
                  onActive={(a) => onFaderActive("twist", a)}
                />
              ))}
            </div>
          </div>
          <div className="lights-bulk">
            <button className="lights-on" type="button" onClick={twistAllOn}>
              <span>All on</span>
              <b>I</b>
            </button>
            <button className="lights-off" type="button" onClick={twistAllOff}>
              <span>All off</span>
              <b>O</b>
            </button>
            <button className="lights-off" type="button" onClick={twistAllReset}>
              <span>All reset</span>
              <b>0</b>
            </button>
          </div>
        </div>
        <div className="control-lights-wrap">
          <div className="card control-card">
            <h2>Manipulators</h2>
            <div className="fader-row">
              {HAND_AXES.map((ax) => (
                <Fader
                  key={ax.key}
                  label={ax.label}
                  value={toPercent(hand[ax.key])}
                  min={-100}
                  max={100}
                  step={stepPercent}
                  bipolar
                  outputSwitch
                  lit={Boolean(handEnabled[ax.key])}
                  onChange={(v) => changeHand(ax.key, v)}
                  onOff={() => handOff(ax.key)}
                  onOn={() => handOn(ax.key)}
                  onActive={(a) => onFaderActive("hand", a)}
                />
              ))}
            </div>
          </div>
          <div className="lights-bulk">
            <button className="lights-on" type="button" onClick={handAllOn}>
              <span>All on</span>
              <b>I</b>
            </button>
            <button className="lights-off" type="button" onClick={handAllOff}>
              <span>All off</span>
              <b>O</b>
            </button>
            <button className="lights-off" type="button" onClick={handAllReset}>
              <span>All reset</span>
              <b>0</b>
            </button>
          </div>
        </div>
        <button className="estop" type="button" onClick={estop}>
          <span>E-STOP</span>
          <b>N</b>
        </button>
      </div>
      <div className="control-lights-wrap">
        <div className="card control-card control-lights">
          <h2>Lights</h2>
          <div className="fader-row">
            {lightNames.map((name) => (
              <Fader
                key={name}
                label={name}
                value={toPercent(lightDuty[name] ?? 0, 0, 100)}
                min={0}
                max={100}
                step={stepPercent}
                outputSwitch
                lit={Boolean(lightEnabled[name])}
                onChange={(v) => changeLight(name, v)}
                onOff={() => lightOff(name)}
                onOn={() => lightOn(name)}
                onActive={(a) => onFaderActive("lights", a)}
              />
            ))}
          </div>
        </div>
        <div className="lights-bulk">
          <button className="lights-on" type="button" onClick={lightsAllOn}>
            <span>All on</span>
            <b>100</b>
          </button>
          <button className="lights-off" type="button" onClick={lightsAllOff}>
            <span>All off</span>
            <b>O</b>
          </button>
          <button className="lights-off" type="button" onClick={lightsAllReset}>
            <span>All reset</span>
            <b>0</b>
          </button>
        </div>
      </div>
      {children}
    </div>
  );
}

function Fader({
  label,
  value,
  min,
  max,
  step,
  bipolar = false,
  outputSwitch = false,
  lit = false,
  onChange,
  onOff,
  onOn,
  onActive,
}) {
  const trackRef = useRef(null);
  const holdTimerRef = useRef(null);
  const holdRepeatRef = useRef(null);
  const pct = roundPct(clamp(value, min, max));
  const pctRef = useRef(pct);
  pctRef.current = pct;
  const span = max - min || 1;
  const t = (pct - min) / span;
  const fillFrom = bipolar ? (0 - min) / span : 0;
  const fillTop = (1 - Math.max(t, fillFrom)) * 100;
  const fillBottom = Math.min(t, fillFrom) * 100;
  const knobTop = (1 - t) * 100;

  const setFromClientY = (clientY) => {
    const el = trackRef.current;
    if (!el) return;
    const rect = el.getBoundingClientRect();
    const ratio = clamp((clientY - rect.top) / rect.height, 0, 1);
    onChange(roundPct(max - ratio * span));
  };

  const nudge = (dir) => {
    const next = roundPct(clamp(pctRef.current + dir * step, min, max));
    pctRef.current = next;
    onChange(next);
  };

  const startHold = (dir) => {
    onActive?.(true);
    clearHold();
    nudge(dir);
    holdTimerRef.current = setTimeout(() => {
      holdTimerRef.current = null;
      holdRepeatRef.current = setInterval(() => nudge(dir), 70);
    }, 280);
  };

  const clearHold = () => {
    if (holdTimerRef.current) {
      clearTimeout(holdTimerRef.current);
      holdTimerRef.current = null;
    }
    if (holdRepeatRef.current) {
      clearInterval(holdRepeatRef.current);
      holdRepeatRef.current = null;
    }
  };

  const onPointerDownTrack = (ev) => {
    ev.preventDefault();
    onActive?.(true);
    ev.currentTarget.setPointerCapture(ev.pointerId);
    setFromClientY(ev.clientY);
  };

  const onPointerMoveTrack = (ev) => {
    if (!ev.currentTarget.hasPointerCapture(ev.pointerId)) return;
    setFromClientY(ev.clientY);
  };

  const endActive = () => {
    clearHold();
    onActive?.(false);
  };

  useEffect(() => () => {
    if (holdTimerRef.current) clearTimeout(holdTimerRef.current);
    if (holdRepeatRef.current) clearInterval(holdRepeatRef.current);
  }, []);

  return (
    <div className={`fader${outputSwitch && lit ? " lit" : ""}`}>
      <div className="fader-val">{pct}%</div>
      <button
        type="button"
        className="fader-btn"
        aria-label={`${label} plus`}
        onPointerDown={(e) => {
          e.preventDefault();
          e.currentTarget.setPointerCapture(e.pointerId);
          startHold(1);
        }}
        onPointerUp={endActive}
        onPointerCancel={endActive}
        onLostPointerCapture={endActive}
      >
        +
      </button>
      <div
        className="fader-track"
        ref={trackRef}
        role="slider"
        aria-label={label}
        aria-valuemin={min}
        aria-valuemax={max}
        aria-valuenow={pct}
        onPointerDown={onPointerDownTrack}
        onPointerMove={onPointerMoveTrack}
        onPointerUp={endActive}
        onPointerCancel={endActive}
      >
        {bipolar && <i className="fader-zero" />}
        <div
          className="fader-fill"
          style={{ top: `${fillTop}%`, bottom: `${fillBottom}%` }}
        />
        <div className="fader-knob" style={{ top: `${knobTop}%` }} />
      </div>
      <button
        type="button"
        className="fader-btn"
        aria-label={`${label} minus`}
        onPointerDown={(e) => {
          e.preventDefault();
          e.currentTarget.setPointerCapture(e.pointerId);
          startHold(-1);
        }}
        onPointerUp={endActive}
        onPointerCancel={endActive}
        onLostPointerCapture={endActive}
      >
        −
      </button>
      {outputSwitch ? (
        <div className="fader-switch">
          <button
            type="button"
            className={`fader-on-btn${lit ? " active" : ""}`}
            aria-label={`${label} on`}
            onClick={() => { onActive?.(true); onOn?.(); onActive?.(false); }}
          >
            On
          </button>
          <button
            type="button"
            className={`fader-off-btn${!lit ? " active" : ""}`}
            aria-label={`${label} off`}
            onClick={() => { onActive?.(true); onOff?.(); onActive?.(false); }}
          >
            Off
          </button>
        </div>
      ) : null}
      <button
        type="button"
        className="fader-zero-btn"
        aria-label={`${label} reset 0`}
        onClick={() => { onActive?.(true); onChange(0); onActive?.(false); }}
      >
        0
      </button>
      <div className="fader-label">{label}</div>
    </div>
  );
}
