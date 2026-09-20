import { useEffect, useRef, useState } from "react";
import { AlertAudio } from "./audio.js";
import { AttitudePanel } from "./Attitude.jsx";
import { CameraPanel } from "./Camera.jsx";
import { ControlDock } from "./Controls.jsx";
import { fmt } from "./format.js";
import { Bar, Dial, Thermo } from "./Meters.jsx";

const EMPTY = { monitor: null, attitude: null, health: { publisher: "never", imu: "never" } };

export default function App() {
  const [cfg, setCfg] = useState(null);
  const [state, setState] = useState(EMPTY);
  const [stream, setStream] = useState("connecting");
  const [armed, setArmed] = useState(true);
  const [muted, setMuted] = useState(false);
  const [fullscreen, setFullscreen] = useState(false);
  const audioRef = useRef(null);
  const statusRef = useRef({ leak: false, publisher: "never", stream: "connecting" });
  const appRef = useRef(null);

  useEffect(() => {
    const prefs = AlertAudio.loadPrefs();
    setMuted(prefs.quietAdvisories);
    const audio = new AlertAudio();
    audio.armed = true;
    audio.quietAdvisories = prefs.quietAdvisories;
    audioRef.current = audio;
    audio.arm().then(() => setArmed(true)).catch(() => {});
    const unlock = () => {
      audio.arm().then(() => {
        setArmed(true);
        audio.update(statusRef.current);
      }).catch(() => {});
    };
    window.addEventListener("pointerdown", unlock, { once: true });
    window.addEventListener("keydown", unlock, { once: true });
    fetch("/api/config")
      .then((r) => r.json())
      .then(setCfg)
      .catch(() => setCfg({ temperatures: [], gauges: {} }));
    return () => {
      window.removeEventListener("pointerdown", unlock);
      window.removeEventListener("keydown", unlock);
    };
  }, []);

  useEffect(() => {
    const es = new EventSource("/api/stream");
    let disconnectTimer = null;
    const markConnected = () => {
      if (disconnectTimer) {
        clearTimeout(disconnectTimer);
        disconnectTimer = null;
      }
      setStream("connected");
    };
    es.addEventListener("state", (ev) => {
      markConnected();
      try {
        const data = JSON.parse(ev.data);
        setState(data);
        if (data.config) setCfg(data.config);
      } catch (_) { /* ignore malformed */ }
    });
    // EventSource fires error on transient reconnects — wait before alarming.
    es.onerror = () => {
      if (disconnectTimer) return;
      disconnectTimer = setTimeout(() => {
        disconnectTimer = null;
        setStream("disconnected");
      }, 3500);
    };
    return () => {
      if (disconnectTimer) clearTimeout(disconnectTimer);
      es.close();
    };
  }, []);

  const monitor = state.monitor;
  const health = state.health || {};
  const leak = Boolean(monitor?.water_ch0_detected || monitor?.water_ch1_detected || health.leak);
  const gauges = cfg?.gauges || {};
  const temps = cfg?.temperatures || [];
  const tempGauge = cfg?.temperature_gauge || { min: 0, max: 80 };
  const leaks = cfg?.leaks || [];

  statusRef.current = { leak, publisher: health.publisher, stream };

  const alertTone = leak
    ? { cls: "alarm", text: "Siren · water leak" }
    : muted
      ? { cls: "quiet", text: "Advisories off (siren on)" }
      : stream === "disconnected"
        ? { cls: "stale", text: "Pulse · stream lost (~5s)" }
        : (health.publisher === "stale" || health.publisher === "never")
          ? { cls: "stale", text: "Beep · publisher stale (~20s)" }
          : { cls: "quiet", text: "Alerts quiet" };

  useEffect(() => {
    audioRef.current?.update({
      leak,
      publisher: health.publisher,
      stream,
    });
  }, [leak, health.publisher, stream, armed, muted]);

  const toggleMute = () => {
    const next = !muted;
    setMuted(next);
    // Quiets stream/publisher advisories only — leak siren stays on.
    audioRef.current?.setQuietAdvisories(next);
  };

  useEffect(() => {
    const onFs = () => setFullscreen(Boolean(document.fullscreenElement));
    document.addEventListener("fullscreenchange", onFs);
    return () => document.removeEventListener("fullscreenchange", onFs);
  }, []);

  const toggleFullscreen = async () => {
    try {
      if (document.fullscreenElement) {
        await document.exitFullscreen();
      } else {
        const el = appRef.current || document.documentElement;
        await el.requestFullscreen?.();
      }
    } catch (_) { /* ignore */ }
  };

  return (
    <div className="app" ref={appRef}>
      <header className="header">
        <div className="brand">Kankai Control Panel</div>
        <Lamp name="Link" status={stream === "connected" ? "live" : stream} />
        <Lamp name="Publisher" status={health.publisher || "never"} />
        <Lamp name="IMU" status={health.imu || "never"} />
        <div className="meta">
          <span>{monitor?.stamp_jst || "---"}</span>
          <span>seq {monitor?.seq ?? "---"}</span>
          <span>elapsed {monitor?.elapsed_hms || "---"}</span>
        </div>
        <div className="header-actions">
          <span
            className={`lamp ${alertTone.cls}`}
            title="Siren = water leak (never muted). Soft pulse ~5s = data stream lost. Soft beep ~20s = publisher stale."
          >
            <i /> {alertTone.text}
          </span>
          <button
            className={muted ? "btn" : "btn armed"}
            onClick={toggleMute}
            title="Turn off advisory pulse/beep only. Leak siren always stays on."
          >
            {muted ? "Advisories off" : "Advisories on"}
          </button>
          <button className="btn" type="button" onClick={toggleFullscreen}>
            {fullscreen ? "Exit full" : "Fullscreen"}
          </button>
        </div>
      </header>
      {stream === "disconnected" && (
        <div className="banner">Data stream disconnected — check publisher / node</div>
      )}
      {stream === "connected" && (health.publisher === "stale" || health.publisher === "never") && (
        <div className="banner">rov/monitor_value stalled — check monitor_value_pub</div>
      )}

      <div className="layout">
        <div className="col">
          <LeakCard monitor={monitor} leak={leak} leaks={leaks} />
          <div className="card">
            <h2>Depth</h2>
            <div className="gauge-row">
              <Dial
                label="Depth (seawater)"
                unit="m"
                value={monitor?.depth_m}
                spec={gauges.depth_m}
                digits={2}
              />
              <Dial
                label="Raw"
                unit="atm"
                value={monitor?.depth_pressure_atm}
                spec={gauges.depth_pressure_atm}
                digits={3}
              />
              <Dial
                label="Temp"
                unit="°C"
                value={monitor?.depth_temp_c}
                spec={tempGauge}
                digits={1}
              />
            </div>
          </div>
          <div className="card card-fill">
            <h2>Temperature</h2>
            <div className="thermo-row">
              {temps.filter((t) => t.key !== "depth_temp_c").map((t) => (
                <Thermo
                  key={t.key}
                  nickname={t.nickname || t.key}
                  value={monitor ? monitor[t.key] : null}
                  spec={tempGauge}
                />
              ))}
            </div>
          </div>
          <div className="card">
            <h2>Hull interior</h2>
            <div className="bars">
              <Bar label="Humidity" unit="%" value={monitor?.bme_humidity_percent} spec={gauges.bme_humidity_percent} />
              <Bar label="Pressure" unit="atm" value={monitor?.bme_pressure_atm} spec={gauges.bme_pressure_atm} digits={3} tickDigits={1} />
            </div>
          </div>
        </div>
        <div className="col col-right">
          <AttitudePanel attitude={state.attitude} />
          <div className="card card-compact">
            <h2>Power</h2>
            <div className="gauge-row gauge-row-4">
              <Dial label="Voltage" unit="V" value={monitor?.voltage_v} spec={gauges.voltage_v} digits={1} />
              <Dial label="Current" unit="A" value={monitor?.current_a} spec={gauges.current_a} digits={2} />
              <Dial label="Power" unit="W" value={monitor?.power_w} spec={gauges.power_w} digits={1} />
              <Dial label="Remaining" unit="%" value={monitor?.remaining_percent} spec={gauges.remaining_percent} digits={0} />
            </div>
            <div className="kv" style={{ marginTop: 8 }}>
              <span>Energy</span><b>{fmt(monitor?.accumulated_energy_wh, 2)} Wh</b>
              <span>Peak</span><b>{fmt(monitor?.peak_power_w, 1)} W</b>
            </div>
          </div>
          <div className="card card-compact">
            <h2>RPi</h2>
            <div className="gauge-row gauge-row-4">
              <Dial label="CPU" unit="%" value={monitor?.rpi_cpu_util_percent} spec={gauges.rpi_cpu_util_percent} digits={1} />
              <Dial label="GPU" unit="%" value={monitor?.rpi_gpu_util_percent} spec={gauges.rpi_gpu_util_percent} digits={1} />
              <Dial label="Fan(PWM)" unit="%" value={monitor?.rpi_fan_pwm_percent} spec={gauges.rpi_fan_pwm_percent} digits={0} />
              <Dial label="Fan" unit="rpm" value={monitor?.rpi_fan_rpm} spec={gauges.rpi_fan_rpm} digits={0} />
            </div>
          </div>
        </div>
        <ControlDock
          command={state.command}
          lights={cfg?.lights}
          stepPercent={cfg?.command?.step_percent ?? 5}
        >
          <CameraPanel config={cfg} />
        </ControlDock>
      </div>
    </div>
  );
}

function Lamp({ name, status }) {
  const cls = status === "live" ? "live"
    : status === "stale" || status === "reconnecting" || status === "connecting" ? "stale"
    : "disconnected";
  return (
    <span className={`lamp ${cls}`}>
      <i /> {name} {labelOf(status)}
    </span>
  );
}

function labelOf(s) {
  if (s === "live") return "live";
  if (s === "stale") return "stale";
  if (s === "never") return "none";
  if (s === "connecting") return "connecting";
  if (s === "disconnected") return "disconnected";
  return s;
}

function LeakCard({ monitor, leak, leaks }) {
  const byCh = Object.fromEntries((leaks || []).map((x) => [Number(x.channel), x.nickname]));
  return (
    <div className={leak ? "card alarm" : "card"}>
      <h2>Water Leakage</h2>
      <div className="leak-grid">
        <Chan
          n={byCh[0] || "ch0"}
          v={monitor?.water_ch0_probe_v}
          hot={monitor?.water_ch0_detected}
        />
        <Chan
          n={byCh[1] || "ch1"}
          v={monitor?.water_ch1_probe_v}
          hot={monitor?.water_ch1_detected}
        />
      </div>
    </div>
  );
}

function Chan({ n, v, hot }) {
  return (
    <div className={hot ? "ch hot blink" : "ch"}>
      <div className="name">{n}</div>
      <div className="big">{fmt(v, 2)} V</div>
      <div className="status">{hot ? "DETECTED" : "ok"}</div>
    </div>
  );
}
