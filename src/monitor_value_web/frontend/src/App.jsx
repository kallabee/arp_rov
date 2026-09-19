import { useEffect, useRef, useState } from "react";
import { AlertAudio } from "./audio.js";
import { AttitudePanel } from "./Attitude.jsx";
import { fmt } from "./format.js";
import { Bar, Dial, Thermo } from "./Meters.jsx";

const EMPTY = { monitor: null, attitude: null, health: { publisher: "never", imu: "never" } };

export default function App() {
  const [cfg, setCfg] = useState(null);
  const [state, setState] = useState(EMPTY);
  const [stream, setStream] = useState("connecting");
  const [armed, setArmed] = useState(true);
  const [muted, setMuted] = useState(false);
  const audioRef = useRef(null);
  const statusRef = useRef({ leak: false, publisher: "never", stream: "connecting" });

  useEffect(() => {
    const prefs = AlertAudio.loadPrefs();
    setMuted(prefs.muted);
    const audio = new AlertAudio();
    audio.armed = true;
    audio.muted = prefs.muted;
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
    es.addEventListener("state", (ev) => {
      setStream("connected");
      try {
        const data = JSON.parse(ev.data);
        setState(data);
        if (data.config) setCfg(data.config);
      } catch (_) { /* ignore malformed */ }
    });
    es.onerror = () => setStream("disconnected");
    return () => es.close();
  }, []);

  const monitor = state.monitor;
  const health = state.health || {};
  const leak = Boolean(monitor?.water_ch0_detected || monitor?.water_ch1_detected || health.leak);
  const gauges = cfg?.gauges || {};
  const temps = cfg?.temperatures || [];
  const tempGauge = cfg?.temperature_gauge || { min: 0, max: 80 };
  const leaks = cfg?.leaks || [];

  statusRef.current = { leak, publisher: health.publisher, stream };

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
    audioRef.current?.setMuted(next);
  };

  return (
    <div className="app">
      <header className="header">
        <div className="brand">ROV Monitor</div>
        <Lamp name="接続" status={stream === "connected" ? "live" : stream} />
        <Lamp name="Publisher" status={health.publisher || "never"} />
        <Lamp name="IMU" status={health.imu || "never"} />
        <div className="meta">
          <span>{monitor?.stamp_jst || "---"}</span>
          <span>seq {monitor?.seq ?? "---"}</span>
          <span>elapsed {monitor?.elapsed_hms || "---"}</span>
        </div>
        <div className="header-actions">
          <span className="lamp live">警報 ON</span>
          <button className={muted ? "btn" : "btn armed"} onClick={toggleMute}>
            {muted ? "ミュート中" : "音あり"}
          </button>
        </div>
      </header>
      {stream === "disconnected" && (
        <div className="banner">データストリーム切断 — publisher / ノードを確認</div>
      )}
      {stream === "connected" && (health.publisher === "stale" || health.publisher === "never") && (
        <div className="banner">rov/monitor_value が途絶 — monitor_value_pub を確認</div>
      )}

      <div className="layout">
        <div className="col">
          <LeakCard monitor={monitor} leak={leak} leaks={leaks} />
          <div className="card">
            <h2>深度 · MS5837</h2>
            <div className="gauge-row">
              <Dial
                label="深度 (海水密度換算)"
                unit="m"
                value={monitor?.depth_m}
                spec={gauges.depth_m}
                digits={2}
              />
              <Dial
                label="生値"
                unit="atm"
                value={monitor?.depth_pressure_atm}
                spec={gauges.depth_pressure_atm}
                digits={3}
              />
              <Dial
                label="温度"
                unit="°C"
                value={monitor?.depth_temp_c}
                spec={tempGauge}
                digits={1}
              />
            </div>
          </div>
          <div className="card card-fill">
            <h2>温度</h2>
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
            <h2>内殻</h2>
            <div className="bars">
              <Bar label="湿度" unit="%" value={monitor?.bme_humidity_percent} spec={gauges.bme_humidity_percent} />
              <Bar label="気圧" unit="atm" value={monitor?.bme_pressure_atm} spec={gauges.bme_pressure_atm} digits={3} />
            </div>
          </div>
        </div>
        <div className="col col-right">
          <AttitudePanel attitude={state.attitude} />
          <div className="card card-compact">
            <h2>電源</h2>
            <div className="gauge-row gauge-row-4">
              <Dial label="電圧" unit="V" value={monitor?.voltage_v} spec={gauges.voltage_v} digits={1} />
              <Dial label="電流" unit="A" value={monitor?.current_a} spec={gauges.current_a} digits={2} />
              <Dial label="電力" unit="W" value={monitor?.power_w} spec={gauges.power_w} digits={1} />
              <Dial label="残量" unit="%" value={monitor?.remaining_percent} spec={gauges.remaining_percent} digits={0} />
            </div>
            <div className="kv" style={{ marginTop: 8 }}>
              <span>積算</span><b>{fmt(monitor?.accumulated_energy_wh, 2)} Wh</b>
              <span>ピーク</span><b>{fmt(monitor?.peak_power_w, 1)} W</b>
            </div>
          </div>
          <div className="card card-compact">
            <h2>RPi</h2>
            <div className="gauge-row">
              <Dial label="CPU" unit="%" value={monitor?.rpi_cpu_util_percent} spec={gauges.rpi_cpu_util_percent} digits={1} />
              <Dial label="GPU" unit="%" value={monitor?.rpi_gpu_util_percent} spec={gauges.rpi_gpu_util_percent} digits={1} />
              <Dial label="Fan" unit="rpm" value={monitor?.rpi_fan_rpm} spec={gauges.rpi_fan_rpm} digits={0} />
            </div>
          </div>
        </div>
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
  if (s === "never") return "なし";
  if (s === "connecting") return "接続中";
  if (s === "disconnected") return "切断";
  return s;
}

function LeakCard({ monitor, leak, leaks }) {
  const byCh = Object.fromEntries((leaks || []).map((x) => [Number(x.channel), x.nickname]));
  return (
    <div className={leak ? "card alarm" : "card"}>
      <h2>漏水</h2>
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
    <div className={hot ? "ch hot" : "ch"}>
      <div className="name">{n}</div>
      <div className="big">{fmt(v, 2)} V</div>
      <div>{hot ? "DETECTED" : "ok"}</div>
    </div>
  );
}
