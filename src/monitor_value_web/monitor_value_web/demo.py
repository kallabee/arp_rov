from __future__ import annotations

import math
import time
from pathlib import Path

from monitor_value_web.config import load_web_config
from monitor_value_web.http_server import DashboardState, start_http_server


def _cfg_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "monitor_value_web.yaml"


def _static_root() -> Path:
    return Path(__file__).resolve().parents[1] / "frontend" / "dist"


def _tick(state: DashboardState, t: float) -> None:
    roll = 12.0 * math.sin(t * 0.4)
    pitch = 8.0 * math.sin(t * 0.27)
    yaw = (t * 18.0) % 360.0 - 180.0
    leak = (int(t) // 12) % 8 == 7
    monitor = {
        "seq": int(t * 10),
        "stamp": None,
        "stamp_jst": time.strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_hms": time.strftime("%H:%M:%S", time.gmtime(t)),
        "elapsed_since_start_sec": t,
        "rpi_fan_rpm": 3200.0,
        "rpi_cpu_temp_c": 58.0 + 3.0 * math.sin(t * 0.1),
        "rpi_cpu_util_percent": 35.0,
        "rpi_gpu_util_percent": 12.0,
        "mag_x": 0.2,
        "mag_y": -0.1,
        "mag_z": 0.4,
        "gyro_x": 0.01,
        "gyro_y": -0.02,
        "gyro_z": 0.0,
        "accel_x": 0.1,
        "accel_y": -0.05,
        "accel_z": 9.7,
        "pose_x": 0.0,
        "pose_y": 0.0,
        "pose_z": 1.2 + 0.05 * math.sin(t * 0.2),
        "ori_x": None,
        "ori_y": None,
        "ori_z": None,
        "ori_w": None,
        "depth_m": 1.2 + 0.05 * math.sin(t * 0.2),
        "depth_temp_c": 18.5,
        "depth_pressure_atm": 1.12,
        "bme_temp_c": 26.4,
        "bme_pressure_atm": 1.01,
        "bme_humidity_percent": 48.0,
        "current_a": 4.2 + math.sin(t * 0.5),
        "voltage_v": 14.6 - 0.1 * math.sin(t * 0.05),
        "power_w": 60.0 + 8.0 * math.sin(t * 0.5),
        "accumulated_energy_wh": 12.3,
        "remaining_percent": 72.0,
        "peak_power_w": 95.0,
        "thermocouple_ch2_temp_c": 42.0,
        "thermocouple_ch3_temp_c": 22.0,
        "ds18b20_ch0_temp_c": 31.0,
        "ds18b20_ch1_temp_c": 36.0,
        "water_ch0_probe_v": 3.4 if not leak else 1.1,
        "water_ch1_probe_v": 3.5,
        "water_ch0_detected": leak,
        "water_ch1_detected": False,
        "roll_deg": roll,
        "pitch_deg": pitch,
        "yaw_deg": yaw,
    }
    state.set_monitor(monitor)
    state.set_attitude(
        {
            "seq": int(t * 10),
            "stamp": None,
            "ori_x": None,
            "ori_y": None,
            "ori_z": None,
            "ori_w": None,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "yaw_deg": yaw,
        }
    )


def main() -> None:
    cfg_path = _cfg_path()
    cfg = load_web_config(cfg_path)
    state = DashboardState(cfg, cfg_path)
    static_root = _static_root()
    start_http_server(state, static_root, cfg["http"]["host"], cfg["http"]["port"])
    print(f"demo dashboard http://0.0.0.0:{cfg['http']['port']}/  static={static_root}", flush=True)
    t0 = time.monotonic()
    while True:
        _tick(state, time.monotonic() - t0)
        time.sleep(0.1)


if __name__ == "__main__":
    main()

