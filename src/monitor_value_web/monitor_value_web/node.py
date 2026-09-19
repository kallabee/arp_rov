from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from monitor_value_interfaces.msg import ImuNavSnapshot, MonitorValue
from monitor_value_web.config import load_web_config
from monitor_value_web.convert import monitor_to_dict, snapshot_to_attitude
from monitor_value_web.http_server import DashboardState, start_http_server


def _cfg_default_path() -> Path:
    # Prefer the workspace YAML next to this package so nickname edits apply
    # without rebuilding the install-space copy.
    src = Path(__file__).resolve().parents[1] / "config" / "monitor_value_web.yaml"
    if src.is_file():
        return src
    env = os.environ.get("MONITOR_VALUE_WEB_CONFIG")
    if env:
        p = Path(env)
        if p.is_file():
            return p
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("monitor_value_web"))
        cand = share / "config" / "monitor_value_web.yaml"
        if cand.is_file():
            return cand
    except Exception:
        pass
    return src


def _static_root() -> Path:
    env = os.environ.get("MONITOR_VALUE_WEB_STATIC")
    if env:
        return Path(env)
    here = Path(__file__).resolve().parents[1] / "frontend" / "dist"
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("monitor_value_web")) / "frontend"
        if (share / "index.html").is_file():
            return share
    except Exception:
        pass
    return here


class MonitorValueWeb(Node):
    def __init__(self) -> None:
        super().__init__("monitor_value_web")
        cfg_path = _cfg_default_path()
        cfg = load_web_config(cfg_path)
        self._cfg = cfg
        self._state = DashboardState(cfg, cfg_path)
        self._last_imu_pub = 0.0
        self._http = start_http_server(
            self._state,
            _static_root(),
            cfg["http"]["host"],
            cfg["http"]["port"],
        )
        qos = qos_profile_sensor_data
        self.create_subscription(MonitorValue, cfg["topics"]["monitor"], self._on_monitor, qos)
        self.create_subscription(
            ImuNavSnapshot, cfg["topics"]["imu_snapshot"], self._on_imu, qos
        )
        if os.environ.get("MONITOR_VALUE_WEB_DEMO") == "1":
            self.create_timer(0.1, self._on_demo)
            self.get_logger().warn("DEMO mode: synthesizing monitor/attitude values")
        self.get_logger().info(f"config: {cfg_path}")
        self.get_logger().info(
            f"http://{cfg['http']['host']}:{cfg['http']['port']}/  "
            f"monitor={cfg['topics']['monitor']} imu={cfg['topics']['imu_snapshot']}"
        )

    def _on_monitor(self, msg: MonitorValue) -> None:
        self._state.set_monitor(monitor_to_dict(msg))

    def _on_imu(self, msg: ImuNavSnapshot) -> None:
        now = time.monotonic()
        min_dt = float(self._cfg["imu_min_interval_sec"])
        if now - self._last_imu_pub < min_dt:
            return
        self._last_imu_pub = now
        self._state.set_attitude(snapshot_to_attitude(msg))

    def _on_demo(self) -> None:
        t = time.monotonic()
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
        self._state.set_monitor(monitor)
        self._state.set_attitude(
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

    def destroy_node(self):
        try:
            self._http.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorValueWeb()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
