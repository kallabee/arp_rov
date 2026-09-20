from __future__ import annotations

import math
import os
import queue
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hand_actuator_interfaces.msg import HandActuator
from light_actuator_interfaces.msg import LightActuator
from monitor_value_interfaces.msg import ImuNavSnapshot, MonitorValue
from monitor_value_web.command import (
    hand_from_msg,
    hand_to_msg,
    lights_from_msg,
    lights_to_msg,
    twist_from_msg,
    twist_to_msg,
)
from monitor_value_web.config import load_web_config
from monitor_value_web.convert import monitor_to_dict, snapshot_to_attitude
from monitor_value_web.http_server import DashboardState, start_http_server
from monitor_value_web.light_pwm import open_light_pwm_driver


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


def _dist_has_assets(root: Path) -> bool:
    index = root / "index.html"
    if not index.is_file():
        return False
    text = index.read_text(encoding="utf-8")
    for rel in (
        part.split('src="', 1)[-1].split('"', 1)[0]
        if 'src="' in part
        else part.split('href="', 1)[-1].split('"', 1)[0]
        for part in text.split()
        if 'src="./assets/' in part or 'href="./assets/' in part
    ):
        asset = (root / rel.lstrip("./")).resolve()
        if not asset.is_file():
            return False
    return True


def _static_root() -> Path:
    env = os.environ.get("MONITOR_VALUE_WEB_STATIC")
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    candidates: list[Path] = []
    for base in (here, *here.parents):
        candidates.append(base / "src" / "monitor_value_web" / "frontend" / "dist")
        candidates.append(base / "frontend" / "dist")
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(Path(get_package_share_directory("monitor_value_web")) / "frontend")
    except Exception:
        pass
    seen: set[Path] = set()
    for cand in candidates:
        try:
            cand = cand.resolve()
        except OSError:
            continue
        if cand in seen:
            continue
        seen.add(cand)
        if _dist_has_assets(cand):
            return cand
    fallback = here.parents[1] / "frontend" / "dist"
    return fallback


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
        self._twist_pub = self.create_publisher(Twist, cfg["topics"]["cmd_vel"], 10)
        self._hand_pub = self.create_publisher(HandActuator, cfg["topics"]["hand_act"], 10)
        self._light_pub = self.create_publisher(LightActuator, cfg["topics"]["lights"], 10)
        self.create_subscription(Twist, cfg["topics"]["cmd_vel"], self._on_twist, 10)
        self.create_subscription(HandActuator, cfg["topics"]["hand_act"], self._on_hand, 10)
        self.create_subscription(LightActuator, cfg["topics"]["lights"], self._on_lights, 10)
        self._pub_q: queue.Queue = queue.Queue()
        self.create_timer(0.02, self._drain_pub)
        self._state.publish_command = self._enqueue_command
        self._light_pwm = None
        try:
            self._light_pwm = open_light_pwm_driver()
            self._light_pwm.all_off()
            self.get_logger().info(
                f"Light PWM ready ({len(self._light_pwm.channels)} channels)"
            )
        except Exception as exc:
            self.get_logger().warn(f"Light PWM disabled (ROS publish only): {exc}")
        if os.environ.get("MONITOR_VALUE_WEB_DEMO") == "1":
            self.create_timer(0.1, self._on_demo)
            self.get_logger().warn("DEMO mode: synthesizing monitor/attitude values")
        self.get_logger().info(f"config: {cfg_path}")
        self.get_logger().info(
            f"http://{cfg['http']['host']}:{cfg['http']['port']}/  "
            f"monitor={cfg['topics']['monitor']} imu={cfg['topics']['imu_snapshot']} "
            f"cmd_vel={cfg['topics']['cmd_vel']} hand={cfg['topics']['hand_act']} "
            f"lights={cfg['topics']['lights']}"
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

    def _enqueue_command(self, groups: dict) -> None:
        self._pub_q.put(groups)

    def _drain_pub(self) -> None:
        names = list(self._cfg.get("lights") or [])
        try:
            while True:
                groups = self._pub_q.get_nowait()
                if "twist" in groups:
                    self._twist_pub.publish(twist_to_msg(groups["twist"], Twist))
                if "hand" in groups:
                    self._hand_pub.publish(hand_to_msg(groups["hand"], HandActuator))
                if "lights" in groups:
                    self._light_pub.publish(lights_to_msg(groups["lights"], names, LightActuator))
                    self._apply_lights_hw(groups["lights"])
        except queue.Empty:
            return

    def _apply_lights_hw(self, duties: dict) -> None:
        if self._light_pwm is None:
            return
        try:
            self._light_pwm.apply_map(duties)
        except Exception as exc:
            self.get_logger().warn(f"Light PWM apply failed: {exc}")

    def _on_twist(self, msg: Twist) -> None:
        self._state.apply_remote_command("twist", twist_from_msg(msg))

    def _on_hand(self, msg: HandActuator) -> None:
        self._state.apply_remote_command("hand", hand_from_msg(msg))

    def _on_lights(self, msg: LightActuator) -> None:
        names = list(self._cfg.get("lights") or [])
        data = lights_from_msg(msg, names)
        if self._state.apply_remote_command("lights", data):
            # Other PC / node command — drive hardware here so lights work
            # even when thruster_controller is not running.
            self._apply_lights_hw(data)

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
            self._state.stop()
        except Exception:
            pass
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
