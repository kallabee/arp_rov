from __future__ import annotations

import math
import os
import threading
from dataclasses import replace
from pathlib import Path
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from monitor_value_interfaces.msg import ImuNavSnapshot, MonitorValue
from monitor_value_lib import MonitorValueCollector, load_config


def _cfg_default_path() -> Path:
    env = os.environ.get("MONITOR_VALUE_CONFIG")
    if env:
        return Path(env)
    return Path("src/monitor_value_pub/config/monitor_value.yaml")


def _snapshot_like_to_dict(s: Any) -> dict[str, float]:
    """Read fields from ``ImuNavSnapshot`` or ``ImuNavSnapshotData``."""
    return {
        "mag_x": float(s.mag_x),
        "mag_y": float(s.mag_y),
        "mag_z": float(s.mag_z),
        "gyro_x": float(s.gyro_x),
        "gyro_y": float(s.gyro_y),
        "gyro_z": float(s.gyro_z),
        "accel_x": float(s.accel_x),
        "accel_y": float(s.accel_y),
        "accel_z": float(s.accel_z),
        "pose_x": float(s.pose_x),
        "pose_y": float(s.pose_y),
        "pose_z": float(s.pose_z),
        "ori_x": float(s.ori_x),
        "ori_y": float(s.ori_y),
        "ori_z": float(s.ori_z),
        "ori_w": float(s.ori_w),
        "depth_m": float(s.depth_m),
        "depth_temp_c": float(s.depth_temp_c),
        "depth_pressure_atm": float(s.depth_pressure_atm),
    }


def _values_to_msg(node: Node, v) -> MonitorValue:
    m = MonitorValue()
    m.seq = int(v.seq)
    m.stamp = node.get_clock().now().to_msg()
    m.elapsed_since_start_sec = float(v.elapsed_since_start_sec)

    m.rpi_fan_rpm = float(v.rpi_fan_rpm)
    m.rpi_cpu_temp_c = float(v.rpi_cpu_temp_c)
    m.rpi_cpu_util_percent = float(v.rpi_cpu_util_percent)
    m.rpi_gpu_util_percent = float(v.rpi_gpu_util_percent)

    m.mag_x = float(v.mag_x)
    m.mag_y = float(v.mag_y)
    m.mag_z = float(v.mag_z)
    m.gyro_x = float(v.gyro_x)
    m.gyro_y = float(v.gyro_y)
    m.gyro_z = float(v.gyro_z)
    m.accel_x = float(v.accel_x)
    m.accel_y = float(v.accel_y)
    m.accel_z = float(v.accel_z)

    m.pose_x = float(v.pose_x)
    m.pose_y = float(v.pose_y)
    m.pose_z = float(v.pose_z)
    m.ori_x = float(v.ori_x)
    m.ori_y = float(v.ori_y)
    m.ori_z = float(v.ori_z)
    m.ori_w = float(v.ori_w)

    m.depth_m = float(v.depth_m)
    m.depth_temp_c = float(v.depth_temp_c)
    m.depth_pressure_atm = float(v.depth_pressure_atm)

    m.bme_temp_c = float(v.bme_temp_c)
    m.bme_pressure_atm = float(v.bme_pressure_atm)
    m.bme_humidity_percent = float(v.bme_humidity_percent)

    m.current_a = float(v.current_a)
    m.voltage_v = float(v.voltage_v)
    m.power_w = float(v.power_w)
    m.accumulated_energy_wh = float(v.accumulated_energy_wh)
    m.remaining_percent = float(v.remaining_percent)
    m.peak_power_w = float(v.peak_power_w)

    m.thermocouple_ch2_temp_c = float(v.thermocouple_ch2_temp_c)
    m.thermocouple_ch3_temp_c = float(v.thermocouple_ch3_temp_c)
    m.ds18b20_ch0_temp_c = float(v.ds18b20_ch0_temp_c)
    m.ds18b20_ch1_temp_c = float(v.ds18b20_ch1_temp_c)
    m.water_ch0_probe_v = float(v.water_ch0_probe_v)
    m.water_ch0_detected = bool(v.water_ch0_detected)
    m.water_ch1_probe_v = float(v.water_ch1_probe_v)
    m.water_ch1_detected = bool(v.water_ch1_detected)
    return m


class MonitorValuePublisher(Node):
    def __init__(self, cfg_path: Optional[Path] = None):
        super().__init__("monitor_value_pub")

        cfg_path = cfg_path or _cfg_default_path()
        cfg = load_config(cfg_path)
        self._collector = MonitorValueCollector(cfg)
        col = self._collector
        self.get_logger().info(
            "collector sensors: "
            f"rpi={'up' if col._rpi is not None else 'down'} "
            f"bme280={'up' if col._bme is not None else 'down'} "
            f"ina226={'up' if col._battery is not None else 'down'} "
            f"thermocouple={'up' if col._thermocouple is not None else 'down'} "
            f"water_leak={'up' if col._water_leak is not None else 'down'} "
            f"ds18b20={'up' if col._ds18b20 is not None else 'down'}"
        )
        self._cfg = cfg
        self._imu_frame_id = str(getattr(cfg, "imu_frame_id", "imu_link"))

        self._pub = self.create_publisher(MonitorValue, "rov/monitor_value", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self._mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)
        self._timer = self.create_timer(float(cfg.sample_interval_seconds), self._on_timer)

        self._imu_lock = threading.Lock()
        self._imu_msg: Optional[ImuNavSnapshot] = None
        self._fusion_runner = None
        self._fusion_stop: Optional[threading.Event] = None
        self._fusion_thread: Optional[threading.Thread] = None

        imu_cfg = getattr(cfg, "imu_nav", {}) or {}
        self._embed = bool(imu_cfg.get("embed_runner", False))
        topic = str(imu_cfg.get("snapshot_topic", "rov/imu_nav/snapshot")).strip()
        if not self._embed and topic:
            self.create_subscription(ImuNavSnapshot, topic, self._on_imu_snapshot, 10)
            self.get_logger().info(f"imu_nav: subscribe {topic} (latest snapshot merged into monitor stream)")
        if self._embed:
            from ament_index_python.packages import get_package_share_directory

            from imu_navigation.fusion_runner import FusionRunner

            rc = str(imu_cfg.get("runner_config_path", "")).strip()
            if not rc:
                rc = str(Path(get_package_share_directory("imu_navigation")) / "config" / "imu_navigation.yaml")
            self._fusion_runner = FusionRunner.from_yaml(rc)
            self._fusion_stop = threading.Event()
            self._fusion_thread = threading.Thread(
                target=self._fusion_runner.run_forever,
                args=(self._fusion_stop,),
                daemon=True,
            )
            self._fusion_thread.start()
            self.get_logger().info(f"imu_nav: embedded FusionRunner ({rc})")

        self.get_logger().info(f"config: {cfg_path}")
        self.get_logger().info(f"log_dir: {cfg.log_dir} rotate_sec: {cfg.log_rotate_seconds}")
        self.get_logger().info(f"topic: rov/monitor_value interval_sec: {cfg.sample_interval_seconds}")
        self.get_logger().info(f"imu topics: /imu/data_raw /imu/mag frame_id: {self._imu_frame_id}")

    def _on_imu_snapshot(self, msg: ImuNavSnapshot) -> None:
        with self._imu_lock:
            self._imu_msg = msg

    def _merge_imu_into(self, v):
        snap = None
        if self._embed and self._fusion_runner is not None:
            snap = self._fusion_runner.get_latest()
        else:
            with self._imu_lock:
                if self._imu_msg is not None:
                    snap = self._imu_msg
        if snap is None:
            return v
        return replace(v, **_snapshot_like_to_dict(snap))

    def _on_timer(self) -> None:
        v, path = self._collector.get_values_and_append()
        v = self._merge_imu_into(v)
        msg = _values_to_msg(self, v)
        self._pub.publish(msg)

        if not all(
            math.isnan(x) for x in (v.gyro_x, v.gyro_y, v.gyro_z, v.accel_x, v.accel_y, v.accel_z)
        ):
            now = self.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = self._imu_frame_id
            imu.angular_velocity.x = float(v.gyro_x) if not math.isnan(v.gyro_x) else 0.0
            imu.angular_velocity.y = float(v.gyro_y) if not math.isnan(v.gyro_y) else 0.0
            imu.angular_velocity.z = float(v.gyro_z) if not math.isnan(v.gyro_z) else 0.0
            imu.linear_acceleration.x = float(v.accel_x) if not math.isnan(v.accel_x) else 0.0
            imu.linear_acceleration.y = float(v.accel_y) if not math.isnan(v.accel_y) else 0.0
            imu.linear_acceleration.z = float(v.accel_z) if not math.isnan(v.accel_z) else 0.0
            self._imu_pub.publish(imu)

        if not all(math.isnan(x) for x in (v.mag_x, v.mag_y, v.mag_z)):
            now = self.get_clock().now().to_msg()
            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = self._imu_frame_id
            mag.magnetic_field.x = float(v.mag_x) if not math.isnan(v.mag_x) else 0.0
            mag.magnetic_field.y = float(v.mag_y) if not math.isnan(v.mag_y) else 0.0
            mag.magnetic_field.z = float(v.mag_z) if not math.isnan(v.mag_z) else 0.0
            self._mag_pub.publish(mag)

    def destroy_node(self):
        if self._fusion_stop is not None:
            self._fusion_stop.set()
        if self._fusion_thread is not None:
            self._fusion_thread.join(timeout=3.0)
        try:
            self._collector.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorValuePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
