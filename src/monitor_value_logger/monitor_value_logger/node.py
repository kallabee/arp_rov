from __future__ import annotations

import os
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node

from monitor_value_interfaces.msg import MonitorValue
from monitor_value_lib.config import load_config
from monitor_value_lib.logger.csv_rotating_logger import RotatingCsvLogger
from monitor_value_lib.values import MonitorValues


def _cfg_default_path() -> Path:
    env = os.environ.get("MONITOR_VALUE_LOGGER_CONFIG")
    if env:
        return Path(env)
    return Path("src/monitor_value_logger/config/monitor_value_logger.yaml")


def _stamp_to_datetime_utc(stamp) -> datetime:
    sec = int(getattr(stamp, "sec", 0))
    nsec = int(getattr(stamp, "nanosec", 0))
    return datetime.fromtimestamp(sec + nsec * 1e-9, tz=timezone.utc)


def _msg_to_values(msg: MonitorValue) -> MonitorValues:
    v = MonitorValues(
        seq=int(msg.seq),
        wall_time_utc=_stamp_to_datetime_utc(msg.stamp),
        elapsed_since_start_sec=float(msg.elapsed_since_start_sec),
    )
    # RPi
    v = replace(
        v,
        rpi_fan_rpm=float(msg.rpi_fan_rpm),
        rpi_cpu_temp_c=float(msg.rpi_cpu_temp_c),
        rpi_cpu_util_percent=float(msg.rpi_cpu_util_percent),
        rpi_gpu_util_percent=float(msg.rpi_gpu_util_percent),
        # 9DoF
        mag_x=float(msg.mag_x),
        mag_y=float(msg.mag_y),
        mag_z=float(msg.mag_z),
        gyro_x=float(msg.gyro_x),
        gyro_y=float(msg.gyro_y),
        gyro_z=float(msg.gyro_z),
        accel_x=float(msg.accel_x),
        accel_y=float(msg.accel_y),
        accel_z=float(msg.accel_z),
        # pose
        pose_x=float(msg.pose_x),
        pose_y=float(msg.pose_y),
        pose_z=float(msg.pose_z),
        ori_x=float(msg.ori_x),
        ori_y=float(msg.ori_y),
        ori_z=float(msg.ori_z),
        ori_w=float(msg.ori_w),
        # depth
        depth_m=float(msg.depth_m),
        depth_temp_c=float(msg.depth_temp_c),
        depth_pressure_atm=float(msg.depth_pressure_atm),
        # bme
        bme_temp_c=float(msg.bme_temp_c),
        bme_pressure_atm=float(msg.bme_pressure_atm),
        bme_humidity_percent=float(msg.bme_humidity_percent),
        # power
        current_a=float(msg.current_a),
        voltage_v=float(msg.voltage_v),
        power_w=float(msg.power_w),
        accumulated_energy_wh=float(msg.accumulated_energy_wh),
        remaining_percent=float(msg.remaining_percent),
        peak_power_w=float(msg.peak_power_w),
        thermocouple_ch2_temp_c=float(msg.thermocouple_ch2_temp_c),
        thermocouple_ch3_temp_c=float(msg.thermocouple_ch3_temp_c),
        ds18b20_ch0_temp_c=float(msg.ds18b20_ch0_temp_c),
        ds18b20_ch1_temp_c=float(msg.ds18b20_ch1_temp_c),
        water_ch0_probe_v=float(msg.water_ch0_probe_v),
        water_ch0_detected=bool(msg.water_ch0_detected),
        water_ch1_probe_v=float(msg.water_ch1_probe_v),
        water_ch1_detected=bool(msg.water_ch1_detected),
    )
    return v


class MonitorValueLogger(Node):
    def __init__(self, cfg_path: Path | None = None):
        super().__init__("monitor_value_logger")

        cfg_path = cfg_path or _cfg_default_path()
        cfg = load_config(cfg_path)

        self._csv_logger = RotatingCsvLogger(
            log_dir=Path(cfg.log_dir),
            rotate_seconds=float(cfg.log_rotate_seconds),
            flush_each_row=True,
        )
        self._sub = self.create_subscription(MonitorValue, "rov/monitor_value", self._on_msg, 10)
        self.get_logger().info(f"config: {cfg_path}")
        self.get_logger().info(f"log_dir: {cfg.log_dir} rotate_sec: {cfg.log_rotate_seconds}")

    def _on_msg(self, msg: MonitorValue) -> None:
        v = _msg_to_values(msg)
        self._csv_logger.append(v)

    def destroy_node(self):
        try:
            self._csv_logger.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorValueLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

