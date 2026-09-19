from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone


def _nan() -> float:
    return math.nan


JST = timezone(timedelta(hours=9))


@dataclass(slots=True)
class MonitorValues:
    # Timing
    seq: int = 0
    wall_time_utc: datetime = datetime.now(timezone.utc)
    elapsed_since_start_sec: float = 0.0

    # RPi
    rpi_fan_rpm: float = _nan()
    rpi_cpu_temp_c: float = _nan()
    rpi_cpu_util_percent: float = _nan()
    rpi_gpu_util_percent: float = _nan()

    # 9DoF
    mag_x: float = _nan()
    mag_y: float = _nan()
    mag_z: float = _nan()
    gyro_x: float = _nan()
    gyro_y: float = _nan()
    gyro_z: float = _nan()
    accel_x: float = _nan()
    accel_y: float = _nan()
    accel_z: float = _nan()

    pose_x: float = _nan()
    pose_y: float = _nan()
    pose_z: float = _nan()
    ori_x: float = _nan()
    ori_y: float = _nan()
    ori_z: float = _nan()
    ori_w: float = _nan()

    # Depth sensor (MS5837)
    depth_m: float = _nan()
    depth_temp_c: float = _nan()
    depth_pressure_atm: float = _nan()

    # BME280
    bme_temp_c: float = _nan()
    bme_pressure_atm: float = _nan()
    bme_humidity_percent: float = _nan()

    # Current monitor (INA226 / ADS1115 future)
    current_a: float = _nan()
    voltage_v: float = _nan()
    power_w: float = _nan()
    accumulated_energy_wh: float = _nan()
    remaining_percent: float = _nan()
    peak_power_w: float = _nan()

    # Thermocouple (AD8495 on ADS1015)
    thermocouple_ch2_temp_c: float = _nan()
    thermocouple_ch3_temp_c: float = _nan()

    # DS18B20 1-Wire
    ds18b20_ch0_temp_c: float = _nan()
    ds18b20_ch1_temp_c: float = _nan()

    # Water leak (ADS1015)
    water_ch0_probe_v: float = _nan()
    water_ch0_detected: bool = False
    water_ch1_probe_v: float = _nan()
    water_ch1_detected: bool = False

    def wall_time_iso8601(self) -> str:
        return self.wall_time_utc.astimezone(timezone.utc).isoformat(timespec="milliseconds")

    def wall_time_jst_str(self) -> str:
        # YYYY-MM-dd hh:mm:ss (JST)
        return self.wall_time_utc.astimezone(JST).strftime("%Y-%m-%d %H:%M:%S")

    def elapsed_hms(self) -> str:
        # hh:mm:ss
        total = int(max(0.0, float(self.elapsed_since_start_sec)))
        h = total // 3600
        m = (total % 3600) // 60
        s = total % 60
        return f"{h:02d}:{m:02d}:{s:02d}"
