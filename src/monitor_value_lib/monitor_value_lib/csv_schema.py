from __future__ import annotations

import math
from dataclasses import asdict
from typing import Any, Iterable, List, Tuple

from monitor_value_lib.values import MonitorValues


CSV_COLUMNS: Tuple[str, ...] = (
    # timing
    "seq",
    "wall_time_jst",
    "elapsed_hms",
    # RPi
    "rpi_fan_rpm",
    "rpi_fan_pwm_percent",
    "rpi_cpu_temp_c",
    "rpi_cpu_util_percent",
    "rpi_gpu_util_percent",
    # 9DoF
    "mag_x",
    "mag_y",
    "mag_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "accel_x",
    "accel_y",
    "accel_z",
    # pose
    "pose_x",
    "pose_y",
    "pose_z",
    "ori_x",
    "ori_y",
    "ori_z",
    "ori_w",
    # depth
    "depth_m",
    "depth_temp_c",
    "depth_pressure_atm",
    # BME280
    "bme_temp_c",
    "bme_pressure_atm",
    "bme_humidity_percent",
    # power
    "current_a",
    "voltage_v",
    "power_w",
    "accumulated_energy_wh",
    "remaining_percent",
    "peak_power_w",
    # thermocouple
    "thermocouple_ch2_temp_c",
    "thermocouple_ch3_temp_c",
    # DS18B20
    "ds18b20_ch0_temp_c",
    "ds18b20_ch1_temp_c",
    # water leak
    "water_ch0_probe_v",
    "water_ch0_detected",
    "water_ch1_probe_v",
    "water_ch1_detected",
)


def _format_cell(v: Any) -> str:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        if math.isinf(v):
            return "inf" if v > 0 else "-inf"
        # Requested: show only 1 decimal place
        return f"{v:.1f}"
    return str(v)


def csv_header() -> List[str]:
    return list(CSV_COLUMNS)


def values_to_csv_row(values: MonitorValues) -> List[str]:
    d = asdict(values)
    d["wall_time_jst"] = values.wall_time_jst_str()
    d["elapsed_hms"] = values.elapsed_hms()
    return [_format_cell(d.get(k)) for k in CSV_COLUMNS]


def rows_to_csv_lines(rows: Iterable[List[str]]) -> List[str]:
    return [",".join(r) + "\n" for r in rows]
