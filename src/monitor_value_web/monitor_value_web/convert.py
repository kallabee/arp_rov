from __future__ import annotations

import math
from datetime import datetime, timedelta, timezone
from typing import Any, Optional

JST = timezone(timedelta(hours=9))

_MONITOR_FLOATS = (
    "elapsed_since_start_sec",
    "rpi_fan_rpm",
    "rpi_fan_pwm_percent",
    "rpi_cpu_temp_c",
    "rpi_cpu_util_percent",
    "rpi_gpu_util_percent",
    "mag_x",
    "mag_y",
    "mag_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "accel_x",
    "accel_y",
    "accel_z",
    "pose_x",
    "pose_y",
    "pose_z",
    "ori_x",
    "ori_y",
    "ori_z",
    "ori_w",
    "depth_m",
    "depth_temp_c",
    "depth_pressure_atm",
    "bme_temp_c",
    "bme_pressure_atm",
    "bme_humidity_percent",
    "current_a",
    "voltage_v",
    "power_w",
    "accumulated_energy_wh",
    "remaining_percent",
    "peak_power_w",
    "thermocouple_ch2_temp_c",
    "thermocouple_ch3_temp_c",
    "ds18b20_ch0_temp_c",
    "ds18b20_ch1_temp_c",
    "water_ch0_probe_v",
    "water_ch1_probe_v",
)


def json_float(v: Any) -> Optional[float]:
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    if math.isnan(x) or math.isinf(x):
        return None
    return x


def quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> dict[str, Optional[float]]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9 or math.isnan(n):
        return {"roll_deg": None, "pitch_deg": None, "yaw_deg": None}
    x, y, z, w = x / n, y / n, z / n, w / n
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return {
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
    }


def _stamp_iso(stamp) -> Optional[str]:
    try:
        sec = int(getattr(stamp, "sec", 0))
        nsec = int(getattr(stamp, "nanosec", 0))
    except (TypeError, ValueError):
        return None
    dt = datetime.fromtimestamp(sec + nsec * 1e-9, tz=timezone.utc)
    return dt.isoformat(timespec="milliseconds")


def _stamp_jst(stamp) -> Optional[str]:
    try:
        sec = int(getattr(stamp, "sec", 0))
        nsec = int(getattr(stamp, "nanosec", 0))
    except (TypeError, ValueError):
        return None
    dt = datetime.fromtimestamp(sec + nsec * 1e-9, tz=timezone.utc).astimezone(JST)
    return dt.strftime("%Y-%m-%d %H:%M:%S")


def elapsed_hms(elapsed_sec: Any) -> str:
    x = json_float(elapsed_sec)
    if x is None:
        return "--:--:--"
    total = int(max(0.0, x))
    h = total // 3600
    m = (total % 3600) // 60
    s = total % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def monitor_to_dict(msg) -> dict[str, Any]:
    out: dict[str, Any] = {
        "seq": int(getattr(msg, "seq", 0)),
        "stamp": _stamp_iso(getattr(msg, "stamp", None)),
        "stamp_jst": _stamp_jst(getattr(msg, "stamp", None)),
        "elapsed_hms": elapsed_hms(getattr(msg, "elapsed_since_start_sec", None)),
        "water_ch0_detected": bool(getattr(msg, "water_ch0_detected", False)),
        "water_ch1_detected": bool(getattr(msg, "water_ch1_detected", False)),
    }
    for name in _MONITOR_FLOATS:
        out[name] = json_float(getattr(msg, name, None))
    rpy = quat_to_rpy_deg(
        out.get("ori_x") or 0.0,
        out.get("ori_y") or 0.0,
        out.get("ori_z") or 0.0,
        out.get("ori_w") or 0.0,
    )
    if all(out.get(k) is None for k in ("ori_x", "ori_y", "ori_z", "ori_w")):
        out.update({"roll_deg": None, "pitch_deg": None, "yaw_deg": None})
    else:
        out.update(rpy)
    return out


def snapshot_to_attitude(msg) -> dict[str, Any]:
    ox = json_float(getattr(msg, "ori_x", None))
    oy = json_float(getattr(msg, "ori_y", None))
    oz = json_float(getattr(msg, "ori_z", None))
    ow = json_float(getattr(msg, "ori_w", None))
    out: dict[str, Any] = {
        "seq": int(getattr(msg, "seq", 0)),
        "stamp": _stamp_iso(getattr(msg, "stamp", None)),
        "ori_x": ox,
        "ori_y": oy,
        "ori_z": oz,
        "ori_w": ow,
    }
    if None in (ox, oy, oz, ow):
        out.update({"roll_deg": None, "pitch_deg": None, "yaw_deg": None})
    else:
        out.update(quat_to_rpy_deg(ox, oy, oz, ow))
    return out
