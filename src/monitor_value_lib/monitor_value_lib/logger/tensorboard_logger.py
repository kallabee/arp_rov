from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from monitor_value_lib.logger.tb_event_writer import PureSummaryWriter
from monitor_value_lib.values import MonitorValues

# Tag groups for TensorBoard sidebar (prefix/name).
_SCALAR_TAGS: tuple[tuple[str, str], ...] = (
    ("rpi/fan_rpm", "rpi_fan_rpm"),
    ("rpi/cpu_temp_c", "rpi_cpu_temp_c"),
    ("rpi/cpu_util_percent", "rpi_cpu_util_percent"),
    ("rpi/gpu_util_percent", "rpi_gpu_util_percent"),
    ("imu/mag_x", "mag_x"),
    ("imu/mag_y", "mag_y"),
    ("imu/mag_z", "mag_z"),
    ("imu/gyro_x", "gyro_x"),
    ("imu/gyro_y", "gyro_y"),
    ("imu/gyro_z", "gyro_z"),
    ("imu/accel_x", "accel_x"),
    ("imu/accel_y", "accel_y"),
    ("imu/accel_z", "accel_z"),
    ("pose/x_m", "pose_x"),
    ("pose/y_m", "pose_y"),
    ("pose/z_m", "pose_z"),
    ("pose/ori_x", "ori_x"),
    ("pose/ori_y", "ori_y"),
    ("pose/ori_z", "ori_z"),
    ("pose/ori_w", "ori_w"),
    ("depth/depth_m", "depth_m"),
    ("depth/temp_c", "depth_temp_c"),
    ("depth/pressure_atm", "depth_pressure_atm"),
    ("bme/temp_c", "bme_temp_c"),
    ("bme/pressure_atm", "bme_pressure_atm"),
    ("bme/humidity_percent", "bme_humidity_percent"),
    ("power/current_a", "current_a"),
    ("power/voltage_v", "voltage_v"),
    ("power/power_w", "power_w"),
    ("power/energy_wh", "accumulated_energy_wh"),
    ("power/remaining_percent", "remaining_percent"),
    ("power/peak_power_w", "peak_power_w"),
    ("thermocouple/ch2_temp_c", "thermocouple_ch2_temp_c"),
    ("thermocouple/ch3_temp_c", "thermocouple_ch3_temp_c"),
    ("ds18b20/ch0_temp_c", "ds18b20_ch0_temp_c"),
    ("ds18b20/ch1_temp_c", "ds18b20_ch1_temp_c"),
    ("water/ch0_probe_v", "water_ch0_probe_v"),
    ("water/ch0_detected", "water_ch0_detected"),
    ("water/ch1_probe_v", "water_ch1_probe_v"),
    ("water/ch1_detected", "water_ch1_detected"),
    ("timing/elapsed_sec", "elapsed_since_start_sec"),
)


def _as_float(v: Any) -> Optional[float]:
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    if isinstance(v, (int, float)):
        f = float(v)
        if math.isnan(f) or math.isinf(f):
            return None
        return f
    return None


@dataclass
class TensorboardLogger:
    """Write MonitorValues as TensorBoard event files (offline + remote viewable)."""

    log_dir: Path
    flush_every_n: int = 1

    _writer: Any = field(default=None, init=False, repr=False)
    _run_dir: Optional[Path] = field(default=None, init=False)
    _n_since_flush: int = field(default=0, init=False)
    _disabled_reason: Optional[str] = field(default=None, init=False)
    # Always-monotonic X axis for TensorBoard (do not use message seq: multiple
    # publishers or restarts make seq non-monotonic → spiderweb plots).
    _step: int = field(default=0, init=False)

    def _ensure_open(self) -> bool:
        if self._writer is not None:
            return True
        if self._disabled_reason is not None:
            return False
        try:
            run = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            self._run_dir = Path(self.log_dir) / run
            self._run_dir.mkdir(parents=True, exist_ok=True)
            self._writer = PureSummaryWriter(log_dir=self._run_dir)
            self._step = 0
        except Exception as e:
            self._disabled_reason = str(e)
            return False
        return True

    @property
    def run_dir(self) -> Optional[Path]:
        return self._run_dir

    @property
    def disabled_reason(self) -> Optional[str]:
        return self._disabled_reason

    def append(self, values: MonitorValues) -> Optional[Path]:
        if not self._ensure_open():
            return None
        assert self._writer is not None
        d = asdict(values)
        step = int(self._step)
        self._step += 1
        wall_sec = values.wall_time_utc.timestamp() if values.wall_time_utc is not None else None
        # Record publisher seq as its own series (debug); never use it as global_step.
        msg_seq = _as_float(values.seq)
        if msg_seq is not None:
            if wall_sec is not None:
                self._writer.add_scalar("timing/msg_seq", msg_seq, global_step=step, walltime=wall_sec)
            else:
                self._writer.add_scalar("timing/msg_seq", msg_seq, global_step=step)
        for tag, key in _SCALAR_TAGS:
            f = _as_float(d.get(key))
            if f is None:
                continue
            if wall_sec is not None:
                self._writer.add_scalar(tag, f, global_step=step, walltime=wall_sec)
            else:
                self._writer.add_scalar(tag, f, global_step=step)

        self._n_since_flush += 1
        if self.flush_every_n > 0 and self._n_since_flush >= self.flush_every_n:
            self._writer.flush()
            self._n_since_flush = 0
        return self._run_dir

    def close(self) -> None:
        if self._writer is None:
            return
        try:
            self._writer.flush()
            self._writer.close()
        finally:
            self._writer = None
