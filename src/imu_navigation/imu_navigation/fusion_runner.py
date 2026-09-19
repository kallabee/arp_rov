from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml

from device_registry import DeviceRegistry
from monitor_value_lib.config import SensorMode, normalize_sensor_entry

from imu_navigation.extrinsics import mag_in_imu_frame
from imu_navigation.filters.factory import make_orientation_filter
from imu_navigation.types import Dof9Sample, ImuNavSnapshotData

# When a scheduled depth read returns NaN:
# - ``nine_axis_continue``: run AHRS + dead reckoning; skip depth z blend only (default).
# - ``skip_fusion_step``: do not advance orientation / velocity / position this tick; ``_last_t`` is not advanced.
DEPTH_INVALID_NINE_AXIS = "nine_axis_continue"
DEPTH_INVALID_SKIP_STEP = "skip_fusion_step"


def _quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Body to world rotation (column vectors): v_w = R @ v_b."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * qw * qx, s * qw * qy, s * qw * qz
    xx, xy, xz = s * qx * qx, s * qx * qy, s * qx * qz
    yy, yz, zz = s * qy * qy, s * qy * qz, s * qz * qz
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


class _DepthReader:
    def __init__(self, bus: int, model_name: str, i2c_addr: Optional[int]) -> None:
        from py_pub_depth import ms5837  # type: ignore

        model = ms5837.MODEL_02BA if str(model_name).upper() in ("02BA", "MS5837_02BA") else ms5837.MODEL_30BA
        self._sensor = ms5837.MS5837(model=model, bus=int(bus), i2c_addr=i2c_addr)
        if not self._sensor.init():
            raise RuntimeError("MS5837 init failed")

    def read(self) -> tuple[float, float, float]:
        if not self._sensor.read():
            return float("nan"), float("nan"), float("nan")
        from py_pub_depth import ms5837  # type: ignore

        return (
            float(self._sensor.depth()),
            float(self._sensor.temperature()),
            float(self._sensor.pressure(ms5837.UNITS_atm)),
        )


class FusionRunner:
    """High-rate IMU + optional depth fusion (no ROS required)."""

    @property
    def fusion_period_sec(self) -> float:
        return 1.0 / max(self._fusion_hz, 1.0)

    @property
    def config(self) -> Dict[str, Any]:
        return self._cfg

    def __init__(self, cfg: Dict[str, Any]):
        self._cfg = cfg
        self._lock = threading.Lock()
        self._latest = ImuNavSnapshotData()
        self._seq = 0
        self._last_t: Optional[float] = None
        self._dof9 = None
        self._depth: Optional[_DepthReader] = None
        self.init_errors: list[str] = []
        self._depth_counter = 0
        self._depth_interval = 1
        self._filter = make_orientation_filter(str(cfg.get("filter", "complementary")), cfg)
        self._extrinsics: Dict[str, Any] = cfg.get("extrinsics") or {}
        self._fusion_hz = float(cfg.get("fusion_hz", 50.0))
        self._depth_hz = float(cfg.get("depth_hz", 5.0))
        self._depth_blend = float(cfg.get("depth_z_blend", 0.05))
        self._use_mag = bool(cfg.get("use_mag", True))
        raw_depth_mode = str(cfg.get("on_depth_invalid", DEPTH_INVALID_NINE_AXIS)).lower().strip()
        if raw_depth_mode in ("skip", "skip_step", "skip_fusion", "skip_fusion_step"):
            self._on_depth_invalid = DEPTH_INVALID_SKIP_STEP
        else:
            self._on_depth_invalid = DEPTH_INVALID_NINE_AXIS
        self._vel = np.zeros(3)
        self._pos = np.zeros(3)
        self._g_w = np.array([0.0, 0.0, 9.81], dtype=float)
        gb = cfg.get("gyro_bias_rad_s")
        if gb is None:
            self._gyro_bias = np.zeros(3, dtype=float)
        else:
            gbl = list(gb)
            self._gyro_bias = np.array(
                [
                    float(gbl[0]),
                    float(gbl[1]) if len(gbl) > 1 else 0.0,
                    float(gbl[2]) if len(gbl) > 2 else 0.0,
                ],
                dtype=float,
            )

        sensors = cfg.get("sensors")
        if not isinstance(sensors, dict):
            sensors = {}
            cfg["sensors"] = sensors
        for _k in ("dof9", "depth"):
            if _k in sensors:
                sensors[_k] = normalize_sensor_entry(sensors[_k] or {})

        dof = sensors.get("dof9") or {}
        dof_mode = str(dof.get("mode", SensorMode.DISABLED))
        if dof_mode != SensorMode.DISABLED:
            typ = str(dof.get("type", "lsm9ds1")).lower()
            if typ != "lsm9ds1":
                raise ValueError(f"dof9.type unsupported: {typ}")
            from imu_navigation.sensors.lsm9ds1_dof9 import Lsm9ds1Dof9

            try:
                self._dof9 = Lsm9ds1Dof9.from_dof_config(dof if isinstance(dof, dict) else {})
                self._dof9.init()
            except Exception as e:
                self.init_errors.append(f"dof9: {e}")
                if dof_mode == SensorMode.ENABLED:
                    raise
                self._dof9 = None

        dep = sensors.get("depth") or {}
        dep_mode = str(dep.get("mode", SensorMode.DISABLED))
        if dep_mode != SensorMode.DISABLED:
            try:
                reg = DeviceRegistry.from_src_default()
                dev_id = str(dep.get("device_id", "depth_sensor"))
                i2c = reg.get_i2c(dev_id)
                if i2c.linux_bus is None:
                    raise ValueError(f"device {dev_id} requires bus in device_i2c.yaml")
                model = str(dep.get("model", "02BA"))
                addr = dep.get("i2c_addr", None)
                addr_i = int(addr) if addr is not None else None
                self._depth = _DepthReader(int(i2c.linux_bus), model, addr_i or int(i2c.addr))
            except Exception as e:
                self.init_errors.append(f"depth: {e}")
                if dep_mode == SensorMode.ENABLED:
                    raise
                self._depth = None

        if self._fusion_hz > 0 and self._depth_hz > 0:
            self._depth_interval = max(1, int(round(self._fusion_hz / self._depth_hz)))

    @classmethod
    def from_yaml(cls, path: str | Path) -> "FusionRunner":
        p = Path(path)
        with p.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError("imu_navigation yaml root must be a mapping")
        return cls(raw)

    def get_latest(self) -> ImuNavSnapshotData:
        with self._lock:
            return replace(self._latest)

    def step(self) -> ImuNavSnapshotData:
        """Single fusion update (blocking). Intended for tests or manual stepping."""
        now = time.monotonic()
        old_last_t = self._last_t
        if old_last_t is None:
            dt = 1.0 / max(self._fusion_hz, 1.0)
        else:
            dt = max(1e-4, float(now - old_last_t))
        self._last_t = now

        if self._dof9 is None:
            self._last_t = now
            with self._lock:
                return replace(self._latest)

        raw = self._dof9.read()
        imu_wall_t = time.time()
        if self._use_mag:
            m = np.array([raw.mag_x, raw.mag_y, raw.mag_z], dtype=float)
            m2 = mag_in_imu_frame(m, self._extrinsics)
        else:
            m2 = np.zeros(3, dtype=float)
        sample = Dof9Sample(
            gyro_x=raw.gyro_x - float(self._gyro_bias[0]),
            gyro_y=raw.gyro_y - float(self._gyro_bias[1]),
            gyro_z=raw.gyro_z - float(self._gyro_bias[2]),
            accel_x=raw.accel_x,
            accel_y=raw.accel_y,
            accel_z=raw.accel_z,
            mag_x=float(m2[0]),
            mag_y=float(m2[1]),
            mag_z=float(m2[2]),
        )

        depth_m = float("nan")
        depth_t = float("nan")
        depth_pressure_atm = float("nan")
        depth_wall_t = float("nan")
        read_depth_now = False
        if self._depth is not None:
            self._depth_counter += 1
            if self._depth_counter >= self._depth_interval:
                self._depth_counter = 0
                read_depth_now = True
                depth_m, depth_t, depth_pressure_atm = self._depth.read()
                depth_wall_t = time.time() if not math.isnan(depth_m) else float("nan")

        skip_fusion = (
            read_depth_now
            and math.isnan(depth_m)
            and self._on_depth_invalid == DEPTH_INVALID_SKIP_STEP
        )
        if skip_fusion:
            self._last_t = old_last_t
            with self._lock:
                return replace(self._latest)

        ox, oy, oz, ow = self._filter.update(sample, dt)
        R = _quat_to_R(ox, oy, oz, ow)
        a_b = np.array([sample.accel_x, sample.accel_y, sample.accel_z], dtype=float)
        a_w = R @ a_b - self._g_w
        self._vel += a_w * dt
        self._pos += self._vel * dt

        if not math.isnan(depth_m):
            # Treat ``depth_m`` as positive downward; map into a crude world z (z up) estimate.
            target_z = -float(depth_m)
            self._pos[2] = (1.0 - self._depth_blend) * self._pos[2] + self._depth_blend * target_z

        self._seq += 1
        snap = ImuNavSnapshotData(
            seq=int(self._seq),
            stamp_sec=float(imu_wall_t),
            depth_sample_time_sec=float(depth_wall_t),
            gyro_x=sample.gyro_x,
            gyro_y=sample.gyro_y,
            gyro_z=sample.gyro_z,
            accel_x=sample.accel_x,
            accel_y=sample.accel_y,
            accel_z=sample.accel_z,
            mag_x=sample.mag_x,
            mag_y=sample.mag_y,
            mag_z=sample.mag_z,
            ori_x=float(ox),
            ori_y=float(oy),
            ori_z=float(oz),
            ori_w=float(ow),
            pose_x=float(self._pos[0]),
            pose_y=float(self._pos[1]),
            pose_z=float(self._pos[2]),
            vel_x=float(self._vel[0]),
            vel_y=float(self._vel[1]),
            vel_z=float(self._vel[2]),
            depth_m=float(depth_m),
            depth_temp_c=float(depth_t),
            depth_pressure_atm=float(depth_pressure_atm),
        )
        with self._lock:
            self._latest = snap
        return snap

    def run_forever(self, stop: threading.Event) -> None:
        period = 1.0 / max(self._fusion_hz, 1.0)
        while not stop.is_set():
            t0 = time.monotonic()
            try:
                self.step()
            except Exception:
                pass
            elapsed = time.monotonic() - t0
            sleep_t = max(0.0, period - elapsed)
            if sleep_t > 0:
                stop.wait(timeout=sleep_t)
