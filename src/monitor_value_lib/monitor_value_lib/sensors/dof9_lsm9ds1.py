from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from device_registry import DeviceRegistry

from monitor_value_lib.sensors.dof9_types import Dof9Sample


@dataclass
class Lsm9ds1Dof9:
    """LSM9DS1 reader (smbus2); I2C bus from ``device_i2c.yaml`` by default."""

    _sensor_cfg: Dict[str, Any] = field(default_factory=dict)
    _dev: Optional[object] = None

    def init(self) -> None:
        if self._dev is not None:
            return
        try:
            import smbus2 as smbus
            from monitor_value_lib.sensors.lsm9ds1_i2c import accelerometer
        except Exception as e:
            raise RuntimeError(f"LSM9DS1 driver not available: {e}") from e

        cfg = self._sensor_cfg
        linux_bus: Optional[int] = None
        if cfg.get("linux_bus") is not None:
            linux_bus = int(cfg["linux_bus"])
        elif bool(cfg.get("use_device_registry", True)):
            reg = DeviceRegistry.from_src_default()
            dev_id = str(cfg.get("device_id", "lsm9ds1_accel_gyro"))
            i2c = reg.get_i2c(dev_id)
            if i2c.linux_bus is not None:
                linux_bus = int(i2c.linux_bus)
        if linux_bus is not None:
            accelerometer.Accelerometer.bus = smbus.SMBus(int(linux_bus))  # type: ignore[attr-defined]

        self._dev = accelerometer.Accelerometer()

    def read(self) -> Dof9Sample:
        if self._dev is None:
            self.init()
        assert self._dev is not None

        gx, gy, gz = list(self._dev.readGyroData())
        ax, ay, az = list(self._dev.readAccData())
        mx, my, mz = list(self._dev.readMagData())

        d2r = math.pi / 180.0
        return Dof9Sample(
            gyro_x=float(gx) * d2r,
            gyro_y=float(gy) * d2r,
            gyro_z=float(gz) * d2r,
            accel_x=float(ax),
            accel_y=float(ay),
            accel_z=float(az),
            mag_x=float(mx),
            mag_y=float(my),
            mag_z=float(mz),
        )
