from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

from device_registry import DeviceRegistry

from imu_navigation.types import Dof9Sample


@dataclass
class Lsm9ds1Dof9:
    """LSM9DS1 reader; I2C bus from ``device_id`` in registry or explicit ``linux_bus``."""

    _linux_bus: Optional[int] = None
    _dev: Optional[object] = None

    @classmethod
    def from_dof_config(cls, dof: Dict[str, Any]) -> "Lsm9ds1Dof9":
        linux_bus: Optional[int] = None
        lb = dof.get("linux_bus", None)
        if lb is not None:
            linux_bus = int(lb)
        elif bool(dof.get("use_device_registry", True)):
            reg = DeviceRegistry.from_src_default()
            dev_id = str(dof.get("device_id", "lsm9ds1_accel_gyro"))
            i2c = reg.get_i2c(dev_id)
            if i2c.linux_bus is not None:
                linux_bus = int(i2c.linux_bus)
        return cls(_linux_bus=linux_bus)

    def init(self) -> None:
        if self._dev is not None:
            return
        try:
            import smbus2 as smbus
            from monitor_value_lib.sensors.lsm9ds1_i2c import accelerometer
        except Exception as e:
            raise RuntimeError(f"LSM9DS1 driver not available: {e}") from e
        if self._linux_bus is not None:
            accelerometer.Accelerometer.bus = smbus.SMBus(int(self._linux_bus))  # type: ignore[attr-defined]
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
