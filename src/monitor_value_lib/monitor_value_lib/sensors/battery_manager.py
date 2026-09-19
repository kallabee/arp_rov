from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from monitor_value_lib.sensors.ina226_driver import INA226


def _nan() -> float:
    return math.nan


@dataclass
class BatteryManager:
    busnum: int
    address: int = 0x40
    shunt_ohms: float = 0.002
    max_expected_amps: float = 25.0
    capacity_wh: float | None = None

    _ina: Optional[INA226] = None
    _last_t: Optional[float] = None
    _last_power_w: Optional[float] = None
    _acc_wh: float = 0.0
    _peak_w: float = 0.0

    def init(self) -> None:
        if self._ina is not None:
            return
        self._ina = INA226(
            busnum=int(self.busnum),
            address=int(self.address),
            max_expected_amps=float(self.max_expected_amps),
            shunt_ohms=float(self.shunt_ohms),
        )
        self._ina.configure()

    def read(self) -> dict[str, float]:
        if self._ina is None:
            self.init()
        assert self._ina is not None

        try:
            voltage_v = float(self._ina.voltage())
        except Exception:
            voltage_v = _nan()
        try:
            current_a = float(self._ina.current())
        except Exception:
            current_a = _nan()
        try:
            power_w = float(self._ina.power())
        except Exception:
            power_w = voltage_v * current_a if not (math.isnan(voltage_v) or math.isnan(current_a)) else _nan()

        now = time.monotonic()
        if self._last_t is None:
            self._last_t = now
            self._last_power_w = power_w if not math.isnan(power_w) else None
        else:
            dt = max(0.0, now - self._last_t)
            p1 = power_w if not math.isnan(power_w) else None
            p0 = self._last_power_w
            if p0 is not None and p1 is not None:
                self._acc_wh += (dt / 3600.0) * 0.5 * (p0 + p1)
                self._peak_w = max(self._peak_w, p1)
            self._last_t = now
            self._last_power_w = p1

        remaining_percent = _nan()
        if self.capacity_wh is not None and self.capacity_wh > 0:
            remaining_percent = 100.0 * max(0.0, (self.capacity_wh - self._acc_wh) / self.capacity_wh)

        peak = self._peak_w if self._peak_w > 0 else _nan()

        return {
            "current_a": current_a,
            "voltage_v": voltage_v,
            "power_w": power_w,
            "accumulated_energy_wh": float(self._acc_wh),
            "remaining_percent": float(remaining_percent),
            "peak_power_w": float(peak),
        }

