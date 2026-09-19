from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional


def _nan() -> float:
    return math.nan


ATM_PER_HPA = 0.000986923  # 1 hPa = 0.000986923 atm


@dataclass
class Bme280Sensor:
    address: int = 0x77
    linux_bus: int | None = None
    use_extended_bus: bool = True
    bitbang_enabled: bool = False
    bitbang_scl_pin: str = "D23"
    bitbang_sda_pin: str = "D22"

    _sensor: Optional[Any] = None

    def init(self) -> None:
        if self._sensor is not None:
            return
        try:
            import busio  # type: ignore
            import board  # type: ignore
            from adafruit_bme280 import basic as adafruit_bme280  # type: ignore

            if self.bitbang_enabled:
                # Follow existing pattern in inner_hull_status_pub.
                from adafruit_blinka.microcontroller.bcm2711 import pin as bcm_pin  # type: ignore

                scl = getattr(bcm_pin, self.bitbang_scl_pin)
                sda = getattr(bcm_pin, self.bitbang_sda_pin)
                i2c = busio.I2C(scl, sda)
            elif self.use_extended_bus and self.linux_bus is not None:
                from adafruit_extended_bus import ExtendedI2C  # type: ignore

                i2c = ExtendedI2C(int(self.linux_bus))
            else:
                i2c = busio.I2C(board.SCL, board.SDA)

            self._sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=int(self.address))
        except Exception as e:
            raise RuntimeError(f"BME280 init failed: {e}") from e

    def read(self) -> dict[str, float]:
        if self._sensor is None:
            self.init()
        assert self._sensor is not None
        try:
            temp_c = float(self._sensor.temperature)
        except Exception:
            temp_c = _nan()
        try:
            press_hpa = float(self._sensor.pressure)
            press_atm = press_hpa * ATM_PER_HPA
        except Exception:
            press_atm = _nan()
        try:
            humid = float(self._sensor.relative_humidity)
        except Exception:
            humid = _nan()
        return {
            "bme_temp_c": temp_c,
            "bme_pressure_atm": press_atm,
            "bme_humidity_percent": humid,
        }

