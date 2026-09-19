from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import smbus2


def _nan() -> float:
    return math.nan


# INA226 registers
_REG_CONFIG = 0x00
_REG_SHUNT_VOLTAGE = 0x01
_REG_BUS_VOLTAGE = 0x02
_REG_POWER = 0x03
_REG_CURRENT = 0x04
_REG_CALIBRATION = 0x05


def _swap16(v: int) -> int:
    return ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)


def _to_signed16(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


@dataclass
class Ina226Sensor:
    busnum: int
    address: int = 0x40
    shunt_ohms: float = 0.002
    max_expected_amps: float = 25.0

    _bus: Optional[smbus2.SMBus] = None
    _current_lsb: float = 0.0
    _power_lsb: float = 0.0

    def init(self) -> None:
        if self._bus is not None:
            return
        try:
            self._bus = smbus2.SMBus(int(self.busnum))
        except Exception as e:
            raise RuntimeError(f"INA226: SMBus({self.busnum}) open failed: {e}") from e

        # LSB selection: choose current LSB so that max expected amps is representable.
        # INA226 current register is signed 16-bit.
        self._current_lsb = float(self.max_expected_amps) / 32767.0
        if self._current_lsb <= 0:
            self._current_lsb = 1e-3
        self._power_lsb = 25.0 * self._current_lsb

        # Calibration register: 0.00512 / (current_lsb * shunt_ohms)
        cal = int(0.00512 / (self._current_lsb * float(self.shunt_ohms)))
        if cal <= 0:
            cal = 1
        self._write_u16(_REG_CALIBRATION, cal)

    def _read_u16(self, reg: int) -> int:
        assert self._bus is not None
        v = self._bus.read_word_data(int(self.address), int(reg))
        return _swap16(int(v))

    def _write_u16(self, reg: int, value: int) -> None:
        assert self._bus is not None
        self._bus.write_word_data(int(self.address), int(reg), _swap16(int(value) & 0xFFFF))

    def read(self) -> dict[str, float]:
        if self._bus is None:
            self.init()
        assert self._bus is not None
        try:
            bus_raw = self._read_u16(_REG_BUS_VOLTAGE)
            # Bus voltage LSB = 1.25mV
            voltage_v = float(bus_raw) * 0.00125
        except Exception:
            voltage_v = _nan()

        try:
            cur_raw = _to_signed16(self._read_u16(_REG_CURRENT))
            current_a = float(cur_raw) * self._current_lsb
        except Exception:
            current_a = _nan()

        try:
            p_raw = self._read_u16(_REG_POWER)
            power_w = float(p_raw) * self._power_lsb
        except Exception:
            # Fallback: compute if V and I are valid
            power_w = voltage_v * current_a if not (math.isnan(voltage_v) or math.isnan(current_a)) else _nan()

        return {"current_a": current_a, "voltage_v": voltage_v, "power_w": power_w}

