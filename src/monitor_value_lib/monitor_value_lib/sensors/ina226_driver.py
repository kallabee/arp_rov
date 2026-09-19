"""INA226 driver (ported from unit_scripts/ina226.py).

This is kept inside monitor_value_lib so the library works without relying on
unit_scripts being importable.
"""

from __future__ import annotations

import logging
from math import trunc

from smbus2 import SMBus


def _to_bytes(register_value: int) -> list[int]:
    return [(register_value >> 8) & 0xFF, register_value & 0xFF]


class DeviceRangeError(Exception):
    __DEV_RNG_ERR = "Current out of range (overflow), for gain %.2fV"

    def __init__(self, gain_volts: float, device_max: bool = False):
        msg = self.__DEV_RNG_ERR % gain_volts
        if device_max:
            msg = msg + ", device limit reached"
        super().__init__(msg)
        self.gain_volts = gain_volts
        self.device_limit_reached = device_max


class INA226:
    AVG_1BIT = 0
    VCT_8244us_BIT = 7

    __REG_CONFIG = 0x00
    __REG_SHUNTVOLTAGE = 0x01
    __REG_BUSVOLTAGE = 0x02
    __REG_POWER = 0x03
    __REG_CURRENT = 0x04
    __REG_CALI = 0x05
    __REG_MASK = 0x06
    __REG_LIMIT = 0x07
    __REG_MANUFACTURER_ID = 0xFE
    __REG_DIE_ID = 0xFF

    __RST = 15
    __AVG0 = 9
    __VBUSCT0 = 6
    __VSHCT0 = 3
    __OVF = 2
    __CVRF = 3
    __CONT_SH_BUS = 7

    __BUS_RANGE = 40.96
    __GAIN_VOLTS = 0.08192
    __SHUNT_MILLIVOLTS_LSB = 0.0025
    __BUS_MILLIVOLTS_LSB = 1.25
    __CALIBRATION_FACTOR = 0.00512
    __MAX_CALIBRATION_VALUE = 0x7FFF
    __MAX_CURRENT_VALUE = 0x7FFF
    __CURRENT_LSB_FACTOR = 32768

    def __init__(
        self,
        busnum: int,
        address: int = 0x40,
        max_expected_amps: float | None = None,
        shunt_ohms: float = 0.002,
        log_level: int = logging.ERROR,
    ):
        if len(logging.getLogger().handlers) == 0:
            logging.basicConfig(level=log_level)
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(log_level)
        self._address = int(address)
        self._i2c = SMBus(int(busnum))
        self._shunt_ohms = float(shunt_ohms)
        self._max_expected_amps = max_expected_amps
        self._min_device_current_lsb = self._calculate_min_current_lsb()

    def configure(self, avg_mode: int = AVG_1BIT, bus_ct: int = VCT_8244us_BIT, shunt_ct: int = VCT_8244us_BIT):
        self._calibrate(self.__BUS_RANGE, self.__GAIN_VOLTS, self._max_expected_amps)
        configuration = (
            avg_mode << self.__AVG0
            | bus_ct << self.__VBUSCT0
            | shunt_ct << self.__VSHCT0
            | self.__CONT_SH_BUS
            | 1 << 14
        )
        self._configuration_register(configuration)

    def voltage(self) -> float:
        value = self._voltage_register()
        return float(value) * self.__BUS_MILLIVOLTS_LSB / 1000.0

    def current(self) -> float:
        self._handle_current_overflow()
        return float(self._current_register()) * self._current_lsb

    def power(self) -> float:
        self._handle_current_overflow()
        return float(self._power_register()) * self._power_lsb

    def shunt_voltage_mv(self) -> float:
        """Return shunt voltage in millivolts (signed)."""
        self._handle_current_overflow()
        raw = self.__read_register(self.__REG_SHUNTVOLTAGE, True)
        return float(raw) * self.__SHUNT_MILLIVOLTS_LSB

    def manufacturer_id(self) -> int:
        return int(self.__read_register(self.__REG_MANUFACTURER_ID))

    def die_id(self) -> int:
        return int(self.__read_register(self.__REG_DIE_ID))

    def _calibrate(self, bus_volts_max: float, shunt_volts_max: float, max_expected_amps: float | None = None):
        max_possible_amps = shunt_volts_max / self._shunt_ohms
        self._current_lsb = self._determine_current_lsb(max_expected_amps, max_possible_amps)
        self._power_lsb = self._current_lsb * 25.2
        calibration = trunc(self.__CALIBRATION_FACTOR / (self._current_lsb * self._shunt_ohms))
        if calibration <= 0:
            calibration = 1
        self._calibration_register(calibration)

    def _determine_current_lsb(self, max_expected_amps: float | None, max_possible_amps: float) -> float:
        if max_expected_amps is not None:
            current_lsb = min(max_expected_amps, max_possible_amps) / self.__CURRENT_LSB_FACTOR
        else:
            current_lsb = max_possible_amps / self.__CURRENT_LSB_FACTOR
        if current_lsb < self._min_device_current_lsb:
            current_lsb = self._min_device_current_lsb
        return float(current_lsb)

    def _calculate_min_current_lsb(self) -> float:
        return float(self.__CALIBRATION_FACTOR / (self._shunt_ohms * self.__MAX_CALIBRATION_VALUE))

    def _has_current_overflow(self) -> bool:
        ovf = (self._read_mask_register() >> self.__OVF) & 1
        return bool(ovf)

    def _handle_current_overflow(self) -> None:
        if self._has_current_overflow():
            raise DeviceRangeError(self.__GAIN_VOLTS)

    def _configuration_register(self, register_value: int) -> None:
        self.__write_register(self.__REG_CONFIG, int(register_value))

    def _voltage_register(self) -> int:
        return self.__read_register(self.__REG_BUSVOLTAGE)

    def _current_register(self) -> int:
        return self.__read_register(self.__REG_CURRENT, True)

    def _power_register(self) -> int:
        return self.__read_register(self.__REG_POWER)

    def _calibration_register(self, register_value: int) -> None:
        self.__write_register(self.__REG_CALI, int(register_value))

    def _read_mask_register(self) -> int:
        return self.__read_register(self.__REG_MASK)

    def __write_register(self, register: int, register_value: int) -> None:
        register_bytes = _to_bytes(int(register_value) & 0xFFFF)
        self._i2c.write_i2c_block_data(self._address, int(register), register_bytes)

    def __read_register(self, register: int, negative_value_supported: bool = False) -> int:
        result = self._i2c.read_word_data(self._address, int(register)) & 0xFFFF
        register_value = ((result << 8) & 0xFF00) + (result >> 8)
        if negative_value_supported and register_value > 32767:
            register_value -= 65536
        return int(register_value)

