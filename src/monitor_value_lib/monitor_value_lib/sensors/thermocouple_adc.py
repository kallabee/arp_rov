from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

from monitor_value_lib.sensors.ads1015_reader import Ads1015Reader

MV_PER_DEGC = 0.005  # AD8495: 5 mV/°C
DEFAULT_VREF_V = 1.25


def _nan() -> float:
    return math.nan


def vout_to_celsius(vout_v: float, vref_v: float) -> float:
    return (vout_v - vref_v) / MV_PER_DEGC


@dataclass
class ThermocoupleAdcSensor:
    """K-type thermocouple via AD8495 on ADS1015 (cold-junction compensated output)."""

    reader: Ads1015Reader
    adc_channels: List[int]
    vref_v: float = DEFAULT_VREF_V

    def init(self) -> None:
        self.reader.init()

    def read(self) -> Dict[str, float]:
        out: Dict[str, float] = {
            "thermocouple_ch2_temp_c": _nan(),
            "thermocouple_ch3_temp_c": _nan(),
        }
        field_by_adc = {2: "thermocouple_ch2_temp_c", 3: "thermocouple_ch3_temp_c"}
        for adc_ch in self.adc_channels:
            key = field_by_adc.get(int(adc_ch))
            if key is None:
                continue
            try:
                vout = float(self.reader.read_voltage(int(adc_ch)))
                out[key] = float(vout_to_celsius(vout, self.vref_v))
            except Exception:
                out[key] = _nan()
        return out
