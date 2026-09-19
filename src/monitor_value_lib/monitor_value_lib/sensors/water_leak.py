from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

from monitor_value_lib.sensors.ads1015_reader import Ads1015Reader


def _nan() -> float:
    return math.nan


@dataclass(frozen=True)
class WaterLeakChannelConfig:
    adc_channel: int
    threshold_v: float
    monitor_index: int  # 0 or 1 -> water_ch0_* / water_ch1_*


def parse_water_leak_channels(cfg: dict) -> List[WaterLeakChannelConfig]:
    """Parse ``channels`` list from YAML (per-channel threshold)."""
    raw = cfg.get("channels")
    if isinstance(raw, list) and raw:
        out: List[WaterLeakChannelConfig] = []
        for i, item in enumerate(raw):
            if not isinstance(item, dict):
                continue
            adc = int(item.get("adc_channel", item.get("adc", i)))
            thr = float(item.get("threshold_v", item.get("threshold", 3.0)))
            idx = int(item.get("monitor_index", i))
            out.append(WaterLeakChannelConfig(adc_channel=adc, threshold_v=thr, monitor_index=idx))
        if out:
            return out

    adc_channels = cfg.get("adc_channels", [0, 1])
    thresholds = cfg.get("threshold_v", [3.0, 3.0])
    if not isinstance(adc_channels, Sequence):
        adc_channels = [0, 1]
    if not isinstance(thresholds, Sequence):
        thresholds = [float(thresholds)]
    adcs = [int(x) for x in adc_channels]
    thrs = [float(x) for x in thresholds]
    while len(thrs) < len(adcs):
        thrs.append(thrs[-1] if thrs else 3.0)
    return [
        WaterLeakChannelConfig(adc_channel=adcs[i], threshold_v=thrs[i], monitor_index=i)
        for i in range(min(2, len(adcs)))
    ]


@dataclass
class WaterLeakSensor:
    reader: Ads1015Reader
    channels: List[WaterLeakChannelConfig]

    def init(self) -> None:
        self.reader.init()

    def read(self) -> Dict[str, float | bool]:
        out: Dict[str, float | bool] = {
            "water_ch0_probe_v": _nan(),
            "water_ch0_detected": False,
            "water_ch1_probe_v": _nan(),
            "water_ch1_detected": False,
        }
        field_map: Dict[int, Tuple[str, str]] = {
            0: ("water_ch0_probe_v", "water_ch0_detected"),
            1: ("water_ch1_probe_v", "water_ch1_detected"),
        }
        for ch_cfg in self.channels:
            if ch_cfg.monitor_index not in field_map:
                continue
            v_key, d_key = field_map[ch_cfg.monitor_index]
            try:
                v = float(self.reader.read_voltage(ch_cfg.adc_channel))
                out[v_key] = v
                out[d_key] = v < float(ch_cfg.threshold_v)
            except Exception:
                out[v_key] = _nan()
                out[d_key] = False
        return out
