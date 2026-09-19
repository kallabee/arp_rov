from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


def _nan() -> float:
    return math.nan


def _read_cpu_temp_c() -> float:
    # Raspberry Pi OS: millidegC in thermal_zone0
    p = Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        raw = p.read_text(encoding="utf-8").strip()
        return float(raw) / 1000.0
    except Exception:
        return _nan()


def _read_proc_stat() -> Optional[Tuple[int, int]]:
    # returns (total_jiffies, idle_jiffies)
    try:
        with open("/proc/stat", "r", encoding="utf-8") as f:
            line = f.readline()
        if not line.startswith("cpu "):
            return None
        parts = line.split()
        values = [int(x) for x in parts[1:8]]  # user nice system idle iowait irq softirq
        total = sum(values)
        idle = values[3] + values[4]
        return total, idle
    except Exception:
        return None


@dataclass
class RpiSensor:
    _last: Optional[Tuple[int, int]] = None
    _last_t: float = 0.0

    def read_cpu_util_percent(self) -> float:
        # Prefer psutil if installed; fallback to /proc/stat deltas.
        try:
            import psutil  # type: ignore

            return float(psutil.cpu_percent(interval=None))
        except Exception:
            now = time.monotonic()
            cur = _read_proc_stat()
            if cur is None:
                return _nan()
            if self._last is None:
                self._last = cur
                self._last_t = now
                return _nan()
            total0, idle0 = self._last
            total1, idle1 = cur
            dt_total = total1 - total0
            dt_idle = idle1 - idle0
            self._last = cur
            self._last_t = now
            if dt_total <= 0:
                return _nan()
            util = 100.0 * (1.0 - (dt_idle / dt_total))
            return float(max(0.0, min(100.0, util)))

    def read(self) -> dict[str, float]:
        return {
            "rpi_fan_rpm": _nan(),  # hardware-specific; left as NaN for now
            "rpi_cpu_temp_c": _read_cpu_temp_c(),
            "rpi_cpu_util_percent": self.read_cpu_util_percent(),
            "rpi_gpu_util_percent": _nan(),  # not portable; NaN by default
        }

