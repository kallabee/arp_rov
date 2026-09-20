from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple


def _nan() -> float:
    return math.nan


HWMON_ROOT = Path("/sys/class/hwmon")
V3D_DRIVER_ROOT = Path("/sys/bus/platform/drivers/v3d")
CPU_TEMP_PATH = Path("/sys/class/thermal/thermal_zone0/temp")
PWM_MAX_DEFAULT = 255.0

# V3D queues that are actual GPU work (exclude CPU-side job accounting).
_GPU_QUEUES = frozenset({"bin", "render", "tfu", "csd", "cache_clean"})


def _read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text(encoding="utf-8").strip()
    except Exception:
        return None


def _read_cpu_temp_c(path: Path = CPU_TEMP_PATH) -> float:
    # Raspberry Pi OS: millidegC in thermal_zone0
    raw = _read_text(path)
    if raw is None:
        return _nan()
    try:
        return float(raw) / 1000.0
    except ValueError:
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


def find_fan_hwmon_dir(hwmon_root: Path = HWMON_ROOT) -> Optional[Path]:
    """Prefer the Pi 5 pwmfan hwmon; otherwise the first fan1_input/pwm1 dir."""
    fallback: Optional[Path] = None
    try:
        dirs = sorted(p for p in hwmon_root.iterdir() if p.is_dir())
    except Exception:
        return None
    for d in dirs:
        has_rpm = (d / "fan1_input").is_file()
        has_pwm = (d / "pwm1").is_file()
        if not has_rpm and not has_pwm:
            continue
        name = (_read_text(d / "name") or "").lower()
        if name == "pwmfan" or "fan" in name:
            return d
        if fallback is None:
            fallback = d
    return fallback


def find_fan_rpm_path(hwmon_root: Path = HWMON_ROOT) -> Optional[Path]:
    d = find_fan_hwmon_dir(hwmon_root)
    if d is None:
        return None
    fan = d / "fan1_input"
    return fan if fan.is_file() else None


def pwm_raw_to_percent(raw: float, pwm_max: float = PWM_MAX_DEFAULT) -> float:
    if pwm_max <= 0:
        return _nan()
    return float(max(0.0, min(100.0, 100.0 * raw / pwm_max)))


def find_gpu_stats_path(v3d_driver_root: Path = V3D_DRIVER_ROOT) -> Optional[Path]:
    try:
        matches = sorted(p for p in v3d_driver_root.glob("*/gpu_stats") if p.is_file())
    except Exception:
        return None
    return matches[0] if matches else None


def parse_gpu_stats(text: str) -> Optional[Tuple[int, dict[str, int]]]:
    """Return (timestamp_ns, {queue: runtime_ns}) from v3d gpu_stats."""
    runtimes: dict[str, int] = {}
    timestamp: Optional[int] = None
    for line in text.splitlines()[1:]:
        parts = line.split()
        if len(parts) < 4:
            continue
        name = parts[0]
        try:
            ts = int(parts[1])
            runtime = int(parts[3])
        except ValueError:
            continue
        if timestamp is None:
            timestamp = ts
        runtimes[name] = runtime
    if timestamp is None or not runtimes:
        return None
    return timestamp, runtimes


def gpu_util_from_samples(
    prev: Tuple[int, dict[str, int]],
    cur: Tuple[int, dict[str, int]],
) -> float:
    ts0, rt0 = prev
    ts1, rt1 = cur
    dt = ts1 - ts0
    if dt <= 0:
        return _nan()
    busy = 0
    for queue, runtime in rt1.items():
        if queue not in _GPU_QUEUES or queue not in rt0:
            continue
        delta = runtime - rt0[queue]
        if delta > busy:
            busy = delta
    return float(max(0.0, min(100.0, 100.0 * busy / dt)))


@dataclass
class RpiSensor:
    hwmon_root: Path = HWMON_ROOT
    v3d_driver_root: Path = V3D_DRIVER_ROOT
    _last: Optional[Tuple[int, int]] = None
    _last_t: float = 0.0
    _fan_hwmon_dir: Optional[Path] = None
    _gpu_stats_path: Optional[Path] = None
    _gpu_last: Optional[Tuple[int, dict[str, int]]] = None

    def _fan_dir(self) -> Optional[Path]:
        d = self._fan_hwmon_dir
        if d is None or not d.is_dir():
            d = find_fan_hwmon_dir(self.hwmon_root)
            self._fan_hwmon_dir = d
        return d

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

    def read_fan_rpm(self) -> float:
        d = self._fan_dir()
        if d is None:
            return _nan()
        raw = _read_text(d / "fan1_input")
        if raw is None:
            return _nan()
        try:
            rpm = float(raw)
        except ValueError:
            return _nan()
        if rpm < 0:
            return _nan()
        return rpm

    def read_fan_pwm_percent(self) -> float:
        d = self._fan_dir()
        if d is None:
            return _nan()
        raw_s = _read_text(d / "pwm1")
        if raw_s is None:
            return _nan()
        try:
            raw = float(raw_s)
        except ValueError:
            return _nan()
        max_s = _read_text(d / "pwm1_max")
        pwm_max = PWM_MAX_DEFAULT
        if max_s is not None:
            try:
                parsed = float(max_s)
            except ValueError:
                parsed = 0.0
            if parsed > 0:
                pwm_max = parsed
        return pwm_raw_to_percent(raw, pwm_max)

    def read_gpu_util_percent(self) -> float:
        path = self._gpu_stats_path
        if path is None or not path.is_file():
            path = find_gpu_stats_path(self.v3d_driver_root)
            self._gpu_stats_path = path
        if path is None:
            return _nan()
        raw = _read_text(path)
        if raw is None:
            self._gpu_stats_path = None
            return _nan()
        cur = parse_gpu_stats(raw)
        if cur is None:
            return _nan()
        prev = self._gpu_last
        self._gpu_last = cur
        if prev is None:
            return _nan()
        return gpu_util_from_samples(prev, cur)

    def read(self) -> dict[str, float]:
        return {
            "rpi_fan_rpm": self.read_fan_rpm(),
            "rpi_fan_pwm_percent": self.read_fan_pwm_percent(),
            "rpi_cpu_temp_c": _read_cpu_temp_c(),
            "rpi_cpu_util_percent": self.read_cpu_util_percent(),
            "rpi_gpu_util_percent": self.read_gpu_util_percent(),
        }
