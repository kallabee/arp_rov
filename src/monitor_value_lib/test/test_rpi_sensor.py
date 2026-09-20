# Copyright 2026
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
from pathlib import Path

from monitor_value_lib.sensors.rpi import (
    RpiSensor,
    find_fan_rpm_path,
    find_gpu_stats_path,
    gpu_util_from_samples,
    parse_gpu_stats,
)


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _gpu_stats(ts: int, runtimes: dict[str, int], jobs: int = 0) -> str:
    lines = ["queue\ttimestamp\tjobs\truntime"]
    for name in ("bin", "render", "tfu", "csd", "cache_clean", "cpu"):
        lines.append(f"{name}\t{ts}\t{jobs}\t{runtimes.get(name, 0)}")
    return "\n".join(lines) + "\n"


def test_find_fan_prefers_pwmfan(tmp_path: Path) -> None:
    _write(tmp_path / "hwmon0" / "name", "cpu_thermal\n")
    _write(tmp_path / "hwmon1" / "name", "other\n")
    _write(tmp_path / "hwmon1" / "fan1_input", "999\n")
    _write(tmp_path / "hwmon2" / "name", "pwmfan\n")
    _write(tmp_path / "hwmon2" / "fan1_input", "4327\n")
    found = find_fan_rpm_path(tmp_path)
    assert found == tmp_path / "hwmon2" / "fan1_input"


def test_find_fan_falls_back_to_any_fan1_input(tmp_path: Path) -> None:
    _write(tmp_path / "hwmon0" / "name", "cpu_thermal\n")
    _write(tmp_path / "hwmon3" / "name", "custom\n")
    _write(tmp_path / "hwmon3" / "fan1_input", "1200\n")
    found = find_fan_rpm_path(tmp_path)
    assert found == tmp_path / "hwmon3" / "fan1_input"


def test_read_fan_rpm_and_pwm(tmp_path: Path) -> None:
    _write(tmp_path / "hwmon2" / "name", "pwmfan\n")
    _write(tmp_path / "hwmon2" / "fan1_input", "4327\n")
    _write(tmp_path / "hwmon2" / "pwm1", "125\n")
    sensor = RpiSensor(hwmon_root=tmp_path, v3d_driver_root=tmp_path / "missing")
    got = sensor.read()
    assert got["rpi_fan_rpm"] == 4327.0
    assert abs(got["rpi_fan_pwm_percent"] - (100.0 * 125.0 / 255.0)) < 1e-6
    assert math.isnan(got["rpi_gpu_util_percent"])


def test_read_fan_pwm_uses_pwm1_max(tmp_path: Path) -> None:
    _write(tmp_path / "hwmon2" / "name", "pwmfan\n")
    _write(tmp_path / "hwmon2" / "pwm1", "50\n")
    _write(tmp_path / "hwmon2" / "pwm1_max", "100\n")
    sensor = RpiSensor(hwmon_root=tmp_path, v3d_driver_root=tmp_path / "missing")
    assert sensor.read_fan_pwm_percent() == 50.0


def test_read_fan_missing_is_nan(tmp_path: Path) -> None:
    sensor = RpiSensor(hwmon_root=tmp_path, v3d_driver_root=tmp_path / "missing")
    assert math.isnan(sensor.read_fan_rpm())


def test_parse_and_gpu_util_uses_max_queue_delta() -> None:
    prev = parse_gpu_stats(_gpu_stats(1_000_000_000, {"bin": 100, "render": 50}))
    cur = parse_gpu_stats(_gpu_stats(2_000_000_000, {"bin": 100, "render": 50 + 250_000_000}))
    assert prev is not None and cur is not None
    # render did 0.25s of work in 1.0s; bin did none. max-delta, not delta-of-max.
    util = gpu_util_from_samples(prev, cur)
    assert abs(util - 25.0) < 1e-6


def test_gpu_util_ignores_cpu_queue() -> None:
    prev = parse_gpu_stats(_gpu_stats(0, {"cpu": 0, "render": 0}))
    cur = parse_gpu_stats(_gpu_stats(1_000_000_000, {"cpu": 900_000_000, "render": 10_000_000}))
    assert prev is not None and cur is not None
    util = gpu_util_from_samples(prev, cur)
    assert abs(util - 1.0) < 1e-6


def test_read_gpu_util_needs_two_samples(tmp_path: Path) -> None:
    stats = tmp_path / "1002000000.v3d" / "gpu_stats"
    _write(stats, _gpu_stats(1_000, {"render": 0}))
    sensor = RpiSensor(hwmon_root=tmp_path / "hwmon", v3d_driver_root=tmp_path)
    assert find_gpu_stats_path(tmp_path) == stats
    assert math.isnan(sensor.read_gpu_util_percent())
    _write(stats, _gpu_stats(1_000 + 500_000_000, {"render": 100_000_000}))
    util = sensor.read_gpu_util_percent()
    assert abs(util - 20.0) < 1e-6
