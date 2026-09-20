"""PCA9685 light PWM helper used by the actuator subscriber."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping

import yaml

from device_registry import DeviceRegistry


def _resolve_light_pwm_yaml() -> Path:
    cwd = Path.cwd().resolve()
    for base in (cwd, *cwd.parents):
        cand = (
            base
            / "src"
            / "thruster_controller"
            / "thruster_controller"
            / "light_pwm.yaml"
        )
        if cand.is_file():
            return cand
    return Path(__file__).resolve().parent / "light_pwm.yaml"


def load_light_pwm_config(path: Path | None = None) -> tuple[dict[str, int], int]:
    path = path or _resolve_light_pwm_yaml()
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"Top-level YAML must be a mapping: {path}")
    channels = raw.get("channels")
    if not isinstance(channels, dict) or not channels:
        raise ValueError(f"{path}: 'channels' must be a non-empty mapping")
    out: dict[str, int] = {}
    for name, ch in channels.items():
        if not isinstance(name, str) or not name:
            raise ValueError(f"{path}: invalid light name {name!r}")
        if not isinstance(ch, int) or ch < 0 or ch > 15:
            raise ValueError(f"{path}: channel for {name} must be 0..15, got {ch}")
        out[name] = ch
    freq = int(raw.get("frequency_hz", 1000))
    if freq <= 0:
        raise ValueError(f"{path}: frequency_hz must be > 0")
    return out, freq


class LightPwmDriver:
    def __init__(self, channels: Mapping[str, int], freq_hz: int, i2c_addr: int, linux_bus: int | None):
        from adafruit_pca9685 import PCA9685

        if linux_bus is not None:
            from adafruit_extended_bus import ExtendedI2C

            i2c = ExtendedI2C(int(linux_bus))
        else:
            import busio
            from board import SCL, SDA

            i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=i2c_addr)
        self.pca.frequency = int(freq_hz)
        self.channels = dict(channels)

    def set_duty(self, name: str, value: float) -> None:
        if name not in self.channels:
            return
        duty = max(0.0, min(1.0, float(value)))
        self.pca.channels[self.channels[name]].duty_cycle = int(0xFFFF * duty)

    def apply(self, names: list[str], duties: list[float]) -> None:
        n = min(len(names), len(duties))
        for i in range(n):
            self.set_duty(names[i], duties[i])

    def all_off(self) -> None:
        for name in self.channels:
            self.set_duty(name, 0.0)


def open_light_pwm_driver() -> LightPwmDriver:
    channels, freq = load_light_pwm_config()
    reg = DeviceRegistry.from_src_default()
    dev = reg.get_i2c("light_pwm")
    return LightPwmDriver(channels, freq, dev.addr, dev.linux_bus)
