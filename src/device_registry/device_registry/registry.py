"""Load per-device I2C settings from ``src/device_i2c.yaml``."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Union

import yaml

Number = Union[int, float]


@dataclass(frozen=True)
class I2CDevice:
    """Resolved I2C endpoint for a logical device."""

    addr: int
    """7-bit I2C address (0..127)."""

    linux_bus: Optional[int] = None
    """Linux adapter index (``N`` in ``/dev/i2c-N``), if configured."""


def _parse_addr(value: Any) -> int:
    if isinstance(value, int):
        if value < 0 or value > 0x7F:
            raise ValueError(f"I2C address out of 7-bit range: {value!r}")
        return value
    if isinstance(value, str):
        s = value.strip().lower()
        if s.startswith("0x"):
            return int(s, 16)
        return int(s, 0)
    raise TypeError(f"I2C address must be int or str, got {type(value)}")


def _parse_bus(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool):
        raise TypeError("linux_bus must not be a bool")
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        return int(value.strip(), 0)
    raise TypeError(f"linux_bus must be int-like, got {type(value)}")


class DeviceRegistry:
    """Read ``device_i2c.yaml`` and resolve entries by logical device id."""

    _ENV_PATH = "DEVICE_I2C_YAML"

    def __init__(self, data: Mapping[str, Any]) -> None:
        self._data: Dict[str, Any] = dict(data)

    @classmethod
    def load(cls, yaml_path: str | Path) -> DeviceRegistry:
        path = Path(yaml_path)
        if not path.is_file():
            raise FileNotFoundError(f"device_i2c yaml not found: {path}")
        with path.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if raw is None:
            raw = {}
        if not isinstance(raw, dict):
            raise ValueError(f"Top-level YAML must be a mapping: {path}")
        return cls(raw)

    @classmethod
    def from_src_default(cls) -> DeviceRegistry:
        """Resolve default YAML next to ``src/`` (workspace layout).

        Order:
        1. ``DEVICE_I2C_YAML`` environment variable (absolute or relative ``cwd``)
        2. ``<workspace>/src/device_i2c.yaml`` where ``<workspace>/src`` is the parent
           of the ``device_registry`` package source tree.
        """
        env = os.environ.get(cls._ENV_PATH)
        if env:
            return cls.load(env)

        # Preferred: workspace layout during development.
        # .../src/device_registry/device_registry/registry.py -> .../src
        here = Path(__file__).resolve()
        src_dir = here.parents[2]
        default = src_dir / "device_i2c.yaml"
        if default.is_file():
            return cls.load(default)

        # Fallbacks: running from an installed environment but current working directory
        # is the workspace root (or inside it).
        cwd = Path.cwd().resolve()
        for base in (cwd, *cwd.parents):
            for candidate in (base / "src" / "device_i2c.yaml", base / "device_i2c.yaml"):
                if candidate.is_file():
                    return cls.load(candidate)

        raise FileNotFoundError(
            "device_i2c yaml not found. Set DEVICE_I2C_YAML or place src/device_i2c.yaml "
            f"under cwd. Tried: {default}"
        )

    def get_raw(self, device: str) -> Any:
        if device not in self._data:
            raise KeyError(f"Unknown device in device_i2c.yaml: {device!r}")
        return self._data[device]

    def get_i2c(self, device: str) -> I2CDevice:
        raw = self.get_raw(device)
        if not isinstance(raw, dict):
            raise ValueError(f"{device!r} must be a mapping with addr (and optional bus)")
        addr = _parse_addr(raw.get("addr"))
        linux_bus = _parse_bus(raw.get("bus", raw.get("linux_bus")))
        return I2CDevice(addr=addr, linux_bus=linux_bus)
