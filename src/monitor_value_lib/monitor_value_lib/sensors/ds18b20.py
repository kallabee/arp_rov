from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

W1_DEVICES = Path("/sys/bus/w1/devices")
DT_BASE = Path("/sys/firmware/devicetree/base")
ROM_ID_RE = re.compile(r"^[0-9a-f]{2}-[0-9a-f]{12}$", re.IGNORECASE)
READ_RETRIES = 5
CONVERSION_WAIT_S = 1.0
RESCAN_WAIT_S = 2.5
POWER_ON_TEMP_C = 85.0


def _nan() -> float:
    return math.nan


def list_masters() -> list[Path]:
    return sorted(W1_DEVICES.glob("w1_bus_master*"), key=lambda p: p.name)


def all_slaves_on_master(master: Path) -> list[str]:
    ids: list[str] = []
    for name in ("w1_master_slaves", "w1_master/slaves"):
        slaves_path = master / name
        if slaves_path.is_file():
            ids.extend(s for s in slaves_path.read_text().split() if ROM_ID_RE.match(s))
    if ids:
        return ids
    return sorted(p.name for p in master.iterdir() if ROM_ID_RE.match(p.name))


def slaves_on_master(master: Path) -> list[str]:
    return [s for s in all_slaves_on_master(master) if s.startswith("28-")]


def bcm_gpio_from_onewire_node(node: Path) -> int:
    raw = (node / "gpios").read_bytes()
    if len(raw) < 8:
        raise ValueError(f"short gpios property on {node.name}")
    return int.from_bytes(raw[4:8], "big")


def list_onewire_nodes() -> list[Path]:
    return sorted(DT_BASE.glob("onewire@*"), key=lambda p: int(p.name.split("@")[1], 16))


def master_to_bcm_gpio() -> dict[str, int]:
    masters = list_masters()
    nodes = list_onewire_nodes()
    return {
        master.name: bcm_gpio_from_onewire_node(node)
        for master, node in zip(masters, nodes)
    }


def rescan_master(master: Path) -> None:
    search = master / "w1_master_search"
    if not search.is_file():
        return
    try:
        search.write_text("1")
    except OSError:
        return
    time.sleep(RESCAN_WAIT_S)


def master_name_for_sensor(sensor: Path) -> str | None:
    for part in sensor.resolve().parts:
        if part.startswith("w1_bus_master"):
            return part
    return None


def discover_gpio_map(gpios: tuple[int, ...]) -> dict[int, Path | None]:
    mapping: dict[int, Path | None] = {gpio: None for gpio in gpios}
    bcm_by_master = master_to_bcm_gpio()

    for master in list_masters():
        bcm = bcm_by_master.get(master.name)
        if bcm not in mapping:
            continue
        slaves = slaves_on_master(master)
        if slaves:
            candidate = W1_DEVICES / slaves[0]
            if candidate.is_dir():
                mapping[bcm] = candidate

    for sensor in sorted(W1_DEVICES.glob("28-*")):
        if not sensor.is_dir():
            continue
        master = master_name_for_sensor(sensor)
        if master is None:
            continue
        bcm = bcm_by_master.get(master)
        if bcm in mapping:
            mapping[bcm] = sensor

    return mapping


def _read_from_temperature_sysfs(sensor_dir: Path) -> float | None:
    temp_path = sensor_dir / "temperature"
    if not temp_path.is_file():
        return None
    try:
        raw = temp_path.read_text().strip()
    except OSError:
        return None
    if not raw:
        return None
    return int(raw) / 1000.0


def _read_from_hwmon(sensor_dir: Path) -> float | None:
    for temp_input in sensor_dir.glob("hwmon/hwmon*/temp*_input"):
        try:
            raw = temp_input.read_text().strip()
        except OSError:
            continue
        if raw:
            return int(raw) / 1000.0
    return None


def _parse_w1_slave_text(text: str) -> float | None:
    lines = text.splitlines()
    if len(lines) < 2 or "YES" not in lines[0]:
        return None
    t_eq = lines[1].split("=")
    if len(t_eq) != 2:
        return None
    return int(t_eq[1]) / 1000.0


def _read_from_w1_slave(sensor_dir: Path) -> float | None:
    path = sensor_dir / "w1_slave"
    if not path.is_file():
        return None
    for _ in range(2):
        try:
            temp = _parse_w1_slave_text(path.read_text())
        except OSError:
            return None
        if temp is not None:
            return temp
        time.sleep(CONVERSION_WAIT_S)
    return None


def read_celsius(sensor_dir: Path) -> float:
    last_err: OSError | None = None
    for attempt in range(READ_RETRIES):
        if not sensor_dir.is_dir():
            raise OSError(f"sensor vanished: {sensor_dir.name}")
        try:
            temp: float | None = None
            for reader in (_read_from_temperature_sysfs, _read_from_hwmon, _read_from_w1_slave):
                temp = reader(sensor_dir)
                if temp is not None:
                    break
            if temp is not None:
                if temp >= POWER_ON_TEMP_C and attempt + 1 < READ_RETRIES:
                    time.sleep(CONVERSION_WAIT_S)
                    continue
                return temp
            last_err = OSError(f"empty or invalid read: {sensor_dir.name}")
        except OSError as exc:
            last_err = exc
        if attempt + 1 < READ_RETRIES:
            time.sleep(CONVERSION_WAIT_S)
    raise last_err or OSError(f"read failed: {sensor_dir.name}")


@dataclass
class Ds18b20Sensor:
    """One DS18B20 per BCM GPIO bus column (ch0/ch1)."""

    gpios: Tuple[int, ...] = (17, 27)
    rescan_interval_sec: float = 10.0

    _last_rescan_monotonic: float = 0.0
    _warned_no_bus: bool = False

    def init(self) -> None:
        if not W1_DEVICES.is_dir():
            raise RuntimeError(
                "1-Wire bus not found (/sys/bus/w1/devices). "
                "Add dtoverlay=w1-gpio,gpiopin=... to config.txt and reboot."
            )

    def _maybe_rescan(self) -> None:
        now = time.monotonic()
        if now - self._last_rescan_monotonic < self.rescan_interval_sec:
            return
        self._last_rescan_monotonic = now
        bcm_by_master = master_to_bcm_gpio()
        for master in list_masters():
            bcm = bcm_by_master.get(master.name)
            if bcm not in self.gpios:
                continue
            if slaves_on_master(master):
                continue
            rescan_master(master)

    def read(self) -> Dict[str, float]:
        out: Dict[str, float] = {
            "ds18b20_ch0_temp_c": _nan(),
            "ds18b20_ch1_temp_c": _nan(),
        }
        field_by_index = {0: "ds18b20_ch0_temp_c", 1: "ds18b20_ch1_temp_c"}

        if not W1_DEVICES.is_dir():
            return out

        self._maybe_rescan()
        gpio_map = discover_gpio_map(self.gpios)
        if not any(gpio_map.values()) and not self._warned_no_bus:
            self._warned_no_bus = True

        for idx, gpio in enumerate(self.gpios):
            if idx > 1:
                break
            key = field_by_index.get(idx)
            if key is None:
                continue
            sensor = gpio_map.get(int(gpio))
            if sensor is None:
                continue
            try:
                out[key] = float(read_celsius(sensor))
            except OSError:
                out[key] = _nan()
        return out
