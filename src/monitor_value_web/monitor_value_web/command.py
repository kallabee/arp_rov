from __future__ import annotations

import time
from typing import Any, Iterable


TWIST_KEYS = ("x", "y", "z", "yaw", "pitch", "roll")
HAND_KEYS = ("grab", "roll")

DEFAULT_LIGHTS = (
    "CeilA",
    "CeilB",
    "VisFwdBtm",
    "VisFwdTel",
    "Uv",
    "Test",
    "Ir",
    "VisFwdTop",
    "VisDwn",
)


def zero_twist() -> dict[str, float]:
    return {k: 0.0 for k in TWIST_KEYS}


def zero_hand() -> dict[str, float]:
    return {k: 0.0 for k in HAND_KEYS}


def zero_lights(names: Iterable[str]) -> dict[str, float]:
    return {name: 0.0 for name in names}


def _clamp(value: Any, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def parse_twist(payload: Any) -> dict[str, float]:
    src = payload if isinstance(payload, dict) else {}
    return {k: _clamp(src.get(k, 0.0), -1.0, 1.0) for k in TWIST_KEYS}


def parse_hand(payload: Any) -> dict[str, float]:
    src = payload if isinstance(payload, dict) else {}
    return {k: _clamp(src.get(k, 0.0), -1.0, 1.0) for k in HAND_KEYS}


def parse_lights(payload: Any, names: Iterable[str]) -> dict[str, float]:
    src = payload if isinstance(payload, dict) else {}
    out: dict[str, float] = {}
    for name in names:
        raw = src.get(name, 0.0)
        out[name] = _clamp(raw, 0.0, 1.0)
    return out


def merge_lights(current: dict[str, float], payload: Any, names: Iterable[str]) -> dict[str, float]:
    src = payload if isinstance(payload, dict) else {}
    out = dict(current)
    for name in names:
        if name in src:
            out[name] = _clamp(src[name], 0.0, 1.0)
        elif name not in out:
            out[name] = 0.0
    return out


def fingerprint(group: str, data: dict[str, float]) -> tuple:
    items = tuple((k, round(float(data[k]), 4)) for k in sorted(data.keys()))
    return (group, items)


class EchoFilter:
    """Ignore our own recently published payloads so UI is not pinned to them."""

    def __init__(self, window_sec: float = 1.5):
        self.window_sec = float(window_sec)
        self._sent: list[tuple[float, tuple]] = []

    def note(self, group: str, data: dict[str, float]) -> None:
        self._gc()
        self._sent.append((time.monotonic(), fingerprint(group, data)))

    def is_echo(self, group: str, data: dict[str, float]) -> bool:
        self._gc()
        fp = fingerprint(group, data)
        return any(stored == fp for _ts, stored in self._sent)

    def _gc(self) -> None:
        now = time.monotonic()
        self._sent = [(ts, fp) for ts, fp in self._sent if now - ts <= self.window_sec]


def twist_from_msg(msg: Any) -> dict[str, float]:
    return parse_twist(
        {
            "x": msg.linear.x,
            "y": msg.linear.y,
            "z": msg.linear.z,
            "yaw": msg.angular.z,
            "pitch": msg.angular.y,
            "roll": msg.angular.x,
        }
    )


def twist_to_msg(data: dict[str, float], cls: Any) -> Any:
    msg = cls()
    msg.linear.x = float(data["x"])
    msg.linear.y = float(data["y"])
    msg.linear.z = float(data["z"])
    msg.angular.x = float(data["roll"])
    msg.angular.y = float(data["pitch"])
    msg.angular.z = float(data["yaw"])
    return msg


def hand_from_msg(msg: Any) -> dict[str, float]:
    return parse_hand({"grab": msg.grab, "roll": msg.roll})


def hand_to_msg(data: dict[str, float], cls: Any) -> Any:
    msg = cls()
    msg.grab = float(data["grab"])
    msg.roll = float(data["roll"])
    return msg


def lights_from_msg(msg: Any, names: Iterable[str]) -> dict[str, float]:
    allowed = set(names)
    n = min(len(msg.names), len(msg.duties))
    incoming: dict[str, float] = {}
    for i in range(n):
        name = str(msg.names[i])
        if name in allowed:
            incoming[name] = _clamp(msg.duties[i], 0.0, 1.0)
    return incoming


def lights_to_msg(data: dict[str, float], names: Iterable[str], cls: Any) -> Any:
    msg = cls()
    ordered = list(names)
    msg.names = ordered
    msg.duties = [float(data.get(name, 0.0)) for name in ordered]
    return msg


def default_command(light_names: Iterable[str]) -> dict[str, Any]:
    return {
        "twist": zero_twist(),
        "hand": zero_hand(),
        "lights": zero_lights(light_names),
        "origin": "none",
        "seq": 0,
    }
