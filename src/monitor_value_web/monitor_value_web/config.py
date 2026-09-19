from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_web_config(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"config root must be a mapping: {path}")
    http = raw.get("http") or {}
    topics = raw.get("topics") or {}
    return {
        "http": {
            "host": str(http.get("host", "0.0.0.0")),
            "port": int(http.get("port", 8080)),
        },
        "topics": {
            "monitor": str(topics.get("monitor", "rov/monitor_value")),
            "imu_snapshot": str(topics.get("imu_snapshot", "rov/imu_nav/snapshot")),
        },
        "stale_sec": float(raw.get("stale_sec", 3.0)),
        "imu_min_interval_sec": float(raw.get("imu_min_interval_sec", 0.1)),
        "temperatures": list(raw.get("temperatures") or []),
        "temperature_gauge": dict(raw.get("temperature_gauge") or {"min": 0, "max": 80}),
        "leaks": list(raw.get("leaks") or []),
        "gauges": dict(raw.get("gauges") or {}),
    }


def public_config(cfg: dict[str, Any]) -> dict[str, Any]:
    return {
        "stale_sec": cfg["stale_sec"],
        "temperatures": cfg["temperatures"],
        "temperature_gauge": cfg["temperature_gauge"],
        "leaks": cfg["leaks"],
        "gauges": cfg["gauges"],
        "topics": cfg["topics"],
    }
