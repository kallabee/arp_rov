from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from monitor_value_web.camera import public_camera_meta
from monitor_value_web.command import DEFAULT_LIGHTS
from monitor_value_web.record import normalize_record_config, recording_public_meta

DEFAULT_CAMERAS = [
    {"id": "cam0", "path": "cam0", "nickname": "Ceiling"},
    {"id": "cam1", "path": "cam1", "nickname": "Canopy"},
]


def _cameras(raw: Any) -> dict[str, Any]:
    src = raw if isinstance(raw, dict) else {}
    items_raw = src.get("items")
    items: list[dict[str, Any]] = []
    if isinstance(items_raw, list) and items_raw:
        for item in items_raw:
            if not isinstance(item, dict) or not item.get("id"):
                continue
            cam_id = str(item["id"])
            row: dict[str, Any] = {
                "id": cam_id,
                "path": str(item.get("path") or cam_id),
                "nickname": str(item.get("nickname") or cam_id),
            }
            if "index" in item:
                row["index"] = int(item["index"])
            items.append(row)
    if not items:
        items = [dict(x) for x in DEFAULT_CAMERAS]
    backend = str(src.get("backend") or "mediamtx").strip().lower()
    momo_raw = src.get("momo") if isinstance(src.get("momo"), dict) else {}
    mtx_raw = src.get("mediamtx") if isinstance(src.get("mediamtx"), dict) else {}
    return {
        "backend": backend,
        "api": str(mtx_raw.get("api") or src.get("api") or "http://127.0.0.1:9997"),
        "mediamtx": {
            "api": str(mtx_raw.get("api") or src.get("api") or "http://127.0.0.1:9997"),
            "hls": str(mtx_raw.get("hls") or "http://127.0.0.1:8888"),
            "webrtc": str(mtx_raw.get("webrtc") or "http://127.0.0.1:8889"),
        },
        "momo": {
            "binary": str(momo_raw.get("binary") or "momo"),
            "extra_args": list(momo_raw.get("extra_args") or ["--no-audio-device", "p2p"]),
            "restart": bool(momo_raw.get("restart", False)),
            "state_path": str(momo_raw.get("state_path") or "/tmp/rov_camera_state.json"),
        },
        "items": items,
        "record": normalize_record_config(src.get("record")),
    }


def load_web_config(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"config root must be a mapping: {path}")
    http = raw.get("http") or {}
    topics = raw.get("topics") or {}
    command = raw.get("command") or {}
    lights_raw = raw.get("lights")
    if isinstance(lights_raw, list) and lights_raw:
        lights = [str(x) for x in lights_raw]
    else:
        lights = list(DEFAULT_LIGHTS)
    return {
        "http": {
            "host": str(http.get("host", "0.0.0.0")),
            "port": int(http.get("port", 8080)),
        },
        "topics": {
            "monitor": str(topics.get("monitor", "rov/monitor_value")),
            "imu_snapshot": str(topics.get("imu_snapshot", "rov/imu_nav/snapshot")),
            "cmd_vel": str(topics.get("cmd_vel", "turtle1/cmd_vel")),
            "hand_act": str(topics.get("hand_act", "turtle1/hand_act")),
            "lights": str(topics.get("lights", "turtle1/lights")),
        },
        "stale_sec": float(raw.get("stale_sec", 3.0)),
        "imu_min_interval_sec": float(raw.get("imu_min_interval_sec", 0.1)),
        "temperatures": list(raw.get("temperatures") or []),
        "temperature_gauge": dict(raw.get("temperature_gauge") or {"min": 0, "max": 80}),
        "leaks": list(raw.get("leaks") or []),
        "gauges": dict(raw.get("gauges") or {}),
        "lights": lights,
        "command": {
            "step_percent": float(command.get("step_percent", 5)),
            "echo_window_sec": float(command.get("echo_window_sec", 1.5)),
        },
        "cameras": _cameras(raw.get("cameras")),
    }


def public_config(cfg: dict[str, Any]) -> dict[str, Any]:
    return {
        "stale_sec": cfg["stale_sec"],
        "temperatures": cfg["temperatures"],
        "temperature_gauge": cfg["temperature_gauge"],
        "leaks": cfg["leaks"],
        "gauges": cfg["gauges"],
        "topics": cfg["topics"],
        "lights": cfg["lights"],
        "command": {
            "step_percent": cfg["command"]["step_percent"],
        },
        "cameras": {
            **public_camera_meta(
                cfg.get("cameras", {}).get("items") or [],
                str((cfg.get("cameras") or {}).get("backend") or "mediamtx"),
                webrtc=str(
                    ((cfg.get("cameras") or {}).get("mediamtx") or {}).get("webrtc")
                    or "http://127.0.0.1:8889"
                ),
            ),
            "record": recording_public_meta((cfg.get("cameras") or {}).get("record") or {}),
        },
    }
