from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from monitor_value_web.command import DEFAULT_LIGHTS

DEFAULT_CAMERAS = [
    {"id": "cam0", "path": "cam0", "nickname": "Ceiling"},
    {"id": "cam1", "path": "cam1", "nickname": "Canopy"},
]


def _camera_ros(raw: Any) -> dict[str, Any]:
    src = raw if isinstance(raw, dict) else {}
    return {
        "snapshot_topic": str(src.get("snapshot_topic") or "rpi_camera/snapshot"),
        "get_service": str(src.get("get_service") or "rpi_camera/get_cameras"),
        "set_service": str(src.get("set_service") or "rpi_camera/set_camera"),
    }


def _cameras_ui(raw: Any) -> dict[str, Any]:
    """UI fallback metadata until rpi_camera_ctrl publishes a snapshot."""
    src = raw if isinstance(raw, dict) else {}
    items_raw = src.get("items")
    items: list[dict[str, Any]] = []
    if isinstance(items_raw, list) and items_raw:
        for item in items_raw:
            if not isinstance(item, dict) or not item.get("id"):
                continue
            cam_id = str(item["id"])
            items.append(
                {
                    "id": cam_id,
                    "path": str(item.get("path") or cam_id),
                    "nickname": str(item.get("nickname") or cam_id),
                }
            )
    if not items:
        items = [dict(x) for x in DEFAULT_CAMERAS]
    webrtc = str(
        src.get("webrtc")
        or ((src.get("mediamtx") or {}) if isinstance(src.get("mediamtx"), dict) else {}).get(
            "webrtc"
        )
        or "http://127.0.0.1:8889"
    )
    return {
        "webrtc": webrtc.rstrip("/"),
        "items": items,
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
    camera_ros_raw = topics if isinstance(topics, dict) else {}
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
            "rpi_camera_snapshot": str(
                topics.get("rpi_camera_snapshot") or "rpi_camera/snapshot"
            ),
            "rpi_camera_get": str(topics.get("rpi_camera_get") or "rpi_camera/get_cameras"),
            "rpi_camera_set": str(topics.get("rpi_camera_set") or "rpi_camera/set_camera"),
        },
        "camera_ros": _camera_ros(
            {
                "snapshot_topic": camera_ros_raw.get("rpi_camera_snapshot"),
                "get_service": camera_ros_raw.get("rpi_camera_get"),
                "set_service": camera_ros_raw.get("rpi_camera_set"),
            }
        ),
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
        "cameras": _cameras_ui(raw.get("cameras")),
    }


def public_config(cfg: dict[str, Any], camera_meta: dict[str, Any] | None = None) -> dict[str, Any]:
    cameras = dict(camera_meta) if camera_meta else {}
    fallback = cfg.get("cameras") or {}
    if not cameras.get("items"):
        cameras.setdefault("items", list(fallback.get("items") or []))
    if not cameras.get("webrtc"):
        cameras["webrtc"] = str(fallback.get("webrtc") or "http://127.0.0.1:8889")
    cameras.setdefault("backend", "mediamtx")
    cameras.setdefault("zoom", [])
    cameras.setdefault("live", [])
    cameras.setdefault("restart", [])
    cameras.setdefault(
        "record",
        {"enabled": False, "dir": "", "segment": "", "part": "", "min_free_bytes": 0},
    )
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
        "cameras": cameras,
    }
