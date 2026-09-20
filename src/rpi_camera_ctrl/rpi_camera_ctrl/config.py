from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from rpi_camera_ctrl.camera import public_camera_meta
from rpi_camera_ctrl.record import normalize_record_config, recording_public_meta

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


def load_camera_config(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"config root must be a mapping: {path}")
    ros = raw.get("ros") if isinstance(raw.get("ros"), dict) else {}
    cameras_raw = raw.get("cameras") if "cameras" in raw else raw
    return {
        "ros": {
            "snapshot_topic": str(ros.get("snapshot_topic") or "rpi_camera/snapshot"),
            "get_service": str(ros.get("get_service") or "rpi_camera/get_cameras"),
            "set_service": str(ros.get("set_service") or "rpi_camera/set_camera"),
            "publish_hz": float(ros.get("publish_hz") or 1.0),
        },
        "cameras": _cameras(cameras_raw),
    }


def public_camera_config(cfg: dict[str, Any]) -> dict[str, Any]:
    cameras = cfg.get("cameras") or {}
    return {
        **public_camera_meta(
            cameras.get("items") or [],
            str(cameras.get("backend") or "mediamtx"),
            webrtc=str((cameras.get("mediamtx") or {}).get("webrtc") or "http://127.0.0.1:8889"),
        ),
        "record": recording_public_meta(cameras.get("record") or {}),
    }
