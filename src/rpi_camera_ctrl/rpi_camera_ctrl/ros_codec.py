"""Convert CameraController dict snapshots to ROS messages."""

from __future__ import annotations

import json
from typing import Any, Mapping

from rpi_camera_interfaces.msg import (
    RpiCameraExposure,
    RpiCameraFocus,
    RpiCameraItem,
    RpiCameraRecordMeta,
    RpiCameraRecording,
    RpiCameraSnapshot,
    RpiCameraState,
    RpiCameraStorage,
    RpiCameraWb,
    RpiCameraZoomPreset,
)


def _f32(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _i32(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def focus_to_msg(data: Mapping[str, Any] | None) -> RpiCameraFocus:
    src = data if isinstance(data, Mapping) else {}
    msg = RpiCameraFocus()
    msg.mode = str(src.get("mode") or "")
    msg.window = str(src.get("window") or "")
    msg.lens_position = _f32(src.get("lens_position"))
    msg.range = str(src.get("range") or "")
    return msg


def exposure_to_msg(data: Mapping[str, Any] | None) -> RpiCameraExposure:
    src = data if isinstance(data, Mapping) else {}
    msg = RpiCameraExposure()
    msg.mode = str(src.get("mode") or "")
    msg.ae_mode = str(src.get("ae_mode") or "")
    msg.shutter_us = _i32(src.get("shutter_us"))
    msg.gain = _f32(src.get("gain"))
    msg.ev = _f32(src.get("ev"))
    msg.metering = str(src.get("metering") or "")
    msg.seed_shutter_us = _i32(src.get("seed_shutter_us"))
    msg.seed_gain = _f32(src.get("seed_gain"))
    return msg


def wb_to_msg(data: Mapping[str, Any] | None) -> RpiCameraWb:
    src = data if isinstance(data, Mapping) else {}
    msg = RpiCameraWb()
    msg.mode = str(src.get("mode") or "")
    gains = src.get("gains") or [0.0, 0.0]
    seed = src.get("seed_gains") or [0.0, 0.0]
    msg.gains = [_f32(gains[0] if len(gains) > 0 else 0.0), _f32(gains[1] if len(gains) > 1 else 0.0)]
    msg.seed_gains = [_f32(seed[0] if len(seed) > 0 else 0.0), _f32(seed[1] if len(seed) > 1 else 0.0)]
    return msg


def recording_to_msg(data: Mapping[str, Any] | None) -> RpiCameraRecording:
    src = data if isinstance(data, Mapping) else {}
    msg = RpiCameraRecording()
    msg.enabled = bool(src.get("enabled"))
    msg.active = bool(src.get("active"))
    return msg


def state_to_msg(data: Mapping[str, Any]) -> RpiCameraState:
    msg = RpiCameraState()
    msg.id = str(data.get("id") or "")
    msg.path = str(data.get("path") or "")
    msg.nickname = str(data.get("nickname") or "")
    msg.ready = bool(data.get("ready"))
    msg.error = str(data.get("error") or "")
    msg.zoom = str(data.get("zoom") or "")
    msg.sensor_mode = str(data.get("sensor_mode") or "")
    msg.width = _i32(data.get("width"))
    msg.height = _i32(data.get("height"))
    msg.fps = _f32(data.get("fps"))
    msg.focus = focus_to_msg(data.get("focus") if isinstance(data.get("focus"), Mapping) else {})
    msg.exposure = exposure_to_msg(
        data.get("exposure") if isinstance(data.get("exposure"), Mapping) else {}
    )
    msg.wb = wb_to_msg(data.get("wb") if isinstance(data.get("wb"), Mapping) else {})
    msg.recording = recording_to_msg(
        data.get("recording") if isinstance(data.get("recording"), Mapping) else {}
    )
    return msg


def snapshot_to_msg(snap: Mapping[str, Any]) -> RpiCameraSnapshot:
    msg = RpiCameraSnapshot()
    msg.backend = str(snap.get("backend") or "")
    msg.webrtc = str(snap.get("webrtc") or "")
    msg.error = str(snap.get("error") or "")
    msg.live = [str(x) for x in (snap.get("live") or [])]
    msg.restart = [str(x) for x in (snap.get("restart") or [])]
    msg.items = []
    for item in snap.get("items") or []:
        if not isinstance(item, Mapping):
            continue
        row = RpiCameraItem()
        row.id = str(item.get("id") or "")
        row.path = str(item.get("path") or "")
        row.nickname = str(item.get("nickname") or "")
        msg.items.append(row)
    msg.zoom = []
    for preset in snap.get("zoom") or []:
        if not isinstance(preset, Mapping):
            continue
        z = RpiCameraZoomPreset()
        z.id = str(preset.get("id") or "")
        z.label = str(preset.get("label") or "")
        z.hint = str(preset.get("hint") or "")
        z.width = _i32(preset.get("width"))
        z.height = _i32(preset.get("height"))
        z.fps = _f32(preset.get("fps"))
        z.sensor_mode = str(preset.get("sensor_mode") or "")
        z.restart = bool(preset.get("restart"))
        msg.zoom.append(z)
    msg.state = [
        state_to_msg(row) for row in (snap.get("state") or []) if isinstance(row, Mapping)
    ]
    storage = snap.get("storage") if isinstance(snap.get("storage"), Mapping) else {}
    msg.storage = RpiCameraStorage()
    msg.storage.path = str(storage.get("path") or "")
    msg.storage.free_bytes = int(storage.get("free_bytes") or 0)
    msg.storage.total_bytes = int(storage.get("total_bytes") or 0)
    msg.storage.min_free_bytes = int(storage.get("min_free_bytes") or 0)
    msg.storage.ok = bool(storage.get("ok", True))
    record = snap.get("record") if isinstance(snap.get("record"), Mapping) else {}
    msg.record = RpiCameraRecordMeta()
    msg.record.enabled = bool(record.get("enabled"))
    msg.record.dir = str(record.get("dir") or "")
    msg.record.segment = str(record.get("segment") or "")
    msg.record.part = str(record.get("part") or "")
    msg.record.min_free_bytes = int(record.get("min_free_bytes") or 0)
    msg.snapshot_json = json.dumps(snap, allow_nan=False)
    return msg


def snapshot_from_json(text: str) -> dict[str, Any]:
    data = json.loads(text or "{}")
    if not isinstance(data, dict):
        raise ValueError("snapshot_json must be an object")
    return data
