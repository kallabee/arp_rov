"""Streamer-agnostic Raspberry Pi Camera 3 controls.

High-level parameters (zoom / focus / exposure / WB) are independent of
MediaMTX or momo. Each backend maps them onto its own API:

- MediaMTX: WB and ME sliders are live. Zoom, AF, and AE on/off restart.
- momo: ``--libcamera-control`` at process start. Every change restarts.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional
from urllib.parse import urlparse


ZOOM_PRESETS: dict[str, dict[str, Any]] = {
    "binned": {
        "id": "binned",
        "label": "1x Wide",
        "hint": "Full field of view, 2x2 binning, 1920x1080.",
        "sensor_mode": "2304:1296",
        "width": 1920,
        "height": 1080,
        "fps": 20.0,
    },
    "full": {
        "id": "full",
        "label": "1x Full res",
        "hint": "Same full field of view as Wide, unbinned sensor. ~14 fps.",
        "sensor_mode": "4608:2592",
        "width": 2304,
        "height": 1296,
        "fps": 14.0,
    },
    "crop": {
        "id": "crop",
        "label": "1.5x Crop",
        "hint": "Hardware center crop (~1.5x). Narrower field of view, no upscale.",
        "sensor_mode": "1536:864",
        "width": 1536,
        "height": 864,
        "fps": 30.0,
    },
}

PIXEL_ARRAY = (4608, 2592)
FOCUS_MODES = ("continuous", "auto", "manual")
WB_AUTO_MODES = (
    "auto",
    "incandescent",
    "tungsten",
    "fluorescent",
    "indoor",
    "daylight",
    "cloudy",
)
EXPOSURE_AE_MODES = ("normal", "short", "long")
AWB_LIBCAMERA = {
    "auto": "Auto",
    "incandescent": "Incandescent",
    "tungsten": "Tungsten",
    "fluorescent": "Fluorescent",
    "indoor": "Indoor",
    "daylight": "Daylight",
    "cloudy": "Cloudy",
}
AF_LIBCAMERA = {"continuous": "Continuous", "auto": "Auto", "manual": "Manual"}
METERING_LIBCAMERA = {"centre": "CentreWeighted", "spot": "Spot", "matrix": "Matrix"}
AE_LIBCAMERA = {"normal": "Normal", "short": "Short", "long": "Long"}

LIVE_GROUPS_BY_BACKEND = {
    "mediamtx": frozenset({"exposure", "wb"}),
    "momo": frozenset(),
}
RESTART_GROUPS_BY_BACKEND = {
    "mediamtx": frozenset({"zoom", "focus"}),
    "momo": frozenset({"zoom", "focus", "exposure", "wb"}),
}


class CameraError(ValueError):
    """Invalid request or streamer communication failure."""


def default_state() -> dict[str, Any]:
    preset = ZOOM_PRESETS["binned"]
    return {
        "zoom": "binned",
        "sensor_mode": preset["sensor_mode"],
        "width": preset["width"],
        "height": preset["height"],
        "fps": preset["fps"],
        "focus": {
            "mode": "continuous",
            "window": "0,0,1,1",
            "lens_position": 0.0,
            "range": "normal",
        },
        "exposure": {
            "mode": "ae",
            "ae_mode": "normal",
            "shutter_us": 0,
            "gain": 0.0,
            "ev": 0.0,
            "metering": "centre",
        },
        "wb": {"mode": "auto", "gains": [0.0, 0.0]},
    }


def zoom_id_from_mode(mode: str) -> str:
    if str(mode).startswith("4608"):
        return "full"
    if str(mode).startswith("1536"):
        return "crop"
    return "binned"


def zoom_id_from_conf(conf: Mapping[str, Any]) -> str:
    """Backward-compatible helper for MediaMTX path config."""
    return zoom_id_from_mode(str(conf.get("rpiCameraMode") or conf.get("sensor_mode") or ""))


def format_af_window(rect: Any) -> str:
    if rect is None or rect == "":
        return ""
    if isinstance(rect, str):
        parts = [p.strip() for p in rect.split(",")]
        if len(parts) != 4:
            raise CameraError("af window must be x,y,w,h")
        vals = [float(p) for p in parts]
    elif isinstance(rect, (list, tuple)) and len(rect) == 4:
        vals = [float(x) for x in rect]
    else:
        raise CameraError("af window must be x,y,w,h")
    if any(v < 0.0 or v > 1.0 for v in vals):
        raise CameraError("af window values must be in 0..1")
    if vals[2] <= 0.0 or vals[3] <= 0.0:
        raise CameraError("af window width/height must be > 0")
    if vals[0] + vals[2] > 1.0001 or vals[1] + vals[3] > 1.0001:
        raise CameraError("af window must stay inside the frame")
    return ",".join(f"{v:.4f}".rstrip("0").rstrip(".") for v in vals)


def _clamp(value: Any, lo: float, hi: float, name: str) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise CameraError(f"{name} must be a number") from exc
    if v < lo or v > hi:
        raise CameraError(f"{name} must be in {lo}..{hi}")
    return v


def parse_request(request: Mapping[str, Any]) -> tuple[dict[str, Any], frozenset[str]]:
    """Parse a high-level request into a canonical partial state."""
    if not isinstance(request, Mapping):
        raise CameraError("json object required")
    partial: dict[str, Any] = {}
    groups: set[str] = set()

    if "zoom" in request and request["zoom"] is not None:
        zoom = str(request["zoom"]).strip()
        if zoom not in ZOOM_PRESETS:
            raise CameraError(f"zoom must be one of: {', '.join(ZOOM_PRESETS)}")
        preset = ZOOM_PRESETS[zoom]
        partial.update(
            {
                "zoom": zoom,
                "sensor_mode": preset["sensor_mode"],
                "width": preset["width"],
                "height": preset["height"],
                "fps": preset["fps"],
            }
        )
        groups.add("zoom")

    if "focus" in request and request["focus"] is not None:
        focus_in = request["focus"]
        if not isinstance(focus_in, Mapping):
            raise CameraError("focus must be an object")
        focus: dict[str, Any] = {}
        if "mode" in focus_in and focus_in["mode"] is not None:
            mode = str(focus_in["mode"]).strip().lower()
            if mode not in FOCUS_MODES:
                raise CameraError("focus.mode must be continuous, auto, or manual")
            focus["mode"] = mode
        if "window" in focus_in:
            focus["window"] = format_af_window(focus_in["window"]) or "0,0,1,1"
        if "lens_position" in focus_in and focus_in["lens_position"] is not None:
            focus["lens_position"] = _clamp(focus_in["lens_position"], 0.0, 15.0, "lens_position")
        if "range" in focus_in and focus_in["range"] is not None:
            rng = str(focus_in["range"]).strip().lower()
            if rng not in ("normal", "macro", "full"):
                raise CameraError("focus.range must be normal, macro, or full")
            focus["range"] = rng
        if not focus:
            raise CameraError("focus has no fields")
        partial["focus"] = focus
        groups.add("focus")

    if "exposure" in request and request["exposure"] is not None:
        exposure_in = request["exposure"]
        if not isinstance(exposure_in, Mapping):
            raise CameraError("exposure must be an object")
        exposure: dict[str, Any] = {}
        mode = str(exposure_in.get("mode") or "").strip().lower()
        if mode in ("ae", "auto"):
            exposure["mode"] = "ae"
            exposure["shutter_us"] = 0
            exposure["gain"] = 0.0
            ae_mode = str(exposure_in.get("ae_mode") or "normal").strip().lower()
            if ae_mode not in EXPOSURE_AE_MODES:
                raise CameraError("exposure.ae_mode must be normal, short, or long")
            exposure["ae_mode"] = ae_mode
        elif mode in ("me", "manual"):
            exposure["mode"] = "me"
            if "shutter_us" in exposure_in and exposure_in["shutter_us"] is not None:
                exposure["shutter_us"] = int(
                    _clamp(exposure_in["shutter_us"], 1, 200_000, "shutter_us")
                )
            if "gain" in exposure_in and exposure_in["gain"] is not None:
                exposure["gain"] = _clamp(exposure_in["gain"], 0.1, 16.0, "gain")
            # shutter/gain may be omitted — MediaMTX backend seeds from last Auto values.
        elif mode:
            raise CameraError("exposure.mode must be ae or me")
        if "ev" in exposure_in and exposure_in["ev"] is not None:
            exposure["ev"] = _clamp(exposure_in["ev"], -10.0, 10.0, "ev")
        if "metering" in exposure_in and exposure_in["metering"] is not None:
            metering = str(exposure_in["metering"]).strip().lower()
            if metering not in ("centre", "spot", "matrix"):
                raise CameraError("metering must be centre, spot, or matrix")
            exposure["metering"] = metering
        if not exposure:
            raise CameraError("exposure has no fields")
        partial["exposure"] = exposure
        groups.add("exposure")

    if "wb" in request and request["wb"] is not None:
        wb_in = request["wb"]
        if not isinstance(wb_in, Mapping):
            raise CameraError("wb must be an object")
        mode = str(wb_in.get("mode") or "").strip().lower()
        wb: dict[str, Any] = {}
        if mode in WB_AUTO_MODES:
            wb["mode"] = mode
            if mode != "custom":
                wb["gains"] = [0.0, 0.0]
        elif mode in ("manual", "custom"):
            gains = wb_in.get("gains")
            if gains is None:
                # Omit gains — backend seeds from last Auto values.
                wb["mode"] = "manual"
            elif not isinstance(gains, (list, tuple)) or len(gains) != 2:
                raise CameraError("manual wb needs gains: [red, blue]")
            else:
                wb["mode"] = "manual"
                wb["gains"] = [
                    _clamp(gains[0], 0.0, 16.0, "wb.gains[0]"),
                    _clamp(gains[1], 0.0, 16.0, "wb.gains[1]"),
                ]
        elif mode:
            raise CameraError("wb.mode must be auto or manual")
        else:
            raise CameraError("wb.mode required")
        partial["wb"] = wb
        groups.add("wb")

    if not groups:
        raise CameraError("no camera parameters to change")
    return partial, frozenset(groups)


def merge_state(base: Mapping[str, Any], partial: Mapping[str, Any]) -> dict[str, Any]:
    out = default_state()
    out.update({k: v for k, v in dict(base).items() if k in out and not isinstance(out[k], dict)})
    for key in ("focus", "exposure", "wb"):
        merged = dict(out[key])
        merged.update(dict(base.get(key) or {}))
        merged.update(dict(partial.get(key) or {}))
        out[key] = merged
    for key in ("zoom", "sensor_mode", "width", "height", "fps"):
        if key in partial:
            out[key] = partial[key]
    if out["zoom"] in ZOOM_PRESETS and "width" not in partial:
        preset = ZOOM_PRESETS[str(out["zoom"])]
        out["sensor_mode"] = preset["sensor_mode"]
        out["width"] = preset["width"]
        out["height"] = preset["height"]
        out["fps"] = preset["fps"]
    return out


def to_mediamtx_patch(partial: Mapping[str, Any]) -> dict[str, Any]:
    patch: dict[str, Any] = {}
    if "sensor_mode" in partial:
        patch["rpiCameraMode"] = partial["sensor_mode"]
    if "width" in partial:
        patch["rpiCameraWidth"] = int(partial["width"])
    if "height" in partial:
        patch["rpiCameraHeight"] = int(partial["height"])
    if "fps" in partial:
        patch["rpiCameraFPS"] = float(partial["fps"])
    focus = partial.get("focus") or {}
    if "mode" in focus:
        patch["rpiCameraAfMode"] = focus["mode"]
    if "window" in focus:
        patch["rpiCameraAfWindow"] = focus["window"]
    if "lens_position" in focus:
        patch["rpiCameraLensPosition"] = float(focus["lens_position"])
    if "range" in focus:
        patch["rpiCameraAfRange"] = focus["range"]
    exposure = partial.get("exposure") or {}
    if "mode" in exposure:
        if exposure["mode"] == "ae":
            patch["rpiCameraShutter"] = 0
            patch["rpiCameraGain"] = 0
            if "ae_mode" in exposure:
                patch["rpiCameraExposure"] = exposure["ae_mode"]
        elif exposure["mode"] == "me":
            if "shutter_us" in exposure:
                patch["rpiCameraShutter"] = int(exposure["shutter_us"])
            if "gain" in exposure:
                patch["rpiCameraGain"] = float(exposure["gain"])
    if "ev" in exposure:
        patch["rpiCameraEV"] = float(exposure["ev"])
    if "metering" in exposure:
        patch["rpiCameraMetering"] = exposure["metering"]
    wb = partial.get("wb") or {}
    if "mode" in wb:
        if wb["mode"] == "manual":
            patch["rpiCameraAWB"] = "custom"
            patch["rpiCameraAWBGains"] = list(wb.get("gains") or [0.0, 0.0])
        else:
            patch["rpiCameraAWB"] = wb["mode"]
            patch["rpiCameraAWBGains"] = [0.0, 0.0]
    return patch


def from_mediamtx_conf(conf: Mapping[str, Any]) -> dict[str, Any]:
    shutter = int(conf.get("rpiCameraShutter") or 0)
    gain = float(conf.get("rpiCameraGain") or 0.0)
    awb = str(conf.get("rpiCameraAWB") or "auto")
    gains = conf.get("rpiCameraAWBGains") or [0, 0]
    if not isinstance(gains, list) or len(gains) != 2:
        gains = [0, 0]
    zoom = zoom_id_from_mode(str(conf.get("rpiCameraMode") or ""))
    preset = ZOOM_PRESETS[zoom]
    return {
        "zoom": zoom,
        "sensor_mode": str(conf.get("rpiCameraMode") or preset["sensor_mode"]),
        "width": int(conf.get("rpiCameraWidth") or preset["width"]),
        "height": int(conf.get("rpiCameraHeight") or preset["height"]),
        "fps": float(conf.get("rpiCameraFPS") or preset["fps"]),
        "focus": {
            "mode": str(conf.get("rpiCameraAfMode") or "continuous"),
            "window": str(conf.get("rpiCameraAfWindow") or ""),
            "lens_position": float(conf.get("rpiCameraLensPosition") or 0.0),
            "range": str(conf.get("rpiCameraAfRange") or "normal"),
        },
        "exposure": {
            "mode": "me" if shutter > 0 or gain > 0 else "ae",
            "ae_mode": str(conf.get("rpiCameraExposure") or "normal"),
            "shutter_us": shutter,
            "gain": gain,
            "ev": float(conf.get("rpiCameraEV") or 0.0),
            "metering": str(conf.get("rpiCameraMetering") or "centre"),
        },
        "wb": {
            "mode": "manual" if awb == "custom" else awb,
            "gains": [float(gains[0]), float(gains[1])],
        },
    }


def summarize_conf(conf: Mapping[str, Any]) -> dict[str, Any]:
    if any(str(k).startswith("rpiCamera") for k in conf):
        return from_mediamtx_conf(conf)
    return merge_state(default_state(), conf)


def _af_window_pixels(window: str) -> str:
    parts = [float(x) for x in (window or "0,0,1,1").split(",")]
    if len(parts) != 4:
        parts = [0.0, 0.0, 1.0, 1.0]
    pw, ph = PIXEL_ARRAY
    return ",".join(
        str(int(round(v * (pw if i % 2 == 0 else ph))))
        for i, v in enumerate(parts)
    )


def to_libcamera_controls(state: Mapping[str, Any]) -> list[tuple[str, str]]:
    """Map canonical state to momo ``--libcamera-control`` pairs."""
    controls: list[tuple[str, str]] = []
    focus = state.get("focus") or {}
    af_mode = str(focus.get("mode") or "continuous")
    controls.append(("AfMode", AF_LIBCAMERA.get(af_mode, "Continuous")))
    controls.append(("AfRange", str(focus.get("range") or "normal").capitalize()))
    window = str(focus.get("window") or "")
    if window and window not in ("0,0,1,1", "0,0,1,0"):
        controls.append(("AfMetering", "Windows"))
        controls.append(("AfWindows", _af_window_pixels(window)))
    if af_mode == "manual":
        controls.append(("LensPosition", str(float(focus.get("lens_position") or 0.0))))
    elif af_mode == "auto":
        controls.append(("AfTrigger", "Start"))

    exposure = state.get("exposure") or {}
    if str(exposure.get("mode") or "ae") == "me":
        controls.append(("ExposureTimeMode", "Manual"))
        controls.append(("AnalogueGainMode", "Manual"))
        if int(exposure.get("shutter_us") or 0) > 0:
            controls.append(("ExposureTime", str(int(exposure["shutter_us"]))))
        if float(exposure.get("gain") or 0) > 0:
            controls.append(("AnalogueGain", str(float(exposure["gain"]))))
    else:
        controls.append(("ExposureTimeMode", "Auto"))
        controls.append(("AnalogueGainMode", "Auto"))
        ae_mode = str(exposure.get("ae_mode") or "normal")
        if ae_mode in AE_LIBCAMERA:
            controls.append(("AeExposureMode", AE_LIBCAMERA[ae_mode]))
    controls.append(("ExposureValue", str(float(exposure.get("ev") or 0.0))))
    metering = str(exposure.get("metering") or "centre")
    controls.append(("AeMeteringMode", METERING_LIBCAMERA.get(metering, "CentreWeighted")))

    wb = state.get("wb") or {}
    wb_mode = str(wb.get("mode") or "auto")
    if wb_mode == "manual":
        controls.append(("AwbEnable", "0"))
        gains = wb.get("gains") or [1.0, 1.0]
        controls.append(("ColourGains", f"{float(gains[0])},{float(gains[1])}"))
    else:
        controls.append(("AwbEnable", "1"))
        controls.append(("AwbMode", AWB_LIBCAMERA.get(wb_mode, "Auto")))

    fps = float(state.get("fps") or 20.0)
    if fps > 0:
        frame_us = str(int(round(1_000_000.0 / fps)))
        controls.append(("FrameDurationLimits", f"{frame_us},{frame_us}"))
    return controls


def momo_argv(state: Mapping[str, Any], *, index: int, extra_args: Iterable[str]) -> list[str]:
    args = [
        "--use-libcamera",
        "--video-input-device",
        str(index),
        "--resolution",
        f"{int(state.get('width') or 1920)}x{int(state.get('height') or 1080)}",
        "--framerate",
        str(int(round(float(state.get("fps") or 20.0)))),
        "--fixed-resolution",
    ]
    for key, value in to_libcamera_controls(state):
        args.extend(["--libcamera-control", key, value])
    args.extend(str(x) for x in extra_args)
    return args


def public_camera_meta(
    cameras: Iterable[Mapping[str, Any]],
    backend: str = "mediamtx",
    *,
    webrtc: str = "http://127.0.0.1:8889",
) -> dict[str, Any]:
    items = []
    for cam in cameras:
        items.append(
            {
                "id": cam["id"],
                "path": cam.get("path") or cam["id"],
                "nickname": cam.get("nickname") or cam["id"],
            }
        )
    live = sorted(LIVE_GROUPS_BY_BACKEND.get(backend, frozenset()))
    restart = sorted(RESTART_GROUPS_BY_BACKEND.get(backend, frozenset({"zoom", "focus"})))
    return {
        "backend": backend,
        "webrtc": str(webrtc or "http://127.0.0.1:8889").rstrip("/"),
        "items": items,
        "zoom": [
            {
                "id": p["id"],
                "label": p["label"],
                "hint": p["hint"],
                "width": p["width"],
                "height": p["height"],
                "fps": p["fps"],
                "sensor_mode": p["sensor_mode"],
                "restart": "zoom" in restart,
            }
            for p in ZOOM_PRESETS.values()
        ],
        "live": live,
        "restart": restart,
    }


def build_patch(request: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
    """MediaMTX-oriented helper kept for tests and CLI mapping."""
    partial, groups = parse_request(request)
    patch, restart = mediamtx_runtime_patch(partial, groups, {})
    return patch, restart


def _toggle_ondemand_timeout(value: Any) -> str:
    raw = str(value or "10s").strip().lower()
    return "11s" if raw.startswith("10") else "10s"


def _force_path_recreate(patch: dict[str, Any], current_conf: Mapping[str, Any]) -> None:
    """Mutate patch so MediaMTX cannot hot-reload (must recreate rpicamera)."""
    patch["sourceOnDemandStartTimeout"] = _toggle_ondemand_timeout(
        current_conf.get("sourceOnDemandStartTimeout")
    )
    if "rpiCameraMode" not in patch:
        mode = str(current_conf.get("rpiCameraMode") or "").strip()
        if not mode:
            mode = str(from_mediamtx_conf(current_conf).get("sensor_mode") or "")
        if mode:
            patch["rpiCameraMode"] = mode
    # AfWindow is not live-reloadable; flip between equivalent full-frame values.
    if "rpiCameraAfWindow" not in patch:
        cur = str(current_conf.get("rpiCameraAfWindow") or "").strip()
        patch["rpiCameraAfWindow"] = "" if cur else "0,0,1,1"


def mediamtx_runtime_patch(
    partial: Mapping[str, Any],
    groups: frozenset[str],
    current_conf: Mapping[str, Any],
) -> tuple[dict[str, Any], bool]:
    """Build a MediaMTX PATCH and whether the rpicamera process must restart.

    Shutter/gain 0 is a live reload, but libcamera keeps the last manual
    exposure. Switching back to AE therefore forces a path recreate.
    EV-only tweaks while already in AE stay live.
    """
    patch = to_mediamtx_patch(partial)
    restart = bool(groups & RESTART_GROUPS_BY_BACKEND["mediamtx"])
    exposure = partial.get("exposure") or {}
    if exposure.get("mode") == "ae":
        patch["rpiCameraShutter"] = 0
        patch["rpiCameraGain"] = 0.0
        if "rpiCameraExposure" not in patch:
            patch["rpiCameraExposure"] = str(
                exposure.get("ae_mode")
                or current_conf.get("rpiCameraExposure")
                or "normal"
            )
        was_manual = (
            int(current_conf.get("rpiCameraShutter") or 0) > 0
            or float(current_conf.get("rpiCameraGain") or 0) > 0
        )
        # AE button (no EV) always recreates; EV slider while AE stays live unless leaving ME.
        ae_button = "ev" not in exposure
        if was_manual or ae_button:
            restart = True
            _force_path_recreate(patch, current_conf)
    return patch, restart


class MediaMtxClient:
    def __init__(self, base_url: str, timeout_sec: float = 20.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = float(timeout_sec)

    def _request(self, method: str, path: str, body: Optional[dict] = None) -> Any:
        url = self.base_url + path
        data = None
        headers = {"Accept": "application/json"}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                raw = resp.read()
                if not raw:
                    return None
                return json.loads(raw.decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise CameraError(f"MediaMTX HTTP {exc.code}: {detail[:300]}") from exc
        except urllib.error.URLError as exc:
            raise CameraError(f"Cannot reach MediaMTX ({self.base_url}): {exc.reason}") from exc
        except TimeoutError as exc:
            raise CameraError("MediaMTX request timed out") from exc
        except json.JSONDecodeError as exc:
            raise CameraError("MediaMTX returned invalid JSON") from exc

    def get_path_conf(self, path: str) -> dict[str, Any]:
        payload = self._request("GET", f"/v3/config/paths/get/{path}")
        if not isinstance(payload, dict):
            raise CameraError(f"unexpected config for path {path}")
        return payload

    def patch_path_conf(self, path: str, patch: Mapping[str, Any]) -> dict[str, Any]:
        self._request("PATCH", f"/v3/config/paths/patch/{path}", dict(patch))
        return self.get_path_conf(path)

    def get_path_runtime(self, path: str) -> dict[str, Any]:
        try:
            payload = self._request("GET", f"/v3/paths/get/{path}")
        except CameraError:
            return {"ready": False}
        if not isinstance(payload, dict):
            return {"ready": False}
        return payload


class _StateStore:
    def __init__(self, path: Path):
        self.path = path

    def load(self) -> dict[str, Any]:
        if not self.path.is_file():
            return {}
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return raw if isinstance(raw, dict) else {}

    def save(self, data: Mapping[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
        tmp.replace(self.path)


class MediaMtxBackend:
    name = "mediamtx"

    def __init__(self, cfg: Mapping[str, Any]):
        cameras = cfg.get("cameras") or {}
        mtx = cameras.get("mediamtx") if isinstance(cameras.get("mediamtx"), dict) else {}
        api = str(mtx.get("api") or cameras.get("api") or "http://127.0.0.1:9997")
        self.client = MediaMtxClient(api)
        self.api_url = api
        host = urlparse(api).hostname or "127.0.0.1"
        self.hls_url = str(mtx.get("hls") or f"http://{host}:8888").rstrip("/")
        # Last non-zero shutter/gain/WB gains per path — used to seed Auto→Manual.
        self._seeds: dict[str, dict[str, Any]] = {}

    @property
    def live_groups(self) -> frozenset[str]:
        return LIVE_GROUPS_BY_BACKEND["mediamtx"]

    @property
    def restart_groups(self) -> frozenset[str]:
        return RESTART_GROUPS_BY_BACKEND["mediamtx"]

    def _seed_for(self, path: str) -> dict[str, Any]:
        return self._seeds.setdefault(
            path,
            {"shutter_us": 8000, "gain": 2.0, "wb_gains": [2.0, 1.5]},
        )

    def _remember_seeds(self, path: str, state: Mapping[str, Any]) -> None:
        seed = self._seed_for(path)
        exp = state.get("exposure") or {}
        shutter = int(exp.get("shutter_us") or 0)
        gain = float(exp.get("gain") or 0.0)
        if shutter > 0:
            seed["shutter_us"] = shutter
        if gain > 0:
            seed["gain"] = gain
        wb = state.get("wb") or {}
        gains = wb.get("gains") or [0.0, 0.0]
        if (
            isinstance(gains, (list, tuple))
            and len(gains) == 2
            and float(gains[0]) > 0
            and float(gains[1]) > 0
        ):
            seed["wb_gains"] = [float(gains[0]), float(gains[1])]

    def _attach_seeds(self, path: str, state: dict[str, Any]) -> dict[str, Any]:
        seed = self._seed_for(path)
        exposure = dict(state.get("exposure") or {})
        exposure["seed_shutter_us"] = int(seed["shutter_us"])
        exposure["seed_gain"] = float(seed["gain"])
        # While Auto, surface last known values so Manual can lock them.
        if exposure.get("mode") == "ae":
            exposure["shutter_us"] = int(seed["shutter_us"])
            exposure["gain"] = float(seed["gain"])
        wb = dict(state.get("wb") or {})
        wb["seed_gains"] = [float(seed["wb_gains"][0]), float(seed["wb_gains"][1])]
        if wb.get("mode") != "manual":
            wb["gains"] = [float(seed["wb_gains"][0]), float(seed["wb_gains"][1])]
        state["exposure"] = exposure
        state["wb"] = wb
        return state

    def _seed_partial_from_auto(
        self, path: str, partial: Mapping[str, Any], current_conf: Mapping[str, Any]
    ) -> dict[str, Any]:
        """Fill Manual shutter/gain/WB gains from last Auto-era seeds when missing."""
        out = dict(partial)
        seed = self._seed_for(path)
        # Refresh seeds from whatever is still non-zero on the live conf.
        self._remember_seeds(path, from_mediamtx_conf(current_conf))

        exposure = dict(out.get("exposure") or {})
        if exposure.get("mode") == "me":
            if int(exposure.get("shutter_us") or 0) <= 0:
                exposure["shutter_us"] = int(seed["shutter_us"])
            if float(exposure.get("gain") or 0) <= 0:
                exposure["gain"] = float(seed["gain"])
            out["exposure"] = exposure

        wb = dict(out.get("wb") or {})
        if wb.get("mode") == "manual":
            gains = wb.get("gains") or [0.0, 0.0]
            if not (
                isinstance(gains, (list, tuple))
                and len(gains) == 2
                and float(gains[0]) > 0
                and float(gains[1]) > 0
            ):
                wb["gains"] = [float(seed["wb_gains"][0]), float(seed["wb_gains"][1])]
            out["wb"] = wb
        return out

    def read(self, item: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
        path = str(item.get("path") or item["id"])
        conf = self.client.get_path_conf(path)
        runtime = self.client.get_path_runtime(path)
        state = from_mediamtx_conf(conf)
        self._remember_seeds(path, state)
        return self._attach_seeds(path, state), bool(runtime.get("ready"))

    def _kick_ondemand(self, path: str) -> None:
        """Start an on-demand rpicamera after the path was recreated."""
        url = f"{self.hls_url}/{path}/index.m3u8"
        try:
            urllib.request.urlopen(url, timeout=8)
        except Exception:
            pass
        for _ in range(8):
            if self.client.get_path_runtime(path).get("ready"):
                return
            time.sleep(0.25)

    def apply(
        self, item: Mapping[str, Any], partial: Mapping[str, Any], groups: frozenset[str]
    ) -> dict[str, Any]:
        path = str(item.get("path") or item["id"])
        current_conf = self.client.get_path_conf(path)
        partial = self._seed_partial_from_auto(path, partial, current_conf)
        patch, restart = mediamtx_runtime_patch(partial, groups, current_conf)
        conf = self.client.patch_path_conf(path, patch)
        if restart:
            self._kick_ondemand(path)
        runtime = self.client.get_path_runtime(path)
        state = from_mediamtx_conf(conf)
        self._remember_seeds(path, state)
        state = self._attach_seeds(path, state)
        return {
            "restart": restart,
            "patch": patch,
            "ready": bool(runtime.get("ready")),
            "warning": None,
            **state,
        }


class MomoBackend:
    name = "momo"

    def __init__(self, cfg: Mapping[str, Any]):
        cameras = cfg.get("cameras") or {}
        momo = cameras.get("momo") if isinstance(cameras.get("momo"), dict) else {}
        self.binary = str(momo.get("binary") or "momo")
        extra = momo.get("extra_args") or ["--no-audio-device", "p2p"]
        self.extra_args = [str(x) for x in extra] if isinstance(extra, list) else ["--no-audio-device", "p2p"]
        self.do_restart = bool(momo.get("restart", False))
        state_path = Path(str(momo.get("state_path") or "/tmp/rov_camera_state.json"))
        self.store = _StateStore(state_path)

    @property
    def live_groups(self) -> frozenset[str]:
        return LIVE_GROUPS_BY_BACKEND["momo"]

    @property
    def restart_groups(self) -> frozenset[str]:
        return RESTART_GROUPS_BY_BACKEND["momo"]

    def _index(self, item: Mapping[str, Any]) -> int:
        if "index" in item:
            return int(item["index"])
        cam_id = str(item.get("id") or "0")
        digits = "".join(ch for ch in cam_id if ch.isdigit())
        return int(digits) if digits else 0

    def read(self, item: Mapping[str, Any]) -> tuple[dict[str, Any], bool]:
        blob = self.store.load()
        rec = blob.get(str(item["id"])) or {}
        state = merge_state(default_state(), rec.get("state") or {})
        pid = rec.get("pid")
        ready = isinstance(pid, int) and _pid_alive(pid)
        return state, ready

    def apply(
        self, item: Mapping[str, Any], partial: Mapping[str, Any], groups: frozenset[str]
    ) -> dict[str, Any]:
        cam_id = str(item["id"])
        blob = self.store.load()
        rec = blob.get(cam_id) or {}
        state = merge_state(rec.get("state") or default_state(), partial)
        argv = [self.binary, *momo_argv(state, index=self._index(item), extra_args=self.extra_args)]
        warning = None
        pid = rec.get("pid")
        if self.do_restart:
            _stop_pid(pid)
            proc = subprocess.Popen(argv, start_new_session=True)
            pid = proc.pid
        elif not (isinstance(pid, int) and _pid_alive(pid)):
            warning = "momo is start-option only. Set restart: true or relaunch with the returned argv"
            pid = None
        blob[cam_id] = {"state": state, "pid": pid, "argv": argv}
        self.store.save(blob)
        return {
            "restart": True,
            "ready": isinstance(pid, int) and _pid_alive(pid),
            "warning": warning,
            "argv": argv,
            **state,
        }


def _pid_alive(pid: Any) -> bool:
    if not isinstance(pid, int) or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _stop_pid(pid: Any) -> None:
    if not _pid_alive(pid):
        return
    os.kill(int(pid), signal.SIGINT)
    for _ in range(40):
        if not _pid_alive(pid):
            return
        time.sleep(0.1)
    os.kill(int(pid), signal.SIGKILL)


def make_backend(cfg: Mapping[str, Any]):
    cameras = cfg.get("cameras") or {}
    name = str(cameras.get("backend") or "mediamtx").strip().lower()
    if name == "momo":
        return MomoBackend(cfg)
    if name in ("mediamtx", "mtx", "rpicamera"):
        return MediaMtxBackend(cfg)
    raise CameraError(f"unknown camera backend: {name}")


class CameraController:
    def __init__(self, cfg: Mapping[str, Any]):
        cameras = cfg.get("cameras") or {}
        self.items = list(cameras.get("items") or [])
        self.backend = make_backend(cfg)
        self._by_id = {str(item["id"]): item for item in self.items if "id" in item}
        mtx = cameras.get("mediamtx") if isinstance(cameras.get("mediamtx"), dict) else {}
        self.webrtc = str(mtx.get("webrtc") or "http://127.0.0.1:8889")

    def meta(self) -> dict[str, Any]:
        return public_camera_meta(self.items, self.backend.name, webrtc=self.webrtc)

    def snapshot(self) -> dict[str, Any]:
        out = dict(self.meta())
        states = []
        error = None
        for item in self.items:
            cam_id = str(item["id"])
            try:
                state, ready = self.backend.read(item)
                states.append(
                    {
                        "id": cam_id,
                        "path": item.get("path") or cam_id,
                        "nickname": item.get("nickname") or cam_id,
                        "ready": ready,
                        **state,
                    }
                )
            except CameraError as exc:
                error = str(exc)
                states.append(
                    {
                        "id": cam_id,
                        "path": item.get("path") or cam_id,
                        "nickname": item.get("nickname") or cam_id,
                        "ready": False,
                        "error": str(exc),
                    }
                )
        out["state"] = states
        out["error"] = error
        out["backend"] = self.backend.name
        return out

    def apply(self, cam_id: str, request: Mapping[str, Any]) -> dict[str, Any]:
        item = self._by_id.get(cam_id)
        if item is None:
            raise CameraError(f"unknown camera: {cam_id}")
        partial, groups = parse_request(request)
        result = self.backend.apply(item, partial, groups)
        return {
            "id": cam_id,
            "path": item.get("path") or cam_id,
            "nickname": item.get("nickname") or cam_id,
            **result,
        }
