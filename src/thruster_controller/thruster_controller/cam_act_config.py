"""YAML-backed configuration for :class:`CamActControllerPT` / :class:`CamActControllerPTZ`."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Union

import yaml

__all__ = [
    "PanTiltAxisPTConfig",
    "PanTiltAxisPTZConfig",
    "LibcameraFocusConfig",
    "CamActPTConfig",
    "CamActPTZConfig",
]


def _req(m: Mapping[str, Any], key: str, ctx: str) -> Any:
    if key not in m:
        raise KeyError(f"{ctx}: missing required key {key!r}")
    return m[key]


def _axis_pt(raw: Mapping[str, Any], ctx: str) -> PanTiltAxisPTConfig:
    ch = int(_req(raw, "channel", ctx))
    if ch < 0 or ch > 15:
        raise ValueError(f"{ctx}: channel must be 0..15, got {ch}")
    sign_raw = raw.get("sign", 1)
    sign = int(sign_raw)
    if sign not in (-1, 1):
        raise ValueError(f"{ctx}: sign must be -1 or 1, got {sign_raw!r}")
    return PanTiltAxisPTConfig(
        center=float(_req(raw, "center", ctx)),
        min=float(_req(raw, "min", ctx)),
        max=float(_req(raw, "max", ctx)),
        gain=float(_req(raw, "gain", ctx)),
        channel=ch,
        sign=sign,
    )


def _axis_ptz(raw: Mapping[str, Any], ctx: str) -> PanTiltAxisPTZConfig:
    return PanTiltAxisPTZConfig(
        center=float(_req(raw, "center", ctx)),
        min=float(_req(raw, "min", ctx)),
        max=float(_req(raw, "max", ctx)),
    )


@dataclass
class PanTiltAxisPTConfig:
    center: float
    min: float
    max: float
    gain: float
    channel: int
    sign: int


@dataclass
class PanTiltAxisPTZConfig:
    center: float
    min: float
    max: float


@dataclass
class LibcameraFocusConfig:
    """Focus control for Raspberry Pi Camera v3 via libcamera/picamera2.

    - enabled: if false, focus controls are ignored.
    - af_mode: 'continuous' | 'auto' | 'manual' | 'off'
    - lens_position_*: manual focus range used when mapping `CameraActuator.focus` (0..1)
    """

    enabled: bool = False
    af_mode: str = "continuous"
    lens_position_min: float = 0.0
    lens_position_max: float = 10.0


@dataclass
class CamActPTConfig:
    pan: PanTiltAxisPTConfig
    tilt: PanTiltAxisPTConfig
    use_cam_act: bool
    focus: LibcameraFocusConfig | None = None

    @classmethod
    def load(cls, path: Union[str, Path]) -> CamActPTConfig:
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"cam_act PT config not found: {p}")
        with p.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if not isinstance(raw, dict):
            raise ValueError(f"Top-level YAML must be a mapping: {p}")
        pan = _axis_pt(_req(raw, "pan", str(p)), f"{p} pan")
        tilt = _axis_pt(_req(raw, "tilt", str(p)), f"{p} tilt")
        focus_raw = raw.get("focus")
        focus = None
        if isinstance(focus_raw, dict):
            focus = LibcameraFocusConfig(
                enabled=bool(focus_raw.get("enabled", False)),
                af_mode=str(focus_raw.get("af_mode", "continuous")).lower(),
                lens_position_min=float(focus_raw.get("lens_position_min", 0.0)),
                lens_position_max=float(focus_raw.get("lens_position_max", 10.0)),
            )
        return cls(
            pan=pan,
            tilt=tilt,
            use_cam_act=bool(_req(raw, "use_cam_act", str(p))),
            focus=focus,
        )


@dataclass
class CamActPTZConfig:
    pan: PanTiltAxisPTZConfig
    tilt: PanTiltAxisPTZConfig
    use_cam_act: bool
    focus_min: int = 1800
    zoom_min: int = 2400

    @classmethod
    def load(cls, path: Union[str, Path]) -> CamActPTZConfig:
        p = Path(path)
        if not p.is_file():
            raise FileNotFoundError(f"cam_act PTZ config not found: {p}")
        with p.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if not isinstance(raw, dict):
            raise ValueError(f"Top-level YAML must be a mapping: {p}")
        pan = _axis_ptz(_req(raw, "pan", str(p)), f"{p} pan")
        tilt = _axis_ptz(_req(raw, "tilt", str(p)), f"{p} tilt")
        return cls(
            pan=pan,
            tilt=tilt,
            use_cam_act=bool(_req(raw, "use_cam_act", str(p))),
            focus_min=int(raw.get("focus_min", 1800)),
            zoom_min=int(raw.get("zoom_min", 2400)),
        )
