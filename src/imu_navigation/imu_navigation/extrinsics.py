from __future__ import annotations

import math
from typing import Any, Dict, Tuple

import numpy as np


def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def _parse_rpy_deg(block: Dict[str, Any]) -> Tuple[float, float, float]:
    rpy = block.get("rpy_deg", [0.0, 0.0, 0.0])
    if not isinstance(rpy, (list, tuple)) or len(rpy) != 3:
        return (0.0, 0.0, 0.0)
    d2r = math.pi / 180.0
    return (float(rpy[0]) * d2r, float(rpy[1]) * d2r, float(rpy[2]) * d2r)


def mag_in_imu_frame(mag_body: np.ndarray, extrinsics: Dict[str, Any]) -> np.ndarray:
    """Rotate magnetometer vector from its mounting frame into the IMU frame."""
    ex = extrinsics.get("mag") or {}
    r, p, y = _parse_rpy_deg(ex if isinstance(ex, dict) else {})
    R = _rpy_to_R(r, p, y)
    return R @ np.asarray(mag_body, dtype=float)


def depth_lever_arm_m(extrinsics: Dict[str, Any]) -> np.ndarray:
    bl = extrinsics.get("depth") or {}
    p = bl.get("position_m", [0.0, 0.0, 0.0])
    if not isinstance(p, (list, tuple)) or len(p) != 3:
        return np.zeros(3)
    return np.array([float(p[0]), float(p[1]), float(p[2])], dtype=float)


def parse_time_offsets(cfg: Dict[str, Any]) -> Dict[str, float]:
    to = cfg.get("time_offsets_sec") or {}
    if not isinstance(to, dict):
        return {}
    out: Dict[str, float] = {}
    for k, v in to.items():
        try:
            out[str(k)] = float(v)
        except (TypeError, ValueError):
            continue
    return out
