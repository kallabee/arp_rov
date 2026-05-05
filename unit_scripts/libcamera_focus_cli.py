#!/usr/bin/env python3
"""Small CLI to test Raspberry Pi Camera v3 focus controls via libcamera/picamera2.

Examples:
  python3 unit_scripts/libcamera_focus_cli.py --list
  python3 unit_scripts/libcamera_focus_cli.py --af-mode continuous
  python3 unit_scripts/libcamera_focus_cli.py --af-mode manual --lens-position 2.0

Notes:
  - Requires `picamera2` (libcamera stack).
  - LensPosition typically uses dioptres (1 / distance[m]); range depends on sensor/driver.
"""

from __future__ import annotations

import argparse
import sys


def _mode_enum(mode: str):
    from libcamera import controls  # type: ignore

    m = mode.lower().strip()
    mapping = {
        "continuous": controls.AfModeEnum.Continuous,
        "auto": controls.AfModeEnum.Auto,
        "manual": controls.AfModeEnum.Manual,
        "off": controls.AfModeEnum.Manual,  # treat off as manual with fixed lens
    }
    if m not in mapping:
        raise ValueError("af mode must be one of: continuous, auto, manual, off")
    return mapping[m]


def main() -> int:
    ap = argparse.ArgumentParser(description="Test libcamera focus controls (picamera2).")
    ap.add_argument("--list", action="store_true", help="Print supported camera controls and exit")
    ap.add_argument(
        "--af-mode",
        default=None,
        help="Set AF mode: continuous|auto|manual|off",
    )
    ap.add_argument(
        "--lens-position",
        type=float,
        default=None,
        help="Set LensPosition (requires manual/off). Example: 2.0",
    )
    args = ap.parse_args()

    try:
        from picamera2 import Picamera2  # type: ignore
    except Exception as exc:
        print(f"picamera2 is not available: {exc}", file=sys.stderr)
        return 1

    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration()
    picam2.configure(cfg)
    picam2.start()

    if args.list:
        # `camera_controls` is a dict of control-name -> ControlInfo
        controls = getattr(picam2, "camera_controls", None)
        if controls is None:
            print("camera_controls is not available on this picamera2 version.")
            return 0
        for k in sorted(controls.keys()):
            print(k)
        return 0

    if args.af_mode is not None:
        try:
            picam2.set_controls({"AfMode": _mode_enum(args.af_mode)})
            print(f"AfMode set to {args.af_mode}")
        except Exception as exc:
            print(f"Failed to set AfMode: {exc}", file=sys.stderr)
            return 1

    if args.lens_position is not None:
        try:
            picam2.set_controls({"LensPosition": float(args.lens_position)})
            print(f"LensPosition set to {args.lens_position}")
        except Exception as exc:
            print(f"Failed to set LensPosition: {exc}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

