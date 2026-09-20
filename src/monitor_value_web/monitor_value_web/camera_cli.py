from __future__ import annotations

import argparse
import json
import sys

from pathlib import Path

from monitor_value_web.camera import ZOOM_PRESETS, CameraController
from monitor_value_web.config import load_web_config


def _default_cfg_path() -> Path:
    src = Path(__file__).resolve().parents[1] / "config" / "monitor_value_web.yaml"
    if src.is_file():
        return src
    return src


def _controller(api: str | None, backend: str | None = None) -> CameraController:
    path = _default_cfg_path()
    if path.is_file():
        cfg = load_web_config(path)
    else:
        cfg = {
            "cameras": {
                "backend": "mediamtx",
                "api": "http://127.0.0.1:9997",
                "items": [
                    {"id": "cam0", "path": "cam0", "index": 0, "nickname": "Ceiling"},
                    {"id": "cam1", "path": "cam1", "index": 1, "nickname": "Canopy"},
                ],
            }
        }
    if api:
        cfg.setdefault("cameras", {})["api"] = api
        cfg.setdefault("cameras", {}).setdefault("mediamtx", {})["api"] = api
    if backend:
        cfg.setdefault("cameras", {})["backend"] = backend
    return CameraController(cfg)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Raspberry Pi camera control (MediaMTX or momo)")
    ap.add_argument("--api", default=None, help="MediaMTX Control API URL")
    ap.add_argument("--backend", choices=["mediamtx", "momo"], default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    get_p = sub.add_parser("get")
    get_p.add_argument("camera")
    set_p = sub.add_parser("set")
    set_p.add_argument("camera")
    set_p.add_argument("--zoom", choices=list(ZOOM_PRESETS))
    set_p.add_argument("--focus-mode", choices=["continuous", "auto", "manual"])
    set_p.add_argument("--af-window")
    set_p.add_argument("--lens", type=float, dest="lens_position")
    set_p.add_argument("--ae", action="store_true")
    set_p.add_argument("--me", action="store_true")
    set_p.add_argument("--ev", type=float)
    set_p.add_argument("--shutter", type=int)
    set_p.add_argument("--gain", type=float)
    set_p.add_argument("--wb", choices=["auto", "manual"])
    set_p.add_argument("--wb-gains")
    args = ap.parse_args(argv)

    ctrl = _controller(args.api, args.backend)
    if args.cmd == "list":
        print(json.dumps(ctrl.snapshot(), indent=2, ensure_ascii=False))
        return 0
    if args.cmd == "get":
        snap = ctrl.snapshot()
        item = next((x for x in snap.get("state") or [] if x.get("id") == args.camera), None)
        if item is None:
            print(f"unknown camera: {args.camera}", file=sys.stderr)
            return 1
        print(json.dumps(item, indent=2, ensure_ascii=False))
        return 0

    req: dict = {}
    if args.zoom:
        req["zoom"] = args.zoom
    focus: dict = {}
    if args.focus_mode:
        focus["mode"] = args.focus_mode
    if args.af_window is not None:
        focus["window"] = "" if args.af_window == "full" else args.af_window
    if args.lens_position is not None:
        focus["lens_position"] = args.lens_position
    if focus:
        req["focus"] = focus
    if args.ae and args.me:
        print("use only one of --ae / --me", file=sys.stderr)
        return 1
    if args.ae:
        req["exposure"] = {"mode": "ae"}
        if args.ev is not None:
            req["exposure"]["ev"] = args.ev
    elif args.me:
        req["exposure"] = {"mode": "me"}
        if args.shutter is not None:
            req["exposure"]["shutter_us"] = args.shutter
        if args.gain is not None:
            req["exposure"]["gain"] = args.gain
    elif args.ev is not None:
        req["exposure"] = {"ev": args.ev}
    if args.wb:
        wb: dict = {"mode": args.wb}
        if args.wb_gains:
            wb["gains"] = [float(x) for x in args.wb_gains.split(",")]
        req["wb"] = wb
    if not req:
        print("nothing to set", file=sys.stderr)
        return 1
    print(json.dumps(ctrl.apply(args.camera, req), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
