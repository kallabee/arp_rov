#!/usr/bin/env python3
"""CLI to exercise CamActControllerPT without ROS (needs I2C + installed workspace packages).

  source /path/to/dev_ws/install/setup.bash
  python3 unit_scripts/cam_act_pt_cli.py
  python3 unit_scripts/cam_act_pt_cli.py --pan 0 --tilt 0
  DEVICE_I2C_YAML=/path/to/device_i2c.yaml python3 unit_scripts/cam_act_pt_cli.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _ensure_workspace_imports() -> None:
    """Make local `src/` + `install/` packages importable without sourcing ROS env.

    This keeps the script usable when invoked as:
      /bin/python3 unit_scripts/cam_act_pt_cli.py
    """

    try:
        import device_registry  # noqa: F401

        return
    except Exception:
        pass

    here = Path(__file__).resolve()
    for base in (here.parent, *here.parents):
        install_dir = base / "install"
        src_dir = base / "src"
        if not (install_dir.is_dir() and src_dir.is_dir()):
            continue

        # Add editable source trees (so it works even before colcon build).
        for pkg in ("device_registry", "thruster_controller"):
            candidate = src_dir / pkg
            if candidate.is_dir():
                sys.path.insert(0, str(candidate))

        # Add installed site-packages (best effort; supports multiple python versions).
        for sp in install_dir.glob("**/site-packages"):
            sys.path.insert(0, str(sp))

        return


def _default_yaml_path(explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit
    # Prefer the workspace source tree so edits take effect immediately.
    here = Path(__file__).resolve().parent
    for base in (here, *here.parents):
        cand = (
            base
            / "src"
            / "thruster_controller"
            / "thruster_controller"
            / "cam_act_pt.yaml"
        )
        if cand.is_file():
            return cand
    try:
        import thruster_controller as tc

        p = Path(tc.__file__).resolve().parent / "cam_act_pt.yaml"
        if p.is_file():
            return p
    except ImportError:
        pass
    # Last resort: relative fallback.
    return here.parent / "src" / "thruster_controller" / "thruster_controller" / "cam_act_pt.yaml"


def _default_device_i2c_yaml(explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit
    # Prefer an explicit env override if present (matches DeviceRegistry behavior).
    env = __import__("os").environ.get("DEVICE_I2C_YAML")
    if env:
        return Path(env)
    # Look for workspace root by walking parents of this script.
    here = Path(__file__).resolve().parent
    for base in (here, *here.parents):
        cand = base / "src" / "device_i2c.yaml"
        if cand.is_file():
            return cand
    return None


def _build_controller(config_path: Path, device_i2c_yaml: Path | None, device_id: str) -> object:
    from device_registry import DeviceRegistry
    from thruster_controller.cam_act_config import CamActPTConfig
    from thruster_controller.cam_act_controllers import CamActControllerPT

    cfg = CamActPTConfig.load(config_path)
    reg = (
        DeviceRegistry.load(device_i2c_yaml)
        if device_i2c_yaml is not None
        else DeviceRegistry.from_src_default()
    )
    d = reg.get_i2c(device_id)
    if d.linux_bus is None:
        raise SystemExit(
            f"device {device_id!r} needs 'bus' in device_i2c.yaml "
            "(or set DEVICE_I2C_YAML to a valid map)."
        )
    return CamActControllerPT(cfg, linux_bus=d.linux_bus, i2c_addr=d.addr)


def _repl(ctrl) -> None:
    print("Commands: p <deg> | t <deg> | q")
    print("Example: p 10.5")
    while True:
        try:
            line = input("cam_act_pt> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line or line.lower() in {"q", "quit", "exit"}:
            break
        parts = line.split(maxsplit=1)
        cmd = parts[0].lower()
        if cmd not in {"p", "t"}:
            print("Unknown command. Use p, t, or q.")
            continue
        if len(parts) < 2:
            print(f"Usage: {cmd} <number>")
            continue
        try:
            val = float(parts[1])
        except ValueError:
            print("Angle must be a number.")
            continue
        if cmd == "p":
            ctrl.move_pan(val)
        else:
            ctrl.move_tilt(val)


def main() -> int:
    _ensure_workspace_imports()
    parser = argparse.ArgumentParser(
        description="Control CamActControllerPT (pan/tilt) from the shell without ROS nodes."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to cam_act_pt.yaml (default: bundled / next to thruster_controller package)",
    )
    parser.add_argument(
        "--device",
        default="cam_act",
        help="device_i2c.yaml logical device id for bus/address (default: cam_act)",
    )
    parser.add_argument(
        "--device-i2c",
        type=Path,
        default=None,
        help="Path to device_i2c.yaml (default: auto-detect or DEVICE_I2C_YAML)",
    )
    parser.add_argument(
        "--pan",
        type=float,
        default=None,
        help="Set pan once and exit (requires hardware / use_cam_act true in yaml)",
    )
    parser.add_argument(
        "--tilt",
        type=float,
        default=None,
        help="Set tilt once and exit",
    )
    args = parser.parse_args()

    cfg_path = _default_yaml_path(args.config)
    if not cfg_path.is_file():
        print(f"Config not found: {cfg_path}", file=sys.stderr)
        return 1
    dev_i2c = _default_device_i2c_yaml(args.device_i2c)

    try:
        ctrl = _build_controller(cfg_path, dev_i2c, args.device)
    except Exception as exc:
        print(f"Failed to open controller: {exc}", file=sys.stderr)
        return 1

    if not getattr(ctrl, "use_cam_act", False):
        print(
            "cam_act_pt.yaml has use_cam_act: false — move_pan/tilt will fail without servo.",
            file=sys.stderr,
        )

    if args.pan is not None:
        ctrl.move_pan(args.pan)
    if args.tilt is not None:
        ctrl.move_tilt(args.tilt)
    if args.pan is not None or args.tilt is not None:
        return 0

    _repl(ctrl)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
