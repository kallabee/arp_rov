#!/usr/bin/env python3
"""Interactive PCA9685 channel + throttle tester.

Use this to map "which channel drives which thruster".

Examples:
  python3 unit_scripts/pwm_channel_cli.py
  python3 unit_scripts/pwm_channel_cli.py --addr 0x73

Commands (REPL):
  ch <0-15> <throttle(-1..1)>   set one channel
  all <throttle(-1..1)>         set all 16 channels
  off                            set all channels to 0
  range <min_us> <max_us> [offset_us]   set pulse range for all channels
  q                              quit
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

import busio
from adafruit_servokit import ServoKit
from board import SCL, SDA


def _parse_int(s: str) -> int:
    return int(s.strip(), 0)


def _set_range(kit: ServoKit, min_us: int, max_us: int, offset_us: int = 0) -> None:
    lo = int(min_us) + int(offset_us)
    hi = int(max_us) + int(offset_us)
    for ch in range(16):
        kit.continuous_servo[ch].set_pulse_width_range(lo, hi)


def _set_channels(kit: ServoKit, channels: Iterable[int], throttle: float) -> None:
    for ch in channels:
        kit.continuous_servo[ch].throttle = throttle


def _init_all(kit: ServoKit, min_us: int, max_us: int, offset_us: int) -> None:
    """Initialize all 16 channels to a known safe state."""
    _set_range(kit, min_us, max_us, offset_us)
    _set_channels(kit, range(16), 0.0)
    # Some ESCs are sensitive; write twice with a short delay.
    time.sleep(0.05)
    _set_channels(kit, range(16), 0.0)


def main() -> int:
    ap = argparse.ArgumentParser(description="Manual PCA9685 channel throttle tester.")
    ap.add_argument("--addr", default="0x73", help="PCA9685 7-bit I2C address (default: 0x73)")
    ap.add_argument("--min-us", type=int, default=1000, help="min pulse width (default: 1000)")
    ap.add_argument("--max-us", type=int, default=2000, help="max pulse width (default: 2000)")
    ap.add_argument("--offset-us", type=int, default=-20, help="pulse width offset")
    args = ap.parse_args()

    addr = _parse_int(args.addr)
    i2c = busio.I2C(SCL, SDA)

    try:
        kit = ServoKit(i2c=i2c, address=addr, channels=16)
    except Exception as exc:
        print(f"Failed to init ServoKit at addr=0x{addr:02x}: {exc}", file=sys.stderr)
        return 1

    _init_all(kit, args.min_us, args.max_us, args.offset_us)

    print(
        "Commands: ch <0-15> <throttle(-1..1)> | all <throttle> | off | "
        "range <min_us> <max_us> [offset_us] | q"
    )

    while True:
        try:
            line = input("pwm> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line.lower() in {"q", "quit", "exit"}:
            break

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == "ch":
                if len(parts) != 3:
                    print("usage: ch <0-15> <throttle>")
                    continue
                ch = int(parts[1])
                if ch < 0 or ch > 15:
                    print("channel must be 0..15")
                    continue
                v = float(parts[2])
                kit.continuous_servo[ch].throttle = v
                print(f"ch={ch} throttle={v:+.3f}")
            elif cmd == "all":
                if len(parts) != 2:
                    print("usage: all <throttle>")
                    continue
                v = float(parts[1])
                _set_channels(kit, range(16), v)
                print(f"all throttle={v:+.3f}")
            elif cmd == "off":
                _set_channels(kit, range(16), 0.0)
                print("all throttle=+0.000")
            elif cmd == "range":
                if len(parts) not in {3, 4}:
                    print("usage: range <min_us> <max_us> [offset_us]")
                    continue
                min_us = int(parts[1])
                max_us = int(parts[2])
                offset_us = int(parts[3]) if len(parts) == 4 else 0
                _set_range(kit, min_us, max_us, offset_us)
                print(f"range set: {min_us}+{offset_us} .. {max_us}+{offset_us} us")
            else:
                print("Unknown command.")
        except OSError as exc:
            print(f"I2C error: {exc}", file=sys.stderr)
        except ValueError as exc:
            print(f"Invalid input: {exc}", file=sys.stderr)

    _set_channels(kit, range(16), 0.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

