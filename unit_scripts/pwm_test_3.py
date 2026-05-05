"""Apply throttle to a configurable channel range."""

import argparse
from adafruit_servokit import ServoKit

from board import SCL, SDA
import busio

# Module is attached on i2c channel 1
i2c_bus = busio.I2C(SCL, SDA)

# ---- Defaults (edit here if you don't want CLI args) ----
DEFAULT_ADDR = "0x73"
DEFAULT_CH_START = 4
DEFAULT_CH_END = 11  # inclusive
DEFAULT_MIN_PULSE_US = 1000
DEFAULT_MAX_PULSE_US = 2000
DEFAULT_OFFSET_US = -20


def main() -> None:
    ap = argparse.ArgumentParser(description="Throttle tester for a channel range.")
    ap.add_argument(
        "--addr",
        default=DEFAULT_ADDR,
        help=f"PCA9685 I2C address (default: {DEFAULT_ADDR})",
    )
    ap.add_argument(
        "--ch-start",
        type=int,
        default=DEFAULT_CH_START,
        help=f"start channel (default: {DEFAULT_CH_START})",
    )
    ap.add_argument(
        "--ch-end",
        type=int,
        default=DEFAULT_CH_END,
        help=f"end channel inclusive (default: {DEFAULT_CH_END})",
    )
    ap.add_argument(
        "--min-pulse",
        type=int,
        default=DEFAULT_MIN_PULSE_US,
        help=f"min pulse width [us] (default: {DEFAULT_MIN_PULSE_US})",
    )
    ap.add_argument(
        "--max-pulse",
        type=int,
        default=DEFAULT_MAX_PULSE_US,
        help=f"max pulse width [us] (default: {DEFAULT_MAX_PULSE_US})",
    )
    ap.add_argument(
        "--offset",
        type=int,
        default=DEFAULT_OFFSET_US,
        help=f"pulse offset [us] (default: {DEFAULT_OFFSET_US})",
    )
    args = ap.parse_args()

    if args.ch_start < 0 or args.ch_end > 15 or args.ch_start > args.ch_end:
        raise SystemExit("channel range must satisfy 0 <= start <= end <= 15")

    addr = int(str(args.addr), 0)
    kit = ServoKit(i2c=i2c_bus, address=addr, channels=16)

    lo = args.min_pulse + args.offset
    hi = args.max_pulse + args.offset
    for ch in range(args.ch_start, args.ch_end + 1):
        kit.continuous_servo[ch].set_pulse_width_range(lo, hi)
        kit.continuous_servo[ch].throttle = 0

    while True:
        v = input(f"Input value (applies to ch {args.ch_start}-{args.ch_end}): ").strip()
        if v == "":
            break
        try:
            t = float(v)
        except ValueError:
            continue
        print(f"throttle = {t}")
        for ch in range(args.ch_start, args.ch_end + 1):
            kit.continuous_servo[ch].throttle = t

    for ch in range(args.ch_start, args.ch_end + 1):
        kit.continuous_servo[ch].throttle = 0

if __name__ == "__main__":
    main()
