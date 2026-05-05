"""Interactive light PWM controller with safe shutdown behavior."""

from __future__ import annotations

import atexit
import signal
import sys

import busio
from adafruit_pca9685 import PCA9685
from board import SCL, SDA


class LightController:
    def __init__(self, i2c_bus: busio.I2C, i2c_addr: int, freq: int):
        self.pca = PCA9685(i2c_bus, address=i2c_addr)
        self.pca.frequency = freq

    def set_duty(self, ch: int, value: float) -> None:
        if not 0.0 <= value <= 1.0:
            raise ValueError("Duty must be in range [0.0, 1.0].")
        self.pca.channels[ch].duty_cycle = int(0xFFFF * value)

    def all_off(self, channels: dict[str, int]) -> None:
        for ch in channels.values():
            self.pca.channels[ch].duty_cycle = 0


# LIGHT_CHANNELS: dict[str, int] = {
#     "VisFwdTop": 10,
#     "VisDwn": 11,
#     "VisFwdTel": 5,
#     "VisFwdBtm": 4,
#     "Uv": 6,
#     "Ir": 9,
# }

LIGHT_CHANNELS: dict[str, int] = {
    # 0
    # 1
    # 2
    # 3

    "VisFwdBtm": 4,
    "VisFwdTel": 5,   
    "Uv": 6,
    # 7
    
    "Test": 8,
    "Ir": 9,
    "VisFwdTop": 10,
    "VisDwn": 11,
    
    # 12
    # 13
    # 14
    # 15
}


def print_channel_list() -> None:
    print("\n=== Light Channels ===")
    for name, ch in LIGHT_CHANNELS.items():
        print(f"- {name}: {ch}")
    print("======================")


def main() -> None:
    i2c_bus = busio.I2C(SCL, SDA)
    i2c_addr = 0x60
    freq = 1000  # [Hz]
    controller = LightController(i2c_bus, i2c_addr, freq)

    def safe_shutdown() -> None:
        controller.all_off(LIGHT_CHANNELS)
        print("\nAll configured channels set to 0%.")

    def handle_signal(signum: int, _frame) -> None:
        print(f"\nReceived signal {signum}, shutting down safely...")
        safe_shutdown()
        sys.exit(0)

    atexit.register(safe_shutdown)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    while True:
        print_channel_list()
        print("Enter 'q' to quit.")

        ch_str = input("Channel number: ").strip()
        if ch_str.lower() in {"q", "quit", "exit"}:
            break

        try:
            ch = int(ch_str)
        except ValueError:
            print("Channel must be an integer.")
            continue

        if ch not in LIGHT_CHANNELS.values():
            print("Unknown channel number.")
            continue

        duty_str = input("PWM duty [0.0-1.0]: ").strip()
        if duty_str.lower() in {"q", "quit", "exit"}:
            break

        try:
            duty = float(duty_str)
            controller.set_duty(ch, duty)
        except ValueError as exc:
            print(f"Invalid input: {exc}")
            continue

        light_name = next((name for name, channel in LIGHT_CHANNELS.items() if channel == ch), "Unknown")
        print(f"Set ch {ch} ({light_name}) to {duty:.3f}.")


if __name__ == "__main__":
    main()
