"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

from board import SCL, SDA
import busio

# Module is attached on i2c channel 1
i2c_bus = busio.I2C(SCL, SDA)

# addr = 0x61
# addr = 0x70
addr = 0x71

kit = ServoKit(i2c=i2c_bus, address=addr, channels=16)


def set_pulse_width_range_all(min, max):
    kit.continuous_servo[0].set_pulse_width_range(min, max)
    kit.continuous_servo[1].set_pulse_width_range(min, max)


def set_throttle(t):
    kit.continuous_servo[0].throttle = t
    kit.continuous_servo[1].throttle = t


min_pulse = 1000
max_pulse = 2000
offset_pulse = 70
set_pulse_width_range_all(min_pulse + offset_pulse, max_pulse + offset_pulse)

while True:
    v = input("Input value :")
    if v == "":
        break
    try:
        v = float(v)
    except:
        continue
    print(f"v = {v}")
    set_throttle(v)
