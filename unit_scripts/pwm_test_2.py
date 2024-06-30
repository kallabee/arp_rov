# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
# kit = ServoKit(channels=8)
kit = ServoKit(channels=16)

def set_pulse_width_range_all(min,max):
    kit.continuous_servo[0].set_pulse_width_range(min, max)
    kit.continuous_servo[1].set_pulse_width_range(min, max)

def set_throttle(t):
    kit.continuous_servo[0].throttle = t
    kit.continuous_servo[1].throttle = t

min_pulse = 1000
max_pulse =2000
offset_pulse = -60
set_pulse_width_range_all(min_pulse+offset_pulse, max_pulse+offset_pulse)

while True:
    v = input("Input value :")
    if v == "":
        break
    v = float(v)
    print(f"v = {v}")
    set_throttle(v)



# kit.continuous_servo[0].throttle = 1
# time.sleep(3)
# kit.continuous_servo[0].throttle = -1000
# time.sleep(3)
# kit.continuous_servo[0].throttle = 0








