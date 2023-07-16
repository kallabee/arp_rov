# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit
# from adafruit-circuitpython-servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(address=0x60, channels=8)

min_pulse = 1000    # [ms]
max_pulse =2000
offset_pulse = -60
num_ch = 8
     
for i in range(num_ch):
    kit.continuous_servo[i].set_pulse_width_range(min_pulse+offset_pulse, max_pulse+offset_pulse)

k=0
while k != 27:
    k=input('>> ')
    t = float(k)
    for ch in range(num_ch):
        kit.continuous_servo[ch].throttle = t


for ch in range(num_ch):
    kit.continuous_servo[ch].throttle = 0