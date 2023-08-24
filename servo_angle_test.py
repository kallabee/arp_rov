﻿# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# ref. https://docs.circuitpython.org/projects/motor/en/latest/_modules/adafruit_motor/servo.html

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit


num_ch = 10


def reset_servo() -> None:
    # addr = 0x60  # No.1
    # addr = 0x62
    addr = 0x63
    # addr = 0x61  # No.3
    kit = ServoKit(address=addr, channels=16)
    # min_pulse = 800  # [us]
    # max_pulse = 2200  # [us]
    min_pulse = 1000  # [us]
    max_pulse = 2000  # [us]
    offset_pulse = 0  # [us]
    # for i in range(self.tc.get_num()):
    for i in range(num_ch):
        kit.servo[i].set_pulse_width_range(
            min_pulse + offset_pulse, max_pulse + offset_pulse
        )
    # self.apply_thrustors(np.zeros((6), dtype=np.float32))

    return kit


kit = reset_servo()


k = 0
while k != 27:
    k = input(">> ")
    t = float(k)
    for ch in range(num_ch):
        kit.servo[ch].angle = t


for ch in range(num_ch):
    kit.servo[ch].angle = 0
