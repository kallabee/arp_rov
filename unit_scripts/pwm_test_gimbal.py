"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

from board import SCL, SDA
import busio


i2c = busio.I2C(SCL, SDA)   # ch1
# i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
# i2c=busio.I2C(D23, D22) #I2C ch6

# addr = 0x61
addr = 0x74 # gimbal for camera
# addr = 0x73 # thrustors

# ch 0: Tilt
# ch 15: Pan

kit = ServoKit(i2c=i2c, address=addr, channels=16)

# thrustors
offset_no = 0
start_no = 0
end_no=16

def set_pulse_width_range_all(min, max):
    for i in range(start_no,end_no):
        # kit.continuous_servo[start_no+i].set_pulse_width_range(min, max)
        kit.continuous_servo[i+offset_no].set_pulse_width_range(min, max)


def set_throttle(t):
    for i in range(start_no,end_no):
        # kit.continuous_servo[use_start_no+i].throttle = t
        kit.continuous_servo[i+offset_no].throttle = t
        print(f"ch:{i:2d}, v:{t:+0.1f}")

min_pulse = 1000
max_pulse = 2000
offset_pulse = 0
# offset_pulse = 30
# offset_pulse = 70
set_pulse_width_range_all(min_pulse + offset_pulse, max_pulse + offset_pulse)

while True:
    v = input("Input value (float, -1.0 to +1.0):")
    if v == "":
        break
    try:
        v = float(v)
    except:
        continue
    # print(f"v = {v}")
    set_throttle(v)
