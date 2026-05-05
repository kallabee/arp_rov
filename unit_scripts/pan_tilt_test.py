"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

from board import SCL, SDA
import busio


i2c = busio.I2C(SCL, SDA)   # ch1
# i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
# i2c=busio.I2C(D23, D22) #I2C ch6


# addr = 0x61
addr = 0x63
# addr = 0x73

ch_pan=0
ch_tilt=15


kit = ServoKit(i2c=i2c, address=addr, channels=16)

# start_no = 4
# end_no=12
start_no = 0
end_no=16

def set_pulse_width_range_all(min, max):
    for i in range(start_no,end_no):
        # kit.servo[start_no+i].set_pulse_width_range(min, max)
        kit.continuous_servo[start_no+i].set_pulse_width_range(min, max)


def set_angle(ch:int, angle:float):
    # 0 <= angle <= 359
    # kit.servo[ch].angle = angle

    kit.continuous_servo[ch].throttle = angle


min_pulse = 1000
max_pulse = 2000
offset_pulse = 0
# offset_pulse = 30
# offset_pulse = 70
set_pulse_width_range_all(min_pulse + offset_pulse, max_pulse + offset_pulse)

while True:
    v = input("Input value :")
    if v == "":
        break
    try:
        v = float(v)
    except:
        continue
    # print(f"v = {v}")
    set_angle(ch_pan, v)
    set_angle(ch_tilt, v)
