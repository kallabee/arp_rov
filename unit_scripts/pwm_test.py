from board import SCL, SDA
import busio

# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA)

addr = 0x61
# addr = 0x71

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus, address=addr)

# Set the PWM frequency to 1kHz.
pca.frequency = 1000

num_ch = 8
k = 0
while k != 27:
    k = input(">> ")
    t = float(k)
    for ch in range(num_ch):
        pca.channels[ch].duty_cycle = (int)(0x7FFF * t)
