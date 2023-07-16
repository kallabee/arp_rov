from as7263 import AS7263
from as7262 import AS7262
from adafruit_blinka.microcontroller.bcm2711.pin import *
from adafruit_blinka.microcontroller.bcm2711.pin import *
import busio

i2c_AS2762=busio.I2C(D5, D4) #i2c3 (Ch B)
i2c_AS7263=busio.I2C(D23, D22) #i2c6 (Ch C)

as7263 = AS7263(i2c_dev=i2c_AS7263)
as7263.set_measurement_mode(2)
as7263.set_illumination_led(1)

as7262 = AS7262(as7262.as7262VirtualRegisterBus(i2c))
as7262.set_measurement_mode(2)
as7262.set_illumination_led(1)