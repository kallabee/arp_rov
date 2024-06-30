from board import SCL,SDA
import busio
import time

#ImportthePCA9685module.
from adafruit_pca9685 import PCA9685

#CreatetheI2Cbusinterface.
i2c_bus=busio.I2C(SCL,SDA)
addr = 0x71
#CreateasimplePCA9685classinstance.
pca=PCA9685(i2c_bus, address=addr)

#SetthePWMfrequencyto60hz.
pca.frequency=60

#SetthePWMdutycycleforchannelzeroto50%.duty_cycleis16bitstomatch

#butthePCA9685willonlyactuallygive12bitsofresolution.
pca.channels[2].duty_cycle=0x7FFF
time.sleep(3)
pca.channels[2].duty_cycle=0