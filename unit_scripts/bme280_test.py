import time
import board
# import digitalio # For use with SPI
import adafruit_bmp280

from adafruit_blinka.microcontroller.bcm2711.pin import *
import busio
# Create sensor object

i2c=busio.I2C(D23, D22, frequency=400000) #

# while not i2c.try_lock():
#     pass
# for i in i2c.scan():
#     print('addr 0x{0:x}'.format(i))
# i2c.deinit()

bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)

# change this to match the location's pressure (hPa) at sea level
bmp280.sea_level_pressure = 1013.25

while True:
    print("\nTemperature: %0.1f C" % bmp280.temperature)
    print("Pressure: %0.1f hPa" % bmp280.pressure)
    print("Altitude = %0.2f meters" % bmp280.altitude)
    time.sleep(2)