import time
import board

import adafruit_bmp280
from adafruit_blinka.microcontroller.bcm2711.pin import *
import busio

# i2c = board.I2C()   # uses board.SCL and board.SDA
i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
# i2c=busio.I2C(D23, D22) #I2C ch6

addr = 0x76
# addr = 0x77

bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=addr)

# OR Create sensor object, communicating over the board's default SPI bus
# spi = board.SPI()
# bmp_cs = digitalio.DigitalInOut(board.D10)
# bmp280 = adafruit_bmp280.Adafruit_BMP280_SPI(spi, bmp_cs)

# change this to match the location's pressure (hPa) at sea level
bmp280.sea_level_pressure = 1013.25

while True:
    print("\nTemperature: %0.1f C" % bmp280.temperature)
    print("Pressure: %0.1f hPa" % bmp280.pressure)
    print("Altitude = %0.2f meters" % bmp280.altitude)
    time.sleep(2)


