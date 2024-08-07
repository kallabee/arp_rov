import board
import time
from adafruit_bme280 import basic as adafruit_bme280  # This library is deprecated...
from adafruit_blinka.microcontroller.bcm2711.pin import *
import busio

# Create sensor object, using the board's default I2C bus.
# i2c = board.I2C()   # uses board.SCL and board.SDA
i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
# i2c=busio.I2C(D23, D22) #I2C ch6

addr = 0x76
# addr = 0x77

bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=addr)

# change this to match the location's pressure (hPa) at sea level
bme280.sea_level_pressure = 1013.25

while True:
    print("\nTemperature: %0.1f C" % bme280.temperature)
    print("Humidity: %0.1f %%" % bme280.relative_humidity)
    print("Pressure: %0.1f hPa" % bme280.pressure)
    print("Altitude = %0.2f meters" % bme280.altitude)
    time.sleep(2)
