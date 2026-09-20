import time
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_extended_bus import ExtendedI2C

# I2C bus 2 (/dev/i2c-2)
i2c = ExtendedI2C(2)

ads = ADS.ADS1015(i2c, address=0x49)
channels = [AnalogIn(ads, ch) for ch in range(4)]

header = "\t".join(f"ch{ch}[V]" for ch in range(4))
print(header)

while True:
    line = "\t".join(f"{c.voltage:6.3f}" for c in channels)
    print(line)
    time.sleep(0.5)
