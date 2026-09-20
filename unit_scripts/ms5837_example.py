#!/usr/bin/python
import ms5837
import time

# sensor = ms5837.MS5837_30BA(3) # Default I2C bus is 1 (Raspberry Pi 3)
#sensor = ms5837.MS5837_30BA(0) # Specify I2C bus
#sensor = ms5837.MS5837_02BA()
#sensor = ms5837.MS5837_02BA(0)
#sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model and bus

sensor = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=2) # Specify model and bus

# We must initialize the sensor before reading it
if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

print(("Pressure: %.3f atm") % (sensor.pressure(ms5837.UNITS_atm)))

print(("Temperature: %.2f ℃") % (sensor.temperature(ms5837.UNITS_Centigrade)))

freshwaterDepth = sensor.depth() # default is freshwater
sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
saltwaterDepth = sensor.depth() # No nead to read() again
sensor.setFluidDensity(1000) # kg/m^3
# Depth / altitude are omitted to keep units atm, ℃ only.

time.sleep(5)

# Spew readings
while True:
        if sensor.read():
                print(("P: %0.3f atm\tT: %0.2f ℃") % (
                sensor.pressure(ms5837.UNITS_atm),
                sensor.temperature(ms5837.UNITS_Centigrade)))
        else:
                print("Sensor read failed!")
                exit(1)
