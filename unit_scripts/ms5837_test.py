#!/usr/bin/python
import python.ms5837_renamed as ms5837_renamed
import time

# sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
sensor = ms5837_renamed.MS5837_30BA(3) # Specify I2C bus
#sensor = ms5837.MS5837_02BA()
#sensor = ms5837.MS5837_02BA(0)
#sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model and bus

# We must initialize the sensor before reading it
if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

print(f"Pressure: {sensor.pressure(ms5837_renamed.UNITS_atm):.3f} atm")

print(f"Temperature: {sensor.temperature(ms5837_renamed.UNITS_Centigrade):.2f} ℃")
freshwaterDepth = sensor.depth() # default is freshwater
sensor.setFluidDensity(ms5837_renamed.DENSITY_SALTWATER)
saltwaterDepth = sensor.depth() # No nead to read() again
sensor.setFluidDensity(1000) # kg/m^3
# Depth / altitude are omitted to keep units atm, ℃ only.

time.sleep(5)

# Spew readings
while True:
        if sensor.read():
                print(f"P: {sensor.pressure(ms5837_renamed.UNITS_atm):0.3f} atm\tT: {sensor.temperature(ms5837_renamed.UNITS_Centigrade):0.2f} ℃")
        else:
                print("Sensor read failed!")
                exit(1)