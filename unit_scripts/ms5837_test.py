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

print(sensor.pressure(ms5837_renamed.UNITS_atm))

print(f"Pressure: {sensor.pressure(ms5837_renamed.UNITS_atm):.2f} atm {sensor.pressure(ms5837_renamed.UNITS_Torr):.2f} Torr  {sensor.pressure(ms5837_renamed.UNITS_psi):.2f} psi")


print(f"Temperature: {sensor.temperature(ms5837_renamed.UNITS_Centigrade):.2f} C  {sensor.temperature(ms5837_renamed.UNITS_Farenheit):.2f} F  {sensor.temperature(ms5837_renamed.UNITS_Kelvin):.2f} K")
freshwaterDepth = sensor.depth() # default is freshwater
sensor.setFluidDensity(ms5837_renamed.DENSITY_SALTWATER)
saltwaterDepth = sensor.depth() # No nead to read() again
sensor.setFluidDensity(1000) # kg/m^3
print(f"Depth: {freshwaterDepth:.3f} m (freshwater)  {saltwaterDepth:.3f} m (saltwater)")

# fluidDensity doesn't matter for altitude() (always MSL air density)
print(f"MSL Relative Altitude: {sensor.altitude():.2f} m")  # relative to Mean Sea Level pressure in air

time.sleep(5)

# Spew readings
while True:
        if sensor.read():
                print(f"P: {sensor.pressure():0.1f} mbar  {sensor.pressure(ms5837_renamed.UNITS_psi):0.3f} psi\tT: {sensor.temperature():0.2f} C  {sensor.temperature(ms5837_renamed.UNITS_Farenheit):0.2f} F")
        else:
                print("Sensor read failed!")
                exit(1)