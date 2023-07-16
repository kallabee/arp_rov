from as7263 import AS7263

# from smbus import SMBus
from smbus2 import SMBus

i2c_as7263 = SMBus(3)  #i2c3 (Ch B)

as7263 = AS7263(i2c_dev=i2c_as7263)
as7263.set_measurement_mode(2)
as7263.set_illumination_led(1)

try:
    while True:
        values = as7263.get_calibrated_values()
        print("""
            610nm:    {}
            680nm: {}
            730nm: {}
            760nm:  {}
            810nm:   {}
            860nm: {}""".format(*values))
except KeyboardInterrupt:
    as7263.set_measurement_mode(3)
    as7263.set_illumination_led(0)