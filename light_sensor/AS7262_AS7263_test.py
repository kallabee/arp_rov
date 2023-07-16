# AS7262 Vis
# AS7263 IR

from as7262 import AS7262
import as7262
from as7263 import AS7263

# from smbus import SMBus
from smbus2 import SMBus

# Be carefull.
i2c_as7262 = SMBus(6)  # i2c3 (Ch B)
i2c_as7263 = SMBus(3)  # i2c3 (Ch B)

as7262 = AS7262(i2c_dev=i2c_as7262)
as7262.set_gain(64)
as7262.set_integration_time(17.857)
as7262.set_measurement_mode(2)
as7262.set_illumination_led(0)


as7263 = AS7263(i2c_dev=i2c_as7263)
as7263.set_measurement_mode(2)
as7263.set_illumination_led(0)


try:
    while True:
        values = as7262.get_calibrated_values()
        print(
            """
            Red:    {}
            Orange: {}
            Yellow: {}
            Green:  {}
            Blue:   {}
            Violet: {}""".format(
                *values
            )
        )

        values = as7263.get_calibrated_values()
        print(
            """
            610nm:    {}
            680nm: {}
            730nm: {}
            760nm:  {}
            810nm:   {}
            860nm: {}""".format(
                *values
            )
        )

except KeyboardInterrupt:
    as7262.set_measurement_mode(3)
    as7263.set_measurement_mode(3)
