import ina226
import time

sensor = ina226.INA226(busnum=3, address=0x40, shunt_ohms=0.002, max_expected_amps=25)
sensor.configure()

while True:
    current = sensor.current()
    power = sensor.power()
    voltage = sensor.voltage()
    print(f"{voltage:4.1f} [V], {current/1000:5.3f} [A], {power/1000:4.1f} [W]")
    time.sleep(1)
