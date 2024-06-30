from temp_pressure_sensor import *
from current_sensor import *
from depth_sensor import *

if __name__ == "__main__":
    pm_busnum = 3
    pm_addr = 0x40
    pm = power_monitor(busnum=pm_busnum, addr=pm_addr)

    # i2c = board.I2C()   # uses board.SCL and board.SDA
    i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
    # i2c=busio.I2C(D23, D22) #I2C ch6

    addr = 0x76
    # addr = 0x77

    sensor_type = "BMP280"
    # sensor_type = 'BME280'

    tph_sensor = temp_pressure_humidity_sensor(
        sensor_type=sensor_type, i2c=i2c, addr=addr
    )

    ds_busnum = 3
    ds = depth_sensor(ds_busnum)

    while True:
        if tph_sensor.is_ready:
            success = tph_sensor.measure()
            if success:
                print(
                    f"{tph_sensor.temperature:4.1f} [deg. C] {tph_sensor.pressure:6.1f} [hPa] {tph_sensor.humidity:5.1f} [%]"
                )
            else:
                print("Failed to measure temperature, etc.")
        else:
            print("temp sensor is not ready.")
            tph_sensor.connect()

        if pm.is_ready:
            success = pm.measure()
            if success:
                print(
                    f"{pm.voltage:4.1f} [V] {pm.current:+6.3f} [A] {pm.power:6.3f} [W] {pm.accumulated_power:6.3f} [Wh] {100*pm.remaining_capacity_ratio:3.0f} [%] {pm.accumulated_time/60:6.1f} [min]"
                )
            else:
                print("Failed to measure current, etc.")
        else:
            print("current sensor is not ready.")
            if pm.connect():
                pm.reset_measurement(7.2 * 2 * 5)

        if ds._is_ready:
            if ds.measure():
                print("{ds.depth:7.3f} [m] {ds.temperature:4.1f} [deg. C]")
            else:
                print("Failed to measure depth.")
        else:
            print("depth sensor is not ready.")
            ds.connect()

        time.sleep(1)
