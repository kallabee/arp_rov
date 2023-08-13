import rclpy
from rclpy.node import Node
import time
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from inner_hull_status_interfaces.msg import InnerHullStatus


from adafruit_bme280 import basic as adafruit_bme280
from adafruit_blinka.microcontroller.bcm2711.pin import *


class water_leak_detector:
    def __init__(self, i2c, water_leak_volt_threshold: float):
        # Create the ADC object using the I2C bus
        ads = ADS.ADS1015(i2c)

        # Create single-ended input on channel 0
        self.chan = AnalogIn(ads, ADS.P0)

        self.water_leak_volt_threshold = water_leak_volt_threshold

    def check(self) -> [bool, float]:
        water_leak_1 = self.chan.voltage < self.water_leak_volt_threshold
        water_leak_1_volt = self.chan.voltage
        return water_leak_1, water_leak_1_volt


class InnerHullStatusPub(Node):
    def __init__(self):
        super().__init__("inner_hull_status")
        history_count = 10
        self.inner_hull_status_pub = self.create_publisher(
            InnerHullStatus, "rov/inner_hull_status", history_count
        )
        self.timer = self.create_timer(2.0, self.publish)

        water_leak_volt_threshold = 3.0  # [V]

        # Create the I2C bus at ch1
        i2c = busio.I2C(board.SCL, board.SDA)
        self.water_leak_detector = water_leak_detector(
            i2c, water_leak_volt_threshold=water_leak_volt_threshold
        )

        # Create sensor object, using the board's default I2C bus.
        i2c_ch6 = busio.I2C(D23, D22)  # I2C ch6
        self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c_ch6)

    def publish(self):
        s = InnerHullStatus()

        s.water_leak_1, s.water_leak_1_volt = self.water_leak_detector.check()

        s.temperature_1 = self.bme280.temperature
        s.barometric_pressure = self.bme280.pressure
        s.humidity = self.bme280.relative_humidity

        print(print_inner_hull_status(s))
        self.inner_hull_status_pub.publish(s)


def print_inner_hull_status(s: InnerHullStatus) -> str:
    msg = f"water_leak:{s.water_leak_1}, water_leak_volt[V]:{s.water_leak_1_volt:3.1f}"
    msg += f", temp1[C.deg]:{s.temperature_1:5.1f}, press[hPa]:{s.barometric_pressure:6.1f}, humid[%]:{s.humidity:2.0f}"
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = InnerHullStatusPub()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    pass
