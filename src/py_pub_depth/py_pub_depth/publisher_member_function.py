# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from depth_sensor_interfaces.msg import Depth

from py_pub_depth import ms5837


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Depth, "topic", 30)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sensor = ms5837.MS5837_30BA(bus=6)  # Specify I2C bus

        # We must initialize the sensor before reading it
        if not self.sensor.init():
            print("Sensor could not be initialized")
            exit(1)

        self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)

    def timer_callback(self):
        depth = Depth()

        # We have to read values from sensor to update pressure and temperature
        if not self.sensor.read():
            print("Sensor read failed!")
            exit(1)

        depth.pressure = self.sensor.pressure()  # [hPa=mbar=millibar]
        depth.depth = self.sensor.depth()  # [m]
        depth.temperature = self.sensor.temperature()  # [deg. Celsuis]

        self.publisher_.publish(depth)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
