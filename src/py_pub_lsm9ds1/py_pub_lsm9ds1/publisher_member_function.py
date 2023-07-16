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

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from py_pub_lsm9ds1 import accelerometer
# import accelerometer


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.data_raw_pub = self.create_publisher(Imu, 'imu/data_raw', 1)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 1)
        # self.temp_pub = self.create_publisher(Float64, 'imu/temperature', 1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.device = accelerometer.Accelerometer()

    def timer_callback(self):
        # Get them at the same time.
        avx, avy, avz = self.device.readGyroData()
        ax, ay, az = self.device.readAccData()
        mx, my, mz = self.device.readMagData()

        imu = Imu()
        imu.angular_velocity.x = avx
        imu.angular_velocity.y = avy
        imu.angular_velocity.z = avz

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        
        mag = MagneticField()
        mag.magnetic_field.x=mx
        mag.magnetic_field.y=my
        mag.magnetic_field.z=mz

        self.data_raw_pub.publish(imu)
        self.mag_pub.publish(mag)

        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()