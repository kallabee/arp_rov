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


import os
import time

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from camera_actuator_interfaces.msg import CameraActuator
from hand_actuator_interfaces.msg import HandActuator

from adafruit_servokit import ServoKit

from thruster_controller.FocuserWrapper import FocuserWrapper
from thruster_controller.Focuser import Focuser

from thruster_controller import thruster_rot_conv


def conv_twist_to_np(t: Twist) -> np.array:
    # t is supposed to be a vector, [x, y, z, wx, wy, wz]
    a = np.array(
        [
            t.linear.x,
            t.linear.y,
            t.linear.z,
            t.angular.x,
            t.angular.y,
            t.angular.z,
        ]
    )
    return a


def conv_hand_act_to_np(t: HandActuator) -> np.array:
    # a = np.array([t.grab, t.roll])
    a = np.array([t.roll, t.grab])
    return a


class CamActController:
    def __init__(
        self,
        i2c_channel=6,
        pan_center=90,
        pan_min=-30,
        pan_max=30,
        tilt_center=90,
        tilt_min=-30,
        tilt_max=30,
    ):
        self.focuser = FocuserWrapper(
            i2c_channel, pan_center, pan_min, pan_max, tilt_center, tilt_min, tilt_max
        )

        self.focus_min = 1800
        self.zoom_min = 2400

    def move(self, ca: CameraActuator):
        print(
            f"pan {ca.pan:+6.1f}, tilt {ca.tilt:+6.1f}, focus {ca.focus:4.2f}, zoom {ca.zoom:4.2f}, ir_cut {ca.ir_cut:1d}"
        )

        self.move_pan(ca.pan)
        self.move_tilt(ca.tilt)
        self.move_focus(ca.focus)
        self.move_zoom(ca.zoom)
        self.change_ir_cut(ca.ir_cut)

    def move_pan(self, pan: float):
        p = -90 * pan  # [-90, 90], [deg]
        self.focuser.set(Focuser.OPT_MOTOR_X, p)

    def move_tilt(self, tilt: float):
        t = -90 * tilt  # [-90, 90], [deg]
        self.focuser.set(Focuser.OPT_MOTOR_Y, t)

    def move_focus(self, focus: float):
        f = int((20000 - self.focus_min) * (1 - focus) + self.focus_min)
        self.focuser.set(Focuser.OPT_FOCUS, f)

    def move_zoom(self, zoom: float):
        z = int((20000 - self.zoom_min) * zoom + self.zoom_min)
        self.focuser.set(Focuser.OPT_ZOOM, z)

    def change_ir_cut(self, ir_cut):
        self.focuser.set(Focuser.OPT_IRCUT, ir_cut == 1)


class ActuatorSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")

        node_name = "turtle1"

        self.subscription = self.create_subscription(
            Twist, f"{node_name}/cmd_vel", self.listener_callback_thruster, 10
        )
        self.subscription = self.create_subscription(
            HandActuator, f"{node_name}/hand_act", self.listener_callback_hand_act, 10
        )
        self.subscription = self.create_subscription(
            CameraActuator, f"{node_name}/cam_act", self.listener_callback_cam_act, 10
        )
        self.subscription  # prevent unused variable warning

        timer_period = 0.01  # [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.no_input_duration = 0.05  # [s], When this time elapsed without new command input, start to decay last command amount.
        self.stop_duration = 0.3  # [s]
        # self.latest_cmd_vel_receive_time = time.perf_counter()
        self.nidh_cmd_vel = no_input_duration_handler(
            self, self.no_input_duration, self.stop_duration
        )

        param_path = os.path.join(os.path.dirname(__file__), "thruster_param.yaml")
        self.tc = thruster_rot_conv.thruster_rot_conv(
            thruster_rot_conv.thruster_rot_conv.load_param(param_path)
        )

        # self.enable_thrusters = True
        self.enable_thrusters = (
            False  # Use this line when you want to debug only camera actuators.
        )

        if self.enable_thrusters:
            self.kit = ServoKit(address=0x60, channels=16)
            min_pulse = 1000  # [ms]
            max_pulse = 2000
            offset_pulse = -60
            # for i in range(self.tc.get_num()):
            for i in range(8):
                self.kit.continuous_servo[i].set_pulse_width_range(
                    min_pulse + offset_pulse, max_pulse + offset_pulse
                )
            self.apply_thrustors(np.zeros((6), dtype=np.float32))

        self.cac = CamActController(6, 90, -45, 45, 90, -45, 45)

    def apply_thrustors(self, ts: np.array) -> None:
        # print("apply_thrustors is called.")
        if self.enable_thrusters:
            for i, t in enumerate(ts):
                self.kit.continuous_servo[i].throttle = t

    def handle_hand_acts(self, ha: HandActuator):
        t = conv_hand_act_to_np(ha)
        self.get_logger().info(f"grab {ha.grab} , roll {ha.roll}")
        if self.enable_thrusters:
            for i in range(2):
                self.kit.continuous_servo[i + 6].throttle = t[i]

    def handle_cam_acts(self, ca: CameraActuator):
        self.cac.move(ca)

    def listener_callback_thruster(self, msg: Twist):
        print("listener_callback_thruster is called.")
        ts = self.tc.to_all_thrusters(conv_twist_to_np(msg))
        self.nidh_cmd_vel.update_cmd(ts)

        self.apply_thrustors(ts)
        self.get_logger().info(f"Converted from {msg} to {ts}")

    def listener_callback_hand_act(self, msg: HandActuator):
        self.latest_hand_act_receive_time = time.perf_counter()
        # ts = self.tc.to_thruster(conv_hand_act_to_np(msg))
        # self.latest_hand_act = ts

        self.handle_hand_acts(msg)
        # self.get_logger().info(f"Converted from {msg.data} to {ts}")

    def listener_callback_cam_act(self, msg: CameraActuator):
        self.handle_cam_acts(msg)
        # self.get_logger().info(f"Converted from {msg.data} to {ts}")

    def timer_callback(self):
        cmd = self.nidh_cmd_vel.handle()
        if cmd is not None:
            self.apply_thrustors(cmd)

        # now = time.perf_counter()
        # elapsed = now - self.latest_cmd_vel_receive_time

        # if elapsed > self.no_input_duration and hasattr(self, "latest_command"):
        #     self.get_logger().info(f"No input is received. So attenuate input.  ")
        #     t = elapsed - self.no_input_duration
        #     if t < self.stop_duration:
        #         ratio = t / self.stop_duration
        #     else:
        #         ratio = 0
        #     new_ts = ratio * self.latest_command
        #     self.apply_thrustors(new_ts)
        # else:
        #     # Do nothing.
        #     pass


class no_input_duration_handler:
    def __init__(self, node: Node, no_input_duration: float, stop_duration: float):
        self.node = node
        self.no_input_duration = no_input_duration
        self.stop_duration = stop_duration

    def update_cmd(self, cmd) -> None:
        now = time.perf_counter()
        self.latest_cmd_vel_receive_time = now
        self.latest_command = cmd

    def handle(self):
        if hasattr(self, "latest_command"):
            now = time.perf_counter()
            elapsed = now - self.latest_cmd_vel_receive_time

            if elapsed > self.no_input_duration:
                t = elapsed - self.no_input_duration
                if t < self.stop_duration:
                    ratio = (self.stop_duration - t) / self.stop_duration
                    # self.node.get_logger().info(
                    #     f"No input is received. So attenuate last input with ratio {ratio:.1f}."
                    # )
                else:
                    ratio = 0
                new_ts = ratio * self.latest_command
                return new_ts
        else:
            # Do nothing.
            return None


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ActuatorSubscriber()

    print("Started.")

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
