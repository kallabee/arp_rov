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

from thruster_controller import angle_servo

# from adafruit_servokit import ServoKit


num_ch = 10


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
    a = np.array([-t.roll, t.grab])
    return a


class CamActControllerPTZ:
    def __init__(
        self,
        i2c_channel=6,
        pan_center=90,
        pan_min=-30,
        pan_max=30,
        tilt_center=90,
        tilt_min=-30,
        tilt_max=30,
        use_cam_act=False,
    ):
        """

        Args:
            i2c_channel (int, optional): Defaults to 6.
            pan_center (int, optional): [degree]. Defaults to 90.
            pan_min (int, optional): [degree]. Defaults to -30.
            pan_max (int, optional): [degree]. Defaults to 30.
            tilt_center (int, optional): [degree]. Defaults to 90.
            tilt_min (int, optional): [degree]. Defaults to -30.
            tilt_max (int, optional): [degree]. Defaults to 30.
            use_cam_act (bool, optional): When you want to debug this package without camera interfaces connection, set this to False. Defaults to False.
        """
        if use_cam_act:
            self.focuser = FocuserWrapper(
                i2c_channel,
                pan_center,
                pan_min,
                pan_max,
                tilt_center,
                tilt_min,
                tilt_max,
            )
        self.use_cam_act = use_cam_act

        self.focus_min = 1800  # [] (register value)
        self.zoom_min = 2400  # [] (register value)

    def move(self, ca: CameraActuator):
        print(
            f"Camera : pan {ca.pan:+6.1f}, tilt {ca.tilt:+6.1f}, focus {ca.focus:4.2f}, zoom {ca.zoom:4.2f}, ir_cut {ca.ir_cut:1d}"
        )

        if self.use_cam_act:
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
        history_depth = 10

        self.subscription = self.create_subscription(
            Twist,
            f"{node_name}/cmd_vel",
            self.listener_callback_thruster,
            history_depth,
        )
        self.subscription = self.create_subscription(
            HandActuator,
            f"{node_name}/hand_act",
            self.listener_callback_hand_act,
            history_depth,
        )
        self.subscription = self.create_subscription(
            CameraActuator,
            f"{node_name}/cam_act",
            self.listener_callback_cam_act,
            history_depth,
        )
        self.subscription  # prevent unused variable warning

        timer_period = 0.01  # [s]
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.no_input_duration = 0.05  # [s], When this time elapsed without new command input, start to decay last command amount.
        self.stop_duration = 0.3  # [s]

        self.nidh_cmd_vel = no_input_duration_handler(
            self, self.no_input_duration, self.stop_duration
        )

        self.stop_duration_hand_act = 0.1  # [s]
        self.nidh_hand_acts = no_input_duration_handler(
            self, self.no_input_duration, self.stop_duration_hand_act
        )

        param_path = os.path.join(os.path.dirname(__file__), "thruster_param.yaml")
        self.tc = thruster_rot_conv.thruster_rot_conv(
            thruster_rot_conv.thruster_rot_conv.load_param(param_path)
        )

        # Set this value False when you want to debug only camera actuators.
        self.enable_thrusters = True
        # self.enable_thrusters = False

        self.reset_servo()

        pan_max = 45
        tilt_max = 45
        # self.cac = CamActControllerPTZ(6, 90, -pan_max, pan_max, 90, -tilt_max, tilt_max)
        self.cac = angle_servo.CamActControllerPT(
            i2c_channel=6,
            i2c_addr=0x63,
            pan_center=90,
            pan_gain=90 / 180,
            pan_min=-pan_max,
            pan_max=pan_max,
            tilt_center=56,
            tilt_gain=75 / 180,
            tilt_min=-tilt_max,
            tilt_max=tilt_max,
            use_cam_act=True,
        )

    def reset_servo(self) -> None:
        if self.enable_thrusters:
            addr = 0x60  # No.1
            # addr = 0x61  # No.3
            self.kit = ServoKit(address=addr, channels=16)
            min_pulse = 1000  # [ms]
            max_pulse = 2000  # [ms]
            offset_pulse = -60  # [ms]
            # for i in range(self.tc.get_num()):
            for i in range(6 + 2):
                self.kit.continuous_servo[i].set_pulse_width_range(
                    min_pulse + offset_pulse, max_pulse + offset_pulse
                )

                # Initialize ESC with neutral pulse width.
                self.kit.continuous_servo[i].throttle = 0
            time.sleep(8)
            self.apply_thrustors(np.zeros((6), dtype=np.float32))

    def set_pwm(self, ch: int, t: float) -> None:
        # When servo circuit module turns down when someone touchs it, OSError occurs.
        # Because it stops this program and it is not good, accept this to happen and don't stop the program.
        try:
            self.kit.continuous_servo[ch].throttle = t
        except OSError as e:
            print(f"{e} at set_pwm()")
            self.reset_servo()

    def apply_thrustors(self, ts: np.array) -> None:
        # self.get_logger().info(
        #     f"PWM : {ts[0]:+4.1f}, {ts[1]:+4.1f}, {ts[2]:+4.1f}, {ts[3]:+4.1f}, {ts[4]:+4.1f}, {ts[5]:+4.1f}"
        # )
        if self.enable_thrusters:
            for i, t in enumerate(ts):
                self.set_pwm(i, t)

    def apply_hand_acts(self, ha: np.array) -> None:
        if self.enable_thrusters:
            for i in range(2):
                self.set_pwm(i + 6, ha[i])

    def apply_cam_acts(self, ca: CameraActuator) -> None:
        print("apply_cam_acts called.")
        self.cac.move(ca)

    def listener_callback_thruster(self, msg: Twist):
        ts = self.tc.to_all_thrusters(conv_twist_to_np(msg))
        self.get_logger().info(
            f"Thruster : {ts[0]:+4.1f}, {ts[1]:+4.1f}, {ts[2]:+4.1f}, {ts[3]:+4.1f}, {ts[4]:+4.1f}, {ts[5]:+4.1f}"
        )

        self.nidh_cmd_vel.update_cmd(ts)
        self.apply_thrustors(ts)

    def listener_callback_hand_act(self, msg: HandActuator):
        self.get_logger().info(f"Hand : grab {msg.grab:+4.1f} , roll {msg.roll:+4.1f}")

        ha = conv_hand_act_to_np(msg)
        print("hoge")
        self.nidh_hand_acts.update_cmd(ha)
        self.apply_hand_acts(ha)

    def listener_callback_cam_act(self, msg: CameraActuator):
        self.apply_cam_acts(msg)

    def timer_callback(self):
        cmd = self.nidh_cmd_vel.calc_cmd()
        if cmd is not None:
            self.apply_thrustors(cmd)

        cmd_hand = self.nidh_hand_acts.calc_cmd()
        if cmd_hand is not None:
            self.apply_hand_acts(cmd_hand)

        # Because camera actuators like pan, tilt, zoom, focus are positions,
        #   not speeds like thrusters, we don't have to decay them when commands are not received.


class no_input_duration_handler:
    def __init__(self, node: Node, no_input_duration: float, stop_duration: float):
        self.node = node
        self.no_input_duration = no_input_duration
        self.stop_duration = stop_duration

    def update_cmd(self, cmd) -> None:
        now = time.perf_counter()
        self.latest_cmd_vel_receive_time = now
        self.latest_command = cmd

    def calc_cmd(self):
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
                return self.latest_command
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
