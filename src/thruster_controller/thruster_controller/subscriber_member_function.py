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
from pathlib import Path

import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from camera_actuator_interfaces.msg import CameraActuator
from hand_actuator_interfaces.msg import HandActuator
from light_actuator_interfaces.msg import LightActuator

from adafruit_servokit import ServoKit

from device_registry import DeviceRegistry

from thruster_controller import thruster_rot_conv
from thruster_controller.cam_act_config import CamActPTConfig
from thruster_controller.cam_act_controllers import CamActControllerBase, CamActControllerPT
from thruster_controller.light_pwm import open_light_pwm_driver

# from adafruit_servokit import ServoKit


num_ch = 10


def _resolve_cam_act_pt_yaml() -> Path:
    """Prefer workspace src YAML during development, fallback to installed package file."""
    # 1) Walk up from cwd: <ws>/src/thruster_controller/thruster_controller/cam_act_pt.yaml
    cwd = Path.cwd().resolve()
    for base in (cwd, *cwd.parents):
        cand = (
            base
            / "src"
            / "thruster_controller"
            / "thruster_controller"
            / "cam_act_pt.yaml"
        )
        if cand.is_file():
            return cand

    # 2) Fallback to the YAML next to this module (installed layout).
    return Path(__file__).resolve().parent / "cam_act_pt.yaml"


def _resolve_thruster_pwm_yaml() -> Path:
    """Prefer workspace src YAML during development, fallback to installed package file."""
    cwd = Path.cwd().resolve()
    for base in (cwd, *cwd.parents):
        cand = (
            base
            / "src"
            / "thruster_controller"
            / "thruster_controller"
            / "thruster_pwm.yaml"
        )
        if cand.is_file():
            return cand
    return Path(__file__).resolve().parent / "thruster_pwm.yaml"


def _load_thruster_pwm_config(path: Path) -> tuple[list[int], list[int], int, int, int, float]:
    import yaml

    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"Top-level YAML must be a mapping: {path}")
    channels = raw.get("channels")
    pulse = raw.get("pulse")
    if not isinstance(channels, dict):
        raise ValueError(f"{path}: 'channels' must be a mapping")
    if not isinstance(pulse, dict):
        raise ValueError(f"{path}: 'pulse' must be a mapping")

    thr = channels.get("thrusters")
    hand = channels.get("hand")
    if not (isinstance(thr, list) and len(thr) == 6 and all(isinstance(x, int) for x in thr)):
        raise ValueError(f"{path}: 'thrusters' must be a list of 6 ints (0..15)")
    if not (isinstance(hand, list) and len(hand) == 2 and all(isinstance(x, int) for x in hand)):
        raise ValueError(f"{path}: 'hand' must be a list of 2 ints (0..15)")
    for x in [*thr, *hand]:
        if x < 0 or x > 15:
            raise ValueError(f"{path}: channel must be 0..15, got {x}")
    if len(set([*thr, *hand])) != 8:
        raise ValueError(f"{path}: thrusters+hand channels must be unique: {thr}+{hand}")

    min_us = int(pulse.get("min_us", 1000))
    max_us = int(pulse.get("max_us", 2000))
    off_us = int(pulse.get("offset_us", 0))
    arm_delay_s = float(pulse.get("arm_delay_s", 8.0))
    if min_us <= 0 or max_us <= 0 or min_us >= max_us:
        raise ValueError(f"{path}: invalid min/max pulse: {min_us}, {max_us}")
    if arm_delay_s < 0:
        raise ValueError(f"{path}: arm_delay_s must be >= 0")
    return thr, hand, min_us, max_us, off_us, arm_delay_s


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
        self.subscription = self.create_subscription(
            LightActuator,
            f"{node_name}/lights",
            self.listener_callback_lights,
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

        reg = DeviceRegistry.from_src_default()

        # Thruster PWM board (PCA9685) for continuous servos / ESCs.
        d_thr = reg.get_i2c("thruster_pwm")
        self.thruster_pwm_addr = d_thr.addr
        thr_pwm_yaml = _resolve_thruster_pwm_yaml()
        (
            self.thruster_channels,
            self.hand_channels,
            self.thr_min_pulse_us,
            self.thr_max_pulse_us,
            self.thr_offset_pulse_us,
            self.thr_arm_delay_s,
        ) = _load_thruster_pwm_config(thr_pwm_yaml)

        self.reset_servo()

        pt_yaml = _resolve_cam_act_pt_yaml()
        pt_cfg = CamActPTConfig.load(pt_yaml)
        d = reg.get_i2c("cam_act")
        if d.linux_bus is None:
            raise ValueError("device cam_act requires 'bus' in device_i2c.yaml")
        # PTZ: load cam_act_ptz.yaml and use CamActControllerPTZ(ptz_cfg, linux_bus=d.linux_bus)
        self.cac: CamActControllerBase = CamActControllerPT(
            pt_cfg, linux_bus=d.linux_bus, i2c_addr=d.addr
        )

        self.light_pwm = None
        try:
            self.light_pwm = open_light_pwm_driver()
            self.light_pwm.all_off()
        except Exception as exc:
            self.get_logger().warn(f"Light PWM disabled: {exc}")

    def reset_servo(self) -> None:
        if self.enable_thrusters:
            self.kit = ServoKit(address=self.thruster_pwm_addr, channels=16)
            min_pulse = self.thr_min_pulse_us  # [us]
            max_pulse = self.thr_max_pulse_us  # [us]
            offset_pulse = self.thr_offset_pulse_us  # [us]
            for i in self.thruster_channels + self.hand_channels:
                self.kit.continuous_servo[i].set_pulse_width_range(
                    min_pulse + offset_pulse, max_pulse + offset_pulse
                )

                # Initialize ESC with neutral pulse width.
                self.kit.continuous_servo[i].throttle = 0
            time.sleep(self.thr_arm_delay_s)
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
                self.set_pwm(self.thruster_channels[i], t)

    def apply_hand_acts(self, ha: np.array) -> None:
        if self.enable_thrusters:
            for i in range(2):
                self.set_pwm(self.hand_channels[i], ha[i])

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

    def listener_callback_lights(self, msg: LightActuator):
        if self.light_pwm is None:
            return
        n = min(len(msg.names), len(msg.duties))
        self.get_logger().info(
            "Lights: " + ", ".join(f"{msg.names[i]}={msg.duties[i]:.2f}" for i in range(n))
        )
        try:
            self.light_pwm.apply(list(msg.names), list(msg.duties))
        except Exception as exc:
            self.get_logger().warn(f"Light PWM apply failed: {exc}")

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
