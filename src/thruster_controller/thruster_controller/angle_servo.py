import time
from adafruit_servokit import ServoKit
from camera_actuator_interfaces.msg import CameraActuator


class ServoAngleRegulator:
    def __init__(
        self,
        center: float,
        gain: float,
        min: float,
        max: float,
        final_min: float = 0,
        final_max: float = 180,
    ) -> None:
        """
        Args:
            center: after regulation [deg]
            gain: including positive or negative sign []
            min: before regulation [deg]
            max: before regulation [deg]
            final_min: [deg]
            final_max: [deg]
        """
        self.center = center
        self.gain = gain
        self.min = min
        self.max = max
        self.final_min = final_min
        self.final_max = final_max

    def regulate(self, b: float) -> float:
        if b < self.min:
            b = self.min
        elif b > self.max:
            b = self.max

        f = b / self.gain + self.center

        if f < self.final_min:
            f = self.final_min
        elif f > self.final_max:
            f = self.final_max

        print(f"f = {f:5.1f}")

        return f


class CamActControllerPT:
    def __init__(
        self,
        i2c_channel=6,
        i2c_addr=0x60,
        pan_center=90,
        pan_gain=90 / 180,
        pan_min=-45,
        pan_max=45,
        tilt_center=90,
        tilt_gain=75 / 180,
        tilt_min=-45,
        tilt_max=45,
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

        self.pan_reg = ServoAngleRegulator(pan_center, pan_gain, pan_min, pan_max)
        self.tilt_reg = ServoAngleRegulator(tilt_center, tilt_gain, tilt_min, tilt_max)

        if use_cam_act:
            self.reset_servo(i2c_addr)
        self.use_cam_act = use_cam_act

    def reset_servo(self, addr: int) -> None:
        kit = ServoKit(address=addr, channels=16)
        min_pulse = 1000  # [us]
        max_pulse = 2000  # [us]
        offset_pulse = 0  # [us]

        for i in range(2):
            kit.servo[i].set_pulse_width_range(
                min_pulse + offset_pulse, max_pulse + offset_pulse
            )
        self.servo = kit

    def move(self, ca: CameraActuator):
        print(f"Camera : pan {ca.pan:+6.1f}, tilt {ca.tilt:+6.1f}")

        # ToDo modify
        r = 45

        if self.use_cam_act:
            self.move_pan(r * ca.pan)
            self.move_tilt(r * ca.tilt)

    def move_pan(self, pan: float):
        self.servo.servo[0].angle = self.pan_reg.regulate(pan)

    def move_tilt(self, tilt: float):
        self.servo.servo[1].angle = self.tilt_reg.regulate(tilt)
