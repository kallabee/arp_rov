from thruster_controller.Focuser import Focuser
import time


class FocuserWrapper(Focuser):
    def __init__(
        self, bus, pan_center, pan_min, pan_max, tilt_center, tilt_min, tilt_max
    ):
        self.pan_center = pan_center
        self.pan_max = pan_max
        self.pan_min = pan_min
        self.tilt_center = tilt_center
        self.tilt_max = tilt_max
        self.tilt_min = tilt_min

        super().__init__(bus)

        time.sleep(0.02)
        self.pan = super().get(Focuser.OPT_MOTOR_X)
        self.tilt = super().get(Focuser.OPT_MOTOR_Y)

    def get(self, opt, flag=0):
        if opt == Focuser.OPT_MOTOR_X:
            return self.pan
        elif opt == Focuser.OPT_MOTOR_Y:
            return self.tilt
        else:
            return super().get(opt, flag)

    def set(self, opt, value, flag=1):
        org_value = value
        if opt == Focuser.OPT_MOTOR_X:
            if value < self.pan_min:
                value = self.pan_min
            elif value > self.pan_max:
                value = self.pan_max
            self.pan = value
            value += self.pan_center

            print(f"pan org:{org_value:+3.0f}, new:{value:+3.0f}")

        elif opt == Focuser.OPT_MOTOR_Y:
            if value < self.tilt_min:
                value = self.tilt_min
            elif value > self.tilt_max:
                value = self.tilt_max
            self.tilt = value
            value += self.tilt_center

            print(f"tilt org:{org_value:+3.0f}, new:{value:+3.0f}")

        elif opt == Focuser.OPT_ZOOM:
            print(f"zoom {value:4d}")

        elif opt == Focuser.OPT_FOCUS:
            print(f"focus {value:4d}")

        value = int(value)

        super().set(opt, value, flag)
