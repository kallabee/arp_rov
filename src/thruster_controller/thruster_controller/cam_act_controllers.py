"""Pan/tilt (and PTZ) camera actuator helpers shared by nodes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Protocol

from adafruit_servokit import ServoKit

if TYPE_CHECKING:
    # Import only for type checking; runtime import requires ROS message support.
    from camera_actuator_interfaces.msg import CameraActuator


class CameraActuatorLike(Protocol):
    pan: float
    tilt: float
    focus: float
    zoom: float
    ir_cut: int

from thruster_controller.cam_act_config import CamActPTConfig, CamActPTZConfig, LibcameraFocusConfig
from thruster_controller.Focuser import Focuser
from thruster_controller.FocuserWrapper import FocuserWrapper

__all__ = [
    "CamActControllerBase",
    "CamActControllerPT",
    "CamActControllerPTZ",
    "ServoAngleRegulator",
]


class CamActControllerBase(ABC):
    """Common contract: apply a ``CameraActuator`` message to hardware.

    Concrete backends: :class:`CamActControllerPT` (ServoKit pan/tilt) or
    :class:`CamActControllerPTZ` (Focuser pan/tilt/focus/zoom/IR).
    """

    use_cam_act: bool

    @abstractmethod
    def move(self, ca: CameraActuatorLike) -> None:
        """Execute pan/tilt (and optional zoom/focus) for ``ca``."""
        ...


class ServoAngleRegulator:
    def __init__(
        self,
        center: float,
        gain: float,
        sign: int,
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
        self.sign = sign
        self.min = min
        self.max = max
        self.final_min = final_min
        self.final_max = final_max

    def regulate(self, b: float) -> float:
        if b < self.min:
            b = self.min
        elif b > self.max:
            b = self.max

        f = (b * self.sign) / self.gain + self.center

        if f < self.final_min:
            f = self.final_min
        elif f > self.final_max:
            f = self.final_max

        print(f"f = {f:5.1f}")

        return f


class CamActControllerPT(CamActControllerBase):
    def __init__(
        self,
        config: CamActPTConfig,
        *,
        linux_bus: int,
        i2c_addr: int,
    ) -> None:
        self._config = config
        self._linux_bus = linux_bus
        self._i2c_addr = i2c_addr
        self.pan_reg = ServoAngleRegulator(
            config.pan.center,
            config.pan.gain,
            config.pan.sign,
            config.pan.min,
            config.pan.max,
        )
        self.tilt_reg = ServoAngleRegulator(
            config.tilt.center,
            config.tilt.gain,
            config.tilt.sign,
            config.tilt.min,
            config.tilt.max,
        )

        if config.use_cam_act:
            self.reset_servo(i2c_addr)
        self.use_cam_act = config.use_cam_act

        self._picam2 = None
        self._focus_cfg: LibcameraFocusConfig | None = getattr(config, "focus", None)
        if self._focus_cfg and self._focus_cfg.enabled:
            self._init_focus_controls(self._focus_cfg)

    def reset_servo(self, addr: int) -> None:
        kit = ServoKit(address=addr, channels=16)
        min_pulse = 1000  # [us]
        max_pulse = 2000  # [us]
        offset_pulse = 0  # [us]

        ch_pan = self._config.pan.channel
        ch_tilt = self._config.tilt.channel
        for i in sorted({ch_pan, ch_tilt}):
            kit.servo[i].set_pulse_width_range(
                min_pulse + offset_pulse, max_pulse + offset_pulse
            )
        self.servo = kit

        # Move to neutral immediately (b=0 => center).
        self.move_pan(0.0)
        self.move_tilt(0.0)

    def _init_focus_controls(self, focus_cfg: LibcameraFocusConfig) -> None:
        """Best-effort init of Picamera2 for AF/LensPosition controls."""
        try:
            from picamera2 import Picamera2  # type: ignore
        except Exception:
            # picamera2 not installed; keep optional.
            return

        try:
            self._picam2 = Picamera2()
            # Don't force a specific stream; start with default preview config.
            cfg = self._picam2.create_preview_configuration()
            self._picam2.configure(cfg)
            self._picam2.start()
        except Exception:
            self._picam2 = None
            return

        # Apply initial AF mode.
        self.set_af_mode(focus_cfg.af_mode)

    def set_af_mode(self, mode: str) -> None:
        """Set autofocus mode: continuous|auto|manual|off (if supported)."""
        if self._picam2 is None:
            return
        mode_l = mode.lower().strip()
        try:
            from libcamera import controls  # type: ignore

            mapping = {
                "continuous": controls.AfModeEnum.Continuous,
                "auto": controls.AfModeEnum.Auto,
                "manual": controls.AfModeEnum.Manual,
                "off": controls.AfModeEnum.Manual,  # treat off as manual w/ fixed lens
            }
            if mode_l not in mapping:
                return
            self._picam2.set_controls({"AfMode": mapping[mode_l]})
        except Exception:
            # libcamera controls not available; ignore
            return

    def set_lens_position(self, pos: float) -> None:
        """Set manual lens position (requires AfMode=Manual)."""
        if self._picam2 is None:
            return
        try:
            self._picam2.set_controls({"LensPosition": float(pos)})
        except Exception:
            return

    def move(self, ca: CameraActuatorLike) -> None:
        print(f"Camera : pan {ca.pan:+6.1f}, tilt {ca.tilt:+6.1f}")

        # ToDo modify
        r = 45

        if self.use_cam_act:
            self.move_pan(r * ca.pan)
            self.move_tilt(r * ca.tilt)

        # Optional focus mapping: ca.focus in [0,1] -> LensPosition.
        if self._picam2 is not None and self._focus_cfg and self._focus_cfg.enabled:
            if self._focus_cfg.af_mode in {"manual", "off"}:
                t = float(ca.focus)
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0
                lp = self._focus_cfg.lens_position_min + t * (
                    self._focus_cfg.lens_position_max - self._focus_cfg.lens_position_min
                )
                self.set_lens_position(lp)

    def move_pan(self, pan: float):
        self.servo.servo[self._config.pan.channel].angle = self.pan_reg.regulate(pan)

    def move_tilt(self, tilt: float):
        self.servo.servo[self._config.tilt.channel].angle = self.tilt_reg.regulate(tilt)


class CamActControllerPTZ(CamActControllerBase):
    def __init__(
        self,
        config: CamActPTZConfig,
        *,
        linux_bus: int,
    ) -> None:
        self._config = config
        self._linux_bus = linux_bus
        if config.use_cam_act:
            self.focuser = FocuserWrapper(
                linux_bus,
                config.pan.center,
                config.pan.min,
                config.pan.max,
                config.tilt.center,
                config.tilt.min,
                config.tilt.max,
            )
        self.use_cam_act = config.use_cam_act

        self.focus_min = config.focus_min
        self.zoom_min = config.zoom_min

    def move(self, ca: CameraActuatorLike) -> None:
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
