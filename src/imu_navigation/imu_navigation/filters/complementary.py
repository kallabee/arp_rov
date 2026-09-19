from __future__ import annotations

import math
from dataclasses import dataclass

from imu_navigation.types import Dof9Sample


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


@dataclass
class ComplementaryFilter:
    alpha: float = 0.98
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def update(self, s: Dof9Sample, dt: float) -> tuple[float, float, float, float]:
        self.roll = _wrap_pi(self.roll + s.gyro_x * dt)
        self.pitch = _wrap_pi(self.pitch + s.gyro_y * dt)
        self.yaw = _wrap_pi(self.yaw + s.gyro_z * dt)

        ax, ay, az = s.accel_x, s.accel_y, s.accel_z
        if not (ax == 0 and ay == 0 and az == 0):
            roll_acc = math.atan2(ay, az)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
            self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_acc
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc

        mx, my, mz = s.mag_x, s.mag_y, s.mag_z
        if not (mx == 0 and my == 0 and mz == 0):
            cr = math.cos(self.roll)
            sr = math.sin(self.roll)
            cp = math.cos(self.pitch)
            sp = math.sin(self.pitch)
            mx2 = mx * cp + mz * sp
            my2 = mx * sr * sp + my * cr - mz * sr * cp
            yaw_mag = math.atan2(-my2, mx2)
            self.yaw = self.alpha * self.yaw + (1 - self.alpha) * yaw_mag

        return _quat_from_rpy(self.roll, self.pitch, self.yaw)
