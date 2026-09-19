from __future__ import annotations

import math
from dataclasses import dataclass

from imu_navigation.types import Dof9Sample


def _inv_sqrt(x: float) -> float:
    return 1.0 / math.sqrt(x) if x > 0 else 0.0


@dataclass
class MadgwickFilter:
    beta: float = 0.1
    q0: float = 1.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0

    def update(self, s: Dof9Sample, dt: float) -> tuple[float, float, float, float]:
        gx, gy, gz = s.gyro_x, s.gyro_y, s.gyro_z
        ax, ay, az = s.accel_x, s.accel_y, s.accel_z
        mx, my, mz = s.mag_x, s.mag_y, s.mag_z

        norm = ax * ax + ay * ay + az * az
        if norm <= 0:
            return (self.q1, self.q2, self.q3, self.q0)
        inv = _inv_sqrt(norm)
        ax *= inv
        ay *= inv
        az *= inv

        norm = mx * mx + my * my + mz * mz
        if norm <= 0:
            return (self.q1, self.q2, self.q3, self.q0)
        inv = _inv_sqrt(norm)
        mx *= inv
        my *= inv
        mz *= inv

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        _2q0mx = 2.0 * q0 * mx
        _2q0my = 2.0 * q0 * my
        _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q0q2 = 2.0 * q0 * q2
        _2q2q3 = 2.0 * q2 * q3
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        hx = (
            mx * q0q0
            - _2q0my * q3
            + _2q0mz * q2
            + mx * q1q1
            + _2q1 * my * q2
            + _2q1 * mz * q3
            - mx * q2q2
            - mx * q3q3
        )
        hy = (
            _2q0mx * q3
            + my * q0q0
            - _2q0mz * q1
            + _2q1mx * q2
            - my * q1q1
            + my * q2q2
            + _2q2 * mz * q3
            - my * q3q3
        )
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        s0 = (
            -_2q2 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
            - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )
        s1 = (
            _2q3 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
            - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
            + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )
        s2 = (
            -_2q0 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
            - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
            + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )
        s3 = (
            _2q1 * (2.0 * q1q3 - _2q0q2 - ax)
            + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
            + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
            + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
            + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
        )

        norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3
        inv = _inv_sqrt(norm) if norm > 0 else 0.0
        s0 *= inv
        s1 *= inv
        s2 *= inv
        s3 *= inv

        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * s1
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * s2
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * s3

        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt

        norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3
        inv = _inv_sqrt(norm) if norm > 0 else 1.0
        self.q0, self.q1, self.q2, self.q3 = q0 * inv, q1 * inv, q2 * inv, q3 * inv

        return (self.q1, self.q2, self.q3, self.q0)
