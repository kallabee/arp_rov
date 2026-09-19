from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Dof9Sample:
    # Gyro [rad/s]
    gyro_x: float
    gyro_y: float
    gyro_z: float

    # Accel [m/s^2]
    accel_x: float
    accel_y: float
    accel_z: float

    # Mag [uT] or arbitrary units (filter only needs consistent scale)
    mag_x: float
    mag_y: float
    mag_z: float

