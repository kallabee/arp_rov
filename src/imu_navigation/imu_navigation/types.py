from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Dof9Sample:
    gyro_x: float
    gyro_y: float
    gyro_z: float
    accel_x: float
    accel_y: float
    accel_z: float
    mag_x: float
    mag_y: float
    mag_z: float


@dataclass
class ImuNavSnapshotData:
    seq: int = 0
    # Wall time (Unix seconds) when IMU read completed; primary time for this snapshot.
    stamp_sec: float = 0.0
    # When depth I2C read completed this tick; NaN if no depth sample.
    depth_sample_time_sec: float = float("nan")
    gyro_x: float = float("nan")
    gyro_y: float = float("nan")
    gyro_z: float = float("nan")
    accel_x: float = float("nan")
    accel_y: float = float("nan")
    accel_z: float = float("nan")
    mag_x: float = float("nan")
    mag_y: float = float("nan")
    mag_z: float = float("nan")
    ori_x: float = float("nan")
    ori_y: float = float("nan")
    ori_z: float = float("nan")
    ori_w: float = float("nan")
    pose_x: float = float("nan")
    pose_y: float = float("nan")
    pose_z: float = float("nan")
    vel_x: float = float("nan")
    vel_y: float = float("nan")
    vel_z: float = float("nan")
    depth_m: float = float("nan")
    depth_temp_c: float = float("nan")
    depth_pressure_atm: float = float("nan")
