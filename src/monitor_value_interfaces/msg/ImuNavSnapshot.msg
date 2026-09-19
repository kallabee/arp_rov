# High-rate fused navigation snapshot (imu_navigation → monitor_value_pub / logger)
uint64 seq
# Message stamp = IMU sample time (wall clock when acc/gyro/mag read completed on host).
builtin_interfaces/Time stamp

# Wall-clock Unix seconds when each sensor finished its I2C read (host time; use for skew / sync).
# depth_sample_time_sec is NaN if depth was not read this fusion tick or read failed.
float64 imu_sample_time_sec
float64 depth_sample_time_sec

# Raw / filtered body-frame IMU (gyro rad/s, accel m/s^2, mag arbitrary consistent units)
float64 gyro_x
float64 gyro_y
float64 gyro_z
float64 accel_x
float64 accel_y
float64 accel_z
float64 mag_x
float64 mag_y
float64 mag_z

# Orientation quaternion (x, y, z, w) in configured world frame
float64 ori_x
float64 ori_y
float64 ori_z
float64 ori_w

# Dead-reckoned position / velocity (low accuracy; meters / m/s in world frame)
float64 pose_x
float64 pose_y
float64 pose_z
float64 vel_x
float64 vel_y
float64 vel_z

# Depth sensor (MS5837)
float64 depth_m
float64 depth_temp_c
float64 depth_pressure_atm
