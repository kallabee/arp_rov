uint64 seq
builtin_interfaces/Time stamp
float64 elapsed_since_start_sec

# RPi
float64 rpi_fan_rpm
float64 rpi_cpu_temp_c
float64 rpi_cpu_util_percent
float64 rpi_gpu_util_percent

# 9DoF sensor
float64 mag_x
float64 mag_y
float64 mag_z
float64 gyro_x
float64 gyro_y
float64 gyro_z
float64 accel_x
float64 accel_y
float64 accel_z

# Pose estimate (from 9DoF; stub for now)
float64 pose_x
float64 pose_y
float64 pose_z
float64 ori_x
float64 ori_y
float64 ori_z
float64 ori_w

# Depth sensor (MS5837)
float64 depth_m
float64 depth_temp_c
float64 depth_pressure_atm

# BME280
float64 bme_temp_c
float64 bme_pressure_atm
float64 bme_humidity_percent

# Current/voltage/power
float64 current_a
float64 voltage_v
float64 power_w
float64 accumulated_energy_wh
float64 remaining_percent
float64 peak_power_w

# Thermocouple (AD8495 on ADS1015) [deg.C]
float64 thermocouple_ch2_temp_c
float64 thermocouple_ch3_temp_c

# DS18B20 1-Wire (per GPIO bus column) [deg.C]
float64 ds18b20_ch0_temp_c
float64 ds18b20_ch1_temp_c

# Water leak (ADS1015 ADC ch0/ch1)
float64 water_ch0_probe_v
bool water_ch0_detected
float64 water_ch1_probe_v
bool water_ch1_detected
