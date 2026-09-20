#!/usr/bin/env bash
# Source ROS + workspace, then launch IMU + monitor pub + logger + web + camera.
# Prefer unit_scripts/start_dashboard.sh — it refuses duplicate launches and can restart.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
cd "${WS}"
exec ros2 launch monitor_value_pub rov_monitor_imu_and_logging.launch.py "$@"
