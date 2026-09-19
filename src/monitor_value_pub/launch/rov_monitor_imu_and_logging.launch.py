# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Bring up IMU (9DoF + fusion), unified monitor publishing, and CSV logging.

Nodes
-----
1. ``imu_navigation/imu_nav_node`` — reads LSM9DS1 (and optional depth) via ``FusionRunner``,
   publishes ``rov/imu_nav/snapshot`` and fused ``/imu/data``, ``/imu/dead_reckon_odom``.
2. ``monitor_value_pub/monitor_value_pub`` — samples RPi / BME / power / leak sensors,
   merges the latest ``ImuNavSnapshot``, publishes ``rov/monitor_value`` and republishes
   ``/imu/data_raw`` + ``/imu/mag`` (from merged values).
3. ``monitor_value_logger/monitor_value_logger`` — subscribes ``rov/monitor_value`` and writes CSV.
4. ``monitor_value_web/monitor_value_web`` — dashboard at ``http://<host>:8080/``.

Run (after ``colcon build`` + ``source install/setup.bash``)::

    ros2 launch monitor_value_pub rov_monitor_imu_and_logging.launch.py
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    imu_config = DeclareLaunchArgument(
        "imu_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("imu_navigation"), "config", "imu_navigation.yaml"]
        ),
        description="imu_navigation YAML passed to imu_nav_node (dof9 should be enabled for real hardware).",
    )
    monitor_value_config = DeclareLaunchArgument(
        "monitor_value_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("monitor_value_pub"), "config", "monitor_value.yaml"]
        ),
        description="monitor_value_lib YAML; imu_nav.snapshot_topic must match imu_nav_node.",
    )
    monitor_logger_config = DeclareLaunchArgument(
        "monitor_logger_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("monitor_value_logger"), "config", "monitor_value_logger.yaml"]
        ),
        description="monitor_value_logger YAML (CSV path, rotation, etc.).",
    )
    snapshot_topic = DeclareLaunchArgument(
        "snapshot_topic",
        default_value="rov/imu_nav/snapshot",
        description="ImuNavSnapshot topic; override if imu_nav_node uses a non-default name.",
    )
    monitor_web_config = DeclareLaunchArgument(
        "monitor_web_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("monitor_value_web"), "config", "monitor_value_web.yaml"]
        ),
        description="Dashboard YAML (port, temperature nicknames, gauge ranges).",
    )

    imu_nav = Node(
        package="imu_navigation",
        executable="imu_nav_node",
        name="imu_navigation",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("imu_config"),
                "snapshot_topic": LaunchConfiguration("snapshot_topic"),
            }
        ],
    )

    monitor_pub = Node(
        package="monitor_value_pub",
        executable="monitor_value_pub",
        name="monitor_value_pub",
        output="screen",
        additional_env={
            "MONITOR_VALUE_CONFIG": LaunchConfiguration("monitor_value_config"),
        },
    )

    monitor_log = Node(
        package="monitor_value_logger",
        executable="monitor_value_logger",
        name="monitor_value_logger",
        output="screen",
        additional_env={
            "MONITOR_VALUE_LOGGER_CONFIG": LaunchConfiguration("monitor_logger_config"),
        },
    )

    monitor_web = Node(
        package="monitor_value_web",
        executable="monitor_value_web",
        name="monitor_value_web",
        output="screen",
        additional_env={
            "MONITOR_VALUE_WEB_CONFIG": LaunchConfiguration("monitor_web_config"),
        },
    )

    return LaunchDescription(
        [
            imu_config,
            monitor_value_config,
            monitor_logger_config,
            snapshot_topic,
            monitor_web_config,
            imu_nav,
            monitor_pub,
            monitor_log,
            monitor_web,
        ]
    )
