from __future__ import annotations

import math
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster

from ament_index_python.packages import get_package_share_directory

from imu_navigation.fusion_runner import FusionRunner
from imu_navigation.types import ImuNavSnapshotData
from monitor_value_interfaces.msg import ImuNavSnapshot


def _wall_sec_to_time(sec: float) -> Time:
    """Unix wall seconds → ``builtin_interfaces/Time`` (sensor acquisition on host)."""
    if math.isnan(sec) or sec <= 0.0:
        return Time()
    si = int(sec)
    ns = int(round((sec - float(si)) * 1e9))
    while ns < 0:
        si -= 1
        ns += 1_000_000_000
    while ns >= 1_000_000_000:
        si += 1
        ns -= 1_000_000_000
    return Time(sec=si, nanosec=ns)


def _fill_imu_msg(imu: Imu, snap: ImuNavSnapshotData, frame_id: str, stamp: Time) -> None:
    imu.header.stamp = stamp
    imu.header.frame_id = frame_id
    imu.orientation.x = float(snap.ori_x)
    imu.orientation.y = float(snap.ori_y)
    imu.orientation.z = float(snap.ori_z)
    imu.orientation.w = float(snap.ori_w)
    imu.angular_velocity.x = float(snap.gyro_x)
    imu.angular_velocity.y = float(snap.gyro_y)
    imu.angular_velocity.z = float(snap.gyro_z)
    imu.linear_acceleration.x = float(snap.accel_x)
    imu.linear_acceleration.y = float(snap.accel_y)
    imu.linear_acceleration.z = float(snap.accel_z)
    for arr in (
        imu.orientation_covariance,
        imu.angular_velocity_covariance,
        imu.linear_acceleration_covariance,
    ):
        for i in range(9):
            arr[i] = 0.0
    imu.orientation_covariance[0] = 1e-2
    imu.orientation_covariance[4] = 1e-2
    imu.orientation_covariance[8] = 1e-2
    imu.angular_velocity_covariance[0] = 1e-4
    imu.angular_velocity_covariance[4] = 1e-4
    imu.angular_velocity_covariance[8] = 1e-4
    imu.linear_acceleration_covariance[0] = 1e-2
    imu.linear_acceleration_covariance[4] = 1e-2
    imu.linear_acceleration_covariance[8] = 1e-2


def _fill_odom_msg(odom: Odometry, snap: ImuNavSnapshotData, frame_id: str, child: str, stamp: Time) -> None:
    odom.header.stamp = stamp
    odom.header.frame_id = frame_id
    odom.child_frame_id = child
    odom.pose.pose.position.x = float(snap.pose_x)
    odom.pose.pose.position.y = float(snap.pose_y)
    odom.pose.pose.position.z = float(snap.pose_z)
    odom.pose.pose.orientation.x = float(snap.ori_x)
    odom.pose.pose.orientation.y = float(snap.ori_y)
    odom.pose.pose.orientation.z = float(snap.ori_z)
    odom.pose.pose.orientation.w = float(snap.ori_w)
    odom.twist.twist.linear.x = float(snap.vel_x)
    odom.twist.twist.linear.y = float(snap.vel_y)
    odom.twist.twist.linear.z = float(snap.vel_z)


def _snap_to_msg(snap: ImuNavSnapshotData) -> ImuNavSnapshot:
    m = ImuNavSnapshot()
    m.seq = int(snap.seq)
    m.stamp = _wall_sec_to_time(float(snap.stamp_sec))
    m.imu_sample_time_sec = float(snap.stamp_sec)
    m.depth_sample_time_sec = float(snap.depth_sample_time_sec)
    m.gyro_x = float(snap.gyro_x)
    m.gyro_y = float(snap.gyro_y)
    m.gyro_z = float(snap.gyro_z)
    m.accel_x = float(snap.accel_x)
    m.accel_y = float(snap.accel_y)
    m.accel_z = float(snap.accel_z)
    m.mag_x = float(snap.mag_x)
    m.mag_y = float(snap.mag_y)
    m.mag_z = float(snap.mag_z)
    m.ori_x = float(snap.ori_x)
    m.ori_y = float(snap.ori_y)
    m.ori_z = float(snap.ori_z)
    m.ori_w = float(snap.ori_w)
    m.pose_x = float(snap.pose_x)
    m.pose_y = float(snap.pose_y)
    m.pose_z = float(snap.pose_z)
    m.vel_x = float(snap.vel_x)
    m.vel_y = float(snap.vel_y)
    m.vel_z = float(snap.vel_z)
    m.depth_m = float(snap.depth_m)
    m.depth_temp_c = float(snap.depth_temp_c)
    m.depth_pressure_atm = float(snap.depth_pressure_atm)
    return m


class ImuNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_navigation")

        self.declare_parameter("config_file", "")
        self.declare_parameter("snapshot_topic", "rov/imu_nav/snapshot")
        cfg_path = str(self.get_parameter("config_file").value).strip()
        if not cfg_path:
            share = Path(get_package_share_directory("imu_navigation"))
            cfg_path = str(share / "config" / "imu_navigation.yaml")

        self._runner = FusionRunner.from_yaml(cfg_path)
        self.get_logger().info(f"imu_navigation config: {cfg_path}")
        self.get_logger().info(
            "fusion sensors: "
            f"dof9={'up' if self._runner._dof9 is not None else 'down'} "
            f"depth={'up' if self._runner._depth is not None else 'down'}"
        )
        for err in self._runner.init_errors:
            self.get_logger().warn(f"fusion init: {err}")

        snap_topic = str(self.get_parameter("snapshot_topic").value)
        self._snap_pub = self.create_publisher(ImuNavSnapshot, snap_topic, 10)

        rl = self._runner.config.get("robot_localization") or {}
        self._imu_frame = str(rl.get("imu_frame_id", "imu_link"))
        self._base_frame = str(rl.get("base_frame_id", "base_link"))
        self._odom_frame = str(rl.get("odom_frame_id", "odom"))
        pub_imu = bool(rl.get("publish_imu", True))
        pub_odom = bool(rl.get("publish_odom", True))
        pub_tf = bool(rl.get("publish_static_tf", True))

        self._imu_pub = self.create_publisher(Imu, "/imu/data", 10) if pub_imu else None
        self._odom_pub = self.create_publisher(Odometry, "/imu/dead_reckon_odom", 10) if pub_odom else None
        self._tf_static: StaticTransformBroadcaster | None = None
        if pub_tf:
            self._tf_static = StaticTransformBroadcaster(self)
            self._publish_static_frames()

        period = self._runner.fusion_period_sec
        self.create_timer(period, self._tick)

    def _publish_static_frames(self) -> None:
        if self._tf_static is None:
            return
        ex = self._runner.config.get("extrinsics") or {}
        imu_ex = ex.get("imu") or {}
        rpy = imu_ex.get("rpy_deg", [0.0, 0.0, 0.0])
        p = imu_ex.get("position_m", [0.0, 0.0, 0.0])
        d2r = math.pi / 180.0
        roll, pitch, yaw = float(rpy[0]) * d2r, float(rpy[1]) * d2r, float(rpy[2]) * d2r
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._base_frame
        t.child_frame_id = self._imu_frame
        t.transform.translation.x = float(p[0])
        t.transform.translation.y = float(p[1])
        t.transform.translation.z = float(p[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self._tf_static.sendTransform(t)

    def _tick(self) -> None:
        try:
            snap = self._runner.step()
        except Exception as e:
            self.get_logger().warn(f"fusion step failed: {e}")
            return
        self._snap_pub.publish(_snap_to_msg(snap))
        imu_stamp = _wall_sec_to_time(float(snap.stamp_sec))
        if self._imu_pub is not None:
            imu = Imu()
            _fill_imu_msg(imu, snap, self._imu_frame, imu_stamp)
            self._imu_pub.publish(imu)
        if self._odom_pub is not None:
            odom = Odometry()
            _fill_odom_msg(odom, snap, self._odom_frame, self._base_frame, imu_stamp)
            self._odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
