#!/usr/bin/env python3
"""Planar wheel/IMU odometry anchored to fixed, learned LiDAR walls.

Requires a stationary flat start and calibrated map-to-odom transform.
No simulator pose is consumed. Suppresses output when sensor data or
wall constraints become stale; external command watchdogs must stop motion."""
import math, time, numpy as np, rclpy
from scipy.spatial.transform import Rotation, Slerp
from collections import deque
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
import landmark_alignment as arena
from wheel_lidar_math import (
    body_attitude, fresh, rolling_velocity, wall_correction,
    match_fixed_planes, scan_velocity,
)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class WheelLidarOdometry(Node):

    def __init__(self):
        super().__init__('wheel_lidar_odometry')
        self.wheel = None
        self.imu = None
        self.calibration = []
        self.reference = None
        self.previous = None
        self.p = np.zeros(3)
        self.last_rotation = None
        self.radius = self.declare_parameter('wheel_radius', 0.0758).value
        self.planes = None
        self.scan_boot = []
        self.last_scan = None
        self.imu_history = deque(maxlen=300)
        self.pose_history = deque(maxlen=150)
        self.scan_positions = deque(maxlen=12)
        self.measured_velocity = None
        self.mount = Rotation.from_euler('z', self.declare_parameter('imu_mount_yaw', -1.095).value)
        self.sensor_height = self.declare_parameter('sensor_height', 0.6).value
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.js = self.create_subscription(JointState, '/joint_states', self.joints, qos)
        self.ms = self.create_subscription(Imu, '/livox/imu', self.inertial, QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cs = self.create_subscription(PointCloud2, '/livox/lidar/pointcloud', self.cloud, qos)
        self.timer = self.create_timer(0.02, self.update)

    def joints(self, m):
        try:
            velocity = np.zeros(3)
            wheels = []
            for prefix in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                a = m.position[m.name.index(prefix + '_steering_joint')]
                s = self.radius * m.velocity[m.name.index(prefix + '_wheel_joint')]
                velocity[:2] += 0.25 * s * np.array([math.cos(a), math.sin(a)])
                wheels.append((a, s))
            if not np.isfinite(velocity).all():
                return
            self.wheel = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-09, time.monotonic(), velocity, wheels)
        except (IndexError, ValueError):
            return

    def inertial(self, m):
        q = np.array([m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])
        if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.1:
            return
        self.imu = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-09, time.monotonic(), Rotation.from_quat(q))
        self.imu_history.append((self.imu[0], self.imu[2]))

    def cloud(self, m):
        if self.reference is None or self.imu is None:
            return
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-09
        samples = list(self.imu_history)
        before = [x for x in samples if x[0] <= stamp]
        after = [x for x in samples if x[0] > stamp]
        if not before or stamp - before[-1][0] > 0.03:
            return
        measured = before[-1][1]
        if after and after[0][0] - stamp < 0.03 and (after[0][0] > before[-1][0]):
            measured = Slerp([before[-1][0], after[0][0]], Rotation.concatenate([before[-1][1], after[0][1]]))([stamp])[0]
        rotation = body_attitude(self.mount, measured, self.reference)
        if not self.pose_history:
            return
        pose_stamp, position = min(self.pose_history, key=lambda x: abs(x[0] - stamp))
        if abs(pose_stamp - stamp) > 0.05:
            return
        a = read_points_numpy(m, field_names=['x', 'y', 'z'], skip_nans=True).astype(float)
        a += np.array([0, 0, 0.03])
        world = rotation.apply(self.mount.apply(a) + np.array([0, 0, self.sensor_height])) + position
        world = world[world[:, 2] > 3.5]
        if len(world) < 100:
            return
        if self.planes is None:
            if self.wheel is None or np.linalg.norm(self.wheel[2]) > 0.01:
                self.scan_boot = []
                return
            self.scan_boot.append(world[:, :2])
            if len(self.scan_boot) < 8:
                return
            self.planes = arena.learn_planes(np.concatenate(self.scan_boot))
            self.scan_boot = []
            if self.planes is not None:
                self.get_logger().info('Fixed LiDAR walls ready for encoder/IMU fusion: ' + str(self.planes))
        else:
            correction = wall_correction(world, self.planes)
            if correction is None:
                correction = match_fixed_planes(arena.learn_planes(world[:, :2]), self.planes)
                if correction is None:
                    return
            scan_position = position[:2] + correction
            self.scan_positions.append((stamp, scan_position.copy()))
            while self.scan_positions and stamp - self.scan_positions[0][0] > 0.4:
                self.scan_positions.popleft()
            self.measured_velocity = scan_velocity(self.scan_positions)
            needs_relocalization = np.max(np.abs(correction)) > 0.15
            correction = np.clip(correction, -0.15, 0.15)
            self.p[:2] += correction
            for _, pose in self.pose_history:
                pose[:2] += correction
            if needs_relocalization:
                self.last_scan = None
                return
        if self.planes is not None:
            self.last_scan = (stamp, time.monotonic())

    def update(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-09
        wall = time.monotonic()
        if self.wheel is None or self.imu is None:
            return
        if any(not fresh(v[0], v[1], t, wall) for v in [self.wheel, self.imu]):
            self.previous = None
            return
        if self.reference is None:
            if np.linalg.norm(self.wheel[2]) > 0.01:
                self.calibration = []
                return
            self.calibration.append(self.imu[2].as_quat())
            if len(self.calibration) < 25:
                return
            self.reference = Rotation.from_quat(np.mean(self.calibration, axis=0))
            self.get_logger().info('Encoder/IMU reference initialized while stationary: ' + str(self.reference.as_quat()))
        rotation = body_attitude(self.mount, self.imu[2], self.reference)
        v = np.zeros(3)
        omega = np.zeros(3)
        v = rolling_velocity(rotation, self.wheel[3])
        if v is None:
            self.previous = None
            return
        if self.previous is not None:
            dt = t - self.previous
            if 0 < dt <= 0.1:
                self.p[:2] += v[:2] * dt
                omega = (self.last_rotation.inv() * rotation).as_rotvec() / dt
        self.previous = t
        self.last_rotation = rotation
        self.pose_history.append((t, self.p.copy()))
        if self.planes is None or self.last_scan is None or t - self.last_scan[0] > 0.5 or (wall - self.last_scan[1] > 0.8):
            return
        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'wheel_base'
        m.pose.pose.position.x = float(self.p[0])
        m.pose.pose.position.y = float(self.p[1])
        m.pose.pose.position.z = float(self.p[2])
        q = rotation.as_quat()
        m.pose.pose.orientation.x = float(q[0])
        m.pose.pose.orientation.y = float(q[1])
        m.pose.pose.orientation.z = float(q[2])
        m.pose.pose.orientation.w = float(q[3])
        measured_v = v.copy()
        if self.measured_velocity is not None:
            measured_v[:2] = self.measured_velocity
        body_v = rotation.inv().apply(measured_v)
        m.twist.twist.linear.x = float(body_v[0])
        m.twist.twist.linear.y = float(body_v[1])
        m.twist.twist.linear.z = float(body_v[2])
        m.twist.twist.angular.x = float(omega[0])
        m.twist.twist.angular.y = float(omega[1])
        m.twist.twist.angular.z = float(omega[2])
        self.pub.publish(m)
        tr = TransformStamped()
        tr.header = m.header
        tr.child_frame_id = m.child_frame_id
        tr.transform.translation.x = m.pose.pose.position.x
        tr.transform.translation.y = m.pose.pose.position.y
        tr.transform.translation.z = m.pose.pose.position.z
        tr.transform.rotation = m.pose.pose.orientation
        self.br.sendTransform(tr)

def main():
    rclpy.init()
    node = WheelLidarOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
if __name__ == '__main__':
    main()
