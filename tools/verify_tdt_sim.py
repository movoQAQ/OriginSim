#!/usr/bin/env python3
"""Check that simulated T-DT localization converges against Gazebo odometry."""
import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self, truth_topic, truth_frame, sensor_frame):
        super().__init__('tdt_sim_verifier')
        self.truth = None
        self.truth_frame = truth_frame
        self.sensor_frame = sensor_frame
        self.state = ''
        self.errors = []
        self.truth_start = None
        self.truth_end = None
        self.counts = {'no_truth': 0, 'untrusted': 0, 'stale': 0, 'no_frame': 0,
                       'no_tf': 0}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, truth_topic, self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/backend_node/localization_pose', self.on_estimate,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/backend_node/localization_state', self.on_state, 10)

    def on_truth(self, msg):
        self.truth = msg
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.truth_start is None:
            self.truth_start = point
        self.truth_end = point

    def on_state(self, msg):
        self.state = msg.data

    def on_estimate(self, msg):
        if self.truth is None:
            self.counts['no_truth'] += 1
            return
        if 'pose_trusted=true' not in self.state:
            self.counts['untrusted'] += 1
            return
        ts = self.truth.header.stamp.sec + self.truth.header.stamp.nanosec * 1e-9
        es = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(ts - es) > 0.15:
            self.counts['stale'] += 1
            return
        p, q = msg.pose.pose, self.truth.pose.pose
        # Gazebo's Odometry bridge uses '/chassis'; tf2 frame IDs omit the slash.
        truth_frame = (self.truth_frame or self.truth.child_frame_id).lstrip('/')
        if not truth_frame:
            self.counts['no_frame'] += 1
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                truth_frame, self.sensor_frame, Time())
        except Exception:
            self.counts['no_tf'] += 1
            return
        offset = transform.transform.translation
        true_yaw = yaw(q)
        c, s = math.cos(true_yaw), math.sin(true_yaw)
        x = q.position.x + c * offset.x - s * offset.y
        y = q.position.y + s * offset.x + c * offset.y
        orientation = transform.transform.rotation
        offset_yaw = math.atan2(2 * (orientation.w * orientation.z +
                                      orientation.x * orientation.y),
                                1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
        distance = math.hypot(p.position.x - x, p.position.y - y)
        heading = abs(math.remainder(yaw(p) - true_yaw - offset_yaw, 2 * math.pi))
        self.errors.append((distance, heading))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--truth-topic', default='/Odometry')
    parser.add_argument('--truth-frame', default='',
                        help='Override the ground truth child frame if the bridge leaves it blank')
    parser.add_argument('--sensor-frame', default='livox_frame',
                        help='Physical IMU/LiDAR frame represented by Point-LIO odometry')
    parser.add_argument('--seconds', type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    monitor = Monitor(args.truth_topic, args.truth_frame, args.sensor_frame)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.2)
        if len(monitor.errors) < 20:
            raise RuntimeError(f'Only {len(monitor.errors)} synchronized trusted samples; '
                               f'rejections={monitor.counts}; last state: {monitor.state}')
        position = statistics.median(x for x, _ in monitor.errors)
        heading = statistics.median(x for _, x in monitor.errors)
        position_p95 = statistics.quantiles((x for x, _ in monitor.errors), n=20)[18]
        heading_p95 = statistics.quantiles((x for _, x in monitor.errors), n=20)[18]
        travel = math.dist(monitor.truth_start, monitor.truth_end)
        trust_fraction = len(monitor.errors) / max(
            1, len(monitor.errors) + monitor.counts['untrusted'] + monitor.counts['stale'])
        print(f'samples={len(monitor.errors)} median_xy_m={position:.3f} '
              f'p95_xy_m={position_p95:.3f} median_yaw_rad={heading:.3f} '
              f'p95_yaw_rad={heading_p95:.3f} displacement_m={travel:.3f} '
              f'trust_fraction={trust_fraction:.3f} '
              f'rejections={monitor.counts} state={monitor.state}')
        if (position > 0.75 or heading > 0.35 or position_p95 > 1.5
                or heading_p95 > 0.7 or trust_fraction < 0.9
                or 'pose_trusted=true' not in monitor.state
                or 'tf_published=true' not in monitor.state):
            raise RuntimeError('Localization exceeds simulation feasibility limits')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
