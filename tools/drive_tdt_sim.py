#!/usr/bin/env python3
"""Send a bounded chassis velocity pulse and a sustained stop in Gazebo."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=0.08)
    parser.add_argument('--yaw-rate', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node('tdt_sim_driver')
    pub = node.create_publisher(Twist, '/cmd_vel_chassis', 10)
    try:
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No Gazebo chassis controller subscribed')

        command = Twist()
        command.linear.x = args.speed
        command.angular.z = args.yaw_rate
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        # The Gazebo chassis controller holds its last target velocity.
        # Repeated zeros ensure the stop reaches it before this publisher exits.
        stop = Twist()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
