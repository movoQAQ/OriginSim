#!/usr/bin/env python3
"""Reduce Gazebo's instantaneous PointCloud2 scan before ZLIO initialization."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudAdapter(Node):
    def __init__(self):
        super().__init__('zlio_cloud_adapter')
        input_topic = self.declare_parameter('input_topic', '/velodyne_points').value
        output_topic = self.declare_parameter('output_topic', '/zlio/points').value
        self.stride = int(self.declare_parameter('stride', 6).value)
        if self.stride < 1:
            raise ValueError('stride must be positive')
        self.publisher = self.create_publisher(PointCloud2, output_topic,
                                               qos_profile_sensor_data)
        self.create_subscription(PointCloud2, input_topic, self.on_cloud,
                                 qos_profile_sensor_data)

    def on_cloud(self, source):
        if (source.point_step == 0 or source.row_step < source.width * source.point_step
                or len(source.data) < source.row_step * source.height):
            self.get_logger().error('Invalid PointCloud2 layout')
            return
        data = bytearray()
        for row in range(source.height):
            start = row * source.row_step
            for column in range(0, source.width, self.stride):
                offset = start + column * source.point_step
                data.extend(source.data[offset:offset + source.point_step])
        cloud = PointCloud2()
        cloud.header = source.header
        cloud.height = 1
        cloud.width = len(data) // source.point_step
        cloud.fields = source.fields
        cloud.is_bigendian = source.is_bigendian
        cloud.point_step = source.point_step
        cloud.row_step = len(data)
        cloud.data = bytes(data)
        cloud.is_dense = source.is_dense
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = CloudAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
