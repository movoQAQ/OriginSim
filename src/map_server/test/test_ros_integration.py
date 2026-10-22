#!/usr/bin/env python3
"""Synthetic ROS integration replay; no claim of real radar/bag/hardware validation."""
import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from sensor_msgs.msg import PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from map_server.msg import DerivedMap, DynamicObstacles, DynamicHealth, RadarTargets, RadarTarget, MapStatus, StaticMap


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-binary', required=True)
    parser.add_argument('--report', required=True)
    args = parser.parse_args()
    os.environ.setdefault("ROS_DOMAIN_ID", "87")
    os.environ.setdefault("ROS_LOG_DIR", str(Path(args.report).resolve().parent / "ros-logs"))
    rclpy.init()
    node = rclpy.create_node('map_server_integration_test')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    messages = {'derived': None, 'dynamic': None, 'health': None, 'static': None}
    counts = {'static': 0, 'derived': 0, 'dynamic': 0, 'health': 0}
    derived_delivery_ms = []
    def receive(key, message):
        counts[key] += 1
        messages[key] = message.health if key == 'health' else message
        if key == 'derived':
            derived_delivery_ms.append((node.get_clock().now().nanoseconds * 1e-9 - message.generated_at_s) * 1000)
    for name, msg_type, topic in [('derived', DerivedMap, '/map_server/derived'),
                                   ('dynamic', DynamicObstacles, '/map_server/dynamic'),
                                   ('health', MapStatus, '/map_server/status'),
                                   ('static', StaticMap, '/map_server/static')]:
        node.create_subscription(msg_type, topic, lambda msg, key=name: receive(key, msg), qos)
    cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', qos_profile_sensor_data)
    ally_pub = node.create_publisher(RadarTargets, '/radar/allies', 10)
    enemy_pub = node.create_publisher(RadarTargets, '/radar/enemies', 10)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for child in ['odom', 'base_link_fake']:
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    broadcaster.sendTransform(transforms)
    report = {'kind': 'synthetic ROS integration', 'checks': [], 'derived_versions': []}
    def check(condition, label):
        if not condition:
            raise AssertionError(label)
        report['checks'].append(label)
    def spin(duration, clouds=False, radar=False, bad_tf=False):
        end = time.monotonic() + duration
        next_cloud = next_radar = 0.0
        while time.monotonic() < end:
            t = time.monotonic()
            if clouds and t >= next_cloud:
                header = Header(frame_id='missing_frame' if bad_tf else 'map',
                                stamp=node.get_clock().now().to_msg())
                points = [(1.0 + i * 0.01, 0.0, 0.2) for i in range(20)]
                cloud_pub.publish(create_cloud_xyz32(header, points))
                next_cloud = t + 0.05
            if radar and t >= next_radar:
                send_radar()
                next_radar = t + 1.0
            rclpy.spin_once(node, timeout_sec=0.005)
    def send_radar(empty=False):
        for publisher, target_id, x in [(ally_pub, 7, 0.0), (enemy_pub, 2, 8.5)]:
            message = RadarTargets()
            message.header.frame_id = 'map'
            message.header.stamp = node.get_clock().now().to_msg()
            message.observation_valid = True
            message.stable_ids = True
            if not empty:
                target = RadarTarget()
                target.id = target_id
                target.position.x = x
                target.observed_at = message.header.stamp
                target.physical_radius_m = 0.2
                message.targets = [target]
            publisher.publish(message)
    with tempfile.TemporaryDirectory(prefix='origin-map-replay-') as temporary:
        temp = Path(temporary)
        assets = Path(__file__).resolve().parents[2] / 'bringup' / 'map'
        map_yaml = assets / 'rmuc_2027.yaml'
        static_pcd = assets / 'rmuc_2027.pcd'
        logfile = open(temp / 'node.log', 'w+')
        process = subprocess.Popen([args.node_binary, '--ros-args', '-p', f'yaml_filename:={map_yaml}',
                                    '-p', f'static_pcd:={static_pcd}', '-p', 'radar.self_id:=7',
                                    '-p', 'voxel_size:=0.005', '-p', 'min_density_per_m2:=1.0',
                                    '-p', 'min_component_area:=0.001', '-p', 'memory.hit_rate:=30.0'],
                                   stdout=logfile, stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+15
            while any(messages[k] is None for k in ('static','derived','dynamic','health')) and process.poll() is None and time.monotonic()<deadline:
                spin(0.1)
            check(process.poll() is None, 'node starts with hxy rmuc_2027 PGM/PCD')
            check(messages['static'] is not None and len(messages['static'].depth) == 0, 'Depth remains disabled')
            check(messages['derived'] is not None, 'complete derived map received')
            spin(3.0, clouds=True, radar=True)
            health = messages['health']
            check(health.cloud == DynamicHealth.VALID, '20 Hz cloud source is valid')
            check(health.radar_enemies == DynamicHealth.VALID, '1 Hz radar source remains valid')
            dynamic = messages['dynamic']
            check(len(dynamic.circles) == 1 and abs(dynamic.circles[0].center.x - 8.5) < 1e-6, 'self ID excluded and enemy retained')
            derived = messages['derived']
            report['derived_versions'].append(derived.version)
            check(any(derived.fused_obstacles), 'fused obstacle grid received on hxy rmuc_2027 geometry')
            n = derived.info.width * derived.info.height
            check(len(derived.costmap) == n == len(derived.collision_mask), 'float/binary arrays share full geometry')
            check(all((c == float('inf')) == bool(v) for c, v in zip(derived.costmap, derived.collision_mask)),
                  'published costmap and collision mask are consistent')
            spin(0.9, clouds=False, radar=True)
            check(messages['health'].cloud == DynamicHealth.STALE, 'cloud timeout independent of continuing radar')
            check(messages['health'].radar_enemies == DynamicHealth.VALID, 'radar does not expire with cloud')
            send_radar(empty=True)
            spin(0.25, clouds=True)
            check(not messages['dynamic'].circles, 'explicit empty radar frame retracts its source')
            spin(2.7, clouds=True)
            check(messages['health'].radar_enemies == DynamicHealth.STALE, '20 Hz cloud cannot renew radar lifetime')
            spin(0.4, clouds=True, bad_tf=True)
            check(messages['health'].cloud == DynamicHealth.ERROR, 'missing measurement-time TF surfaces as cloud error')
            spin(0.5, clouds=True)
            check(messages['health'].cloud == DynamicHealth.VALID, 'cloud recovers with valid measurement TF')
            check(not messages['health'].map_error, 'worker remains healthy throughout replay')
            check(counts['static'] == 1, 'static arrays delivered once during source updates')
            check(not hasattr(derived, 'region_id') and not hasattr(dynamic, 'health'), 'no duplicate static arrays or dynamic health payload')
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise AssertionError('MapServer worker failed to join on shutdown')
            logfile.seek(0)
            report['node_log'] = logfile.read()
            Path(args.report).write_text(json.dumps(report, indent=2))
            logfile.close()
        check(process.returncode == 0, 'clean ROS shutdown joins worker')
    if derived_delivery_ms:
        values = sorted(derived_delivery_ms)
        report['derived_generation_to_receive_ms'] = {
            'samples': len(values), 'p50': values[len(values)//2],
            'p95': values[int((len(values)-1)*0.95)], 'max': values[-1]}
    node.destroy_node()
    rclpy.shutdown()
    Path(args.report).write_text(json.dumps(report, indent=2))
    print(json.dumps({'passed': len(report['checks']), 'checks': report['checks']}, indent=2))


if __name__ == '__main__':
    main()
