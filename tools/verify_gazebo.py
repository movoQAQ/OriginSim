"""Check one nearby goal against a running hxy Gazebo/navigation stack."""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from map_server.msg import DerivedMap, DynamicObstacles, MapStatus, StaticMap
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', default='gazebo-validation.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gazebo_nav_validation', parameter_overrides=[
        Parameter('use_sim_time', value=True),
    ])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    messages, counts, nonzero, poses = {}, {}, [], []
    report = {'source': 'real hxy Gazebo rmuc_2027', 'passed': False, 'checks': []}

    def receive(key, message):
        messages[key] = message
        counts[key] = counts.get(key, 0) + 1
        if key == 'cmd' and math.hypot(message.linear.x, message.linear.y) > 0.02:
            nonzero.append((message.linear.x, message.linear.y))

    for topic, msg_type, key in [
        ('/map_server/static', StaticMap, 'static'),
        ('/map_server/derived', DerivedMap, 'derived'),
        ('/map_server/status', MapStatus, 'status'),
        ('/map_server/dynamic', DynamicObstacles, 'dynamic'),
        ('/plan', Path, 'plan'), ('/plan_original', Path, 'original'),
        ('/plan_smoothed', Path, 'smoothed'),
    ]:
        node.create_subscription(msg_type, topic,
                                 lambda msg, key=key: receive(key, msg), qos)
    for topic, msg_type, key in [
        ('/odom', Odometry, 'odom'),
        ('/cloud_registered', PointCloud2, 'cloud'),
        ('/cmd_vel_chassis', Twist, 'cmd'),
    ]:
        node.create_subscription(msg_type, topic,
                                 lambda msg, key=key: receive(key, msg),
                                 qos_profile_sensor_data)

    # The Gazebo odometry stream is very fast; take one sample per endpoint.
    truth_sub = None

    def capture_truth(message):
        receive('truth', message)
        node.destroy_subscription(truth_sub)

    truth_sub = node.create_subscription(Odometry, '/Odometry', capture_truth,
                                         qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)

    def pose():
        transform = buffer.lookup_transform('map', 'base_link_fake', rclpy.time.Time())
        return transform.transform.translation.x, transform.transform.translation.y

    def check(condition, label):
        if not condition:
            raise RuntimeError(label)
        report['checks'].append(label)

    def ready():
        if not all(k in messages for k in ('static', 'derived', 'dynamic', 'status', 'truth', 'odom', 'cloud')):
            return False
        status, derived, dynamic = (messages[k] for k in ('status', 'derived', 'dynamic'))
        return (buffer.can_transform('map', 'base_link_fake', rclpy.time.Time())
                and status.producer_id == derived.producer_id == dynamic.producer_id == messages['static'].producer_id
                and status.derived_version == derived.version
                and status.dynamic_version == dynamic.version
                and status.health.cloud == status.health.VALID
                and not status.health.map_error)

    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not ready():
            spin(0.1)
        check(ready(), 'real cloud, odometry, TF and coherent map channels ready')
        check(not messages['static'].depth, 'Depth empty')
        start = pose()
        truth = messages['truth'].pose.pose.position
        truth_start = (truth.x, truth.y)
        derived = messages['derived']
        geometry = derived.info

        def free(x, y):
            mx = math.floor((x - geometry.origin.position.x) / geometry.resolution)
            my = math.floor((y - geometry.origin.position.y) / geometry.resolution)
            return (0 <= mx < geometry.width and 0 <= my < geometry.height
                    and math.isfinite(derived.costmap[my * geometry.width + mx]))

        check(free(*start), 'robot starts in traversable map cell')
        candidates = [(start[0] + 1.2, start[1]), (start[0], start[1] - 1.2),
                      (start[0] - 1.2, start[1]), (start[0], start[1] + 1.2)]
        goal = next((p for p in candidates if all(
            free(start[0] + (p[0] - start[0]) * i / 30,
                 start[1] + (p[1] - start[1]) * i / 30) for i in range(31))), None)
        check(goal is not None, 'nearby straight traversable goal available')
        report.update(start=start, truth_start=truth_start, goal=goal)
        command = PoseStamped()
        command.header.frame_id = 'map'
        command.header.stamp = node.get_clock().now().to_msg()
        command.pose.position.x, command.pose.position.y = goal
        command.pose.orientation.w = 1.0
        goal_pub.publish(command)
        deadline, best = time.monotonic() + 35, math.inf
        while time.monotonic() < deadline:
            spin(0.1)
            try:
                position = pose()
            except Exception:
                continue
            poses.append(position)
            best = min(best, math.dist(position, goal))
            if best < 0.28 and len(nonzero) > 5:
                break
        spin(2)
        truth_sub = node.create_subscription(Odometry, '/Odometry', capture_truth,
                                             qos_profile_sensor_data)
        deadline = time.monotonic() + 2
        while counts.get('truth', 0) < 2 and time.monotonic() < deadline:
            spin(0.05)
        check(counts.get('truth', 0) == 2, 'Gazebo truth endpoint sampled')
        final = pose()
        truth = messages['truth'].pose.pose.position
        truth_final = (truth.x, truth.y)
        report.update(final=final, truth_final=truth_final,
                      final_goal_error_m=math.dist(final, goal),
                      minimum_goal_error_m=best,
                      truth_displacement_m=math.dist(truth_start, truth_final),
                      distance_moved=math.dist(start, final),
                      nonzero_commands=len(nonzero),
                      plan_points=len(messages.get('plan', Path()).poses),
                      original_points=len(messages.get('original', Path()).poses),
                      smoothed_points=len(messages.get('smoothed', Path()).poses))
        check(nonzero and report['plan_points'] > 0, 'path and nonzero chassis commands received')
        check(report['truth_displacement_m'] > 0.5, 'Gazebo robot moved over 0.5 m')
        check(report['final_goal_error_m'] < 0.35, 'parked robot within 0.35 m of goal')
        report['passed'] = True
    except Exception as error:
        report['failure'] = str(error)
    finally:
        report.update(counts=dict(counts), pose_samples=poses[::10])
        with open(args.report, 'w') as output:
            json.dump(report, output, indent=2)
        print(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
