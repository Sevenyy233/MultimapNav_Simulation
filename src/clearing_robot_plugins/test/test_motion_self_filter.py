#!/usr/bin/env python3
"""Historical bucket returns must be filtered before motion compensation."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml


def main():
    os.environ['ROS_DOMAIN_ID'] = '218'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    output = Path(tempfile.mkdtemp(prefix='clearing_motion_filter_'))
    os.environ['ROS_LOG_DIR'] = str(output/'ros_logs')
    root = Path(get_package_share_directory('clearing_robot_plugins'))
    boxes = yaml.safe_load((root/'config/robot_model.yaml').read_text())['boxes']
    sensors = [b for b in boxes if b['name'].startswith('rslidar_')]
    bucket = next(b for b in boxes if b['name'] == 'link7')
    bucket_point = np.array(bucket['center']) + Rotation.from_quat(bucket['quaternion']).apply(
        np.array(bucket['size'])*np.array([-.45, -.45, -.45]))
    rotated = Rotation.from_euler('z', -.35).apply(bucket_point)
    # This point is a self return in its acquisition frame, but would leak
    # through the old filter after rotation into the latest body frame.
    assert not any(np.all(abs(Rotation.from_quat(b['quaternion']).inv().apply(
        rotated-b['center'])) <= np.array(b['size'])/2+.10) for b in boxes)
    process = None
    rclpy.init()
    node = rclpy.create_node('motion_self_filter_contract')
    static = StaticTransformBroadcaster(node)
    dynamic = TransformBroadcaster(node)
    transforms = []
    for b in sensors:
        t = TransformStamped()
        t.header.frame_id = 'base_link'
        t.child_frame_id = b['name']
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, b['center'])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, b['quaternion'])
        transforms.append(t)
    map_odom = TransformStamped()
    map_odom.header.frame_id = 'map'
    map_odom.child_frame_id = 'odom'
    map_odom.transform.rotation.w = 1.
    transforms.append(map_odom)
    static.sendTransform(transforms)
    odometry = node.create_publisher(Odometry, '/odom', 10)
    publishers = [node.create_publisher(PointCloud2, f'/livox/lidar_{i}_PointCloud2',
                                      qos_profile_sensor_data) for i in range(1, 6)]
    received = []
    obstacle_received = []
    subscription = node.create_subscription(PointCloud2, '/clearing/points', received.append,
                                           qos_profile_sensor_data)
    obstacle_subscription = node.create_subscription(PointCloud2, '/clearing/obstacles',
        obstacle_received.append, qos_profile_sensor_data)
    try:
        with (output/'perception.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'run', 'clearing_robot_plugins', 'clearing_perception',
                '--ros-args', '-p', f'robot_model_file:={root / "config/robot_model.yaml"}',
                '-p', f'map_file:={root / "maps/clearing_enclosed.pcd"}',
                '-p', f'support_file:={root / "maps/clearing_support.pcd"}'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            def publish_scan(x=8., y=0., base_x=0.):
                latest = node.get_clock().now()-Duration(seconds=.02)
                old = latest-Duration(seconds=.20)
                for stamp, yaw in ((old, 0.), (latest, .35)):
                    t = TransformStamped()
                    t.header.frame_id = 'odom'
                    t.header.stamp = stamp.to_msg()
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = float(base_x)
                    t.transform.rotation.z = float(np.sin(yaw/2))
                    t.transform.rotation.w = float(np.cos(yaw/2))
                    dynamic.sendTransform(t)
                odom = Odometry()
                odom.header.stamp = latest.to_msg()
                odometry.publish(odom)
                for b, publisher in zip(sensors, publishers):
                    points = np.array([bucket_point, [12, -5, 1]]+
                                      [[x+.1*j, y, .5] for j in range(20)])
                    local = Rotation.from_quat(b['quaternion']).inv().apply(points-b['center'])
                    publisher.publish(point_cloud2.create_cloud_xyz32(
                        Header(stamp=old.to_msg(), frame_id=b['name']), local.tolist()))
                end = time.monotonic()+.12
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.01)
            deadline = time.monotonic()+20
            while (not received or not obstacle_received) and time.monotonic() < deadline:
                publish_scan()
            assert received, f'No aligned cloud; {output}'
            points = list(point_cloud2.read_points(received[-1], field_names=('x', 'y', 'z')))
            assert points
            assert all(np.linalg.norm([p[0], p[1]]) > 7.5 for p in points), points
            assert obstacle_received, f'No obstacle observations; {output}'
            obstacles = list(point_cloud2.read_points(obstacle_received[-1], field_names=('x', 'y', 'z')))
            assert len(obstacles) >= 3, obstacles
            assert all(abs(p[1]) < .1 for p in obstacles), obstacles
            print('PASS: historical bucket self returns removed before 0.35 rad motion alignment.')
            print('PASS: supported obstacle retained; isolated ray rejected before obstacle memory.')
            # Look into a different sector longer than the ten-second TTL.
            # Absence of a return in the old sector must not turn it into free space.
            deadline = time.monotonic()+11.2
            while time.monotonic()<deadline:
                publish_scan(16.,8.)
            obstacles = list(point_cloud2.read_points(obstacle_received[-1], field_names=('x','y','z')))
            assert any(7.9<p[0]<10.1 and abs(p[1])<.1 for p in obstacles), obstacles
            deadline = time.monotonic()+.8
            while time.monotonic()<deadline:
                publish_scan(16.,8.,base_x=40.)
            obstacles = list(point_cloud2.read_points(obstacle_received[-1], field_names=('x','y','z')))
            assert not any(7.9<p[0]<10.1 and abs(p[1])<.1 for p in obstacles), obstacles
            print('PASS: nearby blind-sector obstacle survives TTL; distant old cells expire.')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        print('Results:', output)


if __name__ == '__main__':
    main()
