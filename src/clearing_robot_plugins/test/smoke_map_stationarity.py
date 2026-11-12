#!/usr/bin/env python3
"""Check that robot motion does not translate existing map geometry or its frame."""
import argparse
from collections import defaultdict, deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import socket
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from multimap_nav_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
from octomap_msgs.msg import Octomap
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def xyz(message):
    offsets = {f.name: f.offset for f in message.fields}
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['>f4' if message.is_bigendian else '<f4']*3,
                      'offsets': [offsets[k] for k in ['x', 'y', 'z']], 'itemsize': message.point_step})
    points = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step)).reshape(-1)
    result = np.column_stack([points[k] for k in ['x', 'y', 'z']])
    return result[np.isfinite(result).all(axis=1)]


def matrix(transform):
    t, q = transform.translation, transform.rotation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['navigation', 'mapping'], required=True)
    parser.add_argument('--visual', action='store_true')
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=f'{args.mode}_', dir=args.output_root)).resolve()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    domain = secrets.randbelow(150)+40
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                      GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
                      ROS_LOG_DIR=str(output/'ros_logs'), GAZEBO_MODEL_DATABASE_URI='')
    log = (output/'launch.log').open('w')
    launch = ['ros2', 'launch', 'clearing_robot_plugins', f'{args.mode}.launch.py',
              'headless:=true', f'use_rviz:={str(args.visual).lower()}']
    if args.mode == 'mapping':
        launch.append(f'map_output_file:={output / "map.pcd"}')
    process = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('map_stationarity_observer',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data, counts = {}, defaultdict(int)
    clouds, registered = deque(maxlen=50), deque(maxlen=50)
    sources = defaultdict(set)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(key):
        def callback(message):
            data[key] = message
            counts[key] += 1
            if key == 'live':
                clouds.append(message)
            if key == 'registered':
                registered.append(message)
        return callback

    subscriptions = [node.create_subscription(kind, topic, receive(key), qos)
        for key, kind, topic, qos in [
            ('prior', PointCloud2, '/static_map_cloud', latched),
            ('octomap', Octomap, '/octomap_map', latched),
            ('map', PointCloud2, '/Laser_map', qos_profile_sensor_data),
            ('live', PointCloud2, '/clearing/points', qos_profile_sensor_data),
            ('registered', PointCloud2, '/cloud_registered', qos_profile_sensor_data),
            ('odom', Odometry, '/odom', qos_profile_sensor_data),
            ('healthy', Bool, '/clearing/localization_healthy', qos_profile_sensor_data)]]

    def transforms(message):
        for t in message.transforms:
            if t.child_frame_id in ('odom', 'base_link', 'camera_init', 'body'):
                sources[t.child_frame_id].add(t.header.frame_id)

    subscriptions.extend([node.create_subscription(TFMessage, '/tf', transforms, 100),
                          node.create_subscription(TFMessage, '/tf_static', transforms, latched)])
    command = node.create_publisher(Twist, '/cmd_vel', 10)
    physics = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert process.poll() is None, f'Launch exited; {output}'
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(future, timeout=15):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            spin(.05)
        assert future.done(), f'Request timed out; {output}'
        return future.result()

    def map_base():
        return matrix(buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform)

    def physical_base():
        request = GetEntityState.Request()
        request.name = 'clearing_robot_primitives::base_link'
        request.reference_frame = 'world'
        response = wait(physics.call_async(request))
        assert response.success
        pose = response.state.pose
        out = np.eye(4)
        q = pose.orientation
        out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        out[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return out

    def screenshot(name):
        if not args.visual:
            return
        windows = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'], text=True)
        own = []
        for window in re.findall(r'0x[0-9a-fA-F]+', windows):
            owner = subprocess.run(['xprop', '-id', window, '_NET_WM_PID'], text=True, capture_output=True).stdout
            match = re.search(r'= (\d+)', owner)
            if not match:
                continue
            try:
                pid = int(match[1])
                if os.getpgid(pid) != process.pid or not Path(f'/proc/{pid}/comm').read_text().startswith('rviz'):
                    continue
            except (FileNotFoundError, ProcessLookupError):
                continue
            info = subprocess.check_output(['xwininfo', '-id', window], text=True)
            width = int(re.search(r'Width: (\d+)', info)[1])
            height = int(re.search(r'Height: (\d+)', info)[1])
            if width > 600 and height > 400:
                own.append((width*height, window))
        assert own, 'Own RViz window not found'
        subprocess.run(['xwd', '-silent', '-id', max(own)[1], '-out', str(output/f'{name}.xwd')], check=True)
        subprocess.run(['convert', str(output/f'{name}.xwd'), str(output/f'{name}.png')], check=True)
        (output/f'{name}.xwd').unlink()

    try:
        deadline = time.monotonic()+180
        while time.monotonic() < deadline:
            spin(.1)
            ready = ('odom' in data and buffer.can_transform('map', 'base_link', rclpy.time.Time())
                     and counts['registered'] > 5 and physics.service_is_ready())
            if args.mode == 'navigation':
                ready = (ready and 'prior' in data and 'octomap' in data and counts['live'] > 5
                         and 'healthy' in data and data['healthy'].data and navigator.server_is_ready())
            else:
                ready = ready and counts['map'] > 2 and data['map'].width > 1000
            if ready:
                break
        assert ready, dict(counts)
        spin(2)

        def alignment():
            queue = clouds if args.mode == 'navigation' else registered
            for message in reversed(queue):
                try:
                    t = matrix(buffer.lookup_transform('map', message.header.frame_id, message.header.stamp).transform)
                except Exception:
                    continue
                points = xyz(message)[::max(1, message.width//3000)]
                points = points@t[:3, :3].T+t[:3, 3]
                distances = tree.query(points)[0]
                return {'median_m': float(np.median(distances)), 'p90_m': float(np.quantile(distances, .9))}
            raise AssertionError('No timestamp-matched point cloud TF')

        before_map = map_base()
        before_physical = physical_base()
        world_map = before_physical@np.linalg.inv(before_map)
        if args.mode == 'navigation':
            reference = xyz(data['prior'])
        else:
            # Test-only physical reference: newly visible surfaces need not
            # occur in the sparse first scans. Comparing only with those scans
            # confounds field-of-view change with localization drift.
            asset = Path(get_package_share_directory('clearing_robot_plugins'))/'maps/clearing_enclosed.pcd'
            payload = asset.read_bytes().split(b'DATA binary\n', 1)[1]
            world_points = np.frombuffer(payload, dtype='<f4').reshape(-1, 3)
            reference = (world_points-world_map[:3, 3])@world_map[:3, :3]
        tree = cKDTree(reference)
        before_alignment = alignment()
        before_alias = matrix(buffer.lookup_transform('map', 'camera_init', rclpy.time.Time()).transform)
        if args.mode == 'navigation':
            assert data['prior'].header.frame_id == data['octomap'].header.frame_id == 'map'
            prior_hash = hashlib.sha256(bytes(data['prior'].data)).hexdigest()
            octo_hash = hashlib.sha256(bytes(v & 255 for v in data['octomap'].data)).hexdigest()
        else:
            initial_map = bytes(data['map'].data)
            initial_step = data['map'].point_step
            assert data['map'].header.frame_id == 'camera_init'
        screenshot('before')
        if args.mode == 'navigation':
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(before_map[0, 3]+2)
            goal.pose.pose.position.y = float(before_map[1, 3])
            goal.pose.pose.position.z = .01
            goal.pose.pose.orientation.w = 1.
            handle = wait(navigator.send_goal_async(goal))
            assert handle.accepted
            result = wait(handle.get_result_async(), 150)
            assert result.status == GoalStatus.STATUS_SUCCEEDED, result
        else:
            started = node.get_clock().now().nanoseconds
            deadline = time.monotonic()+60
            forward = Twist()
            forward.linear.x = .18
            while (node.get_clock().now().nanoseconds-started)/1e9 < 8 and time.monotonic() < deadline:
                command.publish(forward)
                spin(.05)
            for _ in range(20):
                command.publish(Twist())
                spin(.1)
        spin(2)
        after_map = map_base()
        after_physical = physical_base()
        displacement = float(np.linalg.norm((after_physical-before_physical)[:2, 3]))
        assert displacement > .6, displacement
        relative_error = float(np.linalg.norm((world_map@after_map-after_physical)[:2, 3]))
        assert relative_error < .2, relative_error
        after_alignment = alignment()
        (output/'measurements.json').write_text(json.dumps({
            'physical_displacement_m': displacement, 'relative_localization_error_m': relative_error,
            'alignment_before': before_alignment, 'alignment_after': after_alignment}, indent=2)+'\n')
        if args.mode == 'navigation':
            assert prior_hash == hashlib.sha256(bytes(data['prior'].data)).hexdigest()
            assert octo_hash == hashlib.sha256(bytes(v & 255 for v in data['octomap'].data)).hexdigest()
            assert data['prior'].header.frame_id == data['octomap'].header.frame_id == 'map'
            assert after_alignment['median_m'] < .2, after_alignment
            geometry_check = 'static PCD and OctoMap bytes/frame unchanged'
        else:
            assert data['map'].point_step == initial_step
            assert bytes(data['map'].data).startswith(initial_map)
            assert len(data['map'].data) > len(initial_map)
            after_alias = matrix(buffer.lookup_transform('map', 'camera_init', rclpy.time.Time()).transform)
            assert np.max(abs(before_alias-after_alias)) < 1e-9
            assert after_alignment['median_m'] < .25, after_alignment
            geometry_check = 'existing accumulated map points unchanged; map frame transform constant'
        for child in ('odom', 'base_link', 'camera_init', 'body'):
            assert len(sources[child]) == 1, (child, sources[child])
        publisher_names = [p.node_name for p in node.get_publishers_info_by_topic('/tf')]
        expected = {'fast_lio_sim_bridge', 'robot_state_publisher'}
        if args.mode == 'navigation':
            expected.add('localization_server')
        assert set(publisher_names) == expected, publisher_names
        assert len(publisher_names) == len(expected), publisher_names
        screenshot('after')
        report = {'mode': args.mode, 'physical_displacement_m': displacement,
                  'relative_localization_error_m': relative_error,
                  'geometry_check': geometry_check, 'alignment_before': before_alignment,
                  'alignment_after': after_alignment, 'tf_parents': {k: list(v) for k, v in sources.items()},
                  'tf_publisher_nodes': publisher_names,
                  'counts': dict(counts), 'ros_domain': domain, 'gazebo_port': port}
        (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print('PASS', json.dumps(report), flush=True)
    finally:
        command.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()
        print('Results:', output, flush=True)


if __name__ == '__main__':
    main()
