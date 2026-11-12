#!/usr/bin/env python3
"""Exercise the full-size primitive robot in an isolated headless Gazebo run.

Source the workspace, then run this file. Results and launch logs go to a new
/tmp directory. Only processes started by this test are stopped on exit.
"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def rotation(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    output = Path(tempfile.mkdtemp(prefix='clearing_primitives_smoke_'))
    os.environ.setdefault('ROS_DOMAIN_ID', '173')
    os.environ.setdefault('GAZEBO_MASTER_URI', 'http://127.0.0.1:11373')
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    stow = yaml.safe_load(
        (PACKAGE / 'config/clearing_robot_primitives.yaml').read_text())['initialize_clearing_arm']['ros__parameters']['positions']
    names = [f'joint{i}' for i in range(1, 8)]
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'multimap_nav_simulation', 'clearing_robot_primitives.launch.py',
         'headless:=true'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('smoke_clearing_primitives')
    transforms = Buffer()
    listener = TransformListener(transforms, node)
    entity_state = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    data, counts = {}, {}
    observed_tf = set()
    arm_errors = []
    recording = False

    def receive(key):
        def callback(message):
            data[key] = message
            counts[key] = counts.get(key, 0) + 1
            if key == 'tf':
                observed_tf.update((item.header.frame_id, item.child_frame_id)
                                   for item in message.transforms)
            if key == 'joints' and recording:
                positions = dict(zip(message.name, message.position))
                if all(name in positions for name in names):
                    arm_errors.append(max(abs(positions[name]-target)
                                          for name, target in zip(names, stow)))
        return callback

    subscriptions = []
    for key, kind, topic in (
            ('clock', Clock, '/clock'), ('lio', Odometry, '/odom'),
            ('lio_native', Odometry, '/fast_lio/odometry'),
            ('joints', JointState, '/joint_states'), ('imu', Imu, '/livox/imu_raw'),
            ('corrected_scan', CustomMsg, '/fast_lio/lidar'), ('tf', TFMessage, '/tf'),
            ('commands', Float64MultiArray, '/tracks_velocity_controller/commands')):
        subscriptions.append(node.create_subscription(kind, topic, receive(key), qos_profile_sensor_data))
    for index in range(1, 6):
        subscriptions.append(node.create_subscription(
            PointCloud2, f'/livox/lidar_{index}_PointCloud2',
            receive(f'cloud_{index}'), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(
            CustomMsg, f'/livox/lidar_{index}', receive(f'raw_{index}'), qos_profile_sensor_data))
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    arm = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError(f'Launch exited {process.returncode}; see {output / "launch.log"}')
            rclpy.spin_once(node, timeout_sec=0.02)

    def wait(future, timeout=30.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            spin(0.05)
        assert future.done(), 'Timed out waiting for ROS response'
        return future.result()

    def base_pose():
        # Query physics only for test assertions; the robot publishes no truth odom.
        request = GetEntityState.Request()
        request.name = 'clearing_robot_primitives::base_link'
        request.reference_frame = 'world'
        response = wait(entity_state.call_async(request))
        assert response.success, response
        return response.state.pose

    def pose():
        measured = base_pose()
        p, q = measured.position, measured.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [p.x, p.y, p.z, yaw]

    def sim_time():
        stamp = data['clock'].clock
        return stamp.sec + stamp.nanosec / 1e9

    def drive(linear, angular, seconds):
        start, deadline = sim_time(), time.monotonic() + 240
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        while sim_time()-start < seconds and time.monotonic() < deadline:
            publisher.publish(command)
            spin(0.05)
        assert sim_time()-start >= seconds, 'Simulation stopped advancing'

    def move_arm(target):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions, point.velocities = target, [0.0]*7
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        handle = wait(arm.send_goal_async(goal))
        assert handle.accepted, 'Arm action rejected'
        result = wait(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED, result
        assert result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, result
        spin(0.5)
        positions = dict(zip(data['joints'].name, data['joints'].position))
        error = max(abs(positions[name]-value) for name, value in zip(names, target))
        assert error < 0.03, (target, positions)
        return error

    try:
        required = ['clock', 'lio', 'lio_native', 'joints', 'imu', 'commands', 'tf', 'corrected_scan']
        required += [f'{kind}_{index}' for index in range(1, 6) for kind in ('cloud', 'raw')]
        deadline = time.monotonic() + 240
        while time.monotonic() < deadline and not all(key in data for key in required):
            spin(0.2)
        assert all(key in data for key in required), counts
        assert controllers.wait_for_service(timeout_sec=10)
        assert entity_state.wait_for_service(timeout_sec=10)
        states = {item.name: item.state for item in
                  wait(controllers.call_async(ListControllers.Request())).controller}
        assert states == {'joint_state_broadcaster': 'active', 'arm_controller': 'active',
                          'tracks_velocity_controller': 'active'}, states
        spin(1.0)
        assert arm.wait_for_server(timeout_sec=10)
        initial = pose()
        assert -0.03 < initial[2] < 0.05, initial
        assert data['lio'].header.frame_id == 'odom'
        assert data['lio'].child_frame_id == 'base_link'
        assert data['imu'].header.frame_id == 'imu_link'
        assert abs(data['imu'].linear_acceleration.z - 9.81) < 1.0
        world_base = base_pose()
        world_rotation = rotation(world_base.orientation)
        world_translation = np.array([world_base.position.x, world_base.position.y, world_base.position.z])
        lidar_results = {}
        for index in range(1, 6):
            cloud = data[f'cloud_{index}']
            assert cloud.header.frame_id == f'rslidar_{index}'
            raw = data[f'raw_{index}']
            assert raw.header.frame_id == cloud.header.frame_id
            assert raw.point_num == len(raw.points) == 3000
            fields = {field.name: field for field in cloud.fields}
            dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                              'offsets': [fields[key].offset for key in ('x', 'y', 'z')],
                              'itemsize': cloud.point_step})
            values = np.frombuffer(cloud.data, dtype=dtype)
            xyz = np.column_stack([values[key] for key in ('x', 'y', 'z')])
            ranges = np.linalg.norm(xyz, axis=1)
            valid = np.isfinite(xyz).all(axis=1) & (ranges > 0.1)
            assert np.sum(ranges[valid] > 5.0) > 500, f'Avia {index} has too few distant returns'
            # Each Avia sees forward in its OWN frame, never a synthetic 360 scan.
            assert np.all(xyz[valid, 0] > 0)
            azimuth = np.arctan2(xyz[valid, 1], xyz[valid, 0])
            assert max(abs(azimuth)) < math.radians(36)
            transform = transforms.lookup_transform(
                'base_link', f'rslidar_{index}', rclpy.time.Time()).transform
            base_points = xyz[valid] @ rotation(transform.rotation).T + np.array([
                transform.translation.x, transform.translation.y, transform.translation.z])
            world_points = base_points @ world_rotation.T + world_translation
            ground_points = int(np.sum(abs(world_points[:, 2]) < 0.06))
            assert ground_points > 200, f'Avia {index} cannot see enough floor points: {ground_points}'
            lidar_results[index] = {'points': raw.point_num, 'distant_returns': int(np.sum(ranges[valid] > 5)),
                                    'ground_points': ground_points,
                                    'local_azimuth_deg': np.degrees([min(azimuth), max(azimuth)]).tolist()}
        corrected = data['corrected_scan']
        assert all(point.offset_time == 0 for point in corrected.points)
        assert corrected.timebase == corrected.header.stamp.sec * 10**9 + corrected.header.stamp.nanosec
        lio_initial = data['lio'].pose.pose.position
        lio_initial_xy = np.array([lio_initial.x, lio_initial.y])
        recording = True
        print('READY', json.dumps({'pose': initial, 'controllers': states}), flush=True)
        drive(0.4, 0.0, 6.0)
        forward = pose()
        forward_feedback = data['lio'].twist.twist.linear.x
        assert 0.25 < forward_feedback < 0.55, forward_feedback
        assert forward[0]-initial[0] > 1.4, (initial, forward)
        assert abs(forward[1]-initial[1]) < 0.4, (initial, forward)
        spin(1.2)
        lio_forward = data['lio'].pose.pose.position
        estimated_distance = np.linalg.norm(np.array([lio_forward.x, lio_forward.y]) - lio_initial_xy)
        assert abs(estimated_distance - np.linalg.norm(np.array(forward[:2])-initial[:2])) < 0.6
        assert max(abs(value) for value in data['commands'].data) < 1e-6
        turn_start = pose()[3]
        q = data['lio'].pose.pose.orientation
        lio_turn_start = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        drive(0.0, 0.2, 8.0)
        turned = pose()
        yaw_feedback = data['lio'].twist.twist.angular.z
        assert 0.12 < yaw_feedback < 0.28, yaw_feedback
        yaw_change = math.atan2(math.sin(turned[3]-turn_start), math.cos(turned[3]-turn_start))
        assert yaw_change > 0.5, (turn_start, turned)
        spin(1.2)
        q = data['lio'].pose.pose.orientation
        lio_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        lio_yaw_change = math.atan2(math.sin(lio_yaw-lio_turn_start), math.cos(lio_yaw-lio_turn_start))
        assert abs(lio_yaw_change-yaw_change) < 0.3, (lio_yaw_change, yaw_change)
        assert max(abs(value) for value in data['commands'].data) < 1e-6
        assert abs(data['lio'].twist.twist.linear.x) < 0.1
        assert abs(data['lio'].twist.twist.angular.z) < 0.1
        assert max(arm_errors) < 0.03, max(arm_errors)
        assert -0.03 < turned[2] < 0.05, turned
        recording = False
        target = [value+delta for value, delta in zip(stow, [0.04, -0.03, -0.03, 0.03, 0.04, 0.04, 0.06])]
        moved_error = move_arm(target)
        returned_error = move_arm(stow)
        odom_topics = [name for name, kinds in node.get_topic_names_and_types()
                       if 'nav_msgs/msg/Odometry' in kinds]
        assert set(odom_topics) == {'/odom', '/fast_lio/odometry'}, odom_topics
        assert node.count_publishers('/odom') == 1
        assert ('odom', 'base_link') in observed_tf
        result = {'initial_pose': initial, 'forward_pose': forward, 'turn_pose': turned,
                  'yaw_change': yaw_change, 'lio_yaw_change': lio_yaw_change,
                  'max_navigation_arm_error_rad': max(arm_errors),
                  'arm_motion_error_rad': moved_error, 'arm_return_error_rad': returned_error,
                  'lidars': lidar_results, 'lio_forward_distance': float(estimated_distance),
                  'forward_velocity_feedback': forward_feedback,
                  'yaw_velocity_feedback': yaw_feedback,
                  'controllers': states, 'odom_topics': odom_topics, 'message_counts': counts,
                  'watchdog_zero_command': True}
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        print('PASS', json.dumps(result), flush=True)
        print(f'Results: {output}', flush=True)
    finally:
        arm.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        log.close()


if __name__ == '__main__':
    main()
