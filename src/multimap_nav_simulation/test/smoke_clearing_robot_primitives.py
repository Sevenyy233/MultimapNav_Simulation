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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


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
            ('odom', Odometry, '/odom'), ('truth', Odometry, '/truth/odometry'),
            ('joints', JointState, '/joint_states'), ('imu', Imu, '/livox/imu_raw'),
            ('cloud', PointCloud2, '/points_raw'), ('tf', TFMessage, '/tf'),
            ('commands', Float64MultiArray, '/tracks_velocity_controller/commands')):
        subscriptions.append(node.create_subscription(kind, topic, receive(key), qos_profile_sensor_data))
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

    def pose():
        p, q = data['odom'].pose.pose.position, data['odom'].pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [p.x, p.y, p.z, yaw]

    def sim_time():
        stamp = data['odom'].header.stamp
        return stamp.sec + stamp.nanosec / 1e9

    def drive(linear, angular, seconds):
        start, deadline = sim_time(), time.monotonic() + 90
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
        deadline = time.monotonic() + 150
        while time.monotonic() < deadline and not all(key in data for key in
                ('odom', 'truth', 'joints', 'imu', 'cloud', 'commands', 'tf')):
            spin(0.2)
        assert all(key in data for key in
                   ('odom', 'truth', 'joints', 'imu', 'cloud', 'commands', 'tf')), counts
        assert controllers.wait_for_service(timeout_sec=10)
        states = {item.name: item.state for item in
                  wait(controllers.call_async(ListControllers.Request())).controller}
        assert states == {'joint_state_broadcaster': 'active', 'arm_controller': 'active',
                          'tracks_velocity_controller': 'active'}, states
        spin(1.0)
        assert arm.wait_for_server(timeout_sec=10)
        initial = pose()
        assert -0.03 < initial[2] < 0.05, initial
        assert data['odom'].header.frame_id == 'world'
        assert data['odom'].child_frame_id == 'base_link'
        assert data['truth'].header.frame_id == 'odom'
        assert data['imu'].header.frame_id == 'imu_link'
        assert abs(data['imu'].linear_acceleration.z - 9.81) < 1.0
        cloud = data['cloud']
        assert cloud.header.frame_id == 'lidar_link'
        assert cloud.width*cloud.height == 720*16
        fields = {field.name: field for field in cloud.fields}
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                          'offsets': [fields[key].offset for key in ('x', 'y', 'z')],
                          'itemsize': cloud.point_step})
        values = np.frombuffer(cloud.data, dtype=dtype)
        xyz = np.column_stack([values[key] for key in ('x', 'y', 'z')])
        ranges = np.linalg.norm(xyz[np.isfinite(xyz).all(axis=1)], axis=1)
        assert np.sum(ranges > 5.0) > 5000, 'LiDAR has too few distant scene returns'
        recording = True
        print('READY', json.dumps({'pose': initial, 'controllers': states}), flush=True)
        drive(0.4, 0.0, 6.0)
        forward = pose()
        assert forward[0]-initial[0] > 1.4, (initial, forward)
        assert abs(forward[1]-initial[1]) < 0.4, (initial, forward)
        spin(1.2)
        assert max(abs(value) for value in data['commands'].data) < 1e-6
        turn_start = pose()[3]
        drive(0.0, 0.2, 8.0)
        turned = pose()
        yaw_change = math.atan2(math.sin(turned[3]-turn_start), math.cos(turned[3]-turn_start))
        assert yaw_change > 0.5, (turn_start, turned)
        spin(1.2)
        assert max(abs(value) for value in data['commands'].data) < 1e-6
        assert max(arm_errors) < 0.03, max(arm_errors)
        assert -0.03 < turned[2] < 0.05, turned
        recording = False
        target = [value+delta for value, delta in zip(stow, [0.04, -0.03, -0.03, 0.03, 0.04, 0.04, 0.06])]
        moved_error = move_arm(target)
        returned_error = move_arm(stow)
        odom_topics = [name for name, kinds in node.get_topic_names_and_types()
                       if 'nav_msgs/msg/Odometry' in kinds]
        assert set(odom_topics) == {'/odom', '/truth/odometry'}, odom_topics
        assert node.count_publishers('/odom') == 1
        assert ('odom', 'base_link') in observed_tf
        result = {'initial_pose': initial, 'forward_pose': forward, 'turn_pose': turned,
                  'yaw_change': yaw_change, 'max_navigation_arm_error_rad': max(arm_errors),
                  'arm_motion_error_rad': moved_error, 'arm_return_error_rad': returned_error,
                  'finite_lidar_points': len(ranges), 'distant_lidar_points': int(np.sum(ranges > 5)),
                  'controllers': states, 'odom_topics': odom_topics, 'message_counts': counts,
                  'watchdog_zero_command': True}
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        print('PASS', json.dumps(result), flush=True)
        print(f'Results: {output}', flush=True)
    finally:
        arm.destroy()
        node.destroy_node()
        rclpy.shutdown()
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
