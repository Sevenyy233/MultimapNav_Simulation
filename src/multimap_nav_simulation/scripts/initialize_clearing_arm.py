#!/usr/bin/env python3
"""Confirm the initial arm pose before the launch enables the drive controller."""

import math
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


def wait_for_future(node, future, timeout):
    """Use a wall-clock deadline so missing /clock cannot block startup forever."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
    if not future.done():
        raise RuntimeError('Timed out waiting for arm controller')
    return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = Node('initialize_clearing_arm')
    client = None
    result_code = 1
    try:
        positions = list(node.declare_parameter('positions', [0.0] * 7).value)
        duration = float(node.declare_parameter('move_duration', 1.0).value)
        timeout = float(node.declare_parameter('timeout', 120.0).value)
        if len(positions) != 7 or not all(math.isfinite(x) for x in positions):
            raise RuntimeError('positions must contain seven finite joint angles')
        if not math.isfinite(duration) or duration <= 0 or not math.isfinite(timeout) or timeout <= 0:
            raise RuntimeError('move_duration and timeout must be positive finite numbers')

        client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('Arm trajectory action server is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'joint{index}' for index in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 7
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]
        handle = wait_for_future(node, client.send_goal_async(goal), timeout)
        if not handle.accepted:
            raise RuntimeError('Arm controller rejected the initial pose')
        result = wait_for_future(node, handle.get_result_async(), timeout)
        if (result.status != GoalStatus.STATUS_SUCCEEDED or
                result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError(f'Arm initialization failed: {result.result.error_string}')
        node.get_logger().info(f'Arm initialized and held at {positions} radians')
        result_code = 0
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())
