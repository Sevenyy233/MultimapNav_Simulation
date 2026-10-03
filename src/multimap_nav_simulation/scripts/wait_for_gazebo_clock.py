#!/usr/bin/env python3
"""Wait until Gazebo publishes a progressing clock after loading a saved world."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main(args=None):
    rclpy.init(args=args)
    node = Node('wait_for_gazebo_clock')
    previous = None
    ready = False

    def clock_callback(message):
        nonlocal previous, ready
        current = message.clock.sec * 1000000000 + message.clock.nanosec
        if previous is not None and current > previous:
            ready = True
        previous = current

    node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data)
    timeout = float(node.declare_parameter('timeout', 120.0).value)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and not ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if ready:
            node.get_logger().info('Gazebo clock is progressing; world is ready for spawning')
        else:
            node.get_logger().error('Timed out waiting for Gazebo /clock')
        return 0 if ready else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
