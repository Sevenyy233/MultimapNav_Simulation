#!/usr/bin/env python3
"""Convert /cmd_vel into ros2_control track velocities without wheel odometry."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def track_velocities(linear, angular, radius, separation, count, limit):
    """Keep curvature when saturating the two differential wheel speeds."""
    left = (linear - angular * separation / 2.0) / radius
    right = (linear + angular * separation / 2.0) / radius
    scale = max(1.0, abs(left) / limit, abs(right) / limit)
    return [left / scale] * count + [right / scale] * count


class ClearingTracksDrive(Node):
    def __init__(self):
        super().__init__('clearing_tracks_drive')
        defaults = {
            'wheel_radius': 0.48, 'wheel_separation': 2.8, 'wheels_per_side': 5,
            'publish_rate': 50.0, 'cmd_vel_timeout': 0.5,
            'max_linear_velocity': 1.0, 'max_angular_velocity': 0.35,
            'max_linear_acceleration': 0.3, 'max_angular_acceleration': 0.2,
            'max_wheel_velocity': 8.0,
        }
        self.settings = {key: self.declare_parameter(key, default).value
                         for key, default in defaults.items()}
        if any(not math.isfinite(value) or value <= 0 for value in self.settings.values()):
            raise ValueError('Drive parameters must be positive finite numbers')
        self.target = [0.0, 0.0]
        self.current = [0.0, 0.0]
        self.received_at = None
        self.previous_time = self.get_clock().now()
        self.publisher = self.create_publisher(
            Float64MultiArray, '/tracks_velocity_controller/commands', 1)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.receive, 1)
        self.timer = self.create_timer(1.0 / self.settings['publish_rate'], self.update)
        self.get_logger().info('Differential track drive ready; odometry comes from Gazebo truth')

    def receive(self, message):
        values = [message.linear.x, message.angular.z]
        if not all(math.isfinite(value) for value in values):
            self.target = [0.0, 0.0]
            self.received_at = None
            return
        self.target = [max(-self.settings[key], min(self.settings[key], value))
                       for value, key in zip(values, ('max_linear_velocity', 'max_angular_velocity'))]
        self.received_at = time.monotonic()

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.previous_time).nanoseconds / 1e9
        self.previous_time = now
        expired = (self.received_at is None or
                   time.monotonic() - self.received_at > self.settings['cmd_vel_timeout'])
        if expired or elapsed < 0.0:
            # Timeout is an immediate stop; ordinary commands use acceleration
            # limits in simulation time. A clock reset also requires a new command.
            self.current = [0.0, 0.0]
            self.target = [0.0, 0.0]
            self.received_at = None
        else:
            elapsed = min(elapsed, 0.1)
            for index, key in enumerate(('max_linear_acceleration', 'max_angular_acceleration')):
                step = self.settings[key] * elapsed
                delta = self.target[index] - self.current[index]
                self.current[index] += max(-step, min(step, delta))
        command = Float64MultiArray()
        command.data = track_velocities(
            *self.current, self.settings['wheel_radius'], self.settings['wheel_separation'],
            self.settings['wheels_per_side'], self.settings['max_wheel_velocity'])
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ClearingTracksDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
