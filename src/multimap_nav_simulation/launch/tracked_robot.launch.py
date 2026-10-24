#!/usr/bin/env python3
"""Start robot/tracked_robot.xacro with config/tracked_robot.yaml."""

from multimap_nav_simulation.simulation_launch import small_robot_launch


def generate_launch_description():
    return small_robot_launch("tracked")
