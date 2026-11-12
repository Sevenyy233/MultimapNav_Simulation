#!/usr/bin/env python3
"""Simulation only: robot/robot.xacro with config/robot.yaml."""

from multimap_nav_simulation.simulation_launch import small_robot_launch


def generate_launch_description():
    return small_robot_launch("diff_drive")
