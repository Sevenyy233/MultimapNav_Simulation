#!/usr/bin/env python3
"""启动机器人，支持 flat、terrain、floor 三类地图及自动出生位姿。"""

from multimap_nav_simulation.simulation_launch import small_robot_launch


def generate_launch_description():
    return small_robot_launch("tracked")
