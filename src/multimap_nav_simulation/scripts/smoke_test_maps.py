#!/usr/bin/env python3
"""Exercise installed Gazebo maps with isolated ROS domains and Gazebo masters.

Each case checks progressing simulation time, active controllers, live lidar,
stable ground contact and actual /cmd_vel motion. Optional cases drive upstairs
and follow the connecting ramps. Only processes created by this script are stopped.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

from multimap_nav_simulation.map_catalog import load_catalog
from validate_map_suite import support_height
from stl_to_navigation_pcd import read_binary_stl


def unused_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def yaw_of(orientation):
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def run_case(name, record, level, robot, package, log_dir, domain):
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    env = os.environ.copy()
    env.update({"GAZEBO_MASTER_URI": f"http://127.0.0.1:{unused_port()}",
                "ROS_LOG_DIR": str(log_dir / "ros"), "GAZEBO_MODEL_DATABASE_URI": ""})
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(f"map_smoke_{name}_{level}", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state = {"clock": None, "odom": None, "clouds": 0, "cloud_points": 0}
    node.create_subscription(Clock, "/clock", lambda msg: state.update(
        clock=msg.clock.sec + msg.clock.nanosec / 1e9), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", lambda msg: state.update(odom=msg), qos_profile_sensor_data)

    def cloud_received(msg):
        state["clouds"] += 1
        state["cloud_points"] = msg.width * msg.height

    node.create_subscription(PointCloud2, "/points_raw", cloud_received, qos_profile_sensor_data)
    velocity = node.create_publisher(Twist, "/cmd_vel", 10)
    controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
    launch = {"tracked": "tracked_robot.launch.py", "diff_drive": "robot.launch.py",
              "clearing": "clearing_robot.launch.py"}[robot]
    args = ["ros2", "launch", "multimap_nav_simulation", launch,
            f"world_file_name:={name}", "headless:=true", "gui:=false",
            f"spawn_level:={'lower' if level == 'ramp' else level}"]
    if level == "ramp":
        pose = record["ramp_start"]
        args.extend(f"spawn_{axis}:={v}" for axis, v in zip(("x", "y", "z", "yaw"), pose))
    else:
        pose = (record["upper_spawn"] if level == "upper" else
                record.get("clearing_spawn", record["spawn"]) if robot == "clearing" else record["spawn"])
    logfile = log_dir / f"{name}_{level}_{robot}.log"
    process = None
    last_command = 0.0

    def publish(command):
        nonlocal last_command
        now = time.monotonic()
        if now - last_command >= .05:
            velocity.publish(command)
            last_command = now

    def wait_until(predicate, timeout=90):
        deadline = time.monotonic() + timeout
        last_notice = time.monotonic()
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if process.poll() is not None:
                raise RuntimeError(f"Launch exited {process.returncode}: {logfile}")
            if predicate():
                return
            if time.monotonic() - last_notice >= 15 and state["odom"] is not None:
                p = state["odom"].pose.pose.position
                print(f"PROGRESS {name}/{level}: t={state['clock']:.1f}; xyz=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})", flush=True)
                last_notice = time.monotonic()
        p = state["odom"].pose.pose.position if state["odom"] is not None else None
        location = [p.x, p.y, p.z] if p is not None else None
        raise TimeoutError(f"Case timed out: {name}/{level}; t={state['clock']}; xyz={location}; {logfile}")

    def advance(seconds, command=None):
        start = state["clock"]

        def update():
            publish(command or Twist())
            return state["clock"] - start >= seconds

        wait_until(update, max(45, seconds * 8))

    try:
        with logfile.open("w") as stream:
            process = subprocess.Popen(args, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait_until(lambda: state["clock"] is not None and state["odom"] is not None and state["clouds"] > 0
                       and controllers.service_is_ready() and "robot ready" in logfile.read_text())
            # Query after startup has finished. Rapidly polling list_controllers
            # while its variable-length response grows can abort Fast DDS here.
            future = controllers.call_async(ListControllers.Request())
            wait_until(future.done)
            statuses = {item.name: item.state for item in future.result().controller}
            assert statuses.get("diff_drive_controller") == "active", statuses
            assert statuses.get("joint_state_broadcaster") == "active", statuses
            if robot == "clearing":
                assert statuses.get("arm_controller") == "active", statuses
            advance(.6)
            start = state["odom"].pose.pose.position
            first = np.array([start.x, start.y, start.z])
            expected_ground = support_height(read_binary_stl(package / record["mesh"]), pose)
            assert abs(first[2] - expected_ground) < .25, (name, level, first, expected_ground)
            before_clouds = state["clouds"]
            if level == "ramp":
                for x, y in record["ramp_waypoints"]:
                    def follow():
                        p = state["odom"].pose.pose.position
                        dx, dy = x - p.x, y - p.y
                        distance = math.hypot(dx, dy)
                        error = math.atan2(dy, dx) - yaw_of(state["odom"].pose.pose.orientation)
                        error = math.atan2(math.sin(error), math.cos(error))
                        command = Twist()
                        command.angular.z = float(np.clip(error * 2, -1.0, 1.0))
                        command.linear.x = (.4 * min(1, distance * 2) * math.cos(error)
                                            if abs(error) < .25 else 0.)
                        publish(command)
                        return distance < .12

                    wait_until(follow, timeout=240)
                    advance(.25)
                advance(.6)
                z = state["odom"].pose.pose.position.z
                assert abs(z - (record["upper_spawn"][2] - .08)) < .2, (name, z)
            else:
                command = Twist(); command.linear.x = .18
                advance(1.2, command)
                advance(.3)
            last = state["odom"].pose.pose.position
            displacement = math.hypot(last.x - first[0], last.y - first[1])
            assert displacement > .04, f"Robot did not move: {name}/{level}, {displacement}"
            assert state["clouds"] > before_clouds and state["cloud_points"] > 0, state
            if level != "ramp":
                assert displacement < .75, f"Unexpected displacement: {name}/{robot}, {displacement}"
                assert abs(last.z - first[2]) < .18, f"Robot lost ground contact: {name}/{level}"
            result = {"name": name, "level": level, "robot": robot, "passed": True,
                      "displacement": displacement, "start_z": float(first[2]), "end_z": last.z,
                      "cloud_messages": state["clouds"], "cloud_points": state["cloud_points"],
                      "controllers": statuses, "log": str(logfile)}
            print(f"PASS {name}/{level}/{robot}: moved {displacement:.3f} m; z={last.z:.3f}; lidar={state['clouds']}", flush=True)
            return result
    finally:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=12)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--maps", nargs="+", help="Map names; omit for all 15.")
    parser.add_argument("--levels", nargs="+", choices=("lower", "upper", "ramp"), default=["lower"])
    parser.add_argument("--robot", choices=("tracked", "diff_drive", "clearing"), default="tracked")
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/multimap-gazebo-smoke"))
    parser.add_argument("--domain-base", type=int, default=90, help="Use distinct bases for concurrent test runs.")
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    records = load_catalog(args.package_dir)
    results = []; domain = args.domain_base
    for name in args.maps or records:
        for level in args.levels:
            record = records[name]
            if level == "upper" and "upper_spawn" not in record:
                continue
            if level == "ramp" and "ramp_start" not in record:
                continue
            domain += 1
            print(f"START {name}/{level}/{args.robot}", flush=True)
            try:
                results.append(run_case(name, record, level, args.robot, args.package_dir, args.log_dir, domain))
            except Exception as error:
                results.append({"name": name, "level": level, "robot": args.robot, "passed": False,
                                "error": str(error)})
                print(f"FAIL {name}/{level}: {error}", flush=True)
            (args.log_dir / "report.json").write_text(json.dumps(results, indent=2) + "\n")
    assert all(result["passed"] for result in results), f"Failures: see {args.log_dir / 'report.json'}"


if __name__ == "__main__":
    main()
