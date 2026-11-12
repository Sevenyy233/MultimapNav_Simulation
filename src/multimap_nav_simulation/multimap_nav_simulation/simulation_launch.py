"""Simulation-only startup for the small differential and tracked robots."""

from pathlib import Path
import re
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import xacro

from .map_catalog import gazebo_environment, resolve_world, spawn_pose


def after_success(action, following):
    def on_exit(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f"Simulation startup failed: exit {event.returncode}"))]
        return following
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def small_robot_launch(robot):
    def setup(context):
        package = Path(get_package_share_directory("multimap_nav_simulation"))
        config = context.launch_configurations
        boolean = lambda name: config[name].lower() in ("true", "1", "yes")
        world = resolve_world(package, config["world_file"].strip() or config["world_file_name"])
        defaults = spawn_pose(package, world, config["spawn_level"], robot)
        pose = [config[f"spawn_{axis}"].strip() or str(default)
                for axis, default in zip(("x", "y", "z", "yaw"), defaults)]
        xacro_path = package / "robot" / ("tracked_robot.xacro" if robot == "tracked" else "robot.xacro")
        lateral_friction = float(config["tracked_lateral_friction"])
        if not 0.0 <= lateral_friction <= 10.0:
            raise ValueError("tracked_lateral_friction must be finite and between 0 and 10")
        mappings = {"lateral_friction": str(lateral_friction)} if robot == "tracked" else {}
        use_fast_lio = robot == "tracked" and config["odometry_source"] == "fast_lio"
        use_body_control = use_fast_lio and boolean("body_velocity_control")
        if robot == "tracked":
            if config["lidar_model"] not in ("mid360", "legacy"):
                raise ValueError("lidar_model must be mid360 or legacy")
            if config["odometry_source"] != "fast_lio":
                raise ValueError("tracked_robot requires fast_lio; ground truth is evaluation-only")
            if use_fast_lio and config["lidar_model"] != "mid360":
                raise ValueError("FAST-LIO requires the timestamped MID-360 input")
            mappings.update(lidar_model=config["lidar_model"], lidar_noise_seed=config["seed"],
                            publish_ground_truth=config["publish_ground_truth"])
            mappings["cmd_vel_topic"]="/actuation/cmd_vel" if use_body_control else "/cmd_vel"
        robot_xml = ET.fromstring(xacro.process_file(str(xacro_path), mappings=mappings).toxml())
        if not boolean("enable_camera"):
            for parent in robot_xml.iter():
                for child in list(parent):
                    if child.tag == "sensor" and child.get("type") in ("depth", "camera"):
                        parent.remove(child)
        description = ET.tostring(robot_xml, encoding="unicode")
        description = " ".join(re.sub(r"<!--.*?-->", "", description, flags=re.DOTALL).split())
        sim_time = boolean("use_sim_time")
        server = ExecuteProcess(
            cmd=["gzserver", "--verbose", "--seed", config["seed"], "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", str(world)],
            additional_env=gazebo_environment(package), output="screen")
        state_publisher = Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": sim_time}], output="screen")
        wait = Node(package="multimap_nav_simulation", executable="wait_for_gazebo_clock.py",
                    parameters=[{"timeout": 120.0}], output="screen")
        spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
                     arguments=["-entity", "tracked_robot" if robot == "tracked" else "diff_drive_robot",
                                "-topic", "robot_description", "-x", pose[0], "-y", pose[1],
                                "-z", pose[2], "-Y", pose[3], "-timeout", "120"], output="screen")

        def spawner(controller):
            return Node(package="controller_manager", executable="spawner",
                        arguments=[controller, "--controller-manager", "/controller_manager",
                                   "--controller-manager-timeout", "120"], output="screen")

        joints = spawner("joint_state_broadcaster")
        drive = spawner("diff_drive_controller")
        ready_actions = [LogInfo(msg=f"{robot} robot ready.")]
        if use_fast_lio:
            estimator_config = str(package / "config/tracked_robot.yaml")
            bridge_files=[estimator_config]
            if config['bridge_overrides']:
                extra=Path(config['bridge_overrides']).resolve()
                if not extra.is_file():raise FileNotFoundError(str(extra))
                bridge_files.append(str(extra))
            ready_actions[:0] = [
                Node(package="multimap_nav_simulation", executable="fast_lio_sim_bridge",
                     parameters=[*bridge_files, {"use_sim_time": sim_time}], output="screen"),

                Node(package="fast_lio", executable="fastlio_mapping", name="fastlio_mapping",
                     parameters=[estimator_config, {"use_sim_time": sim_time}],
                     remappings=[("/Odometry", "/fast_lio/odometry"),
                                 ("/cloud_registered_body", "/fast_lio/deskewed_body_points"),
                                 ("/tf", "/fast_lio/tf")], output="screen"),
            ]
        if use_body_control:
            ready_actions.insert(1, Node(package="multimap_nav_simulation", executable="body_velocity_controller",
                parameters=[str(package / "config/tracked_robot.yaml"), {"use_sim_time":sim_time}],output="screen"))
        actions = [
            RegisterEventHandler(OnProcessExit(target_action=server,
                on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server exited."))])),
            after_success(wait, [spawn]), after_success(spawn, [joints]),
            after_success(joints, [drive]), after_success(drive, ready_actions),
            LogInfo(msg=f"World: {world}; spawn x y z yaw: {pose}"), state_publisher, server, wait,
        ]
        if boolean("gui") and not boolean("headless"):
            actions.append(ExecuteProcess(cmd=["gzclient"], additional_env=gazebo_environment(package), output="screen"))
        if boolean("publish_odom_tf") and not use_fast_lio:
            actions.append(Node(package="multimap_nav_simulation", executable="odom_tf_broadcaster",
                                parameters=[{"use_sim_time": sim_time}], output="screen"))
        return actions

    declarations = [
        DeclareLaunchArgument("world_file_name", default_value="flat_1",
                              description="flat_1..5, terrain_1..5, floor_1..7; category/name also works."),
        DeclareLaunchArgument("world_file", default_value="", description="Optional absolute world path."),
        DeclareLaunchArgument("spawn_level", default_value="lower", description="lower or upper (floor maps)."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("seed", default_value="0", description="Gazebo random seed."),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("enable_camera", default_value="true", description="Enable the optional camera; LiDAR is retained."),
        DeclareLaunchArgument("tracked_lateral_friction", default_value="0.3", description="Simulated lateral track friction; longitudinal friction is unchanged."),
        DeclareLaunchArgument("headless", default_value="false", description="Suppress the GUI."),
        DeclareLaunchArgument("publish_odom_tf", default_value="true"),
        DeclareLaunchArgument("lidar_model", default_value="mid360"),
        DeclareLaunchArgument("body_velocity_control", default_value="true",
                              description="Use sensor-feedback body control; false is a controlled diagnostic bypass."),
        DeclareLaunchArgument("bridge_overrides", default_value=""),
        DeclareLaunchArgument("odometry_source", default_value="fast_lio"),
        DeclareLaunchArgument("publish_ground_truth", default_value="true",
                              description="Publish only /evaluation/ground_truth for the independent evaluator."),
    ]
    declarations.extend(DeclareLaunchArgument(f"spawn_{axis}", default_value="",
                         description="Empty selects the map's documented spawn position.")
                        for axis in ("x", "y", "z", "yaw"))
    return LaunchDescription(declarations + [OpaqueFunction(function=setup)])
