"""! @file 启动保持常驻的导航模式管理、传感器处理和 RViz2。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    """! @brief 加载同一场景的先验点云，组装常驻节点和模式管理进程。"""
    def option(name):
        """! @brief 读取名称对应的当前启动参数。"""
        return LaunchConfiguration(name).perform(context)

    plugins = Path(get_package_share_directory("clearing_robot_plugins"))
    simulation = Path(get_package_share_directory("clearing_robot_simulation"))
    robot_config = str(simulation / "config" / "clearing_robot.yaml")
    pcd = Path(option("pcd_file") or simulation / "worlds" / f"{option('scene')}.pcd").expanduser()
    if not pcd.is_file():
        raise ValueError(f"场景 PCD 不存在: {pcd}")
    model = option("onnx_model")
    if option("controller") == "drl" and not Path(model).expanduser().is_file():
        raise ValueError("DRL 导航需要可读的 onnx_model")

    nodes = [
        Node(package="clearing_robot_plugins", executable="clearing_local_map", output="screen",
             parameters=[{"use_sim_time": True, "prior_pcd": str(pcd), "points_topic": "/points_raw"}]),
        Node(package="clearing_robot_plugins", executable="clearing_avia_fusion", output="screen",
             parameters=[{"use_sim_time": True}]),
        Node(package="multimap_nav_simulation", executable="fast_lio_sim_bridge", output="screen",
             parameters=[robot_config, {"use_sim_time": True, "lidar_topic": "/clearing/no_primary_lidar",
                                         "lidar_frame": "imu_link",
                                         "body_cloud_input_topic": "/cloud_registered_body"}]),
        Node(package="clearing_robot_plugins", executable="estimator_supervisor.py", output="screen",
             parameters=[{"use_sim_time": True, "fast_lio_config": robot_config,
                          "fusion_config": str(plugins / "config" / "fast_lio_fusion.yaml")}]),
        Node(package="clearing_robot_plugins", executable="navigation_modes.py", output="screen",
             parameters=[{"pcd_file": str(pcd.resolve()), "map_mode": option("map_mode"),
                          "controller": option("controller"), "onnx_model": model}]),
    ]
    if option("controller") == "training":
        nodes.append(Node(package="clearing_robot_plugins", executable="clearing_training_control",
                          parameters=[{"use_sim_time": True}], output="screen"))
    if option("use_rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", parameters=[{"use_sim_time": True}],
                          arguments=["-d", str(plugins / "config" / "clearing.rviz")], output="screen"))
    return nodes


def generate_launch_description():
    """! @brief 声明独立导航启动参数，默认开启 RViz2 和自动模式选择。"""
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="training", choices=["training", "evaluation", "training_rough_v2"]),
        DeclareLaunchArgument("pcd_file", default_value=""),
        DeclareLaunchArgument("map_mode", default_value="auto",
                              choices=["auto", "costmap2d", "gridmap", "octomap"]),
        DeclareLaunchArgument("controller", default_value="mppi", choices=["mppi", "drl", "training"]),
        DeclareLaunchArgument("onnx_model", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_nodes),
    ])
