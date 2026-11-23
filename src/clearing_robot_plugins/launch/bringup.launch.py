"""! @file 一条命令启动匹配的清舱场景、MultiMapNav 导航与图形界面。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """! @brief 使用同名场景启动 Gazebo 和导航，默认打开 Gazebo 客户端与 RViz2。
    @return 仿真、导航及其共用启动参数。
    """
    simulation_share = Path(get_package_share_directory("clearing_robot_simulation"))
    plugins_share = Path(get_package_share_directory("clearing_robot_plugins"))
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "clearing_robot.launch.py")),
        launch_arguments={
            "world_file_name": LaunchConfiguration("scene"),
            "gui": LaunchConfiguration("gui"),
            "use_fast_lio": "false",
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(plugins_share / "launch" / "navigation.launch.py")),
        launch_arguments={
            "scene": LaunchConfiguration("scene"),
            "map_mode": LaunchConfiguration("map_mode"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "controller": LaunchConfiguration("controller"),
            "onnx_model": LaunchConfiguration("onnx_model"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="training", choices=["training", "evaluation", "training_rough_v2"]),
        DeclareLaunchArgument("map_mode", default_value="auto",
                              choices=["auto", "costmap2d", "gridmap", "octomap"]),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("controller", default_value="mppi",
                              choices=["mppi", "drl", "training"]),
        DeclareLaunchArgument("onnx_model", default_value=""),
        simulation,
        navigation,
    ])
