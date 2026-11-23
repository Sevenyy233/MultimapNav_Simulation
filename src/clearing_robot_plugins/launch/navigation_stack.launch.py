"""! @file 启动可重新装载的 MultiMapNav 地图、定位与规划控制节点。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _nodes(context):
    """! @brief 组合 MultiMapNav 公共配置和选中控制模式的清舱配置。"""
    def option(name):
        """! @brief 在当前启动上下文获取一个参数的字符串值。"""
        return LaunchConfiguration(name).perform(context)

    package = Path(get_package_share_directory("clearing_robot_plugins"))
    simulation = Path(get_package_share_directory("clearing_robot_simulation"))
    config_dir = Path(get_package_share_directory("bring_up")) / "config"
    config = package / "config"
    modes = ("costmap2d", "gridmap", "octomap")
    profile_files = [str(config_dir / f"{mode}_nav_params.yaml") for mode in modes]
    profile_files += [str(config_dir / name) for name in
                      ("auto_nav_params.yaml", "sensor_navigation_params.yaml")]
    pcd = Path(option("pcd_file") or simulation / "worlds" / f"{option('scene')}.pcd").expanduser()
    if not pcd.is_file():
        raise ValueError(f"场景 PCD 不存在: {pcd}")
    controller = option("controller")
    training = controller == "training"
    if controller == "drl" and not Path(option("onnx_model")).expanduser().is_file():
        raise ValueError("DRL 导航需要可读的 onnx_model")
    if controller not in ("mppi", "drl", "training"):
        raise ValueError("controller 应为 mppi、drl 或 training")
    requested_mode = option("map_mode")
    if requested_mode not in ("auto", *modes):
        raise ValueError("map_mode 应为 auto、costmap2d、gridmap 或 octomap")

    selected_controller = (
        "clearing_robot::DrlController" if controller == "drl" else "clearing_robot::MppiController")
    planners = {
        "costmap2d": "clearing_robot::CostmapAStar",
        "gridmap": "clearing_robot::GridMapAStar",
        "octomap": "clearing_robot::OctomapAStar",
    }
    profiles = {}
    for mode in modes:
        profiles[mode] = {
            "global_planner_plugin_type": planners[mode],
            "local_controller_plugin_type": selected_controller,
            "controller_odom_topic": "/odom",
            "controller_odom_twist_in_world_frame": False,
            "collision_monitor_enabled": False,
            "controller_frequency": 10.0,
            "goal_position_tolerance": 0.45,
            "goal_yaw_tolerance": 3.14,
            "default_server_timeout": 15000,
            "bt_xml": str(package / "config" / "navigate.xml"),
        }

    with (config / "navigation.yaml").open(encoding="utf-8") as stream:
        custom = yaml.safe_load(stream)

    def node(exec_name, name=None, overrides=None, remappings=None, node_package="multimap_nav_servers"):
        """! @brief 设置服务器共享 YAML、节点级专属参数和话题重映射。"""
        name = name or exec_name
        parameters = [*profile_files, custom.get(name, {}), {"use_sim_time": True}]
        if overrides:
            parameters.append(overrides)
        return Node(package=node_package, executable=exec_name, name=name, output="screen",
                    parameters=parameters, remappings=remappings or [])

    nodes = [
        node("navigation_mode_selector_server",
             overrides={"pcd_file_path": str(pcd), "forced_mode": requested_mode}),
        node("map_server", overrides={"map_file_path": str(pcd)}),
        node("localization_server"),
        node("global_planner_server"),
        node("local_controller_server", overrides={
            f"{mode}_controller.onnx_model": str(Path(option("onnx_model")).expanduser())
            for mode in modes},
            remappings=[("cmd_vel", "/clearing/disabled_nav_cmd")] if training else []),
        # 整车路径检查不允许原地旋转恢复，隔离通用恢复输出。
        node("recovery_server", remappings=[("cmd_vel", "/clearing/disabled_recovery_cmd")]),
        node("bt_navigator", node_package="multimap_nav_bt"),
        node("lifecycle_manager", node_package="multimap_nav_lifecycle_manager",
             overrides={"profiles": profiles, **({"node_names": [
                 "navigation_mode_selector_server", "map_server", "localization_server",
                 "global_planner_server"]} if training else {})}),
    ]
    return nodes


def generate_launch_description():
    """! @brief 声明模式管理器传递的内部导航参数。"""
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="training", choices=["training", "evaluation", "training_rough_v2"]),
        DeclareLaunchArgument("pcd_file", default_value=""),
        DeclareLaunchArgument("map_mode", default_value="auto",
                              choices=["auto", "costmap2d", "gridmap", "octomap"]),
        DeclareLaunchArgument("controller", default_value="mppi"),
        DeclareLaunchArgument("onnx_model", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
