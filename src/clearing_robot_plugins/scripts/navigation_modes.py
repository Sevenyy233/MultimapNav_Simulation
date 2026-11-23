#!/usr/bin/env python3
"""! @file 重新装载导航模式，保持仿真、里程计和 RViz 进程常驻。"""

import os
import signal
import subprocess
import threading
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist, TwistStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from multimap_nav_msgs.srv import GetSelectedNavigationMode
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


class NavigationModes(Node):
    """! @brief 管理导航进程组及其切换状态，订阅停止和定位反馈。"""

    MODES = ("auto", "costmap2d", "gridmap", "octomap")

    def __init__(self):
        """! @brief 建立服务客户端、状态话题和初次启动任务。"""
        super().__init__("clearing_navigation_modes")
        self._pcd = self.declare_parameter("pcd_file", "").value
        self._controller = self.declare_parameter("controller", "mppi").value
        self._model = self.declare_parameter("onnx_model", "").value
        requested = self.declare_parameter("map_mode", "auto").value
        if requested not in self.MODES:
            raise ValueError("无效的初始地图模式")

        self._stack = None
        self._healthy = (False, 0.0)
        self._drive = None
        self._ready_request = None
        self._stop = threading.Event()
        self._worker = None
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status = self.create_publisher(String, "/clearing/active_map_mode", retained)
        self._velocity = self.create_publisher(Twist, "/cmd_vel", 1)
        self._health_sub = self.create_subscription(Bool, "/localization/healthy", self._on_health, 1)
        self._drive_sub = self.create_subscription(
            TwistStamped, "/clearing/cmd_vel_applied", self._on_drive, 1)
        node = "global_planner_server" if self._controller == "training" else "bt_navigator"
        self._ready_client = self.create_client(GetState, f"/{node}/get_state")
        self._selection = self.create_client(
            GetSelectedNavigationMode, "/navigation_mode_selector_server/get_selected_navigation_mode")
        self._cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self._callback_handle = self.add_on_set_parameters_callback(self._on_parameters)
        self._monitor = self.create_timer(0.25, self._check_child)
        self._begin(requested)

    def _on_health(self, message):
        """! @brief 记录最新定位健康反馈与墙钟时间。"""
        self._healthy = (message.data, time.monotonic())

    def _on_drive(self, message):
        """! @brief 保存底盘执行速度反馈。"""
        self._drive = (message.twist, time.monotonic())

    def _on_parameters(self, changes):
        """! @brief 接受合法、非并发的 map_mode 更新并启动切换。"""
        for param in changes:
            if param.name != "map_mode":
                continue
            if param.type_ != Parameter.Type.STRING or param.value not in self.MODES:
                return SetParametersResult(successful=False, reason="选择 auto、costmap2d、gridmap 或 octomap")
            if self._controller == "training":
                return SetParametersResult(successful=False, reason="训练期间请使用启动时的地图模式")
            if self._worker and self._worker.is_alive():
                return SetParametersResult(successful=False, reason="正在装载导航，请等待当前切换完成")
            if param.value != self._ready_request:
                self._begin(param.value)
        return SetParametersResult(successful=True)

    def _begin(self, mode):
        """! @brief 后台切换导航，避免阻塞 ROS 回调。"""
        self._ready_request = None
        self._status.publish(String(data=""))
        self._worker = threading.Thread(target=self._replace_stack, args=(mode,), daemon=True)
        self._worker.start()

    def _check_child(self):
        """! @brief 子进程意外结束时撤销对外就绪状态。"""
        if self._ready_request and self._stack and self._stack.poll() is not None:
            self._ready_request = None
            self._status.publish(String(data=""))
            self.get_logger().error("导航进程已退出，可以重新设置 map_mode")

    def _call(self, client, request, timeout):
        """! @brief 在工作线程限时等待服务，回调由主线程继续处理。"""
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if done.wait(timeout) and not future.cancelled():
            return future.result()
        future.cancel()
        return None

    def _terminate(self):
        """! @brief 关闭本节点创建的独立导航进程组。"""
        stack = self._stack
        if stack is None:
            return
        if stack.poll() is None:
            stack.send_signal(signal.SIGINT)
            try:
                stack.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(stack.pid, signal.SIGTERM)
                try:
                    stack.wait(timeout=4.0)
                except subprocess.TimeoutExpired:
                    os.killpg(stack.pid, signal.SIGKILL)
                    stack.wait(timeout=2.0)
        # roslaunch 退出后也回收同组内尚未返回的子进程。
        try:
            os.killpg(stack.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        self._stack = None

    def _wait_stopped(self):
        """! @brief 发送零速并等待新鲜的底盘停车反馈。"""
        deadline = time.monotonic() + 6.0
        beginning = time.monotonic()
        stopped_since = None
        while not self._stop.is_set() and time.monotonic() < deadline:
            self._velocity.publish(Twist())
            if self._drive is not None:
                velocity, stamp = self._drive
                fresh = stamp > beginning and time.monotonic() - stamp < .35
                stationary = abs(velocity.linear.x) < .015 and abs(velocity.angular.z) < .015
                if fresh and stationary:
                    if stopped_since is None:
                        stopped_since = stamp
                    elif stamp - stopped_since >= .25:
                        return True
                else:
                    stopped_since = None
            self._stop.wait(.1)
        return False

    def _replace_stack(self, mode):
        """! @brief 取消旧目标、等待停车，重新执行 MultiMapNav 的模式选择与就绪检查。"""
        try:
            if self._stack is not None:
                self._call(self._cancel, CancelGoal.Request(), 1.5)
                self._terminate()
                if not self._wait_stopped():
                    raise RuntimeError("底盘未确认停止，未启用新规划器")
            if self._stop.is_set():
                return

            args = ["ros2", "launch", "clearing_robot_plugins", "navigation_stack.launch.py",
                    f"pcd_file:={self._pcd}", f"map_mode:={mode}",
                    f"controller:={self._controller}"]
            if self._model:
                args.append(f"onnx_model:={self._model}")
            starting_time = time.monotonic()
            self._stack = subprocess.Popen(args, start_new_session=True)
            deadline = starting_time + 75.0
            while not self._stop.is_set() and time.monotonic() < deadline:
                if self._stack.poll() is not None:
                    raise RuntimeError("导航子进程提前退出")
                valid_pose, stamp = self._healthy
                if valid_pose and stamp > starting_time and time.monotonic() - stamp < 1.0:
                    ready = self._call(self._ready_client, GetState.Request(), .7)
                    if ready and ready.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        selected = self._call(self._selection, GetSelectedNavigationMode.Request(), .7)
                        if selected and selected.success and selected.ready:
                            self._ready_request = mode
                            self._status.publish(String(data=selected.selected_mode))
                            self.get_logger().info(f"导航模式已就绪: {mode} → {selected.selected_mode}")
                            return
                self._stop.wait(.25)
            if not self._stop.is_set():
                raise RuntimeError("等待地图、导航和定位就绪超时")
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            if not self._stop.is_set():
                self.get_logger().error(f"导航模式 {mode} 切换失败: {error}")
                self._terminate()

    def close(self):
        """! @brief 停止切换任务并回收导航子进程。"""
        self._stop.set()
        if self._worker:
            self._worker.join(timeout=15.0)
        self._terminate()


def main():
    """! @brief 运行清舱机器人导航模式管理节点。"""
    rclpy.init()
    node = NavigationModes()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
