#!/usr/bin/env python3
"""! @file 启动本次仿真的 FAST-LIO，训练复位时重建时序状态。"""

import os
from collections import OrderedDict
from pathlib import Path
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from clearing_robot_plugins.srv import ResetEstimator
from multimap_nav_msgs.srv import SetInitialPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg
from std_msgs.msg import Empty


class EstimatorSupervisor(Node):
    """! @brief 管理本节点启动的 FAST-LIO 子进程和复位命令。"""

    def __init__(self):
        """! @brief 创建 FAST-LIO 工作进程并连接有时间戳的反馈。"""
        super().__init__("clearing_estimator_supervisor")
        config = self.declare_parameter("fast_lio_config", "").value
        overrides = self.declare_parameter("fusion_config", "").value
        if not Path(config).is_file() or not Path(overrides).is_file():
            raise ValueError("FAST-LIO 参数文件不存在")
        fast_lio = Path(get_package_prefix("fast_lio")) / "lib/fast_lio/fastlio_mapping"
        self._command = [str(fast_lio), "--ros-args", "--params-file", config,
                         "--params-file", overrides, "-r", "__node:=fastlio_mapping", "-r", "/Odometry:=/fast_lio/odometry",
                         "-r", "/tf:=/fast_lio/tf"]
        self._child_env = os.environ.copy()
        # FAST-LIO 经常与五个 Gazebo 雷达并发运行，避免 BLAS/OMP 过量建线程。
        self._child_env.setdefault("OMP_NUM_THREADS", "2")
        self._mutex = threading.Lock()
        self._fresh = threading.Condition()
        # 本轮重启后收集原始 FAST-LIO 与桥接输出的同一测量时间。
        self._epoch_stamp = -1
        self._epoch_active = False
        self._raw_stamps = OrderedDict()
        self._bridge_stamps = OrderedDict()
        self._sensor_status = {}
        self._process = None
        group = ReentrantCallbackGroup()
        self._odom = self.create_subscription(Odometry, "/odom", self._on_odom,
                                              qos_profile_sensor_data, callback_group=group)
        self._raw_odom = self.create_subscription(Odometry, "/fast_lio/odometry", self._on_raw_odom,
                                                  qos_profile_sensor_data, callback_group=group)
        self._imu = self.create_subscription(Imu, "/livox/imu_raw", self._on_imu,
                                              qos_profile_sensor_data, callback_group=group)
        self._lidar = self.create_subscription(CustomMsg, "/fast_lio/lidar", self._on_lidar,
                                                qos_profile_sensor_data, callback_group=group)
        self._reset_notice = self.create_publisher(Empty, "/clearing/estimator_reset", 1)
        self._localization = self.create_client(SetInitialPose, "/set_initial_pose",
                                                callback_group=group)
        self._reset_service = self.create_service(ResetEstimator, "/clearing/reset_estimator",
                                                 self._reset, callback_group=group)
        self._start()

    @staticmethod
    def _stamp_ns(message):
        """! @brief 转换消息头时间戳为纳秒，不更改测量时间。"""
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _observe_odom(self, message, raw):
        """! @brief 记录当前重启周期中，帧名正确且时间有效的原始和桥接里程计。"""
        expected = ("camera_init", "body") if raw else ("odom", "base_link")
        stamp = self._stamp_ns(message)
        received = time.monotonic()
        with self._fresh:
            key = "raw_odom" if raw else "bridge_odom"
            self._sensor_status[key] = (stamp, received, message.header.frame_id, message.child_frame_id)
            if self._epoch_active and stamp > self._epoch_stamp and (
                    message.header.frame_id, message.child_frame_id) == expected:
                records = self._raw_stamps if raw else self._bridge_stamps
                records[stamp] = received
                while len(records) > 256:
                    records.popitem(last=False)
            self._fresh.notify_all()

    def _on_raw_odom(self, message):
        """! @brief 记录 FAST-LIO 原始输出。"""
        self._observe_odom(message, True)

    def _on_odom(self, message):
        """! @brief 记录桥接到机器人底盘的里程计。"""
        self._observe_odom(message, False)

    def _on_imu(self, message):
        """! @brief 保存 IMU 诊断时间与接收时间。"""
        with self._fresh:
            self._sensor_status["imu"] = (self._stamp_ns(message), time.monotonic(), message.header.frame_id, "")

    def _on_lidar(self, message):
        """! @brief 保存融合雷达诊断时间与接收时间。"""
        with self._fresh:
            self._sensor_status["lidar"] = (self._stamp_ns(message), time.monotonic(), message.header.frame_id, "")

    def _diagnostics(self):
        """! @brief 汇总各输入的仿真时间、接收墙钟年龄与子进程状态。"""
        now_ns = self.get_clock().now().nanoseconds
        now_wall = time.monotonic()
        items = [f"sim={now_ns * 1e-9:.3f}s", f"child_rc={self._process.poll() if self._process else 'none'}"]
        with self._fresh:
            for name in ("lidar", "imu", "raw_odom", "bridge_odom"):
                data = self._sensor_status.get(name)
                if data is None:
                    items.append(f"{name}=missing")
                else:
                    stamp, received, frame, child = data
                    items.append(f"{name}:stamp={stamp * 1e-9:.3f},sim_age={(now_ns-stamp)*1e-9:.3f},wall_age={now_wall-received:.3f},frame={frame}/{child}")
        return "; ".join(items)

    def _start(self):
        """! @brief 启动使用五路融合点云、IMU 和仿真时钟的 FAST-LIO。"""
        self._process = subprocess.Popen(self._command, env=self._child_env)
        self.get_logger().info(f"启动 FAST-LIO，PID={self._process.pid}")

    def _stop(self):
        """! @brief 关闭本节点创建的进程，最多等待六秒。"""
        process = self._process
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def _reset(self, request, response):
        """! @brief 在已停车并移位后重启 LIO，随后请求用新位姿初值验证匹配。"""
        if request.pose.header.frame_id != "map":
            response.reason = "复位位姿需要 map 坐标系"
            return response
        if not self._mutex.acquire(blocking=False):
            response.reason = "估计器正在复位"
            return response
        try:
            self._reset_notice.publish(Empty())
            self._stop()
            # 未收到 /clock 的零值不能作为此次重启的时间基准。
            clock_deadline = time.monotonic() + 5.0
            stamp = self.get_clock().now().nanoseconds
            while stamp <= 0 and time.monotonic() < clock_deadline:
                time.sleep(.05)
                stamp = self.get_clock().now().nanoseconds
            if stamp <= 0:
                response.reason = "仿真时钟尚未就绪，无法建立估计器复位时间基准"
                self.get_logger().error(response.reason)
                return response
            with self._fresh:
                self._epoch_active = False
                self._epoch_stamp = stamp
                self._raw_stamps.clear()
                self._bridge_stamps.clear()
            self._start()
            with self._fresh:
                self._epoch_active = True
            deadline = time.monotonic() + 30.0
            matched = None
            # 即使没有任何消息到达也定期检查子进程退出。
            while time.monotonic() < deadline:
                if self._process.poll() is not None:
                    response.reason = f"FAST-LIO 子进程提前退出 (rc={self._process.returncode}); {self._diagnostics()}"
                    self.get_logger().error(response.reason)
                    return response
                now_ns = self.get_clock().now().nanoseconds
                with self._fresh:
                    common = self._raw_stamps.keys() & self._bridge_stamps.keys()
                    # 不接受未来时间或积压过旧的输出；匹配必须属于当前进程重启周期。
                    valid = [sample for sample in common if stamp < sample <= now_ns + 100_000_000
                             and now_ns - sample < 2_000_000_000]
                    if valid:
                        matched = max(valid)
                        break
                    self._fresh.wait(timeout=min(.2, max(0.0, deadline-time.monotonic())))
            if matched is None:
                response.reason = "FAST-LIO 重启后 30 秒内未收到匹配的新原始及桥接里程计; " + self._diagnostics()
                self.get_logger().error(response.reason)
                return response
            if self._process.poll() is not None:
                response.reason = f"FAST-LIO 取得里程计后退出 (rc={self._process.returncode}); {self._diagnostics()}"
                self.get_logger().error(response.reason)
                return response
            self.get_logger().info(f"FAST-LIO 重启数据链已确认: stamp={matched*1e-9:.3f}; {self._diagnostics()}")
            if not self._localization.wait_for_service(timeout_sec=3.0):
                response.reason = "MultiMapNav 初始位姿服务不可用"
                return response
            initial = SetInitialPose.Request()
            initial.pose.header = request.pose.header
            initial.pose.header.stamp = self.get_clock().now().to_msg()
            initial.pose.pose.pose = request.pose.pose
            for axis in (0, 7, 14, 21, 28, 35):
                initial.pose.pose.covariance[axis] = 0.25
            completion = threading.Event()
            future = self._localization.call_async(initial)
            future.add_done_callback(lambda _: completion.set())
            if (not completion.wait(timeout=10.0) or future.exception() is not None or
                    future.result() is None):
                response.reason = "MultiMapNav 初始位姿服务调用失败"
                return response
            response.success = True
            response.reason = "新里程计已到达并提交匹配初值，等待定位健康后开始控制"
            return response
        except (OSError, subprocess.SubprocessError) as error:
            response.reason = str(error)
            return response
        finally:
            with self._fresh:
                self._epoch_active = False
            self._mutex.release()


def main():
    """! @brief 运行独立回调线程，在服务等待时继续接收里程计。"""
    rclpy.init()
    node = EstimatorSupervisor()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
