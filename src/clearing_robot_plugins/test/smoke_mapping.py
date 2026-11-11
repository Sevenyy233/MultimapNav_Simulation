#!/usr/bin/env python3
"""Isolated simulation: map from five lidars, move, then save a nonempty PCD."""
import argparse
import json
import math
import os
from pathlib import Path
import secrets
import re
import signal
import socket
import subprocess
import tempfile
import time

from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-name', default='clearing_enclosed')
    parser.add_argument('--spawn-x', default='0')
    parser.add_argument('--spawn-y', default='0')
    parser.add_argument('--visual', action='store_true')
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=f'{args.map_name}_', dir=args.output_root)).resolve()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    domain = secrets.randbelow(150)+40
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                      GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
                      ROS_LOG_DIR=str(output/'ros_logs'), GAZEBO_MODEL_DATABASE_URI='')
    pcd = output/'map.pcd'
    log = (output/'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'clearing_robot_plugins', 'mapping.launch.py',
        'headless:=true', f'use_rviz:={str(args.visual).lower()}', f'map_name:={args.map_name}',
        f'spawn_x:={args.spawn_x}', f'spawn_y:={args.spawn_y}',
        f'map_output_file:={pcd}'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('mapping_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    data, counts = {}, {}

    def receive(key):
        def callback(msg):
            data[key] = msg
            counts[key] = counts.get(key, 0)+1
        return callback

    subscriptions = [node.create_subscription(kind, topic, receive(key), qos_profile_sensor_data)
        for key, kind, topic in [('fused', CustomMsg, '/clearing/lidar_fused'),
                                 ('odom', Odometry, '/odom'),
                                 ('map', PointCloud2, '/Laser_map'),
                                 ('scan', PointCloud2, '/cloud_registered')]]
    command = node.create_publisher(Twist, '/cmd_vel', 10)
    saver = node.create_client(Trigger, '/map_save')

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert process.poll() is None, f'Launch exited; {output}'
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        deadline = time.monotonic()+180
        while time.monotonic() < deadline:
            spin(.1)
            if (counts.get('map', 0) >= 2 and 'odom' in data and data['map'].width > 1000
                    and buffer.can_transform('map', 'base_link', rclpy.time.Time())
                    and saver.service_is_ready()):
                break
        assert counts.get('map', 0) >= 2 and data['map'].width > 1000, counts
        names = node.get_node_names()
        assert not set(names).intersection({'map_server', 'localization_server', 'global_planner_server',
            'local_planner_server', 'trajectory_planner_server', 'bt_navigator', 'navigation_mode_selector_server'})
        assert node.count_publishers('/clearing/obstacles') == 0
        assert node.count_publishers('/odom') == 1
        transform = buffer.lookup_transform('map', 'odom', rclpy.time.Time()).transform
        assert abs(transform.translation.x)+abs(transform.translation.y)+abs(transform.translation.z) < 1e-8
        assert abs(transform.rotation.w-1) < 1e-8
        before = data['odom'].pose.pose.position
        before = (before.x, before.y)
        initial_points = data['map'].width
        started = node.get_clock().now().nanoseconds
        deadline = time.monotonic()+60
        forward = Twist()
        forward.linear.x = .18
        while (node.get_clock().now().nanoseconds-started)/1e9 < 8 and time.monotonic() < deadline:
            command.publish(forward)
            spin(.05)
        for _ in range(25):
            command.publish(Twist())
            spin(.1)
        after = data['odom'].pose.pose.position
        displacement = math.hypot(after.x-before[0], after.y-before[1])
        assert .6 < displacement < 2, displacement
        assert data['map'].width > initial_points
        future = saver.call_async(Trigger.Request())
        deadline = time.monotonic()+20
        while not future.done() and time.monotonic() < deadline:
            spin(.1)
        assert future.done() and future.result().success
        assert pcd.is_file() and pcd.stat().st_size > 10000
        header = pcd.read_bytes().split(b'DATA binary\n', 1)[0].decode()
        points = int(next(line.split()[1] for line in header.splitlines() if line.startswith('POINTS ')))
        assert points > 1000
        result = {'map_name': args.map_name, 'spawn_xy': [args.spawn_x, args.spawn_y],
                  'counts': counts, 'initial_map_points': initial_points, 'saved_points': points,
                  'odometry_displacement_m': displacement, 'pcd': str(pcd),
                  'pcd_bytes': pcd.stat().st_size, 'ros_domain': domain, 'gazebo_port': port,
                  'navigation_servers_absent': True, 'prior_map_not_used': True}
        if args.visual:
            spin(3)
            windows=subprocess.check_output(['xprop','-root','_NET_CLIENT_LIST'],text=True)
            own=[]
            for window in re.findall(r'0x[0-9a-fA-F]+',windows):
                owner=subprocess.run(['xprop','-id',window,'_NET_WM_PID'],text=True,capture_output=True).stdout
                match=re.search(r'= (\d+)',owner)
                if not match:continue
                try:
                    pid=int(match[1])
                    if os.getpgid(pid)!=process.pid or not Path(f'/proc/{pid}/comm').read_text().startswith('rviz'):continue
                except (FileNotFoundError,ProcessLookupError):continue
                info=subprocess.check_output(['xwininfo','-id',window],text=True)
                width=int(re.search(r'Width: (\d+)',info)[1]);height=int(re.search(r'Height: (\d+)',info)[1])
                if width>600 and height>400:own.append((width*height,window))
            assert own, 'Own mapping RViz main window not found'
            window=max(own)[1]
            subprocess.run(['xwd','-silent','-id',window,'-out',str(output/'rviz.xwd')],check=True)
            subprocess.run(['convert',str(output/'rviz.xwd'),str(output/'rviz.png')],check=True)
            (output/'rviz.xwd').unlink()
            result['rviz_capture']=str(output/'rviz.png')
        (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print('PASS', json.dumps(result), flush=True)
    finally:
        command.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()
        print('Results:', output, flush=True)


if __name__ == '__main__':
    main()
