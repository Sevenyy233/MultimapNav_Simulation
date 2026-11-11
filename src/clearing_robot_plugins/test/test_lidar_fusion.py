#!/usr/bin/env python3
"""Check five-lidar extrinsics/self filtering without odometry or Gazebo."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mapping-only',action='store_true')
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='217';os.environ['ROS_LOCALHOST_ONLY']='1'
    output=Path(tempfile.mkdtemp(prefix='clearing_fusion_test_'))
    os.environ['ROS_LOG_DIR']=str(output/'ros_logs')
    root=Path(get_package_share_directory('clearing_robot_plugins'))
    geometry=yaml.safe_load((root/'config/robot_model.yaml').read_text())
    sensors={int(b['name'][-1]):b for b in geometry['boxes'] if b['name'].startswith('rslidar_')}
    process=None;node=None
    try:
        rclpy.init();node=rclpy.create_node('five_lidar_fusion_contract')
        broadcaster=StaticTransformBroadcaster(node);transforms=[]
        for name,xyz,quaternion in [('imu_link',[0,0,1.65],[0,0,0,1])]+[
                (f'rslidar_{i}',sensors[i]['center'],sensors[i]['quaternion']) for i in range(1,6)]:
            t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id=name
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,quaternion)
            transforms.append(t)
        broadcaster.sendTransform(transforms)
        publishers=[node.create_publisher(CustomMsg,f'/livox/lidar_{i}',qos_profile_sensor_data) for i in range(1,6)]
        received=[]
        subscription=node.create_subscription(CustomMsg,'/clearing/lidar_fused',received.append,qos_profile_sensor_data)
        with (output/'perception.log').open('w') as log:
            parameters=['-p',f'robot_model_file:={root / "config/robot_model.yaml"}']
            if args.mapping_only:
                parameters+=['-p','mapping_only:=true']
            else:
                parameters+=['-p',f'map_file:={root / "maps/clearing_enclosed.pcd"}',
                             '-p',f'support_file:={root / "maps/clearing_support.pcd"}']
            process=subprocess.Popen(['ros2','run','clearing_robot_plugins','clearing_perception','--ros-args',*parameters],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+15
            while not received and time.monotonic()<end:
                stamp=node.get_clock().now().to_msg()
                for i,publisher in enumerate(publishers,1):
                    model=sensors[i];rotation=Rotation.from_quat(model['quaternion']).as_matrix()
                    center=np.array(model['center']);m=CustomMsg();m.header.stamp=stamp;m.header.frame_id=f'rslidar_{i}'
                    for j in range(40):
                        for target in ([8+.01*j,0,.5],[0,0,.7]):
                            local=rotation.T@(np.array(target)-center)
                            point=CustomPoint();point.x,point.y,point.z=map(float,local)
                            m.points.append(point)
                    m.point_num=len(m.points);publisher.publish(m)
                rclpy.spin_once(node,timeout_sec=.1)
            assert received,f'No fusion output; {output}'
            m=received[-1];assert m.header.frame_id=='imu_link'
            assert m.point_num==200,len(m.points)
            for point in m.points:
                assert abs(point.y)<1e-5 and abs(point.z+1.15)<1e-5
                assert 7.999<point.x<8.4 and point.offset_time==0
            assert {line:sum(p.line==line for p in m.points) for line in range(5)}=={i:40 for i in range(5)}
            if args.mapping_only:
                assert node.count_publishers('/clearing/obstacles')==0
                print('PASS: mapping fusion starts without map/support files or navigation outputs.')
            print('PASS: all five extrinsics agree in IMU frame, self points removed, no odometry dependency.')
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print('Results:',output)


if __name__=='__main__':main()
