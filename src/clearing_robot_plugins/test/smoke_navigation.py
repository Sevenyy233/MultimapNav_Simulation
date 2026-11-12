#!/usr/bin/env python3
"""Isolated end-to-end clearing-robot navigation checks; no truth odometry publisher."""
import argparse
import atexit
import json
import math
import os
from pathlib import Path
import re
import signal
import secrets
import socket
import subprocess
import tempfile
import time

from action_msgs.msg import GoalStatus
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from multimap_nav_msgs.msg import Trajectory
from multimap_nav_msgs.action import NavigateToPose, RunRecovery
from multimap_nav_msgs.srv import GetSelectedNavigationMode
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def boxes_overlap(a_center, a_rotation, a_half, b_center, b_half):
    r=a_rotation.T; t=r@(b_center-a_center); ar=abs(r)+1e-9
    if any(abs(t[i])>a_half[i]+np.dot(b_half,ar[i]) for i in range(3)):return False
    if any(abs(np.dot(t,r[:,j]))>b_half[j]+np.dot(a_half,ar[:,j]) for j in range(3)):return False
    for i in range(3):
        for j in range(3):
            ra=a_half[(i+1)%3]*ar[(i+2)%3,j]+a_half[(i+2)%3]*ar[(i+1)%3,j]
            rb=b_half[(j+1)%3]*ar[i,(j+2)%3]+b_half[(j+2)%3]*ar[i,(j+1)%3]
            if abs(t[(i+2)%3]*r[(i+1)%3,j]-t[(i+1)%3]*r[(i+2)%3,j])>ra+rb:return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['startup', 'visual', 'navigate', 'turn', 'obstacle', 'sensor_loss', 'relocalize', 'recovery', 'chassis'], default='navigate')
    parser.add_argument('--domain', type=int, default=None)
    parser.add_argument('--gazebo-port', type=int, default=None)
    parser.add_argument('--forced-mode', choices=['auto','costmap2d','gridmap','octomap'], default='auto')
    parser.add_argument('--output-root', type=Path, default=None, help='Keep test artifacts in this directory')
    args = parser.parse_args()
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(('127.0.0.1', args.gazebo_port or 0))
        args.gazebo_port=probe.getsockname()[1]
    if args.domain is None:args.domain=secrets.randbelow(150)+40
    if args.output_root is not None:
        args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='clearing_navigation_', dir=args.output_root))
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GAZEBO_MASTER_URI'] = f'http://127.0.0.1:{args.gazebo_port}'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(output/'ros_logs')
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    log = (output/'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'clearing_robot_plugins', 'navigation.launch.py',
                                'headless:=true', f'use_rviz:={str(args.phase=="visual").lower()}',
                                f'forced_mode:={args.forced_mode}'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def stop_launch():
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    atexit.register(stop_launch)
    rclpy.init()
    node = rclpy.create_node('clearing_navigation_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer = Buffer(); listener = TransformListener(buffer, node)
    data, counts, commands, backends, guard_reasons = {}, {}, [], {}, {}

    def receive(key):
        def callback(msg):
            data[key] = msg; counts[key] = counts.get(key, 0)+1
            if key == 'guard':
                reason=json.loads(msg.data)['reason']
                guard_reasons[reason]=guard_reasons.get(reason,0)+1
            if key == 'command':
                commands.append([time.monotonic(), msg.linear.x, msg.angular.z])
            if key in ('shaped','applied'):
                values=[msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,msg.twist.linear.x,msg.twist.angular.z]
                (shaped if key=='shaped' else applied).append(values)
            if key=='trajectory':
                signs=[1 if p.velocity.angular.z>0 else -1 for p in msg.points if abs(p.velocity.angular.z)>.025]
                steering_reversals.append(sum(a!=b for a,b in zip(signs,signs[1:])))
            if key == 'backend':
                status=json.loads(msg.data);backends[status['node']]=status
        return callback

    subscriptions = [node.create_subscription(kind, topic, receive(key), qos_profile_sensor_data)
        for key, kind, topic in [('clock', Clock, '/clock'), ('healthy', Bool, '/clearing/localization_healthy'),
                                ('sensors', Bool, '/clearing/sensors_healthy'),
                                ('status', String, '/clearing/localization_status'),
                                ('guard', String, '/clearing/safety_status'),
                                ('odom', Odometry, '/odom'), ('localized', Odometry, '/localization/odometry'),
                                ('shaped', TwistStamped, '/clearing/cmd_vel_smooth'),
                                ('applied', TwistStamped, '/clearing/cmd_vel_applied'),
                                ('trajectory', Trajectory, '/local_trajectory'),
                                ('command', Twist, '/cmd_vel'), ('global', NavPath, '/global_path'),
                                ('obstacles', PointCloud2, '/clearing/obstacles'),
                                ('local', NavPath, '/local_path'), ('markers', MarkerArray, '/clearing/collision_model')]]
    subscriptions.append(node.create_subscription(String,'/clearing/map_backend',receive('backend'),
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    recovery = ActionClient(node, RunRecovery, '/run_recovery')
    physics = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    selector = node.create_client(GetSelectedNavigationMode,
        '/navigation_mode_selector_server/get_selected_navigation_mode')
    geometry=yaml.safe_load((Path(get_package_share_directory('clearing_robot_plugins'))/'config/robot_model.yaml').read_text())['boxes']
    paused_pid=None;obstacle_active=False;obstacle_checks=0
    body_samples=[]
    shaped=[];applied=[];steering_reversals=[]

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited {process.returncode}; {output}')
            rclpy.spin_once(node, timeout_sec=.02)

    def wait(future, timeout):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            spin(.05)
        assert future.done(), f'ROS request timed out; {output}'
        return future.result()

    def actual_state():
        request = GetEntityState.Request()
        request.name = 'clearing_robot_primitives::base_link'; request.reference_frame = 'world'
        response = wait(physics.call_async(request), 10)
        assert response.success
        return response.state

    def actual():
        return actual_state().pose

    def map_pose():
        return buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform

    def check_body(measured):
        nonlocal obstacle_checks
        q=measured.orientation
        r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        t=np.array([measured.position.x,measured.position.y,measured.position.z])
        body_samples.append({'time':time.monotonic(),'position':t.tolist(),'quaternion':[q.x,q.y,q.z,q.w],
                             'map_position':[map_pose().translation.x,map_pose().translation.y],
                             'obstacle_points':data['obstacles'].width if 'obstacles' in data else 0})
        for box in geometry:
            center=r@np.array(box['center'])+t;rotation=r@Rotation.from_quat(box['quaternion']).as_matrix()
            assert not boxes_overlap(center,rotation,np.array(box['size'])/2,
                                     np.array([5.5,5.5,.75]),np.array([.5,.5,.75])),(box['name'],body_samples[-1])
        obstacle_checks+=1

    try:
        deadline = time.monotonic()+180
        next_status = time.monotonic()+15
        while time.monotonic() < deadline:
            spin(.1)
            if time.monotonic() > next_status:
                print('WAIT', json.dumps({'counts': counts, 'backends': backends,
                    'status': data.get('status').data if 'status' in data else None}), flush=True)
                next_status += 15
            if ('healthy' in data and data['healthy'].data and 'sensors' in data and data['sensors'].data
                    and 'localized' in data and buffer.can_transform('map', 'base_link', rclpy.time.Time())
                    and navigator.server_is_ready() and 'markers' in data
                    and counts.get('obstacles',0)>=3
                    and all(backends.get(name,{}).get('ready') for name in (
                        'global_planner_server','local_planner_server','trajectory_planner_server','recovery_server'))):
                break
        assert 'healthy' in data and data['healthy'].data, f'Localization unavailable; {output}'
        assert navigator.server_is_ready(), f'Navigator inactive; {output}'
        assert all(backends.get(name,{}).get('ready') for name in (
            'global_planner_server','local_planner_server','trajectory_planner_server','recovery_server')),backends
        assert len(data['markers'].markers) == len(geometry)
        assert physics.wait_for_service(timeout_sec=10)
        truth = actual(); estimated = map_pose()
        error = math.hypot(truth.position.x-estimated.translation.x, truth.position.y-estimated.translation.y)
        assert error < .35, error
        topics = [name for name, kinds in node.get_topic_names_and_types() if 'nav_msgs/msg/Odometry' in kinds]
        assert '/ground_truth/odom' not in topics
        assert node.count_publishers('/odom') == 1
        result = {'initial_position_error_m': error, 'markers': len(geometry), 'odometry_topics': topics,
                  'ros_domain':args.domain,'gazebo_port':args.gazebo_port}
        assert selector.wait_for_service(timeout_sec=10)
        selection=wait(selector.call_async(GetSelectedNavigationMode.Request()),15)
        expected='octomap' if args.forced_mode=='auto' else args.forced_mode
        assert selection.success and selection.ready and selection.selected_mode==expected,selection
        result['forced_mode']=args.forced_mode
        result['selected_mode']=selection.selected_mode
        result['selection_reason']=selection.reason
        result['selection_features']={'flat':selection.flat_ratio,'slope':selection.slope_ratio,
                                      'multilayer':selection.multilayer_ratio,'rough':selection.rough_ratio}
        assert all(status['mode']==expected and status['source']=='topic' and status['ready']
                   for status in backends.values()),backends
        result['map_backends']=backends.copy()
        result['lifecycle_states']={}
        for name in ('navigation_mode_selector_server','map_server','localization_server','global_planner_server',
                     'local_planner_server','trajectory_planner_server','recovery_server','bt_navigator'):
            client=node.create_client(GetState,f'/{name}/get_state')
            assert client.wait_for_service(timeout_sec=10)
            state=wait(client.call_async(GetState.Request()),10).current_state
            assert state.id==3,(name,state)
            result['lifecycle_states'][name]=state.label
            node.destroy_client(client)
        result['loaded_plugins']={}
        for name,key in (('map_server','map_plugin_type'),('localization_server','localization_plugin_type'),
                         ('global_planner_server','planner_plugin_type'),('local_planner_server','local_planner_plugin_type'),
                         ('trajectory_planner_server','trajectory_planner_plugin_type')):
            client=node.create_client(GetParameters,f'/{name}/get_parameters')
            assert client.wait_for_service(timeout_sec=10)
            request=GetParameters.Request();request.names=[key]
            loaded=wait(client.call_async(request),10).values[0].string_value
            result['loaded_plugins'][name]=loaded
            if name=='map_server':
                assert loaded=={'costmap2d':'clearing_robot_plugins::Costmap2DAdapter',
                    'gridmap':'clearing_robot_plugins::GridMapAdapter','octomap':'multimap_nav_plugins::OctoMapPlugin'}[expected]
            else:assert loaded.startswith('clearing_robot_plugins::'),loaded
            node.destroy_client(client)
        result['navigation_action_endpoints']={}
        services=node.get_service_names_and_types_by_node('bt_navigator','/')
        endpoints=[s for s,_ in services if s.endswith('/_action/send_goal')]
        assert endpoints==['/navigate_to_pose/_action/send_goal'],endpoints
        result['navigation_action_endpoints']['bt_navigator']=endpoints
        assert node.count_publishers('/cmd_vel')==1,'cmd_vel must have exactly one governor publisher'
        print('READY', json.dumps(result), flush=True)

        if args.phase == 'visual':
            spin(10)
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
            assert own,'Own RViz main window missing'
            window=max(own)[1]
            subprocess.run(['xwd','-silent','-id',window,'-out',str(output/'rviz.xwd')],check=True)
            subprocess.run(['convert',str(output/'rviz.xwd'),str(output/'rviz.png')],check=True)
            (output/'rviz.xwd').unlink()
            result['rviz_capture']=str(output/'rviz.png')

        if args.phase == 'relocalize':
            initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
            deadline=time.monotonic()+10
            while initial.get_subscription_count()==0 and time.monotonic()<deadline:spin(.1)
            assert initial.get_subscription_count()>0
            epoch=json.loads(data['status'].data)['initialization_epoch']
            seed=PoseWithCovarianceStamped();seed.header.frame_id='map';seed.pose.pose.position.x=.5;seed.pose.pose.position.y=.3
            seed.pose.pose.position.z=.01;seed.pose.pose.orientation.z=math.sin(.05/2);seed.pose.pose.orientation.w=math.cos(.05/2)
            initial.publish(seed);spin(3)
            deadline=time.monotonic()+40
            error=math.inf
            while time.monotonic()<deadline:
                spin(.1)
                if data['healthy'].data and json.loads(data['status'].data)['initialization_epoch']>epoch:
                    measured=actual();p=map_pose();error=math.hypot(measured.position.x-p.translation.x,measured.position.y-p.translation.y)
                    if error<.2:break
            assert error<.2,error
            result['reinitialized_position_error_m']=error
            result['reinitialization_epoch']=json.loads(data['status'].data)['initialization_epoch']
        if args.phase in ('navigate','turn','obstacle','sensor_loss','chassis'):
            targets=[(6.,0.,0.)]
            if args.phase=='chassis':targets=[(2.,0.,0.),(-2.,0.,0.),(2.,2.,.35)]
            if args.phase=='turn':targets.append((6.,6.,math.pi/2))
            if args.phase=='obstacle':targets=[(0.,0.,.55),(10.,6.,0.)]
            completed=[]
            for number,target in enumerate(targets):
                if args.phase=='obstacle' and number==1:
                    assert spawn.wait_for_service(timeout_sec=10)
                    request=SpawnEntity.Request();request.name='navigation_test_obstacle'
                    request.xml='''<sdf version="1.6"><model name="navigation_test_obstacle"><static>true</static><link name="box"><collision name="box"><geometry><box><size>1 1 1.5</size></box></geometry></collision><visual name="box"><geometry><box><size>1 1 1.5</size></box></geometry></visual></link></model></sdf>'''
                    request.initial_pose.position.x=5.5;request.initial_pose.position.y=5.5;request.initial_pose.position.z=.75;request.initial_pose.orientation.w=1.
                    assert wait(spawn.call_async(request),15).success
                    obstacle_active=True;spin(2)
                    check_body(actual())
                    assert 'obstacles' in data
                    cloud=data['obstacles'];fields={f.name:f.offset for f in cloud.fields}
                    points=np.frombuffer(cloud.data,dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                        'offsets':[fields[k] for k in ['x','y','z']],'itemsize':cloud.point_step}))
                    hits=(abs(points['x']-5.5)<.6)&(abs(points['y']-5.5)<.6)&(points['z']>.2)&(points['z']<1.6)
                    assert hits.sum()>3,int(hits.sum())
                    result['dynamic_obstacle_returns']=int(hits.sum())
                command_begin=len(commands)
                goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = node.get_clock().now().to_msg()
                goal.pose.pose.position.x, goal.pose.pose.position.y, yaw = target
                goal.pose.pose.position.z = .01
                goal.pose.pose.orientation.z = math.sin(yaw/2); goal.pose.pose.orientation.w = math.cos(yaw/2)
                handle = wait(navigator.send_goal_async(goal), 15)
                assert handle.accepted, 'Navigation rejected'
                end = handle.get_result_async(); deadline = time.monotonic()+240; last = time.monotonic();checked=time.monotonic()
                while not end.done() and time.monotonic() < deadline:
                    spin(.1)
                    if obstacle_active and time.monotonic()-checked>.3:
                        check_body(actual());checked=time.monotonic()
                    if args.phase=='sensor_loss' and not result.get('sensor_loss_stop_checked') and len(commands)>20 and abs(data['odom'].twist.twist.linear.x)>.15:
                        for proc in Path('/proc').iterdir():
                            if not proc.name.isdigit():continue
                            try:
                                if proc.joinpath('comm').read_text().strip().startswith('clearing_percep') and os.getpgid(int(proc.name))==process.pid:
                                    paused_pid=int(proc.name);break
                            except (FileNotFoundError,ProcessLookupError,PermissionError):pass
                        assert paused_pid
                        os.kill(paused_pid,signal.SIGSTOP)
                        stopped_at=node.get_clock().now().nanoseconds
                        until=time.monotonic()+20
                        while (node.get_clock().now().nanoseconds-stopped_at)/1e9<5 and time.monotonic()<until:
                            spin(.1)
                        assert abs(data['command'].linear.x)<.03 and abs(data['command'].angular.z)<.03
                        state=actual_state()
                        stopped_speed=math.hypot(state.twist.linear.x,state.twist.linear.y)
                        assert stopped_speed<.05 and abs(state.twist.angular.z)<.06, state.twist
                        result['sensor_loss_actual_speed_mps']=stopped_speed
                        os.kill(paused_pid,signal.SIGCONT);paused_pid=None;result['sensor_loss_stop_checked']=True
                    if time.monotonic()-last > 15:
                        measured=actual()
                        print('DRIVE', {'physical_xy':[measured.position.x, measured.position.y],
                            'odom_xy':[data['odom'].pose.pose.position.x,data['odom'].pose.pose.position.y],
                            'odom_vw':[data['odom'].twist.twist.linear.x,data['odom'].twist.twist.angular.z],
                            'status':data.get('status').data if 'status' in data else None,
                            'guard':data['guard'].data if 'guard' in data else None},flush=True)
                        last=time.monotonic()
                assert end.done(), f'Navigation did not finish; guard reasons={guard_reasons}; {output}'
                assert end.result().status == GoalStatus.STATUS_SUCCEEDED, end.result()
                measured = actual(); location = map_pose()
                result['final_position'] = [measured.position.x, measured.position.y]
                result['goal_position_error_m'] = math.hypot(measured.position.x-target[0], measured.position.y-target[1])
                result['localization_error_m'] = math.hypot(location.translation.x-measured.position.x,location.translation.y-measured.position.y)
                assert result['goal_position_error_m'] < .5
                assert result['localization_error_m'] < .35
                if math.hypot(target[0], target[1]) > .5:
                    assert any(abs(c[1]) > .05 for c in commands)
                else:
                    assert any(abs(c[2]) > .02 for c in commands)
                q=measured.orientation
                measured_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                expected_yaw=target[2]
                result['goal_yaw_error_rad']=abs(math.atan2(math.sin(measured_yaw-expected_yaw),math.cos(measured_yaw-expected_yaw)))
                if args.phase=='chassis' and number==1:
                    assert min(c[1] for c in commands[command_begin:])<-.08,commands[command_begin:]
                    assert abs(measured_yaw)<.2,measured_yaw
                    result['reverse_target_preserved_heading']=True
                assert result['goal_yaw_error_rad']<.22,result['goal_yaw_error_rad']
                assert counts.get('global',0)>0 and counts.get('local',0)>0
                completed.append({'target':target,'physical_position':result['final_position'],
                                  'goal_error':result['goal_position_error_m'],'localization_error':result['localization_error_m']})
            result['completed_goals']=completed
            if obstacle_active:result['collision_free_body_checks']=obstacle_checks
        if args.phase == 'recovery':
            assert recovery.wait_for_server(timeout_sec=10)
            for name in ('back_up', 'spin'):
                before=actual()
                goal=RunRecovery.Goal(); goal.behavior_name=name
                handle=wait(recovery.send_goal_async(goal),10); assert handle.accepted
                response=wait(handle.get_result_async(),60)
                assert response.status==GoalStatus.STATUS_SUCCEEDED, response
                after=actual()
                if name=='back_up':
                    result['backup_distance_m']=math.hypot(after.position.x-before.position.x,after.position.y-before.position.y)
                    assert .4<result['backup_distance_m']<.8
                else:
                    def yaw(p):
                        q=p.orientation;return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    result['spin_angle_rad']=abs(math.atan2(math.sin(yaw(after)-yaw(before)),math.cos(yaw(after)-yaw(before))))
                    assert .17<result['spin_angle_rad']<.35
            result['recoveries']=['back_up','spin']
        if args.phase=='chassis':
            def rates(values):
                a=np.array(values);dt=np.diff(a[:,0]);good=dt>1e-5
                acceleration=np.diff(a[:,1:],axis=0)[good]/dt[good,None]
                t=((a[:-1,0]+a[1:,0])/2)[good]
                jerk=np.diff(acceleration,axis=0)/np.diff(t)[:,None]
                return {'max_acceleration':np.max(abs(acceleration),axis=0).tolist(),
                        'max_jerk':np.max(abs(jerk),axis=0).tolist()}
            result['smoothed_command_limits']=rates(shaped)
            result['applied_command_limits']=rates(applied)
            assert result['smoothed_command_limits']['max_acceleration'][0]<.19,result
            assert result['smoothed_command_limits']['max_acceleration'][1]<.09,result
            assert result['smoothed_command_limits']['max_jerk'][0]<.43,result
            assert result['smoothed_command_limits']['max_jerk'][1]<.18,result
            assert result['applied_command_limits']['max_acceleration'][0]<.31,result
            assert result['applied_command_limits']['max_jerk'][0]<.63,result
            result['trajectory_steering_reversals_max']=max(steering_reversals,default=0)
            result['trajectory_steering_reversals_p95']=float(np.quantile(steering_reversals,.95)) if steering_reversals else 0
            assert result['trajectory_steering_reversals_p95']<=1,result
        assert not guard_reasons.get('invalid_control_interval',0),guard_reasons
        result['guard_reasons']=guard_reasons
        result['counts']=counts
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('PASS',json.dumps(result),flush=True)
    finally:
        if paused_pid is not None:os.kill(paused_pid,signal.SIGCONT)
        def pose_dict(p):
            q=p.orientation
            return {'xyz':[p.position.x,p.position.y,p.position.z],
                    'quaternion':[q.x,q.y,q.z,q.w]}
        final_reference={}
        if rclpy.ok() and physics.service_is_ready():
            try:final_reference['physical']=pose_dict(actual())
            except Exception as exc:final_reference['query_error']=str(exc)
        for key in ('odom','localized'):
            if key in data:final_reference[key]=pose_dict(data[key].pose.pose)
        (output/'debug.json').write_text(json.dumps({'counts': counts,
            'guard_reasons':guard_reasons,
            'last_guard_status':data['guard'].data if 'guard' in data else None,
            'last_localization_status': data['status'].data if 'status' in data else None,
            'sensors_healthy': data['sensors'].data if 'sensors' in data else None,
            'commands': commands, 'shaped_commands':shaped,'applied_commands':applied,'steering_reversals':steering_reversals, 'body_samples':body_samples,'final_reference':final_reference,'map_backends':backends,
            'local_path':[[p.pose.position.x,p.pose.position.y,p.pose.orientation.z,p.pose.orientation.w]
                          for p in data['local'].poses] if 'local' in data else []}, indent=2)+'\n')
        if 'obstacles' in data:
            cloud=data['obstacles'];fields={f.name:f.offset for f in cloud.fields}
            points=np.frombuffer(cloud.data,dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                'offsets':[fields[k] for k in ['x','y','z']],'itemsize':cloud.point_step}))
            np.save(output/'last_obstacles.npy',np.column_stack([points[k] for k in ['x','y','z']]))
        navigator.destroy(); recovery.destroy(); node.destroy_node(); rclpy.try_shutdown()
        stop_launch()
        log.close(); print('Results:',output,flush=True)


if __name__=='__main__': main()
