"""自然仿真时间的 ROS 在线采样、评估及记录。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import random
import sys
import time
import uuid
import numpy as np
import torch
from .core import Config, Replay, SAC
from .gates import convergence
from .goals import load_scene_goals, candidates, evaluation_goal_arguments
from .resume import (resolve_resume, locate_run_directory, previous_run, scene_digest,
                     append_record, preserve_summary)
from .artifacts import RunLayout, artifact_tag
from .terrain_sampling import TerrainSampler
from .stop_gate import StopGate
from .evaluation_environment import EvaluationEnvironment
from .training_metrics import TrainingMetrics

def source_directory():
    """! @brief 从源码、工作空间或显式环境变量定位记录目录。"""
    override=os.environ.get('CLEARING_DRL_SOURCE')
    if override:
        return Path(override).expanduser().resolve()
    roots=[Path(__file__).resolve(),Path.cwd().resolve()]
    roots.extend(Path(p).resolve() for p in os.environ.get('AMENT_PREFIX_PATH','').split(os.pathsep) if p)
    for root in roots:
        for ancestor in (root,*root.parents):
            for candidate in (ancestor,ancestor/'src'/'clearing_robot_drl',ancestor/'MultiMapNav_Simulation'/'src'/'clearing_robot_drl'):
                if (candidate/'package.xml').is_file() and (candidate/'clearing_robot_drl'/'core.py').is_file():
                    return candidate
    # 无源码安装时使用用户工作目录；可通过环境变量或 CLI 覆盖。
    return Path.cwd().resolve()

SOURCE=source_directory()
OBSERVATION_REVISION = 'obstacle-only-localmap-v2'

class PlanningTimeout(TimeoutError):
    """! @brief 标记规划 action 超时，和其他服务故障分开记录。"""

class SpawnCollision(PlanningTimeout):
    """! @brief 复位定位有效，但候选起点持续碰撞。"""

class RecoverableFault(RuntimeError):
    """! @brief 可停车后重试的数据或服务暂时不可用。"""

class RecoveryExhausted(RuntimeError):
    """! @brief 独立评估在恢复预算耗尽后结束本次评估。"""

def yaw(q):
    """! @brief 从四元数计算横滚、俯仰、航向角。"""
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
    heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,pitch,heading

def tracking(path,pose):
    """! @brief 把位姿投影到完整路径。
    @return 横向误差（m）、航向误差（rad）和从路径起点累计的弧长（m）。
    """
    p=pose.position
    best=None
    arc_length = 0.
    heading = yaw(pose.orientation)[2]
    for a,b in zip(path.poses,path.poses[1:]):
        x,y=a.pose.position.x,a.pose.position.y
        dx=b.pose.position.x-x
        dy=b.pose.position.y-y
        length2=dx*dx+dy*dy
        if length2<1e-10:
            continue
        t=max(0.,min(1.,((p.x-x)*dx+(p.y-y)*dy)/length2))
        d=math.hypot(p.x-x-t*dx,p.y-y-t*dy)
        h=math.atan2(dy,dx)
        err=math.remainder(heading-h, 2*math.pi)
        edge_length = math.sqrt(length2)
        if best is None or d<best[0]:
            best=(d,err,arc_length+t*edge_length)
        arc_length += edge_length
    if best is None:
        raise ValueError('empty path')
    return best

def main():
    """! @brief 运行指定场景的训练或配对评估。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import TwistStamped
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseStamped
    from gazebo_msgs.srv import GetEntityState, SetEntityState
    from clearing_robot_plugins.srv import TrainingControl, ResetEstimator
    from multimap_nav_msgs.action import ComputePathToPose
    from nav_msgs.msg import Path as NavPath
    ap=argparse.ArgumentParser()
    ap.add_argument('--mode',choices=['train','eval'],default='eval')
    ap.add_argument('--policy',choices=['hybrid','no_prior','base','mppi'],default='hybrid')
    ap.add_argument('--weights')
    ap.add_argument('--resume',help='原运行目录、完整检查点路径或 latest（需指定运行 ID）')
    ap.add_argument('--experiment',default='sac-hp',help='实验名称，用于生成可读产物名')
    ap.add_argument('--check-resume',action='store_true',help='只读验证续训检查点')
    ap.add_argument('--reset-stop-sim-timeout', type=float, default=12.)
    ap.add_argument('--reset-stop-wall-timeout', type=float, default=55.)
    ap.add_argument('--reset-ready-timeout', type=float, default=20.,
                    help='每次等待复位后新定位、TF 和地图的墙钟秒数')
    ap.add_argument('--recovery-attempts', type=int, default=3, help='每个暂态故障的尝试上限（含首次）')
    ap.add_argument('--planner-recovery-timeout', type=float, default=30.,
                    help='等待同一规划目标确认释放的墙钟秒数')
    ap.add_argument('--progress-timeout', type=float, default=20., help='无路径进展超时（仿真秒）')
    ap.add_argument('--progress-min-distance', type=float, default=.50, help='进展窗口需要的路径弧长增量（米）')
    ap.add_argument('--spawn-attempts',type=int,default=3,help='每回合随机起点复试次数')
    ap.add_argument('--seed',type=int,default=1)
    ap.add_argument('--records',default=str(SOURCE/'records'))
    ap.add_argument('--models',default=str(SOURCE/'models'))
    ap.add_argument('--run-id',default=None)
    ap.add_argument('--episodes',type=int,default=100)
    ap.add_argument('--scene',default='training')
    ap.add_argument('--entity',default='clearing_robot')
    ap.add_argument('--spawn-x',type=float,default=0.)
    ap.add_argument('--spawn-y',type=float,default=0.)
    ap.add_argument('--spawn-z',type=float,default=.03)
    ap.add_argument('--goal-source',default=None,help='scene（默认）、rectangle 或目标 JSON 文件路径')
    ap.add_argument('--eval-goal-source',default='scene',help='独立评估目标来源')
    ap.add_argument('--goal-x-min',type=float,default=None)
    ap.add_argument('--goal-x-max',type=float,default=None)
    ap.add_argument('--goal-y-min',type=float,default=None)
    ap.add_argument('--goal-y-max',type=float,default=None)
    ap.add_argument('--goal-z',type=float,default=None)
    ap.add_argument('--goal-min-distance',type=float,default=2.)
    ap.add_argument('--observation-timeout',type=float,default=12.0)
    ap.add_argument('--local-planning-timeout',type=float,default=3.0)
    ap.add_argument('--planning-timeout',type=float,default=18.)
    ap.add_argument('--planning-timeout-retries',type=int,default=3)
    ap.add_argument('--goal-tolerance',type=float,default=.6)
    ap.add_argument('--timeout',type=float,default=240.)
    ap.add_argument('--period',type=float,default=.2)
    ap.add_argument('--batch',type=int,default=256)
    ap.add_argument('--warmup',type=int,default=1000)
    ap.add_argument('--max-steps',type=int,default=1000000)
    ap.add_argument('--record-every-steps',type=int,default=50)
    ap.add_argument('--log-every-steps',type=int,default=20,help='终端进度输出间隔（步）')
    ap.add_argument('--checkpoint-every-steps',type=int,default=5000)
    ap.add_argument('--roll-limit',type=float,default=.35)
    ap.add_argument('--pitch-limit',type=float,default=.35)
    ap.add_argument('--gate-success',type=float,default=.9)
    ap.add_argument('--gate-collision',type=float,default=.05)
    ap.add_argument('--eval-scene',default='evaluation')
    ap.add_argument('--eval-every-steps',type=int,default=5000)
    ap.add_argument('--eval-episodes',type=int,default=20)
    ap.add_argument('--eval-seed',type=int,default=100000)
    ap.add_argument('--eval-wall-timeout',type=float,default=7200.)
    ap.add_argument('--eval-startup-timeout',type=float,default=300.)
    ap.add_argument('--convergence-window',type=int,default=3)
    ap.add_argument('--reward-relative',type=float,default=.05)
    ap.add_argument('--reward-trend',type=float,default=.015)
    ap.add_argument('--quality-trend-fraction',type=float,default=.45)
    ap.add_argument('--entropy-upper-bound',type=float,default=.8)
    ap.add_argument('--success-spread',type=float,default=.03)
    ap.add_argument('--entropy-drop',type=float,default=.05)
    ap.add_argument('--entropy-spread',type=float,default=.1)
    ap.add_argument('--entropy-jump',type=float,default=.1)
    ap.add_argument('--max-oscillation',type=float,default=.04)
    ap.add_argument('--oscillation-spread',type=float,default=.02)
    ap.add_argument('--max-speed-change',type=float,default=.1)
    ap.add_argument('--speed-change-spread',type=float,default=.03)
    ap.add_argument('--collision-spread',type=float,default=.03)
    ap.add_argument('--yaw-deadband',type=float,default=.01)
    args=ap.parse_args()
    rectangle_fields=('goal_x_min','goal_x_max','goal_y_min','goal_y_max','goal_z')
    explicit_rectangle=any(getattr(args,name) is not None for name in rectangle_fields)
    if args.goal_source is None:
        args.goal_source='rectangle' if explicit_rectangle else ('terrain' if args.scene == 'training_rough_v2' else 'scene')
    elif explicit_rectangle and args.goal_source != 'rectangle':
        ap.error('目标矩形参数仅可与 --goal-source rectangle 一起使用')
    defaults=(8.,25.,8.,15.,0.)
    rectangle=tuple(default if getattr(args,name) is None else getattr(args,name)
                    for name,default in zip(rectangle_fields,defaults))
    sampler = None
    if args.goal_source == 'terrain':
        try:
            sampler = TerrainSampler.from_scene(args.scene)
        except (OSError, ValueError, LookupError) as exc:
            ap.error('无法加载随机采样地形: '+str(exc))
        goal_list, goal_resource = None, args.scene + '_sampling.json'
    elif args.goal_source == 'rectangle':
        if not all(math.isfinite(v) for v in rectangle) or rectangle[0]>=rectangle[1] or rectangle[2]>=rectangle[3]:
            ap.error('invalid goal rectangle')
        goal_list,goal_resource=None,'rectangle'
    else:
        try:
            goal_list,goal_resource=load_scene_goals(args.scene,args.goal_source)
        except (OSError, ValueError, ImportError, LookupError) as exc:
            ap.error('无法加载场景目标: '+str(exc))
    if args.check_resume and not args.resume:
        ap.error('--check-resume 需要 --resume')
    recovery_limits = (args.reset_stop_sim_timeout, args.reset_stop_wall_timeout, args.reset_ready_timeout,
                       args.planner_recovery_timeout, args.progress_timeout, args.progress_min_distance)
    if args.recovery_attempts < 1 or not all(math.isfinite(t) and t > 0 for t in recovery_limits):
        ap.error('恢复尝试次数、时限和路径进展阈值必须为有限正数')
    if args.spawn_attempts < 1:
        ap.error('--spawn-attempts 必须大于零')
    resume_directory = None
    try:
        if args.resume:
            argument = Path(args.resume).expanduser()
            resume_directory = argument if argument.is_dir() else None
            args.resume = str(resolve_resume(args.resume, models_root=args.models,
                                             run_id=args.run_id, records_root=args.records))
    except (ValueError, FileNotFoundError) as exc:
        ap.error(str(exc))
    if args.resume and args.weights:
        ap.error('choose --resume or --weights')
    if args.mode=='train' and args.policy=='mppi':
        ap.error('MPPI is an evaluation baseline')
    if args.period<=0 or args.timeout<=0 or args.episodes<1 or args.eval_every_steps<1 or args.eval_episodes<1 or args.convergence_window<2 or args.max_steps<1 or args.record_every_steps<1 or args.log_every_steps<1 or args.checkpoint_every_steps<1 or args.planning_timeout<=0 or args.local_planning_timeout<=0 or args.observation_timeout<=0 or args.planning_timeout_retries<1 or args.eval_wall_timeout<=0 or args.eval_startup_timeout<=0:
        ap.error('invalid duration, window or episode count')
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    cfg=Config(ablation='hybrid' if args.policy=='mppi' else args.policy)
    if args.resume:
        agent,replay,extra=SAC.load(args.resume,resume=True)
        if agent.cfg.ablation != cfg.ablation:
            raise ValueError('resume policy mismatch')
    elif args.weights:
        agent,_,_=SAC.load(args.weights)
        if args.policy != 'mppi' and agent.cfg.ablation != args.policy:
            raise ValueError('weight policy mismatch')
        replay=Replay(seed=args.seed)
        extra={}
    else:
        agent=SAC(cfg)
        replay=Replay(seed=args.seed)
        extra={}
    if args.mode=='eval' and args.policy!='mppi' and not args.weights:
        ap.error('evaluation requires --weights')
    extra = extra or {}
    runid=args.run_id or extra.get('run_id') or uuid.uuid4().hex[:12]
    layout = None
    scene_signature = scene_digest(args.scene) if args.mode == 'train' else None
    migration = None
    if args.resume:
        if args.mode != 'train' or extra.get('run_id') != runid or replay is None:
            ap.error('续训须使用同一运行的完整训练检查点')
        resumed_step = int(extra.get('global_step', 0))
        try:
            run_root = resume_directory or locate_run_directory(args.resume, args.records, runid)
            layout = RunLayout.open(run_root)
            directory = layout.records
            prior_run = previous_run(layout.root, extra, resumed_step)
            args.experiment = prior_run.get('experiment', args.experiment)
        except (OSError, ValueError) as exc:
            ap.error(str(exc))
        old_scene = extra.get('scene', prior_run['scene'])
        old_signature = extra.get('scene_signature')
        old_observation = extra.get('observation_revision', 'legacy-localmap')
        change = (old_scene != args.scene or (old_signature and old_signature != scene_signature) or
                  old_observation != OBSERVATION_REVISION)
        if change:
            # 场景切换后开启新回合，跳过原 CSV 已写出的回合编号。
            next_episode = extra.get('episode', 0)
            episode_trace = directory / 'episodes.csv'
            if episode_trace.is_file():
                with episode_trace.open(newline='', encoding='utf-8') as stream:
                    next_episode = max(next_episode, 1 + max(
                        (int(row['episode']) for row in csv.DictReader(stream) if row.get('episode')), default=-1))
            migration = dict(previous_scene=old_scene, scene=args.scene, checkpoint_step=resumed_step,
                             previous_observation_revision=old_observation,
                             observation_revision=OBSERVATION_REVISION,
                             resume_episode=next_episode,
                             first_interaction_step=resumed_step + 1, checkpoint=args.resume,
                             scene_signature=scene_signature, old_replay_size=len(replay),
                             replay_policy='reset', evaluation_history_policy='reset')
            replay = Replay(capacity=replay.capacity, seed=args.seed + resumed_step)
            extra.update(episode=next_episode, evaluation_history=[], last_eval_step=resumed_step)
        if args.check_resume:
            print(json.dumps(dict(checkpoint=args.resume, run_id=runid, step=resumed_step,
                                  updates=agent.updates, replay_size=len(replay), scene_change=migration),
                             ensure_ascii=False), flush=True)
            return
    def log(message):
        """! @brief 输出可及时刷新的训练运行状态。"""
        print(f'[DRL {runid}] {message}',flush=True)
    log(f'启动 mode={args.mode} scene={args.scene} policy={args.policy} seed={args.seed} goal_source={goal_resource}')
    if layout is None:
        layout = RunLayout.create(args.records, args.experiment, args.scene, runid)
    else:
        layout.ensure_output_dirs()
    directory = layout.records
    log(f'运行目录={layout.root}')
    meta=vars(args).copy()
    meta.update(run_id=runid,algorithm='SAC-HP',training_updates_at_start=agent.updates,
                goal_resource=goal_resource, scene_signature=scene_signature, observation_revision=OBSERVATION_REVISION,
                start_step=int(extra.get('global_step', 0)))
    if args.resume:
        preserve_summary(directory, meta['start_step'])
        append_record(layout.parameters/'sessions.jsonl', meta)
        if migration:
            log(f"目标场景 {args.scene} 的首个交互拟从 step={migration['first_interaction_step']} 开始，旧回放 {migration['old_replay_size']} 条保存在原检查点")
    else:
        layout.run_json.write_text(json.dumps(meta,ensure_ascii=False,indent=2))
    def writer(name,fields):
        """! @brief 创建或追加指定字段的 CSV 表格。"""
        f=(directory/name).open('a',newline='',encoding='utf8')
        w=csv.DictWriter(f,fieldnames=fields)
        if f.tell()==0:
            w.writeheader()
        return f,w
    sf,sw=writer('steps.csv',['episode','step','episode_step','sim_time','dt','reward','distance','lateral_error','heading_error','roll','pitch','action_v','action_w','prior_v','prior_w','collision','entropy_sample','reward_progress','reward_time','reward_tracking','reward_event','yaw_oscillation','yaw_oscillation_amplitude','speed_change'])
    ef,ew=writer('episodes.csv',['episode','seed','goal_x','goal_y','goal_z','outcome','duration','steps','return','mean_lateral_error','max_lateral_error','mean_abs_heading_error','entropy_mean','yaw_oscillation_rate','yaw_oscillation_amplitude','speed_change_mean','reward_progress','reward_time','reward_tracking','reward_event'])
    errors=(directory/'errors.jsonl').open('a',encoding='utf8')
    metrics_writer = TrainingMetrics(layout.root/'tensorboard', flush_every=args.record_every_steps)
    # Ctrl+C 先进入 Python 保存和停车逻辑，再关闭 ROS 上下文。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('clearing_drl_runtime',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)
    control=node.create_client(TrainingControl,'/clearing/training_control')
    gazebo=node.create_client(SetEntityState,'/gazebo/set_entity_state')
    gazebo_state=node.create_client(GetEntityState,'/gazebo/get_entity_state')
    reset=node.create_client(ResetEstimator,'/clearing/reset_estimator')
    planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    applied_feedback = None

    def on_applied(message):
        """! @brief 记录履带驱动执行的停驶状态，供仿真复位使用。"""
        nonlocal applied_feedback
        tag = message.header.stamp
        applied_feedback = (time.monotonic(), tag.sec + tag.nanosec * 1e-9,
                            (message.twist.linear.x, message.twist.angular.z))

    drive_feedback = node.create_subscription(TwistStamped, '/clearing/cmd_vel_applied', on_applied, 1)
    def wait(future,seconds=20.):
        """! @brief 等待 ROS 异步调用，墙钟超时后抛出超时异常。"""
        deadline=time.monotonic()+seconds
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(.05, remaining))
        if not future.done():
            raise TimeoutError('ROS request timed out')
        result=future.result()
        if result is None:
            raise RuntimeError('ROS request failed')
        return result
    def call(client,request,seconds=20.):
        """! @brief 在指定墙钟秒数内完成 ROS 服务发现和请求。"""
        deadline = time.monotonic() + seconds
        if not client.wait_for_service(timeout_sec=seconds):
            raise TimeoutError('service unavailable: '+client.srv_name)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError('service request budget exhausted: '+client.srv_name)
        return wait(client.call_async(request), remaining)
    def stop(seconds=20.):
        """! @brief 在指定墙钟秒数内请求训练控制节点停车。"""
        req=TrainingControl.Request()
        req.operation=2
        response=call(control,req,seconds)
        if not response.success:
            raise RuntimeError('STOP: '+response.reason)

    def record_stage(stage, result, started, reason='', attempt=None):
        """! @brief 记录恢复阶段、墙钟耗时和首个失败理由。"""
        detail = dict(episode=episode, step=global_step, sim_time=stamp(), stage=stage,
                      result=result, wall_wait=time.monotonic()-started, reason=reason, attempt=attempt)
        append_record(directory/'recovery.jsonl', detail)
        metrics_writer.reset(global_step, detail)

    def hold_for_operator(stage, reason):
        """! @brief 恢复用尽时停车、保存并等待操作员中断。
        @param stage 最先无法恢复的阶段。
        @param reason 最近的诊断和时间戳。
        """
        started = time.monotonic()
        log(f'进入安全等待 stage={stage} step={global_step} 原因={reason}')
        record_stage(stage, 'waiting', started, reason)
        try:
            stop(3.)
        except (TimeoutError, RuntimeError) as exc:
            log(f'停车请求未确认，等待底盘看门狗停止命令: {exc}')
        if args.mode == 'eval':
            raise RecoveryExhausted(f'{stage}: {reason}')
        save_progress(episode, 'recovery_wait', partial=True)
        while rclpy.ok():
            log(f'安全等待 stage={stage} step={global_step}，按 Ctrl+C 保存并结束后可续训')
            deadline = time.monotonic() + 10.
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.10)
        raise ExternalShutdownException()

    def recover(stage, operation, report_success=False):
        """! @brief 对暂态错误做有限次原阶段重试，预算耗尽后进入安全等待。"""
        last_error = ''
        for attempt in range(1, args.recovery_attempts + 1):
            started = time.monotonic()
            if report_success:
                log(f'回合 {episode} stage={stage} 尝试 {attempt}/{args.recovery_attempts}')
            try:
                result = operation()
            except PlanningTimeout:
                raise
            except (TimeoutError, RecoverableFault) as exc:
                last_error = str(exc)
                record_stage(stage, 'retry' if attempt < args.recovery_attempts else 'exhausted',
                             started, last_error, attempt)
                log(f'恢复 stage={stage} 尝试 {attempt}/{args.recovery_attempts} 原因={last_error}')
                if attempt < args.recovery_attempts:
                    try:
                        stop(3.)
                    except (TimeoutError, RuntimeError) as stop_error:
                        log(f'重试前停车请求未确认: {stop_error}')
                    retry_at = time.monotonic() + 1.
                    while rclpy.ok() and time.monotonic() < retry_at:
                        rclpy.spin_once(node, timeout_sec=.10)
                    if not rclpy.ok():
                        raise ExternalShutdownException()
                continue
            if report_success or attempt > 1:
                record_stage(stage, 'ready', started, attempt=attempt)
                log(f'回合 {episode} stage={stage} 已就绪，耗时 {time.monotonic()-started:.2f}s')
            return result
        hold_for_operator(stage, last_error)

    def message_stamp(header):
        """! @brief 返回消息头的仿真时间（秒）。"""
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def tf_readiness(pose, epoch):
        """! @brief 核查两段动态 TF 的时间和地图位姿一致性。
        @param pose 与控制地图配对的定位位姿。
        @param epoch 最近一次实体移位确认时间（仿真秒）。
        @return 未就绪的具体原因，空字符串表示可用。
        """
        now = stamp()
        for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('map', 'base_link')):
            try:
                transform = tf_buffer.lookup_transform(parent, child, Time())
            except TransformException as exc:
                return f'TF {parent}->{child} 不可用: {exc}'
            measured = message_stamp(transform.header)
            if measured <= epoch or not -.1 <= now - measured <= .85:
                return f'TF {parent}->{child} 时间={measured:.3f}s clock={now:.3f}s epoch={epoch:.3f}s'
        position = transform.transform.translation
        if (math.hypot(position.x-pose.position.x, position.y-pose.position.y) > .65 or
                abs(position.z-pose.position.z) > .35):
            return 'TF map->base_link 与控制观测位置不一致'
        return ''

    def wait_observation(path, seconds, *, after=0., epoch=0.):
        """! @brief 在墙钟预算内等新定位、地图、驱动反馈和同期 TF。"""
        req = TrainingControl.Request()
        if path is not None:
            req.path = path
        deadline = time.monotonic() + seconds
        reason = '无控制观测'
        stopped = False
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            fresh = call(control, req, min(4., remaining))
            observed_at = message_stamp(fresh.header)
            if not fresh.success:
                reason = fresh.reason or '控制观测未就绪'
            elif observed_at <= max(after, epoch):
                reason = f'控制观测仍旧 obs={observed_at:.3f}s after={max(after, epoch):.3f}s'
            else:
                reason = tf_readiness(fresh.pose, epoch)
                if not reason:
                    return fresh
            if path is not None and not stopped:
                stop(3.)
                stopped = True
            rclpy.spin_once(node, timeout_sec=.04)
        if not rclpy.ok():
            raise ExternalShutdownException()
        raise RecoverableFault(reason)

    def observe(path, after=0.):
        """! @brief 获取较指定时间新的、含有效 TF 的控制观测。"""
        return recover('observation', lambda: wait_observation(
            path, args.observation_timeout, after=after, epoch=episode_epoch))

    def feasible_observation(path, after=0.):
        """! @brief 短时地图封堵时停车等待，规划超时后结束回合。"""
        result = observe(path, after)
        if result.path_valid or result.collision:
            return result
        stop()
        deadline = stamp() + args.local_planning_timeout
        while stamp() < deadline:
            recover('simulation_clock', lambda: advance(.10))
            result = observe(path, message_stamp(result.header))
            if result.path_valid or result.collision:
                break
        return result
    def stamp():
        """! @brief 返回 ROS 仿真时钟的秒数。"""
        return node.get_clock().now().nanoseconds*1e-9
    def advance(seconds):
        """! @brief 等待仿真时钟自然推进指定秒数。"""
        start=stamp()
        deadline=time.monotonic()+max(10.,seconds*20.)
        while rclpy.ok() and stamp()-start<seconds:
            if time.monotonic()>deadline:
                raise TimeoutError('simulation clock stalled')
            rclpy.spin_once(node,timeout_sec=.02)
    def set_spawn_attitude(pose, x, y, yaw):
        """! @brief 由碰撞网格局部坡度估计复位四元数，让履带贴合地面。"""
        roll = pitch = 0.0
        if sampler:
            dx = (sampler.height_at(x + .3, y) - sampler.height_at(x - .3, y)) / .6
            dy = (sampler.height_at(x, y + .3) - sampler.height_at(x, y - .3)) / .6
            pitch = -math.atan(dx * math.cos(yaw) + dy * math.sin(yaw))
            roll = math.atan(-dx * math.sin(yaw) + dy * math.cos(yaw))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose.orientation.w = cr * cp * cy + sr * sp * sy

    def wait_for_stopped():
        """! @brief 验证零速送至驱动并确认 Gazebo 实体实际停稳，失败时拒绝复位。"""
        begin = time.monotonic()
        deadline = begin + args.reset_stop_wall_timeout
        clock_at_entry = stamp()
        request_stamp = clock_at_entry
        # 训练节点刚订阅 /clock 时可能读到 0，即使 Gazebo 已运行很久。
        # 等到时钟实际前进，再从发送 STOP 前的仿真时间开始计算停稳预算。
        while rclpy.ok() and request_stamp <= clock_at_entry:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(.05, remaining))
            request_stamp = stamp()
        detail = dict(clock_at_entry=clock_at_entry, stop_request_stamp=request_stamp,
                      clock_wait_wall=time.monotonic()-begin, stop_acknowledged=False)
        if request_stamp <= max(0.0, clock_at_entry) or time.monotonic() >= deadline:
            detail.update(result='simulation_clock_not_advancing', timeout_limit='wall_time')
            write_stop_gate(dict(episode=episode, timeout=True, **detail))
            raise TimeoutError(f'复位前未收到推进中的仿真时钟: {detail}')

        gate = StopGate(request_stamp)
        get = GetEntityState.Request()
        get.name = args.entity
        get.reference_frame = 'world'
        stop(min(20., deadline-time.monotonic()))
        detail.update(result='no_gazebo_state', stop_acknowledged=True)
        next_stop = time.monotonic() + .7
        timeout_limit = 'wall_time'
        while rclpy.ok() and time.monotonic() < deadline:
            now = stamp()
            if now < request_stamp:
                detail.update(result='simulation_clock_rewound', sim_stamp=now)
                timeout_limit = 'clock_rewind'
                break
            if now - request_stamp >= args.reset_stop_sim_timeout:
                timeout_limit = 'sim_time'
                break
            if time.monotonic() >= next_stop:
                stop(min(20., deadline-time.monotonic()))
                next_stop = time.monotonic() + .7
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            state = call(gazebo_state, get, min(3., remaining))
            if state.success:
                body = state.state.twist
                linear = (body.linear.x, body.linear.y, body.linear.z)
                angular = (body.angular.x, body.angular.y, body.angular.z)
                feedback_stamp = None
                applied = ()
                if applied_feedback is not None and time.monotonic() - applied_feedback[0] < 1.:
                    _, feedback_stamp, applied = applied_feedback
                tag = state.header.stamp
                clock = tag.sec + tag.nanosec * 1e-9
                ready, reason = gate.check(clock, feedback_stamp, applied, linear, angular)
                detail.update(result=reason, sim_stamp=clock, sim_wait=clock-request_stamp,
                              wall_wait=time.monotonic()-begin, applied=list(applied),
                              applied_stamp=feedback_stamp, gazebo_linear=list(linear),
                              gazebo_angular=list(angular))
                now = stamp()
                if now < request_stamp:
                    detail.update(result='simulation_clock_rewound', sim_stamp=now)
                    timeout_limit = 'clock_rewind'
                    break
                if max(now, clock) - request_stamp >= args.reset_stop_sim_timeout:
                    timeout_limit = 'sim_time'
                    break
                if ready and time.monotonic() < deadline:
                    write_stop_gate(dict(episode=episode, **detail))
                    return
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                rclpy.spin_once(node, timeout_sec=min(.02, remaining))
        detail.update(timeout_limit=timeout_limit, wall_wait=time.monotonic()-begin)
        write_stop_gate(dict(episode=episode, timeout=True, **detail))
        raise TimeoutError(f'底盘未能在复位前停稳: {detail}')

    def write_stop_gate(detail):
        """! @brief 追加物理停稳门原始测量和 TensorBoard 指标。"""
        append_record(directory/'resets.jsonl', detail)
        metrics_writer.reset(global_step, dict(detail, stage='stop_gate'))

    def wait_after_reset(pose, epoch):
        """! @brief 等待同一次移位之后的有效观测，拒绝持续碰撞的起点。
        @param pose Gazebo 接受的地图系初始位姿。
        @param epoch 实体移位确认时间（仿真秒）。
        @return 对应新 TF 和局部地图的控制位姿。
        """
        deadline = time.monotonic() + args.reset_ready_timeout
        collision_since = None
        after = epoch
        collision_reason = ''
        while time.monotonic() < deadline:
            fresh = wait_observation(None, deadline-time.monotonic(), after=after, epoch=epoch)
            after = message_stamp(fresh.header)
            if fresh.collision:
                collision_reason = fresh.reason or '碰撞来源未报告'
                if collision_since is None:
                    collision_since = after
                elif after - collision_since >= .75:
                    raise SpawnCollision(f'复位起点持续碰撞，观测={after:.3f}s: {collision_reason}')
                continue
            position = fresh.pose.position
            heading = yaw(fresh.pose.orientation)[2]
            expected_heading = yaw(pose.orientation)[2]
            if (math.hypot(position.x-pose.position.x, position.y-pose.position.y) > .8 or
                    abs(position.z-pose.position.z) > .45 or
                    abs(math.remainder(heading-expected_heading, 2*math.pi)) > .28):
                raise PlanningTimeout('复位后的有效定位与采样位姿偏差过大')
            if sampler and not sampler.is_feasible(position.x, position.y, heading):
                raise PlanningTimeout('定位后的完整车体包络未通过起点检查')
            return fresh.pose
        raise RecoverableFault(f'复位确认等待超时，最后碰撞原因={collision_reason}')

    def reset_episode(start_pose):
        """! @brief 停稳后移动实体，等待新定位和无碰撞的局部地图。
        @param start_pose 地图坐标系下的 x、y、z（m）和航向角（rad）。
        @return 通过地图匹配确认的机器人位姿。
        """
        nonlocal episode_epoch
        recover('stop', wait_for_stopped, report_success=True)
        req=SetEntityState.Request()
        req.state.name=args.entity
        req.state.reference_frame='world'
        x, y, z, angle = start_pose
        req.state.pose.position.x=x
        req.state.pose.position.y=y
        req.state.pose.position.z=z + args.spawn_z if sampler else z
        set_spawn_attitude(req.state.pose, x, y, angle)
        def teleport():
            """! @brief 发送同一姿态的幂等实体移位请求。"""
            if not call(gazebo, req, 5.).success:
                raise RecoverableFault('Gazebo 拒绝设置机器人状态')
        recover('gazebo_pose', teleport, report_success=True)
        episode_epoch = stamp()
        req2=ResetEstimator.Request()
        req2.pose.header.frame_id='map'
        req2.pose.header.stamp=node.get_clock().now().to_msg()
        req2.pose.pose=req.state.pose
        def restart_estimator():
            """! @brief 重启估计器并提交当前回合初值。"""
            result = call(reset, req2, 60.)
            if not result.success:
                raise RecoverableFault(result.reason)
        recover('lio_restart', restart_estimator, report_success=True)
        reinitialize = False

        def confirm_pose():
            """! @brief 定位确认超时后只以实测静止姿态重送初值，保持同一回合。"""
            nonlocal reinitialize, episode_epoch
            if reinitialize:
                wait_for_stopped()
                entity = GetEntityState.Request()
                entity.name = args.entity
                entity.reference_frame = 'world'
                measured = call(gazebo_state, entity, 4.)
                if not measured.success:
                    raise RecoverableFault('无法读取停稳后的实体位姿')
                actual = measured.state.pose
                if math.hypot(actual.position.x-x, actual.position.y-y) > .8:
                    raise PlanningTimeout('等待定位期间机器人已偏离起点')
                req2.pose.pose = actual
                req2.pose.header.stamp = measured.header.stamp
                episode_epoch = message_stamp(measured.header)
                restart_estimator()
            try:
                return wait_after_reset(req.state.pose, episode_epoch)
            except RecoverableFault as exc:
                reinitialize = str(exc).startswith('定位健康状态未恢复')
                raise

        return recover('post_reset_tf_observation', confirm_pose, report_success=True)
    def finish_overdue_plan(handle, result_future):
        """! @brief 等待超时规划释放目标；未确认前保持停车。
        @param handle 已接受的规划目标句柄。
        @param result_future 同一目标的原始结果请求。
        """
        terminal_statuses = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                             GoalStatus.STATUS_ABORTED)
        cancel_terminal = (CancelGoal.Response.ERROR_GOAL_TERMINATED,
                           CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID)
        goal_id = bytes(handle.goal_id.uuid).hex()
        cancel_future = None
        cancel_code = None
        cancel_accepted = None
        status = None
        started = time.monotonic()
        retry_at = started
        recovery_deadline = started + args.planner_recovery_timeout
        terminal_at = None
        while rclpy.ok():
            if cancel_future is not None and cancel_future.done():
                response = cancel_future.result()
                cancel_code = response.return_code
                cancel_accepted = any(item.goal_id == handle.goal_id for item in response.goals_canceling)
                cancel_future = None
            if result_future.done():
                status = result_future.result().status
            elif handle.status in terminal_statuses:
                status = handle.status
            # GetResult 返回 UNKNOWN 表示服务端已无目标。取消已受理只表示 CANCELING。
            terminal = (status in terminal_statuses or status == GoalStatus.STATUS_UNKNOWN or
                        cancel_code in cancel_terminal)
            now = time.monotonic()
            if terminal:
                if result_future.done() and cancel_future is None:
                    break
                if terminal_at is None:
                    terminal_at = now
                if now - terminal_at >= .25:
                    break
            elif now >= retry_at:
                if retry_at > started:
                    log(f'规划目标 {goal_id} 尚未确认结束，保持停车等待，取消响应码={cancel_code}')
                    if cancel_future is not None:
                        cancel_future.cancel()
                cancel_future = handle.cancel_goal_async()
                retry_at = now + 10.
            if not terminal and now >= recovery_deadline:
                hold_for_operator('planner_cancel', f'目标={goal_id} 取消响应码={cancel_code}')
            rclpy.spin_once(node, timeout_sec=.05)
        else:
            raise ExternalShutdownException()
        for future in (result_future, cancel_future):
            if future is not None and not future.done():
                future.cancel()
        detail = dict(episode=episode, step=global_step, goal_id=goal_id,
                      status=status, cancel_code=cancel_code, cancel_accepted=cancel_accepted,
                      wall_wait=time.monotonic()-started)
        append_record(directory/'planner_events.jsonl', detail)
        log(f'超时规划已结束，目标={goal_id} 状态={status} 取消响应码={cancel_code}')

    def request_global_plan(goal, seconds):
        """! @brief 请求一次规划，超时后排空该目标并丢弃迟到的路径。
        @param goal 发送给全局规划 action 的目标。
        @param seconds 目标受理和规划共用的墙钟预算（秒）。
        @return 结果响应与是否发生超时。拒绝或超时时不返回路径。
        """
        deadline = time.monotonic() + seconds
        acceptance = planner.send_goal_async(goal)
        timed_out = False
        try:
            handle = wait(acceptance, seconds)
        except TimeoutError:
            timed_out = True
            stop()
            log('规划目标受理响应超时，停车等待同一请求')
            try:
                handle = wait(acceptance, args.planner_recovery_timeout)
            except TimeoutError:
                hold_for_operator('planner_goal_ack', '规划请求的受理状态未知')
        if not handle.accepted:
            return None, timed_out
        result_future = handle.get_result_async()
        if not timed_out:
            try:
                return wait(result_future, max(0., deadline - time.monotonic())), False
            except TimeoutError:
                stop()
        finish_overdue_plan(handle, result_future)
        return None, True

    def route(rng, pose):
        # 独立 episode RNG 使不同算法以同 seed 得到相同候选序列。
        """! @brief 以当前回合的种子生成目标，用全局规划筛选可达候选。"""
        current=pose.position
        current_yaw=yaw(pose.orientation)[2]
        start = (current.x,current.y,current.z,current_yaw)
        if not planner.wait_for_server(timeout_sec=20.):
            raise PlanningTimeout('全局规划服务器未就绪')
        planning_timeouts=0
        planning_deadline = time.monotonic() + args.planning_timeout * args.planning_timeout_retries
        goals = (sampler.sample_goals(rng, start, max(10., args.goal_min_distance), 24) if sampler else
                 ((x,y,z, math.atan2(y-current.y,x-current.x)) for x,y,z in candidates(rng,goal_list,rectangle)))
        for x,y,z,angle in goals:
            remaining = planning_deadline - time.monotonic()
            if remaining <= 0:
                raise PlanningTimeout('目标采样期间未在规划预算内获得可行全局路径')
            if math.hypot(x-current.x,y-current.y)<args.goal_min_distance:
                continue
            goal=ComputePathToPose.Goal()
            goal.goal.header.frame_id='map'
            goal.goal.header.stamp=node.get_clock().now().to_msg()
            goal.goal.pose.position.x=x
            goal.goal.pose.position.y=y
            goal.goal.pose.position.z=z
            goal.goal.pose.orientation.z=math.sin(angle/2)
            goal.goal.pose.orientation.w=math.cos(angle/2)
            result, timed_out = request_global_plan(goal, min(remaining, args.planning_timeout))
            if timed_out:
                planning_timeouts+=1
                if planning_timeouts>=args.planning_timeout_retries:
                    raise PlanningTimeout('连续多次全局规划超时，相关目标均已结束')
            if result is None:
                continue
            path=result.result.path
            if result.status!=GoalStatus.STATUS_SUCCEEDED or len(path.poses)<2:
                continue
            end=path.poses[-1].pose.position
            if math.hypot(end.x-x,end.y-y)>args.goal_tolerance:
                continue
            return path,x,y,z
        raise PlanningTimeout('未在候选目标中找到可行路径')
    outcomes=[]
    global_step=(extra or {}).get('global_step',0)
    episode_epoch = 0.
    first=(extra or {}).get('episode',0)
    history=(extra or {}).get('evaluation_history',[])
    last_eval_step=(extra or {}).get('last_eval_step',0)
    last_checkpoint_step=(extra or {}).get('last_checkpoint_step',global_step)
    stop_reason='maximum_episodes'
    def evaluate_snapshot(episode):
        """! @brief 按需启动隔离仿真环境并运行固定种子评估。"""
        eval_id=runid+f'-eval-{global_step}-'+uuid.uuid4().hex[:8]
        log(f'开始独立评估 step={global_step} scene={args.eval_scene} seed={args.eval_seed}')
        snapshot=layout.weights/(artifact_tag(args.experiment,args.scene,runid,global_step,episode,'eval')+'-weights.pt')
        agent.weights(snapshot)
        checkpoint=save_progress(episode,'evaluation',partial=False)
        command=[sys.executable,'-m','clearing_robot_drl.runtime','--mode','eval','--policy',args.policy,'--weights',str(snapshot),
                 '--scene',args.eval_scene,'--experiment',args.experiment,'--run-id',eval_id,'--records',args.records,'--models',args.models,
                 '--episodes',str(args.eval_episodes),'--seed',str(args.eval_seed),'--entity',args.entity,
                 '--period',str(args.period),'--timeout',str(args.timeout)]
        for option in ('observation_timeout', 'local_planning_timeout', 'planning_timeout',
                       'planning_timeout_retries', 'spawn_x', 'spawn_y', 'spawn_z',
                       'goal_min_distance', 'goal_tolerance', 'roll_limit', 'pitch_limit',
                       'yaw_deadband', 'reset_ready_timeout', 'recovery_attempts',
                       'planner_recovery_timeout', 'progress_timeout', 'progress_min_distance'):
            command.extend(['--'+option.replace('_','-'),str(getattr(args,option))])
        command.extend(evaluation_goal_arguments(args.eval_goal_source,rectangle))
        try:
            with EvaluationEnvironment(args.eval_scene,directory,eval_id,args.eval_startup_timeout, status=log) as environment:
                returncode=environment.run(command,args.eval_wall_timeout)
            metrics_path=RunLayout.open(RunLayout.path_for(args.records,args.experiment,args.eval_scene,eval_id)).records/'metrics.json'
            if returncode!=0 or not metrics_path.exists():
                raise RuntimeError('evaluation subprocess failed, see '+str(directory/(eval_id+'-worker.log')))
            metrics=json.loads(metrics_path.read_text())
            log(f"独立评估完成 success={metrics.get('success_rate')} collision={metrics.get('collision_rate')} complete={metrics.get('complete')}")
            metrics.update(training_step=global_step,training_episode=episode,weights=str(snapshot),evaluation_run=eval_id,scene=args.eval_scene,seed=args.eval_seed,checkpoint=str(checkpoint))
            return metrics
        except Exception as exc:
            log(f'独立评估失败 step={global_step} error={exc!r}')
            errors.write(json.dumps({'episode':episode,'training_step':global_step,'evaluation_error':repr(exc)})+'\n')
            errors.flush()
            return {'complete':False,'training_step':global_step,'error':repr(exc)}
    def save_progress(episode,reason,partial=False):
        """! @brief 按运行、累计步数和唯一标识保存不可覆盖的快照。"""
        state={'run_id':runid,'episode':episode if partial else episode+1,'global_step':global_step,
               'evaluation_history':history,'last_eval_step':last_eval_step,
               'last_checkpoint_step':last_checkpoint_step,'stop_reason':reason,
               'scene':migration['previous_scene'] if migration else args.scene,
               'scene_signature':None if migration else scene_signature,
               'observation_revision': old_observation if migration else OBSERVATION_REVISION,
               'partial_episode':partial}
        tag=artifact_tag(args.experiment,args.scene,runid,global_step,episode,reason)
        weights=layout.weights/(tag+'-weights.pt')
        checkpoint=layout.checkpoints/(tag+'-checkpoint.pt')
        parameter=layout.parameters/(tag+'-parameters.json')
        agent.weights(weights)
        agent.checkpoint(checkpoint,replay,state)
        with parameter.open('x', encoding='utf-8') as stream:
            json.dump(dict(meta, checkpoint_step=global_step, checkpoint_episode=episode,
                           checkpoint=str(checkpoint)),stream,ensure_ascii=False,indent=2)
        log(f'保存 reason={reason} step={global_step} checkpoint={checkpoint}')
        (directory/'stop.json').write_text(json.dumps({'reason':reason,'episode':episode,'partial_episode':partial,
            'training_step':global_step,'weights':str(weights),'checkpoint':str(checkpoint)},indent=2))
        return checkpoint
    episode_complete=False
    episode=first
    try:
        if sampler:
            settings = node.create_client(GetParameters, '/clearing_navigation_modes/get_parameters')
            request = GetParameters.Request()
            request.names = ['pcd_file']
            reply = call(settings, request, 10.)
            if not reply.values or Path(reply.values[0].string_value).name != f'{args.scene}.pcd':
                raise RuntimeError(f'导航 PCD 与采样场景 {args.scene} 不匹配')
        for episode in range(first,first+args.episodes):
            episode_complete=False
            if args.mode=='train' and global_step>=args.max_steps:
                save_progress(episode,'maximum_steps',partial=True)
                break
            start = None
            steps=0
            total=0.
            lat=[]
            heading=[]
            entropies=[]
            changes=[]
            oscillations=[]
            amplitudes=[]
            components=dict(progress=0.,time=0.,tracking=0.,event=0.)
            last_speed=None
            last_sign=0
            last_angular=0.0
            gx=gy=gz=float('nan')
            outcome='error'
            try:
                log(f'回合 {episode} 开始复位')
                rng = np.random.default_rng(args.seed + episode)
                for attempt in range(args.spawn_attempts if sampler else 1):
                    candidate = sampler.sample_start(rng) if sampler else (args.spawn_x,args.spawn_y,args.spawn_z,0.)
                    try:
                        pose = reset_episode(candidate)
                        log(f'回合 {episode} 复位验收通过，开始全局规划')
                        path, gx, gy, gz = route(rng, pose)
                        break
                    except PlanningTimeout as exc:
                        log(f'起终点配对第 {attempt+1} 次未通过: {exc}')
                        if attempt+1 == (args.spawn_attempts if sampler else 1):
                            raise
                if sampler:
                    append_record(directory/'sampling.jsonl', dict(episode=episode, step=global_step,
                        scene=args.scene, spawn=list(candidate), goal=[gx,gy,gz]))
                    log(f'回合 {episode} 随机起点={candidate}')
                log(f'回合 {episode} 目标=({gx:.2f},{gy:.2f},{gz:.2f})m 路径点={len(path.poses)}')
                obs = feasible_observation(path)
                start = stamp()
                _, _, progress_anchor = tracking(path, obs.pose)
                progress_since = start
                outcome = ''
                if not obs.path_valid:
                    if obs.collision:
                        outcome = 'collision'
                        components['event'] = -20
                    else:
                        outcome = 'planning_timeout'
                        components['event'] = -10
                    total = components['event']
                previous=math.hypot(obs.pose.position.x-gx,obs.pose.position.y-gy)
                action_failures = 0
                while not outcome:
                    o=np.asarray(obs.observation,dtype=np.float32)
                    prior=np.asarray(obs.prior,dtype=np.float32)
                    action=prior if args.policy=='mppi' else agent.act(o,prior,deterministic=args.mode=='eval')
                    entropy=None
                    if args.policy!='mppi':
                        with torch.no_grad():
                            ot=torch.as_tensor(o,device=agent.device).reshape(1,28)
                            pt=torch.as_tensor(prior,device=agent.device).reshape(1,2)
                            _,lp=agent.policy.sample(ot,pt)
                            entropy=float(-lp.item())
                    req=TrainingControl.Request()
                    req.operation=1
                    req.path=path
                    req.action=action.tolist()
                    req.observation_stamp=obs.header.stamp
                    action_started = time.monotonic()
                    try:
                        applied=call(control,req,4.)
                    except TimeoutError as exc:
                        record_stage('action_ack', 'retry', action_started, str(exc), action_failures+1)
                        recover('action_stop', lambda: stop(3.))
                        applied = TrainingControl.Response(reason=str(exc))
                    if not applied.success:
                        action_failures += 1
                        if action_failures >= args.recovery_attempts:
                            hold_for_operator('action_ack', applied.reason)
                        obs = feasible_observation(path, message_stamp(obs.header))
                        if not obs.path_valid:
                            outcome = 'collision' if obs.collision else 'planning_timeout'
                            penalty = -20 if obs.collision else -10
                            components['event'] += penalty
                            total += penalty
                        previous = math.hypot(obs.pose.position.x-gx,obs.pose.position.y-gy)
                        continue
                    action_failures = 0
                    then=stamp()
                    recover('simulation_clock', lambda: advance(args.period))
                    nxt=feasible_observation(path, message_stamp(obs.header))
                    now=stamp()
                    dt=now-then
                    if dt<=0:
                        raise RuntimeError('nonpositive simulated duration')
                    dist=math.hypot(nxt.pose.position.x-gx,nxt.pose.position.y-gy)
                    lateral,herr,path_progress=tracking(path,nxt.pose)
                    roll,pitch,_=yaw(nxt.pose.orientation)
                    collision=bool(applied.collision or nxt.collision)
                    tilt=abs(roll)>args.roll_limit or abs(pitch)>args.pitch_limit
                    success=dist<=args.goal_tolerance
                    timeout=now-start>=args.timeout
                    outcome='collision' if collision else 'tilt' if tilt else 'success' if success else 'timeout' if timeout else 'planning_timeout' if not nxt.path_valid else ''
                    if path_progress >= progress_anchor + args.progress_min_distance:
                        progress_anchor = path_progress
                        progress_since = now
                    if not outcome and now-progress_since >= args.progress_timeout:
                        outcome = 'planning_timeout'
                        detail = dict(episode=episode, step=global_step+1, sim_time=now, reason='no_path_progress',
                                      window_sim_s=now-progress_since, progress_m=path_progress,
                                      progress_anchor_m=progress_anchor, distance_m=dist,
                                      safe_linear_mps=applied.command.linear.x,
                                      body_linear_mps=nxt.velocity.linear.x,
                                      prior_reason=nxt.reason)
                        append_record(directory/'control_events.jsonl', detail)
                        log(f'回合 {episode} 无路径进展 {now-progress_since:.1f} 仿真秒，'
                            f'结束当前回合 distance={dist:.2f}m safe_v={applied.command.linear.x:.3f}m/s '
                            f'prior={nxt.reason or "无附加诊断"}')
                    pieces=dict(progress=2*(previous-dist),time=-.02*dt,tracking=-.04*dt*lateral**2,event=(8 if outcome=='success' else -20 if outcome in ('collision','tilt') else -10 if outcome in ('timeout','planning_timeout') else 0))
                    reward=sum(pieces.values())
                    for key in components:
                        components[key]+=pieces[key]
                    speed=float(nxt.velocity.linear.x)
                    angular=float(nxt.velocity.angular.z)
                    change=abs(speed-last_speed) if last_speed is not None else 0.
                    last_speed=speed
                    sign=1 if angular>args.yaw_deadband else -1 if angular< -args.yaw_deadband else 0
                    oscillation=int(sign!=0 and last_sign!=0 and sign!=last_sign)
                    amplitude = 0.5 * abs(angular - last_angular) if oscillation else 0.0
                    if sign:
                        last_sign = sign
                        last_angular = angular
                    changes.append(change)
                    oscillations.append(oscillation)
                    amplitudes.append(amplitude)
                    if entropy is not None:
                        entropies.append(entropy)
                    losses=None
                    if args.mode=='train':
                        if migration:
                            append_record(directory/'scene_changes.jsonl', migration)
                            log(f"新场景已启用，首个交互步 {global_step+1}")
                            migration = None
                        replay.add(o,prior,action,reward,nxt.observation,nxt.prior,bool(outcome))
                        if len(replay)>=max(args.batch,args.warmup):
                            losses=agent.update(replay,args.batch)
                    sw.writerow(dict(episode=episode,step=global_step+1,episode_step=steps,sim_time=now,dt=dt,reward=reward,distance=dist,lateral_error=lateral,heading_error=herr,roll=roll,pitch=pitch,action_v=action[0],action_w=action[1],prior_v=prior[0],prior_w=prior[1],collision=collision,entropy_sample=entropy if entropy is not None else '',reward_progress=pieces['progress'],reward_time=pieces['time'],reward_tracking=pieces['tracking'],reward_event=pieces['event'],yaw_oscillation=oscillation,yaw_oscillation_amplitude=amplitude,speed_change=change))
                    total+=reward
                    lat.append(lateral)
                    heading.append(abs(herr))
                    steps+=1
                    global_step+=1
                    metrics_writer.step(global_step, entropy=entropy, losses=losses)
                    if global_step % args.log_every_steps == 0 or outcome:
                        loss_text = ''
                        if losses is not None:
                            loss_text = (f" critic={losses['critic']:.4f} actor={losses['actor']:.4f}"
                                         f" alpha={losses['alpha']:.4f}")
                        log(f'回合={episode} 步={global_step} sim={now:.1f}s '
                            f'请求动作归一化(v,w)=({action[0]:.3f},{action[1]:.3f}) '
                            f'安全后指令(v,w)=({applied.command.linear.x:.3f}m/s,{applied.command.angular.z:.3f}rad/s) '
                            f'驱动反馈v={nxt.command.linear.x:.3f}m/s '
                            f'实测(vx,vy)=({nxt.velocity.linear.x:.3f},{nxt.velocity.linear.y:.3f})m/s '
                            f'roll={roll:.3f}rad pitch={pitch:.3f}rad 距离={dist:.2f}m reward={reward:.3f}'
                            + loss_text)
                    previous=dist
                    obs=nxt
                    if global_step % args.record_every_steps==0:
                        sf.flush()
                    if args.mode=='train' and global_step-last_checkpoint_step>=args.checkpoint_every_steps:
                        last_checkpoint_step=global_step
                        if not outcome:
                            stop()
                        save_progress(episode,'periodic',partial=True)
                        if not outcome:
                            obs=feasible_observation(path)
                            if not obs.path_valid:
                                outcome = 'collision' if obs.collision else 'planning_timeout'
                            previous=math.hypot(obs.pose.position.x-gx,obs.pose.position.y-gy)
                    if args.mode=='train' and global_step>=args.max_steps and not outcome:
                        outcome='maximum_steps'
                    if outcome:
                        break
            except ExternalShutdownException:
                raise
            except Exception as exc:
                log(f'回合 {episode} 异常: {exc!r}')
                errors.write(json.dumps({'episode':episode,'sim_time':stamp(),'error':repr(exc)},ensure_ascii=False)+'\n')
                errors.flush()
                outcome = ('collision' if isinstance(exc, SpawnCollision) else
                           'planning_timeout' if isinstance(exc, PlanningTimeout) else 'error')
                if outcome in ('collision', 'planning_timeout'):
                    penalty = -20 if outcome == 'collision' else -10
                    components['event'] += penalty
                    total += penalty
            finally:
                try:
                    stop()
                except Exception as exc:
                    log(f'回合 {episode} 停车失败: {exc!r}')
                    errors.write(json.dumps({'episode':episode,'stop_error':repr(exc)})+'\n')
                    errors.flush()
                    outcome='error'
            duration = 0. if start is None else max(0., stamp()-start)
            row=dict(episode=episode,seed=args.seed+episode,goal_x=gx,goal_y=gy,goal_z=gz,outcome=outcome,duration=duration,steps=steps,**{'return':total},mean_lateral_error=float(np.mean(lat)) if lat else '',max_lateral_error=max(lat) if lat else '',mean_abs_heading_error=float(np.mean(heading)) if heading else '',entropy_mean=float(np.mean(entropies)) if entropies else '',yaw_oscillation_rate=float(np.mean(oscillations)) if oscillations else '',yaw_oscillation_amplitude=float(np.mean(amplitudes)) if amplitudes else '',speed_change_mean=float(np.mean(changes)) if changes else '',reward_progress=components['progress'],reward_time=components['time'],reward_tracking=components['tracking'],reward_event=components['event'])
            log(f'回合 {episode} 结束 outcome={outcome} steps={steps} return={total:.3f}')
            ew.writerow(row)
            metrics_writer.episode(global_step, row)
            ef.flush()
            outcomes.append(row)
            episode_complete=True
            valid=[x for x in outcomes if x['outcome'] in ('success','collision','tilt','timeout','planning_timeout')]
            policy_valid=[x for x in valid if x['steps']>0]
            def mean_field(key):
                """! @brief 计算具备有效动作样本的回合均值。"""
                vals=[x[key] for x in policy_valid if x[key]!='']
                return float(np.mean(vals)) if len(vals)==len(policy_valid) and vals else None
            metrics={'episodes':len(outcomes),'valid':len(valid),'errors':len(outcomes)-len(valid),
                     'complete':len(outcomes)==args.episodes and len(valid)==args.episodes,
                     'success_rate':sum(x['outcome']=='success' for x in valid)/len(valid) if valid else None,
                     'collision_rate':sum(x['outcome']=='collision' for x in valid)/len(valid) if valid else None,
                     'timeout_rate':sum(x['outcome'] in ('timeout','planning_timeout') for x in valid)/len(valid) if valid else None,
                     'planning_timeout_rate':sum(x['outcome']=='planning_timeout' for x in valid)/len(valid) if valid else None,
                     'policy_valid':len(policy_valid),
                     'reward_mean':mean_field('return'),'entropy_mean':mean_field('entropy_mean'),
                     'yaw_oscillation_rate':mean_field('yaw_oscillation_rate'),'yaw_oscillation_amplitude':mean_field('yaw_oscillation_amplitude'),'speed_change_mean':mean_field('speed_change_mean'),
                     'mean_lateral_error':mean_field('mean_lateral_error'),
                     'reward_components':{k:mean_field('reward_'+k) for k in ('progress','time','tracking','event')}}
            (directory/'metrics.json').write_text(json.dumps(metrics,indent=2))
            metrics_writer.rates(global_step, metrics)
            if outcome in ('error','maximum_steps'):
                if args.mode=='train':
                    save_progress(episode,'maximum_steps' if outcome=='maximum_steps' else 'runtime_error',partial=True)
                break
            if args.mode=='train':
                if global_step>=args.max_steps:
                    save_progress(episode,'maximum_steps',partial=False)
                    break
                if global_step-last_eval_step>=args.eval_every_steps:
                    last_eval_step=global_step
                    history.append(evaluate_snapshot(episode))
                    metrics_writer.rates(global_step, history[-1], prefix='evaluation')
                    gates=convergence(history,window=args.convergence_window,min_success=args.gate_success,max_collision=args.gate_collision,
                        reward_relative=args.reward_relative,success_spread=args.success_spread,entropy_drop=args.entropy_drop,
                        entropy_spread=args.entropy_spread,entropy_jump=args.entropy_jump,max_oscillation=args.max_oscillation,
                        max_speed_change=args.max_speed_change,collision_spread=args.collision_spread,
                        oscillation_spread=args.oscillation_spread,speed_change_spread=args.speed_change_spread,
                        reward_trend=args.reward_trend,quality_trend_fraction=args.quality_trend_fraction,
                        entropy_upper_bound=args.entropy_upper_bound)
                    history[-1]['gates']=gates
                    (directory/'evaluations.json').write_text(json.dumps({'history':history,'gates':gates},indent=2))
                    if gates['converged']:
                        stop_reason='converged'
                if stop_reason=='converged' or episode==first+args.episodes-1:
                    save_progress(episode,stop_reason)
                if stop_reason=='converged':
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        stop_reason='manual_interrupt'
        log('收到中断，正在停车并保存状态')
        try:
            stop()
        except Exception:
            pass
        if args.mode=='train':
            save_progress(episode,stop_reason,partial=not episode_complete)
    finally:
        log('运行结束，正在停车并关闭记录')
        try:
            stop()
        except Exception:
            pass
        sf.close()
        ef.close()
        errors.close()
        metrics_writer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
