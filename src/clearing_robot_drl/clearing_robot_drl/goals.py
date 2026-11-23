"""按场景读取地形目标，或在显式矩形中采样。"""
import json
import math
from pathlib import Path


def load_scene_goals(scene, source='scene'):
    """! @brief 从已安装场景或自定义 JSON 读取三维目标。"""
    if source == 'scene':
        if scene not in ('training', 'evaluation'):
            raise ValueError('自定义场景须通过 --goal-source 提供目标 JSON 或指定 rectangle')
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('clearing_robot_simulation')) / 'worlds' / (scene + '_goals.json')
    else:
        path = Path(source).expanduser()
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('目标 JSON 必须是对象')
    if source == 'scene' and data.get('scene', scene) != scene:
        raise ValueError('目标文件和场景名称不匹配')
    frame = data.get('frame_id', data.get('frame', 'map'))
    if frame not in ('map', 'world'):
        raise ValueError('目标文件坐标系须为 map 或与其原点、轴一致的 world')
    goals = data.get('goals')
    if not isinstance(goals, list) or not goals:
        raise ValueError('目标文件必须包含非空 goals 数组')
    result = []
    for goal in goals:
        if not isinstance(goal, dict):
            raise ValueError('每个目标必须是含 x、y、z 的对象')
        try:
            xyz = tuple(float(goal[k]) for k in ('x', 'y', 'z'))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('每个目标均须包含有限的 x、y、z') from exc
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError('目标坐标必须为有限值')
        result.append(xyz)
    return result, str(path.resolve())


def candidates(rng, goals=None, bounds=None, limit=100):
    """! @brief 按随机源生成可复现候选；列表模式每轮无重复。"""
    if goals is not None:
        if not goals:
            raise ValueError('目标列表为空')
        count = 0
        while count < limit:
            for index in rng.permutation(len(goals)):
                yield goals[int(index)]
                count += 1
                if count >= limit:
                    return
    else:
        if bounds is None:
            raise ValueError('矩形采样缺少边界')
        xmin, xmax, ymin, ymax, z = bounds
        if not all(math.isfinite(v) for v in bounds) or xmin >= xmax or ymin >= ymax:
            raise ValueError('目标矩形边界无效')
        for _ in range(limit):
            yield float(rng.uniform(xmin, xmax)), float(rng.uniform(ymin, ymax)), z


def evaluation_goal_arguments(source, rectangle):
    """! @brief 默认不向独立评估透传训练场景的矩形目标。"""
    args = ['--goal-source', source]
    if source == 'rectangle':
        names = ('goal-x-min', 'goal-x-max', 'goal-y-min', 'goal-y-max', 'goal-z')
        for name, value in zip(names, rectangle):
            args.extend(['--' + name, str(value)])
    return args
