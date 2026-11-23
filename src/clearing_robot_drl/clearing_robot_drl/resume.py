"""检查点选择与运行目录的续训记录。"""
import csv
import hashlib
import json
from pathlib import Path
import re


def _valid_run_id(run_id):
    """! @brief 防止运行标识被解释为目录遍历或通配符。"""
    if not run_id or not re.fullmatch(r'[A-Za-z0-9_.-]+',run_id) or run_id in ('.','..'):
        raise ValueError('需要合法的 --run-id')
    return run_id


def _checkpoint_candidates(directory, run_id):
    """! @brief 查找指定运行 ID 对应的所有命名检查点。"""
    run_id=_valid_run_id(run_id)
    # 新命名允许在运行 ID 前加实验、场景前缀；ID 左侧必须是分隔符或文件开头。
    pattern=re.compile(r'(?:^|-)'+re.escape(run_id)+r'-step-(\d+)-episode-(\d+)-.+-checkpoint\.pt$')
    root=Path(directory)
    if not root.is_dir():
        return []
    matches=[]
    for path in root.rglob('*-checkpoint.pt'):
        match=pattern.search(path.name)
        if match and path.is_file():
            matches.append((int(match[1]),path.stat().st_mtime_ns,str(path),path))
    return matches


def latest_checkpoint(models, run_id):
    """! @brief 从旧模型目录或嵌套新布局中选择指定运行的最新检查点。"""
    matches=_checkpoint_candidates(models,run_id)
    if not matches:
        raise FileNotFoundError(f'运行 {run_id} 没有已落盘检查点')
    return max(matches)[-1]


def resolve_resume(selection, models_root=None, run_id=None, records_root=None):
    """! @brief 接受检查点、运行目录或 latest，兼容旧平铺模型和新目录。
    @param selection 指向检查点文件、运行目录或字符串 latest。
    @param models_root 旧平铺模型目录，可选。
    @param run_id latest 选择时的运行标识；目录模式可自动读取。
    @param records_root 新运行所在根目录，用于 latest 跨目录查找。
    @return 选中的 checkpoint 路径，不修改任何文件。
    """
    from .artifacts import RunLayout
    if str(selection)=='latest':
        _valid_run_id(run_id)
        matches=[]
        for root in (models_root,records_root):
            if root is not None:
                matches.extend(_checkpoint_candidates(root,run_id))
        if not matches:
            raise FileNotFoundError(f'运行 {run_id} 没有已落盘检查点')
        return max(matches)[-1]
    path=Path(selection).expanduser()
    if path.is_file():
        if not path.name.endswith('-checkpoint.pt'):
            raise ValueError('恢复路径必须指向 checkpoint 文件')
        return path
    layout=RunLayout.open(path)
    metadata=json.loads(layout.run_json.read_text(encoding='utf-8'))
    id_in_file=_valid_run_id(metadata.get('run_id'))
    if run_id and id_in_file!=run_id:
        raise ValueError('指定的运行 ID 与目录元数据不匹配')
    matches=_checkpoint_candidates(layout.checkpoints,id_in_file)
    if models_root is not None:
        matches.extend(_checkpoint_candidates(models_root,id_in_file))
    # 老运行可通过 stop.json 引用平铺模型，不要求调用方显式指定旧模型目录。
    stop=layout.records/'stop.json'
    if stop.is_file():
        pointer=json.loads(stop.read_text(encoding='utf-8')).get('checkpoint')
        if pointer:
            pointed=Path(pointer).expanduser()
            if pointed.is_file():
                matches.extend(item for item in _checkpoint_candidates(pointed.parent,id_in_file) if item[-1]==pointed)
    if not matches:
        raise FileNotFoundError(f'运行 {id_in_file} 没有已落盘检查点')
    return max(matches)[-1]



def locate_run_directory(checkpoint, records_root, run_id):
    """! @brief 根据检查点和身份定位唯一运行目录，拒绝含糊的选择。"""
    from .artifacts import RunLayout
    _valid_run_id(run_id)
    checkpoint=Path(checkpoint).expanduser()
    candidates=[]
    # 新布局的检查点自身携带父运行目录；优先使用此明确归属。
    if checkpoint.parent.name=='checkpoints':
        try:
            layout=RunLayout.open(checkpoint.parent.parent)
            data=json.loads(layout.run_json.read_text(encoding='utf-8'))
            if data.get('run_id')==run_id:
                return layout.root
        except (ValueError,FileNotFoundError):
            pass
    root=Path(records_root)
    if root.is_dir():
        for child in root.iterdir():
            if not child.is_dir():
                continue
            try:
                layout=RunLayout.open(child)
                data=json.loads(layout.run_json.read_text(encoding='utf-8'))
                if data.get('run_id')==run_id:
                    candidates.append(layout.root)
            except (ValueError,FileNotFoundError,json.JSONDecodeError):
                continue
    if len(candidates)!=1:
        raise ValueError(f'无法唯一定位运行 {run_id} 的记录目录（匹配 {len(candidates)} 个）')
    return candidates[0]


def previous_run(directory, extra, step):
    """! @brief 验证运行身份和全部已有步骤记录的高水位，防止旧检查点追加。"""
    directory=Path(directory)
    candidates=(directory/'run.json',directory/'parameters'/'run.json',directory.parent/'parameters'/'run.json')
    meta_path=next((path for path in candidates if path.is_file()),None)
    if meta_path is None:
        raise ValueError(f'续训缺少原运行配置：{directory}')
    previous=json.loads(meta_path.read_text(encoding='utf-8'))
    if previous.get('run_id') != extra.get('run_id'):
        raise ValueError('检查点和原运行目录身份不匹配')
    # 同时覆盖旧根目录、新 records 子目录，以及调用方传入 records 子目录的形式。
    traces={directory/'steps.csv',directory/'records'/'steps.csv'}
    if directory.name=='records':
        traces.add(directory.parent/'steps.csv')
    last=0
    for trace in traces:
        if trace.is_file():
            with trace.open(newline='',encoding='utf-8') as stream:
                last=max(last,max((int(row['step']) for row in csv.DictReader(stream) if row.get('step')),default=0))
    if last>step:
        raise ValueError(f'现有步日志已经到 {last}，检查点仅到 {step}；请选择更晚的检查点')
    return previous


def scene_digest(scene):
    """! @brief 给已安装的世界、先验地图和采样网格生成稳定版本标识。"""
    from ament_index_python.packages import get_package_share_directory
    worlds = Path(get_package_share_directory('clearing_robot_simulation')) / 'worlds'
    result = hashlib.sha256()
    for relative in (f'{scene}.world', f'{scene}.pcd', f'terrain_mesh/{scene}.stl', f'{scene}_sampling.json'):
        path = worlds / relative
        if path.is_file():
            result.update(relative.encode())
            with path.open('rb') as source:
                for chunk in iter(lambda: source.read(1 << 20), b''):
                    result.update(chunk)
    return result.hexdigest()


def append_record(path, item):
    """! @brief 以 JSON Lines 追加一条审计记录。"""
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(item, ensure_ascii=False) + '\n')


def preserve_summary(directory, step):
    """! @brief 在更新当前会话的摘要前保留既有快照。"""
    for name in ('metrics', 'stop', 'evaluations'):
        previous = directory / f'{name}.json'
        if previous.is_file():
            snapshot = directory / f'{name}-before-resume-{step:012d}.json'
            if not snapshot.exists():
                with snapshot.open('xb') as out:
                    out.write(previous.read_bytes())
