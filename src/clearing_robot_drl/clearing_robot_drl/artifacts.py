"""运行目录、产物路径与可读文件名，不迁移或修改旧运行。"""
from dataclasses import dataclass
from pathlib import Path
import re
import uuid


def _slug(value):
    """! @brief 将标签规范化成可安全用于文件名的可读文本。"""
    result=re.sub(r'[^\w.-]+','-',str(value),flags=re.UNICODE).strip('.-_')
    if not result:
        raise ValueError('实验、场景及运行名不得为空')
    return result[:80]


@dataclass(frozen=True)
class RunLayout:
    """! @brief 新旧运行路径；旧记录继续保留在原有根目录。"""
    root: Path
    parameters: Path
    weights: Path
    checkpoints: Path
    records: Path
    run_json: Path
    legacy: bool

    @staticmethod
    def path_for(records_root, experiment, scene, run_id):
        """! @brief 计算新运行的目录位置，不创建文件。"""
        return Path(records_root) / '-'.join((_slug(experiment), _slug(scene), _slug(run_id)))

    @classmethod
    def create(cls, records_root, experiment, scene, run_id):
        """! @brief 创建全新实验目录，绝不覆盖已有运行。"""
        root=cls.path_for(records_root, experiment, scene, run_id)
        root.mkdir(parents=True,exist_ok=False)
        for name in ('parameters','weights','checkpoints','records'):
            (root/name).mkdir()
        return cls(root,root/'parameters',root/'weights',root/'checkpoints',root/'records',root/'parameters'/'run.json',False)

    @classmethod
    def open(cls, run_dir):
        """! @brief 只读识别新布局或旧根目录，不重命名、不迁移文件。"""
        root=Path(run_dir)
        if not root.is_dir():
            raise FileNotFoundError(f'运行目录不存在: {root}')
        if (root/'parameters'/'run.json').is_file():
            return cls(root,root/'parameters',root/'weights',root/'checkpoints',root/'records',root/'parameters'/'run.json',False)
        if (root/'run.json').is_file():
            return cls(root,root/'parameters',root/'weights',root/'checkpoints',root,root/'run.json',True)
        raise ValueError(f'目录中未找到运行元数据: {root}')

    def ensure_output_dirs(self):
        """! @brief 准备未来输出目录；旧记录和元数据保持原位置。"""
        for path in (self.parameters,self.weights,self.checkpoints,self.records):
            path.mkdir(parents=True,exist_ok=True)


def artifact_tag(experiment,scene,run_id,step,episode,reason):
    """! @brief 返回含实验、场景、步数及回合的唯一产物名。"""
    if step < 0 or episode < 0:
        raise ValueError('步数和回合必须非负')
    return (f'{_slug(experiment)}-{_slug(scene)}-{_slug(run_id)}-step-{step:012d}'
            f'-episode-{episode:06d}-{_slug(reason)}-{uuid.uuid4().hex[:8]}')
