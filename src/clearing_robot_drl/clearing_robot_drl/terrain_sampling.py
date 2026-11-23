"""利用地形三角网及障碍物进行连续随机起终点采样。"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class SamplingLimits:
    """! @brief 车体轮廓、地形安全余量和采样预算。角度单位为弧度。"""
    length: float = 9.9
    width: float = 3.9
    forward_offset: float = 1.3
    obstacle_margin: float = 0.25
    boundary_margin: float = 0.25
    turning_radius: float = 4.2
    max_slope: float = 0.30
    max_step: float = 0.30
    max_plane_residual: float = 0.30
    sample_spacing: float = 0.8
    attempts_per_sample: int = 5000


class TerrainSampler:
    """! @brief 基于车身朝向、STL 三角插值和静态障碍物筛选候选位姿。

    @note 返回的 z 为车体原点位置的地表高度；调用方需另加模型原点离地偏移。
    几何筛选不证明路径连通，必须再由全局规划器验证。
    """
    def __init__(self, data, limits=None):
        self.limits = limits or SamplingLimits()
        self.scene = str(data['scene'])
        self.origin = np.asarray(data['origin'], dtype=float)
        self.resolution = float(data['resolution'])
        self.elevation = np.asarray(data['elevation'], dtype=float)
        c = self.limits
        if (self.origin.shape != (2,) or not np.isfinite(self.origin).all() or
                not math.isfinite(self.resolution) or self.resolution <= 0 or
                self.elevation.ndim != 2 or min(self.elevation.shape) < 2 or
                not np.isfinite(self.elevation).all()):
            raise ValueError('无效地形网格')
        if (not all(math.isfinite(v) for v in (c.length,c.width,c.forward_offset,c.obstacle_margin,
                c.boundary_margin,c.turning_radius,c.max_slope,c.max_step,c.max_plane_residual,c.sample_spacing)) or
                min(c.length,c.width,c.turning_radius,c.max_slope,c.max_step,c.max_plane_residual,c.sample_spacing) <= 0 or
                min(c.obstacle_margin,c.boundary_margin) < 0 or c.attempts_per_sample < 1 or c.max_slope >= math.pi/2):
            raise ValueError('无效采样约束')
        self.obstacles=[]
        for item in data.get('obstacles', []):
            ob=tuple(float(item[k]) for k in ('x','y','length','width','yaw'))
            if not all(math.isfinite(v) for v in ob) or ob[2] <= 0 or ob[3] <= 0:
                raise ValueError('无效障碍物轮廓')
            self.obstacles.append(ob)
        self.upper = self.origin + self.resolution * np.array([self.elevation.shape[1]-1, self.elevation.shape[0]-1])
        xs=np.linspace(c.forward_offset-c.length/2,c.forward_offset+c.length/2,max(2,math.ceil(c.length/c.sample_spacing)+1))
        ys=np.linspace(-c.width/2,c.width/2,max(2,math.ceil(c.width/c.sample_spacing)+1))
        self.local_x,self.local_y=np.meshgrid(xs,ys)
        self.design=np.stack((self.local_x.ravel(),self.local_y.ravel(),np.ones(self.local_x.size)),axis=1)
        self.design_pinv=np.linalg.pinv(self.design)

    @classmethod
    def from_scene(cls, scene, limits=None):
        """! @brief 从 ROS 安装包读取与场景同名的采样网格。"""
        if not scene or Path(scene).name != scene:
            raise ValueError('scene 必须是单个场景名')
        from ament_index_python.packages import get_package_share_directory
        path=Path(get_package_share_directory('clearing_robot_simulation'))/'worlds'/f'{scene}_sampling.json'
        return cls.from_file(path,expected_scene=scene,limits=limits)

    @classmethod
    def from_file(cls, path, expected_scene=None, limits=None):
        """! @brief 加载自定义地形网格并检查场景名。"""
        data=json.loads(Path(path).read_text(encoding='utf-8'))
        if expected_scene is not None and data.get('scene') != expected_scene:
            raise ValueError('采样网格与所选场景不匹配')
        return cls(data,limits)

    def _height_and_slope(self, x, y):
        """! @brief 在单元格左下至右上对角线划分的两个三角面上插值。"""
        x=np.asarray(x,dtype=float); y=np.asarray(y,dtype=float)
        gx=(x-self.origin[0])/self.resolution; gy=(y-self.origin[1])/self.resolution
        ix=np.floor(gx).astype(int); iy=np.floor(gy).astype(int)
        if np.any(ix < 0) or np.any(iy < 0) or np.any(ix >= self.elevation.shape[1]-1) or np.any(iy >= self.elevation.shape[0]-1):
            raise ValueError('超出三角网范围')
        fx=gx-ix; fy=gy-iy
        z00=self.elevation[iy,ix]; z10=self.elevation[iy,ix+1]
        z01=self.elevation[iy+1,ix]; z11=self.elevation[iy+1,ix+1]
        low=fx>=fy
        dx=np.where(low,z10-z00,z11-z01)/self.resolution
        dy=np.where(low,z11-z10,z01-z00)/self.resolution
        height=np.where(low,z00+(z10-z00)*fx+(z11-z10)*fy,
                        z00+(z11-z01)*fx+(z01-z00)*fy)
        return height,np.hypot(dx,dy)

    def height_at(self,x,y):
        """! @brief 返回坐标对应三角面的高度，越界或非有限坐标抛出异常。"""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('无效地形坐标')
        height,_=self._height_and_slope(x,y)
        return float(height)

    def _clear_obstacles(self,x,y,yaw):
        """! @brief 对旋转矩形执行分离轴测试并检查 4.2 m 转弯圆区。"""
        cfg=self.limits
        ca,sa=math.cos(yaw),math.sin(yaw)
        ux=np.array([ca,sa]); uy=np.array([-sa,ca])
        center=np.array([x,y])+cfg.forward_offset*ux
        for ox,oy,length,width,theta in self.obstacles:
            co,so=math.cos(theta),math.sin(theta)
            vx=np.array([co,so]); vy=np.array([-so,co])
            delta=np.array([ox,oy])-center
            h1=length/2+cfg.obstacle_margin; h2=width/2+cfg.obstacle_margin
            separated=False
            for axis in (ux,uy,vx,vy):
                robot=cfg.length/2*abs(np.dot(ux,axis))+cfg.width/2*abs(np.dot(uy,axis))
                obstacle=h1*abs(np.dot(vx,axis))+h2*abs(np.dot(vy,axis))
                if abs(np.dot(delta,axis)) > robot+obstacle:
                    separated=True
                    break
            if not separated:
                return False
            base=np.array([x-ox,y-oy])
            distance=math.hypot(max(abs(np.dot(base,vx))-h1,0.),max(abs(np.dot(base,vy))-h2,0.))
            if distance <= cfg.turning_radius:
                return False
        return True

    def is_feasible(self,x,y,yaw):
        """! @brief 按边界、足迹、转弯空间和地形坡度检查单个位姿。"""
        if not all(math.isfinite(v) for v in (x,y,yaw)):
            return False
        c=self.limits
        xw=x+math.cos(yaw)*self.local_x-math.sin(yaw)*self.local_y
        yw=y+math.sin(yaw)*self.local_x+math.cos(yaw)*self.local_y
        margin=c.boundary_margin
        if (x-c.turning_radius < self.origin[0]+margin or x+c.turning_radius >= self.upper[0]-margin or
            y-c.turning_radius < self.origin[1]+margin or y+c.turning_radius >= self.upper[1]-margin or
            xw.min()<self.origin[0]+margin or xw.max()>=self.upper[0]-margin or
            yw.min()<self.origin[1]+margin or yw.max()>=self.upper[1]-margin):
            return False
        if not self._clear_obstacles(x,y,yaw):
            return False
        heights,grad=self._height_and_slope(xw,yw)
        if np.any(grad > math.tan(c.max_slope)):
            return False
        if (np.max(np.abs(np.diff(heights,axis=0)))>c.max_step or
                np.max(np.abs(np.diff(heights,axis=1)))>c.max_step):
            return False
        flat=heights.ravel()
        residual=flat-self.design @ (self.design_pinv @ flat)
        return bool(np.max(np.abs(residual))<=c.max_plane_residual)

    def _draw_xy(self,rng):
        """! @brief 在完整地图范围进行连续均匀采样。"""
        return (float(rng.uniform(self.origin[0],self.upper[0])),
                float(rng.uniform(self.origin[1],self.upper[1])))

    def sample_start(self,rng):
        """! @brief 返回连续随机的可行起点 (x,y,地表z,yaw)。"""
        for _ in range(self.limits.attempts_per_sample):
            x,y=self._draw_xy(rng); yaw=float(rng.uniform(-math.pi,math.pi))
            if self.is_feasible(x,y,yaw):
                z,_=self._height_and_slope(x,y)
                return x,y,float(z),yaw
        raise RuntimeError('无法在采样预算内找到符合约束的起点')

    def sample_goals(self,rng,start,min_distance,limit):
        """! @brief 产生全地图候选终点，航向由起点指向终点；A* 另行确认可达。"""
        if len(start)<2 or not all(math.isfinite(float(v)) for v in start[:2]):
            raise ValueError('无效起点')
        if not math.isfinite(min_distance) or min_distance<0 or not isinstance(limit,int) or limit<1:
            raise ValueError('无效终点距离或候选数量')
        found=0
        for _ in range(self.limits.attempts_per_sample * limit):
            x,y=self._draw_xy(rng)
            dx=x-start[0]; dy=y-start[1]
            if math.hypot(dx,dy)<min_distance:
                continue
            yaw=math.atan2(dy,dx)
            if not self.is_feasible(x,y,yaw):
                continue
            z,_=self._height_and_slope(x,y)
            yield x,y,float(z),yaw
            found+=1
            if found>=limit:
                return
        if not found:
            raise RuntimeError('无法在采样预算内找到符合约束的终点')
