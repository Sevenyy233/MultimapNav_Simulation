#!/usr/bin/env python3
"""! @file 由同一三角高程网格与碰撞盒生成连续地形、Gazebo 场景和先验点云。"""
import json, math, struct
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
OUTPUT=Path(__file__).resolve().parents[1]/'worlds'
GRID=1.; SAMPLE=.25
XMIN,XMAX,YMIN,YMAX=-25.,95.,-50.,50.
def node(parent,tag,text=None,**attrs):
    """! @brief 添加 SDF 元素。"""
    child=ET.SubElement(parent,tag,attrs)
    if text is not None: child.text=str(text)
    return child
def height(x,y,scene):
    """! @brief 平坦起点经平滑过渡进入具有不同相位的起伏地形。"""
    phase=0. if scene=='training' else .72
    u=np.clip((x-7.)/15.,0.,1.);fade=u*u*(3.-2.*u)
    amplitude=1. if scene=='training' else .83
    return amplitude*fade*(1.35*np.sin((x-13.)/18.+phase)+.30*np.sin((x-13.)/8.8+phase*.4)+.22*np.sin(y/14.+x/27.+phase))
def terrain(scene):
    """! @brief 返回实际以 float32 顶点写入 STL 的网格。"""
    xs=np.arange(XMIN,XMAX+GRID/2,GRID);ys=np.arange(YMIN,YMAX+GRID/2,GRID)
    xx,yy=np.meshgrid(xs,ys,indexing='xy');vertices=np.stack((xx,yy,height(xx,yy,scene)),axis=-1).astype('<f4')
    p00,p10=vertices[:-1,:-1],vertices[:-1,1:];p01,p11=vertices[1:,:-1],vertices[1:,1:]
    return vertices,np.stack((np.stack((p00,p10,p11),axis=-2),np.stack((p00,p11,p01),axis=-2)),axis=-3).reshape(-1,3,3)
def surface(x,y,vertices):
    """! @brief 使用 STL 的同一对角线按三角形重心坐标插值。"""
    x,y=np.asarray(x),np.asarray(y);gx=np.clip((x-XMIN)/GRID,0,vertices.shape[1]-1-1e-10);gy=np.clip((y-YMIN)/GRID,0,vertices.shape[0]-1-1e-10)
    ix,iy=np.floor(gx).astype(int),np.floor(gy).astype(int);u,v=gx-ix,gy-iy
    z00=vertices[iy,ix,2];z10=vertices[iy,ix+1,2];z01=vertices[iy+1,ix,2];z11=vertices[iy+1,ix+1,2]
    return np.where(v<=u,z00+(z10-z00)*u+(z11-z10)*v,z00+(z11-z01)*u+(z01-z00)*v)
def save_stl(path,triangles):
    """! @brief 写入与点云采样完全相同的二进制 STL 三角形。"""
    with path.open('wb') as f:
        f.write(b'clearing rolling terrain'.ljust(80,b'\0'));f.write(struct.pack('<I',len(triangles)))
        for tri in triangles:
            normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);normal/=np.linalg.norm(normal)
            f.write(struct.pack('<12fH',*normal,*tri.reshape(-1),0))
def face(size,axis,side):
    """! @brief 以固定间距采样碰撞盒外表面。"""
    dims=[i for i in range(3) if i!=axis];a,b=np.meshgrid(*[np.linspace(-size[d]/2,size[d]/2,math.ceil(size[d]/SAMPLE)+1) for d in dims]);p=np.zeros((a.size,3));p[:,axis]=side*size[axis]/2;p[:,dims[0]]=a.ravel();p[:,dims[1]]=b.ravel();return p
def obstacles(scene,vertices):
    """! @brief 返回避开中部路线的地标、堆体与高障碍。"""
    if scene=='training':
        raw=[(-11,5.8,1.4,2.,3.6,.42),(-2,-9,3.2,1.25,2.7,-.35),(12,17,4,3,2.2,.2),(21,-19,3,4,4.8,-.3),(31,17,5,3,1.3,.15),(43,-19,3,3,5,.3),(55,18,5,3,2.2,-.2),(67,-17,3.5,4,4.,.1),(79,20,5,4,1.7,-.25),(87,-22,4,3,5.2,.3),(38,29,6,4,1.,.3)]
    else:
        raw=[(-12,-6,1.8,1.5,4.,-.2),(-2,9.5,2.7,1.4,3.2,.3),(13,-18,3,4,3.6,-.2),(23,20,4,3,1.5,.4),(34,-18,3,3.5,5.1,-.15),(46,18,4,4,2.4,.1),(57,-19,5,3,1.4,.2),(69,18,3,3,5.4,-.3),(81,-20,4,5,2.,.1),(88,24,4,3,4.3,-.2),(35,-30,5,4,1.1,-.3)]
    boxes=[]
    for i,(x,y,sx,sy,sz,yaw) in enumerate(raw):
        radius=math.hypot(sx,sy)/2;support=max(float(surface(x+dx*radius,y+dy*radius,vertices)) for dx,dy in ((-1,0),(1,0),(0,-1),(0,1),(0,0)))
        boxes.append((f'landmark_{i:02d}',(x,y,support+sz/2-.06),(sx,sy,sz),yaw))
    boxes += [('wall_w',(-25.25,0,2.5),(.5,101,9),0),('wall_e',(95.25,0,2.5),(.5,101,9),0),('wall_s',(35,-50.25,2.5),(120,.5,9),0),('wall_n',(35,50.25,2.5),(120,.5,9),0)]
    return boxes
def make(scene):
    """! @brief 生成场景、配套 PCD、三角网和带地形高度的目标清单。"""
    vertices,triangles=terrain(scene);mesh_dir=OUTPUT/'terrain_mesh';mesh_dir.mkdir(parents=True,exist_ok=True);save_stl(mesh_dir/f'{scene}.stl',triangles)
    root=ET.Element('sdf',version='1.6');world=node(root,'world',name=scene);node(world,'gravity','0 0 -9.81');physics=node(world,'physics',name='ode',type='ode');node(physics,'max_step_size','0.001');node(physics,'real_time_update_rate','1000')
    ode=node(physics,'ode');solver=node(ode,'solver');node(solver,'type','quick');node(solver,'iters','150');node(solver,'sor','1.3');constraints=node(ode,'constraints');node(constraints,'cfm','0');node(constraints,'erp','.2');node(constraints,'contact_max_correcting_vel','.1');node(constraints,'contact_surface_layer','.001')
    plugin=node(world,'plugin',name='gazebo_ros_state',filename='libgazebo_ros_state.so');node(node(plugin,'ros'),'namespace','/gazebo');node(world,'plugin',name='clearing_robot_contacts',filename='libclearing_robot_contacts.so');node(node(world,'include'),'uri','model://sun')
    ground=node(world,'model',name='ground_terrain');node(ground,'static','true');link=node(ground,'link',name='body')
    for kind in ('collision','visual'):
        mesh=node(node(node(link,kind,name=kind),'geometry'),'mesh');node(mesh,'uri',f'file://worlds/terrain_mesh/{scene}.stl')
    xs=np.arange(XMIN,XMAX+SAMPLE/2,SAMPLE);ys=np.arange(YMIN,YMAX+SAMPLE/2,SAMPLE);xx,yy=np.meshgrid(xs,ys);arrays=[np.column_stack((xx.ravel(),yy.ravel(),surface(xx,yy,vertices).ravel()))]
    for name,center,size,yaw in obstacles(scene,vertices):
        model=node(world,'model',name=name);node(model,'static','true');node(model,'pose',f'{center[0]} {center[1]} {center[2]} 0 0 {yaw}');link=node(model,'link',name='body')
        for kind in ('collision','visual'):node(node(node(node(link,kind,name=kind),'geometry'),'box'),'size',' '.join(map(str,size)))
        c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        for axis in range(3):
            for side in (-1,1):
                if axis==2 and side==-1: continue
                sampled=face(size,axis,side)@R.T+center;arrays.append(sampled[sampled[:,2]>=surface(sampled[:,0],sampled[:,1],vertices)-1e-5])
    ET.indent(root);ET.ElementTree(root).write(OUTPUT/f'{scene}.world',encoding='utf-8',xml_declaration=True)
    points=np.unique(np.round(np.concatenate(arrays),5),axis=0).astype('<f4')
    with (OUTPUT/f'{scene}.pcd').open('wb') as f:
        f.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'+f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode());f.write(points.tobytes())
    def goal(x,y):return {'x':x,'y':y,'z':round(float(surface(x,y,vertices)),5)}
    goals=[goal(x,y) for x in (30,36,44,52) for y in (-8,0,8)]
    routes=[{'name':f'center_to_{y:+d}','points':[goal(x,round(y*min(1,max(0,(x-10)/30)),2)) for x in (0,10,20,30,40,52,65,75)]} for y in (-8,0,8)]
    (OUTPUT/f'{scene}_goals.json').write_text(json.dumps({'scene':scene,'frame_id':'map','spawn':goal(0,0),'goals':goals,'routes':routes},ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':
    make('training');make('evaluation')
