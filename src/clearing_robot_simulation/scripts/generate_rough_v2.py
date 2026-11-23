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
# 地形中的平滑深坑和堆体，排斥范围供采样器参考。
PITS=[(-8.,29.,3.9,4.5),(25.,-30.,4.2,4.8),(60.,30.,4.0,4.5),(82.,-29.,3.7,4.2)]
MOUNDS=[(4,21,1.1,2.1),(15,-20,1.,1.8),(26,20,1.2,2.),(37,-20,1.1,2.2),(48,20,1.3,2.1),(59,-20,1.,1.9),(70,20,1.2,2.),(81,-19,1.,2.),(-15,-24,1.1,2.),(-4,40,1.,1.8),(11,-41,1.2,2.),(38,40,1.3,2.2),(51,-40,1.,1.9),(72,41,1.1,2.2),(87,35,1.2,2.1),(91,-41,1.1,2.)]
def height(x,y,scene):
    """! @brief 平滑滚动地形叠加自然融合的深坑和散布堆体。"""
    u=np.clip((x-7.)/15.,0.,1.);fade=u*u*(3.-2.*u)
    z=fade*(1.35*np.sin((x-13.)/18.)+.30*np.sin((x-13.)/8.8)+.22*np.sin(y/14.+x/27.))
    for cx,cy,depth,sigma in PITS: z=z-depth*np.exp(-((x-cx)**2+(y-cy)**2)/(2*sigma*sigma))
    for cx,cy,h,sigma in MOUNDS: z=z+h*np.exp(-((x-cx)**2+(y-cy)**2)/(2*sigma*sigma))
    return z
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
    """! @brief 生成远离主路线的施工地标及堆放箱体。"""
    raw=[(-12,7,1.6,2,3.8,.4),(-3,-10,3,1.5,3.,-.3),(7,-15,2.5,2,2.8,.1),(14,31,4,3,3.2,.2),(22,-19,3,2.5,4.5,-.3),(32,14,3,3,4.,.15),(40,-27,4,2,2.8,.4),(48,29,3,3,5.,-.2),(55,-15,2,3,3.8,.1),(66,16,3,2,4.2,-.3),(74,-25,3,3,4.3,.2),(84,16,3.5,2.5,5.,.3),(-17,16,2,2,3.4,.2),(2,-35,4,3,2.6,-.4),(33,42,4,3,2.9,.1),(53,-32,2,2,4.8,-.1),(77,36,4,2,3.,.2),(89,-12,3,3,4.4,-.2),(5,44,4,2,2.5,.2),(91,8,2,3,3.6,.1)]
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
    # 数组与 STL 顶点同源；保守矩形标注高斯坑和堆体范围。
    records=[{'x':center[0],'y':center[1],'length':size[0],'width':size[1],'yaw':yaw} for _,center,size,yaw in obstacles(scene,vertices)]
    records += [{'x':x,'y':y,'length':5*sigma,'width':5*sigma,'yaw':0.0} for x,y,_,sigma in PITS]
    records += [{'x':x,'y':y,'length':4*sigma,'width':4*sigma,'yaw':0.0} for x,y,_,sigma in MOUNDS]
    metadata={'scene':scene,'origin':[XMIN,YMIN],'resolution':GRID,'elevation':vertices[:,:,2].tolist(),'obstacles':records}
    (OUTPUT/f'{scene}_sampling.json').write_text(json.dumps(metadata,separators=(',',':'))+'\n')
if __name__=='__main__':
    make('training_rough_v2')
