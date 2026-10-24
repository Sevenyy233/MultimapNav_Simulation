#!/usr/bin/env python3
"""Build the development suite with Blender; sample PCDs using the package converter.
Run: blender -b --python this_file -- --package-dir PACKAGE [--maps NAME ...]
All distances are meters. Runtime mode selection still analyzes the point cloud.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
import bpy
import numpy as np
from mathutils import Vector
from mathutils.geometry import tessellate_polygon
import xml.etree.ElementTree as ET


def specs():
    result=[]
    flat=[(14,10,'compact_rooms'),(42,9,'long_warehouse'),(28,30,'wide_plaza'),(30,24,'L_workshop'),(60,38,'large_factory')]
    terrain=[(18,14,'small_hills'),(44,12,'gentle_long_grade'),(30,32,'winding_valley'),(46,34,'wide_rolling_field'),(13,22,'short_terraces')]
    floor=[(20,16,'compact_straight_ramp'),(56,14,'long_mezzanine'),(34,38,'gallery_switchback'),(44,28,'L_bridge_bottleneck'),(48,40,'large_dual_ramp'),(22,18,'flat_lower_rolling_upper'),(24,20,'rolling_lower_rolling_upper')]
    for category,items in [('flat',flat),('terrain',terrain),('floor',floor)]:
        for number,(sx,sy,layout) in enumerate(items,1):result.append(dict(name=f'{category}_{number}',category=category,index=number,size=(sx,sy),layout=layout))
    return result


def profile(number,x,y):
    x=np.asarray(x);y=np.asarray(y)
    if number==1:return .70*np.exp(-((x-1)**2/13+(y+1)**2/10))+.35*np.exp(-((x+4)**2+(y-3)**2)/10)
    if number==2:return .55*(1+np.tanh((x+2)/10))+.08*np.sin(x/6)*np.cos(y/4)
    if number==3:
        c=2*np.sin(y/6)
        return 1.5*np.exp(-((x-c-7)/5)**2)+1.0*np.exp(-((x-c+8)/5)**2)
    if number==4:return .45+.20*np.sin(x/4)*np.cos(y/5)+.16*np.cos((x+y)/8)+.65*np.exp(-((x+5)**2+(y-4)**2)/70)
    def smooth(t):
        t=np.clip(t,0,1);return t*t*(3-2*t)
    return .8*smooth((y+5)/8)+.5*smooth((y-4)/7)+.04*np.cos(x/3)**2


def floor_profile(number, layer, x, y):
    """Smooth 0.4--0.6 m relief, with level ramp entry/exit corridors.

    Coordinates are world coordinates before compaction. Keeping the whole
    landing strip level also keeps the triangulated mesh flush with the ramp.
    """
    x=np.asarray(x);y=np.asarray(y)
    def blend(value):
        value=np.clip(value,0,1);return value*value*(3-2*value)
    if layer=='lower':
        relief=(.36*np.exp(-((x+1)**2/15+(y+3)**2/9))
                +.26*np.exp(-((x-7)**2/12+(y-6)**2/10))
                +.12*(1+np.sin(x/2.8)*np.cos(y/3.2)))
        # floor_7 ramp runs west-to-east along y=5.
        return relief*blend((np.abs(y-5)-1.5)/1.2)
    relief=(.42*np.exp(-((x-6)**2/7+(y+2)**2/9))
            +.30*np.exp(-((x-8)**2/9+(y-4.5)**2/6))
            +.10*(1+np.sin((x-1)/2.2)*np.cos(y/2.8)))
    landing_y=4. if number==6 else 5.
    # Upper surface begins at x=1; preserve a level joining strip there.
    return relief*blend((x-2)/1.5)*blend((np.abs(y-landing_y)-1.5)/1.0)


def surface_height(shape, x, y, spec):
    scale=shape.get('profile_scale',1.)
    if 'floor_layer' in shape:
        return shape['pose'][2]+float(floor_profile(spec['index'],shape['floor_layer'],x/scale,y/scale))*scale
    return float(profile(spec['index'],x/scale,y/scale))*scale


def layout(spec):
    sx,sy=spec['size'];n=spec['index'];category=spec['category'];shapes=[]
    def add(name,kind,size,pose,color='obstacle'):shapes.append(dict(name=name,kind=kind,size=list(size),pose=list(pose),color=color))
    def box(name,x,y,z,dx,dy,dz,color='structure',pitch=0,yaw=0):add(name,'box',(dx,dy,dz),(x,y,z,0,pitch,yaw),color)
    def slab(name,x,y,dx,dy,z=0,color='ground'):box(name,x,y,z-.10,dx,dy,.20,color)
    def boundary(poly,z=0,height=1.6,prefix='boundary'):
        for i,(a,b) in enumerate(zip(poly,poly[1:]+poly[:1])):
            dx,dy=b[0]-a[0],b[1]-a[1]
            box(f'{prefix}_{i}',(a[0]+b[0])/2,(a[1]+b[1])/2,z+height/2,math.hypot(dx,dy),.16,height,'wall',yaw=math.atan2(dy,dx))
    def rect(dx,dy,cx=0,cy=0):return [[cx-dx/2,cy-dy/2],[cx+dx/2,cy-dy/2],[cx+dx/2,cy+dy/2],[cx-dx/2,cy+dy/2]]
    def ramp(name,a,b,width):
        run=math.hypot(b[0]-a[0],b[1]-a[1]);rise=b[2]-a[2];angle=math.atan2(rise,run);yaw=math.atan2(b[1]-a[1],b[0]-a[0]);length=math.hypot(run,rise);normal=(-math.sin(angle)*math.cos(yaw),-math.sin(angle)*math.sin(yaw),math.cos(angle))
        center=[(a[i]+b[i])/2-.09*normal[i]for i in range(3)]
        box(name,*center,length,width,.18,'ramp',pitch=-angle,yaw=yaw)
        for side in [-1,1]:
            x=center[0]-math.sin(yaw)*side*(width/2+.06);y=center[1]+math.cos(yaw)*side*(width/2+.06)
            box(name+f'_rail_{side}',x,y,center[2]+.28,length,.10,.50,'accent',pitch=-angle,yaw=yaw)
    def obstacle(name,x,y,z=0,kind='box',size=(.9,.8,.7)):
        if kind=='box':box(name,x,y,z+size[2]/2,*size,'obstacle')
        elif kind=='cylinder':add(name,kind,size,(x,y,z+size[1]/2,0,0,0),'accent')
        else:add(name,kind,size,(x,y,z+size[0],0,0,0),'prop')
    polygon=rect(sx,sy);start=(-sx/2+2,-sy/2+2,0.,0.);goal=(sx/2-2,sy/2-2,0.,math.pi)
    upper=None;ramp_points=[]
    if category=='flat':
        if n==4:
            polygon=[[-15,-12],[15,-12],[15,-2],[-5,-2],[-5,12],[-15,12]]
            slab('west_wing',-10,0,10,24);slab('south_wing',5,-7,20,10)
            start=(-11,8,0.,-math.pi/2);goal=(12,-8,0.,0.)
            for i,(x,y)in enumerate([(-10,2),(-8,-6),(2,-7),(9,-5)]):obstacle(f'workbench_{i}',x,y,size=(1.7,1.0,.8))
        else:
            slab('floor',0,0,sx,sy)
            if n==1:
                for i,(x,y,dx,dy)in enumerate([(-1,-1,5,.18),(2,2,.18,4),(-4,2,1.1,1.0),(4,-2,1.2,.8)]):obstacle(f'partition_{i}',x,y,size=(dx,dy,1.1))
                start=(-5,-3,0.,0.);goal=(5,3,0.,math.pi/2)
            elif n==2:
                for i,x in enumerate([-12,-4,4,12]):
                    y=1.7 if i%2 else -1.7
                    obstacle(f'rack_{i}',x,y,size=(3.4,1.2,1.3))
                start=(-18,0,0.,0.);goal=(18,0,0.,0.)
            elif n==3:
                for i,(x,y)in enumerate([(-7,-6),(5,-8),(-5,5),(7,7),(0,0)]):
                    obstacle(f'island_{i}',x,y,size=(2.2,1.8,.8));obstacle(f'pillar_{i}',x+3,y+2,kind='cylinder',size=(.35,1.3))
            else:
                for row,y in enumerate([-10,0,10]):
                    for col,x in enumerate([-18,-7,7,18]):
                        if row==1 and col in [1,2]:continue
                        obstacle(f'machine_{row}_{col}',x,y,size=(3.4,2.4,1.4))
        boundary(polygon,height=1.8)
    elif category=='terrain':
        add('ground_surface','surface',spec['size'],(0,0,0,0,0,0),'ground')
        # Paths traverse height change; obstacle locations differ in scale/layout.
        locations={1:[(-2,2),(3,-2),(5,3)],2:[(-12,3),(-3,-3),(6,3),(14,-3)],3:[(-9,-3),(4,8),(10,-6),(-5,12)],4:[(-12,4),(2,-8),(13,7),(-4,12),(17,-11)],5:[(-3,-4),(3,2),(-2,7)]}[n]
        for i,(x,y)in enumerate(locations):
            z=float(profile(n,x,y))
            if i%3==0:obstacle(f'rock_box_{i}',x,y,z,size=(1.1,1.0,.65))
            elif i%3==1:obstacle(f'rock_round_{i}',x,y,z,'sphere',(.38,))
            else:obstacle(f'stone_column_{i}',x,y,z,'cylinder',(.34,.8))
        boundary(polygon,z=-.10,height=3.)
        if n==2:start=(-19,-3,0.,0.);goal=(19,3,0.,math.pi)
        if n==5:start=(-4,-9,0.,math.pi/2);goal=(4,9,0.,math.pi/2)
        start=(*start[:2],float(profile(n,*start[:2])),start[3]);goal=(*goal[:2],float(profile(n,*goal[:2])),goal[3])
    else:
        if n==1:
            slab('lower',0,0,sx,sy);slab('upper',6,0,8,16,2.4,'upper')
            ramp('ramp',(-8,4,0),(2.05,4,2.4),2.2);ramp_points=[[-8,4],[2.05,4]]
            for i,(x,y,z)in enumerate([(-2,-3,0),(4,-5,2.4),(7,-2,2.4)]):obstacle(f'crate_{i}',x,y,z)
            start=(-8,-6,0.,0.);upper=(7,5,2.4,-math.pi/2)
            boundary(rect(8,16,6,0),2.4,.7,'upper_rail')
            # Open the west rail at the ramp landing.
            shapes=[s for s in shapes if s['name']!='upper_rail_3']
        elif n==2:
            slab('lower',0,0,sx,sy);slab('upper',20,1.5,16,11,2.8,'upper')
            ramp('ramp',(-4,4,0),(12.05,4,2.8),2.6);ramp_points=[[-4,4],[12.05,4]]
            for i,x in enumerate([-15,-5,4]):obstacle(f'lower_rack_{i}',x,-2,size=(2.3,1.2,1.1))
            obstacle('upper_crate',18,-2,2.8,size=(1.4,1.1,.7));start=(-25,-5,0.,0.);upper=(24,4,2.8,math.pi)
            boundary(rect(16,11,20,1.5),2.8,.75,'upper_rail');shapes=[s for s in shapes if s['name']!='upper_rail_3']
        elif n==3:
            slab('lower',0,0,sx,sy);slab('landing',-13,2,4,4,1.6,'upper')
            slab('east_upper',10.5,5,13,26,3.2,'upper');slab('north_upper',-5,15.5,18,5,3.2,'upper')
            ramp('first_ramp',(-13,-12,0),(-13,.05,1.6),2.5);ramp('second_ramp',(-11,3,1.6),(4.05,3,3.2),2.5)
            ramp_points=[[-13,-12],[-13,2],[-11,3],[4.05,3]]
            for i,(x,y,z)in enumerate([(-5,-8,0),(5,-4,0),(11,0,3.2),(8,12,3.2)]):obstacle(f'gallery_display_{i}',x,y,z,size=(1.4,1.1,.8))
            start=(-14,-15,0.,0.);upper=(-10,15.5,3.2,math.pi)
            boundary([[4,-8],[17,-8],[17,18],[-14,18],[-14,13],[4,13]],3.2,.7,'gallery_rail')
            shapes=[s for s in shapes if s['name']!='gallery_rail_5']
            box('gallery_west_lower',4,-3.225,3.55,.16,9.55,.7,'accent');box('gallery_west_upper',4,8.725,3.55,.16,8.55,.7,'accent')
        elif n==4:
            polygon=[[-22,-14],[22,-14],[22,-2],[-10,-2],[-10,14],[-22,14]]
            slab('west_lower',-16,0,12,28);slab('south_lower',6,-8,32,12)
            slab('west_upper',-16,9,12,10,2.8,'upper');slab('bridge_upper',0,10,20,5,2.8,'upper')
            ramp('ramp',(-16,-11,0),(-16,4.05,2.8),2.6);ramp_points=[[-16,-11],[-16,4.05]]
            obstacle('partial_corridor_block',-4,11.1,2.8,size=(3.2,1.8,1.0))
            obstacle('lower_crate',-18,-3,0,size=(1.2,.9,.8));start=(-19,-11,0.,0.);upper=(7,9,2.8,math.pi)
            upper_outline=[[-22,4],[-10,4],[-10,7.5],[10,7.5],[10,12.5],[-10,12.5],[-10,14],[-22,14]]
            boundary(upper_outline,2.8,.7,'upper_rail');shapes=[s for s in shapes if s['name']!='upper_rail_0']
            box('south_rail_left',-19.7,4,3.15,4.6,.16,.7,'accent');box('south_rail_right',-12.3,4,3.15,4.6,.16,.7,'accent')
        elif n==5:
            slab('lower',0,0,sx,sy);slab('upper',8,6,24,20,3.6,'upper')
            ramp('west_ramp',(-22,-1,0),(-3.95,-1,3.6),3.0);ramp('south_ramp',(15,-18,0),(15,-3.95,3.6),3.0)
            ramp_points=[[-22,-1],[-3.95,-1],[15,-18],[15,-3.95]]
            for i,(x,y)in enumerate([(-10,-10),(0,-9),(5,4),(15,10)]):obstacle(f'lower_pillar_{i}',x,y,kind='cylinder',size=(.3,2.6))
            for i,(x,y)in enumerate([(6,8),(14,4),(1,11)]):obstacle(f'upper_machine_{i}',x,y,3.6,size=(1.5,1.3,.8))
            start=(-20,-15,0.,0.);upper=(15,13,3.6,math.pi)
            boundary(rect(24,20,8,6),3.6,.7,'upper_rail');shapes=[s for s in shapes if s['name']not in ['upper_rail_0','upper_rail_3']]
            box('west_lower_rail',-4,-3.3,3.95,.16,1.4,.7,'accent');box('west_upper_rail',-4,8.3,3.95,.16,15.4,.7,'accent')
            box('south_left_rail',4.7,-4,3.95,17.4,.16,.7,'accent');box('south_right_rail',18.3,-4,3.95,3.4,.16,.7,'accent')
        else:
            # Compact mixed-surface floors. Each layer is a separate physical
            # solid; the space between the floors remains empty.
            height=2.1 if n==6 else 2.2
            deck_width=10. if n==6 else 11.
            deck_depth=sy-2.;deck_x=1.+deck_width/2
            if n==6:
                slab('lower',0,0,sx,sy)
            else:
                add('lower_surface','surface',(sx,sy),(0,0,0,0,0,0),'ground')
                shapes[-1]['floor_layer']='lower'
            add('upper_surface','surface',(deck_width,deck_depth),(deck_x,0,height,0,0,0),'upper')
            shapes[-1]['floor_layer']='upper'
            landing_y=4. if n==6 else 5.
            ramp('ramp',(-sx/2+2,landing_y,0),(1.05,landing_y,height),2.4)
            # The ramp levels out before the deck edge. A straight ramp
            # ending at that edge leaves a sharp support-plane transition.
            slope_shape=next(s for s in shapes if s['name']=='ramp')
            begin_x=-sx/2+2;end_x=1.05
            slope_shape.update(kind='smooth_ramp',size=[end_x-begin_x,2.4,.18],pose=[(begin_x+end_x)/2,landing_y,0,0,0,0],rise=height)
            ramp_points=[[-sx/2+2,landing_y],[1.05,landing_y]]
            start=(-sx/2+2,-sy/2+2,0.,0.)
            if n==7:start=(*start[:2],float(floor_profile(n,'lower',*start[:2])),start[3])
            upper=(sx/2-2,-sy/2+4,height,math.pi/2)
            upper=(*upper[:2],height+float(floor_profile(n,'upper',*upper[:2])),upper[3])
            boundary(rect(deck_width,deck_depth,deck_x,0),height,.9,'upper_rail')
            shapes=[s for s in shapes if s['name']!='upper_rail_3']
            gap=1.35;edge=deck_depth/2
            for label,ya,yb in [('south',-edge,landing_y-gap),('north',landing_y+gap,edge)]:
                if yb>ya:box('upper_west_'+label,1,(ya+yb)/2,height+.45,.16,yb-ya,.9,'accent')
        boundary(polygon,height=2.)
        goal=upper
    return shapes,polygon,start,goal,upper,ramp_points


def compact_scene(spec,shapes,polygon,start,goal,upper,ramp_points):
    # Uniformly shrink geometry and height to retain ramp grades; preserve a
    # usable tracked-robot aisle rather than scaling it below body clearance.
    targets={'flat':[14,26,21,21,30],'terrain':[16,27,23,28,19],'floor':[18,30,26,28,30,22,24]}
    scale=min(1.,targets[spec['category']][spec['index']-1]/max(spec['size']))
    source_size=spec['size'];spec['size']=tuple(v*scale for v in source_size)
    polygon=[[x*scale,y*scale]for x,y in polygon]
    def point(p):return (p[0]*scale,p[1]*scale,p[2]*scale,p[3])
    start,goal=point(start),point(goal);upper=point(upper)if upper else None
    ramp_points=[[x*scale,y*scale]for x,y in ramp_points]
    for shape in shapes:
        shape['size']=[v*scale for v in shape['size']]
        shape['pose']=[v*scale for v in shape['pose'][:3]]+shape['pose'][3:]
        if shape['kind']=='surface':shape['profile_scale']=scale
    for shape in shapes:
        if shape['color']!='ramp':continue
        old=shape['size'][1];new=max(1.8,old);shape['size'][1]=new
        for side in [-1,1]:
            rail=next(s for s in shapes if s['name']==shape['name']+f'_rail_{side}')
            yaw=rail['pose'][5];delta=side*(new-old)/2
            rail['pose'][0]-=math.sin(yaw)*delta;rail['pose'][1]+=math.cos(yaw)*delta
    # A single polygon slab removes the buried joint faces responsible for the
    # fake flat_4 seam. Its world collision, Blender mesh and PCD share geometry.
    if spec['name']=='flat_4':
        shapes=[s for s in shapes if s['name']not in ['west_wing','south_wing']]
        shapes.insert(0,dict(name='continuous_L_floor',kind='polygon',size=[.20*scale],pose=[0,0,0,0,0,0],color='ground',polygon=polygon))
    rng=np.random.default_rng(20261008+spec['index']+100*['flat','terrain','floor'].index(spec['category']))
    def inside(p,poly):
        x,y=p;result=False
        for a,b in zip(poly,poly[1:]+poly[:1]):
            if(a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:result=not result
        return result
    def distance_segment(p,a,b):
        d=np.array(b)-np.array(a);t=np.clip(np.dot(np.array(p)-a,d)/max(1e-8,np.dot(d,d)),0,1);return float(np.linalg.norm(np.array(p)-a-t*d))
    regions=[]
    for s in shapes:
        if s['color']not in ['ground','upper']:continue
        if s['kind']=='surface':
            x,y,z=s['pose'][:3];dx,dy=s['size']
            regions.append(([[x-dx/2,y-dy/2],[x+dx/2,y-dy/2],[x+dx/2,y+dy/2],[x-dx/2,y+dy/2]],z,s))
        elif s['kind']=='polygon':regions.append((polygon,0.,None))
        elif s['kind']=='box':
            x,y,z=s['pose'][:3];dx,dy,dz=s['size'];regions.append(([[x-dx/2,y-dy/2],[x+dx/2,y-dy/2],[x+dx/2,y+dy/2],[x-dx/2,y+dy/2]],z+dz/2,None))
    counts={'flat':[11,16,18,15,24],'terrain':[12,15,18,22,15],'floor':[12,17,19,17,22,28,32]}
    existing=[s for s in shapes if s['color']in ['obstacle','prop']or (s['color']=='accent'and s['kind']in ['cylinder','sphere'])]
    for s in existing:
        if s['kind']=='box':s['pose'][5]=float(rng.uniform(-1.,1.))
    desired=counts[spec['category']][spec['index']-1]
    i=0
    for attempt in range(3000):
        if len(existing)>=desired:break
        poly,level,surface=regions[attempt%len(regions)];x=float(rng.uniform(min(p[0]for p in poly)+.7,max(p[0]for p in poly)-.7));y=float(rng.uniform(min(p[1]for p in poly)+.7,max(p[1]for p in poly)-.7))
        if not inside((x,y),poly):continue
        if min(math.hypot(x-p[0],y-p[1])for p in [start,goal])<1.7:continue
        blocked=False
        for r in shapes:
            if r['color']=='ramp':
                px,py=r['pose'][:2];length=r['size'][0]*math.cos(r['pose'][4]);yaw=r['pose'][5];a=[px-length/2*math.cos(yaw),py-length/2*math.sin(yaw)];b=[px+length/2*math.cos(yaw),py+length/2*math.sin(yaw)]
                if distance_segment((x,y),a,b)<r['size'][1]/2+.75:blocked=True;break
        if blocked:continue
        if any(abs(s['pose'][2]-level)<1.2 and math.hypot(x-s['pose'][0],y-s['pose'][1])<max(s['size'][:2])*.65+1.0 for s in existing):continue
        kind=['box','cylinder','sphere','cone'][i%4];height=float(rng.uniform(.4,1.0));ground=level
        if surface is not None:ground=surface_height(surface,x,y,spec)
        if kind=='box':size=[float(rng.uniform(.5,1.15)),float(rng.uniform(.4,.85)),height];z=ground+height/2
        elif kind=='cylinder':size=[float(rng.uniform(.25,.42)),height];z=ground+height/2
        elif kind=='cone':size=[float(rng.uniform(.28,.46)),height];z=ground+height/2
        else:size=[float(rng.uniform(.25,.4))];z=ground+size[0]
        s=dict(name=f'irregular_prop_{i}',kind=kind,size=size,pose=[x,y,z,0,0,float(rng.uniform(-math.pi,math.pi))],color='obstacle'if kind=='box'else'prop')
        shapes.append(s);existing.append(s);i+=1
    dynamic=[]
    if spec['index']==3:
        dynamic=distributed_motion(spec,shapes,regions,start,goal,rng)
    return shapes,polygon,start,goal,upper,ramp_points,dynamic


def distributed_motion(spec, shapes, regions, start, goal, rng):
    """Place independent routes across spatial strata, away from fixed solids.

    A clearance check covers every route segment, including the ramp rails.
    Static obstacles are retained; moving geometry never enters the static PCD.
    """
    category=spec['category']
    if category=='floor':
        usable=[r for r in regions if (max(p[0]for p in r[0])-min(p[0]for p in r[0]))*(max(p[1]for p in r[0])-min(p[1]for p in r[0]))>12.]
        allocations=[(usable[0],2,2),(usable[1],1,3),(usable[2],1,1)]
        speeds=[.25,.65,.40,.95,.30,.80,.50,.90]
    else:
        allocations=[(regions[0],2,4 if category=='flat' else 3)]
        speeds=([.25,.65,.40,.95,.30,.80,.50,.90] if category=='flat'
                else [.25,.65,.40,.80,.30,.75])
    strata=[]
    for region,nx,ny in allocations:
        poly,level,surface=region
        xmin,xmax=min(p[0]for p in poly),max(p[0]for p in poly)
        ymin,ymax=min(p[1]for p in poly),max(p[1]for p in poly)
        for ix in range(nx):
            for iy in range(ny):
                strata.append((region,(xmin+(ix+.5)*(xmax-xmin)/nx,ymin+(iy+.5)*(ymax-ymin)/ny),((xmax-xmin)/nx,(ymax-ymin)/ny)))
    result=[]
    def segment_distance(point,a,b):
        delta=b-a;fraction=np.clip(np.dot(point-a,delta)/max(1e-8,np.dot(delta,delta)),0,1)
        return float(np.linalg.norm(point-a-fraction*delta))
    def free(points,region):
        poly,level,surface=region
        xmin,xmax=min(p[0]for p in poly),max(p[0]for p in poly)
        ymin,ymax=min(p[1]for p in poly),max(p[1]for p in poly)
        for x,y,z in points:
            if not(xmin+.7<x<xmax-.7 and ymin+.7<y<ymax-.7):return False
            if any(abs(z-.4-p[2])<.9 and math.hypot(x-p[0],y-p[1])<1.6 for p in [start,goal]):return False
            for shape in shapes:
                if shape['color']in ['ground','upper']:continue
                sx,sy,sz=shape['pose'][:3];kind=shape['kind'];size=shape['size']
                halfheight=size[2]/2 if kind=='box' else size[0] if kind=='sphere' else size[1]/2
                # The obstacle occupies ground..ground+.8; unrelated floors
                # and upper rails above that interval do not block this route.
                if sz-halfheight>z+.4+.15 or sz+halfheight<z-.4+.08:continue
                yaw=shape['pose'][5];dx=x-sx;dy=y-sy
                if kind=='box':
                    local_x=math.cos(yaw)*dx+math.sin(yaw)*dy
                    local_y=-math.sin(yaw)*dx+math.cos(yaw)*dy
                    half_x=size[0]*abs(math.cos(shape['pose'][4]))/2
                    distance=math.hypot(max(0,abs(local_x)-half_x),max(0,abs(local_y)-size[1]/2))
                else:distance=max(0,math.hypot(dx,dy)-size[0])
                if distance<.62:return False
            for prior in result:
                previous=np.asarray(prior['waypoints'])
                if abs(z-previous[:,2].mean())>.9:continue
                if any(segment_distance(np.array([x,y]),a[:2],b[:2])<1.0 for a,b in zip(previous,previous[1:])):return False
        return True
    for i,(region,anchor,cell) in enumerate(strata):
        poly,level,surface=region
        for attempt in range(3000):
            center=np.asarray(anchor)+rng.uniform(-.34,.34,2)*cell
            angle=float(rng.uniform(0,math.pi));half=float(rng.uniform(1.5,2.8))
            delta=np.array([math.cos(angle),math.sin(angle)])*half
            xy=np.linspace(center-delta,center+delta,40)
            points=[[float(x),float(y),(surface_height(surface,x,y,spec)if surface else level)+.4]for x,y in xy]
            if free(points,region):break
        else:raise RuntimeError(f'Cannot place a safe dispersed dynamic route: {spec["name"]} stratum {i}')
        if i%2:points.reverse()
        result.append(dict(name=f'moving_obstacle_{i+1}',kind='box'if i%2==0 else'cylinder',size=[.55,.55,.8]if i%2==0 else[.3,.8],waypoints=points,speed=speeds[i],start_delay=.25*(i%4),color=[1.,.015,.015,1.],region_height=level))
    return result


def make_object(shape,legacy,package,name,category):
    kind=shape['kind'];size=shape['size'];pose=shape['pose']
    if kind=='box':bpy.ops.mesh.primitive_cube_add(size=1);obj=bpy.context.object;obj.dimensions=size
    elif kind=='cylinder':bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=size[0],depth=size[1]);obj=bpy.context.object
    elif kind=='sphere':bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,radius=size[0]);obj=bpy.context.object
    elif kind=='cone':bpy.ops.mesh.primitive_cone_add(vertices=32,radius1=size[0],radius2=.03,depth=size[1]);obj=bpy.context.object
    elif kind=='polygon':
        poly=shape['polygon'];thickness=size[0];count=len(poly);top=[Vector((x,y,0.))for x,y in poly];vertices=[tuple(v)for v in top]+[(v.x,v.y,-thickness)for v in top]
        def index(v):return v if isinstance(v,int)else min(range(count),key=lambda k:(top[k]-v).length)
        faces=[]
        for tri in tessellate_polygon([top]):
            ids=tuple(index(v)for v in tri);faces.append(ids);faces.append(tuple(k+count for k in reversed(ids)))
        for k in range(count):j=(k+1)%count;faces.append((k,k+count,j+count,j))
        mesh=bpy.data.meshes.new(shape['name']);mesh.from_pydata(vertices,[],faces);mesh.update();obj=bpy.data.objects.new(shape['name'],mesh);bpy.context.collection.objects.link(obj)
    elif kind=='smooth_ramp':
        sx,sy,thickness=size
        def height(x,y):
            t=np.clip((x+sx/2-.5)/(sx-.5-.65),0,1)
            return shape['rise']*t*t*(3-2*t)
        triangles=legacy.heightfield((-sx/2,sx/2,-sy/2,sy/2),height,step=.15,thickness=thickness)
        vertices=triangles.reshape(-1,3);faces=[(i,i+1,i+2)for i in range(0,len(vertices),3)]
        mesh=bpy.data.meshes.new(shape['name']);mesh.from_pydata(vertices.tolist(),[],faces);mesh.update()
        obj=bpy.data.objects.new(shape['name'],mesh);bpy.context.collection.objects.link(obj)
    else:
        sx,sy=size;n=int(name.split('_')[-1])
        scale=shape.get('profile_scale',1.)
        if 'floor_layer' in shape:
            height=lambda x,y:floor_profile(n,shape['floor_layer'],(x+pose[0])/scale,(y+pose[1])/scale)*scale
        else:height=lambda x,y:profile(n,x/scale,y/scale)*scale
        triangles=legacy.heightfield((-sx/2,sx/2,-sy/2,sy/2),height,step=.25 if 'floor_layer' in shape else .35,thickness=.25)
        vertices=triangles.reshape(-1,3);faces=[(i,i+1,i+2)for i in range(0,len(vertices),3)]
        mesh=bpy.data.meshes.new(shape['name']);mesh.from_pydata(vertices.tolist(),[],faces);mesh.update()
        obj=bpy.data.objects.new(shape['name'],mesh);bpy.context.collection.objects.link(obj)
    obj.name=shape['name'];obj.location=pose[:3];obj.rotation_euler=pose[3:]
    bpy.context.view_layer.objects.active=obj;obj.select_set(True);bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    color=legacy.PALETTES[category][shape['color']];obj.color=tuple(map(float,color.split()))
    mat=bpy.data.materials.new(shape['name']+'_material');mat.diffuse_color=obj.color;obj.data.materials.append(mat)
    return obj


def export_mesh(objects,path):
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:obj.select_set(True)
    bpy.ops.export_mesh.stl(filepath=str(path),use_selection=True,ascii=False,check_existing=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--package-dir',type=Path,required=True);parser.add_argument('--maps',nargs='+');args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    package=args.package_dir.resolve();sys.path.insert(0,str(package/'scripts'));import generate_map_suite as legacy
    existing=json.loads((package/'worlds/catalog.json').read_text());catalog=existing;catalog['scene_version']='compact_mixed_floors_dynamic_20261008_v3';catalog['maps']={} if not args.maps else catalog['maps']
    surveyed=catalog.pop('tracked_initial_poses', {}) if args.maps else {}
    design_path=package/'worlds/scene_design.json'
    design=[r for r in json.loads(design_path.read_text())if r['name']not in args.maps] if args.maps and design_path.exists()else[]
    for spec in specs():
        name=spec['name'];category=spec['category'];sx,sy=spec['size']
        if args.maps and name not in args.maps:continue
        bpy.ops.wm.read_factory_settings(use_empty=True);bpy.context.scene.unit_settings.system='METRIC'
        shapes,polygon,start,goal,upper,ramp_points=layout(spec)
        shapes,polygon,start,goal,upper,ramp_points,dynamic=compact_scene(spec,shapes,polygon,start,goal,upper,ramp_points)
        sx,sy=spec['size']
        sdf,link=legacy.new_world(name,spec['size']);objects=[]
        folder=package/'3DModel'/category;folder.mkdir(parents=True,exist_ok=True)
        for shape in shapes:
            obj=make_object(shape,legacy,package,name,category);objects.append(obj)
            if shape['kind']in ['surface','polygon','cone','smooth_ramp']:
                surface_path=folder/'surfaces'/f'{name}_{shape["name"]}.stl';surface_path.parent.mkdir(parents=True,exist_ok=True);export_mesh([obj],surface_path)
                legacy.add_shape(link,shape['name'],'mesh',f'3DModel/{category}/surfaces/{surface_path.name}',(0,0,0,0,0,0),shape['color'],collide=True)
            else:legacy.add_shape(link,shape['name'],shape['kind'],shape['size'],shape['pose'],shape['color'],collide=True)
        mesh_path=folder/(name+'.stl');export_mesh(objects,mesh_path)
        # Individual physical solids preserve the empty space below upper floors.
        for d in dynamic:
            model=ET.SubElement(sdf.find('world'),'model',name=d['name']);legacy.text(model,'static','true');legacy.text(model,'pose',legacy.vec([*d['waypoints'][0],0,0,0]))
            moving_link=ET.SubElement(model,'link',name='obstacle');legacy.add_shape(moving_link,'body',d['kind'],d['size'],color='accent',collide=True)
            material=moving_link.find('visual/material')
            for field in ['ambient','diffuse']:material.find(field).text=legacy.vec(d['color'])
            legacy.text(material,'emissive','0.25 0 0 1')
            plugin=ET.SubElement(model,'plugin',name='path_motion',filename='libmultimap_obstacle_motion.so');legacy.text(plugin,'speed',d['speed']);legacy.text(plugin,'start_delay',d['start_delay']);points=ET.SubElement(plugin,'waypoints')
            for point in d['waypoints']:legacy.text(points,'point',legacy.vec(point))
        world_path=package/'worlds'/category/(name+'.world');legacy.save_xml(world_path,sdf)
        floor_types={6:['flat','terrain'],7:['terrain','terrain']}.get(spec['index'],['flat','flat'])
        record=dict(category=category,description=spec['layout'],world=f'worlds/{category}/{name}.world',mesh=f'3DModel/{category}/{name}.stl',pcd=f'pcd/{category}/{name}.pcd',size_xy=[sx,sy],floor_types=floor_types if category=='floor'else[category],spawn=[start[0],start[1],start[2]+.15,start[3]],goal=[goal[0],goal[1],goal[2]+.03,goal[3]],expected_mode={'flat':'costmap2d','terrain':'gridmap','floor':'octomap'}[category],footprint_polygon=polygon)
        if upper:record.update(upper_spawn=[upper[0],upper[1],upper[2]+.15,upper[3]],ramp_start=[*ramp_points[0],.15,0.],ramp_waypoints=ramp_points)
        record['dynamic_obstacles']=dynamic
        catalog['maps'][name]=record
        surveyed[name]=dict(lower=dict(x=start[0],y=start[1],base_z=start[2]+.03),world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest())
        if upper:surveyed[name]['upper']=dict(x=upper[0],y=upper[1],base_z=upper[2]+.03)
        camera_data=bpy.data.cameras.new('Overview');camera=bpy.data.objects.new('Overview',camera_data);bpy.context.collection.objects.link(camera)
        camera.location=(sx*.65,-sy*.85,max(sx,sy)*.85);camera.rotation_euler=(Vector((0,0,1))-camera.location).to_track_quat('-Z','Y').to_euler();camera_data.type='ORTHO';camera_data.ortho_scale=max(sx,sy)*1.45;bpy.context.scene.camera=camera
        bpy.ops.wm.save_as_mainfile(filepath=str(folder/(name+'.blend')),check_existing=False)
        design.append(dict(name=name,layout=spec['layout'],size_xy=spec['size'],polygon=polygon,start=start,goal=goal,objects=len(shapes),ramp_widths=[s['size'][1]for s in shapes if s['name'].endswith('ramp') or s['name']=='ramp'],shapes=shapes,dynamic_obstacles=dynamic))
        print('BLENDER_SCENE',name,spec['size'],len(shapes),flush=True)
    from stl_to_navigation_pcd import read_binary_stl,sample_navigation_surfaces,write_binary_pcd,interior_from_size
    catalog['pcd_sampling']={'terrain_spacing':.09,'wall_spacing':.08}
    for name,record in catalog['maps'].items():
        if args.maps and name not in args.maps:continue
        triangles=read_binary_stl(package/record['mesh'])
        points,stats=sample_navigation_surfaces(triangles,.09,.08,interior_from_size(record['size_xy']))
        if record['category']=='flat':points=points[points[:,2]>=-.02]
        output=package/record['pcd'];output.parent.mkdir(parents=True,exist_ok=True);write_binary_pcd(output,points)
        mirror=package/'pcd_modified'/record['category']/(name+'.pcd');mirror.parent.mkdir(parents=True,exist_ok=True);write_binary_pcd(mirror,points)
        record.update(points=len(points),triangles=len(triangles),pcd_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),bounds_min=points.min(axis=0).astype(float).tolist(),bounds_max=points.max(axis=0).astype(float).tolist())
    catalog['tracked_initial_poses']=surveyed
    (package/'worlds/catalog.json').write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
    (package/'worlds/scene_design.json').write_text(json.dumps(design,ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':main()
