"""Deterministic local ENU geometry. This OFFLINE exporter may read/write truth.
Online code consumes only maps/prior and sensor_config.json, never this module.
"""
import argparse, hashlib, json, math, random
from pathlib import Path
from xml.etree import ElementTree as ET
import numpy as np
import yaml
from shapely import constrained_delaunay_triangles
from shapely.geometry import Polygon, LineString, Point, box, mapping
from shapely.affinity import scale, translate
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'optimization_v2'
def dump(p,d):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def feature(g,**props):return dict(type='Feature',geometry=mapping(g),properties=props)
def collection(fs):return dict(type='FeatureCollection',features=fs)
def geometries(variant='base',targets=(1650,1140),width=6):
    vertices=[[(8,24),(28,18),(52,22),(60,38),(54,48),(57,66),(38,77),(15,70),(10,52),(19,42)],
              [(108,42),(125,34),(143,41),(152,57),(144,76),(127,80),(112,70),(117,57),(105,52)]]
    zones=[]
    for pts,area,c in zip(vertices,targets,[(34,47),(128,58)]):
        p=Polygon(pts);s=math.sqrt(area/p.area);p=scale(p,s,s,origin='centroid');p=translate(p,c[0]-p.centroid.x,c[1]-p.centroid.y)
        if variant=='notch':p=p.difference(box(c[0]-3,p.bounds[3]-8,c[0]+3,p.bounds[3]+1))
        if variant=='holes':p=p.difference(box(c[0]-11,c[1]+5,c[0]-5,c[1]+9))
        if not (p.is_valid and p.geom_type=='Polygon' and p.area>0):raise ValueError('invalid/disconnected workspace')
        zones.append(p)
    control=[[(49,42),(65,34),(71,32),(82,50)],[(82,50),(93,68),(101,70),(122,58)]]
    points=[]
    for part in control:
        p=np.array(part)
        for t in np.linspace(0,1,81):points.append(((1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t*t*p[2]+t**3*p[3]).tolist())
    line=LineString(points);return zones,line,line.buffer(width/2,quad_segs=12)
def write_mesh(path,triangles):
    lines=['# Original generated S02 geometry; units metre']
    for tri in triangles:
        a,b,c=np.array(tri,dtype=float);n=np.cross(b-a,c-a);n/=np.linalg.norm(n)
        lines.extend('v '+' '.join(f'{v:.8f}' for v in p) for p in tri)
        lines.append('vn '+' '.join(f'{v:.8f}' for v in n))
    lines.extend(f'f {i*3+1}//{i+1} {i*3+2}//{i+1} {i*3+3}//{i+1}' for i in range(len(triangles)))
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text('\n'.join(lines)+'\n')
def mesh_polygon(p,path,z):
    triangles=list(constrained_delaunay_triangles(p).geoms)
    if abs(sum(t.area for t in triangles)-p.area)>1e-6:raise ValueError('triangulation mismatch')
    faces=[]
    for t in triangles:
        vs=list(t.exterior.coords)[:3]
        if not t.exterior.is_ccw:vs.reverse()
        faces.append([(x,y,z) for x,y in vs])
    write_mesh(path,faces)
def text(parent,tag,value):e=ET.SubElement(parent,tag);e.text=str(value);return e
def geom(parent,kind,data):
    g=ET.SubElement(parent,'geometry');e=ET.SubElement(g,kind)
    if kind=='box':text(e,'size',' '.join(map(str,data)))
    elif kind=='sphere':text(e,'radius',data)
    elif kind=='cylinder':text(e,'radius',data[0]);text(e,'length',data[1])
    else:text(e,'uri','file://'+str(data))
def part(link,name,kind,data,pose,color,collision=True):
    for tag in ['visual','collision'] if collision else ['visual']:
        e=ET.SubElement(link,tag,name=name if tag=='collision' else name+'_visual');text(e,'pose',' '.join(map(str,pose)))
        geom(e,kind,data)
        if tag=='visual':
            m=ET.SubElement(e,'material');text(m,'ambient',color);text(m,'diffuse',color)
    return e
def model(world,name,position,dynamic=False,mass=70):
    m=ET.SubElement(world,'model',name=name);text(m,'static',str(not dynamic).lower());text(m,'pose',' '.join(map(str,position))+' 0 0 0')
    l=ET.SubElement(m,'link',name='body')
    if dynamic:
        text(l,'gravity','false');i=ET.SubElement(l,'inertial');text(i,'mass',mass);it=ET.SubElement(i,'inertia')
        for k,v in [('ixx',mass*.2),('iyy',mass*.2),('izz',mass*.1),('ixy',0),('ixz',0),('iyz',0)]:text(it,k,v)
    return l
def target(a,t):
    q=2*math.pi*t/a['period']+a['phase'];return [a['center'][0]+a['amplitude'][0]*math.sin(q),a['center'][1]+a['amplitude'][1]*math.sin(2*q),a['center'][2]]
def generate(out,seed=41,variant='base',fixture=False):
    out=Path(out);cfg=yaml.safe_load((BASE/'config/scenes.yaml').read_text());zones,line,road=geometries(variant,[cfg['area_targets_m2'][k] for k in ['spray','mowing']],cfg['connector_width_m'])
    prior=out/'maps/prior';assets=out/'assets';worldpath=assets/'worlds/irregular_farm.sdf'
    zonefs=[feature(p,id=n,task=t,area_m2=p.area,connected=True) for p,n,t in zip(zones,['A','B'],['spray','mowing'])]
    dump(prior/'workspaces.geojson',collection(zonefs))
    checkpoints=[[*line.interpolate(f,normalized=True).coords[0],0] for f in np.linspace(0,1,9)]
    dump(prior/'connector.geojson',collection([feature(road,id='road',width_m=cfg['connector_width_m']),feature(line,id='centerline',length_m=line.length,elevation_m=0,slope_deg=0,checkpoints=checkpoints)]))
    known=[dict(id='pump_house',center=[35,22,1.25],size=[3,3,2.5]),dict(id='shed',center=[137,69,1.5],size=[5,4,3]),dict(id='utility',center=[49,57,.75],size=[2,2,1.5])]
    if variant=='holes':
        for i,p in enumerate(zones):
            c=Polygon(p.interiors[0]).centroid;known.append(dict(id='island_'+str(i),center=[c.x,c.y,1],size=[6,4,2]))
    dump(prior/'permanent.geojson',collection([feature(box(k['center'][0]-k['size'][0]/2,k['center'][1]-k['size'][1]/2,k['center'][0]+k['size'][0]/2,k['center'][1]+k['size'][1]/2),id=k['id'],height_m=k['size'][2]) for k in known]))
    dump(prior/'terrain_coarse.geojson',collection([feature(box(38,61,49,67),id='coarse_grade_1',slope_band_deg=[0,8],resolution_m=3,uncertainty_deg=3)]))
    public_hash=hashlib.sha256(b''.join(p.read_bytes() for p in sorted(prior.glob('*.geojson')))).hexdigest()
    dump(prior/'map_metadata.json',dict(map_version=cfg['map_version'],map_id=public_hash,variant=variant,coordinate_system=cfg['coordinate_system'],world_bounds=[0,0,160,112],cluster_assignment=None,clearance_status='S03_NOT_TESTED'))
    dump(out/'config/sensor_config.json',dict(probe=cfg['probe'],topics=['/clock','/s02/probe/lidar/points','/s02/probe/image'],prior_path=str(prior),ros_domain_id=119))
    rng=random.Random(seed)
    low=[dict(id='unknown_low_'+str(i),center=[x+rng.uniform(-.15,.15),y+rng.uniform(-.15,.15),h/2],size=[.9,.8,h]) for i,(x,y,h) in enumerate([(26,23.5,.08),(28,26,.15),(30,28,.30)])]
    actors=[]
    for i,(c,a,period) in enumerate([([31,30,.9],[4,1,0],28),([77,47,.9],[3,2,0],34),([129,64,.9],[4,1.5,0],31),([44,58,.9],[3,1,0],26),([35,34,.55],[5,1.2,0],35)]):
        actors.append(dict(id='ped_'+str(i) if i<4 else 'equipment',center=c,amplitude=a,period=period,phase=rng.uniform(-.4,.4),size=[.5,.5,1.8] if i<4 else [1.6,.9,1.1]))
    terrain=[dict(id='grade_5',origin=[39,62,0],length=8,width=3,slope_deg=5,prior='coarse'),dict(id='grade_10',origin=[120,44,0],length=6,width=3,slope_deg=10,prior='unknown'),dict(id='grade_18',origin=[138,54,0],length=4,width=3,slope_deg=18,prior='unknown')]
    truth=dict(seed=seed,variant=variant,map_id=public_hash,low_obstacles=low,permanent=known,actors=actors,terrain=terrain,fixture=fixture,policy='SIMULATION_AND_EVALUATION_ONLY')
    sdf=ET.Element('sdf',version='1.10');w=ET.SubElement(sdf,'world',name='irregular_farm')
    physics=ET.SubElement(w,'physics',name='default',type='ignored');text(physics,'max_step_size',.002);text(physics,'real_time_factor',1)
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('contact','Contact')]:ET.SubElement(w,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name)
    s=ET.SubElement(w,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors');text(s,'render_engine','ogre')
    text(w,'gravity','0 0 -9.81');scene=ET.SubElement(w,'scene');text(scene,'ambient','0.6 0.6 0.6 1');text(scene,'background','0.68 0.81 0.91 1');text(scene,'shadows','false')
    sun=ET.SubElement(w,'light',name='sun',type='directional');text(sun,'pose','0 0 50 0 0 0');text(sun,'diffuse','.8 .8 .8 1');text(sun,'specular','.1 .1 .1 1');text(sun,'direction','-.3 -.4 -1')
    part(model(w,'ground',[80,56,-.1]),'ground','box',[160,112,.2],[0]*6,'.32 .39 .24 1')
    for p,name,color,z in [(zones[0],'spray_workspace','.57 .67 .29 1',.006),(zones[1],'mowing_workspace','.23 .53 .30 1',.006),(road,'connector','.63 .58 .46 1',.012)]:
        path=assets/'meshes'/f'{name}.obj';mesh_polygon(p,path,z);part(model(w,name,[0,0,0]),name,'mesh',path,[0]*6,color,False)
    for k in known+low:
        part(model(w,k['id'],k['center']),k['id'],'box',k['size'],[0]*6,'.62 .65 .69 1' if k in known else '.92 .35 .07 1')
    for t in terrain:
        x,y,z=t['origin'];l=t['length'];b=t['width'];h=l/2*math.tan(math.radians(t['slope_deg']))
        vs=[(0,0,0),(l,0,0),(l/2,0,h),(0,b,0),(l,b,0),(l/2,b,h)]
        # Outward winding is required by physical mesh contacts (not just rendering).
        faces=[(2,3,1),(6,5,4),(6,4,1),(3,6,1),(5,6,3),(2,5,3),(5,2,1),(4,5,1)]
        path=assets/'meshes'/f'{t["id"]}.obj';write_mesh(path,[[vs[i-1] for i in f] for f in faces])
        part(model(w,t['id'],[x,y,z]),'terrain','mesh',path,[0]*6,'.51 .43 .25 1')
    motion=ET.SubElement(w,'plugin',filename='libs02_motion.so',name='s02::ScriptedMotion')
    for a in actors:
        e=ET.SubElement(motion,'actor');text(e,'name',a['id'])
        for k in ['center','amplitude']:text(e,k,' '.join(map(str,a[k])))
        for k in ['period','phase']:text(e,k,a[k])
        l=model(w,a['id'],target(a,0),True)
        if a['id'].startswith('ped'):
            # Every visible primitive has the SAME physical geometry and transform.
            part(l,'torso','box',[.44,.30,.7],[0,0,.1,0,0,0],'.08 .35 .85 1')
            part(l,'head','sphere',.16,[0,0,.65,0,0,0],'.85 .64 .43 1')
            for y in [-.14,.14]:part(l,'leg_'+str(y),'box',[.18,.14,.7],[0,y,-.55,0,0,0],'.12 .15 .2 1')
            for y in [-.24,.24]:part(l,'arm_'+str(y),'box',[.14,.12,.65],[0,y,.08,0,0,0],'.08 .35 .85 1')
        else:
            part(l,'cart','box',[1.6,.9,.7],[0,0,0,0,0,0],'.95 .61 .08 1')
            for x in [-.5,.5]:
                for y in [-.45,.45]:part(l,f'wheel_{x}_{y}','cylinder',[.2,.12],[x,y,-.35,math.pi/2,0,0],'.08 .09 .10 1')
    if fixture:
        p=target(actors[0],3);p[2]=.45;truth['fixture_start']=p
        l=model(w,'contact_fixture',p,True,2);l.find('gravity').text='true'
        part(l,'test_cube','box',[.5,.5,.9],[0]*6,'.95 .1 .5 1')
        s=ET.SubElement(l,'sensor',name='contact',type='contact');c=ET.SubElement(s,'contact');text(c,'collision','test_cube');text(c,'topic','/s02/evaluation/contacts');text(s,'always_on','true');text(s,'update_rate',50)
    l=model(w,'validation_probe',cfg['probe']['position'])
    s=ET.SubElement(l,'sensor',name='lidar',type='gpu_lidar');text(s,'topic','/s02/probe/lidar');text(s,'update_rate',10);text(s,'always_on','true')
    ld=ET.SubElement(s,'lidar');scan=ET.SubElement(ld,'scan')
    for name,n,lo,hi in [('horizontal',512,-1.2,1.2),('vertical',64,-.3,.2)]:
        sc=ET.SubElement(scan,name)
        for k,v in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]:text(sc,k,v)
    ran=ET.SubElement(ld,'range')
    for k,v in [('min',.08),('max',40),('resolution',.01)]:text(ran,k,v)
    s=ET.SubElement(l,'sensor',name='camera',type='camera');text(s,'topic','/s02/probe/image');text(s,'pose','0 0 0 0 .08 0');text(s,'update_rate',5);text(s,'always_on','true')
    c=ET.SubElement(s,'camera');text(c,'horizontal_fov',1.4);im=ET.SubElement(c,'image');text(im,'width',640);text(im,'height',360);text(im,'format','R8G8B8');cl=ET.SubElement(c,'clip');text(cl,'near',.05);text(cl,'far',100)
    worldpath.parent.mkdir(parents=True,exist_ok=True);ET.indent(sdf);ET.ElementTree(sdf).write(worldpath,encoding='utf-8',xml_declaration=True)
    dump(out/'evaluation/truth_manifest.json',truth)
    resources=[dict(path=str(p.relative_to(out)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(assets.rglob('*')) if p.is_file()]
    dump(out/'evaluation/resource_manifest.json',dict(resources=resources,seed=seed,variant=variant,license='Original generated geometry; no remote model downloads'))
    return dict(world=str(worldpath),areas=[p.area for p in zones],road_length=line.length,map_id=public_hash)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=BASE);p.add_argument('--seed',type=int,default=41);p.add_argument('--variant',choices=['base','notch','holes'],default='base');p.add_argument('--fixture',action='store_true');a=p.parse_args();print(json.dumps(generate(a.out,a.seed,a.variant,a.fixture),indent=2))
