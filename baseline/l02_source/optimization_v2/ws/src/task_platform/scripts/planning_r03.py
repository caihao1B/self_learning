"""Versioned task scope, connected METIS clusters and footprint-checked routes."""
from common import *
import math,heapq,time,importlib.util,yaml
import reeds_r03
import numpy as np
import shapely
from shapely.geometry import shape,mapping,Polygon,Point,LineString
from shapely import union_all,set_precision
from scipy.interpolate import CubicSpline
sys.path.insert(0,str(BASE/'ws/src/coverage_partition/scripts'))
from geometry11 import build_graph,polygons
from partition11 import solve
sys.path.insert(0,str(BASE/'ws/src/coverage_planner/scripts'))
from common12 import backend
from spray12 import nearest_2opt
spec=importlib.util.spec_from_file_location('r03_dubins',ROOT/'offline_coverage/motion.py');dubins=importlib.util.module_from_spec(spec);spec.loader.exec_module(dubins)
DS=.10;MARGIN=.10

def curve_line(a,b):
 a=np.array(a);b=np.array(b);d=b-a;theta=math.atan2(d[1],d[0]);xy=np.linspace(a,b,max(2,math.ceil(np.linalg.norm(d)/DS)+1));return np.c_[xy,np.full(len(xy),theta)].tolist()

def footprint_ok(poses,c,allowed):
 p=np.asarray(poses);local=np.asarray(c['footprint']);co=np.cos(p[:,2]);si=np.sin(p[:,2]);x=p[:,0,None]+co[:,None]*local[:,0]-si[:,None]*local[:,1];y=p[:,1,None]+si[:,None]*local[:,0]+co[:,None]*local[:,1]
 # Sampled swept envelope with numerical margin larger than the half-step.
 hulls=shapely.polygons(np.stack([x,y],axis=2));return bool(np.all(shapely.covers(allowed.buffer(-MARGIN),hulls)))

def segment(poses,kind,c,tool=False,**extras):
 a=np.asarray(poses);ds=np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1);dy=np.abs(np.arctan2(np.sin(np.diff(a[:,2])),np.cos(np.diff(a[:,2]))));length=float(ds.sum());speed=(.25 if extras.get('direction',1)<0 else .5) if c['task_id']=='mowing' else .4
 return dict(kind=kind,poses=poses,tool_on=tool,length_m=length,nominal_seconds=length/speed+float(dy.sum())/.4+2.,**extras)

def visibility(nodes,safe):
 corners=[]
 for poly in polygons(safe):
  for ring in [poly.exterior,*poly.interiors]:corners.extend(list(ring.coords)[:-1])
 xy=np.asarray([*nodes,*corners]);adj=[[] for _ in xy];allowed=safe.buffer(1e-8)
 for i in range(len(xy)):
  js=np.arange(i+1,len(xy))
  if not len(js):continue
  lines=shapely.linestrings(np.stack([np.tile(xy[i],(len(js),1)),xy[js]],axis=1));mask=shapely.covers(allowed,lines)
  for j in js[mask]:d=float(np.linalg.norm(xy[i]-xy[j]));adj[i].append((int(j),d));adj[j].append((i,d))
 costs=[];paths={}
 for i in range(len(nodes)):
  dist=[math.inf]*len(xy);dist[i]=0;prev={};q=[(0.,i)]
  while q:
   d,u=heapq.heappop(q)
   if d>dist[u]+1e-10:continue
   for v,w in adj[u]:
    if d+w<dist[v]-1e-10:dist[v]=d+w;prev[v]=u;heapq.heappush(q,(d+w,v))
  if not all(math.isfinite(x) for x in dist[:len(nodes)]):raise ValueError('DISCONNECTED_SERVICE_PATH')
  costs.append(dist[:len(nodes)])
  for j in range(len(nodes)):
   ids=[j]
   while ids[-1]!=i:ids.append(prev[ids[-1]])
   paths[i,j]=xy[list(reversed(ids))].tolist()
 return np.asarray(costs),paths

def spray_plan(meta,g,labels,c,out):
 safe=meta['motion'].buffer(-max(math.hypot(*p) for p in c['footprint'])-.15,join_style=2);plans=[]
 for cluster in range(3):
  points=[p for p in g['points'] if int(labels[p['unit']])==cluster];nodes=[p['xy'] for p in points]
  if len(nodes)<2:raise ValueError('EMPTY_SERVICE_CLUSTER')
  if not all(safe.covers(Point(p)) for p in nodes):raise ValueError('SERVICE_CENTER_NOT_REACHABLE')
  cost,paths=visibility(nodes,safe);baseline,bval=nearest_2opt(cost);ort=backend(dict(mode='tsp',cost=np.rint(cost*1000).astype(int).tolist()),out,f'tsp_{cluster}')
  orders=[('nn_2opt',baseline),('ortools',ort['order'])];method,order=min(orders,key=lambda x:sum(cost[x[1][i],x[1][(i+1)%len(x[1])]] for i in range(len(x[1]))))
  segs=[];current=[*nodes[order[0]],0.]
  for index,point_id in enumerate(order):
   p=points[point_id];segs.append(segment([list(current)],'POINT_SERVICE',c,True,point_id=p['id'],target=p['xy'],action_seconds=3.));dest=order[(index+1)%len(order)]
   for a,b in zip(paths[point_id,dest],paths[point_id,dest][1:]):
    if math.dist(a,b)<1e-8:continue
    line=curve_line(a,b);target=line[0][2];dy=math.atan2(math.sin(target-current[2]),math.cos(target-current[2]));n=max(2,math.ceil(abs(dy)/.08)+1)
    turn=[[current[0],current[1],current[2]+z*dy] for z in np.linspace(0,1,n)]
    if not footprint_ok(turn,c,meta['motion']) or not footprint_ok(line,c,meta['motion']):raise ValueError('SPRAY_SWEEP_INVALID')
    segs.extend([segment(turn,'ROTATE',c),segment(line,'TRANSIT',c)]);current=line[-1]
  plans.append(dict(cluster=f'A{cluster+1}',task='spray',method=method,segments=segs,start=segs[0]['poses'][0],end=current,service_points=points,point_count=len(points),planned_point_count=len(order),unplanned_point_ids=[]))
 return plans

def mowing_plan(meta,g,labels,c,out):
 domain=meta['domain'];raw=backend(dict(mode='swaths',work=mapping(domain),motion=mapping(meta['motion']),spacing=1.40,headland=0),out,'whole_zone_swaths');regions=[union_all([p for i,p in enumerate(g['cells']) if labels[i]==k]) for k in range(3)];radius=c['physical_contract']['nominal_bicycle_radius_m']+.10;plans=[];coverage=[]
 def lines(geom):
  if geom.is_empty:return []
  if geom.geom_type=='LineString':return [geom]
  return [l for part in getattr(geom,'geoms',[]) for l in lines(part)]
 for cluster,region in enumerate(regions):
  work=[]
  for row in raw['swaths']:
   for line in lines(LineString([row['a'],row['b']]).intersection(region)):
    if line.length<.5:continue
    a=np.asarray(line.coords[0]);b=np.asarray(line.coords[-1]);options=[]
    for first,last in [(a,b),(b,a)]:
     u=(last-first)/np.linalg.norm(last-first);offset=c['tool']['mount_xyz'][0]*u;poses=curve_line(first-offset,last-offset)
     if footprint_ok(poses,c,meta['motion']):options.append(segment(poses,'COVERAGE',c,True,row_id=row['id']))
    if options:work.append(dict(id=len(work),options=options))
  if not work:raise ValueError('NO_MOWING_SWATHS')
  # The same complete task zone is partitioned; cluster boundaries are accounting only.
  current=min(work,key=lambda w:tuple(w['options'][0]['poses'][0]))['options'][0]['poses'][0];start=list(current);segments=[];remaining={w['id']:w for w in work};unconnected=[]
  while remaining:
   choices=sorted([(math.dist(current[:2],o['poses'][0][:2])+radius*abs(dubins.wrap(current[2]-o['poses'][0][2])),wid,o) for wid,item in remaining.items() for o in item['options']],key=lambda x:(x[0],x[1]))
   selected=None;valid=[];options=choices[:32]
   rs=reeds_r03.query(current,[o['poses'][0] for _,wid,o in options],radius,out/f'rs_{cluster}')
   for (_,wid,opt),proposal in zip(options,rs):
    goal=opt['poses'][0];chunks=reeds_r03.sample(current,proposal['primitives'],DS)
    if chunks and (math.dist(chunks[-1]['poses'][-1][:2],goal[:2])>1e-6 or abs(dubins.wrap(chunks[-1]['poses'][-1][2]-goal[2]))>1e-6):raise ValueError('RS_ENDPOINT_ERROR')
    if all(footprint_ok(ch['poses'],c,meta['motion']) for ch in chunks):
     score=sum(ch['length_m']/(.25 if ch['direction']<0 else .5)+4 for ch in chunks)
     valid.append((score,wid,opt,chunks))
   if valid:
    _,wid,opt,chunks=min(valid,key=lambda x:(x[0],x[1]));selected=(wid,opt,chunks)
   else:
    for _,wid,opt in options:
     turn=next((p for p in dubins.dubins(current,opt['poses'][0],radius,DS) if footprint_ok(p['poses'],c,meta['motion'])),None)
     if turn is not None:selected=(wid,opt,[dict(poses=turn['poses'],direction=1)]);break
   if selected is None:unconnected=list(remaining);break
   wid,opt,chunks=selected
   for ch in chunks:
    segments.append(segment(ch['poses'],'TURN_TOOL_OFF',c,False,turn_radius_m=radius,direction=ch['direction'],stop_before=True,stop_after=True,stop_at_curvature_change=True))
   segments.append(opt);current=opt['poses'][-1];del remaining[wid]
  cuts=[]
  for s in segments:
   if s['tool_on']:
    p=np.array(s['poses']);off=c['tool']['mount_xyz'][0];xy=p[:,:2]+off*np.c_[np.cos(p[:,2]),np.sin(p[:,2])];cuts.append(LineString(xy).buffer(c['tool']['effective_width_m']/2,cap_style=2))
  cut=union_all(cuts).intersection(region);coverage.append(cut)
  plans.append(dict(cluster=f'B{cluster+1}',task='mowing',method='Fields2Cover whole-zone swaths + OMPL Reeds-Shepp / Dubins with full-body checks',segments=segments,start=start,end=current,all_swaths_connected=not unconnected,unconnected_swath_ids=unconnected,work_area_m2=region.area,planned_cut_m2=cut.area,planned_fraction=cut.area/region.area,planned_residual=mapping(region.difference(cut)),swath_spacing_m=1.4,turn_radius_m=radius,global_swath_angle=raw['angle']))
 return plans,union_all(coverage)

def plan_all():
 require('R02');out=edition('R03');out.mkdir(parents=True,exist_ok=True);sources=read(BASE/'maps/prior/workspaces.geojson')['features'];permanent=union_all([shape(f['geometry']) for f in read(BASE/'maps/prior/permanent.geojson')['features']]);results={};partitioned={};excluded={};cfg=yaml.safe_load((BASE/'config/partition.yaml').read_text());cfg.update(cell_size_m=1.5,spray_point_spacing_m=3.,seed=17023,repair_balance_passes=8)
 for zone,task,inset,speed in [('A','spray',1.0,.4),('B','mowing',3.8,.5)]:
  full=shape(next(f['geometry'] for f in sources if f['properties']['id']==zone)).difference(permanent);domain=set_precision(full.buffer(-inset,join_style=2),1e-8)
  if domain.geom_type!='Polygon':raise ValueError('TASK_SCOPE_NOT_CONNECTED')
  meta=dict(zone=zone,variant='R03_independent_task',domain=domain,workspace=full,motion=full,centre=domain,speed_mps=speed,swath_spacing_m=1.4,service_s=3.,map_id=read(BASE/'maps/prior/map_metadata.json')['map_id'],frame='prior_local_enu')
  g=build_graph(domain,cfg,meta);sol=solve(meta,g,cfg,'metis',17023,out/zone/'partition');labels=np.array(sol['labels']);c=read(edition('R02')/'vehicles'/f'{task}.json');sol['metrics']['optimistic_total_lower_bound_s']=(domain.area/(1.4*speed) if zone=='B' else len(g['points'])*meta['service_s']);dump(out/zone/'partition/partition.json',sol);partitioned[zone]=sol;excluded[zone]=dict(original_free_area_m2=full.area,new_task_area_m2=domain.area,reserved_or_excluded_m2=full.difference(domain).area,geometry=mapping(full.difference(domain)),reason='explicit boundary buffer' if zone=='A' else 'shared physical headland; not credited as cut work')
  if task=='spray':plans=spray_plan(meta,g,labels,c,out/zone/'routes')
  else:
   plans,cut=mowing_plan(meta,g,labels,c,out/zone/'routes');dump(out/zone/'planned_cut.geojson',dict(type='Feature',geometry=mapping(cut),properties={'planned_only':True}))
  for plan in plans:
   plan['motion_m']=sum(s['length_m'] for s in plan['segments']);plan['work_m']=sum(s['length_m'] for s in plan['segments'] if s['tool_on']);plan['tool_off_m']=plan['motion_m']-plan['work_m'];plan['estimated_seconds']=1.25*sum(s['nominal_seconds']+s.get('action_seconds',0) for s in plan['segments'])+60
   plan['budget_s']=1800;plan['vehicle_hash']=c['config_hash'];plan['frame']='prior_local_enu';plan['execution_allowed']=False;plan['speed_qualification_pending']='R04';plan['localization_qualification_pending']='R05';plan['scope']='independent R03 task; original formal obligations preserved';plan['version']=canonical(plan)
  results[task]=plans
 # Inspection is one independent curved route, not a compulsory transfer between work jobs.
 road=read(BASE/'maps/prior/connector.geojson')['features'];xy=np.asarray(road[1]['geometry']['coordinates']);xy=xy[np.r_[True,np.linalg.norm(np.diff(xy,axis=0),axis=1)>1e-8]];s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))];spl=CubicSpline(s,xy,axis=0);ss=np.linspace(0,s[-1],math.ceil(s[-1]/DS)+1);d=spl(ss,1);poses=np.c_[spl(ss),np.arctan2(d[:,1],d[:,0])].tolist();c=read(edition('R02')/'vehicles/inspection.json');allowed=union_all([shape(f['geometry']) for f in sources]+[shape(road[0]['geometry'])]).difference(permanent)
 if not footprint_ok(poses,c,allowed):raise ValueError('INSPECTION_SWEEP_FAILED')
 seg=segment(poses,'INSPECTION',c);plan=dict(task='inspection',cluster='road',segments=[seg],start=poses[0],end=poses[-1],motion_m=seg['length_m'],estimated_seconds=seg['length_m']/.6*1.25+60,budget_s=1800,vehicle_hash=c['config_hash'],frame='prior_local_enu',execution_allowed=False,work_credit=False);plan['version']=canonical(plan);results['inspection']=[plan]
 manifest=dict(schema='R03-INDEPENDENT-TASKS/1',map_source=dict(path='optimization_v2/assets/worlds/irregular_farm.sdf',sha256=sha(BASE/'assets/worlds/irregular_farm.sdf')),task_plans=results,scope_changes=excluded,partitions={z:dict(verification=v['verification'],metrics=v['metrics'],assignment_sha256=v['assignment_sha256']) for z,v in partitioned.items()},formal_inputs_unchanged=True,execution_allowed=False,unknown_low_obstacles_used=False,hidden_truth_used=False)
 manifest['version']=canonical(manifest);dump(out/'mission.json',manifest);return manifest
