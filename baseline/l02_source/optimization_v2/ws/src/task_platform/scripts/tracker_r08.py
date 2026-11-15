"""Current point-cloud tracks projected onto the actual active navigation path."""
from common import BASE
import argparse,json,math,signal,sys,copy
from pathlib import Path
import numpy as np
sys.path.insert(0,str(BASE/'ws/src/object_fusion/scripts'))
from tracker import Tracker
from prefix_r08 import validate
from crossing_r08 import crossing,MotionMemory
from activity_r08 import ActivityMemory,apply_grid,route_conflict
from shapely.geometry import LineString,Point
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid,Path as RosPath

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);cfg=json.loads(a.config.read_text());tracker=Tracker();motion_memory=MotionMemory();activity=ActivityMemory();rclpy.init();n=rclpy.create_node('r08_observed_leader',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);st=dict(stop=False,loc=None,grid=None,path=np.asarray(cfg['route_map']),path_epoch=0.);pub=n.create_publisher(String,'/r08/leader',10);riskpub=n.create_publisher(OccupancyGrid,'/r08/dynamic_risk',2);log=(a.out/'tracks.jsonl').open('w')
 n.create_subscription(String,'/r07/localization',lambda m:st.update(loc=json.loads(m.data)),10);n.create_subscription(OccupancyGrid,'/r07/risk_local',lambda m:st.update(grid=m),2)
 def path(m):
  if m.header.frame_id!='map' or len(m.poses)<2:return
  rows=[]
  for p in m.poses:
   q=p.pose.orientation;rows.append([p.pose.position.x,p.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
  st.update(path=np.asarray(rows),path_epoch=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
 n.create_subscription(RosPath,'/r08/active_path',path,10)
 def geometry(m):
  d=json.loads(m.data);t=d['t']
  if tracker.last is not None and t<=tracker.last:return
  tracks,ambiguous=tracker.update(t,d['objects']);recent_motion=motion_memory.update(t,tracks);loc=st['loc'];result=dict(t=t,valid=False,on_path=False,tracks=tracks,ambiguous=ambiguous,path_valid=False,path_epoch=st['path_epoch'],source='CURRENT_PCL_GEOMETRY_AND_MEASURED_ODOMETRY',truth_used=False,predicted_conflict=False,crossing_evidence=[])
  if loc and loc.get('valid') and abs(t-loc['t'])<.3:
   tr=loc['transform'];co,si=math.cos(tr[2]),math.sin(tr[2]);R=np.array([[co,-si],[si,co]]);ego=np.array(loc['pose'][:2]);original_line=LineString(st['path'][:,:2]);end=st['path'][-1,:2];delta=end-st['path'][-2,:2];tangent_end=delta/max(1e-6,np.linalg.norm(delta));line=LineString(np.vstack([st['path'][:,:2],end+7*tangent_end]));ego_arc=line.project(Point(ego));candidates=[];activity_eligible=set()
   for track in tracks:
    pos=R@np.array(track['position'][:2])+tr[:2];obj_arc=line.project(Point(pos));along=obj_arc-ego_arc;radius=max(track['size'][:2])/2;lateral=line.distance(Point(pos));gap=along-radius-max(x for x,y in cfg['vehicle']['footprint'])
    if track['confirmed'] and track['state']=='OBSERVED' and track['observed_age_s']<.25:
     rotation=np.zeros((4,4));rotation[:2,:2]=R;rotation[2:,2:]=R;C=rotation@np.array(track['covariance']).reshape(4,4)@rotation.T;future=crossing(st['path'],ego,max(.15,loc.get('v',0)),pos,R@np.array(track['velocity']),C,track['size'],cfg['vehicle']['footprint'],recent_motion=track['id'] in recent_motion)
     future['recent_observed_motion']=track['id'] in recent_motion
     if track['id'] in recent_motion and future.get('reason')!='SAME_DIRECTION_LEADER_BRANCH':activity_eligible.add(track['id'])
     if future['conflict']:result['predicted_conflict']=True;result['crossing_evidence'].append(dict(id=track['id'],**future))
    if 0<along<7 and lateral<max(abs(y) for x,y in cfg['vehicle']['footprint'])+.1+radius and track['confirmed']:
     a0=np.asarray(line.interpolate(max(0,obj_arc-.1)).coords[0]);b0=np.asarray(line.interpolate(min(line.length,obj_arc+.1)).coords[0]);tangent=(b0-a0)/max(1e-6,np.linalg.norm(b0-a0));normal=np.array([-tangent[1],tangent[0]]);velocity=R@np.array(track['velocity']);speed=float(velocity@tangent);side=float(velocity@normal);hits=tracker.tracks[track['id']]['hits'];beyond_goal=obj_arc>original_line.length+.1
     if beyond_goal and not (speed>.05 and abs(side)<.15 or track['id']==st.get('last_leader')):continue
     confidence=min(.98,.5+.08*hits) if track['observed_age_s']<.25 and track['state']=='OBSERVED' else 0.;candidates.append(dict(id=track['id'],gap=gap,lateral=lateral,speed=speed,same_direction=speed>.05 and abs(side)<.15,confidence=confidence,ttc=gap/max(.001,loc.get('v',0)-speed),state=track['state']))
   envelope=activity.update(t,tracks,activity_eligible);odom_path=(st['path'][:,:2]-np.asarray(tr[:2]))@R;dynamic=route_conflict(envelope,odom_path,loc['odom'][:2],cfg['vehicle']['footprint']);result.update(activity_evidence=dynamic,activity_clear_ahead_m=dynamic['clear_ahead_m']);result['predicted_conflict']=result['predicted_conflict'] or dynamic['conflict']
   grid=st['grid'];evidence=dict(valid=False,reason='OBSERVATION_STALE')
   if grid and abs(t-(grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9))<.3 and grid.header.frame_id=='odom':
    cost=apply_grid(np.asarray(grid.data).reshape(grid.info.height,grid.info.width),[grid.info.origin.position.x,grid.info.origin.position.y],grid.info.resolution,envelope);combined=copy.deepcopy(grid);combined.data=cost.astype('int8').ravel().tolist();riskpub.publish(combined);evidence=validate(cost,[grid.info.origin.position.x,grid.info.origin.position.y],grid.info.resolution,loc['odom'],loc['pose'],tr,st['path'],cfg['vehicle']['footprint'])
   result.update(valid=True,path_valid=evidence['valid'],prefix_evidence=evidence)
   if candidates:
    chosen=min(candidates,key=lambda x:x['gap']);result.update(on_path=True,leader=chosen);st['last_leader']=chosen['id']
  pub.publish(String(data=json.dumps(result)));log.write(json.dumps(result)+'\n');log.flush();tmp=a.out/'live.tmp';tmp.write_text(json.dumps(result));tmp.replace(a.out/'live.json')
 n.create_subscription(String,'/s09/geometry',geometry,10)
 def stop(s,f):st['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (a.out/'ready').write_text('geometry tracking and current executed prefix; no fixture subscription')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.01)
 finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
