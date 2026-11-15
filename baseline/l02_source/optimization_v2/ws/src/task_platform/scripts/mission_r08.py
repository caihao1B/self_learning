"""FollowPath inspection with observed replanning, bounded retries and honest residuals."""
import argparse,json,time,signal,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import FollowPath,ComputePathToPose
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();out=a.out;cfg=json.loads((out/'mission.json').read_text());spec=json.loads((out/'spec.json').read_text());full=np.asarray(cfg['route_map']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(full[:,:2],axis=0),axis=1))];limit=spec['distance_limit_m'];count=len(full) if limit<=0 else max(2,int(np.searchsorted(arc,limit,side='right')));route=full[:count];selected=float(arc[count-1]);rclpy.init();n=rclpy.create_node('r08_inspection_task',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);nav=ActionClient(n,FollowPath,'/follow_path');planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');manager=n.create_client(Trigger,'/lifecycle_manager/is_active');pub=n.create_publisher(String,'/r07/mission',10);pathpub=n.create_publisher(RosPath,'/r08/active_path',10);hp=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10)
 st=dict(t=0.,stop=False,phase='STARTUP',nav_idle=True,complete=False,nav_failed=False,replans=0,deferrals=0,alternative=False,index=0,bt=None,gate=None,loc=None,tracking=None,patch_active=False,path_epoch=0.);startwall=time.monotonic();active=None;hinted=False;error=None;manual_reason=None;patch=None;patch_rejoin=None;prepared_goal=None;last_progress_t=None;last_progress_arc=0.;trace=(out/'inspection_trace.jsonl').open('w');terminal='FAULT';motion_started=False;published_reference=None
 def log(event,**d):
  with (out/'mission_events.jsonl').open('a') as f:f.write(json.dumps(dict(t=st['t'],event=event,**d))+'\n')
 n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 for topic,key in [('/r06/directive','bt'),('/r07/gate','gate'),('/r07/localization','loc'),('/r08/leader','tracking')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20)
 def publish():
  nonlocal last_progress_t,last_progress_arc
  loc=st['loc'] or {}
  if loc.get('valid') and 'pose' in loc:
   lo=max(0,st['index']-2);hi=min(len(route),st['index']+80);i=lo+int(np.argmin(np.linalg.norm(route[lo:hi,:2]-np.asarray(loc['pose'][:2]),axis=1)));st['index']=max(st['index'],i);trace.write(json.dumps(dict(t=st['t'],pose=loc['pose'],route_index=i,estimated_arc_m=float(arc[i]),patch_active=st['patch_active']))+'\n');trace.flush()
   if arc[st['index']]-last_progress_arc>=.1:last_progress_arc=float(arc[st['index']]);last_progress_t=st['t']
  keys=['t','phase','nav_idle','complete','nav_failed','replans','deferrals','alternative','index','patch_active','path_epoch'];d={k:st[k] for k in keys};d.update(selected_length_m=selected,original_length_m=float(arc[-1]));pub.publish(String(data=json.dumps(d)));tmp=out/'live.tmp';tmp.write_text(json.dumps(dict(d,bt=st['bt'],gate=st['gate'],loc=st['loc'])));tmp.replace(out/'live.json')
 n.create_timer(.03,publish)
 def tick(allow_stop=False):
  nonlocal hinted
  rclpy.spin_once(n,timeout_sec=.01)
  if st['t']>8 and not hinted:
   h=spec['operator_hint'];m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=h[0];m.pose.pose.position.y=h[1];m.pose.pose.orientation.z=math.sin(h[2]/2);m.pose.pose.orientation.w=math.cos(h[2]/2);m.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hp.publish(m);hinted=True;log('COARSE_PUBLIC_ROUTE_HINT')
  if not allow_stop:
   if st['stop']:raise InterruptedError('SESSION_STOP_REQUEST')
   if st['t']>spec['sim_seconds'] or time.monotonic()-startwall>spec['wall_budget']-30:raise TimeoutError('TASK_TIME_BUDGET')
   if motion_started and last_progress_t is not None and st['t']-last_progress_t>45:raise TimeoutError('TASK_PROGRESS_TIMEOUT_ACROSS_HOLDS')
 def wait(pred,seconds=20,allow_stop=False):
  begin=st['t'];wall=time.monotonic()
  while not pred():
   tick(allow_stop)
   if st['t']-begin>seconds or time.monotonic()-wall>max(20,seconds*4):raise TimeoutError('WAIT_DEADLINE')
 def result(f,seconds=20):wait(f.done,seconds);return f.result()
 def cancel():
  nonlocal active
  if active:
   f=active.cancel_goal_async();wait(f.done,8,True);log('FOLLOW_PATH_CANCEL_ACK',accepted=bool(f.result() and f.result().goals_canceling));f=active.get_result_async();wait(f.done,8,True);active=None
  st.update(nav_idle=True,phase='STOPPING');wait(lambda:bool(st['gate'] and st['gate']['stopped']),5,True)
 def pose(x,y,h):
  p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);return p
 def path_goal(points):
  nonlocal published_reference
  p=np.asarray(st['loc']['pose']);points=np.asarray(points);i=int(np.argmin(np.linalg.norm(points[:,:2]-p[:2],axis=1)))
  if math.dist(p[:2],points[i,:2])>.65:raise RuntimeError('OUTSIDE_QUALIFIED_ROUTE_CORRIDOR')
  g=FollowPath.Goal();g.controller_id='RPP';g.goal_checker_id='goal';g.progress_checker_id='progress';g.path.header.frame_id='map';g.path.header.stamp=n.get_clock().now().to_msg();g.path.poses=[pose(*p)]+[pose(*q) for q in points[min(i+1,len(points)-1):]]
  reference=patch if patch is not None else route
  if published_reference is None or not np.array_equal(published_reference,reference):
   reference_msg=RosPath();reference_msg.header=g.path.header;reference_msg.poses=[pose(*q) for q in reference];st['path_epoch']=g.path.header.stamp.sec+g.path.header.stamp.nanosec*1e-9;pathpub.publish(reference_msg);published_reference=reference.copy();log('ACTIVE_REFERENCE_CHANGED',epoch=st['path_epoch'],poses=len(reference),reference_map=reference.tolist(),patch=patch is not None)
  return g
 def bt_ready():return bool(st['bt'] and st['bt']['action'] in ['EXECUTE','FOLLOW','DETOUR','REPLAN','HUMAN','STOP'])
 def replan_query():
  nonlocal patch,patch_rejoin,manual_reason,prepared_goal
  cancel()
  if st['replans']>=2:manual_reason='BOUNDED_REPLAN_EXHAUSTED';raise RuntimeError(manual_reason)
  st.update(phase='REPLANNING',patch_active=False);patch=None;st['replans']+=1;leader=(st['tracking'] or {}).get('leader',{});rejoin_ahead=max(6.,min(20.,max(leader.get('gap',0.)+max(x for x,y in cfg['vehicle']['footprint'])+2.,(st['tracking'] or {}).get('activity_clear_ahead_m',0.))));patch_rejoin=min(len(route)-1,int(np.searchsorted(arc,arc[st['index']]+rejoin_ahead)));wait(planner.server_is_ready,5);g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal=pose(*route[patch_rejoin]);h=result(planner.send_goal_async(g),5)
  if not h.accepted:log('PLANNER_QUERY_REJECTED',attempt=st['replans']);st['nav_failed']=True;return False
  r=result(h.get_result_async(),15);log('PLANNER_QUERY_RESULT',status=r.status,poses=len(r.result.path.poses),attempt=st['replans'],rejoin_index=patch_rejoin,candidate_admission='PRIOR_PLUS_CURRENT_HAZARDS; EXECUTION_PREFIX_REQUIRES_SEPARATE_OBSERVATIONS')
  if r.status!=4 or len(r.result.path.poses)<2:st['nav_failed']=True;return False
  patch=np.array([[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in r.result.path.poses]);st.update(patch_active=True,nav_failed=False,phase='VALIDATING_PATCH');prepared_goal=path_goal(patch);epoch=st['path_epoch'];start=st['t']
  while st['t']-start<3:
   tick();tr=st['tracking'] or {}
   if tr.get('path_valid') and abs(tr.get('path_epoch',-1)-epoch)<.001 and 0<=st['t']-tr.get('t',-100)<.35:
    log('OBSERVED_PATCH_ADMITTED',epoch=epoch,rejoin_index=patch_rejoin,prefix=tr.get('prefix_evidence'),remaining_candidate='NOT_CURRENTLY_CERTIFIED');return True
  log('PATCH_PREFIX_REJECTED',evidence=(st['tracking']or{}).get('prefix_evidence'));patch=None;st.update(patch_active=False,nav_failed=True);return False
 def stop(s,f):st['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (out/'observer_ready').write_text('real FollowPath and bounded planner actions; no tool/actuator publisher')
 try:
  wait(lambda:nav.server_is_ready() and bool(st['loc'] and st['loc']['valid']) and bool(st['gate'] and st['gate']['tool_stopped']),75);wait(manager.service_is_ready,15);begin=st['t']
  while not result(manager.call_async(Trigger.Request()),5).success:
   if st['t']-begin>35:raise TimeoutError('NAV_STARTUP')
   t=st['t'];wait(lambda:st['t']>t+.3,2)
  log('NAV_LIFECYCLE_ACTIVE_VERIFIED')
  while not st['complete']:
   st['phase']='READY';wait(bt_ready,60)
   action=st['bt']['action']
   if action=='REPLAN':
    if not replan_query():
     if st['replans']>=2:manual_reason='NO_VALIDATED_ROUTE_WITHIN_BOUNDED_SEARCH';raise RuntimeError(manual_reason)
     t=st['t'];wait(lambda:st['bt'].get('evidence',{}).get('stamp',-1)>t,3)
     continue
    wait(lambda:st['bt'] and st['bt']['action'] in ['DETOUR','HUMAN','STOP'],5);action=st['bt']['action']
   if action in ['HUMAN','STOP']:manual_reason='BT_'+st['bt'].get('reason_code','HOLD');raise RuntimeError(manual_reason)
   points=patch if patch is not None else route[st['index']:];goal=prepared_goal if prepared_goal is not None else path_goal(points);prepared_goal=None
   epoch=st['path_epoch'];wait(lambda:bool(st['tracking'] and abs(st['tracking'].get('path_epoch',-1)-epoch)<.001 and 0<=st['t']-st['tracking'].get('t',-100)<.35),4)
   barrier=st['t'];wait(lambda:bt_ready() and st['bt'].get('evidence',{}).get('stamp',-1)>barrier,60)
   if st['bt']['action'] not in ['EXECUTE','FOLLOW','DETOUR']:continue
   st.update(phase='NAVIGATING',nav_idle=False);active=result(nav.send_goal_async(goal),10)
   if not active.accepted:active=None;st.update(nav_idle=True);raise RuntimeError('FOLLOW_PATH_REJECTED')
   if not motion_started:motion_started=True;last_progress_t=st['t']
   log('FOLLOW_PATH_GOAL',start_index=st['index'],poses=len(goal.path.poses),selected_length_m=selected,patch=patch is not None,path_epoch=st['path_epoch']);future=active.get_result_async();interrupted=False;started=st['t']
   while not future.done():
    tick();bt=st['bt'] or {}
    if bt.get('action') in ['CANCEL','WAIT','REPLAN','HUMAN','STOP','DEFER'] and st['t']-started>.5:
     log('BT_INTERRUPTS_PATH',action=bt['action'],reason=bt.get('reason_code'));interrupted=True;cancel();break
   if future.done() and future.result().status==4:
    active=None;st.update(nav_idle=True,phase='STOPPING');wait(lambda:bool(st['gate'] and st['gate']['stopped']),5)
    target=patch[-1] if patch is not None else route[-1];distance=math.dist(st['loc']['pose'][:2],target[:2]);log('FOLLOW_PATH_SUCCESS',endpoint_error_m=distance,patch=patch is not None)
    if distance>=.2:raise RuntimeError('ARRIVAL_NOT_MEASURED')
    if patch is not None:log('PATCH_REJOIN_CONFIRMED',index=st['index'],planned_index=patch_rejoin);patch=None;st.update(patch_active=False,nav_failed=False);t=st['t'];wait(lambda:st['bt'].get('evidence',{}).get('stamp',-1)>t,3)
    else:
     st['complete']=st['index']>=len(route)-5
     if not st['complete']:raise RuntimeError('ROUTE_PROGRESS_NOT_MEASURED')
   else:
    if active:cancel()
    st.update(phase='OBSERVATION_HOLD' if interrupted else 'NAV_FAILED',nav_failed=not interrupted);failure_stamp=st['t'];allowed_next=['EXECUTE','FOLLOW','DETOUR','REPLAN','HUMAN','STOP'] if interrupted else ['REPLAN','HUMAN','STOP'];log('AWAIT_FRESH_BT_AFTER_PATH_RESULT',interrupted=interrupted,minimum_context_stamp=failure_stamp,allowed_next=allowed_next);wait(lambda:bool(st['bt'] and st['bt'].get('evidence',{}).get('stamp',-1)>failure_stamp and st['bt']['action'] in allowed_next),60);log('RESUME_REMAINING_ROUTE',index=st['index'],selected=st['bt']['action'])
  st['phase']='DONE';end=st['t'];wait(lambda:st['t']>end+2,4);terminal='COMPLETED'
 except InterruptedError as ex:error=repr(ex);terminal='CANCELLED';log('TASK_STOP',error=error)
 except BaseException as ex:
  error=repr(ex);manual_reason=manual_reason or str(ex);terminal='HUMAN_REQUIRED' if isinstance(ex,(RuntimeError,TimeoutError)) else 'FAULT'
  if 'CANCEL_OR_STOP_CONFIRMATION_TIMEOUT' in manual_reason:terminal='FAULT'
  log('TASK_STOP',error=error,manual_reason=manual_reason,terminal=terminal)
 finally:
  st['phase']='STOPPING'
  try:cancel()
  except BaseException as ex:terminal='FAULT';error=error or repr(ex);log('STOP_UNCONFIRMED',error=repr(ex))
  ownership={topic:[i.node_name for i in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/s03/actuator/request','/s10/tool/command']};final=dict(passed=terminal=='COMPLETED' and st['complete'],terminal=terminal,controlled_hold=terminal=='HUMAN_REQUIRED',error=error,manual_reason=manual_reason,t=st['t'],selected_length_m=selected,original_length_m=float(arc[-1]),full_route_selected=count==len(full),remaining_original_m=max(0,float(arc[-1])-float(arc[st['index']])),route_index=st['index'],replans=st['replans'],final_gate=st['gate'],ownership=ownership,wall_seconds=time.monotonic()-startwall,scope='Inspection navigation; residuals remain; no spray/mow credit');(out/'mission_result.json').write_text(json.dumps(final,indent=2));trace.close();n.destroy_node();rclpy.try_shutdown()
 return 0 if terminal in ['COMPLETED','HUMAN_REQUIRED'] else 2
if __name__=='__main__':raise SystemExit(main())
