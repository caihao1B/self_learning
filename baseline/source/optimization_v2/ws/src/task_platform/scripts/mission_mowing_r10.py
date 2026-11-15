"""Segment-aware Ackermann worker; real Nav2/ToolTask actions, feedback-only credit."""
import argparse,json,time,signal,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import FollowPath,ComputePathToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path as NavPath
import ompl_endpoint_r10
from mission_interfaces.action import ToolTask
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Point32
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from groups_mowing_r10 import execution_groups,passage,retry_exhausted
from recovery_mowing_r10 import validate_candidate,yaw_error,observed_cursor,gear_runs,tracking_distance,retreat_prefix,terminal_straight
from arrival_mowing_r10 import stable_arrival
from start_work_r10 import wait_startup,working_feedback_ready
from coverage_rejoin_r10 import rejoin_candidates,forward_window
from revalidation_spray_r10 import path_hash
from progress_spray_r10 import stopped_decision
from safe_plan_r10 import work_identity

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);out=ap.parse_args().out;cfg=json.loads((out/'mission.json').read_text());spec=json.loads((out/'spec.json').read_text());toolcfg=json.loads((out/'scene/tool.json').read_text());cluster=next(x for x in cfg['clusters'] if x['cluster']==spec['cluster']);allsegments=cluster['segments'];eligible=allsegments[spec.get('segment_start',0):];selected=eligible[:spec['segment_limit']] if spec['segment_limit'] else eligible;segments=execution_groups(selected,cfg['navigation_execution']['mode'])
 rclpy.init();n=rclpy.create_node('r10_mowing_task',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);nav=ActionClient(n,FollowPath,'/follow_path');planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');pathcheck=n.create_client(IsPathValid,'/is_path_valid');tool=ActionClient(n,ToolTask,'/s10/tool_task');manager=n.create_client(Trigger,'/lifecycle_manager/is_active');pub=n.create_publisher(String,'/r07/mission',10);hintpub=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10)
 st=dict(t=0.,stop=False,phase='STARTUP',nav_idle=True,complete=False,nav_failed=False,replans=0,deferrals=0,alternative=False,index=0,segment='TRANSIT',segment_id='',direction=1,work_complete=False,bt=None,gate=None,loc=None,tool=None);active=None;active_tool=None;tool_future=None;start=time.monotonic();hinted=False;terminal='FAULT';error=None;completed=[];work_results=[];last_progress_t=None;last_p=None;active_group=None;checkpoint_index=0;passages=[]
 prepare=n.create_client(Trigger,'/r10/localization/prepare_revalidation');commit=n.create_client(Trigger,'/r10/localization/commit_revalidation');proofpub=n.create_publisher(String,'/r10/localization/plan_proof',10);revalidations={}
 def log(event,**d):
  with (out/'mission_events.jsonl').open('a') as f:f.write(json.dumps(dict(t=st['t'],event=event,**d))+'\n')
 n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 for topic,key in [('/r06/directive','bt'),('/r07/gate','gate'),('/r07/localization','loc'),('/s10/action/state','tool'),('/r07/safety','safety')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20)
 def publish():
  d={k:st[k] for k in ['t','phase','nav_idle','complete','nav_failed','replans','deferrals','alternative','index','segment','segment_id','direction','work_complete']};d['endpoint_recovery']=st.get('endpoint_recovery',False);d['execution_segment_id']=st['segment_id'];d['segment_id']=work_identity(active_group)[0] if active_group and st['segment']=='COVERAGE' else st['segment_id'];pub.publish(String(data=json.dumps(d)));tmp=out/'live.tmp';tmp.write_text(json.dumps(dict(d,bt=st['bt'],gate=st['gate'],loc=st['loc'],tool=st['tool'])));tmp.replace(out/'live.json')
 n.create_timer(.03,publish)
 def record_passage():
  nonlocal checkpoint_index
  loc=st['loc'] or {}
  if active_group is None or not loc.get('valid') or checkpoint_index>=len(active_group['members']):return
  member=active_group['members'][checkpoint_index];ok,d,h=passage(loc['pose'],member['endpoint'])
  if ok:
   record=dict(segment=member['id'],xy_error_m=d,yaw_error_rad=h,t=st['t']);passages.append(record);checkpoint_index+=1;log('ORIGINAL_SEGMENT_PASSAGE',segment=member['id'],xy_error_m=d,yaw_error_rad=h,stop_required=checkpoint_index==len(active_group['members']))
 def tick(allow_stop=False):
  nonlocal hinted,last_progress_t,last_p
  rclpy.spin_once(n,timeout_sec=.01)
  if st['t']>8 and not hinted:
   h=spec['operator_hint'];m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=h[0];m.pose.pose.position.y=h[1];m.pose.pose.orientation.z=math.sin(h[2]/2);m.pose.pose.orientation.w=math.cos(h[2]/2);m.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hintpub.publish(m);hinted=True;log('COARSE_PUBLIC_CLUSTER_HINT')
  loc=st['loc'] or {}
  if loc.get('valid'):
   if last_p is None or math.dist(last_p,loc['pose'][:2])>.1:last_p=loc['pose'][:2];last_progress_t=st['t']
  if st['phase']=='NAVIGATING':record_passage()
  if not allow_stop:
   if st['stop']:raise InterruptedError('SESSION_STOP')
   if st['t']>spec['sim_seconds'] or time.monotonic()-start>spec['wall_budget']-30:raise TimeoutError('TASK_BUDGET')
   if active and last_progress_t is not None and st['t']-last_progress_t>45:raise TimeoutError('NO_OBSERVED_PROGRESS')
 def wait(pred,seconds=20,allow_stop=False):
  t=st['t'];wall=time.monotonic()
  while not pred():
   tick(allow_stop)
   if st['t']-t>seconds or time.monotonic()-wall>max(20,seconds*4):raise TimeoutError('WAIT_DEADLINE')
 def result(f,seconds=20):wait(f.done,seconds);return f.result()
 def pose(p):
  m=PoseStamped();m.header.frame_id='map';m.pose.position.x=float(p[0]);m.pose.position.y=float(p[1]);m.pose.orientation.z=math.sin(p[2]/2);m.pose.orientation.w=math.cos(p[2]/2);return m
 def tool_finish_result():
  nonlocal active_tool,tool_future
  if tool_future is not None:
   wait(tool_future.done,5,True);r=tool_future.result().result;d=dict(segment=st['segment_id'],state=r.state,reason=r.reason,confirmed=r.confirmed_quantity,remaining=r.remaining_quantity,uncertain=r.uncertainty_pending);work_results.append(d);log('TOOL_RESULT',**d);active_tool=None;tool_future=None
   if d['uncertain']:raise RuntimeError('TOOL_STOP_UNCONFIRMED')
   return d
 def stop_all():
  nonlocal active
  st['phase']='STOPPING';nav_cancel=active.cancel_goal_async() if active else None;tool_cancel=active_tool.cancel_goal_async() if active_tool else None
  for f in [nav_cancel,tool_cancel]:
   if f is not None:wait(f.done,5,True)
  if active:
   f=active.get_result_async();wait(f.done,5,True);active=None
  st['nav_idle']=True;tool_finish_result();wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped']),5,True);log('BOTH_PHYSICAL_STOPS_CONFIRMED',segment=st['segment_id'])
 def localization_hold():return bool((st['loc'] or {}).get('native_supervisor',{}).get('plan_revalidation_required'))
 def recover_localization(s):
  if not localization_hold():return
  if (st['bt'] or {}).get('action') in ['HUMAN','STOP']:raise RuntimeError('LOCALIZATION_MANUAL_LATCH_RETAINED')
  revalidations[s['id']]=revalidations.get(s['id'],0)+1
  if revalidations[s['id']]>cfg['stopped_revalidation']['max_attempts_per_segment']:raise RuntimeError('LOCALIZATION_REVALIDATION_BOUND')
  stop_all();st.update(phase='RELOCALIZING',nav_idle=True);log('STOPPED_RELOCALIZATION_REQUEST',segment=s['id'],attempt=revalidations[s['id']])
  wait(lambda:bool((st['loc'] or {}).get('native_supervisor',{}).get('recovery_ready')),cfg['stopped_revalidation']['observation_window_s'])
  if (st['bt'] or {}).get('action') in ['HUMAN','STOP']:raise RuntimeError('LOCALIZATION_MANUAL_LATCH_RETAINED')
  wait(prepare.service_is_ready,3);answer=result(prepare.call_async(Trigger.Request()),3)
  if not answer.success:raise RuntimeError('LOCALIZATION_PREPARE_REJECTED '+answer.message)
  prepared=json.loads(answer.message);epoch=prepared['epoch'];log('LOCALIZATION_PREVIEW_PREPARED',prepared_t=prepared['t'],epoch=epoch,reason=prepared['reason'])
  wait(lambda:bool(st['loc'].get('planning_valid') and st.get('safety') and st['safety'].get('valid') and st['safety'].get('localization_epoch')==epoch and st['safety']['t']>prepared['t']),8)
  start_pose=list(st['loc']['pose']);target=s['poses'][-1];g=ComputePathToPose.Goal();g.planner_id='GridBased';g.use_start=True;g.start=pose(start_pose);g.goal=pose(target)
  wait(planner.server_is_ready,5);h=result(planner.send_goal_async(g),5)
  if not h.accepted:raise RuntimeError('REVALIDATION_PLANNER_REJECTED')
  answer=result(h.get_result_async(),12)
  if answer.status!=4:raise RuntimeError('REVALIDATION_PLANNER_FAILED')
  path=answer.result.path;points=[]
  for m in path.poses:
   q=m.pose.orientation;points.append([m.pose.position.x,m.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
  geometric,why=validate_candidate(points,start_pose,target,cfg,cfg['stopped_revalidation']['maximum_path_m'])
  if not geometric:raise RuntimeError('REVALIDATION_GEOMETRY_'+why)
  gear_runs(points);wait(pathcheck.service_is_ready,5);req=IsPathValid.Request();req.path=path;collision=result(pathcheck.call_async(req),5)
  log('STOPPED_PLAN_REVALIDATION',segment=s['id'],epoch=epoch,path_valid=collision.is_valid,footprint_valid=geometric,path=points,path_sha256=path_hash(points),original_goal=target)
  if not collision.is_valid:raise RuntimeError('REVALIDATION_OBSERVED_PATH_BLOCKED')
  proof=dict(epoch=epoch,plan_revision=cfg['version'],map_revision=cfg['map_revision'],path_valid=True,footprint_valid=True,path_sha256=path_hash(points),segment=s['id']);wait(commit.service_is_ready,3);accepted=False
  for _ in range(3):
   if (st['bt'] or {}).get('action') in ['HUMAN','STOP']:raise RuntimeError('LOCALIZATION_MANUAL_LATCH_RETAINED')
   proof['t']=st['t'];proofpub.publish(String(data=json.dumps(proof)));sent=st['t'];wait(lambda:st['t']>sent+.08,2);answer=result(commit.call_async(Trigger.Request()),3)
   if answer.success:accepted=True;break
  if not accepted:raise RuntimeError('REVALIDATION_COMMIT_REJECTED '+answer.message)
  log('LOCALIZATION_PLAN_COMMITTED',segment=s['id'],epoch=epoch,proof=proof)
  wait(lambda:bool(st['loc'] and st['loc']['valid']),12);st.update(phase='READY',nav_failed=False);log('LOCALIZATION_REVALIDATED_RESUME',segment=s['id'],epoch=epoch)
 def observation_decision(s,choices,seconds,after=None):
  def decision():
   bt=st['bt'] or {};action=stopped_decision(bt.get('action'),localization_hold(),choices)
   if action in ['HUMAN','STOP','RECOVER_LOCALIZATION']:return action
   return action if after is None or bt.get('evidence',{}).get('stamp',-1)>after else None
  wait(lambda:decision() is not None,seconds);action=decision()
  if action=='RECOVER_LOCALIZATION':
   log('LATE_LOCALIZATION_HOLD_DISPATCHED',segment=s['id'],phase=st['phase']);recover_localization(s);barrier=st['t']
   wait(lambda:bool(st['bt'] and st['bt'].get('evidence',{}).get('stamp',-1)>barrier and st['bt']['action'] in list(choices)+['HUMAN','STOP']),seconds);action=st['bt']['action']
  if action in ['HUMAN','STOP']:raise RuntimeError('MANUAL_OR_SAFETY_HOLD '+str((st['bt'] or {}).get('reason_code')))
  return action
 def begin_tool(s):
  nonlocal active_tool,tool_future
  work_key,work_target=work_identity(s)
  g=ToolTask.Goal();g.operation=ToolTask.Goal.START_WORK;g.task_id='R09_'+cluster['cluster'];g.idempotency_key=cfg['version']+':'+work_key;g.segment_id=work_key;g.map_id=cfg['map_id'];g.map_revision=cfg['map_revision'];g.frame_epoch=cfg['frame_epoch'];g.plan_revision=cfg['version'];g.tool_revision=toolcfg['tool_revision'];g.target=pose(work_target);wire=np.asarray(s['target_area'],dtype=np.float32).astype(float);g.target_area.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in wire];g.requested_quantity=float(abs(np.dot(wire[:,0],np.roll(wire[:,1],-1))-np.dot(wire[:,1],np.roll(wire[:,0],-1)))/2);g.timeout_sim_s=300.
  st['phase']='START_WORK';log('START_WORK_REQUEST',segment=s['id']);publish()
  active_tool=result(tool.send_goal_async(g),5)
  if not active_tool.accepted:active_tool=None;raise RuntimeError('TOOL_ACTION_REJECTED')
  tool_future=active_tool.get_result_async()
  interrupted=wait_startup(tick,lambda:st['t'],lambda:st['bt'],lambda:tool_future.done() or working_feedback_ready(st['tool'],st['t'],g.idempotency_key))
  if interrupted:
   log('BT_INTERRUPTS_START_WORK',action=(st['bt'] or {}).get('action'),reason=interrupted,segment=s['id']);stop_all();return False
  if tool_future.done():
   d=tool_finish_result()
   if d['state']=='SUCCEEDED':st['work_complete']=True;return True
   raise RuntimeError('TOOL_ADMISSION_FAILED')
  log('TOOL_WORKING_CONFIRMED',segment=s['id'],requested_area_m2=g.requested_quantity)
  return True
 def replan_evidence(s):
  # An alternative must retain the fixed swath entry and nonholonomic geometry.
  # No silent replacement of coverage work by a free-space drive.
  st.update(phase='REPLANNING',replans=st['replans']+1)
  if s['kind']=='COVERAGE':return coverage_rejoin(s)
  wait(planner.server_is_ready,5);g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal=pose(s['poses'][-1]);h=result(planner.send_goal_async(g),5)
  if not h.accepted:log('ACK_PLANNER_REJECTED',attempt=st['replans']);return
  r=result(h.get_result_async(),12);log('ACK_PLANNER_RESULT',attempt=st['replans'],status=r.status,candidate_poses=len(r.result.path.poses),claim='bounded current observed search only; no global impossibility claim')
  if r.status==4 and len(r.result.path.poses)>1:
   if s['kind']=='TURN_TOOL_OFF' and len(s['members'])==1:
    log('TOOL_OFF_ALTERNATE_ADMITTED_FOR_VALIDATION',segment=s['id'],same_original_endpoint=s['poses'][-1],work_credit=False)
    return 'SEGMENT_COMPLETE' if recover_endpoint(s,obstruction=True) else None
   raise RuntimeError('ALTERNATE_ROUTE_REQUIRES_COVERAGE_RESEQUENCING_RETAIN_RESIDUAL')
  return False
 def coverage_rejoin(s):
  if len(s['members'])!=1:raise RuntimeError('COVERAGE_REJOIN_REQUIRES_ORIGINAL_SINGLE_STRIP')
  stop_all();st['phase']='REPLANNING';policy=cfg['coverage_recovery'];points=s['poses'];start=list(st['loc']['pose'])
  def checked(points):
   path=NavPath();path.header.frame_id='map';path.poses=[pose(p) for p in points]
   wait(pathcheck.service_is_ready,4);request=IsPathValid.Request();request.path=path;answer=result(pathcheck.call_async(request),4)
   return answer.is_valid,list(answer.invalid_pose_indices)
  current=min(range(len(points)),key=lambda i:math.dist(points[i][:2],start[:2]))
  ok,bad=checked(forward_window(points,current,policy['resume_window_m']))
  log('ORIGINAL_WORK_WINDOW_RECHECK',segment=s['id'],cursor=current,accepted=ok,invalid_pose_indices=bad,scope='observed near prefix only; does not claim whole strip is clear')
  if ok:
   st.update(nav_failed=False,phase='READY');return 'ORIGINAL_REVALIDATED'
  for proposal in rejoin_candidates(points,start,policy['candidate_spacing_m'],policy['maximum_advance_m'],policy['maximum_candidates']):
   goal=proposal['pose'];ok,bad=checked(forward_window(points,proposal['index'],policy['resume_window_m']))
   log('COVERAGE_REJOIN_WINDOW',segment=s['id'],**proposal,accepted=ok,invalid_pose_indices=bad)
   if not ok:continue
   wait(planner.server_is_ready,5);g=ComputePathToPose.Goal();g.planner_id='GridBased';g.use_start=True;g.start=pose(start);g.goal=pose(goal)
   h=result(planner.send_goal_async(g),5);st['coverage_planner_queries']=st.get('coverage_planner_queries',0)+1
   if not h.accepted:continue
   answer=result(h.get_result_async(),12);candidate=[]
   for m in answer.result.path.poses:
    q=m.pose.orientation;candidate.append([m.pose.position.x,m.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
   ok,why=validate_candidate(candidate,start,goal,cfg,policy['maximum_connector_m']) if answer.status==4 else (False,'PLANNER_FAILED')
   log('COVERAGE_REJOIN_PLAN',segment=s['id'],target_index=proposal['index'],status=answer.status,accepted=ok,reason=why,poses=candidate)
   if not ok:continue
   try:gear_runs(candidate)
   except ValueError as ex:log('COVERAGE_REJOIN_KINEMATICS_REFUSED',reason=str(ex));continue
   ok,bad=checked(candidate);log('COVERAGE_REJOIN_FULL_COSTMAP',accepted=ok,invalid_pose_indices=bad)
   if not ok:continue
   transit=dict(s,id=s['id']+':rejoin:'+str(proposal['index']),kind='TURN_TOOL_OFF',tool_on=False,poses=candidate,members=[dict(id=s['id'],endpoint=goal)])
   st.update(segment='TURN_TOOL_OFF',nav_failed=False)
   try:recover_endpoint(transit,coverage_mode=True,prepared=candidate)
   finally:st.update(segment=s['kind'],segment_id=s['id'],direction=s['direction'])
   st.update(nav_failed=False,phase='READY',coverage_rejoins=st.get('coverage_rejoins',0)+1)
   log('COVERAGE_REJOIN_CONFIRMED',segment=s['id'],original_cursor=current,rejoin_index=proposal['index'],same_idempotency_key=cfg['version']+':'+work_identity(s)[0],skipped_work_credit=False,residual_policy='original target area minus actual confirmed feedback; no area removed')
   return 'COVERAGE_REJOINED'
  raise RuntimeError('NO_ADMITTED_COVERAGE_REJOIN_RETAIN_RESIDUAL')
 def stopped_original_arrival(s,window=1.5):
  samples=[];began=st['t']
  while st['t']-began<window:
   tick();loc=st['loc'] or {};gate=st['gate'] or {}
   if (st['bt'] or {}).get('action') in ['HUMAN','STOP']:raise RuntimeError('RECOVERY_MANUAL_LATCH_RETAINED')
   if not (loc.get('valid') and -.03<=st['t']-loc.get('t',-100)<.15 and gate.get('stopped') and gate.get('tool_stopped')):
    samples=[];continue
   if not samples or loc['t']>samples[-1][0]:samples.append((loc['t'],loc['pose']))
   samples=[x for x in samples if loc['t']-x[0]<=.75]
   if stable_arrival(samples,s['poses'][-1],cfg['navigation_execution']['intermediate_xy_tolerance_m'],cfg['navigation_execution']['intermediate_yaw_tolerance_rad']):
    log('STOPPED_ORIGINAL_ARRIVAL_CONFIRMED',segment=s['id'],samples=len(samples),span_s=samples[-1][0]-samples[0][0],xy_error_m=math.dist(loc['pose'][:2],s['poses'][-1][:2]),yaw_error_rad=yaw_error(loc['pose'][2],s['poses'][-1][2]),limits=[.10,.12]);return True
  log('STOPPED_ORIGINAL_ARRIVAL_NOT_CONFIRMED',segment=s['id']);return False
 def recover_endpoint(s,obstruction=False,coverage_mode=False,prepared=None):
  nonlocal active
  if s['kind']!='TURN_TOOL_OFF' or checkpoint_index<len(s['members'])-1:raise RuntimeError('RECOVERY_NOT_ADMITTED')
  stop_all();deadline=st['t']+(cfg['coverage_recovery']['maximum_sim_seconds'] if coverage_mode else 90.);length_limit=cfg['coverage_recovery']['maximum_connector_m'] if coverage_mode else 12.
  if localization_hold():recover_localization(s)
  if stopped_original_arrival(s):
   log('ENDPOINT_RECOVERY_SUCCESS',segment=s['id'],attempt=0,pause_restarts=0,method='measured stopped original segment admission; no extra motion');return True
  old_direction=st['direction'];st.update(endpoint_recovery=True,direction=0,nav_failed=False,endpoint_recoveries=st.get('endpoint_recoveries',0)+1);query_count=0;pause_restarts=0
  try:
   for attempt in range(2):
    if st['t']>=deadline:raise TimeoutError('ENDPOINT_RECOVERY_BUDGET')
    if (st['bt'] or {}).get('action') in ['HUMAN','STOP']:raise RuntimeError('RECOVERY_MANUAL_LATCH_RETAINED')
    st['phase']='REPLANNING';wait(planner.server_is_ready,5)
    def plan_to(target,start_pose=None):
     nonlocal query_count
     if query_count>=6 or st['t']>=deadline:raise TimeoutError('ENDPOINT_PLANNING_BUDGET')
     query_count+=1;st['endpoint_planner_queries']=st.get('endpoint_planner_queries',0)+1;log('ENDPOINT_PLANNER_QUERY',sequence=st['endpoint_planner_queries'],local_query=query_count,target=target,explicit_start=start_pose)
     g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal=pose(target)
     if start_pose is not None:g.use_start=True;g.start=pose(start_pose)
     h=result(planner.send_goal_async(g),5)
     if not h.accepted:raise RuntimeError('ENDPOINT_PLANNER_REJECTED')
     return result(h.get_result_async(),min(12.,max(.1,deadline-st['t'])))
    actual_start=list(st['loc']['pose']);prefix=retreat_prefix(actual_start,cfg['navigation_execution']['endpoint_recovery']['obstruction_retreat_m']) if obstruction else []
    planning_start=prefix[-1] if prefix else actual_start
    if prefix:log('TOOL_OFF_RETREAT_CANDIDATE',segment=s['id'],distance_m=cfg['navigation_execution']['endpoint_recovery']['obstruction_retreat_m'],start=actual_start,retreat_goal=planning_start,claim='candidate only; whole swept geometry and live costmap still required')
    if prepared is not None and attempt==0:candidate=prepared;plan_status=4
    else:
     answer=plan_to(s['poses'][-1],planning_start if prefix else None);path=answer.result.path;plan_status=answer.status;candidate=[]
     for m in path.poses:
      q=m.pose.orientation;candidate.append([m.pose.position.x,m.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
     if prefix and candidate:candidate=prefix[:-1]+candidate
    ok,why=validate_candidate(candidate,actual_start,s['poses'][-1],cfg,length_limit) if plan_status==4 else (False,'PLANNER_FAILED')
    log('ENDPOINT_RECOVERY_PLAN',attempt=attempt,status=plan_status,accepted=ok,reason=why,poses=candidate,segment=s['id'],coverage_rejoin=coverage_mode)
    if not ok:
     start_pose=planning_start;folder=out/'endpoint_candidates'/f"{st['endpoint_recoveries']}_{attempt}"
     tail=terminal_straight(s['poses'][-1],cfg['navigation_execution']['endpoint_recovery']['terminal_straight_approach_m']) if attempt==0 else []
     child,response=ompl_endpoint_r10.launch(start_pose,tail[0] if tail else s['poses'][-1],1/cfg['vehicle']['steering']['max_curvature']+.10,folder)
     try:wait(lambda:child.poll() is not None,3)
     finally:
      if child.poll() is None:child.kill();child.wait(timeout=2)
     stdout,stderr=child.communicate()
     if child.returncode:log('CONTINUOUS_RECOVERY_SOLVER_FAILED',exit=child.returncode,stderr=stderr.decode()[-500:]);continue
     candidate,details=ompl_endpoint_r10.sample(start_pose,response)
     if tail:
      candidate=candidate+tail[1:];details['terminal_straight_approach_m']=cfg['navigation_execution']['endpoint_recovery']['terminal_straight_approach_m']
     candidate=prefix[:-1]+candidate if prefix else candidate;ok,why=validate_candidate(candidate,actual_start,s['poses'][-1],cfg,length_limit);st['continuous_recovery_queries']=st.get('continuous_recovery_queries',0)+1;log('CONTINUOUS_RECOVERY_CANDIDATE',attempt=attempt,accepted=ok,reason=why,poses=candidate,details=details)
     if not ok:continue
     path=NavPath();path.header.frame_id='map';path.header.stamp=n.get_clock().now().to_msg();path.poses=[pose(p) for p in candidate]
    # Every gear run has its own action goal and measured stop. A whole mixed
    # Reeds-Shepp path must never be pruned past a cusp by a local controller.
    full=NavPath();full.header.frame_id='map';full.header.stamp=n.get_clock().now().to_msg();full.poses=[pose(p) for p in candidate]
    wait(pathcheck.service_is_ready,5);request=IsPathValid.Request();request.path=full;collision=result(pathcheck.call_async(request),5)
    log('ENDPOINT_WHOLE_CANDIDATE_COSTMAP',accepted=collision.is_valid,poses=len(full.poses))
    if not collision.is_valid:continue
    runs=gear_runs(candidate);candidate_ok=True
    log('ENDPOINT_GEAR_RUNS',attempt=attempt,runs=runs)
    for gear_index,run in enumerate(runs):
     gear_path=run['poses'];cursor=0;gear_done=False
     while not gear_done:
      if st['t']>=deadline:raise TimeoutError('ENDPOINT_RECOVERY_BUDGET')
      # Gear selection has no motion authority. While physically stopped, it
      # makes the unchanged directional risk envelope inspect the intended side.
      st.update(phase='READY',direction=run['direction'],nav_failed=False)
      wait(lambda:bool(st['gate'] and st['gate'].get('armed_direction')==run['direction'] and st['gate']['stopped'] and st['gate']['tool_stopped']),3)
      barrier=st['t']
      wait(lambda:bool(st['loc'] and st['loc']['valid'] and st['bt'] and st['bt'].get('evidence',{}).get('stamp',-1)>barrier and st['bt']['action']=='EXECUTE' and st['gate']['stopped'] and st['gate']['tool_stopped']),min(8.,max(.1,deadline-st['t'])))
      remaining=gear_path[min(cursor,len(gear_path)-2):]
      ok,why=validate_candidate(remaining,st['loc']['pose'],gear_path[-1],cfg,length_limit)
      if not ok:log('ENDPOINT_RESUME_REJECTED',reason=why,cursor=cursor,gear_index=gear_index);candidate_ok=False;break
      path=NavPath();path.header.frame_id='map';path.header.stamp=n.get_clock().now().to_msg();path.poses=[pose(p) for p in remaining]
      wait(pathcheck.service_is_ready,5);request=IsPathValid.Request();request.path=path;collision=result(pathcheck.call_async(request),5);log('ENDPOINT_FULL_COSTMAP_VALIDATION',accepted=collision.is_valid,invalid_pose_indices=list(collision.invalid_pose_indices),poses=len(path.poses),cursor=cursor,gear_index=gear_index)
      if not collision.is_valid:candidate_ok=False;break
      st.update(phase='READY',direction=run['direction']);wait(lambda:bool(st['gate'] and st['gate'].get('armed_direction')==run['direction'] and st['gate']['stopped'] and st['gate']['tool_stopped']),3)
      log('RECOVERY_GEAR_ARMED',gear_index=gear_index,direction=run['direction'],candidate_attempt=attempt)
      goal=FollowPath.Goal();goal.controller_id='RecoveryRPP';goal.goal_checker_id='goal';goal.progress_checker_id='progress';goal.path=path;st.update(phase='NAVIGATING',nav_idle=False);active=result(nav.send_goal_async(goal),5)
      if not active.accepted:active=None;raise RuntimeError('ENDPOINT_CONTROLLER_REJECTED')
      f=active.get_result_async();began=st['t'];temporary_interrupt=False;tracking_failed=False;wrong_gear_since=None
      while not f.done():
       tick()
       if (st['loc'] or {}).get('valid'):
        cursor=observed_cursor(gear_path,st['loc']['pose'],cursor);distance=tracking_distance(gear_path,st['loc']['pose'],cursor)
        if distance>cfg['navigation_execution']['endpoint_recovery']['max_tracking_error_m']:
         log('RECOVERY_TRACKING_STOP',distance_m=distance,gear_index=gear_index,cursor=cursor,pose=st['loc']['pose']);stop_all();tracking_failed=True;break
       raw=(st['gate'] or {}).get('raw_nav_v') or 0.
       wrong=raw*run['direction']<-.001
       if not wrong:wrong_gear_since=None
       elif wrong_gear_since is None:wrong_gear_since=st['t']
       if wrong_gear_since is not None and st['t']-wrong_gear_since>.6:
        log('RECOVERY_TERMINAL_GEAR_CONFLICT',gear_index=gear_index,raw_v=raw);stop_all()
        if gear_index==len(runs)-1 and stopped_original_arrival(s):
         log('ENDPOINT_RECOVERY_SUCCESS',segment=s['id'],attempt=attempt,pause_restarts=pause_restarts,method='stopped original arrival after terminal gear conflict');return True
        tracking_failed=True;break
       if st['t']>deadline:raise TimeoutError('ENDPOINT_RECOVERY_BUDGET')
       if st['t']-began>.4 and (st['bt'] or {}).get('action')!='EXECUTE':
        action=(st['bt'] or {}).get('action');log('ENDPOINT_RECOVERY_INTERRUPTED',action=action,reason=(st['bt'] or {}).get('reason_code'),cursor=cursor,gear_index=gear_index);stop_all()
        if action in ['HUMAN','STOP']:raise RuntimeError('ENDPOINT_RECOVERY_MANUAL_LATCH_RETAINED')
        temporary_interrupt=True;break
      success=f.done() and f.result().status==4;stop_all()
      if success:
       p=st['loc']['pose'];q=gear_path[-1];xy=math.dist(p[:2],q[:2]);heading=yaw_error(p[2],q[2])
       if xy>.05 or heading>.08:
        log('RECOVERY_CUSP_NOT_CONFIRMED',gear_index=gear_index,xy_m=xy,yaw_rad=heading);candidate_ok=False;break
       log('RECOVERY_CUSP_STOP_CONFIRMED',gear_index=gear_index,direction=run['direction'],xy_m=xy,yaw_rad=heading);gear_done=True;continue
      if tracking_failed or not temporary_interrupt:candidate_ok=False;break
      pause_restarts+=1
      if pause_restarts>cfg['navigation_execution']['endpoint_recovery']['max_observation_resumes']:raise RuntimeError('ENDPOINT_OBSERVATION_RESUME_BOUND')
      barrier=st['t'];st['phase']='OBSERVATION_HOLD'
      observation_decision(s,['EXECUTE','HUMAN','STOP'],min(12.,max(.1,deadline-st['t'])),after=barrier)
      if st['bt']['action'] in ['HUMAN','STOP']:raise RuntimeError('ENDPOINT_RECOVERY_MANUAL_LATCH_RETAINED')
      log('ENDPOINT_OBSERVATION_RESUME',attempt=attempt,cursor=cursor,gear_index=gear_index,pause_restarts=pause_restarts,remaining_budget_s=deadline-st['t'])
     if not candidate_ok:break
    if candidate_ok:
     p=st['loc']['pose'];q=s['poses'][-1]
     if math.dist(p[:2],q[:2])>.10 or yaw_error(p[2],q[2])>.12:raise RuntimeError('RECOVERY_ARRIVAL_NOT_CONFIRMED')
     log('ENDPOINT_RECOVERY_SUCCESS',segment=s['id'],attempt=attempt,pause_restarts=pause_restarts);return True
   raise RuntimeError('ENDPOINT_RECOVERY_EXHAUSTED')
  finally:st.update(endpoint_recovery=False,direction=old_direction)
 def end(s,f):st['stop']=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
 (out/'observer_ready').write_text('Ackermann FollowPath + actual ToolTask feedback; no actuator or truth authority')
 try:
  wait(lambda:nav.server_is_ready() and tool.server_is_ready() and bool(st['loc'] and st['loc']['valid'] and st['gate'] and st['gate']['tool_stopped']),75);wait(manager.service_is_ready,15)
  while not result(manager.call_async(Trigger.Request()),5).success:
   t=st['t'];wait(lambda:st['t']>t+.3,2)
  log('NAV_LIFECYCLE_ACTIVE_VERIFIED')
  for index,s in enumerate(segments):
   active_group=s;checkpoint_index=0
   stop_all();st.update(index=index,segment=s['kind'],segment_id=s['id'],direction=s['direction'],work_complete=False,nav_failed=False,replans=0,phase='READY');done=False;attempts=0
   while not done:
    action=observation_decision(s,['EXECUTE','REPLAN','HUMAN','STOP'],55)
    if action in ['HUMAN','STOP']:raise RuntimeError('BT_'+st['bt'].get('reason_code','HOLD'))
    if action=='REPLAN':
     if st['replans']>=2:raise RuntimeError('ACK_BOUNDED_SEARCH_EXHAUSTED')
     outcome=replan_evidence(s)
     if outcome in ['ORIGINAL_REVALIDATED','COVERAGE_REJOINED']:
      attempts+=1
      if retry_exhausted(attempts,len(s['members']),False):raise RuntimeError('SEGMENT_RETRY_BOUND')
      barrier=st['t'];wait(lambda:bool(st['bt'] and st['bt'].get('evidence',{}).get('stamp',-1)>barrier and st['bt']['action'] in ['EXECUTE','HUMAN','STOP']),15)
      continue
     if outcome=='SEGMENT_COMPLETE':
      active=None;st['nav_idle']=True;stop_all();p=st['loc']['pose'];q=s['poses'][-1];dist=math.dist(p[:2],q[:2]);angle=yaw_error(p[2],q[2]);log('SEGMENT_ARRIVAL',segment=s['id'],xy_error_m=dist,yaw_error_rad=angle,route='validated tool-off alternative to same original endpoint')
      if dist>.10 or angle>.12:raise RuntimeError('SEGMENT_ARRIVAL_NOT_CONFIRMED')
      record_passage()
      if checkpoint_index!=len(s['members']):raise RuntimeError('ORIGINAL_SEGMENT_PASSAGE_NOT_CONFIRMED '+s['members'][checkpoint_index]['id'])
      completed.extend(m['id'] for m in s['members']);done=True;break
     st['nav_failed']=True;barrier=st['t'];wait(lambda:(st['bt']or{}).get('evidence',{}).get('stamp',-1)>barrier,3);continue
    st['phase']='READY';wait(lambda:bool(st['gate'] and st['gate'].get('armed_direction')==s['direction'] and st['gate']['stopped'] and st['gate']['tool_stopped']),3);log('DIRECTION_ARMED_WHILE_STOPPED',direction=s['direction'],segment=s['id'])
    if s['kind']=='COVERAGE' and not st['work_complete'] and not begin_tool(s):
     attempts+=1
     if retry_exhausted(attempts,len(s['members']),False):raise RuntimeError('SEGMENT_RETRY_BOUND')
     st['phase']='OBSERVATION_HOLD';barrier=st['t']
     observation_decision(s,['EXECUTE','REPLAN','HUMAN','STOP'],55,after=barrier)
     continue
    points=np.asarray(s['poses']);p=np.asarray(st['loc']['pose']);i=int(np.argmin(np.linalg.norm(points[:,:2]-p[:2],axis=1)))
    if math.dist(p[:2],points[i,:2])>.4:raise RuntimeError('SEGMENT_CAPTURE_OUTSIDE_QUALIFIED_CORRIDOR')
    goal=FollowPath.Goal();goal.controller_id='TurnRPP' if s['kind']=='TURN_TOOL_OFF' else 'RPP';goal.goal_checker_id='goal';goal.progress_checker_id='progress';goal.path.header.frame_id='map';goal.path.header.stamp=n.get_clock().now().to_msg();goal.path.poses=[pose(x) for x in points[min(max(0,i-2),len(points)-2):]];st.update(phase='NAVIGATING',nav_idle=False);active=result(nav.send_goal_async(goal),8)
    if not active.accepted:active=None;st['nav_idle']=True;raise RuntimeError('FOLLOW_PATH_REJECTED')
    log('SEGMENT_GOAL',segment=s['id'],kind=s['kind'],direction=s['direction'],original_poses=len(points),remaining_poses=len(goal.path.poses),tool_expected=s['tool_on'],original_members=[m['id'] for m in s['members']]);f=active.get_result_async();interrupted=False;began=st['t'];recovered=False;endpoint_best=float('inf');endpoint_progress=st['t']
    while not f.done():
     tick()
     if tool_future is not None and tool_future.done():
      d=tool_finish_result()
      if d['state']!='SUCCEEDED':raise RuntimeError('WORK_ACTION_FAILED_BEFORE_SEGMENT_ARRIVAL '+d['reason'])
      st['work_complete']=True;log('WORK_QUANTITY_CONFIRMED_BEFORE_PATH_END',segment=s['id']);wait(lambda:bool(st['gate'] and st['gate']['tool_stopped']),4)
     bt=st['bt'] or {}
     if bt.get('action') in ['CANCEL','WAIT','REPLAN','HUMAN','STOP','DEFER'] and st['t']-began>.4:log('BT_INTERRUPTS_SEGMENT',action=bt['action'],reason=bt.get('reason_code'));interrupted=True;stop_all();break
     if s['kind']=='TURN_TOOL_OFF' and checkpoint_index>=len(s['members'])-1 and (st['loc'] or {}).get('valid'):
      distance=math.dist(st['loc']['pose'][:2],points[-1,:2])
      if distance<endpoint_best-.01:endpoint_best=distance;endpoint_progress=st['t']
      if distance<.25 and st['t']-endpoint_progress>3. and bt.get('action')=='EXECUTE':
       log('ENDPOINT_POSE_STALL',segment=s['id'],distance_m=distance,yaw_error_rad=yaw_error(st['loc']['pose'][2],points[-1,2]));recovered=recover_endpoint(s);break
    if recovered or (f.done() and f.result().status==4):
     active=None;st['nav_idle']=True;stop_all();p=st['loc']['pose'];q=points[-1];dist=math.dist(p[:2],q[:2]);angle=abs(math.atan2(math.sin(p[2]-q[2]),math.cos(p[2]-q[2])));log('SEGMENT_ARRIVAL',segment=s['id'],xy_error_m=dist,yaw_error_rad=angle)
     if dist>.10 or angle>.12:raise RuntimeError('SEGMENT_ARRIVAL_NOT_CONFIRMED')
     record_passage()
     if checkpoint_index!=len(s['members']):raise RuntimeError('ORIGINAL_SEGMENT_PASSAGE_NOT_CONFIRMED '+s['members'][checkpoint_index]['id'])
     completed.extend(m['id'] for m in s['members']);done=True
    else:
     if active:stop_all()
     st.update(nav_failed=not interrupted,phase='OBSERVATION_HOLD');barrier=st['t'];choices=['EXECUTE','REPLAN','HUMAN','STOP'] if interrupted else ['REPLAN','HUMAN','STOP'];observation_decision(s,choices,55,after=barrier)
    attempts+=1
    if retry_exhausted(attempts,len(s['members']),done):raise RuntimeError('SEGMENT_RETRY_BOUND')
  st.update(complete=True,phase='DONE');log('SELECTED_SEGMENTS_TRAVERSED',selected=len(selected),navigation_groups=len(segments),original=len(allsegments),coverage_completion='read feedback ledger separately');t=st['t'];wait(lambda:st['t']>t+2,4);terminal='COMPLETED'
 except InterruptedError as ex:error=repr(ex);terminal='CANCELLED';log('TASK_STOP',error=error)
 except BaseException as ex:error=repr(ex);terminal='HUMAN_REQUIRED' if isinstance(ex,(RuntimeError,TimeoutError)) and 'STOP_UNCONFIRMED' not in str(ex) and 'CONFIRMATION_TIMEOUT' not in str(ex) else 'FAULT';log('TASK_STOP',error=error,terminal=terminal)
 finally:
  try:stop_all()
  except BaseException as ex:terminal='FAULT';error=error or repr(ex);log('STOP_UNCONFIRMED',error=repr(ex))
  ownership={topic:[i.node_name for i in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/s03/actuator/request','/s10/tool/command']};final=dict(passed=terminal=='COMPLETED' and len(completed)==len(selected),terminal=terminal,controlled_hold=terminal=='HUMAN_REQUIRED',error=error,t=st['t'],selected_segments=len(selected),navigation_groups=len(segments),original_segment_passages=passages,original_segments=len(allsegments),execution_segments=len(allsegments),reference_original_segments=cluster.get('source_original_segments',len(allsegments)),original_reference_status=cfg.get('safe_execution_plan',{}).get('original_reference_status'),safe_plan_revision=cfg.get('safe_execution_plan',{}).get('revision'),localization_revalidations=revalidations,completed_segments=completed,remaining_selected=[s['id'] for s in selected if s['id'] not in completed],not_selected=[s['id'] for s in allsegments if s['id'] not in {x['id'] for x in selected}],work_results=work_results,replans=st['replans'],coverage_rejoins=st.get('coverage_rejoins',0),coverage_planner_queries=st.get('coverage_planner_queries',0),endpoint_recoveries=st.get('endpoint_recoveries',0),endpoint_planner_queries=st.get('endpoint_planner_queries',0),continuous_recovery_queries=st.get('continuous_recovery_queries',0),final_gate=st['gate'],ownership=ownership,wall_seconds=time.monotonic()-start,execution_mode=cfg['navigation_execution']['mode'],scope='All current approved execution goals checked in order; original 34 reference remains PARTIAL; unchanged original work polygons use physical feedback only');(out/'mission_result.json').write_text(json.dumps(final,indent=2));n.destroy_node();rclpy.try_shutdown()
 return 0 if terminal in ['COMPLETED','HUMAN_REQUIRED'] else 2
if __name__=='__main__':raise SystemExit(main())
