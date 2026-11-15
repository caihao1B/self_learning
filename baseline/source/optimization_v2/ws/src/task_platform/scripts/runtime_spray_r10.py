from transport_r08 import configure_transport,transport_environment
"""Bounded real sensor / native RTAB sessions with owned teardown and process restart."""
from common import *
import time,signal,shutil
from overlay_r08 import overlay
from scene_spray_r10 import compose
from lifecycle_r1 import environment

def run(task="spray",seconds=240,rtf=.4,cluster="A1",point_limit=3,case="nominal"):
 require('R09');lock=lock_session();out=APP/'results/R10'/(task+'_'+stamp());out.mkdir(parents=True);session=out.name;start=time.monotonic();c=compose(out/'scene',task,session,rtf=rtf,cluster=cluster);adapt=overlay(task,out/'overlay');binding=read(edition('R05')/task/'binding.json');source=read(BASE/'maps/frozen/manifest.json')
 spec=dict(task=task,session=session,sim_seconds=seconds,wall_budget=max(600,int(seconds/min(rtf,.25))+90),cluster=cluster,point_limit=point_limit,case=case,operator_hint=read(out/'scene/fixture.json')['coarse_operator_map_hint'],localization_backend=binding['localization_backend'],rig_revision=binding['rig_revision'],binding_revision=binding['revision']);spec['online_probe_sha256']=sha(PKG/'scripts/mission_spray_r10.py');spec['runtime_sha256']=sha(Path(__file__));bound=list((PKG/'scripts').glob('*_r07.py'))+list((PKG/'scripts').glob('*_r08.py'))+[PKG/'scripts'/name for name in ['runtime_spray_r10.py', 'mission_spray_r10.py', 'scene_spray_r10.py', 'control_spray_r10.py', 'docking_r10.py', 'local_dock_r10.py', 'progress_spray_r10.py', 'approach_handoff_r10.py', 'revalidation_spray_r10.py', 'reference_spray_r10.py', 'risk_spray_r10.py', 'ledger_spray_r10.py', 'ledger_mowing_r10.py', 'runtime_binding_r10.py', 'prepare_r12.py', 'analyze_spray_r10.py', 'common.py']]+list((edition('R10')/'spray').rglob('*'))+list((PKG/'r08_backend').glob('*'))+list((PKG/'r08_nav').glob('*'))+[APP/'build/r08/r08_global_manager',APP/'build/r08_nav_install/lib/libr08_observed_prior.so']+list((PKG/'r07_nav').glob('*'))+[APP/'build/r09/r09_tool_action_server',APP/'build/r09_source/tool_action_server.cpp',APP/'build/r09_source/ToolActuator.cc',APP/'build/r09/libs10_tool_r09.so',PKG/'scripts/tool_build_r09.py',APP/'build/r08/r08_terrain',APP/'build/r08/r08_supervisor',APP/'build/r07_nav_install/lib/libr07_exact_goal.so',BASE/'ws/src/work_ledger/scripts/ledger_core.py'];bound += [PKG/'scripts/global_filter_r10.py',PKG/'r10_backend/correction_filter.hpp',PKG/'r10_backend/stopped_revalidation.inc',APP/'build/r10_global_source/global_manager.cpp',APP/'build/r10_spray_global/r10_global_manager'];bound += [PKG/'scripts/bridge_r10.py',PKG/'r10_backend/tool_bridge.cpp',APP/'build/r10_bridge/r10_tool_bridge'];bound += [PKG/'scripts/spray_global_r10.py',APP/'build/r10_spray_global_source/global_manager.cpp',APP/'build/r10_spray_global_source/stopped_revalidation.inc'];bound += [PKG/'scripts/precision_supervisor_r10.py',PKG/'r10_backend/precision_progress.hpp',APP/'build/r10_spray_bt/r10_spray_supervisor',APP/'build/r10_spray_bt_source/policy.hpp',APP/'build/r10_spray_bt_source/supervisor.cpp'];spec['implementation_bindings']={str(p.relative_to(ROOT)):sha(p) for p in bound if p.is_file()};dump(out/'spec.json',spec);dump(out/'request.json',dict(map_id=source['map_id'],rig_binding_revision=binding['revision'],localization_backend=binding['localization_backend'],rig_revision=binding['rig_revision']))
 cfg=read((edition('R10')/'spray')/'mission.json');dump(out/'mission.json',cfg);bt=read(edition('R06')/'spray.json');bt.update(session=session,task_version=cfg['version'],rig_revision=cfg['rig']['revision'],precision_progress_policy=cfg['precision_progress_policy']);dump(out/'bt.json',bt)
 from rig_r08 import bind_adapter
 bind_adapter(adapt,cfg);spec.update(rig_revision=cfg['rig']['revision'],base_calibration_revision=binding['rig_revision']);dump(out/'spec.json',spec)
 env=os.environ.copy();env.update(ROS_DOMAIN_ID='119',GZ_PARTITION='r10_'+session,R05_BINDING=str(edition('R05')/task/'binding.json'),R05_ADAPTER=str(adapt),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');configure_transport(env,out);env['GZ_SIM_SYSTEM_PLUGIN_PATH']=':'.join([str(APP/'build/r09'),str(APP/'build/platform'),*[str(BASE/'ws/install'/p/'lib') for p in ['sensor_sim_adapter','chassis_bridge','scenario_world','tool_sim']],env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')]);env['AMENT_PREFIX_PATH']=str(APP/'build/r08_nav_install')+':'+str(APP/'build/r07_nav_install')+':'+env.get('AMENT_PREFIX_PATH','');env['LD_LIBRARY_PATH']=str(APP/'build/r08_nav_install/lib')+':'+str(APP/'build/r07_nav_install/lib')+':'+env.get('LD_LIBRARY_PATH','');raw=env.copy();raw['ROS_DOMAIN_ID']='121';ev=env.copy();ev['ROS_DOMAIN_ID']='120';children=[];records=[];logs=[];forced=[];error=None;server=None;slam=None;retired=set();auxiliary=set();nav_started=False;nav_shutdown=None;fault_sent=False
 def launch(role,args,e=env,aux=False):
  log=(out/(role+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,env=environment(role,transport_environment(role,e),out),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);
  if aux:auxiliary.add(p)
  records.append(dict(role=role,pid=p.pid,args=args,domain=e['ROS_DOMAIN_ID'],partition=e['GZ_PARTITION'],dds_profile=transport_environment(role,e).get('FASTRTPS_DEFAULT_PROFILES_FILE','DEFAULT')));return p
 def gz(args):return subprocess.run(['gz',*args],env=env,capture_output=True,text=True,timeout=7)
 def start_slam(name):return launch(name,[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/regional_r08.py'),'--out',str(out/('quality_restarted' if name.endswith('restarted') else 'quality')),'--request',str(out/'request.json')])
 s05=BASE/'ws/src/localization_guard/scripts';bridge='/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
 try:
  server=launch('server',['gz','sim','--force-version','8','-s','--seed','41',str(out/'scene/world.sdf')]);deadline=time.monotonic()+45
  while time.monotonic()<deadline:
   if server.poll() is not None:raise RuntimeError('SERVER_FAILED')
   if '/world/irregular_farm/control' in gz(['service','-l']).stdout:break
   time.sleep(.2)
  else:raise TimeoutError('WORLD_STARTUP')
  launch('online_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s03/encoders@std_msgs/msg/String[gz.msgs.StringMsg','/s03/actuator/request@std_msgs/msg/String]gz.msgs.StringMsg',*['/r07/near_'+u['name']+'/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' for u in cfg['near_sensor']['units']]])
  launch('tool_bridge',[str(APP/'build/r10_bridge/r10_tool_bridge')])
  launch('raw_bridge',[bridge,'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/s04/raw/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked','/s04/raw/left@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/right@sensor_msgs/msg/Image[gz.msgs.Image','/s04/raw/imu@sensor_msgs/msg/Imu[gz.msgs.IMU','/s04/raw/gnss@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat','/s04/raw/joints@std_msgs/msg/String[gz.msgs.StringMsg'],raw)
  launch('evaluation_bridge',[bridge,'/s03/evaluation/pose@std_msgs/msg/String[gz.msgs.StringMsg','/s03/evaluation/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/s10/evaluation/effect@std_msgs/msg/String[gz.msgs.StringMsg'],ev);launch('evaluator',['/usr/bin/python3',str(BASE/'ws/src/chassis_bridge/scripts/evaluator.py'),'--out',str(out/'evaluation')],ev)
  for mode in ['motion','vision']:launch('generator_'+mode,['/usr/bin/python3',str(adapt/'adapter.py'),'--vehicle',str(out/'scene/vehicle.json'),'--out',str(out/('generation_'+mode)),'--seed','41','--profile','nominal','--streams',mode],raw)
  launch('fanout',['/usr/bin/python3',str(PKG/'scripts/near_r08.py'),'--out',str(out/'near')]);launch('cloud',['/usr/bin/python3',str(adapt/'cloud_filter.py'),'--out',str(out/'cloud')])
  launch('icp',['/opt/ros/jazzy/lib/rtabmap_odom/icp_odometry','--ros-args','-r','__node:=s05_icp','-r','__ns:=/s05/icp','--params-file',str(out/'scene/icp.yaml'),'-r','scan_cloud:=/s05/icp/cloud','-r','scan:=/s05/unused_scan','-r','odom:=/s05/icp/raw','-r','odom_info:=/s05/icp/info','-r','/tf:=/s05/internal/icp_tf','--log-level','warn'])
  for role,script in [('icp_gate','icp_gate.py'),('inputs','input_adapter.py'),('odom_output','odom_output.py')]:launch(role,['/usr/bin/python3',str(adapt/script if role in ['icp_gate','inputs'] else s05/script),'--out',str(out/role)])
  for name,topic in [('fused','/s05/raw/fused'),('wheel_imu','/s05/raw/wheel_imu')]:launch(name,['/opt/ros/jazzy/lib/robot_localization/ekf_node','--ros-args','-r','__node:=s05_'+name,'-r','__ns:=/s05/'+name,'--params-file',str(out/('scene/'+name+'.yaml')),'-r','odometry/filtered:='+topic,'--log-level','warn','-r','/tf:=/s05/internal/'+name+'_tf'])
  launch('icp_motion_guess',['/usr/bin/python3',str(PKG/'scripts/guess_r07.py'),'--out',str(out/'icp_motion_guess')])
  launch('mapping_inputs',['/usr/bin/python3',str(PKG/'scripts/map_inputs_r08.py'),'--out',str(out/'mapping_inputs')]);slam=start_slam('slam_initial');
  launch('global_sources',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/sources_r07.py'),'--out',str(out/'global_sources'),'--config',str(out/'mission.json')])
  launch('global_manager',[str(APP/'build/r10_spray_global/r10_global_manager'),'--ros-args','-p','use_sim_time:=true','-p','policy_file:='+str((edition('R10')/'spray')/'manager.json')])
  launch('reference',['/usr/bin/python3',str(PKG/'scripts/reference_spray_r10.py'),'--out',str(out/'reference'),'--config',str(out/'mission.json')])
  launch('terrain',[str(APP/'build/r08/r08_terrain'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str((edition('R10')/'spray')/'terrain.json')])
  launch('risk',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/risk_spray_r10.py'),'--out',str(out/'risk'),'--config',str(out/'mission.json')])
  launch('control',['/usr/bin/python3',str(PKG/'scripts/control_spray_r10.py'),'--out',str(out/'control'),'--config',str(out/'mission.json'),'--session',session])
  launch('bt',[str(APP/'build/r10_spray_bt/r10_spray_supervisor'),'--ros-args','-p','use_sim_time:=true','-p','config_file:='+str(out/'bt.json'),'-p','tree_file:='+str(edition('R06')/'common.xml'),'-p','log_file:='+str(out/'decisions.jsonl')])
  launch('tool_observer',['/usr/bin/python3',str(PKG/'scripts/tool_observer_r07.py'),'--out',str(out/'tool_observer'),'--config',str(out/'scene/tool.json')])
  launch('ledger',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/ledger_spray_r10.py'),'--out',str(out/'ledger'),'--config',str(out/'scene/tool.json')])
  launch('tool_server',[str(APP/'build/r09/r09_tool_action_server'),'--ros-args','-p','config_file:='+str(out/'scene/tool.json')])
  launch('tool_evaluator',[str(BASE/'venvs/scene/bin/python'),str(BASE/'ws/src/tool_sim/scripts/evaluator10.py'),'--out',str(out/'tool_evaluation'),'--config',str(out/'scene/tool.json')],ev)
  for package,name in [('nav2_map_server','map_server'),('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_bt_navigator','bt_navigator'),('nav2_lifecycle_manager','lifecycle_manager')]:
   launch(name,['/opt/ros/jazzy/lib/'+package+'/'+name,'--ros-args','-r','__node:='+name,'--params-file',str((edition('R10')/'spray')/'nav2.yaml'),'-r','cmd_vel:=/r07/nav_cmd_vel','--log-level','warn'])
  observer=launch('observer',[str(BASE/'venvs/scene/bin/python'),str(PKG/'scripts/mission_spray_r10.py'),'--out',str(out)])
  ready=['generation_motion/ready','generation_vision/ready','cloud/ready','icp_gate/ready','inputs/ready','odom_output/ready','icp_motion_guess/ready','mapping_inputs/ready','quality/ready','observer_ready','evaluation/ready','near/ready','reference/ready','global_sources/ready','risk/ready','control/ready','tool_observer/ready','ledger/ready','tool_evaluation/ready'];deadline=time.monotonic()+40
  while time.monotonic()<deadline:
   if any(p.poll() is not None for p in children):raise RuntimeError('NODE_STARTUP_FAILED')
   if all((out/p).exists() for p in ready):break
   time.sleep(.15)
  else:raise TimeoutError('NODES_NOT_READY')
  time.sleep(1.5);r=gz(['service','-s','/world/irregular_farm/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: false'])
  if r.returncode or 'true' not in r.stdout:raise RuntimeError('UNPAUSE_FAILED')
  restarted=False
  while time.monotonic()-start<spec['wall_budget']:
   if observer.poll() is not None:
    if observer.returncode:raise RuntimeError('OBSERVER_NOT_COMPLETE')
    break
   if any(p.poll() is not None for p in children if p not in retired and p not in auxiliary):raise RuntimeError('ESSENTIAL_NODE_EXITED')
   if case=='tool_jam' and not fault_sent and (out/'live.json').exists():
    try:task_live=read(out/'live.json')
    except (ValueError,OSError):task_live={}
    if task_live.get('phase')=='WORKING' and task_live.get('index')==1:
     fault=dict(mode='JAM',until=task_live['t']+3.);answer=gz(['topic','-t','/s10/tool/fault','-m','gz.msgs.StringMsg','-p','data: '+json.dumps(json.dumps(fault))]);dump(out/'fault_injection.json',dict(t=task_live['t'],fault=fault,exit=answer.returncode,scope='declared simulator actuator fault; no pose injection'));fault_sent=True
   reference=out/'reference/live.json'
   if not nav_started and reference.exists():
    try:ready_ref=read(reference)
    except (ValueError,OSError):ready_ref={}
    if ready_ref.get('valid') and ready_ref.get('t',0)>12:
     launch('nav_startup',['ros2','service','call','/lifecycle_manager/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 0}'],aux=True);nav_started=True
   time.sleep(.08)
  else:raise TimeoutError('WALL_WATCHDOG')
 except BaseException as exc:error=repr(exc)
 finally:
  for role in ['observer']:
   for rec,p in zip(records,children):
    if rec['role']==role and p.poll() is None:
     p.send_signal(signal.SIGINT)
     try:p.wait(timeout=20)
     except subprocess.TimeoutExpired:forced.append(role);p.kill();p.wait(timeout=3)
  try:
   shutdown=subprocess.run(['ros2','service','call','/lifecycle_manager/manage_nodes','nav2_msgs/srv/ManageLifecycleNodes','{command: 4}'],env=env,cwd=ROOT,capture_output=True,text=True,timeout=15)
   nav_shutdown=dict(exit=shutdown.returncode,output=shutdown.stdout[-2000:],stderr=shutdown.stderr[-1000:],passed=shutdown.returncode==0 and 'success=True' in shutdown.stdout);dump(out/'nav_shutdown.json',nav_shutdown)
  except BaseException as exc:nav_shutdown=dict(passed=False,error=repr(exc));dump(out/'nav_shutdown.json',nav_shutdown)
  time.sleep(.5)
  from shutdown_r08 import stop_ros
  stop_ros(records,children,forced,out)
  if server and server.poll() is None:
   try:gz(['service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true']);server.wait(timeout=15)
   except (subprocess.TimeoutExpired,OSError):forced.append('server');server.send_signal(signal.SIGINT)
   if server.poll() is None:
    try:server.wait(timeout=3)
    except subprocess.TimeoutExpired:server.kill();server.wait(timeout=3)
  for f in logs:f.close()
  for rec,p in zip(records,children):rec['exit']=p.returncode
  dump(out/'runtime.json',dict(passed=not error and not forced and bool(nav_shutdown and nav_shutdown['passed']) and all(p.returncode==0 for p in children),error=error,forced=forced,processes=records,wall_seconds=time.monotonic()-start,original_db_unchanged=sha(BASE/'maps/frozen/map.db')==source['source_database_sha256']))
 dump(APP/'results/R10'/(task+'_'+case+'_latest.json'),dict(path=str(out.relative_to(ROOT))));print(out,flush=True);return out
