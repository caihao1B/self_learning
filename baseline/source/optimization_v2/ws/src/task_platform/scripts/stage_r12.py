"""Truthful separate recordings; partial upstream acceptance never becomes a full release."""
from common import *
import importlib.util
from prepare_r12 import build

CASES=dict(spray=dict(task='spray',case='tool_jam',point_limit=3,seconds=300),spray_full_A1=dict(task='spray',case='nominal',point_limit=0,seconds=1800),inspection=dict(task='inspection',case='nominal',seconds=480),inspection_leader=dict(task='inspection',case='leader',distance_limit=20.,seconds=150),inspection_blocked=dict(task='inspection',case='blocked',distance_limit=20.,seconds=120),mowing=dict(task='mowing',case='nominal',segment_limit=5,seconds=420),mowing_full_B1=dict(task='mowing',case='nominal',segment_limit=0,seconds=1800))

CASES['mowing_startup_perception_loss']=dict(task='mowing',case='startup_perception_loss',segment_limit=5,seconds=420)

def declared_scope_matches(receipt,spec,name):
 expected=CASES[name];fields={'seconds':'sim_seconds','distance_limit':'distance_limit_m'}
 if expected['task']=='mowing' and spec.get('segment_start',0)!=0:return False
 if receipt.get('case')!=name or receipt.get('task')!=expected['task'] or spec.get('task')!=expected['task']:return False
 if not all(spec.get(fields.get(k,k))==v for k,v in expected.items() if k!='task'):return False
 return expected['task']!='inspection' or spec.get('distance_limit_m')==expected.get('distance_limit',0.)

def recheck(receipt):
 """A saved PASS is not enough after code/config or video changes."""
 out=ROOT/receipt['path'];task=receipt['task'];spec=read(out/'spec.json')
 if task=='spray':
  from analyze_spray_r10 import analyze
 elif task=='mowing':
  from analyze_mowing_r10 import analyze
 else:
  from analyze_r08 import analyze
 analysis=analyze(out);rec=read(out/'video/recording.json');frames=[json.loads(x) for x in (out/'video/frames.jsonl').read_text().splitlines()];gates=[json.loads(x) for x in (out/'control/gate.jsonl').read_text().splitlines()];start=next((x['t'] for x in gates if x.get('mode')=='DRIVE' and abs(x.get('v',0))+abs(x.get('w',0))>.003),None)
 if receipt['case']=='mowing_startup_perception_loss':
  events=[json.loads(l) for l in (out/'mission_events.jsonl').read_text().splitlines()];start=min([e['t'] for e in events if e['event']=='START_WORK_REQUEST']+([start] if start is not None else []),default=None)
 end=analysis['task_result'].get('t',1e9);active=[f for f in frames if (start or 0)<=f['sim_t']<=end]
 checks=dict(current_task_evidence=analysis['passed'],video_hash=sha(out/'video/full.mp4')==receipt['source_video_sha256'],encoder=rec['valid'] and rec['encoder_exit']==0,frame_count=rec['frames']==len(frames),complete_timeline=bool(start is not None and frames and frames[0]['sim_t']<=start and frames[-1]['sim_t']>=end-.5),monotonic_clock=all(b['sim_t']>=a['sim_t'] and b['video_s']>a['video_s'] for a,b in zip(frames,frames[1:])),fresh_images=len(active)>=10 and max((f['image_wall_age_s'] for f in active),default=1e9)<=2.5 and max((b['sim_t']-a['sim_t'] for a,b in zip(active,active[1:])),default=1e9)<=1.)
 return dict(receipt,passed=all(checks.values()),current_recheck=checks,task_passed=analysis['passed'])

def record(name):
 require('R09')
 if name not in CASES:raise ValueError('UNKNOWN_RECORDING_CASE')
 spec=CASES[name].copy();task=spec.pop('task');path=edition('R12')/('runtime_'+task+'.py');binding=next(x for x in read(edition('R12')/'runtime_derivation.json')['records'] if x['task']==task)
 if task=='mowing':
  from prepare_mowing_video_r12 import build as build_video_transport
  path,binding=build_video_transport()
 if task=='spray':
  from prepare_spray_video_r12 import build as build_video_transport
  path,binding=build_video_transport()
 if sha(ROOT/binding['source'])!=binding['source_sha256'] or sha(path)!=binding['derived_sha256']:raise ValueError('REBUILD_R12_AFTER_RUNTIME_CHANGE')
 module=importlib.util.spec_from_file_location('r12_'+task,path);runtime=importlib.util.module_from_spec(module);module.loader.exec_module(runtime)
 # Long mowing and held-obstacle recordings previously exceeded image age on this host.
 # Lower only wall-clock pacing; task clock, actions and physical limits remain original.
 out=runtime.run(rtf=.3,**spec) if name=='inspection_blocked' else runtime.run(rtf=.4,**spec) if name=='mowing_full_B1' else runtime.run(rtf=.5,**spec) if name=='spray_full_A1' else runtime.run(**spec)
 if task=='spray':
  from analyze_spray_r10 import analyze
 elif task=='mowing':
  from analyze_mowing_r10 import analyze
 else:
  from analyze_r08 import analyze
 result=analyze(out);video=read(out/'video/recording.json') if (out/'video/recording.json').exists() else {};frames=[json.loads(l) for l in (out/'video/frames.jsonl').read_text().splitlines()] if (out/'video/frames.jsonl').exists() else [];task_result=result.get('task_result',{});events=[json.loads(l) for l in (out/'mission_events.jsonl').read_text().splitlines()];first_action=next((x['t'] for x in events if x['event'] in ['NAV_GOAL','SEGMENT_GOAL','FOLLOW_PATH_SENT','FOLLOW_PATH_GOAL','ROUTE_GOAL']),None)
 gates=[json.loads(l) for l in (out/'control/gate.jsonl').read_text().splitlines()];first_action=next((g['t'] for g in gates if g.get('mode')=='DRIVE' and abs(g.get('v',0))+abs(g.get('w',0))>.003),first_action)
 if name=='mowing_startup_perception_loss':first_action=min([e['t'] for e in events if e['event']=='START_WORK_REQUEST']+([first_action] if first_action is not None else []),default=None)
 coverage=bool(frames and frames[0]['sim_t']<=(first_action if first_action is not None else 15.) and frames[-1]['sim_t']>=task_result.get('t',1e9)-.5)
 active_frames=[f for f in frames if (first_action or 0)<=f['sim_t']<=task_result.get('t',0)]
 max_age=max((f['image_wall_age_s'] for f in active_frames),default=1e9);max_gap=max((b['sim_t']-a['sim_t'] for a,b in zip(active_frames,active_frames[1:])),default=1e9);quality=bool(len(active_frames)>=10 and max_age<=2.5 and max_gap<=1.0)
 receipt=dict(case=name,task=task,path=str(out.relative_to(ROOT)),task_passed=result['passed'],video_valid=video.get('valid',False),mission_timeline_complete=coverage,active_video_quality=quality,max_active_image_age_wall_s=max_age,max_active_frame_gap_sim_s=max_gap,passed=bool(result['passed'] and video.get('valid') and coverage and quality),task_terminal=task_result.get('terminal'),scope='Actual independent recording, retains full or selected original obligation count',sim_seconds=task_result.get('t'),video_seconds=video.get('duration_s'),source_video_sha256=sha(out/'video/full.mp4') if (out/'video/full.mp4').exists() else None,analysis_checks=result.get('checks',{}));dump(out/'recording_acceptance.json',receipt);dump(APP/'results/R12'/(name+'_latest.json'),receipt);return out,result,receipt

def main(command,args):
 out=APP/'results/R12';out.mkdir(parents=True,exist_ok=True)
 if command=='build':build();return 0
 if command=='view-live':
  active=[p for p in out.glob('*_20*') if p.is_dir() and (p/'spec.json').exists() and not (p/'runtime.json').exists()]
  if not active:raise ValueError('NO_ACTIVE_R12_SESSION; start a recording in another terminal first')
  run=max(active,key=lambda p:p.name.split('_',1)[1]);spec=read(run/'spec.json');prefix={'spray':'r10_','inspection':'r08_','mowing':'r10_mowing_'}[spec['task']];env=os.environ.copy();env['GZ_PARTITION']=prefix+spec['session']
  print('Observe existing session: '+str(run),flush=True);return subprocess.run(['gz','sim','--force-version','8','-g'],env=env).returncode
 if command=='record':
  _,_,d=record(args[0] if args else 'spray');print(json.dumps(d,ensure_ascii=False));return 0 if d['passed'] else 2
 if command=='export':
  from export_r12 import export
  export(args or ['spray','inspection','mowing']);return 0
 if command!='finalize':raise ValueError('build / record CASE / view-live / export [CASES] / finalize / check / open')
 with (out/'tests.log').open('w') as f:component=subprocess.run([str(BASE/'venvs/scene/bin/python'),'-m','unittest','discover','-s',str(PKG/'tests'),'-p','*_r12.py','-v'],stdout=f,stderr=subprocess.STDOUT,timeout=20)
 cases={}
 for name in CASES:
  pointer=out/(name+'_latest.json')
  if not pointer.exists():continue
  receipt=read(pointer)
  try:
   cases[name]=recheck(receipt);spec=read(ROOT/receipt['path']/'spec.json');expected=CASES[name]
   scope=declared_scope_matches(receipt,spec,name)
   cases[name]['current_recheck']['declared_scope']=scope;cases[name]['passed']=cases[name]['passed'] and scope
  except (OSError,ValueError,KeyError,IndexError,TypeError) as ex:cases[name]=dict(receipt,passed=False,current_recheck_error=repr(ex))
 available={d['task'] for d in cases.values() if d['passed']};pending=[]
 if component.returncode:pending.append('Recording scope regression tests failed')
 if available!=set(TASKS):pending.append('Missing qualified full-timeline task recordings: '+str(set(TASKS)-available))
 for stage in ['R10','R11']:
  if not check(stage)['passed']:pending.append(stage+' is not PASS')
 export_path=out/'media/manifest.json'
 if not export_path.exists():pending.append('Complete and shortened exports missing')
 else:
  media=read(export_path)
  for name in ['full','short']+(['behaviors'] if 'behaviors' in media else []):
   artifact=ROOT/media[name]['path']
   if not artifact.is_file() or sha(artifact)!=media[name]['sha256']:pending.append(name+' export hash changed')
  for e in media['episodes']:
   video=ROOT/e['full_path'];current=cases.get(e['case'],{})
   if not video.is_file() or sha(video)!=e['full_sha256'] or current.get('path')!=e['run']:pending.append('Export source changed: '+e['case'])
 summary=dict(passed=not pending,recording_components_passed=available==set(TASKS) and component.returncode==0,qualified_recorded_tasks=sorted(available),cases=cases,pending=pending,scope='Three independent real-camera recordings and labeled excerpts; original S17/S18 PARTIAL; no full six-cluster or mechanical work effectiveness claim')
 report=render('R12','R12 / 三任务录像与交付',summary,[]);files=list(edition('R12').rglob('*'))+list((out/'media').rglob('*'))+[report]+list(out.glob('*_latest.json'))
 for run in {d['path'] for d in cases.values()}:files+=list((ROOT/run).rglob('*'))
 files.append(out/'tests.log');sources=list((PKG/'scripts').glob('*_r12.py'))+list((PKG/'tests').glob('*_r12.py'))+list((ROOT/'docs/three_tasks/R12').glob('*.md'));seal('R12',summary,[p for p in files if p.name!='latest.json' and '__pycache__' not in p.parts],sources,None);print(json.dumps(summary,ensure_ascii=False));return 0 if summary['passed'] else 2
