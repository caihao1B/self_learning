"""Bounded, read-only adapters for recorded task evidence (no ROS/control imports)."""
from collections import deque
from pathlib import Path
import json, math, time

STAGES = ('R12', 'R10', 'R09', 'R08', 'R07')

def json_value(value):
    """ROS/numpy scalar values and invalid measurements become valid JSON."""
    if isinstance(value,dict): return {k:json_value(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [json_value(v) for v in value]
    if hasattr(value,'item'): return json_value(value.item())
    if isinstance(value,float) and not math.isfinite(value): return None
    return value

def read_json(path, limit=8_000_000):
    try:
        if path.stat().st_size > limit:
            return {}
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}

def tail_records(path, limit=131072, count=120):
    """Ignore an unfinished last line; never read an unbounded runtime log."""
    try:
        with path.open('rb') as stream:
            size = stream.seek(0, 2)
            start = max(0, size-limit)
            stream.seek(start)
            raw = stream.read(limit)
        lines = raw.split(b'\n')
        if start:
            lines = lines[1:]
        lines = lines[:-1]
        rows = []
        for line in lines:
            try:
                row = json.loads(line)
                if isinstance(row, dict): rows.append(row)
            except (ValueError, UnicodeError):
                pass
        return rows[-count:], start == 0 and len(rows) <= count
    except OSError:
        return [], False

def last(path, key=None, value=None):
    rows, _ = tail_records(path)
    return next((r for r in reversed(rows) if key is None or r.get(key) == value), {})

def age(path):
    try: return max(0., time.time()-path.stat().st_mtime)
    except OSError: return None

def freshness(stamp, now, maximum=.5):
    if not isinstance(stamp, (int,float)) or not isinstance(now, (int,float)):
        return 'MISSING', None
    delta = now-stamp
    if not math.isfinite(delta): return 'MISSING', None
    return ('FUTURE' if delta < -.1 else 'FRESH' if delta <= maximum else 'STALE'), delta

class Evidence:
    def __init__(self, app):
        self.app=Path(app).resolve(); self.results=self.app/'results'
        self._runs=[]; self._listed=0.; self._geometry={}; self._samples={}

    def runs(self):
        if time.monotonic()-self._listed < 3: return self._runs
        runs=[]
        for stage in STAGES:
            parent=self.results/stage
            if not parent.is_dir(): continue
            for p in parent.iterdir():
                if not p.is_dir() or not (p/'spec.json').is_file(): continue
                spec=read_json(p/'spec.json')
                if spec.get('task') not in ('spray','inspection','mowing'): continue
                stamp=p.name.split('_',1)[-1]
                runs.append(dict(id=stage+'/'+p.name, task=spec['task'], case=spec.get('case',''), sort=stamp))
        self._runs=sorted(runs,key=lambda r:r['sort'],reverse=True)[:160]
        self._listed=time.monotonic(); return self._runs

    def resolve(self, name='auto'):
        runs=self.runs()
        if name=='auto':
            # Prefer a live writer over a newly created but not yet started directory.
            for r in runs:
                p=self.results/r['id']
                if not (p/'runtime.json').exists() and (age(p/'live.json') or 1e9)<8:
                    return p,r['id']
            if not runs: return None,None
            name=runs[0]['id']
        if name not in {r['id'] for r in runs}: raise ValueError('Unknown run')
        p=(self.results/name).resolve()
        if not p.is_relative_to(self.results.resolve()): raise ValueError('Run outside results')
        return p,name

    def geometry(self, name='auto'):
        p,rid=self.resolve(name)
        if p is None:return {}
        file=p/'mission.json'
        try:key=(rid,file.stat().st_mtime_ns)
        except OSError:return {}
        if key in self._geometry:return self._geometry[key]
        cfg=read_json(file);spec=read_json(p/'spec.json');clusters=cfg.get('clusters',[])
        cluster=next((c for c in clusters if c.get('id',c.get('cluster'))==spec.get('cluster')), clusters[0] if clusters else {})
        segments=[]
        for s in cluster.get('segments',[]):
            pts=s.get('poses',[]);stride=max(1,len(pts)//300)
            sample=pts[::stride]
            if pts and sample[-1]!=pts[-1]:sample.append(pts[-1])
            segments.append(dict(id=s.get('id'),tool_on=s.get('tool_on',False),direction=s.get('direction'),points=sample))
        g=dict(run=rid,task=spec.get('task'),cluster=spec.get('cluster'),
               allowed=cfg.get('allowed_prior'),area=cluster.get('area_map'),
               points=cluster.get('points',[]),segments=segments,route=cfg.get('route_map',[]),
               original_point_count=cluster.get('original_point_count'),work_area_m2=cluster.get('work_area_m2'),
               omitted=cluster.get('omitted_original_work',[]),
               versions={k:cfg.get(k) for k in ['version','map_revision','source_plan_version','safe_execution_plan']},
               rig=cfg.get('rig',{}),vehicle=cfg.get('vehicle',{}))
        total=len(g['points']) if spec.get('task')=='spray' else len(g['segments'])
        limit=spec.get('point_limit',0) if spec.get('task')=='spray' else spec.get('segment_limit',0)
        g.update(selected_count=min(limit,total) if limit else total,original_count=total,
                 selection_scope='Current run selection; unselected original obligations remain in the plan')
        if spec.get('task')=='inspection':
            original=cfg.get('original_route_length_m')
            if original is None:
                route=g['route'];original=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(route,route[1:]))
            distance=spec.get('distance_limit_m',0.)
            g.update(original_route_length_m=original,selected_route_length_m=min(distance,original) if distance else original)
        if len(self._geometry)>8:self._geometry.clear()
        self._geometry[key]=g;return g

    def state(self, name='auto'):
        p,rid=self.resolve(name)
        if p is None:return dict(available=False,runs=self.runs())
        spec=read_json(p/'spec.json');live=read_json(p/'live.json');mission=read_json(p/'mission_result.json')
        closed=(p/'runtime.json').exists();now=live.get('t',mission.get('t'))
        wall_age=age(p/'live.json');status='CLOSED' if closed else 'LIVE' if wall_age is not None and wall_age<5 else 'STALE_OR_STARTING'
        loc=read_json(p/'reference/live.json') or live.get('loc') or {}
        gate=live.get('gate') or last(p/'control/gate.jsonl')
        risk=read_json(p/'risk/live.json') or last(p/'risk/risk.jsonl')
        bt=live.get('bt') or last(p/'decisions.jsonl')
        ledger=read_json(p/'ledger/live.json')
        ev,complete=tail_records(p/'mission_events.jsonl',limit=1_048_576,count=2000)
        def channel(label,source,data,maximum=.5):
            state,delta=freshness(data.get('t',data.get('stamp')),now,maximum)
            return dict(label=label,source=source,data=data,freshness=state,age_sim_s=delta)
        channels={
            'imu':channel('IMU / 偏置与去重力','inputs/inputs.jsonl',last(p/'inputs/inputs.jsonl','key','imu')),
            'wheel':channel('轮速 / 滑移检测','inputs/inputs.jsonl',last(p/'inputs/inputs.jsonl','key','wheel')),
            'lidar':channel('3D 雷达 / 采集对齐','mapping_inputs/frames.jsonl',last(p/'mapping_inputs/frames.jsonl')),
            'near':channel('近场雷达 / 多视角拼接','near/near.jsonl',last(p/'near/near.jsonl')),
            'icp':channel('激光里程计 / ICP','icp_gate/quality.jsonl',last(p/'icp_gate/quality.jsonl'),1.5),
            'gnss':channel('RTK GNSS / 独立校核','global_sources/sources.jsonl',last(p/'global_sources/sources.jsonl','kind','gnss')),
            'map':channel('冻结地图 / 匹配质量','quality/evidence.jsonl',last(p/'quality/evidence.jsonl'),1.),
        }
        observer=read_json(self.app/'monitor_state/live.json')
        if observer.get('run')!=rid or (age(self.app/'monitor_state/live.json') or 1e9)>3:observer={}
        frame=last(p/'video/frames.jsonl')
        analysis={}
        if closed:
            for file in ['analysis_spray_r10.json','analysis_mowing_r10.json','analysis_r08.json','analysis_r09.json']:
                analysis=read_json(p/file)
                if analysis:break
        samples=self._samples.setdefault(rid,deque(maxlen=1200))
        if isinstance(now,(int,float)) and (not samples or samples[-1]['t']!=now):
            samples.append(dict(t=now,v=gate.get('v'),w=gate.get('w'),raw_v=gate.get('raw_nav_v'),
                                pose=loc.get('pose'),innovation=loc.get('native_supervisor',{}).get('innovation_m')))
        if len(self._samples)>6:
            for k in list(self._samples):
                if k!=rid:del self._samples[k];break
        accepted=[x for x in ev if x.get('event')=='TOOL_RESULT' and x.get('state')=='SUCCEEDED' and not x.get('replayed')]
        arrivals=[x for x in ev if x.get('event')=='SEGMENT_ARRIVAL']
        return dict(available=True,run=dict(id=rid,task=spec.get('task'),case=spec.get('case'),
                    status=status,wall_age_s=wall_age,sim_seconds=spec.get('sim_seconds'),cluster=spec.get('cluster')),
                    sim_t=now,live=live,mission_result=mission,bt=bt,loc=loc,gate=gate,risk=risk,ledger=ledger,
                    sensors=channels,observer=observer,events=ev[-150:],events_complete=complete,
                    confirmed_points=len(accepted),confirmed_ids=[x.get('point') for x in accepted],
                    segment_arrivals=len(arrivals),samples=list(samples),video=frame,
                    analysis=analysis,recording=read_json(p/'recording_acceptance.json'),
                    stages={s:read_json(self.results/s/'latest.json').get('automated_status','NOT_RUN') for s in ['R08','R09','R10','R11','R12']},
                    source_scope='Observed task logs and optional ROS subscriptions. No simulator truth; no control authority.')
