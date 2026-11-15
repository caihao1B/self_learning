"""Thirty bounded, offline learning labs. Python standard library only.

Inputs marked synthetic are teaching values, never production defaults.
No ROS imports, network access, simulation, or writes to the robot workspace.
"""
from pathlib import Path
from collections import Counter, deque
import argparse, hashlib, heapq, itertools, json, math, os, random
import sqlite3, statistics, subprocess, sys, tempfile

ROOT = Path(__file__).resolve().parents[2]
E = ROOT / 'evidence'

def read(relative):
    return json.loads((ROOT / relative).read_text(encoding='utf-8'))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def lab01():
    """Historical scope: a subset has a different denominator."""
    r = read('evidence/validation/R12/spray/mission_result.json')
    # Field names are inspected rather than inferred from the video title.
    return dict(kind='historical evidence', result=r,
                lesson='Read selected count and original obligation separately.')

def lab02():
    """Paths and quoting, with a disposable directory containing a space."""
    with tempfile.TemporaryDirectory(prefix='robot learning ') as tmp:
        p = Path(tmp) / '笔记.txt'
        p.write_text('navigation != work\n', encoding='utf-8')
        assert p.is_file() and p.read_text(encoding='utf-8').endswith('\n')
        return dict(absolute=p.is_absolute(), file_name=p.name,
                    relative=str(p.relative_to(tmp)), suffix=p.suffix,
                    exists_during_lab=p.exists(), temporary_cleanup=True)

def lab03():
    """A child receives a copy of its parent's exported environment."""
    env = os.environ.copy(); env['ROBOT_LESSON'] = 'parent'
    code = "import os; print(os.environ['ROBOT_LESSON']); os.environ['ROBOT_LESSON']='child'"
    run = subprocess.run([sys.executable, '-c', code], env=env,
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0 and env['ROBOT_LESSON'] == 'parent'
    return dict(child_stdout=run.stdout.strip(), parent_copy=env['ROBOT_LESSON'],
                exit_code=run.returncode, shell_interpolation=False)

def lab04():
    """Validation prevents bool, NaN and absent data masquerading as a number."""
    def distance(x):
        if isinstance(x, bool) or not isinstance(x, (int, float)):
            raise TypeError('numeric distance required')
        if not math.isfinite(x) or x < 0:
            raise ValueError('finite nonnegative distance required')
        return x
    rejected = []
    for value in [None, True, float('nan'), -1]:
        try: distance(value)
        except (TypeError, ValueError): rejected.append(repr(value))
    assert len(rejected) == 4
    return dict(valid_m=distance(.03), rejected=rejected)

def lab05():
    """Teaching action protocol: acceptance, feedback and result are distinct."""
    events = [('GOAL_SENT', 0), ('ACCEPTED', 1), ('FEEDBACK', 2),
              ('CANCEL_REQUEST', 3), ('CANCELED', 4), ('STOP_CONFIRMED', 5)]
    accepted = False; terminal = None; stopped = False
    trace = []
    for event, t in events:
        if event == 'ACCEPTED': accepted = True
        if event == 'CANCELED': terminal = event
        if event == 'STOP_CONFIRMED': stopped = True
        trace.append(dict(t=t,event=event,accepted=accepted,terminal=terminal,stopped=stopped))
    assert trace[3]['terminal'] is None and trace[-1]['stopped']
    return dict(kind='synthetic protocol, not ROS action test', trace=trace)

def lab06():
    """SE(2) composition; degrees converted to radians exactly once."""
    yaw = math.radians(90); p = (1., 0.); translation = (2., 3.)
    mapped = (translation[0]+math.cos(yaw)*p[0]-math.sin(yaw)*p[1],
              translation[1]+math.sin(yaw)*p[0]+math.cos(yaw)*p[1])
    age = 10.2 - 10.0
    assert math.dist(mapped,(2.,4.)) < 1e-12
    return dict(map_point_m=mapped, age_sim_s=age, wrapped_rad=wrap(math.radians(359)))

def lab07():
    """Permission to move is an intersection of independent conditions."""
    inputs = dict(version=True, localization=True, perception=True,
                  command_fresh=True, tool_safe=True)
    rows = []
    for bad in [None,*inputs]:
        current=inputs.copy()
        if bad: current[bad]=False
        rows.append(dict(failed=bad,drive_allowed=all(current.values())))
    assert sum(r['drive_allowed'] for r in rows)==1
    return dict(kind='teaching gate, not production policy', cases=rows)

def lab08():
    """Execute the existing L01 real-evidence walkthrough."""
    r=subprocess.run([sys.executable,str(ROOT/'examples/inspect_spray.py')],
                     capture_output=True,text=True,timeout=10)
    assert r.returncode==0, r.stderr
    return dict(kind='historical evidence',output=r.stdout.strip())

def lab09():
    """Cell ownership and occupancy are different layers."""
    cells=[dict(id=i,zone='A',permanent=(i==2),observed=None) for i in range(6)]
    cells[3]['observed']='low_obstacle'
    statuses={c['id']:('BLOCKED_PRIOR' if c['permanent'] else
                     'BLOCKED_OBSERVED' if c['observed'] else 'UNKNOWN_OBSERVATION') for c in cells}
    assert statuses[0]=='UNKNOWN_OBSERVATION' and statuses[3]=='BLOCKED_OBSERVED'
    return dict(kind='synthetic layer example',status=statuses,
                lesson='Permitted prior does not imply currently observed free.')

def lab10():
    """Rear-axle bicycle radius and the larger outer front corner sweep."""
    L=1.2; delta=math.radians(30); half_width=.5; front=1.5
    radius=L/math.tan(delta)
    outer=math.hypot(radius+half_width,front)
    v=.5; omega=v/radius; r=.2; track=1.
    left=(v-track*omega/2)/r; right=(v+track*omega/2)/r
    assert outer>radius and right>left
    return dict(kind='synthetic dimensions, not Dino calibration',rear_radius_m=radius,
                outer_front_sweep_m=outer,omega_rad_s=omega,wheel_rad_s=[left,right])

def lab11():
    """Delayed packets keep acquisition stamps; old arrivals can be rejected."""
    packets=[(1.0,.0),(1.1,.4),(1.2,.0),(1.3,.0)]
    queue=[(t+delay,t) for t,delay in packets];heapq.heapify(queue)
    last=-math.inf; accepted=[]; rejected=[]
    while queue:
        arrival,stamp=heapq.heappop(queue)
        if stamp<=last:rejected.append(dict(stamp=stamp,arrival=arrival));continue
        last=stamp;accepted.append(dict(stamp=stamp,arrival=arrival))
    assert [p['stamp'] for p in rejected]==[1.1]
    return dict(kind='synthetic timing',accepted=accepted,rejected=rejected)

def lab12():
    """Scalar Kalman correction, for intuition; production is multidimensional."""
    predicted=1.; P=.04; measured=1.2; R=.01
    innovation=measured-predicted; K=P/(P+R)
    estimate=predicted+K*innovation; posterior=(1-K)*P
    assert abs(estimate-1.16)<1e-12 and posterior<P
    return dict(predicted_m=predicted,measured_m=measured,K=K,
                corrected_m=estimate,posterior_variance_m2=posterior,NIS=innovation**2/(P+R))

def lab13():
    """One-dimensional loop closure distributed through a four-edge pose graph."""
    odom_edges=[1.02,1.03,-.98,-.97]; drift=sum(odom_edges)
    correction=drift/len(odom_edges); optimized=[x-correction for x in odom_edges]
    assert abs(sum(optimized))<1e-12
    return dict(kind='equal-weight 1D teaching graph',before_closure_m=drift,
                optimized_edges_m=optimized,after_closure_m=sum(optimized))

def lab14():
    """Antenna lever arm and a diagonal two-dimensional innovation test."""
    body=(10.,20.); lever=(.5,0.); yaw=math.pi/2
    antenna=(body[0]+math.cos(yaw)*lever[0],body[1]+math.sin(yaw)*lever[0])
    fix=(10.03,20.52); residual=(fix[0]-antenna[0],fix[1]-antenna[1])
    variance=.02**2+.05**2; nis=sum(x*x/variance for x in residual)
    assert nis<9.21
    return dict(kind='synthetic pose with production-style gate',antenna_m=antenna,
                residual_m=residual,NIS=nis,threshold_2d=9.21,
                omitted_arm_residual_m=math.dist(fix,body))

def lab15():
    """Robust statistics do not create missing observations."""
    heights=[.00,.01,.02,.01,.00,1.5]
    median=statistics.median(heights); mean=statistics.mean(heights)
    slope=math.degrees(math.atan2(.10,1.0))
    assert median<mean and slope>5
    return dict(kind='synthetic surface samples',median_m=median,mean_m=mean,
                slope_deg=slope,passes_five_deg=False,empty_cell='UNKNOWN')

def lab16():
    """TTC uses closing speed; following also needs geometry and freshness."""
    gap=2.; ego=.6; leader=.3
    closing=ego-leader; ttc=gap/closing if closing>0 else None
    same_direction=True; observed=True; path_valid=True
    teaching_allowed=all([same_direction,observed,path_valid,gap>=1.,ttc is None or ttc>=3.])
    assert teaching_allowed
    return dict(kind='synthetic thresholds',gap_m=gap,TTC_s=ttc,
                following_candidate=teaching_allowed,crossing_velocity_mps=[0.,.4],
                lesson='A transverse mover must not reuse this following decision.')

def lab17():
    """Cache break-even includes feature, query, validation and maintenance cost."""
    inference=8.; feature=1.; query=.5; verify=1.; maintenance=.2
    overhead=feature+query+verify+maintenance
    break_even=overhead/inference
    rows=[dict(hit_rate=h,cost_ms=overhead+(1-h)*inference,
               saving_ms=inference-overhead-(1-h)*inference) for h in [.1,.4,.8]]
    assert rows[0]['saving_ms']<0 and rows[-1]['saving_ms']>0
    return dict(kind='synthetic costs, not measured S16 speedup',break_even=break_even,cases=rows)

def connected(vertices,edges):
    vertices=set(vertices)
    if not vertices:return False
    seen={next(iter(vertices))};q=deque(seen)
    while q:
        u=q.popleft()
        for v in edges[u]:
            if v in vertices and v not in seen:seen.add(v);q.append(v)
    return seen==vertices

def lab18():
    """Validate partition outputs; this small lab does not implement METIS."""
    edges={i:[j for j in [i-1,i+1] if 0<=j<9] for i in range(9)}
    weights=[1,1,4,2,2,2,3,1,2]
    labels=[0,0,0,1,1,1,2,2,2]
    clusters=[[i for i,x in enumerate(labels) if x==k] for k in range(3)]
    loads=[sum(weights[i] for i in c) for c in clusters]
    assert all(connected(c,edges) for c in clusters)
    assert not connected([0,2],edges)
    return dict(kind='synthetic checked partition',loads=loads,
                imbalance=max(loads)/(sum(loads)/3)-1,connected=True)

def lab19():
    """Tiny exhaustive TSP checks a closed tour and separates work time."""
    cost=[[0,2,9,3],[2,0,3,8],[9,3,0,2],[3,8,2,0]]
    def length(order):return sum(cost[a][b] for a,b in zip(order,order[1:]+order[:1]))
    routes=[(0,*p) for p in itertools.permutations([1,2,3])]
    best=min(routes,key=length);distance=length(best)
    assert distance==10
    return dict(kind='four-node exact teaching solver; production OR-Tools heuristic',
                tour=best,distance_m=distance,travel_s=distance/.5,
                service_s=4*3,total_lower_bound_s=distance/.5+4*3)

def lab20():
    """Tool sweep uses unions; shortening a route does not shrink obligation."""
    original=100.; intervals=[(0.,5.),(4.,8.),(10.,12.)]
    merged=[]
    for start,end in sorted(intervals):
        if merged and start<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],end)
        else:merged.append([start,end])
    length=sum(b-a for a,b in merged);width=1.4;area=length*width
    assert length==10
    return dict(kind='parallel-strip teaching union, not polygon solver',confirmed_area_m2=area,
                original_area_m2=original,remaining_m2=original-area,coverage_fraction=area/original)

def lab21():
    """A path may be geometrically close but impossible for the vehicle."""
    poses=[(0.,0.,0.),(.1,0.,math.pi/2)]
    ds=math.dist(poses[0][:2],poses[1][:2]);dy=abs(wrap(poses[1][2]-poses[0][2]))
    curvature=dy/ds;limit=.5
    assert curvature>limit
    return dict(kind='synthetic endpoint counterexample',curvature_1_per_m=curvature,
                limit_1_per_m=limit,follow_path_admissible=False,
                navigate_to_pose_must_plan=True)

def lab22():
    """Pure pursuit and actuator speed-curvature limits."""
    x=1.;y=.2;v=.5;curvature=2*y/(x*x+y*y);omega=v*curvature
    stop_distance=v*v/(2*.4)+v*.2
    assert math.isfinite(omega) and stop_distance>.3
    return dict(kind='synthetic control calculation',curvature_1_per_m=curvature,
                angular_velocity_rad_s=omega,stop_distance_with_delay_m=stop_distance)

def lab23():
    """Priority ordering illustration, not a replacement for BehaviorTree.CPP."""
    def decide(e):
        if e['unsafe']:return 'STOP'
        if not e['fresh']:return 'STOP'
        if not e['localized']:return 'WAIT'
        if e['manual']:return 'HUMAN'
        if e['complete']:return 'COMPLETE'
        return 'EXECUTE'
    normal=dict(unsafe=False,fresh=True,localized=True,manual=False,complete=True)
    unsafe={**normal,'unsafe':True};unlocalized={**normal,'localized':False}
    assert decide(unsafe)=='STOP' and decide(unlocalized)=='WAIT'
    return dict(complete=decide(normal),complete_but_unsafe=decide(unsafe),
                complete_but_unlocalized=decide(unlocalized))

def lab24():
    """Cancellation handshake prevents a new owner before old motion stops."""
    active=True;cancel_ack=False;stopped=False;states=[]
    for event in ['CANCEL_REQUEST','CANCEL_ACK','MEASURED_STOP','NEW_GOAL']:
        if event=='CANCEL_ACK':cancel_ack=True;active=False
        if event=='MEASURED_STOP':stopped=True
        allowed=(not active) and cancel_ack and stopped
        if event=='NEW_GOAL':assert allowed
        states.append(dict(event=event,new_goal_allowed=allowed))
    assert not states[1]['new_goal_allowed']
    return dict(kind='synthetic state machine',trace=states)

def lab25():
    """In-memory SQLite unique event key avoids duplicate credit."""
    db=sqlite3.connect(':memory:')
    db.execute('CREATE TABLE confirmed(event TEXT PRIMARY KEY, amount REAL NOT NULL)')
    def confirm(event,amount):
        with db:db.execute('INSERT OR IGNORE INTO confirmed VALUES (?,?)',(event,amount))
        return db.execute('SELECT COALESCE(SUM(amount),0) FROM confirmed').fetchone()[0]
    first=confirm('job1:sample1',1.2);replay=confirm('job1:sample1',1.2);last=confirm('job1:sample2',.8)
    assert first==replay and last==2.
    db.close()
    return dict(kind='teaching transaction, production has richer feedback checks',
                first=first,replay=replay,after_new_feedback=last)

def lab26():
    """Check shipped immutable material without installing dependencies."""
    counts=[]
    for name in ['baseline/manifest.json','baseline/l02_manifest.json']:
        m=read(name);checked=0
        for e in m['entries']:
            if not e.get('local'):continue
            data=(ROOT/e['local']).read_bytes()
            assert hashlib.sha256(data).hexdigest()==e['sha256'],e['path']
            checked+=1
        counts.append(dict(manifest=name,checked=checked,commit=m['commit']))
    return dict(kind='actual snapshot verification',manifests=counts)

def lab27():
    """Read declared recording cases directly from frozen Python AST."""
    import ast
    p=ROOT/'baseline/source/optimization_v2/ws/src/task_platform/scripts/stage_r12.py'
    tree=ast.parse(p.read_text(encoding='utf-8'))
    cases={}
    def literal(node):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='dict':
            return {k.arg:literal(k.value) for k in node.keywords}
        return ast.literal_eval(node)
    for stmt in tree.body:
        if isinstance(stmt,ast.Assign):
            target=stmt.targets[0]
            if isinstance(target,ast.Name) and target.id=='CASES':cases.update(literal(stmt.value))
            elif isinstance(target,ast.Subscript) and isinstance(target.value,ast.Name) and target.value.id=='CASES':
                cases[ast.literal_eval(target.slice)]=literal(stmt.value)
    assert cases['spray']['point_limit']==3 and cases['mowing']['segment_limit']==5
    return dict(kind='actual source, parsed not imported',cases=cases)

def lab28():
    """Freshness badge must be derived from timestamps, not file existence."""
    def status(now,stamp,valid):
        if stamp>now:return 'FUTURE_STAMP'
        if now-stamp>.3:return 'STALE'
        return 'VALID' if valid else 'REJECTED'
    rows=[status(10,9.9,True),status(10,9,True),status(10,9.9,False),status(10,11,True)]
    assert rows==['VALID','STALE','REJECTED','FUTURE_STAMP']
    return dict(kind='synthetic GUI semantics',badges=rows)

def lab29():
    """Inspect the shipped failure summary without inventing cause counts."""
    p=E/'failure_summary.json'
    d=json.loads(p.read_text(encoding='utf-8'))
    return dict(kind='historical classified attempts',summary=d,
                limitation='Stop-code frequency is not an independently diagnosed root-cause frequency.')

def lab30():
    """Dependency closure identifies artifacts invalidated by a geometry change."""
    graph={'vehicle':['plan','sensor_extrinsics','controller'],
           'plan':['task_version'],'sensor_extrinsics':['localization'],
           'controller':['short_trial'],'localization':['short_trial'],
           'task_version':['short_trial'],'short_trial':['full_trial']}
    todo=deque(['vehicle']);affected=set()
    while todo:
        node=todo.popleft()
        if node in affected:continue
        affected.add(node);todo.extend(graph.get(node,[]))
    assert 'full_trial' in affected and 'sensor_extrinsics' in affected
    return dict(kind='teaching dependency graph',changed='vehicle',recheck=sorted(affected))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('chapter',choices=[f'{i:02}' for i in range(1,31)]+['all'])
    a=p.parse_args();numbers=range(1,31) if a.chapter=='all' else [int(a.chapter)]
    for number in numbers:
        result=globals()[f'lab{number:02}']()
        print(json.dumps(dict(chapter=f'{number:02}',result=result,passed=True),ensure_ascii=False,allow_nan=False))

if __name__=='__main__':main()
