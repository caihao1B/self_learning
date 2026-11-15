"""New, explicit known-terrain planning edition; immutable R09 reference retained.

The user approved replanning after the offline original-world conflict audit.
The known static ramp is declared as a new prior, never presented as an online
sensor discovery. Original work polygons remain the feedback denominators.
"""
from common import *
import copy,math,numpy as np,yaml
from planning_envelope_r10 import envelope as make_envelope
from shapely.geometry import shape,box,mapping,Polygon
from shapely import union_all
from footprint_r09 import body_polygon
from recovery_mowing_r10 import gear_runs,validate_candidate

def prepare():
    original_path=edition('R09')/'mission.json';original=read(original_path);old=original['clusters'][0]
    target=edition('R10')/'mowing_safe_plan.json';work=APP/'results/R10/safe_plan_generation_tracking_envelope_v3';work.mkdir(parents=True,exist_ok=True)
    binary=APP/'build/r10_safe_route/r10_safe_route'
    if not binary.exists():
        from build_safe_route_r10 import build
        binary=build()
    # Explicit new static planning prior authorized by the user. This does not
    # modify the frozen localization map, original prior, task areas or world.
    # Keep V2's hard prior unchanged. Its late grid showed that nearby lethal
    # cells came from this prior itself, so enlarging it moved the collision
    # boundary with the path. Additional tracking room belongs in the footprint.
    hazard=box(120.,44.,126.,47.).buffer(.60,join_style=2)
    navigation_prior=shape(original['allowed_prior']).difference(hazard)
    allowed=navigation_prior.buffer(-original['footprint_margin_m'])
    nav=yaml.safe_load((edition('R09')/'nav2.yaml').read_text());costmaps=[nav[k][k]['ros__parameters'] for k in ['local_costmap','global_costmap']]
    envelope,clearance=make_envelope(original['vehicle']['footprint'],max(c['footprint_padding'] for c in costmaps),max(c['resolution'] for c in costmaps))
    strips=[];omitted=[];source_areas=[];source_segments=[]
    for old_index,s in enumerate(old['segments']):
        if s['kind']!='COVERAGE':continue
        source_areas.append(Polygon(s['target_area']));source_segments.append(copy.deepcopy(s));groups=[];group=[]
        for world,p in zip(s['poses_world'],s['poses']):
            if allowed.covers(body_polygon(envelope,*world)):group.append((world,p))
            elif group:groups.append(group);group=[]
        if group:groups.append(group)
        admitted=0
        for index,group in enumerate(groups):
            length=sum(math.dist(a[1][:2],b[1][:2]) for a,b in zip(group,group[1:]))
            if length<1.:continue
            segment=copy.deepcopy(s);segment.update(id=s['id']+':safe:'+str(index),poses_world=[x[0] for x in group],poses=[x[1] for x in group],length_m=length,nominal_seconds=length/.35,source_segment_id=s['id'],source_segment_index=old_index,work_key=s['id'],work_target_pose=s['poses'][0],prior_revision='known_grade10_tracking_envelope_v3')
            strips.append(segment);admitted+=1
        if not admitted:omitted.append(dict(segment=s['id'],reason='NO_AT_LEAST_1M_SAFE_WORK_FRAGMENT',target_area=s['target_area'],requested_area_m2=s['requested_area_m2']))
    if len(strips)<2:raise RuntimeError('INSUFFICIENT_SAFE_WORK_FRAGMENTS')
    cfg=copy.deepcopy(original);cfg['allowed_prior']=mapping(navigation_prior);segments=[];connections=[]
    for index,strip in enumerate(strips):
        if segments:
            start=segments[-1]['poses'][-1];goal=strip['poses'][0];candidates=[]
            for trial in range(3):
                folder=work/f'connector_{index}_{trial}';folder.mkdir(exist_ok=True);request=dict(start=start,goal=goal,radius=old['turn_radius_m'],allowed=mapping(allowed),footprint=envelope,seed=41+index*31+trial,seconds=3.)
                dump(folder/'request.json',request)
                with (folder/'solver.log').open('w') as log:r=subprocess.run([str(binary),str(folder/'request.json'),str(folder/'result.json')],stdout=log,stderr=subprocess.STDOUT,timeout=12)
                if r.returncode:continue
                result=read(folder/'result.json');points=result['points'];valid,reason=validate_candidate(points,start,goal,cfg,60.)
                if not valid:dump(folder/'rejected.json',dict(reason=reason));continue
                runs=gear_runs(points)
                if result['length_m']>60.:continue
                candidates.append((result['length_m'],result,runs,str(folder.relative_to(ROOT))))
                if result['method'].startswith('direct') or result['length_m']<1.6*math.dist(start[:2],goal[:2])+8:break
            if not candidates:raise RuntimeError('NO_SAFE_OFFLINE_CONNECTOR '+str(index))
            _,result,runs,folder=min(candidates,key=lambda x:x[0]);connections.append(dict(before=strip['id'],length_m=result['length_m'],method=result['method'],evidence=folder))
            for gear_index,run in enumerate(runs):
                points=run['poses'];length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]));segments.append(dict(id=f'B1:safe_turn:{index}:{gear_index}',kind='TURN_TOOL_OFF',tool_on=False,direction=run['direction'],poses=points,poses_world=[[p[0]+28,p[1]+14,p[2]] for p in points],length_m=length,nominal_seconds=length/.35,row_id=None,source_segment_id=None,target_area=[],requested_area_m2=0.))
        segments.append(strip)
    cluster=copy.deepcopy(old);motion=sum(s['length_m'] for s in segments);work_m=sum(s['length_m'] for s in strips)
    cluster.update(segments=segments,start=segments[0]['poses_world'][0],end=segments[-1]['poses_world'][-1],motion_m=motion,work_m=work_m,tool_off_m=motion-work_m,estimated_seconds=motion/.25+len(segments)*5.+100.,source_original_segments=len(old['segments']),original_work_segments=source_segments,omitted_original_work=omitted,scope='New known-terrain safe execution edition. Original work polygons and cluster area unchanged; original 34 waypoint traversal remains historical PARTIAL.')
    if cluster['estimated_seconds']>cluster['budget_s']:raise RuntimeError('SAFE_PLAN_ESTIMATE_EXCEEDS_ORIGINAL_BUDGET')
    original_union=union_all(source_areas);new_union=union_all([Polygon(s['target_area']) for s in strips]+[Polygon(s['target_area']) for s in omitted])
    if original_union.symmetric_difference(new_union).area>1e-8:raise RuntimeError('ORIGINAL_WORK_AREA_CHANGED')
    report=dict(schema='R10-SAFE-EXECUTION-PLAN/2',source_reference=str(original_path.relative_to(ROOT)),source_sha256=sha(original_path),source_version=original['version'],source_reference_segments=34,original_reference_status='PARTIAL',original_world_unchanged=True,original_cluster_area_m2=old['work_area_m2'],original_work_target_union_m2=original_union.area,original_work_union_unchanged=True,navigation_prior=mapping(navigation_prior),known_static_exclusion=dict(geometry=mapping(hazard),grade_degrees=10.,qualification_degrees=5.,planning_buffer_m=.60,buffer_basis='Hard known-terrain prior retained exactly from V2. Late-grid audit identified prior-generated lethal cells; separate footprint reserve handles raster and tracking effects.',source='Original S02 grade_10 mesh; explicitly user-authorized new offline known-terrain prior, not an online sensor discovery'),planning_envelope=envelope,planning_clearance=clearance,envelope_source_sha256=sha(PKG/'scripts/planning_envelope_r10.py'),cluster=cluster,connections=connections,backend_sha256=sha(binary),backend_source_sha256=sha(PKG/'r10_backend/safe_route.cpp'),generator_sha256=sha(Path(__file__)),online_truth_used=False,formal_S17_S18='PARTIAL')
    report['revision']=canonical(report);dump(target,report);dump(work/'summary.json',{k:v for k,v in report.items() if k not in ['cluster','navigation_prior']});print(json.dumps(dict(path=str(target),segments=len(segments),work_fragments=len(strips),omitted_strips=len(omitted),motion_m=motion,estimated_seconds=cluster['estimated_seconds'],original_work_target_union_m2=original_union.area)));return report

if __name__=='__main__':prepare()
