"""Admission for the explicitly approved safe execution edition, not area credit."""
import copy, math
import yaml
from shapely.geometry import shape, Polygon
from shapely import union_all
from common import read, sha, canonical, edition, PKG, APP
from footprint_r09 import body_polygon
from recovery_mowing_r10 import validate_candidate, gear_runs, yaw_error
from planning_envelope_r10 import envelope as make_envelope


def validate(plan, original):
    old=original['clusters'][0]; new=plan['cluster']; segments=new['segments']
    old_work=[s for s in old['segments'] if s['kind']=='COVERAGE']
    original_by_id={s['id']:s for s in old_work}
    old_union=union_all([Polygon(s['target_area']) for s in old_work])
    retained=union_all([Polygon(s['target_area']) for s in new['original_work_segments']])
    if new['original_work_segments']!=old_work or old_union.symmetric_difference(retained).area>1e-8:
        raise ValueError('ORIGINAL_WORK_POLYGONS_CHANGED')
    if new['work_area_m2']!=old['work_area_m2'] or new['area_map']!=old['area_map']:
        raise ValueError('ORIGINAL_CLUSTER_AREA_CHANGED')
    if new['budget_s']!=old['budget_s'] or new['estimated_seconds']>old['budget_s']:
        raise ValueError('ORIGINAL_TIME_BUDGET_CHANGED_OR_EXCEEDED')
    prior=shape(plan['navigation_prior']); allowed=prior.buffer(-original['footprint_margin_m'])
    if prior.difference(shape(original['allowed_prior'])).area>1e-8:
        raise ValueError('NAVIGATION_AUTHORITY_EXPANDED')
    nav=yaml.safe_load((edition('R09')/'nav2.yaml').read_text())
    maps=[nav[k][k]['ros__parameters'] for k in ['local_costmap','global_costmap']]
    expected,policy=make_envelope(original['vehicle']['footprint'],max(c['footprint_padding'] for c in maps),max(c['resolution'] for c in maps))
    if plan.get('planning_envelope')!=expected or plan.get('planning_clearance')!=policy:
        raise ValueError('GRID_AND_TRACKING_ENVELOPE_CHANGED')
    cfg=copy.deepcopy(original);cfg['allowed_prior']=plan['navigation_prior']
    previous=None
    for s in segments:
        if previous and (math.dist(previous[:2],s['poses'][0][:2])>1e-6 or yaw_error(previous[2],s['poses'][0][2])>1e-6):
            raise ValueError('DISCONNECTED_EXECUTION_PATH')
        valid,why=validate_candidate(s['poses'],s['poses'][0],s['poses'][-1],cfg,60.)
        if not valid:raise ValueError(why)
        runs=gear_runs(s['poses'])
        if len(runs)!=1 or runs[0]['direction']!=s['direction']:raise ValueError('UNDECLARED_GEAR_CHANGE')
        for p,w in zip(s['poses'],s['poses_world']):
            if math.dist([p[0]+28,p[1]+14,p[2]],w)>1e-7:raise ValueError('FRAME_MISMATCH')
            if not allowed.covers(body_polygon(plan['planning_envelope'],*w)):raise ValueError('PLANNING_ENVELOPE_OUTSIDE_PRIOR')
        if s['kind']=='COVERAGE':
            source=original_by_id[s['work_key']]
            if s['target_area']!=source['target_area'] or s['requested_area_m2']!=source['requested_area_m2'] or s['work_target_pose']!=source['poses'][0]:
                raise ValueError('FEEDBACK_OBLIGATION_CHANGED')
        elif s['tool_on'] or s['target_area'] or s['requested_area_m2']!=0:raise ValueError('CONNECTOR_HAS_WORK_CREDIT')
        previous=s['poses'][-1]
    return dict(passed=True,execution_segments=len(segments),reference_original_segments=len(old['segments']),original_reference_status='PARTIAL',original_work_target_union_m2=old_union.area,original_cluster_area_m2=old['work_area_m2'],original_work_union_unchanged=True)


def load_checked(path=None):
    path=path or edition('R10')/'mowing_safe_plan.json'; plan=read(path)
    revision=plan.pop('revision'); computed=canonical(plan);plan['revision']=revision
    if computed!=revision:raise ValueError('SAFE_PLAN_REVISION_CHANGED')
    source=edition('R09')/'mission.json'
    if plan['source_sha256']!=sha(source):raise ValueError('REFERENCE_PLAN_CHANGED')
    for field,p in [('backend_sha256',APP/'build/r10_safe_route/r10_safe_route'),('backend_source_sha256',PKG/'r10_backend/safe_route.cpp'),('generator_sha256',PKG/'scripts/prepare_safe_plan_r10.py'),('envelope_source_sha256',PKG/'scripts/planning_envelope_r10.py')]:
        if sha(p)!=plan[field]:raise ValueError('SAFE_PLAN_GENERATOR_CHANGED '+field)
    validate(plan,read(source))
    return plan


def work_identity(segment):
    """Fragments resume one original ledger job, retaining its entire obligation."""
    return segment.get('work_key',segment['id']),segment.get('work_target_pose',segment['poses'][0])


def requested_target(cluster,selected,full):
    obligations=cluster.get('original_work_segments',selected) if full else selected
    return union_all([Polygon(s['target_area']) for s in obligations if s['kind']=='COVERAGE'])
