"""Admission for tool-off endpoint recovery; no path or arrival substitution."""
import math

def terminal_straight(goal,distance=1.2,spacing=.04):
    """Same-pose arrival with a straight tangent before it; proposal, not admission."""
    if len(goal)!=3 or not all(math.isfinite(v) for v in goal) or not 0<distance<=1.2 or not 0<spacing<=.1:
        raise ValueError('INVALID_TERMINAL_APPROACH')
    x,y,h=goal;count=math.ceil(distance/spacing)
    return [[x-distance*(1-i/count)*math.cos(h),y-distance*(1-i/count)*math.sin(h),h] for i in range(count+1)]

def retreat_prefix(start,distance=.6,spacing=.04):
    """Proposal only: a straight tool-off retreat, later swept/costmap checked."""
    if not all(math.isfinite(v) for v in start) or not 0<distance<=.8 or spacing<=0:
        raise ValueError('INVALID_RETREAT_BOUND')
    count=math.ceil(distance/spacing);x,y,h=start
    return [[x-distance*i/count*math.cos(h),y-distance*i/count*math.sin(h),h] for i in range(count+1)]

def gear_runs(points):
    """Split at a sign change of motion along body heading; share the cusp pose."""
    runs=[];current=[];previous=0
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        if math.hypot(dx,dy)<1e-7:continue
        heading=a[2]+.5*math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        longitudinal=dx*math.cos(heading)+dy*math.sin(heading)
        if abs(longitudinal)<.9*math.hypot(dx,dy):raise ValueError('LATERAL_ACKERMANN_PATH')
        direction=1 if longitudinal>0 else -1
        if direction!=previous:
            if current:runs.append(dict(direction=previous,poses=current))
            current=[a];previous=direction
        current.append(b)
    if current:runs.append(dict(direction=previous,poses=current))
    if not runs:raise ValueError('NO_TRANSLATION')
    return runs

def tracking_distance(points,pose,cursor):
    """Distance to local traversed/current edges, not an overlapping future branch."""
    edges=list(zip(points[max(0,cursor-2):cursor+4],points[max(0,cursor-2)+1:cursor+5]))
    distances=[]
    for a,b in edges:
        dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
        u=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/den)) if den else 0.
        distances.append(math.hypot(pose[0]-a[0]-u*dx,pose[1]-a[1]-u*dy))
    return min(distances,default=math.dist(points[cursor][:2],pose[:2]))

def yaw_error(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))

def gear_permitted(previous,requested,stopped,tool_stopped):
    return not(previous and requested and previous!=requested) or bool(stopped and tool_stopped)

def arm_direction(previous,requested,phase,stopped,tool_stopped):
    """Select a normal segment's gear before starting its working implement."""
    return requested if phase=='READY' and requested in (-1,1) and stopped and tool_stopped else previous

def observed_cursor(points,pose,cursor,window_m=.20):
    """Advance only through a local prefix, never jump to a nearby return branch."""
    end=cursor;distance=0.
    while end+1<len(points) and distance+math.dist(points[end][:2],points[end+1][:2])<=window_m:
        distance+=math.dist(points[end][:2],points[end+1][:2]);end+=1
    admitted=[i for i in range(cursor,end+1) if math.dist(points[i][:2],pose[:2])<=.15 and yaw_error(points[i][2],pose[2])<=.10]
    return min(admitted,key=lambda i:math.dist(points[i][:2],pose[:2])+.25*yaw_error(points[i][2],pose[2])) if admitted else cursor

def validate_candidate(points,start,goal,cfg,max_length=12.):
    from shapely.geometry import shape
    from footprint_r09 import body_polygon
    if len(points)<2 or any(not all(math.isfinite(v) for v in p) for p in points):return False,'MALFORMED_PATH'
    if math.dist(points[0][:2],start[:2])>.15 or yaw_error(points[0][2],start[2])>.10:return False,'START_DISCONTINUITY'
    if math.dist(points[-1][:2],goal[:2])>.05 or yaw_error(points[-1][2],goal[2])>.08:return False,'GOAL_MISMATCH'
    # The authoritative prior is in world coordinates; navigation is ENU.
    allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m'])
    for p in points:
        if not allowed.covers(body_polygon(cfg['vehicle']['footprint'],p[0]+28,p[1]+14,p[2])):return False,'FOOTPRINT_OUTSIDE_PRIOR'
    length=0.
    for a,b in zip(points,points[1:]):
        ds=math.dist(a[:2],b[:2]);angle=yaw_error(a[2],b[2]);length+=ds
        if ds>.4:return False,'PATH_DISCONTINUITY'
        # Arc-length curvature from chord; identical cusp points may keep yaw.
        if ds<1e-5:
            if angle>1e-4:return False,'IN_PLACE_ROTATION'
        elif 2*math.sin(angle/2)/ds>cfg['vehicle']['steering']['max_curvature']+.005:return False,'CURVATURE_EXCEEDED'
        signed=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));steps=max(1,math.ceil(ds/.04),math.ceil(angle/.02))
        for j in range(1,steps):
            u=j/steps;p=[a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]),a[2]+u*signed]
            if not allowed.covers(body_polygon(cfg['vehicle']['footprint'],p[0]+28,p[1]+14,p[2])):return False,'SWEPT_FOOTPRINT_OUTSIDE_PRIOR'
    if not math.isfinite(max_length) or max_length<=0:return False,'INVALID_LENGTH_BOUND'
    if length>max_length:return False,'RECOVERY_TOO_LONG'
    return True,'PRIOR_AND_KINEMATICS_VALID_OBSERVED_RISK_STILL_REQUIRED'
