"""Clip complete work obligations into cells; retain boundary slivers and holes."""
import math,time
import numpy as np
import yaml
from shapely import set_precision,union_all,STRtree
from shapely.geometry import shape,box,Point,Polygon
from common11 import *

def parameters():
    c=yaml.safe_load((BASE/'config/partition.yaml').read_text())
    if not (isinstance(c['k'],int) and 2<=c['k']<=8 and .2<=c['cell_size_m']<=5):raise ValueError('PARTITION_CONFIG')
    return c

def polygons(g):
    if g.is_empty:return []
    if g.geom_type=='Polygon':return [g]
    if g.geom_type in ('MultiPolygon','GeometryCollection'):
        return [p for part in g.geoms for p in polygons(part)]
    return []

def components(adj,selected=None):
    unseen=set(range(len(adj))) if selected is None else set(selected);out=[]
    while unseen:
        start=min(unseen);unseen.remove(start);part=[start];stack=[start]
        while stack:
            for j in sorted(adj[stack.pop()]):
                if j in unseen:unseen.remove(j);part.append(j);stack.append(j)
        out.append(sorted(part))
    return sorted(out,key=lambda x:(-len(x),x[0]))

def load_domain(zone,variant,c):
    if zone not in ('A','B') or variant not in ('base','notch','holes'):raise ValueError('UNKNOWN_GEOMETRY')
    source=next(f for f in read(BASE/'maps/prior/workspaces.geojson')['features'] if f['properties']['id']==zone)
    workspace=shape(source['geometry']);cx,cy=workspace.centroid.coords[0]
    # Same documented S02 public boundary perturbations; no world/truth module imported.
    if variant=='notch':workspace=workspace.difference(box(cx-3,workspace.bounds[3]-8,cx+3,workspace.bounds[3]+1))
    if variant=='holes':workspace=workspace.difference(box(cx-11,cy+5,cx-5,cy+9))
    permanent=union_all([shape(f['geometry']) for f in read(BASE/'maps/prior/permanent.geojson')['features']])
    if not workspace.is_valid:raise ValueError('INVALID_WORKSPACE')
    domain=set_precision(workspace.difference(permanent),c['geometry_precision_m'])
    if domain.geom_type!='Polygon' or domain.area<=0:raise ValueError('DISCONNECTED_OR_EMPTY_WORK_DOMAIN')
    vehicle='diff' if zone=='A' else 'ackermann';tool='spray' if zone=='A' else 'mower'
    q=read(BASE/'config/vehicles/qualified_profiles.json')['vehicles'][vehicle]
    tools=yaml.safe_load((BASE/'config/tool_profiles.yaml').read_text());t=tools[tool]
    # Tool-centred circumscribed envelope is conservative for every yaw. It is a
    # screening layer, not an erosion of work obligations or an Ackermann route proof.
    ox,oy=t['mount_xyz'][:2];radius=max(math.hypot(x-ox,y-oy) for x,y in q['footprint'])
    radius=max(radius,math.hypot(t['collision_size'][0]/2,t['collision_size'][1]/2))+c['clearance_margin_m']
    centre=domain.buffer(-radius,quad_segs=8)
    contract=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text())
    loc=yaml.safe_load((BASE/'config/localization_supervisor.yaml').read_text())
    speed=min(q['selected_work_mps'],loc['nominal_speed'],contract['work_max_speed'])
    erosion=contract['pose_erosion_m']+math.hypot(.35,.45)*contract['yaw_erosion_rad']+contract['ledger_precision_m']
    spacing=min(tools['mower']['virtual_effective_width_m']-tools['mower']['nominal_overlap_m'],tools['mower']['virtual_effective_width_m']-2*erosion-contract['ledger_precision_m'])
    return dict(zone=zone,variant=variant,workspace=workspace,permanent=permanent.intersection(workspace),domain=domain,centre=centre,
      unresolved_boundary=domain.difference(centre),radius_m=radius,speed_mps=speed,swath_spacing_m=spacing,
      service_s=tools['spray']['service_duration_sim_s'],minimum_turn_radius_m=q['planning_min_radius_m'],
      map_id=read(BASE/'maps/prior/map_metadata.json')['map_id'],frame='prior_local_enu',frame_note='S02 prior coordinates; not registered to the S07 SLAM map frame')

def build_graph(domain,c,meta=None):
    started=time.perf_counter()
    if domain.geom_type!='Polygon' or not domain.is_valid or domain.area<=0:raise ValueError('DOMAIN_MUST_BE_CONNECTED_VALID_POLYGON')
    size=c['cell_size_m'];x0,y0,x1,y1=domain.bounds;cells=[];grid=[]
    for iy in range(math.floor(y0/size),math.ceil(y1/size)):
        for ix in range(math.floor(x0/size),math.ceil(x1/size)):
            g=set_precision(domain.intersection(box(ix*size,iy*size,(ix+1)*size,(iy+1)*size)),c['geometry_precision_m'])
            for piece in sorted(polygons(g),key=lambda p:(p.centroid.x,p.centroid.y,p.area)):
                if piece.area>0:cells.append(piece);grid.append([ix,iy])
            if len(cells)>c['max_nodes']:raise ValueError('NODE_BUDGET_EXCEEDED')
    if len(cells)<c['k']:raise ValueError('TOO_FEW_UNITS')
    xy=np.array([p.representative_point().coords[0] for p in cells]);tree=STRtree(cells);adj=[{} for _ in cells];edges=[]
    for i,g in enumerate(cells):
        for j in sorted(int(j) for j in tree.query(g,predicate='intersects') if j>i):
            shared=g.boundary.intersection(cells[j].boundary);length=shared.length
            if length<=c['adjacency_min_shared_m']:continue
            factor=c['mowing_along_strip_edge_factor'] if meta and meta['zone']=='B' and abs(xy[j,0]-xy[i,0])>abs(xy[j,1]-xy[i,1]) else 1
            ew=max(1,round(length*c['edge_weight_scale']*factor));adj[i][j]=ew;adj[j][i]=ew
            edges.append([i,j,ew,float(length)])
    if len(components(adj))!=1:raise ValueError('DISCONNECTED_CELL_GRAPH')
    points=[];counts=np.zeros(len(cells),dtype=int)
    if meta and meta['zone']=='A':
        step=c['spray_point_spacing_m']
        for iy in range(math.floor(y0/step),math.ceil(y1/step)):
            for ix in range(math.floor(x0/step),math.ceil(x1/step)):
                p=Point((ix+.5)*step,(iy+.5)*step)
                if not domain.covers(p):continue
                owners=sorted(int(v) for v in tree.query(p,predicate='intersects'))
                if not owners:raise ValueError('POINT_WITHOUT_UNIT')
                i=owners[0];counts[i]+=1
                points.append(dict(id=f'point_{ix}_{iy}',xy=list(p.coords[0]),unit=i,clearance_screen='PASS' if meta['centre'].covers(p) else 'NEEDS_ORIENTED_ROUTE_CHECK'))
    area=np.array([p.area for p in cells]);coarse=union_all([shape(f['geometry']) for f in read(BASE/'maps/prior/terrain_coarse.geojson')['features']]) if meta else Polygon()
    rough=np.array([g.intersection(coarse).area/g.area for g in cells]);factor=1+rough*(c['coarse_terrain_time_multiplier']-1)
    speed=meta['speed_mps'] if meta else .12;zone=meta['zone'] if meta else 'B'
    if zone=='A':
        service=counts*(meta['service_s']+c['spray_docking_overhead_s']);travel=area/c['spray_point_spacing_m']/speed
        turn=area/c['spray_point_spacing_m']/c['turn_distance_scale_m']*c['spray_turn_overhead_s']
    else:
        spacing=meta['swath_spacing_m'] if meta else .7;service=np.zeros(len(cells));travel=area/spacing/speed
        turn=area/spacing/c['turn_distance_scale_m']*c['mowing_turn_overhead_s']
    weights=(service+travel+turn)*factor
    iw=np.maximum(1,np.rint(weights*c['node_weight_scale'])).astype(np.int64)
    return dict(cells=cells,xy=xy,grid=grid,adj=adj,edges=edges,points=points,weights=weights,iw=iw,area=area,
       terms=dict(service_s=service.tolist(),travel_s=travel.tolist(),turn_s=turn.tolist(),terrain_factor=factor.tolist()),
       build_wall_s=time.perf_counter()-started)
