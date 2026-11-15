"""Seeded METIS / K-means candidates, connectivity repair and independent checks."""
import math,time,subprocess
import numpy as np
from shapely import union_all,STRtree
from shapely.geometry import shape
from geometry11 import components,polygons
from common11 import *

def metis(g,k,seed,c,out):
    x=[0];a=[];ew=[]
    for neighbors in g['adj']:
        for j,w in sorted(neighbors.items()):a.append(j);ew.append(w)
        x.append(len(a))
    request=dict(xadj=x,adjncy=a,adjwgt=ew,vwgt=g['iw'].tolist(),k=k,seed=seed,ufactor=c['metis_ufactor'])
    dump(out/'csr.json',request)
    r=subprocess.run([str(BIN),str(out/'csr.json'),str(out/'metis.json')],capture_output=True,text=True,timeout=25)
    dump(out/'metis_command.json',dict(command=r.args,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr))
    if r.returncode:raise ValueError('METIS_REJECTED: '+r.stderr.strip())
    return np.array(read(out/'metis.json')['labels'],dtype=int)

def kmeans(g,k,seed):
    xy=g['xy'];rng=np.random.default_rng(seed);ids=[int(rng.integers(len(xy)))];distance=((xy-xy[ids[0]])**2).sum(axis=1)
    for _ in range(1,k):
        j=int(rng.choice(len(xy),p=distance/distance.sum()));ids.append(j);distance=np.minimum(distance,((xy-xy[j])**2).sum(axis=1))
    centres=xy[ids].copy();labels=np.full(len(xy),-1,dtype=int)
    for _ in range(100):
        dist=((xy[:,None,:]-centres[None,:,:])**2).sum(axis=2);new=dist.argmin(axis=1)
        # Never silently reduce k after an empty Euclidean cluster.
        for label in range(k):
            if not np.any(new==label):
                counts=np.bincount(new,minlength=k);eligible=np.where(counts[new]>1)[0]
                j=int(eligible[np.argmax(dist[eligible,new[eligible]])]);new[j]=label
        if np.array_equal(labels,new):break
        labels=new;centres=np.array([xy[labels==i].mean(axis=0) for i in range(k)])
    return labels

def repair(g,labels,k,c):
    labels=np.asarray(labels,dtype=int).copy();adj=g['adj'];weights=g['weights'];events=[]
    if len(labels)!=len(adj) or set(labels)!=set(range(k)):raise ValueError('EMPTY_OR_INVALID_LABELS')
    raw=[len(components(adj,np.flatnonzero(labels==i))) for i in range(k)]
    if max(raw)>1:
        # Retain each largest component. Grow only across a real shared edge.
        cores=[set(components(adj,np.flatnonzero(labels==i))[0]) for i in range(k)]
        assigned=np.full(len(labels),-1,dtype=int)
        for i,core in enumerate(cores):assigned[list(core)]=i
        loads=np.array([weights[list(core)].sum() for core in cores]);remaining=set(np.flatnonzero(assigned<0));target=weights.sum()/k
        while remaining:
            candidates=[]
            for u in sorted(remaining):
                ns=sorted({int(assigned[v]) for v in adj[u] if assigned[v]>=0})
                for dest in ns:candidates.append(((loads[dest]+weights[u])/target+.05*(dest!=labels[u]),u,dest))
            if not candidates:raise ValueError('REPAIR_DISCONNECTED_INPUT')
            _,u,dest=min(candidates);assigned[u]=dest;loads[dest]+=weights[u];remaining.remove(u)
        events.append(dict(kind='connected_region_growth',moved=int(np.count_nonzero(labels!=assigned))))
        labels=assigned
    loads=np.bincount(labels,weights=weights,minlength=k);mean=weights.sum()/k;total_edges=sum(e[2] for e in g['edges']);moves=0
    # Single-cell exchanges preserve both donor and receiver connectivity.
    for iteration in range(c['repair_balance_passes']):
        changed=0
        for u in sorted(range(len(adj)),key=lambda x:(-loads[labels[x]],x)):
            src=int(labels[u]);neighbors=adj[u]
            choices=[]
            for dst in sorted({int(labels[v]) for v in neighbors if labels[v]!=src}):
                delta=3*((loads[src]-weights[u]-mean)**2+(loads[dst]+weights[u]-mean)**2-(loads[src]-mean)**2-(loads[dst]-mean)**2)/mean**2
                cut_delta=sum(w*(int(dst!=labels[v])-int(src!=labels[v])) for v,w in neighbors.items())
                score=delta+cut_delta/max(1,total_edges)
                if score < -1e-10:choices.append((score,dst))
            if not choices:continue
            donor=set(np.flatnonzero(labels==src));donor.remove(u)
            if not donor or len(components(adj,donor))!=1:continue
            _,dst=min(choices);labels[u]=dst;loads[src]-=weights[u];loads[dst]+=weights[u];changed+=1
        moves+=changed
        if not changed:break
    events.append(dict(kind='connectivity_preserving_balance',moved=moves,passes=iteration+1))
    # Canonical IDs are sorted by computed centroid, not predetermined locations.
    order=sorted(range(k),key=lambda i:tuple(g['xy'][labels==i].mean(axis=0)))
    labels=np.array([order.index(int(i)) for i in labels],dtype=int)
    return labels,dict(raw_components=raw,events=events)

def verify(domain,g,labels,k,c,regions=None):
    labels=np.asarray(labels,dtype=int)
    if len(labels)!=len(g['cells']) or set(labels)!=set(range(k)):return dict(passed=False,reason='ASSIGNMENT_OR_EMPTY_CLUSTER')
    regions=regions if regions is not None else [union_all([g['cells'][u] for u in np.flatnonzero(labels==i)]) for i in range(k)]
    union=union_all(regions);missing=domain.difference(union).area;outside=union.difference(domain).area
    overlaps=sum(regions[i].intersection(regions[j]).area for i in range(k) for j in range(i+1,k))
    graph_components=[len(components(g['adj'],np.flatnonzero(labels==i))) for i in range(k)]
    geometry_components=[len(polygons(r)) for r in regions]
    cadj=[{} for _ in range(k)];shared=[]
    for i in range(k):
        for j in range(i+1,k):
            length=regions[i].boundary.intersection(regions[j].boundary).length
            if length>c['adjacency_min_shared_m']:cadj[i][j]=1;cadj[j][i]=1;shared.append([i,j,float(length)])
    # Check each assigned polygon is actually contained in its exported owner region.
    assigned_area=max((g['cells'][u].difference(regions[int(labels[u])]).area for u in range(len(labels))),default=0)
    ids=[p['id'] for p in g['points']];point_valid=len(ids)==len(set(ids)) and all(regions[int(labels[p['unit']])].covers(shape({'type':'Point','coordinates':p['xy']})) for p in g['points'])
    tolerance=c['area_relative_tolerance']*domain.area
    checks=dict(exactly_k_nonempty=len(regions)==k and all(r.area>0 for r in regions),graph_connected=max(graph_components)==1,
        geometry_connected=max(geometry_components)==1 and all(r.is_valid for r in regions),cluster_adjacency_connected=len(components(cadj))==1,
        omission=missing<=tolerance,overlap=overlaps<=tolerance,outside=outside<=tolerance,assignment_geometry=assigned_area<=tolerance,points_unique=point_valid)
    return dict(passed=all(checks.values()),checks=checks,missing_m2=missing,overlap_m2=overlaps,outside_m2=outside,
        difference_ratio=(missing+outside)/domain.area,graph_components=graph_components,geometry_components=geometry_components,adjacency=shared,
        assigned_unit_count=len(labels),unique_point_count=len(ids))

def solve(meta,g,c,algorithm,seed,out):
    out.mkdir(parents=True,exist_ok=True);start=time.perf_counter();k=c['k']
    raw=metis(g,k,seed,c,out) if algorithm=='metis' else kmeans(g,k,seed)
    labels,rep=repair(g,raw,k,c);partition_seconds=time.perf_counter()-start
    regions=[union_all([g['cells'][u] for u in np.flatnonzero(labels==i)]) for i in range(k)]
    verification=verify(meta['domain'],g,labels,k,c,regions)
    if not verification['passed']:raise ValueError('PARTITION_VERIFICATION_FAILED: '+json.dumps(verification))
    loads=np.bincount(labels,weights=g['weights'],minlength=k)+c['cluster_setup_s'];cut=sum(e[2] for e in g['edges'] if labels[e[0]]!=labels[e[1]])
    cut_length=sum(e[3] for e in g['edges'] if labels[e[0]]!=labels[e[1]])
    lower=meta['domain'].area/(.9*meta['speed_mps']) if meta['zone']=='B' else len(g['points'])*meta['service_s']
    props=dict(status='PROVISIONAL',execution_allowed=False,frame=meta['frame'],map_id=meta['map_id'],zone=meta['zone'],variant=meta['variant'],algorithm=algorithm,seed=seed)
    fs=[]
    for i,r in enumerate(regions):
        us=np.flatnonzero(labels==i).tolist();unit_set=set(us);points=[p['id'] for p in g['points'] if p['unit'] in unit_set]
        fs.append(feature(r,**props,cluster=i,label_point=list(r.representative_point().coords[0]),unit_count=len(us),point_ids=points,area_m2=r.area,estimated_seconds=float(loads[i]),
          estimate_terms={name:float(sum(g['terms'][name][u]*g['terms']['terrain_factor'][u] for u in us)) for name in ['service_s','travel_s','turn_s']},setup_s=c['cluster_setup_s']))
    d=dict(**props,work_area_m2=meta['domain'].area,clusters=collection(fs),verification=verification,repair=rep,
        labels=labels.tolist(),raw_labels=raw.tolist(),assignment_sha256=identity(labels.tolist()),
        metrics=dict(max_estimate_s=float(max(loads)),loads_s=loads.tolist(),imbalance=float(max(loads)/loads.mean()),weighted_cut=int(cut),cut_length_m=float(cut_length),
          partition_wall_s=partition_seconds,geometry_build_wall_s=g['build_wall_s'],verify_and_export_wall_s=time.perf_counter()-start-partition_seconds,
          speed_mps=meta['speed_mps'],width_spacing_m=meta['swath_spacing_m'],optimistic_total_lower_bound_s=lower,
          hard_budget_lower_bound_infeasible=lower>k*c['hard_budget_s'],estimated_budget_status='EXCEEDS_30_MIN' if max(loads)>c['hard_budget_s'] else 'ESTIMATE_ONLY_NOT_VALIDATED',
          lower_bound_scope='ideal straight parallel work swaths, zero turn/transit/wait; not a general motion theorem',actual_task_sim_seconds=None),points=[p|dict(cluster=int(labels[p['unit']])) for p in g['points']])
    dump(out/'partition.json',d);dump(out/'clusters.geojson',d['clusters']);return d
