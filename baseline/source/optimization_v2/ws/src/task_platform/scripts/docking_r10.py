"""A stable observed stop, not one lucky pose sample, admits point work."""
import math,statistics

def stable_dock(history,target):
 if not history:return False,dict(reason='NO_OBSERVATION')
 times=[x[0] for x in history];xy=[x[1] for x in history];median=[statistics.median(p[j] for p in xy) for j in [0,1]];dist=math.dist(median,target);errors=sorted(math.dist(p,median) for p in xy);p95=errors[min(len(errors)-1,math.ceil(.95*len(errors))-1)];span=times[-1]-times[0];gap=max((b-a for a,b in zip(times,times[1:])),default=float('inf'));latest=math.dist(xy[-1],target)
 ok=len(xy)>=20 and span>=1.8 and gap<=.15 and dist<=.035 and latest<=.045 and p95<=.015
 return ok,dict(samples=len(xy),span_s=span,max_gap_s=gap,median_distance_m=dist,latest_distance_m=latest,position_scatter_p95_m=p95)
