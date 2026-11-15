"""Offline tracking error relative to the actual published reference, including detours."""
from shapely.geometry import LineString,Point
import numpy as np

def errors(truth,events,route_world):
 changes=sorted([e for e in events if e['event']=='ACTIVE_REFERENCE_CHANGED'],key=lambda e:e['t']);admitted={e['epoch'] for e in events if e['event']=='OBSERVED_PATCH_ADMITTED'};line=LineString(route_world);index=0;values=[];authorized=True
 for row in truth:
  while index<len(changes) and changes[index]['t']<=row['t']:
   e=changes[index]
   if 'reference_map' not in e:authorized=False
   else:line=LineString(np.asarray(e['reference_map'])[:,:2]+[28,14])
   # Rejected candidates may be published while stopped, but never admit travel.
   if e.get('patch') and e.get('epoch') not in admitted:
    until=changes[index+1]['t'] if index+1<len(changes) else truth[-1]['t'];points=[r['xyz'][:2] for r in truth if e['t']<=r['t']<=until]
    if points and max(np.linalg.norm(np.asarray(points)-points[0],axis=1))>.1:authorized=False
   index+=1
  values.append(line.distance(Point(row['xyz'][:2])))
 return values,authorized and bool(changes)
