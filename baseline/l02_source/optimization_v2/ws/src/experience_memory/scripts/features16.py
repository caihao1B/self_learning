"""Causal context only: no track ID, absolute location or future in stored vector."""
import math,ctypes
import numpy as np
from common16 import BASE
REASONS=['ADMITTED','NONFINITE','VERSION_MISMATCH','EXPIRED','QUERY_TIMEOUT','STALE_INPUT','LOCALIZATION_OR_UNCERTAINTY','OUT_OF_DISTRIBUTION','CURRENT_CONFLICT','OPPOSITE_DIRECTION']
class Admission:
 def __init__(self,cfg):
  self.fn=ctypes.CDLL(str(BASE/'ws/install/experience_memory/lib/libmemory_admission.so')).memory_admit
  self.fn.argtypes=[ctypes.c_double]*7+[ctypes.c_int]*4+[ctypes.POINTER(ctypes.c_double)];self.fn.restype=ctypes.c_int
  self.limits=(ctypes.c_double*7)(cfg['max_observation_age_s'],cfg['max_context_age_s'],cfg['max_sigma_m'],cfg['max_speed_mps'],cfg['max_acceleration_mps2'],cfg['minimum_clearance_m'],cfg['minimum_direction_cosine'])
 def check(self,x,cosine=1.,version=True,expired=False,timeout=False):
  code=self.fn(x['age'],x['context_age'],x['sigma'],x['clearance'],cosine,x['speed'],x['accel'],x['loc_ok'],version,expired,timeout,self.limits)
  return REASONS[code]
def context(f,features,frame):
 s=np.array(f['state']);v=s[-3:,2:4].mean(0);previous=s[:3,2:4].mean(0);a=s[-3:,4:6].mean(0)
 c=frame.get('context',{});robot=c.get('robot',{});yaw=robot.get('yaw',0.);rot=np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])
 relative=rot@(s[-1,:2]-np.array([robot.get('x',0.),robot.get('y',0.)]));v=rot@v;previous=rot@previous;a=rot@a
 speed=float(np.linalg.norm(v));accel=float(np.linalg.norm(a));sigma=float(np.sqrt(max(0,np.linalg.eigvalsh(f['covariance']).max())))
 neighbors=[np.linalg.norm(np.array(z['state'])[-1,:2]-s[-1,:2]) for z in features if z['id']!=f['id']]
 neighbor_distance=float(min(neighbors,default=10.));close_neighbors=sum(d<3 for d in neighbors)
 # Proxy corridor is relative to CURRENT observed ego heading, not a fabricated map lane.
 crossing=abs(v[1])>abs(v[0]);mode='STOP' if speed<.08 else 'CROSS' if crossing else 'ALONG'
 vector=[*f['size'],*v,*previous,*a,min(neighbor_distance,5.)/5,sigma,f['confidence'],float(crossing),min(abs(relative[1]),5.)/5,min(close_neighbors,5)/5,robot.get('v',0.),robot.get('w',0.)]
 if len(vector)!=17:raise ValueError('FEATURE_DIMENSION')
 return dict(vector=list(map(float,vector)),category=f['classification'],mode=mode,velocity=v.tolist(),speed=speed,accel=accel,sigma=sigma,
  age=frame['t']-f['t'],context_age=max(frame['t']-c.get('t',-100),frame['t']-robot.get('t',-100)),loc_ok=bool(c.get('navigation_allowed',False)),
  clearance=float(np.linalg.norm(relative)-.66-.5*np.linalg.norm(f['size'][:2])),road_relation='CURRENT_EGO_FORWARD_CORRIDOR')
class Trigger:
 def __init__(self,cfg):self.cfg=cfg;self.last={}
 def event(self,identity,t,x):
  old=self.last.get(identity);v=np.array(x['velocity'])
  reasons=[]
  if old is None:reasons.append('NEW_HISTORY')
  else:
   if t-old[0]>=self.cfg['event_period_s']-1e-8:reasons.append('PERIOD')
   if np.linalg.norm(v-old[1])>=self.cfg['event_velocity_delta_mps']:reasons.append('MOTION_CHANGE')
   if x['mode']!=old[2]:reasons.append('MODE_CHANGE')
  if x['clearance']<self.cfg['minimum_clearance_m']:reasons.append('CURRENT_CONFLICT')
  if not x['loc_ok'] or x['context_age']>self.cfg['max_context_age_s']:reasons.append('LOCALIZATION_QUALITY')
  if reasons:self.last[identity]=(t,v,x['mode'])
  # ID is only an ephemeral event state; reset on track loss by engine.
  return bool(reasons),reasons or ['BETWEEN_EVENTS']

class CausalContext:
 """Bounded message buffers; select by source timestamp, never callback recency."""
 def __init__(self):
  from collections import deque
  self.status=deque(maxlen=200);self.robots=deque(maxlen=400)
 def add_status(self,j):self.status.append(dict(t=j['t'],navigation_allowed=j['navigation_allowed']))
 def add_robot(self,j):self.robots.append(dict(j))
 def at(self,t):
  statuses=[x for x in self.status if x['t']<=t];robots=[x for x in self.robots if x['t']<=t]
  s=dict(max(statuses,key=lambda x:x['t'])) if statuses else {}
  s['robot']=dict(max(robots,key=lambda x:x['t'])) if robots else {}
  return s
