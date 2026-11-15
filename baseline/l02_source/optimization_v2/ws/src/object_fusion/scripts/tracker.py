"""Short-horizon association only; S15 future behavior prediction is separate."""
import numpy as np
from scipy.optimize import linear_sum_assignment
class Tracker:
 def __init__(self):self.tracks={};self.next_id=1;self.last=None
 def update(self,t,observations):
  if self.last is not None and t<=self.last:raise ValueError('nonmonotonic tracking time')
  self.last=t
  for tr in self.tracks.values():
   dt=min(2.,t-tr['t']);F=np.eye(4);F[0,2]=F[1,3]=dt;G=np.array([[dt*dt/2,0],[0,dt*dt/2],[dt,0],[0,dt]]);tr['x']=F@tr['x'];tr['P']=F@tr['P']@F.T+G@G.T*.8;tr['t']=t;tr['state']='COASTING'
  ids=list(self.tracks);cost=np.full((len(ids),len(observations)),1e5);H=np.eye(4)[:2]
  for i,k in enumerate(ids):
   tr=self.tracks[k]
   for j,o in enumerate(observations):
    delta=np.array(o['position'][:2])-tr['x'][:2];S=tr['P'][:2,:2]+np.array(o['covariance']).reshape(3,3)[:2,:2];nis=float(delta@np.linalg.solve(S,delta));size=np.linalg.norm(np.array(o['size'])-tr['size'])
    if np.linalg.norm(delta)<1.5 and nis<9.21 and size<1.0:cost[i,j]=nis+size
  matches=[];ambiguous=set()
  if cost.size:
   rr,cc=linear_sum_assignment(cost)
   for i,j in zip(rr,cc):
    if cost[i,j]>=1e5:continue
    alternatives=np.sort(cost[:,j]);row=np.sort(cost[i]);uncertain=(len(alternatives)>1 and alternatives[1]-alternatives[0]<.5) or (len(row)>1 and row[1]-row[0]<.5)
    if uncertain:ambiguous.add(j);self.tracks[ids[i]]['state']='AMBIGUOUS_COAST';continue
    matches.append((ids[i],j))
  used=set()
  for k,j in matches:
   tr=self.tracks[k];o=observations[j];R=np.array(o['covariance']).reshape(3,3)[:2,:2];K=tr['P']@H.T@np.linalg.inv(H@tr['P']@H.T+R);tr['x']+=K@(np.array(o['position'][:2])-H@tr['x']);I=np.eye(4)-K@H;tr['P']=I@tr['P']@I.T+K@R@K.T;tr['size']=.7*tr['size']+.3*np.array(o['size']);tr['obs']=o;tr['last_seen']=t;tr['state']='OBSERVED';tr['hits']+=1;used.add(j)
  for j,o in enumerate(observations):
   if j in used or j in ambiguous:continue
   k=self.next_id;self.next_id+=1;self.tracks[k]=dict(id=k,t=t,last_seen=t,x=np.array([*o['position'][:2],0.,0.]),P=np.diag([.0036,.0036,.5,.5]),size=np.array(o['size']),obs=o,state='TENTATIVE',hits=1)
  for k in list(self.tracks):
   if t-self.tracks[k]['last_seen']>1.2:del self.tracks[k]
  result=[]
  for k,tr in self.tracks.items():
   o=tr['obs'];age=t-tr['last_seen'];result.append(dict(id=k,t=t,observation_t=tr['last_seen'],frame='odom',position=[*tr['x'][:2].tolist(),o['position'][2]],velocity=tr['x'][2:].tolist(),size=tr['size'].tolist(),covariance=tr['P'].reshape(-1).tolist(),classification=o.get('classification','unknown_obstacle') if age<.6 else 'unknown_obstacle',confidence=o.get('confidence',0.) if age<.6 else 0.,state=tr['state'],confirmed=tr['hits']>=3,observed_age_s=age,semantic=o.get('semantic',{})))
  return result,[int(j) for j in sorted(ambiguous)]
