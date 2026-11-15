"""Candidate only: model selection memory never replays old predictions."""
import time
import numpy as np
from common16 import config
from engine15 import Engine
from features16 import context,Trigger
class MemoryEngine:
 def __init__(self,group,store,learned=None):
  if group not in ['full','event','cache']:raise ValueError('GROUP')
  self.group=group;self.store=store;self.cfg=config();self.base=Engine('imm');self.trigger=Trigger(self.cfg)
  if learned is None:
   from trajectron15 import Trajectron
   learned=Trajectron()
  self.learned=learned
 def predict(self,frames,now=None):
  start=time.perf_counter();cpu=time.process_time();observe=time.perf_counter()
  for frame in frames:self.base.observe(frame)
  observe_ms=(time.perf_counter()-observe)*1000;frame=frames[-1]
  feature_start=time.perf_counter();fs=self.base.history.features(frame['t']);items=[]
  for f in fs:
   if frame['t']-f['t']>.6:continue
   r=self.base.history.tracks[f['id']][-1]
   if f['classification']!='person' and np.linalg.norm(r['velocity'])<.05:continue
   if len(items)>=6:break
   x=context(f,fs,frame);event,reasons=self.trigger.event(f['id'],frame['t'],x);items.append((f,x,event,reasons))
  for key in list(self.trigger.last):
   if key not in self.base.history.tracks:del self.trigger.last[key]
  feature_ms=(time.perf_counter()-feature_start)*1000
  predictions=[];decisions=[];infer_ms=query_ms=maintenance_ms=0.;learned_calls=imm_calls=0
  for f,x,event,reasons in items:
   answer=dict(hit=False,reason='NOT_QUERIED',query_ms=0.,maintenance_ms=0.)
   if self.group=='cache' and event and f['classification']=='person':answer=self.store.lookup(x,time.time() if now is None else now)
   full=f['classification']=='person' and (self.group=='full' or (event and not answer['hit']))
   ts=time.perf_counter()
   if full:prediction=self.learned.predict(f,fs,3.2);learned_calls+=1
   else:prediction=self.base.filters[f['id']].predict(3.2);imm_calls+=1
   infer_ms+=(time.perf_counter()-ts)*1000
   prediction.update(id=f['id'],size=f['size'],classification=f['classification'],observed_age_s=frame['t']-f['t'],candidate_status='LEARNED_PERSON' if full else 'IMM_CURRENT_STATE')
   predictions.append(prediction);query_ms+=answer['query_ms'];maintenance_ms+=answer['maintenance_ms']
   decisions.append(dict(id=f['id'],t=f['t'],event=event,event_reasons=reasons,selected='learned' if full else 'fresh_imm',context=x,lookup=answer))
  return dict(t=frame['t'],frame='odom',method=self.group,status='OK',forecasts=predictions,future_labels_used=False,
   memory=dict(decisions=decisions,observe_ms=observe_ms,feature_ms=feature_ms,query_ms=query_ms,maintenance_ms=maintenance_ms,inference_ms=infer_ms,learned_calls=learned_calls,imm_calls=imm_calls),
   wall_ms=(time.perf_counter()-start)*1000,cpu_ms=(time.process_time()-cpu)*1000)
