"""Feedback-only SQLite accounting; no ROS, simulator state, or planned-path credit."""
import sqlite3,json,hashlib,math,time
from shapely.geometry import Polygon,Point,box,mapping,shape
from shapely.affinity import rotate,translate
from shapely import wkb,set_precision,union

def canonical(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def latest_at(records,t):
 eligible=[x for x in records if x['t']<=t+1e-6]
 return max(eligible,key=lambda x:x['t']) if eligible else None
def footprint(x,y,yaw,length,width,erosion=0.):
 p=box(-length/2,-width/2,length/2,width/2).buffer(-erosion,join_style=2)
 return translate(rotate(p,yaw,use_radians=True),x,y)
class Ledger:
 def __init__(self,path,cfg):
  self.c=cfg;self.db=sqlite3.connect(str(path),isolation_level=None);self.db.execute('PRAGMA journal_mode=WAL');self.db.execute('PRAGMA synchronous=FULL');self.db.execute('PRAGMA busy_timeout=2000')
  self.db.executescript('''CREATE TABLE IF NOT EXISTS jobs(key TEXT PRIMARY KEY,fingerprint TEXT NOT NULL,goal TEXT NOT NULL,status TEXT NOT NULL,quantity REAL NOT NULL DEFAULT 0,credit BLOB,attempt TEXT,uncertain INTEGER NOT NULL DEFAULT 0,reason TEXT NOT NULL DEFAULT '',updated REAL NOT NULL DEFAULT 0);
CREATE TABLE IF NOT EXISTS events(id TEXT PRIMARY KEY,key TEXT,attempt TEXT,t REAL,accepted INTEGER,reason TEXT,quantity REAL,observation TEXT);
CREATE TABLE IF NOT EXISTS transitions(id INTEGER PRIMARY KEY,key TEXT,t REAL,status TEXT,reason TEXT);''')
  self.db.execute("UPDATE jobs SET status='RECOVERY_REQUIRED',uncertain=1,reason='PROCESS_RESTART_UNCONFIRMED_STOP' WHERE status='RUNNING'")
  self.active=None;self.previous=None;self.streak=None;self.last_seq={};self.rejected={}
 def goal(self,key):
  row=self.db.execute('SELECT goal FROM jobs WHERE key=?',(key,)).fetchone();return json.loads(row[0]) if row else None
 def snapshot(self,key,geometry=True):
  column='credit' if geometry else 'NULL'
  r=self.db.execute(f'SELECT status,quantity,{column},attempt,uncertain,reason,goal FROM jobs WHERE key=?',(key,)).fetchone()
  if not r:return dict(ok=False,error='UNKNOWN_KEY')
  status,q,blob,attempt,u,reason,g=r;g=json.loads(g)
  result=dict(ok=True,key=key,state=status,confirmed_quantity=q,remaining_quantity=max(0,g['requested_quantity']-q),uncertainty_pending=bool(u),attempt=attempt,reason=reason,operation=g['operation'])
  if geometry:
   area=Polygon(g['area']) if len(g['area'])>=3 else Point(g['target'][:2]).buffer(self.c['tool'].get('virtual_effect_radius_m',.2));credit=wkb.loads(blob) if blob else Polygon();residual=area.difference(credit) if g['operation']==2 else (area if q<g['requested_quantity'] else Polygon());result.update(residual_geojson=json.dumps(mapping(residual)),credit_geojson=mapping(credit))
  return result
 def reserve(self,g,attempt,now=0.):
  for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']:
   if g.get(k)!=self.c.get(k):return dict(ok=False,error='VERSION_'+k)
  if g.get('operation') not in [1,2] or not g.get('idempotency_key') or not g.get('task_id'):return dict(ok=False,error='INVALID_GOAL')
  if (g['operation']==1)!=(self.c['vehicle']=='diff'):return dict(ok=False,error='CAPABILITY')
  if g.get('frame')!='map' or not all(math.isfinite(x) for x in [*g['target'],g['requested_quantity'],g['timeout_sim_s']]) or not 0<g['timeout_sim_s']<=300 or g['requested_quantity']<=0:return dict(ok=False,error='GOAL_RANGE')
  if g['operation']==2:
   p=Polygon(g['area'])
   if not p.is_valid or p.area<.01 or g['requested_quantity']>p.area+1e-6:return dict(ok=False,error='TARGET_AREA')
  key=g['idempotency_key'];fp=canonical(g);r=self.db.execute('SELECT fingerprint,status,uncertain FROM jobs WHERE key=?',(key,)).fetchone()
  if r and r[0]!=fp:return dict(ok=False,error='IDEMPOTENCY_CONFLICT')
  if r and r[1]=='SUCCEEDED':return dict(self.snapshot(key),replayed=True)
  if r and (r[2] or r[1]=='RECOVERY_REQUIRED'):return dict(ok=False,error='RECOVERY_REQUIRED',**{k:v for k,v in self.snapshot(key).items() if k!='ok'})
  if self.active:return dict(ok=False,error='BUSY')
  self.db.execute('BEGIN IMMEDIATE')
  try:
   if not r:self.db.execute('INSERT INTO jobs(key,fingerprint,goal,status,attempt) VALUES(?,?,?,?,?)',(key,fp,json.dumps(g),'RUNNING',attempt))
   else:self.db.execute("UPDATE jobs SET status='RUNNING',attempt=?,reason='' WHERE key=?",(attempt,key))
   self.db.execute('INSERT INTO transitions(key,t,status,reason) VALUES(?,?,?,?)',(key,now,'RUNNING','RESERVED_NOT_COMPLETED'));self.db.execute('COMMIT')
  except BaseException:self.db.execute('ROLLBACK');raise
  self.active=key;self.previous=None;self.streak=None;return dict(self.snapshot(key),replayed=False)
 def guard(self,ctx,g,now,for_start=False):
  if not ctx or not -1e-6<=now-ctx.get('t',-100)<=self.c['context_age_s']:return 'CONTEXT_STALE'
  if not all(math.isfinite(x) for x in [ctx.get('v',float('nan')),ctx.get('w',float('nan')),ctx.get('yaw',float('nan')),*ctx.get('tool_xy',[float('nan')])]):return 'NONFINITE_CONTEXT'
  if not ctx.get('localization'):return 'LOCALIZATION'
  if not ctx.get('target_ok'):return 'TARGET_CONDITION'
  v,w=abs(ctx['v']),abs(ctx['w'])
  if g['operation']==1:
   if v>self.c['spray_stopped_v'] or w>self.c['spray_stopped_w']:return 'NOT_STOPPED'
   if math.dist(ctx['tool_xy'],g['target'][:2])>self.c['tool']['docking_error_m']:return 'NOT_DOCKED'
  else:
   if not ctx.get('perception'):return 'PERCEPTION'
   if ctx.get('segment')!='COVERAGE' or ctx.get('segment_id')!=g['segment_id']:return 'NON_WORK_SEGMENT'
   if not (0. if for_start else self.c['work_min_speed'])<=v<=min(ctx.get('max_speed',0),self.c['work_max_speed']) or w>self.c['work_max_yaw_rate']:return 'WORK_SPEED'
  if not ctx.get('perception'):return 'PERCEPTION'
  return 'OK'
 def observe(self,m,ctx,now):
  if not self.active:return dict(accepted=False,reason='NO_ACTIVE_JOB')
  key=self.active;g=self.goal(key);snap=self.snapshot(key,geometry=False);reason='OK';t=m.get('t',-100);seq=m.get('seq',-1);epoch=m.get('sensor_epoch','');identity=epoch+':'+str(seq)+':'+m.get('attempt','')
  if self.db.execute('SELECT 1 FROM events WHERE id=?',(identity,)).fetchone():return dict(accepted=False,reason='DUPLICATE')
  if m.get('key')!=key or m.get('attempt')!=snap['attempt']:return dict(accepted=False,reason='WRONG_ATTEMPT')
  if m.get('tool_revision')!=g['tool_revision'] or epoch!=self.c['session']:reason='VERSION'
  elif not math.isfinite(t) or not -1e-6<=now-t<=self.c['feedback_age_sim_s']:reason='TIMESTAMP'
  elif seq<=self.last_seq.get(epoch,-1):reason='SEQUENCE'
  elif not ctx or abs(ctx['t']-t)>self.c['pose_skew_s']:reason='POSE_SKEW'
  else:reason=self.guard(ctx,g,now)
  self.last_seq[epoch]=max(seq,self.last_seq.get(epoch,-1))
  if reason=='OK' and not all(math.isfinite(m.get(k,float('nan'))) for k in ['rpm_rad_s','flow']):reason='NONFINITE_FEEDBACK'
  if reason=='OK' and (not m.get('healthy') or not m.get('request_on') or abs(m.get('rpm_rad_s',0))<self.c['working_min_rad_s']):reason='TOOL_NOT_WORKING'
  prev=self.previous;dt=t-prev['t'] if prev else 0
  if reason=='OK':
   if prev is None or dt<=0 or dt>self.c['feedback_gap_s']:self.streak=t;reason='FIRST_OR_GAP'
   elif self.streak is None:self.streak=t;reason='CONFIRMING'
   elif t-self.streak<self.c['working_confirm_sim_s']:reason='CONFIRMING'
  else:self.streak=None
  q=snap['confirmed_quantity'];blob=self.db.execute('SELECT credit FROM jobs WHERE key=?',(key,)).fetchone()[0];credit=wkb.loads(blob) if blob else Polygon()
  if reason=='OK':
   if g['operation']==1:q+=max(0,min(1.,m.get('flow',0),prev.get('flow',0)))*dt
   else:
    erosion=self.c['pose_erosion_m']+math.hypot(.35,.45)*self.c['yaw_erosion_rad'];precision=self.c['ledger_precision_m'];p=footprint(*ctx['tool_xy'],ctx['yaw'],.70,self.c['tool']['virtual_effective_width_m'],erosion+precision).intersection(Polygon(g['area']).buffer(-precision));credit=union(credit,set_precision(p,precision),grid_size=precision);q=credit.area
  self.db.execute('BEGIN IMMEDIATE')
  try:
   self.db.execute('INSERT INTO events VALUES(?,?,?,?,?,?,?,?)',(identity,key,snap['attempt'],t,int(reason=='OK'),reason,q,json.dumps(dict(feedback=m,context=ctx))))
   self.db.execute('UPDATE jobs SET quantity=?,credit=?,updated=? WHERE key=?',(q,wkb.dumps(credit),now,key));self.db.execute('COMMIT')
  except BaseException:self.db.execute('ROLLBACK');raise
  self.previous=m if reason in ['OK','FIRST_OR_GAP','CONFIRMING'] else None
  self.rejected[reason]=self.rejected.get(reason,0)+1
  return dict(accepted=reason=='OK',reason=reason,quantity=q)
 def finish(self,key,state,reason,uncertain,t):
  if self.active!=key:return dict(ok=False,error='NOT_ACTIVE')
  s=self.snapshot(key);g=self.goal(key);threshold=g['requested_quantity']*(1. if g['operation']==1 else self.c['completion_fraction'])
  if state=='SUCCEEDED' and s['confirmed_quantity']+1e-9<threshold:state='PARTIAL' if s['confirmed_quantity']>0 else 'FAILED';reason='INSUFFICIENT_CONFIRMED_WORK'
  self.db.execute('BEGIN IMMEDIATE')
  try:
   self.db.execute('UPDATE jobs SET status=?,reason=?,uncertain=?,updated=? WHERE key=?',(state,reason,int(uncertain),t,key));self.db.execute('INSERT INTO transitions(key,t,status,reason) VALUES(?,?,?,?)',(key,t,state,reason));self.db.execute('COMMIT')
  except BaseException:self.db.execute('ROLLBACK');raise
  self.active=None;self.previous=None;self.streak=None;return self.snapshot(key)
 def close(self):self.db.execute('PRAGMA wal_checkpoint(TRUNCATE)');self.db.close()
