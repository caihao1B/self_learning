from pathlib import Path
import os,sys,json,hashlib,datetime,fcntl,subprocess,html
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';APP=BASE/'three_tasks';PKG=BASE/'ws/src/task_platform';R1=BASE/'ws/src/task_runtime/scripts'
sys.path.insert(0,str(R1))
from common_r1 import read,dump,sha,canonical,stamp
TASKS=('spray','inspection','mowing')
def edition(stage):return APP/'editions'/stage
def check(stage):
 p=APP/'results'/stage/'latest.json'
 if not p.exists():return dict(passed=False,stage=stage,reason='NOT_RUN')
 d=read(p);bad=[x['path'] for x in d['artifacts'] if not (ROOT/x['path']).is_file() or sha(ROOT/x['path'])!=x['sha256']]
 return dict(passed=d['automated_status']=='PASS' and not bad,stage=stage,changed=bad,status=d['automated_status'])
def require(stage):
 if stage=='R01':
  from report_r1 import verify_receipt
  r=verify_receipt()
 else:r=check(stage)
 if not r['passed']:raise ValueError('PREDECESSOR_NOT_VALID '+stage+' '+json.dumps(r))
def seal(stage,summary,files,sources,next_stage):
 out=APP/'results'/stage;out.mkdir(parents=True,exist_ok=True)
 artifacts={str(Path(p).relative_to(ROOT)):dict(path=str(Path(p).relative_to(ROOT)),sha256=sha(p)) for p in [*files,*sources] if Path(p).is_file()}
 result=dict(stage=stage,automated_status='PASS' if summary['passed'] else 'PARTIAL',summary=summary,artifacts=list(artifacts.values()),next_step=next_stage if summary['passed'] else None,formal_credit=False,original_S17='PARTIAL',original_S18='PARTIAL',created_utc=stamp())
 dump(out/'latest.json',result);return result

def render(stage,title,summary,images):
 out=APP/'results'/stage;out.mkdir(parents=True,exist_ok=True)
 text='<html lang="zh"><meta charset="UTF-8"><title>'+html.escape(title)+'</title><style>body{max-width:1100px;margin:36px auto;font:17px/1.7 system-ui;background:#f3f6f8;color:#18313b}img{max-width:100%;background:white}pre{white-space:pre-wrap;background:white;padding:20px}</style><h1>'+html.escape(title)+'</h1><p>原 S17 / S18 仍为 PARTIAL；本阶段实测范围见下。</p>'
 for image in images:text+='<img src="'+os.path.relpath(image,out)+'">'
 text+='<pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre></html>';(out/'report.html').write_text(text);return out/'report.html'
def lock_session():
 f=(ROOT/'native_20261108/runs/session.lock').open('a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);return f
