"""Three-task migration namespace. Frozen S01--S18 remain read-only."""
from pathlib import Path
import os,sys,json,hashlib,datetime
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'optimization_v2'
APP=BASE/'three_tasks'
PKG=BASE/'ws/src/task_runtime'
def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
def dump(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.tmp.'+str(os.getpid()))
    tmp.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n');os.replace(tmp,path)
def stamp():return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
def add17():sys.path.insert(0,str(BASE/'ws/src/acceptance_runner/scripts'))
