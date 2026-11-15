"""Copy only learning-site files to an already registered Sites checkout."""
from pathlib import Path
import argparse,hashlib,json,shutil,re
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--checkout',required=True,type=Path);a=p.parse_args();checkout=a.checkout.resolve()
identity=json.loads((ROOT/'L03/site.json').read_text());hosting=json.loads((checkout/'.openai/hosting.json').read_text())
assert hosting['project_id']==identity['project_id'] and hosting['static']['directory']=='dist'
assert (checkout/'.git').is_dir(),'Open the registered Site using the source helper first.'
assert json.loads((ROOT/'validation/L03/static_check.json').read_text())['passed']
browser=json.loads((ROOT/'validation/L03/browser_check.json').read_text());assert browser['passed']
for name,digest in browser['inputs'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'Recheck changed website: '+name
files=[]
for name in ['index.html','README.md','requirements-build.txt','web','L01','L02','L03','assets','baseline','evidence','examples','tools','validation']:
 target=ROOT/name
 for f in ([target] if target.is_file() else target.rglob('*')):
  if not f.is_file() or '__pycache__' in f.parts or f.name=='chrome.log' or f.suffix=='.pyc':continue
  # Publication receipts live on GitHub. Do not copy stale deployment state into a new version.
  if f.relative_to(ROOT).as_posix() in ['L03/deployment.json','validation/L03/site_package.json']:continue
  if f.is_symlink():raise RuntimeError('No symlinks in static package: '+str(f))
  raw=f.read_bytes()
  try:text=raw.decode('utf-8')
  except UnicodeDecodeError:text=''
  if re.search(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{50,}',text):raise RuntimeError('Credential-like material in '+str(f.relative_to(ROOT)))
  files.append((f,raw))
dist=checkout/'dist';dist.mkdir(exist_ok=True)
expected=set()
for f,raw in files:
 rel=f.relative_to(ROOT);dest=dist/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest);expected.add(dest)
for old in dist.rglob('*'):
 if old.is_file() and old not in expected:old.unlink()
records=[dict(path=f.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw)) for f,raw in files]
receipt=dict(passed=True,project_id=identity['project_id'],files=len(records),bytes=sum(x['bytes'] for x in records),records=records,scope='Exact static copy; excludes runtime, credentials, compilation and stale deployment receipts.')
(ROOT/'validation/L03/site_package.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in receipt.items() if k!='records'},ensure_ascii=False))
