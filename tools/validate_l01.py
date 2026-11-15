"""Check the teaching package's local links, fixed source hashes and examples.

This deliberately does not execute any ROS/Gazebo command from the manuscript.
"""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import urlsplit, unquote
import hashlib, json, re, subprocess

ROOT=Path(__file__).resolve().parents[1]
class Links(HTMLParser):
    def __init__(self):super().__init__();self.links=[];self.ids=[]
    def handle_starttag(self,tag,attrs):
        a=dict(attrs)
        if 'id' in a:self.ids.append(a['id'])
        for k in ['href','src']:
            if k in a:self.links.append(a[k])

manifest=json.loads((ROOT/'baseline/manifest.json').read_text())
errors=[];checked=0
for e in manifest['entries']:
    if not e['local']:continue
    p=ROOT/e['local'];checked+=1
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=e['sha256']:
        errors.append('SOURCE_HASH '+e['path'])
local_count=0;external_count=0
pages=[ROOT/'index.html',*sorted((ROOT/'baseline/source_views').glob('*.html'))]
all_ids=0
for path in pages:
    page=Links();page.feed(path.read_text());ids=set(page.ids);all_ids+=len(ids)
    if len(page.ids)!=len(ids):errors.append('DUPLICATE_HTML_ID '+str(path.relative_to(ROOT)))
    for url in page.links:
        u=urlsplit(url)
        if u.scheme in ['http','https','mailto']:external_count+=1;continue
        local_count+=1
        if u.path:
            target=(path.parent/unquote(u.path)).resolve()
            if not target.is_relative_to(ROOT) or not target.is_file():errors.append('LOCAL_LINK '+url)
        elif u.fragment and unquote(u.fragment) not in ids:errors.append('ANCHOR '+url)
for p in (ROOT/'L01').glob('*.md'):
    body=p.read_text().split('<!-- generated fixed-revision references -->')[0]
    definitions=set(re.findall(r'^\[([^\]]+)\]:',p.read_text(),re.M))
    for ref in re.findall(r'\[[^\]]+\]\[([^\]]+)\]',body):
        if ref not in definitions:errors.append(f'UNRESOLVED_REF {p.name}:{ref}')
run=subprocess.run(['python3',str(ROOT/'examples/inspect_spray.py')],capture_output=True,text=True,timeout=10)
if run.returncode:errors.append('PYTHON_EXAMPLE '+run.stderr[-500:])
precision=subprocess.run([str(ROOT/'build/precision_demo')],capture_output=True,text=True,timeout=10)
if precision.returncode or not all(x in precision.stdout for x in ['ordinary=0.100','precision=0.009','out_of_range=0.100']):errors.append('CPP_EXAMPLE')
stages=json.loads((ROOT/'validation/stage_checks.json').read_text())['records']
if any(r['check'].get('changed') for r in stages):errors.append('ROBOT_ARTIFACT_CHANGED')
if [(r['stage'],r['exit_code']) for r in stages]!=[('R08',0),('R09',0),('R10',2),('R11',2),('R12',2)]:errors.append('UNEXPECTED_STAGE_STATE')
result=dict(passed=not errors,errors=errors,frozen_snapshots_checked=checked,source_tree_files=len(manifest['source_files']),
            local_links_checked=local_count,external_links_present=external_count,html_ids=all_ids,html_pages_checked=len(pages),
            python_example_exit=run.returncode,cpp_example_exit=precision.returncode,
            scope='L01 documents, fixed bytes, local navigation and educational examples; no robot simulation; external project links are pinned to inspected Git tree, not all fetched individually')
(ROOT/'validation/package_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 2)
