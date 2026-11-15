"""Verify manuscript artifacts and bounded offline examples. Never run ROS commands."""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import urlsplit,unquote
import ast,hashlib,json,re,subprocess,sys
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'validation/L02';OUT.mkdir(parents=True,exist_ok=True)
errors=[];checks={};commands=[]
def run(name,args,timeout=30):
    r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,timeout=timeout)
    (OUT/(name+'.txt')).write_text(r.stdout+r.stderr)
    commands.append(dict(name=name,args=args,exit_code=r.returncode))
    if r.returncode:errors.append(name+': '+r.stderr[-300:])
    return r

count=0
for name in ['baseline/manifest.json','baseline/l02_manifest.json']:
    m=json.loads((ROOT/name).read_text());assert m['commit']=='9aff8337b7794f704c5556a973a5220e73f17808'
    for e in m['entries']:
        if not e.get('local'):continue
        count+=1;p=ROOT/e['local']
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=e['sha256']:errors.append('SNAPSHOT '+e['path'])
checks['frozen_entries']=count
catalog=json.loads((ROOT/'L02/catalog.json').read_text())
if [c['number'] for c in catalog]!=list(range(1,31)):errors.append('CHAPTER_SEQUENCE')
snippet_count=0;bash_blocks=0
for c in catalog:
    p=ROOT/c['path'];text=p.read_text()
    if c['authored_characters']<1400:errors.append('SHORT_AUTHORED_CHAPTER '+str(c['number']))
    if len(re.findall(r'^## ',text,re.M))<6:errors.append('CHAPTER_STRUCTURE '+str(c['number']))
    if not all(word in text for word in ['练习','代码']) or not re.search(r'故障|排错|排查|失败|常见问题',text):errors.append('CHAPTER_REQUIREMENTS '+str(c['number']))
    if '<!-- production -->' in text or '<!-- lab -->' in text:errors.append('UNBUILT_CHAPTER '+str(c['number']))
    block=text.split('<!-- generated-production-start -->')[1].split('<!-- generated-production-end -->')[0]
    snippets=re.findall(r'```[^\n]*\n(.*?)\n```',block,re.S)
    if len(snippets)!=len(c['sources']):errors.append('EXCERPT_COUNT '+str(c['number']))
    for row,code in zip(c['sources'],snippets):
        original='\n'.join((ROOT/row['local']).read_text().splitlines()[row['start']-1:row['end']])
        if code!=original or hashlib.sha256(code.encode()).hexdigest()!=row['sha256']:errors.append('EXCERPT_BYTES '+row['path'])
        snippet_count+=1
    for i,code in enumerate(re.findall(r'```bash\n(.*?)\n```',text,re.S)):
        # Parse only. Do not execute installation or simulation commands.
        r=subprocess.run(['bash','-n'],input=code,text=True,capture_output=True)
        if r.returncode:errors.append(f'BASH_SYNTAX {p.name}:{i} '+r.stderr)
        bash_blocks+=1
    definitions=set(re.findall(r'^\[([^\]]+)\]:',text,re.M))
    prose=re.sub(r'```[^\n]*\n.*?\n```','',text,flags=re.S)
    prose=re.sub(r'`[^`]*`','',prose)
    for ref in re.findall(r'\[[^\]]+\]\[([^\]]+)\]',prose):
        if ref not in definitions:errors.append('UNRESOLVED_REF '+p.name+':'+ref)
checks.update(chapters=len(catalog),production_excerpts=snippet_count,bash_blocks_parsed_not_executed=bash_blocks)

class Page(HTMLParser):
    def __init__(self):super().__init__();self.links=[];self.ids=[]
    def handle_starttag(self,tag,attrs):
        attrs=dict(attrs)
        if 'id' in attrs:self.ids.append(attrs['id'])
        for key in ['href','src']:
            if key in attrs:self.links.append(attrs[key])
pages=[ROOT/'index.html',*sorted((ROOT/'L02').rglob('*.html')),*sorted((ROOT/'baseline/l02_views').glob('*.html'))]
parsed={}
for p in pages:
    parser=Page();parser.feed(p.read_text());parsed[p.resolve()]=parser
    if len(parser.ids)!=len(set(parser.ids)):errors.append('DUPLICATE_ID '+str(p.relative_to(ROOT)))
local=external=0
id_sets={p:set(parser.ids) for p,parser in parsed.items()}
for p,parser in list(parsed.items()):
    for link in parser.links:
        u=urlsplit(link)
        if u.scheme in ['https','http','mailto']:external+=1;continue
        local+=1;target=(p.parent/unquote(u.path)).resolve() if u.path else p
        if not target.is_relative_to(ROOT) or not target.is_file():errors.append('LOCAL_LINK '+str(p.relative_to(ROOT))+' '+link);continue
        if u.fragment and target.suffix=='.html':
            other=parsed.get(target)
            if other is None:
                other=Page();other.feed(target.read_text());parsed[target]=other;id_sets[target]=set(other.ids)
            if unquote(u.fragment) not in id_sets[target]:errors.append('ANCHOR '+link)
checks.update(html_pages=len(pages),local_links_and_anchors=local,external_links_present=external)

labs=run('python_labs',[sys.executable,'examples/l02/labs.py','all'],30)
rows=[json.loads(l) for l in labs.stdout.splitlines()] if not labs.returncode else []
if len(rows)!=30 or not all(x['passed'] for x in rows):errors.append('PYTHON_LABS')
checks['python_labs_passed']=len(rows)
(ROOT/'build').mkdir(exist_ok=True)
run('compile_precision',['g++','-std=c++17','-Wall','-Wextra','-pedantic','examples/precision_demo.cpp','-o','build/precision_demo'],60)
r=run('precision_cpp',['build/precision_demo'])
if not all(x in r.stdout for x in ['ordinary=0.100','precision=0.009','out_of_range=0.100']):errors.append('PRECISION_OUTPUT')
run('compile_native',['g++','-std=c++17','-Wall','-Wextra','-pedantic','-I/usr/include/eigen3','examples/l02/native_checks.cpp','-o','build/l02_native_checks'],60)
r=run('native_cpp',['build/l02_native_checks'])
if 'PASS 9 direct production-helper checks' not in r.stdout:errors.append('NATIVE_OUTPUT')
checks['cpp_direct_helper_checks']=9
for p in [*list((ROOT/'tools').glob('*l02*.py')),ROOT/'examples/l02/labs.py']:
    ast.parse(p.read_text(),filename=str(p))
checks['python_syntax']=True
stage=json.loads((ROOT/'validation/stage_checks.json').read_text())['records']
checks['historical_stage_status']={r['stage']:r['check'].get('status') for r in stage}
checks['no_robot_simulation_executed']=True
result=dict(passed=not errors,checks=checks,errors=errors,commands=commands,
scope='L02 manuscript, fixed source excerpts, offline labs, direct pure C++ helpers, HTML links and Bash syntax. No simulation, installs or new-machine acceptance.',
windows_native_manual='PENDING_USER',L03='READY_FOR_IMPLEMENTATION' if not errors else 'PENDING_L02_FIXES')
(OUT/'package_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
raise SystemExit(0 if result['passed'] else 2)
