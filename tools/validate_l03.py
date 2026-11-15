"""Bounded checks for L03; immutable input and portable website contracts."""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import urlsplit,unquote,parse_qs
import hashlib,json,re,ast
ROOT=Path(__file__).resolve().parents[1];errors=[];checks={}
manifest=json.loads((ROOT/'L03/input_manifest.json').read_text())
for e in manifest['files']:
 p=ROOT/e['path']
 if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=e['sha256']:errors.append('FROZEN_INPUT '+e['path'])
checks['unchanged_learning_files']=len(manifest['files'])
s=(ROOT/'web/book-data.js').read_text();data=json.loads(s.split('window.ROBOT_BOOK=',1)[1].rstrip().removesuffix(';'))
assert data['meta']['baseline']==manifest['robot_baseline']
checks['chapters']=len(data['chapters'])
if [c['id'] for c in data['chapters']]!=list(range(1,31)):errors.append('CHAPTER_SEQUENCE')
source={s['id']:s for s in data['sources']};docs={d['id']:d for d in data['documents']}
for row in source.values():
 original=(ROOT/row['local']).read_text()
 if original!=row['code']:errors.append('SOURCE_TEXT '+row['path'])
 if hashlib.sha256((ROOT/row['local']).read_bytes()).hexdigest()!=row['sha256']:errors.append('SOURCE_HASH '+row['path'])
checks['exact_source_files']=len(source)
class Page(HTMLParser):
 def __init__(self):super().__init__();self.links=[];self.ids=[];self.scripts=[]
 def handle_starttag(self,tag,attrs):
  d=dict(attrs)
  if 'id' in d:self.ids.append(d['id'])
  for k in ['href','src']:
   if k in d:self.links.append(d[k])
  if tag=='script':self.scripts.append(d.get('src'))
parsed={}
for x in [*data['chapters'],*data['documents']]:
 p=Page();p.feed(x['html']);parsed[str(x['id']) if 'short' in x else 'd'+x['id']]=p
 if len(p.ids)!=len(set(p.ids)):errors.append('DUPLICATE_ANCHOR '+x['title'])
root=Page();root.feed((ROOT/'index.html').read_text());links=root.links[:]
for p in parsed.values():links+=p.links
links += [r['path'] for r in data['resources']]
for u in links:
 if u.startswith('#/'):
  path,_,q=u[2:].partition('?');segments=path.split('/');kind=segments[0];target=None
  if kind=='chapter':target=parsed.get(segments[1])
  elif kind=='document':target=parsed.get('d'+segments[1])
  elif kind=='source':
   if len(segments)<2 or segments[1] not in source:errors.append('SOURCE_ROUTE '+u)
  elif kind not in ['home','contents','sources','materials','search','notebook','guide','status','resource']:errors.append('APP_ROUTE '+u)
  if kind in ['chapter','document']:
   if target is None:errors.append('DOCUMENT_ROUTE '+u)
   elif parse_qs(q).get('anchor',[''])[0] and parse_qs(q)['anchor'][0] not in target.ids:errors.append('DOCUMENT_ANCHOR '+u)
 elif u.startswith('#'):continue
 elif urlsplit(u).scheme:continue
 else:
  p=(ROOT/unquote(urlsplit(u).path)).resolve()
  if not p.is_relative_to(ROOT) or not p.is_file():errors.append('LOCAL_FILE '+u)
checks['resource_and_route_links']=len(links)
checks['search_sections']=len(data['search'])
checks['explicit_unbundled_references']=len(data['missing'])
for p in ['web/site.js','web/book-data.js','web/site.css','index.html']:
 text=(ROOT/p).read_text()
 if p in ['web/site.js','index.html'] and re.search(r'\bfetch\s*\(|XMLHttpRequest|https://[^"\s]+\.js',text):errors.append('RUNTIME_REMOTE_DEPENDENCY '+p)
checks['no_runtime_fetch_or_remote_scripts']=True
for p in (ROOT/'tools').glob('*l03*.py'):ast.parse(p.read_text())
checks['python_syntax']=True
portable=[]
for folder in ['web','L01','L02','L03','assets','baseline','evidence','examples']:
 for p in (ROOT/folder).rglob('*'):
  if not p.is_file() or '__pycache__' in p.parts:continue
  rel=p.relative_to(ROOT).as_posix();portable.append(rel)
  for part in p.relative_to(ROOT).parts:
   if re.search(r'[<>:"|?*]',part) or part.endswith((' ','.')) or re.fullmatch(r'(?:CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?',part,re.I):errors.append('WINDOWS_FILENAME '+rel)
if len({p.casefold() for p in portable})!=len(portable):errors.append('WINDOWS_CASE_COLLISION')
checks['windows_compatible_filenames']=len(portable)
checks['longest_relative_path_characters']=max(map(len,portable))
checks['retained_stage_states']=data['meta']['project_states']
expected={'R08':'PASS','R09':'PASS','R10':'PARTIAL','R11':'PARTIAL','R12':'PARTIAL','S17':'PARTIAL','S18':'PARTIAL'}
if checks['retained_stage_states']!=expected:errors.append('STATUS_CHANGED')
result=dict(passed=not errors,checks=checks,errors=errors,scope='L03 static content, links and source identity; no robot simulation.',windows_native_manual='PENDING_USER')
(ROOT/'validation/L03/static_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False));raise SystemExit(0 if not errors else 2)
