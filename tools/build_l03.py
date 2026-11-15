"""Build a fetch-free, portable learning station from unchanged L01/L02 inputs."""
from pathlib import Path
from html.parser import HTMLParser
from urllib.parse import urlsplit,unquote
import ast,hashlib,html,json,posixpath,re
import markdown
ROOT=Path(__file__).resolve().parents[1]
REV='9aff8337b7794f704c5556a973a5220e73f17808'
CAT=json.loads((ROOT/'L02/catalog.json').read_text())
PARTS=[(1,4,'项目问题与学习基础'),(5,8,'ROS 与完整系统'),(9,14,'环境、传感器与定位'),(15,17,'感知与动态预测'),(18,20,'覆盖规划与工时'),(21,25,'导航、决策与作业反馈'),(26,28,'环境搭建与人工操作'),(29,30,'故障分析与研究方法')]
manifest=json.loads((ROOT/'L03/input_manifest.json').read_text())
for entry in manifest['files']:
 assert hashlib.sha256((ROOT/entry['path']).read_bytes()).hexdigest()==entry['sha256'],entry['path']
rows={}
for name in ['baseline/manifest.json','baseline/l02_manifest.json']:
 for e in json.loads((ROOT/name).read_text())['entries']:
  if e.get('local'):rows.setdefault(e['path'],e)

def ident(path):return hashlib.sha256(path.encode()).hexdigest()[:12]
def plain(markup):
 class Text(HTMLParser):
  def __init__(self):super().__init__();self.parts=[]
  def handle_data(self,data):self.parts.append(data)
 p=Text();p.feed(markup);return ' '.join(p.parts)

def language(path):
 ext=Path(path).suffix.lower()
 return {'.py':'Python','.cpp':'C++','.cc':'C++','.hpp':'C++','.h':'C++','.inc':'C++','.sh':'Bash','.yaml':'YAML','.xml':'XML','.action':'ROS Action','.txt':'CMake'}.get(ext,'Text')

sources=[];source_paths={};view_paths={}
for e in rows.values():
 path=e['path'];local=e['local']
 if not path.startswith('optimization_v2/') or Path(path).suffix not in ['.py','.cpp','.cc','.hpp','.h','.inc','.sh','.yaml','.xml','.action','.txt']:continue
 if not local.startswith('baseline/'):continue
 code=(ROOT/local).read_text();sid=ident(path);symbols=[]
 if path.endswith('.py'):
  try:symbols=[dict(name=n.name,line=n.lineno) for n in ast.walk(ast.parse(code)) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef))]
  except SyntaxError:pass
 else:
  for i,line in enumerate(code.splitlines(),1):
   m=re.search(r'\b(?:inline\s+)?(?:bool|void|double|int|J|json|Eigen::Isometry3d|std::\w+(?:<.*?>)?)\s+(\w+)\s*\(',line)
   if m:symbols.append(dict(name=m[1],line=i))
 source=dict(id=sid,path=path,name=Path(path).name,lang=language(path),code=code,lines=len(code.splitlines()),local=local,url=e['url'],sha256=e['sha256'],chapters=[c['number'] for c in CAT if any(s['path']==path for s in c['sources'])],symbols=sorted(symbols,key=lambda x:x['line']))
 sources.append(source);source_paths[local]=sid
 for d in ['baseline/source_views','baseline/l02_views']:
  vp=d+'/'+e['sha256'][:12]+'-'+Path(path).name+'.html'
  if (ROOT/vp).exists():view_paths[vp]=sid
sources.sort(key=lambda s:s['path'])

doc_paths=sorted(set([*['L02/'+x for x in ['learning_routes.md','reference.md','RESULTS.md']],*[p.relative_to(ROOT).as_posix() for p in (ROOT/'L01').glob('*.md')],*[p.relative_to(ROOT).as_posix() for p in (ROOT/'evidence/docs').rglob('*.md')],*[p.relative_to(ROOT).as_posix() for p in (ROOT/'L03').glob('*.md')]]))
doc_ids={p:ident(p) for p in doc_paths}
chapter_paths={c['path']:c['number'] for c in CAT};chapter_paths.update({c['html']:c['number'] for c in CAT})

missing=[]

def render(text,path):
 md=markdown.Markdown(extensions=['tables','fenced_code','toc']);markup=md.convert(text)
 def link(m):
  attr,url=m[1],html.unescape(m[2]);u=urlsplit(url)
  if u.scheme or url.startswith('//'):return m[0]
  target=posixpath.normpath(posixpath.join(posixpath.dirname(path),unquote(u.path))) if u.path else path
  fragment=unquote(u.fragment)
  if attr=='href' and target in chapter_paths:
   href='#/chapter/'+str(chapter_paths[target])+(('?anchor='+fragment) if fragment else '')
  elif attr=='href' and target in view_paths:
   href='#/source/'+view_paths[target]+(('?line='+fragment[1:]) if re.fullmatch('L[0-9]+',fragment) else '')
  elif attr=='href' and target in source_paths:href='#/source/'+source_paths[target]
  elif attr=='href' and target in doc_ids:href='#/document/'+doc_ids[target]+(('?anchor='+fragment) if fragment else '')
  elif target=='L02/README.md' or target=='L02/index.html':href='#/contents'
  else:
   candidate=(ROOT/target).resolve()
   if attr=='href' and (not candidate.is_relative_to(ROOT) or not candidate.is_file()):
    mid=ident(path+'|'+url)
    if not any(x['id']==mid for x in missing):missing.append(dict(id=mid,origin=path,original=url,resolved=target))
    href='#/resource/'+mid
   else:href=target+('#'+fragment if fragment else '')
  return attr+'="'+html.escape(href,quote=True)+'"'
 markup=re.sub(r'(href|src)="([^"]+)"',link,markup)
 headings=[dict(level=int(n),id=unquote(i),text=plain(t)) for n,i,t in re.findall(r'<h([23]) id="([^"]+)"[^>]*>(.*?)</h\1>',markup,re.S)]
 return markup,headings

search=[];chapters=[]
for c in CAT:
 text=(ROOT/c['path']).read_text();markup,toc=render(text,c['path'])
 summary=plain(re.search(r'<p>(.*?)</p>',markup,re.S)[1])
 item=dict(id=c['number'],title=c['title'],short=re.sub(r'^第\s*\d+\s*章\s*','',c['title']),part=next(t for lo,hi,t in PARTS if lo<=c['number']<=hi),summary=summary,html=markup,toc=toc,path=c['path'],reading=c['html'],sources=[ident(s['path']) for s in c['sources']],characters=c['authored_characters'])
 chapters.append(item)
 sections=re.split(r'(?=<h[23] id=)',markup)
 for i,section in enumerate(sections):
  h=re.search(r'<h[23] id="([^"]+)"[^>]*>(.*?)</h[23]>',section,re.S)
  search.append(dict(type='chapter',title=c['title'],subtitle=plain(h[2]) if h else '章节开篇',text=plain(section),route='#/chapter/'+str(c['number'])+(('?anchor='+h[1]) if h else ''),chapter=c['number']))

docs=[]
for p in doc_paths:
 text=(ROOT/p).read_text();markup,toc=render(text,p);title=text.splitlines()[0].lstrip('# ')
 docs.append(dict(id=doc_ids[p],path=p,title=title,html=markup,toc=toc,kind='guide' if p.startswith('L03/') else 'historical' if p.startswith(('L01/','evidence/')) else 'appendix'))
 for i,section in enumerate(re.split(r'(?=<h[23] id=)',markup)):
  h=re.search(r'<h[23] id="([^"]+)"[^>]*>(.*?)</h[23]>',section,re.S)
  search.append(dict(type='material',title=title,subtitle=plain(h[2]) if h else p,text=plain(section),route='#/document/'+doc_ids[p]+(('?anchor='+h[1]) if h else '')))
for s in sources:
 search.append(dict(type='source',title=s['name'],subtitle=s['path'],text=s['code'],route='#/source/'+s['id'],lang=s['lang']))

labels={
 'architecture':('完整系统架构','观察定位、规划、BT、控制与工具反馈的责任分工。',[7]),
 'data_flow':('观测与真值的数据边界','区分在线观测支路与独立评估真值。',[6,11]),
 'spray_sequence':('点喷的完整动作链','导航、精停、工具反馈与残余恢复。',[8,25]),
 'code_flow':('一次运行的代码调用链','从命令入口追踪到动作和实际结果。',[3,8]),
 'l02_frames':('坐标与时间','区分 map、odom、车体与传感器外参。',[6]),
 'l02_fusion':('多源融合与质量门控','轮速、IMU、激光与全局修正的关系。',[12]),
 'l02_planning':('从工作区到执行路径','图划分、路径生成、工时和几何验证。',[18,19,20]),
 'l02_bt':('可解释行为选择','为什么等待、跟随、重规划与请求人工。',[23]),
 'l02_cancel':('动作取消与停止确认','WAIT/CANCEL 在工具启动阶段怎样生效。',[24]),
 'l02_ledger':('有效作业账本','有效观测、幂等工作键与原义务残余。',[25]),
 'gui_overview':('机器人监视 GUI · 总览','固定历史截图，不代表机器人当前正在运行。',[28]),
 'gui_sensors':('机器人监视 GUI · 传感器','学习数据年龄、来源和状态的观察方法。',[11,28])}
resources=[]
for stem,(title,desc,related) in labels.items():
 paths=list((ROOT/'assets').glob(stem+'.svg'))+list((ROOT/'assets').glob(stem+'.png'))
 if not paths:continue
 path=paths[0].relative_to(ROOT).as_posix();resources.append(dict(id=stem,type='image',title=title,description=desc,path=path,chapters=related,category='screenshots' if stem.startswith('gui_') else 'diagrams'))
for d in docs:
 resources.append(dict(id=d['id'],type='document',title=d['title'],description=d['path'],route='#/document/'+d['id'],path=d['path'],category='guides' if d['kind'] in ['guide','appendix'] else 'evidence',chapters=[]))
for p in sorted((ROOT/'evidence/validation').rglob('*.json')):
 if p.stat().st_size>40000:continue
 resources.append(dict(id=ident(p.relative_to(ROOT).as_posix()),type='file',title=p.name,description=p.relative_to(ROOT).as_posix(),path=p.relative_to(ROOT).as_posix(),category='evidence',chapters=[]))
for n in range(1,31):
 resources.append(dict(id='lab-'+str(n),type='file',title=f'第 {n:02} 章 · 离线练习实际输出',description='合成教学或真实历史数据，详见输出中的 kind；不是机器人新验收。',path=f'validation/L02/lab_{n:02}.json',category='labs',chapters=[n]))
meta=dict(title='农田机器人 · 自学工坊',baseline=REV,learning_commit='bd1f9c671934a5a523dee2271a5e47a339dcbf57',version='L03-1',project_states={'R08':'PASS','R09':'PASS','R10':'PARTIAL','R11':'PARTIAL','R12':'PARTIAL','S17':'PARTIAL','S18':'PARTIAL'},excerpt_count=56,site_url='https://ceihao-robot-learning.ceihao.chatgpt.site')
payload=dict(meta=meta,parts=[dict(first=a,last=b,title=t) for a,b,t in PARTS],chapters=chapters,sources=sources,documents=docs,resources=resources,search=search,missing=missing)
(ROOT/'web/book-data.js').write_text('/* Generated from fixed manuscripts. No runtime network access required. */\nwindow.ROBOT_BOOK='+json.dumps(payload,ensure_ascii=False,separators=(',',':')).replace('</',r'<\/')+';\n')
legacy=ROOT/'L03/legacy-l01.html'
if not legacy.exists():
 old=(ROOT/'index.html').read_text()
 def fix(m):
  u=m[2]
  if u.startswith(('#','http:','https:','data:','//')):return m[0]
  return m[1]+'="../'+u+'"'
 legacy.write_text(re.sub(r'(href|src)="([^"]+)"',fix,old))
(ROOT/'index.html').write_text((ROOT/'web/index.template.html').read_text())
summary=dict(passed=True,chapters=len(chapters),sources=len(sources),documents=len(docs),resources=len(resources),search_sections=len(search),unchanged_learning_inputs=len(manifest['files']),data_bytes=(ROOT/'web/book-data.js').stat().st_size,baseline=REV)
(ROOT/'validation/L03/build.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False))
