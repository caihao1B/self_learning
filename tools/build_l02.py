"""Build L02 manuscript excerpts and offline reading edition, not L03 website.

Only frozen files and local teaching examples are used. No robot code imports.
"""
from pathlib import Path
from urllib.parse import urlsplit, unquote
import ast, hashlib, html, importlib.util, json, os, posixpath, re, subprocess
import markdown
from l02_catalog import ALIASES, SOURCES, PARTS

ROOT=Path(__file__).resolve().parents[1]
BOOK=ROOT/'L02'; CH=BOOK/'chapters'; OUT=ROOT/'validation/L02'
OUT.mkdir(parents=True,exist_ok=True)
REV='9aff8337b7794f704c5556a973a5220e73f17808'
entries={}
for name in ['baseline/manifest.json','baseline/l02_manifest.json']:
    m=json.loads((ROOT/name).read_text())
    assert m['commit']==REV
    for row in m['entries']:
        if row.get('local'):entries.setdefault(row['path'],row)

# The full L01 sample is retained as chapter 08, with only relative-link relocation.
p=CH/'08_spray_walkthrough.md'
if not p.exists():
    source=(ROOT/'L01/07_sample_spray.md').read_text()
    source=source.replace('# 从启动命令到三点点喷闭环','# 第 08 章 从启动命令到三点点喷闭环',1)
    def relocate(m):
        target=m[1]
        if urlsplit(target).scheme or target.startswith('#'):return m[0]
        path=posixpath.normpath(posixpath.join('L01',target))
        return ']('+posixpath.relpath(path,'L02/chapters')+')'
    source=re.sub(r'\]\(([^)]+)\)',relocate,source)
    before,defs=source.split('<!-- generated fixed-revision references -->',1)
    p.write_text(before+'\n<!-- production -->\n\n<!-- lab -->\n\n<!-- generated fixed-revision references -->'+defs)

spec=importlib.util.spec_from_file_location('l02_teaching_labs',ROOT/'examples/l02/labs.py')
labs=importlib.util.module_from_spec(spec);spec.loader.exec_module(labs)
labtext=(ROOT/'examples/l02/labs.py').read_text();lablines=labtext.splitlines()
labnodes={n.name:n for n in ast.walk(ast.parse(labtext)) if isinstance(n,ast.FunctionDef)}
views=ROOT/'baseline/l02_views';views.mkdir(exist_ok=True)
view_paths={ALIASES[alias] for selections in SOURCES.values() for alias,_,_ in selections}
expected_views=set()
for row in entries.values():
    if row['path'] not in view_paths:continue
    if not row['path'].startswith('optimization_v2/') or not row['local'].startswith('baseline/'):continue
    data=(ROOT/row['local']).read_bytes()
    assert hashlib.sha256(data).hexdigest()==row['sha256']
    try:code=data.decode('utf-8')
    except UnicodeDecodeError:continue
    view=views/(row['sha256'][:12]+'-'+Path(row['path']).name+'.html')
    expected_views.add(view)
    row['view']=str(view.relative_to(ROOT))
    numbered='\n'.join(f'<span class="line" id="L{i}"><a href="#L{i}">{i:4}</a> {html.escape(line)}</span>' for i,line in enumerate(code.splitlines(),1))
    view.write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(Path(row['path']).name)+'</title><style>body{margin:25px;background:#f4f7f8;color:#18343e;font:16px/1.7 system-ui}pre{background:white;padding:20px;border:1px solid #d6e2e3;font:13px/1.8 Consolas,monospace;overflow:auto}.line{white-space:pre-wrap;overflow-wrap:anywhere}.line:target{background:#fff0b0}a{color:#086b71}p{overflow-wrap:anywhere}</style><a href="../../L02/index.html">← L02 教材目录</a><h1>'+html.escape(Path(row['path']).name)+'</h1><p>'+html.escape(row['path'])+'</p><p>固定基线 '+REV+' · <a href="'+html.escape(row['url'])+'">GitHub 原文件</a></p><p>原始文件按字节保留；显示换行不改变行号。快照用于阅读，不直接作为运行入口。</p><pre><code>'+numbered+'</code></pre></html>')
for obsolete in views.glob('*.html'):
    if obsolete not in expected_views:obsolete.unlink()

def excerpt(alias,selector):
    row=entries[ALIASES[alias]];text=(ROOT/row['local']).read_text();lines=text.splitlines()
    if selector.startswith('fn:'):
        name=selector[3:]
        if row['path'].endswith('.py'):
            node=next(n for n in ast.walk(ast.parse(text)) if isinstance(n,(ast.FunctionDef,ast.ClassDef,ast.AsyncFunctionDef)) and n.name==name)
            start,end=node.lineno,node.end_lineno
        else:
            start=next(i for i,l in enumerate(lines,1) if re.search(r'\b'+re.escape(name)+r'\s*\(',l))
            end=next(i for i in range(start,len(lines)+1) if i>start and lines[i-1]=='}')
    else:start,end=map(int,selector.split(':'))
    end=min(end,len(lines),start+69)
    code='\n'.join(lines[start-1:end])
    return row,start,end,code

catalog=[];excerpt_records=[]
for chapter in sorted(CH.glob('[0-9][0-9]_*.md')):
    number=int(chapter.name[:2]);text=chapter.read_text()
    # Rebuild only explicitly generated blocks; preserve authored paragraphs.
    text=re.sub(r'<!-- generated-production-start -->.*?<!-- generated-production-end -->','<!-- production -->',text,flags=re.S)
    text=re.sub(r'<!-- generated-lab-start -->.*?<!-- generated-lab-end -->','<!-- lab -->',text,flags=re.S)
    parts=['<!-- generated-production-start -->','## 固定生产代码逐段阅读','',
           '下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。','']
    source_rows=[]
    for alias,selector,explanation in SOURCES[number]:
        row,start,end,code=excerpt(alias,selector)
        lang={'.py':'python','.sh':'bash','.cpp':'cpp','.cc':'cpp','.hpp':'cpp','.yaml':'yaml','.xml':'xml','.action':'text'}.get(Path(row['path']).suffix,'text')
        local=posixpath.relpath(row['view'],'L02/chapters')
        parts.extend([f'### {Path(row["path"]).name} 第 {start} 至 {end} 行','',
                      f'[离线带行号源码]({local}#L{start}) · [固定 Git 提交]({row["url"]}#L{start})','',
                      f'```{lang}\n{code}\n```','',explanation,''])
        record=dict(chapter=number,path=row['path'],local=row['local'],start=start,end=end,
                    sha256=hashlib.sha256(code.encode()).hexdigest(),selector=selector)
        excerpt_records.append(record);source_rows.append(record)
    parts.append('<!-- generated-production-end -->')
    assert '<!-- production -->' in text,chapter
    text=text.replace('<!-- production -->','\n'.join(parts))
    node=labnodes[f'lab{number:02}'];code='\n'.join(lablines[node.lineno-1:node.end_lineno])
    result=getattr(labs,f'lab{number:02}')()
    output=dict(chapter=f'{number:02}',passed=True,result=result)
    output_file=OUT/f'lab_{number:02}.json';output_file.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    display=json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)
    if len(display)>5000:display=display[:4200]+'\n[显示节选；完整结构请打开本章输出文件]'
    labpart='\n'.join(['<!-- generated-lab-start -->','### 本章练习核心代码与实际输出','',
        f'[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_{number:02}.json)','',
        '代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。','',
        f'```python\n{code}\n```','',f'```text\n{display}\n```','',
        '本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。',
        '<!-- generated-lab-end -->'])
    assert '<!-- lab -->' in text,chapter
    text=text.replace('<!-- lab -->',labpart);chapter.write_text(text)
    authored=re.sub(r'<!-- generated-.*?-start -->.*?<!-- generated-.*?-end -->','',text,flags=re.S).split('<!-- generated fixed-revision references -->')[0]
    catalog.append(dict(number=number,title=text.splitlines()[0].lstrip('# '),path=str(chapter.relative_to(ROOT)),
                        characters=len(text),authored_characters=len(authored),sources=source_rows,lab=f'examples/l02/labs.py {number:02}',
                        learning_check='本章练习、常见故障与自测答案',html='L02/read/'+chapter.stem+'.html'))
assert [c['number'] for c in catalog]==list(range(1,31))
(BOOK/'catalog.json').write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n')
(OUT/'excerpts.json').write_text(json.dumps(excerpt_records,ensure_ascii=False,indent=2)+'\n')

index=['# L02 完整三十章教材','',
'固定机器人基线 `9aff8337`。从 Linux 与编程基础，到传感器、定位、规划、BT、工具账本和人工操作。原机器人 R10–R12 仍为 PARTIAL；本教材的完成状态独立记录。','',
'Windows 下载完整仓库后打开根目录 index.html，再进入本目录。每章都有固定源码节选、逐段解释、可运行离线练习、输出与故障排查。真实仿真命令集中在第 08、26、27 章并明确前置。','',
'[先看学习路线](learning_routes.md) · [术语与命令索引](reference.md) · [检查结果](RESULTS.md) · [L01 设计依据](../L01/05_curriculum.md)','']
for low,high,title in PARTS:
    index.extend([f'## {title}','','|章节|本章代码与练习|','|---|---|'])
    for c in catalog:
        if low<=c['number']<=high:index.append(f'|[{c["title"]}](chapters/{Path(c["path"]).name})|{len(c["sources"])} 段生产源码；练习 {c["number"]:02}|')
    index.append('')
(BOOK/'README.md').write_text('\n'.join(index)+'\n')

css='''*{box-sizing:border-box}body{margin:0;background:#eff4f5;color:#17343d;font:17px/1.95 "Segoe UI","Microsoft YaHei","Noto Sans CJK SC",sans-serif}header{background:#163c45;color:#e5f3ef;padding:22px max(20px,calc((100vw - 1080px)/2))}header a{color:#bceadf;margin-right:20px}main{max-width:1140px;margin:28px auto;padding:34px 48px;background:white;border:1px solid #dae5e8;border-radius:12px}h1{font-size:32px;line-height:1.5;color:#174b50}h2{margin-top:42px;font-size:24px;color:#256466;border-bottom:1px solid #dce9e8;padding-bottom:9px}h3{font-size:20px;color:#345b62}a{color:#08777a;text-underline-offset:3px}p,li{overflow-wrap:anywhere}pre{background:#112c38;color:#e3f0ed;padding:20px;border-radius:8px;overflow:auto;font:13px/1.8 Consolas,monospace;white-space:pre;overflow-wrap:normal}code{font-family:Consolas,monospace}p code,li code,td code{background:#edf3f4;padding:2px 4px}table{display:block;overflow:auto;border-collapse:collapse;font-size:15px;margin:22px 0}th,td{border:1px solid #cedfe1;padding:10px;min-width:100px;vertical-align:top}th{background:#e6f2ef}img{max-width:100%;height:auto;margin:22px auto;display:block}blockquote{border-left:4px solid #3c9083;padding:8px 20px;background:#f0f7f4;margin:20px 0}.toc{background:#f3f8f7;padding:12px 25px;font-size:14px}footer{max-width:1100px;margin:24px auto;padding:20px;color:#47666c}button{background:#e7f4ed;color:#245448;border:0;padding:7px 12px;border-radius:5px;cursor:pointer}@media(max-width:700px){body{font-size:16px}main{padding:22px 18px;margin:12px;border-radius:8px}h1{font-size:26px}h2{font-size:22px}header{padding:16px}pre{font-size:12px}}@media print{header,footer,button,.toc{display:none}main{border:0;max-width:none;margin:0;padding:0}pre{background:#eee;color:black;white-space:pre-wrap;overflow-wrap:anywhere}a{color:black}h2{break-after:avoid}img{max-height:230mm}}'''
docs=[BOOK/'README.md',*sorted(CH.glob('*.md')),*[p for p in [BOOK/'learning_routes.md',BOOK/'reference.md',BOOK/'RESULTS.md'] if p.exists()]]
mapping={str(p.relative_to(ROOT)):('L02/index.html' if p.name=='README.md' else 'L02/read/'+p.stem+'.html') for p in docs}
for p in docs:
    src=str(p.relative_to(ROOT));dest=ROOT/mapping[src];dest.parent.mkdir(parents=True,exist_ok=True)
    md=markdown.Markdown(extensions=['tables','fenced_code','toc']);body=md.convert(p.read_text())
    def adjust(m):
        attr,url=m[1],html.unescape(m[2]);u=urlsplit(url)
        if u.scheme or url.startswith('//') or not u.path:return m[0]
        target=posixpath.normpath(posixpath.join(posixpath.dirname(src),unquote(u.path)))
        target=mapping.get(target,target)
        relative=posixpath.relpath(target,posixpath.dirname(mapping[src]))
        return f'{attr}="'+html.escape(relative+('#'+u.fragment if u.fragment else ''),quote=True)+'"'
    body=re.sub(r'(href|src)="([^"]+)"',adjust,body)
    title=p.read_text().splitlines()[0].lstrip('# ')
    number=int(p.name[:2]) if p.parent==CH else None
    top='<a href="'+posixpath.relpath('index.html',posixpath.dirname(mapping[src]))+'">学习首页</a><a href="'+posixpath.relpath('L02/index.html',posixpath.dirname(mapping[src]))+'">三十章目录</a>'
    if number:
        if number>1:top+=f'<a href="{Path(catalog[number-2]["html"]).name}">← 上一章</a>'
        if number<30:top+=f'<a href="{Path(catalog[number]["html"]).name}">下一章 →</a>'
    page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+css+'</style><header>'+top+'<button onclick="window.print()">打印本章</button></header><main>'+('<details class="toc"><summary>本章目录</summary>'+md.toc+'</details>' if number else '')+body+'</main><footer>L02 内容阅读版 · 固定基线 9aff8337 · 原机器人 R10–R12 PARTIAL · L03 完整站点尚待执行</footer></html>'
    dest.write_text(page)
summary=dict(chapters=len(catalog),characters=sum(c['characters'] for c in catalog),authored_characters=sum(c['authored_characters'] for c in catalog),excerpts=len(excerpt_records),reading_pages=len(docs),source_views=len(list(views.glob('*.html'))),baseline=REV)
(OUT/'build.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False))
