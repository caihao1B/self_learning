"""Build the L01 reading preview and verified source index; no robot execution."""
from pathlib import Path
import hashlib, html, json, posixpath, re, subprocess
from urllib.parse import urlsplit, unquote
import markdown

ROOT=Path(__file__).resolve().parents[1]
manifest=json.loads((ROOT/'baseline/manifest.json').read_text())
by_path={e['path']:e for e in manifest['entries']}
P='optimization_v2/ws/src/task_platform/'
aliases={
 'env':'optimization_v2/env.sh','entry':'optimization_v2/three_tasks/advance.sh',
 'dispatch':P+'scripts/dispatch.py','common':P+'scripts/common.py',
 'stage12':P+'scripts/stage_r12.py','derive':P+'scripts/prepare_spray_video_r12.py',
 'runtime':P+'scripts/runtime_spray_r10.py','mission':P+'scripts/mission_spray_r10.py',
 'control':P+'scripts/control_spray_r10.py','docking':P+'scripts/docking_r10.py',
 'precision':P+'r10_backend/precision_progress.hpp',
 'precision-builder':P+'scripts/precision_supervisor_r10.py','bt':P+'r08_backend/policy.hpp',
 'tool-server':'optimization_v2/ws/src/tool_sim/src/action_server.cpp',
 'tool-build':P+'scripts/tool_build_r09.py',
 'tool-action':'optimization_v2/ws/src/mission_interfaces/action/ToolTask.action',
 'cmake':P+'CMakeLists.txt',
 'gui-page':'optimization_v2/three_tasks/monitor/index.html',
 'gui-data':'optimization_v2/three_tasks/monitor/data.py',
 'gui-observer':'optimization_v2/three_tasks/monitor/ros_observer.py',
 'point-result':'validation/R12/spray/mission_result.json',
 'point-events':'validation/R12/spray/mission_events.jsonl',
 'fault':'validation/R12/spray/fault_injection.json',
 'recording':'validation/R12/spray/recording_acceptance.json',
 'full-a1':'validation/R10/bounded_A1_result.json','full-b1':'validation/R10/bounded_B1_result.json',
 'attempts':'validation/R10/repair_attempts.json',
 'historical-plan':'docs/optimization_v2/命令Prompt合集.md',
 'r-plan':'docs/three_tasks/实施步骤.md',
 'localization-doc':'docs/three_tasks/R05/TECHNICAL.md',
 'inspection-doc':'docs/three_tasks/R08/TECHNICAL.md',
 'mowing-doc':'docs/three_tasks/R09/TECHNICAL.md',
 'repair-doc':'docs/three_tasks/R10/GUI_AND_BOUNDED_REPAIR.md',
 'prediction':'docs/optimization_v2/S15/RESULTS.md',
 'cache':'docs/optimization_v2/S16/RESULTS.md',
 'resources-doc':'docs/three_tasks/R12/LOCAL_REPRODUCTION.md',
 'resources':'validation/GUI/local_resource_inventory.json',
}
defs='\n\n<!-- generated fixed-revision references -->\n'+''.join(f'[{key}]: {by_path[path]["url"]}\n' for key,path in aliases.items())

lines=['# 源码与证据导航','',
 '这里的外部链接固定到机器人提交 `'+manifest['commit']+'`；本地源码快照保持字节一致。它们用于阅读和小型教学调用，不是完整机器人运行包。Python 函数行号由 AST 提取，源码文件与 Git blob 已核对。','',
 '## 核心源码','', '|职责与文件|先读的位置|离线快照|', '|---|---|---|']
symbols=[]
views=ROOT/'baseline/source_views';views.mkdir(exist_ok=True)
for e in manifest['entries']:
 if e['kind']!='source':continue
 sy=e.get('symbols',[])
 priorities=['main','record','run','check','progressDelta','stable_dock','derive','build','service','wait','recover_localization','settle_and_redock']
 chosen=sorted(sy,key=lambda s:(priorities.index(s['name']) if s['name'] in priorities else 999,s['line']))[:4]
 focus='；'.join(f'[{s["name"]}:{s["line"]}]({e["url"]}#L{s["line"]})' for s in chosen) or f'[从第 1 行开始]({e["url"]}#L1)'
 view=views/(e['sha256'][:12]+'-'+Path(e['path']).name+'.html')
 code=(ROOT/e['local']).read_text()
 numbered=''.join(f'<span class="line" id="L{i}"><a href="#L{i}">{i:4}</a> {html.escape(line)}</span>\n' for i,line in enumerate(code.splitlines(),1))
 view.write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(e['label'])+'</title><style>body{margin:30px;background:#f4f7f8;color:#18343e;font:16px/1.7 system-ui}pre{background:white;border:1px solid #d5e1e5;padding:20px;overflow:auto;font:13px/1.7 Consolas,monospace}.line{white-space:pre-wrap;overflow-wrap:anywhere}.line:target{background:#fff2b5}.line a{color:#78949b;text-decoration:none}a{color:#076d76}p{overflow-wrap:anywhere}</style><a href="../../index.html#04-code-evidence">← 返回源码导航</a><h1>'+html.escape(e['label'])+'</h1><p>'+html.escape(e['path'])+'</p><p>固定提交 '+manifest['commit']+' · SHA-256 '+e['sha256']+'</p><p><a href="'+html.escape(e['url'])+'">GitHub 固定版本</a> · 生产源码快照；视觉换行不改变原始行号。使用编辑器修改前先了解任务版本绑定。</p><pre><code>'+numbered+'</code></pre></html>')
 lines.append(f'|{e["label"]}<br>`{Path(e["path"]).name}`|{focus}|[带行号阅读](../{view.relative_to(ROOT)})|')
 symbols.append(dict(path=e['path'],url=e['url'],sha256=e['sha256'],symbols=sy))
lines+=['','## 精选证据','', '|证据|固定版本|离线副本|','|---|---|---|']
for e in manifest['entries']:
 if e['kind'] in ['evidence','document']:
  lines.append(f'|{e["label"]}|[原文件]({e["url"]})|[副本](../{e["local"]})|')
lines+=['','## 三条建议的阅读链','',
 '点喷：启动入口 → dispatch → stage_r12.record → 录像派生器 → runtime.run → mission.main → service 与工具服务器 → ledger_core → recording_acceptance。','',
 '定位失败：mission.recover_localization → reference 与 global manager 的状态 → stopped_revalidation.inc → 当前地图/计划证明 → 事件与最后失败结果。完整 C++ 管理器包含生成过程，不能只看片段推断全部行为。','',
 '割草：离线条带与安全包络 → OMPL 连接 → mission_mowing → recover_endpoint 与分挡停车 → 工具工作反馈和面积账本。','',
 '看到行号时先核对版本。看到文件名带 R07 或节点名带旧阶段号时，检查谁正在调用它；复用的历史名字不等于正在运行旧任务。',
]
(ROOT/'L01/04_code_evidence.md').write_text('\n'.join(lines)+'\n')
(ROOT/'baseline/symbol_index.json').write_text(json.dumps(symbols,ensure_ascii=False,indent=2)+'\n')

for p in sorted((ROOT/'L01').glob('*.md')):
 text=p.read_text().split('<!-- generated fixed-revision references -->')[0].rstrip()
 p.write_text(text+defs)

BASE='''graph [bgcolor="#f7fafb",pad="0.3",nodesep="0.35",ranksep="0.55",fontname="Noto Sans CJK SC",fontsize=16];
node [shape=box,style="rounded,filled",fillcolor="#e8f3f1",color="#39776e",fontcolor="#163d38",fontname="Noto Sans CJK SC",fontsize=13,margin="0.18,0.13"];
edge [color="#648189",fontname="Noto Sans CJK SC",fontsize=11,arrowsize=0.7];'''
graphs={
'architecture':'''digraph G {rankdir=TB;BASE
 sim[label="Gazebo 场景与车辆\\n物理 · 传感器 · 工具反馈"];
 local[label="定位\\nRTAB ICP + 轮速/IMU EKF\\n区域地图匹配 + GNSS 监督"];
 risk[label="感知与通行\\n点云地形 · 障碍跟踪\\n实时二维代价"];
 plan[label="离线计划\\nMETIS · OR-Tools\\nFields2Cover · OMPL"];
 bt[label="C++ 可解释 BT\\n条件 · 原因 · 有界恢复"];
 task[label="Python 任务执行器\\n目标 · 路径 · 工具动作"];
 nav[label="Nav2\\n规划与路径跟踪"];
 gate[label="唯一底盘门控\\n时钟 · 定位 · 感知 · 运动约束"];
 tool[label="工具动作服务器\\n准入 · 取消 · 停止确认"];
 ledger[label="有效作业账本\\n工作键 · 确认量 · 残余"];
 gui[label="只读 GUI\\n状态 · 数据 · 决策 · 证据",fillcolor="#fff1dc",color="#ac824a"];
 sim->local;sim->risk;local->nav;risk->nav;local->bt;risk->bt;plan->task;bt->task;task->nav;task->tool;nav->gate;bt->gate;gate->sim;tool->sim;sim->ledger;ledger->task;
 task->gui[style=dashed];ledger->gui[style=dashed];local->gui[style=dashed];risk->gui[style=dashed];
 {rank=same;local;risk;plan} {rank=same;gate;tool;ledger}
}''',
'data_flow':'''digraph G {rankdir=LR;BASE
 raw[label="实际传感器观测\\n采集时间 · 外参 · 噪声"];
 pre[label="有效性与预处理\\n自车过滤 · 运动补偿"];
 loc[label="定位支路\\n结构点 / ICP / EKF\\n参考地图匹配"];
 obs[label="避障支路\\n地面 / 坡度 / 障碍\\n未知保持未知"];
 ctx[label="在线任务与控制\\nROS domain 119\\n观测和授权先验"];
 truth[label="仿真真值\\n独立评估域 120",fillcolor="#f7e9e6",color="#a0645b"];
 audit[label="运行后的分析\\n误差 · 接触 · 物理反馈\\n不能回流在线控制",fillcolor="#f7e9e6",color="#a0645b"];
 raw->pre;pre->loc;pre->obs;loc->ctx;obs->ctx;truth->audit;ctx->audit[label="保存的结果"];
}''',
'spray_sequence':'''digraph G {rankdir=TB;BASE
 a[label="启动与准入\\n服务器 / 定位 / 工具已停"];
 b[label="NavigateToPose\\n持续处理 BT 与定位消息"];
 c[label="实测停车与静止观测\\n位置 · 散布 · 时间窗口"];
 d[label="POINT_SERVICE\\n版本 + 工作键 + 请求数量"];
 e[label="有效反馈进入账本\\n确认量与剩余量"];
 f[label="SUCCEEDED\\n计入一个完成点"];
 g[label="下一个点或最终停止"];
 jam[label="实际 JAM\\n45.450 至 48.450 s",fillcolor="#fff1dc",color="#ac824a"];
 fail[label="工具失败 / 停止确认\\n保留残余与不确定状态",fillcolor="#fff1dc",color="#ac824a"];
 retry[label="满足恢复条件\\n同一工作键补残余",fillcolor="#fff1dc",color="#ac824a"];
 replay[label="重放已完成工作键\\n返回原确认 / 不再次作业"];
 a->b;b->c;c->d;d->e;e->f;f->g;jam->fail;d->fail[label="故障"];fail->retry;retry->c;f->replay;replay->g;
}''',
'code_flow':'''digraph G {rankdir=TB;BASE
 bash[label="advance.sh → env.sh\\nBash：环境与参数"];
 dispatch[label="dispatch.py\\nargparse → stage_r12"];
 stage[label="record('spray')\\n案例范围 / 版本核查"];
 derive[label="prepare_spray_video_r12.py\\n确定性派生录像运行器"];
 runtime[label="runtime.run\\n锁 / 环境 / Popen / 收尾"];
 mission[label="mission_spray_r10.main\\nROS 回调与动作 Future"];
 cpp[label="原生 C++ 组件\\nBT / 工具 / 仿真插件"];
 lib[label="上游库与节点\\nNav2 / RTAB / EKF"];
 result[label="JSONL · SQLite · JSON\\n运行分析与录像回执"];
 bash->dispatch;dispatch->stage;stage->derive;derive->runtime;runtime->mission;runtime->cpp;runtime->lib;mission->cpp[label="动作与状态"];mission->lib[label="导航请求"];mission->result;cpp->result;lib->result;
}'''
}
for name,source in graphs.items():
 dot=ROOT/'assets'/(name+'.dot');dot.write_text(source.replace('BASE',BASE))
 subprocess.run(['dot','-Tsvg',str(dot),'-o',str(dot.with_suffix('.svg'))],check=True)

docs=[ROOT/'README.md']+sorted((ROOT/'L01').glob('*.md'))
slugs={str(p.relative_to(ROOT)):('intro' if p.name=='README.md' else p.stem.lower().replace('_','-')) for p in docs}
titles={str(p.relative_to(ROOT)):p.read_text().splitlines()[0].lstrip('# ') for p in docs}
nav=''.join(f'<a href="#{slugs[path]}">{html.escape(title)}</a>' for path,title in titles.items())
articles=[]
for p in docs:
 rel=str(p.relative_to(ROOT));slug=slugs[rel]
 body=markdown.markdown(p.read_text(),extensions=['tables','fenced_code','toc'])
 body=re.sub(r' id="([^"]+)"',lambda m:' id="'+slug+'--'+m[1]+'"',body)
 def adjust(m):
  attr,url=m[1],html.unescape(m[2]);u=urlsplit(url)
  if u.scheme or url.startswith('//'):return m[0]
  if not u.path:return f'{attr}="#{slug}--{html.escape(u.fragment,quote=True)}"' if u.fragment else m[0]
  target=posixpath.normpath(posixpath.join(posixpath.dirname(rel),unquote(u.path)))
  if target in slugs:
   dest='#'+slugs[target]+('--'+u.fragment if u.fragment else '')
  else:dest=target+('#'+u.fragment if u.fragment else '')
  return attr+'="'+html.escape(dest,quote=True)+'"'
 body=re.sub(r'(href|src)="([^"]+)"',adjust,body)
 articles.append(f'<article id="{slug}">{body}</article>')
css='''*{box-sizing:border-box}html{scroll-behavior:auto}body{margin:0;background:#eef3f5;color:#19313b;font:16px/1.9 "Segoe UI","Microsoft YaHei","Noto Sans CJK SC",sans-serif}aside{position:fixed;inset:0 auto 0 0;width:258px;padding:28px 20px;overflow:auto;background:#102c35;color:#edf8f5}aside b{display:block;font-size:21px;margin:12px 0 20px}aside small{color:#a6cecb}nav a{display:block;color:#c9e0e3;text-decoration:none;padding:8px;border-radius:6px;font-size:14px;line-height:1.6}nav a:hover{background:#21444e;color:white}main{margin-left:258px;padding:36px 48px;max-width:1510px}header{background:#173e49;color:#f5faf7;padding:32px 38px;border-radius:14px}header p{max-width:900px;color:#c4e2e2}header a{color:#b5f3df}.badge{display:inline-block;background:#27594e;color:#c6f3dc;padding:3px 12px;border-radius:20px;font-size:13px}article{background:white;margin:28px 0;padding:34px 40px;border:1px solid #d8e3e6;border-radius:12px;scroll-margin-top:20px}h1{font-size:30px;line-height:1.45;color:#143d49}h2{font-size:23px;margin-top:38px;color:#245d60;border-bottom:1px solid #dfebed;padding-bottom:8px}h3{font-size:19px;color:#31575c}a{color:#096f75;text-underline-offset:3px}p{margin:15px 0}pre{background:#102a36;color:#e2f2ee;padding:20px;border-radius:9px;overflow:auto;line-height:1.7;font-size:13px}code{font-family:Consolas,"DejaVu Sans Mono",monospace;font-size:.9em;overflow-wrap:anywhere}p code,li code,td code{background:#edf4f5;padding:2px 4px;border-radius:3px}table{display:block;overflow:auto;width:100%;border-collapse:collapse;font-size:14px;margin:22px 0}th{background:#e6f1f0;color:#194947;text-align:left}td,th{border:1px solid #d6e3e5;padding:10px 12px;vertical-align:top;min-width:95px}td{line-height:1.7}img{max-width:100%;height:auto;display:block;margin:22px auto;border:1px solid #dce6e8;border-radius:8px;background:#f7fafb}blockquote{border-left:4px solid #368a7c;background:#eff7f3;padding:5px 20px;margin:18px 0}details{margin-top:20px}details nav a{color:#d9eeec}button{background:#edf8f3;border:0;border-radius:6px;color:#204b46;padding:8px 12px;cursor:pointer}footer{padding:25px;color:#5e737b}.mobile-nav{display:none}@media(max-width:900px){aside{display:none}main{margin:0;padding:14px}header{padding:22px}article{padding:22px 18px}h1{font-size:25px}h2{font-size:21px}.mobile-nav{display:block}table{font-size:13px}}@media print{header h1{color:#19313b!important}aside,.mobile-nav,button{display:none}main{margin:0;padding:0;max-width:none}header{background:white;color:#19313b}header p{color:#19313b}article{border:0;padding:10px;break-before:page}pre{white-space:pre-wrap;background:#eee;color:black}table{display:table;font-size:10px}img{max-height:230mm}a{color:#173f47}}'''
page='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>农田机器人项目学习教材 L01</title><style>'+css+'</style><aside><small>FIELD ROBOT · LEARNING</small><b>项目学习教材</b><span class="badge">L01 资料与样章</span><nav>'+nav+'</nav></aside><main><header><span class="badge">固定基线 9aff8337</span><h1 style="color:white">从一个点喷任务理解整个机器人系统</h1><p>需求、架构、技术选择、真实代码与失败证据。适合 Windows 离线阅读，Linux 实验单独标明。</p><a href="#07-sample-spray">直接阅读详细样章 →</a>　<button onclick="window.print()">打印 / 保存 PDF</button><details class="mobile-nav"><summary>展开章节目录</summary><nav>'+nav+'</nav></details></header>'+''.join(articles)+'<footer>L01 本地阅读预览 · 完整教材 L02 与完整网站 L03 尚待执行 · 机器人 R10–R12 保持 PARTIAL</footer></main></html>'
(ROOT/'index.html').write_text(page)
print(json.dumps(dict(documents=len(docs),diagram_count=len(graphs),source_entries=len(symbols),html_bytes=len(page.encode())),ensure_ascii=False))
