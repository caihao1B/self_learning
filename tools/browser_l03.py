"""Exercise the real browser UI over HTTP, file URLs and Unicode/space paths."""
from pathlib import Path
import hashlib,base64,functools,http.server,json,subprocess,tempfile,time,threading,urllib.request,shutil,os
import websocket
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'validation/L03';OUT.mkdir(exist_ok=True)
records=[];errors=[];seq=0;runtime_exceptions=[]
class Quiet(http.server.SimpleHTTPRequestHandler):
 def log_message(self,*args):pass
with tempfile.TemporaryDirectory(prefix='l03-browser-') as temp:
 temp=Path(temp);preview=temp/'Windows 学习 folder';preview.mkdir()
 for name in ['index.html','web','assets','L02','L03','baseline','evidence','examples','validation']:
  p=ROOT/name
  if p.is_dir():shutil.copytree(p,preview/name,ignore=shutil.ignore_patterns('chrome.log','__pycache__','*.pyc'))
  else:shutil.copy2(p,preview/name)
 server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(temp)))
 thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
 with (OUT/'chrome.log').open('w') as log:
  proc=subprocess.Popen(['/usr/bin/google-chrome','--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check','--remote-debugging-port=0','--remote-allow-origins=http://localhost','--user-data-dir='+str(temp/'profile'),'about:blank'],stdout=log,stderr=log)
  try:
   marker=temp/'profile/DevToolsActivePort';deadline=time.monotonic()+20
   while not marker.exists():
    if time.monotonic()>deadline or proc.poll() is not None:raise RuntimeError('CHROME_START')
    time.sleep(.1)
   port=marker.read_text().splitlines()[0];tab=next(x for x in json.load(urllib.request.urlopen(f'http://127.0.0.1:{port}/json')) if x['type']=='page')
   wire=websocket.create_connection(tab['webSocketDebuggerUrl'],origin='http://localhost',timeout=20)
   def call(method,params=None):
    global seq
    seq+=1;wire.send(json.dumps(dict(id=seq,method=method,params=params or {})))
    while True:
     d=json.loads(wire.recv())
     if d.get('method')=='Runtime.exceptionThrown':runtime_exceptions.append(d)
     if d.get('id')==seq:
      if 'error' in d:raise RuntimeError(d['error'])
      return d.get('result',{})
   def js(expression):
    d=call('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=True))
    if d.get('exceptionDetails'):raise RuntimeError(d['exceptionDetails'])
    return d.get('result',{}).get('value')
   def wait(expression,timeout=10):
    deadline=time.monotonic()+timeout
    while not js(expression):
     if time.monotonic()>deadline:raise RuntimeError('WAIT '+expression)
     time.sleep(.05)
   def navigate(url):
    call('Page.navigate',dict(url=url));wait('document.readyState==="complete" && !!window.RobotLearning && !!document.querySelector("main h1")')
   def route(hash,expected=None):
    js('location.hash='+json.dumps(hash));wait('location.hash.startsWith('+json.dumps(hash.split('?')[0])+')');time.sleep(.08)
    if expected:wait(expected)
   def check(name,expression):
    value=js(expression);ok=bool(value);records.append(dict(name=name,passed=ok,value=value))
    if not ok:errors.append(name)
   def shot(name):
    r=call('Page.captureScreenshot',dict(format='png',captureBeyondViewport=False));(OUT/(name+'.png')).write_bytes(base64.b64decode(r['data']))
   def input_value(selector,value):
    js(f'(()=>{{const e=document.querySelector({json.dumps(selector)});e.value={json.dumps(value)};e.dispatchEvent(new Event("input",{{bubbles:true}}));}})()')
   call('Page.enable');call('Runtime.enable');call('Network.enable');call('Emulation.setDeviceMetricsOverride',dict(width=1440,height=1050,deviceScaleFactor=1,mobile=False))
   url=f'http://127.0.0.1:{server.server_port}/Windows%20%E5%AD%A6%E4%B9%A0%20folder/index.html'
   navigate(url+'#/home');check('http_home_loaded','document.querySelector("main h1").textContent.includes("理解一台机器人")');shot('home_desktop')
   check('no_body_overflow_home','document.documentElement.scrollWidth<=innerWidth+1')
   route('#/contents');check('thirty_chapter_cards','document.querySelectorAll(".chapter-card").length===30');input_value('#chapter-filter','定位');check('contents_filter','document.querySelectorAll(".chapter-card").length>0&&document.querySelectorAll(".chapter-card").length<30')
   route('#/home');call('Input.dispatchKeyEvent',dict(type='keyDown',key='k',code='KeyK',modifiers=2));wait('document.querySelector("#search-dialog").open');check('keyboard_ctrl_k','document.querySelector("#search-dialog").open');call('Input.dispatchKeyEvent',dict(type='keyDown',key='Escape',code='Escape'));wait('!document.querySelector("#search-dialog").open');check('keyboard_escape','!document.querySelector("#search-dialog").open');js('document.querySelector("#search-launch").click()');wait('document.activeElement.id==="quick-query"');check('search_dialog_focus','document.querySelector("#search-dialog").open && document.activeElement.id==="quick-query"')
   input_value('#quick-query','GNSS NIS');js('document.querySelector("#quick-search-form").requestSubmit()');wait('!!document.querySelector("#search-results .result")');check('multiterm_chapter_result','Array.from(document.querySelectorAll(".result a")).some(a=>a.href.includes("/chapter/14"))');shot('search_desktop')
   input_value('#full-query','zzzz_unfindable_01234');time.sleep(.2);check('empty_search_state','!!document.querySelector(".empty-state")')
   input_value('#full-query','<img src=x onerror=alert(1)>');time.sleep(.2);check('query_html_safe','!document.querySelector("#search-results img")')
   for n in range(1,31):
    route('#/chapter/'+str(n),'!!document.querySelector("article")')
    check(f'chapter_{n:02}_loaded',f'document.querySelector("article h1").textContent.includes("{n:02}")')
    check(f'chapter_{n:02}_code','document.querySelectorAll(".code-wrap").length>=2')
    check(f'chapter_{n:02}_no_overflow','document.documentElement.scrollWidth<=innerWidth+1')
    check(f'chapter_{n:02}_images','Array.from(document.querySelectorAll("article img")).every(i=>i.complete&&i.naturalWidth>0)')
   route('#/chapter/14');shot('chapter_desktop')
   js('document.querySelector("[data-wrap]").click()');check('code_wrap','document.querySelector(".code-wrap").classList.contains("wrapped")')
   js('document.querySelector("[data-collapse]").click()');check('code_collapse','getComputedStyle(document.querySelector("article pre")).display==="none"');js('document.querySelector("[data-collapse]").click()')
   js('window.__copied=null; navigator.clipboard.writeText=async text=>{window.__copied=text;}; document.querySelector("[data-copy]").click()');check('copy_exact_block','window.__copied===document.querySelector("article pre code").textContent')
   js('document.querySelector("#font-up").click()');check('font_preference','RobotLearning.getState().font===18')
   js('document.querySelector("#mark-read").click()');input_value('#chapter-note','学习测试：NIS 与杆臂 <script>不可执行</script>');check('notes_and_completion','RobotLearning.getState().completed.includes(14)&&RobotLearning.getState().notes[14].includes("杆臂")')
   js('window.__reload_probe=true');call('Page.reload');wait('!window.__reload_probe && !!document.querySelector("#chapter-note")');check('notes_persist_reload','document.querySelector("#chapter-note").value.includes("杆臂")')
   route('#/notebook');check('notebook_text_safe','!document.querySelector(".notebook-row script")&&document.querySelector(".notebook-row p").textContent.includes("<script>")')
   download=temp/'downloads';download.mkdir();call('Browser.setDownloadBehavior',dict(behavior='allow',downloadPath=str(download)))
   js('document.querySelector("#export-notes").click()');deadline=time.monotonic()+5
   while not list(download.glob('*.json')):
    if time.monotonic()>deadline:raise RuntimeError('EXPORT_DOWNLOAD');
    time.sleep(.1)
   exported=json.loads(next(download.glob('*.json')).read_text());records.append(dict(name='export_actual_download',passed=exported['notes']['14'].startswith('学习测试')))
   payload=dict(exported,completed=[14,20],notes={'20':'跨系统导入练习'},last=20);imp=temp/'notes.json';imp.write_text(json.dumps(payload,ensure_ascii=False))
   doc=call('DOM.getDocument');node=call('DOM.querySelector',dict(nodeId=doc['root']['nodeId'],selector='#notes-file'))['nodeId'];call('DOM.setFileInputFiles',dict(nodeId=node,files=[str(imp)]));wait('document.querySelector("#import-status").textContent.includes("导入成功")');check('import_ui_merge','RobotLearning.getState().completed.includes(14)&&RobotLearning.getState().completed.includes(20)&&RobotLearning.getState().notes[14].includes("杆臂")')
   bad=temp/'invalid.json';bad.write_text(json.dumps({**payload,'baseline':'wrong'}));doc=call('DOM.getDocument');node=call('DOM.querySelector',dict(nodeId=doc['root']['nodeId'],selector='#notes-file'))['nodeId'];call('DOM.setFileInputFiles',dict(nodeId=node,files=[str(bad)]));wait('document.querySelector("#import-status").textContent.includes("导入失败")');check('invalid_import_keeps_data','RobotLearning.getState().notes[20]==="跨系统导入练习"')
   route('#/sources');check('source_count','document.querySelectorAll(".source-row").length===ROBOT_BOOK.sources.length');input_value('#source-filter','precision_progress');check('source_filename_filter','document.querySelectorAll(".source-row").length===1');sid=js('ROBOT_BOOK.sources.find(s=>s.name==="precision_progress.hpp").id')
   route('#/source/'+sid+'?line=9');time.sleep(.1);check('source_deep_line','document.querySelector(".source-line.selected").dataset.line==="9"');check('source_exact_text','Array.from(document.querySelectorAll(".line-text")).map(e=>e.textContent===" "?"":e.textContent).join(String.fromCharCode(10))===ROBOT_BOOK.sources.find(s=>s.id==='+json.dumps(sid)+').code.slice(0,-1)')
   input_value('#code-query','return');check('source_find','document.querySelector("#source-results").textContent.includes("命中")');js('document.querySelector("#next-hit").click()');check('source_next_match','document.querySelector("#source-results").textContent.includes("2 /")');shot('source_desktop')
   js('window.__copied=null; navigator.clipboard.writeText=async text=>{window.__copied=text;}; document.querySelector("#copy-source").click()');check('source_copy_exact','window.__copied===ROBOT_BOOK.sources.find(s=>s.id==='+json.dumps(sid)+').code')
   route('#/materials');wait('Array.from(document.querySelectorAll("#material-list img")).every(i=>i.complete&&i.naturalWidth>0)');check('diagrams_displayed','document.querySelectorAll(".asset-card").length===10');shot('materials_desktop');js('document.querySelector("[data-image]").click()');check('image_lightbox','!!document.querySelector("dialog.lightbox[open]")');js('document.querySelector("dialog.lightbox").close()');js('document.querySelector("[data-kind=screenshots]").click()');check('historical_screenshots','document.querySelectorAll(".asset-card").length===2')
   route('#/resource/'+js('ROBOT_BOOK.missing[0].id'));check('unbundled_boundary','document.querySelector("main h1").textContent.includes("未包含")')
   route('#/status');check('original_partial_retained','document.querySelector("main").textContent.includes("28 / 56")&&document.querySelectorAll(".badge.partial").length===6')
   route('#/chapter/14');js('document.querySelector("#theme-toggle").click()');check('dark_theme','document.documentElement.dataset.theme==="dark"');shot('chapter_dark');js('document.querySelector("#theme-toggle").click()')
   call('Emulation.setEmulatedMedia',dict(media='print'));check('print_hides_navigation','getComputedStyle(document.querySelector(".sidebar")).display==="none"');check('print_full_code','getComputedStyle(document.querySelector("pre")).maxHeight==="none"');call('Emulation.setEmulatedMedia',dict(media='screen'))
   call('Emulation.setDeviceMetricsOverride',dict(width=430,height=932,deviceScaleFactor=1,mobile=True));route('#/home');check('mobile_no_overflow','document.documentElement.scrollWidth<=innerWidth+1');shot('home_mobile');js('document.querySelector("#menu-toggle").click()');check('mobile_menu','document.querySelector("#sidebar").classList.contains("open")');js('document.querySelector("#sidebar-scrim").click()');check('mobile_close_menu','!document.querySelector("#sidebar").classList.contains("open")');route('#/chapter/14');check('mobile_chapter_no_overflow','document.documentElement.scrollWidth<=innerWidth+1');shot('chapter_mobile');route('#/source/'+sid);check('mobile_source_no_overflow','document.documentElement.scrollWidth<=innerWidth+1')
   call('Emulation.setDeviceMetricsOverride',dict(width=1440,height=1050,deviceScaleFactor=1,mobile=False));call('Page.addScriptToEvaluateOnNewDocument',dict(source='Object.defineProperty(window,"localStorage",{get(){throw new Error("blocked for acceptance test");}})'))
   call('Network.emulateNetworkConditions',dict(offline=True,latency=0,downloadThroughput=0,uploadThroughput=0));navigate(preview.joinpath('index.html').as_uri()+'#/home');check('file_url_unicode_spaces','location.protocol==="file:"&&decodeURIComponent(location.pathname).includes("Windows 学习 folder")');check('storage_unavailable_notice','!document.querySelector("#storage-message").hidden');route('#/search?q=GNSS');wait('!!document.querySelector(".result")');check('offline_search','document.querySelectorAll(".result").length>0');route('#/chapter/12');check('offline_content','!!document.querySelector("article h1")');check('offline_no_remote_assets','Array.from(document.querySelectorAll("script[src],link[rel=stylesheet],img")).every(n=>!(n.getAttribute("src")||n.getAttribute("href")||"").startsWith("http"))')
   js('navigator.clipboard.writeText=async()=>{throw new Error("denied for check");}; document.execCommand=()=>false; document.querySelector("[data-copy]").click()');wait('document.querySelector("#toast").textContent.includes("Ctrl+C")');check('clipboard_denied_manual_guidance','document.querySelector("#toast").textContent.includes("Ctrl+C")')
   check('no_runtime_exceptions',json.dumps(not runtime_exceptions))
   result=dict(passed=not errors and all(x['passed'] for x in records),inputs={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['index.html','web/site.js','web/site.css','web/book-data.js']},records=records,errors=errors,runtime_exceptions=runtime_exceptions,scope='Real UI on Linux Chrome; HTTP nested Unicode/space path, file URL, desktop/mobile/print, notes downloads and file import. No robot simulation.',windows_native_manual='PENDING_USER')
   (OUT/'browser_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(passed=result['passed'],checks=len(records),errors=errors,exceptions=len(runtime_exceptions)),ensure_ascii=False));wire.close()
   if not result['passed']:raise SystemExit(2)
  except Exception as error:
   (OUT/'browser_check.json').write_text(json.dumps(dict(passed=False,records=records,errors=[*errors,str(error)],scope='Interrupted browser check; not a passing receipt.'),ensure_ascii=False,indent=2)+'\n')
   raise
  finally:
   proc.terminate()
   try:proc.wait(8)
   except subprocess.TimeoutExpired:proc.kill();proc.wait(3)
   server.shutdown();server.server_close()
