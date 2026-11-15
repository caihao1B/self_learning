"""Check every local L02 reading page with an isolated Linux Chrome profile."""
from pathlib import Path
import base64,json,subprocess,tempfile,time,urllib.request
import websocket
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'validation/L02';seq=0;exceptions=[]
catalog=json.loads((ROOT/'L02/catalog.json').read_text())
with tempfile.TemporaryDirectory(prefix='l02-reading-check-') as profile:
    with (OUT/'chrome.log').open('w') as log:
        proc=subprocess.Popen(['/usr/bin/google-chrome','--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check','--remote-debugging-port=0','--remote-allow-origins=http://localhost','--user-data-dir='+profile,'about:blank'],stdout=log,stderr=log)
        try:
            marker=Path(profile)/'DevToolsActivePort';deadline=time.monotonic()+20
            while not marker.exists():
                if time.monotonic()>deadline or proc.poll() is not None:raise RuntimeError('CHROME_START')
                time.sleep(.1)
            port=marker.read_text().splitlines()[0]
            tab=next(x for x in json.load(urllib.request.urlopen(f'http://127.0.0.1:{port}/json')) if x['type']=='page')
            wire=websocket.create_connection(tab['webSocketDebuggerUrl'],origin='http://localhost',timeout=15)
            def call(method,params=None):
                global seq
                seq+=1;wire.send(json.dumps(dict(id=seq,method=method,params=params or {})))
                while True:
                    d=json.loads(wire.recv())
                    if d.get('method')=='Runtime.exceptionThrown':exceptions.append(d)
                    if d.get('id')==seq:
                        if 'error' in d:raise RuntimeError(d['error'])
                        return d.get('result',{})
            def js(expression):
                d=call('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=True))
                if d.get('exceptionDetails'):raise RuntimeError(d)
                return d.get('result',{}).get('value')
            def navigate(p):
                call('Page.navigate',dict(url=p.as_uri()));deadline=time.monotonic()+10
                while not js('document.readyState==="complete" && !!document.querySelector("h1")'):
                    if time.monotonic()>deadline:raise RuntimeError('PAGE_LOAD '+str(p))
                    time.sleep(.05)
            def shot(name):
                r=call('Page.captureScreenshot',dict(format='png',captureBeyondViewport=False))
                (OUT/(name+'.png')).write_bytes(base64.b64decode(r['data']))
            call('Runtime.enable');call('Page.enable');call('Emulation.setDeviceMetricsOverride',dict(width=1440,height=1000,deviceScaleFactor=1,mobile=False))
            records=[]
            for chapter in catalog:
                navigate(ROOT/chapter['html'])
                title=js('document.querySelector("h1").textContent')
                checks=dict(title=title==chapter['title'],file_url=js('location.protocol==="file:"'),
                    images=js('Array.from(document.images).every(i=>i.complete&&i.naturalWidth>0)'),
                    no_overflow=js('document.documentElement.scrollWidth<=innerWidth+1'),
                    no_remote_assets=js('Array.from(document.querySelectorAll("script[src],link[rel=stylesheet],img")).every(n=>!(n.getAttribute("src")||n.getAttribute("href")||"").startsWith("http"))'),
                    code_present=js('document.querySelectorAll("pre code").length>=2'),
                    next_previous=js('document.querySelector("header").textContent.includes("三十章目录")'))
                records.append(dict(chapter=chapter['number'],checks=checks,passed=all(checks.values())))
                if chapter['number'] in [10,14,23,26,27]:shot(f'chapter_{chapter["number"]:02}_desktop')
            navigate(ROOT/'L02/index.html');shot('contents_desktop')
            index_checks=dict(thirty_chapter_links=js(r'Array.from(document.querySelectorAll("main a")).filter(a=>/read\/[0-9][0-9]_/.test(a.getAttribute("href")||"")).length')==30)
            navigate(ROOT/'L02/read/14_global_localization.html')
            call('Emulation.setDeviceMetricsOverride',dict(width=430,height=932,deviceScaleFactor=1,mobile=True))
            index_checks['mobile_no_overflow']=js('document.documentElement.scrollWidth<=innerWidth+1');shot('chapter_14_mobile')
            # Reach a real code block; long source lines must wrap without body overflow.
            js('document.querySelector("pre").scrollIntoView()');shot('code_mobile')
            index_checks['mobile_code_no_overflow']=js('document.documentElement.scrollWidth<=innerWidth+1')
            call('Emulation.setEmulatedMedia',dict(media='print'))
            index_checks['print_header_hidden']=js('getComputedStyle(document.querySelector("header")).display==="none"')
            index_checks['print_text_visible']=js('getComputedStyle(document.querySelector("h1")).color!=="rgb(255, 255, 255)"')
            call('Emulation.setEmulatedMedia',dict(media='screen'))
            view=next((ROOT/'baseline/l02_views').glob('*-precision_progress.hpp.html'))
            navigate(view);index_checks['source_view']=js('document.body.textContent.includes("progressDelta")&&document.querySelectorAll(".line").length>10')
            index_checks['no_exceptions']=not exceptions
            result=dict(passed=all(r['passed'] for r in records) and all(index_checks.values()),chapters=records,shared_checks=index_checks,exceptions=exceptions,
                environment='Linux Chrome headless, file URL, desktop 1440x1000, mobile emulation 430x932, print CSS',windows_native_manual='PENDING_USER',scope='L02 reading edition; not L03 search/hosting or robot simulation')
            (OUT/'browser_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps(dict(passed=result['passed'],chapters=len(records),chapter_checks=sum(len(r['checks']) for r in records),shared_checks=index_checks,failed=[r for r in records if not r['passed']]),ensure_ascii=False))
            wire.close()
            if not result['passed']:raise SystemExit(2)
        finally:
            proc.terminate()
            try:proc.wait(8)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(3)
