"""Inspect the built book through a separate headless Chrome, using file URLs."""
from pathlib import Path
import base64,json,subprocess,tempfile,time,urllib.request
import websocket

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'validation';errors=[];seq=0
with tempfile.TemporaryDirectory(prefix='l01-chrome-') as profile:
    with (OUT/'chrome.log').open('w') as log:
        proc=subprocess.Popen(['/usr/bin/google-chrome','--headless=new','--disable-gpu',
            '--no-first-run','--no-default-browser-check','--remote-debugging-port=0',
            '--remote-allow-origins=http://localhost','--user-data-dir='+profile,'about:blank'],stdout=log,stderr=log)
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
                    if d.get('method')=='Runtime.exceptionThrown':errors.append(d)
                    if d.get('id')==seq:
                        if 'error' in d:raise RuntimeError(d['error'])
                        return d.get('result',{})
            def js(expression):
                d=call('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=True))
                if d.get('exceptionDetails'):raise RuntimeError(d)
                return d.get('result',{}).get('value')
            def shot(name):
                r=call('Page.captureScreenshot',dict(format='png',captureBeyondViewport=False));(OUT/(name+'.png')).write_bytes(base64.b64decode(r['data']))
            call('Runtime.enable');call('Page.enable')
            call('Emulation.setDeviceMetricsOverride',dict(width=1440,height=1000,deviceScaleFactor=1,mobile=False))
            call('Page.navigate',dict(url=(ROOT/'index.html').as_uri()))
            deadline=time.monotonic()+15
            while not js('document.readyState==="complete" && document.querySelectorAll("article").length>10'):
                if time.monotonic()>deadline:raise RuntimeError('PAGE_LOAD')
                time.sleep(.1)
            checks={}
            checks['file_url_reading']=js('location.protocol')=='file:'
            checks['all_images_loaded']=js('Array.from(document.images).every(i=>i.complete&&i.naturalWidth>0)')
            checks['no_external_runtime_assets']=js('Array.from(document.querySelectorAll("script[src],link[rel=stylesheet],img")).every(n=>!(n.getAttribute("src")||n.getAttribute("href")||"").startsWith("http"))')
            checks['desktop_no_body_overflow']=js('document.documentElement.scrollWidth<=innerWidth+1')
            shot('book_desktop')
            js('location.hash="#07-sample-spray"');time.sleep(.6)
            checks['sample_navigation']=js('Math.abs(document.getElementById("07-sample-spray").getBoundingClientRect().top)<100')
            shot('sample_desktop')
            js('document.getElementById("02-architecture").scrollIntoView()');time.sleep(.6);shot('architecture_desktop')
            call('Emulation.setDeviceMetricsOverride',dict(width=430,height=932,deviceScaleFactor=1,mobile=True))
            js('window.scrollTo(0,0)');time.sleep(.5)
            checks['mobile_no_body_overflow']=js('document.documentElement.scrollWidth<=innerWidth+1')
            checks['mobile_menu_present']=js('getComputedStyle(document.querySelector(".mobile-nav")).display!=="none"')
            shot('book_mobile')
            call('Emulation.setEmulatedMedia',dict(media='print'))
            checks['print_navigation_hidden']=js('getComputedStyle(document.querySelector("aside")).display==="none"')
            checks['print_title_visible']=js('getComputedStyle(document.querySelector("header h1")).color!=="rgb(255, 255, 255)"')
            call('Emulation.setEmulatedMedia',dict(media='screen'))
            call('Emulation.setDeviceMetricsOverride',dict(width=1440,height=1000,deviceScaleFactor=1,mobile=False))
            view=next((ROOT/'baseline/source_views').glob('*-mission_spray_r10.py.html'))
            call('Page.navigate',dict(url=view.as_uri()))
            deadline=time.monotonic()+10
            while not js('document.readyState==="complete" && document.querySelectorAll("span.line").length>200'):
                if time.monotonic()>deadline:raise RuntimeError('CODE_VIEW_LOAD')
                time.sleep(.1)
            checks['offline_code_view']=js('document.body.textContent.includes("ActionClient") && document.querySelectorAll("script").length===0')
            shot('source_desktop')
            checks['no_browser_exceptions']=not errors
            result=dict(passed=all(checks.values()),checks=checks,errors=errors,environment='Linux headless Google Chrome, local file URL, desktop/mobile/print emulation',windows_native_manual='PENDING_USER',scope='L01 reading preview, not full L03 website or robot GUI acceptance')
            (OUT/'browser_check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps(result,ensure_ascii=False));wire.close()
            if not result['passed']:raise SystemExit(2)
        finally:
            proc.terminate()
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=5)
