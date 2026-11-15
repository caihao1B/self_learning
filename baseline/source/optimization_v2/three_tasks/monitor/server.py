"""Local read-only mission inspector. GET endpoints only; no actuator interface."""
import argparse,json,os,signal,subprocess,sys,threading,time
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse,parse_qs
from data import Evidence,json_value

BASE=Path(__file__).resolve().parent

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',type=int,default=8088);ap.add_argument('--no-ros',action='store_true')
    ap.add_argument('--app',type=Path,default=BASE.parent)
    args=ap.parse_args();evidence=Evidence(args.app);lock=threading.RLock();child=None
    state=args.app/'monitor_state';state.mkdir(parents=True,exist_ok=True)
    def start_observer():
        nonlocal child
        env=os.environ.copy();env['ROS_DOMAIN_ID']='119';env['OMP_NUM_THREADS']='1';env['OPENBLAS_NUM_THREADS']='1'
        # Use the project's already validated large-message profile, if present.
        p,_=evidence.resolve()
        if p and (p/'fastdds.xml').exists():
            env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(p/'fastdds.xml');env['FASTDDS_DEFAULT_PROFILES_FILE']=str(p/'fastdds.xml')
        with (state/'ros_observer.log').open('a') as logfile:
            child=subprocess.Popen([sys.executable,str(BASE/'ros_observer.py'),'--app',str(args.app)],env=env,stdout=logfile,stderr=subprocess.STDOUT)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*a):pass
        def reply(self,data,kind='application/json; charset=utf-8',code=200):
            if isinstance(data,(dict,list)):data=json.dumps(json_value(data),ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            u=urlparse(self.path);q=parse_qs(u.query);name=q.get('run',['auto'])[0]
            try:
                with lock:
                    if u.path in ['/','/index.html']:return self.reply((BASE/'index.html').read_bytes(),'text/html; charset=utf-8')
                    if u.path=='/api/runs':return self.reply(evidence.runs())
                    if u.path=='/api/state':return self.reply(evidence.state(name))
                    if u.path=='/api/geometry':return self.reply(evidence.geometry(name))
                    if u.path=='/api/preview':
                        p,_=evidence.resolve(name);f=p/'video/preview.png' if p else None
                        if f and f.is_file():return self.reply(f.read_bytes(),'image/png')
                    if u.path=='/api/camera':
                        p,rid=evidence.resolve(name);s=evidence.state(name)
                        f=state/'camera.jpg'
                        if s.get('observer',{}).get('run')==rid and s.get('observer',{}).get('camera_run')==rid and f.is_file():return self.reply(f.read_bytes(),'image/jpeg')
                self.reply({'error':'Not available'},code=404)
            except (ValueError,OSError,KeyError) as e:self.reply({'error':str(e)},code=400)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.daemon_threads=True
    stopping=threading.Event()
    def stop(*_):
        if not stopping.is_set():stopping.set();threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        # Claim the HTTP port first: a duplicate launch must not orphan a ROS node.
        if not args.no_ros:start_observer()
        print(f'机器人实时监视： http://127.0.0.1:{args.port}  （只读，Ctrl+C 退出）',flush=True)
        (state/'server.json').write_text(json.dumps(dict(pid=os.getpid(),port=args.port,readonly=True,started_wall=time.time())))
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(5)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(5)
                except subprocess.TimeoutExpired:child.kill();child.wait(2)
        (state/'server.json').write_text(json.dumps(dict(pid=os.getpid(),port=args.port,stopped_wall=time.time(),readonly=True)))
if __name__=='__main__':main()
