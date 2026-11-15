from common import *
import argparse,importlib
p=argparse.ArgumentParser();p.add_argument('stage');p.add_argument('command',nargs='?',default='status');p.add_argument('args',nargs=argparse.REMAINDER);a=p.parse_args()
if a.stage=='status':
 print(json.dumps({f'R{i:02}':check(f'R{i:02}') for i in range(2,13)},indent=2));raise SystemExit()
if a.stage not in [f'R{i:02}' for i in range(2,13)]:raise ValueError('UNKNOWN_STAGE')
if a.command=='check':
 r=check(a.stage);print(json.dumps(r));raise SystemExit(0 if r['passed'] else 2)
if a.command=='open':
 subprocess.Popen(['xdg-open',str(APP/'results'/a.stage/'report.html')],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);raise SystemExit()
if a.command=='status':print(json.dumps(check(a.stage)));raise SystemExit()
m=importlib.import_module('stage_'+a.stage.lower());raise SystemExit(m.main(a.command,a.args))
