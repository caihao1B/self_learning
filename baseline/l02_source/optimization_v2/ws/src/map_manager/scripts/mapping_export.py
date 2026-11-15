"""Read/export the real DB after normal RTAB shutdown; bind every payload hash."""
import argparse,subprocess,time,sqlite3,uuid
from s06_common import *
def command(out,name,args,timeout=120):
    t=time.monotonic()
    with (out/(name+'.log')).open('w') as f:
        r=subprocess.run(args,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,timeout=timeout)
    with (out/'export_commands.jsonl').open('a') as f:f.write(json.dumps(dict(name=name,args=args,exit_code=r.returncode,wall_seconds=time.monotonic()-t))+'\n')
    if r.returncode:raise RuntimeError(name+' failed')
def safe(root,relative):
    p=Path(relative)
    if p.is_absolute() or '..' in p.parts:raise ValueError('unsafe manifest path')
    p=(root/p).resolve()
    if not p.is_relative_to(root.resolve()) or not p.is_file():raise ValueError('missing/escaped artifact')
    return p
def verify_manifest(path):
    d=read(path)
    if d['schema']!='S06-MAPPING/1':raise ValueError('unsupported map schema')
    for item in d['bindings']:
        if digest(safe(ROOT,item['path']))!=item['sha256']:raise ValueError('MAP_BINDING_MISMATCH: '+item['path'])
    return True
def export(out):
    out=Path(out);db=out/'mapping.db';before=digest(db)
    with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as conn:
        check=conn.execute('PRAGMA integrity_check').fetchone()[0]
    if check!='ok':raise ValueError('SQLite integrity check failed')
    command(out,'graph_probe',[str(BASE/'ws/install/map_manager/lib/map_manager/graph_probe'),str(db),str(out/'db_graph.json')])
    for name,opt in [('map_optimized',2),('map_odometry',3)]:
        command(out,'export_'+name,['rtabmap-export','--cloud','--scan','--poses','--poses_format','11','--ascii','--voxel','0.20','--max_range','35','--opt',str(opt),'--output',name,'--output_dir',str(out),'--threads','2',str(db)],180)
    if digest(db)!=before:raise RuntimeError('Read-only export modified source DB')
    files=[db,out/'db_graph.json',out/'map_optimized_cloud.ply',out/'map_odometry_cloud.ply',out/'scene/world.sdf',out/'scene/vehicle.json',out/'scene/sensor_manifest.json']
    files+=list((out/'scene').glob('*.png'))
    captured=read(out/'capture_inputs.json')
    for item in captured:
        if digest(safe(ROOT,item['path']))!=item['sha256']:raise ValueError('CAPTURE_SOURCE_CHANGED: '+item['path'])
    bindings=[dict(path=str(p.relative_to(ROOT)),sha256=digest(p)) for p in files]+captured
    manifest=dict(schema='S06-MAPPING/1',map_id=str(uuid.uuid4()),mode='MAPPING',frame='map',frame_epoch=str(uuid.uuid4()),source='Actual noisy stereo and GPU lidar measurements; not SDF geometry conversion',database_version='0.23.13',local_pose_source='/s05/odometry',global_tf_owner='s06_slam',bindings=bindings,db_sha256=before,sqlite_integrity=check,manual_status='NOT_CHECKED')
    dump(out/'map_manifest.json',manifest);verify_manifest(out/'map_manifest.json');return manifest
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();print(json.dumps(export(a.out),ensure_ascii=False))
