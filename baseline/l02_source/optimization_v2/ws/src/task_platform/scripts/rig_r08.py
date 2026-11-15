"""Versioned sampling change; sealed R05 calibration and noise remain immutable."""
import copy
from common import canonical,read,dump

def effective_rig(base,near):
    rig=copy.deepcopy(base)
    rig.update(schema='R08-EFFECTIVE-SENSORS/1',base_calibration_revision=base['revision'],near_sensor=copy.deepcopy(near))
    rig['lidar'].update(horizontal=2048,vertical=128)
    rig['branches']['perception_sampling']=dict(max_rings=64,max_azimuth=512,near_channels='all original independent rays',localization='full 128 x 2048')
    rig['sampling_change_reason']='Actual extra simulator rays for sparse frozen-map landmarks; no repeated-point inflation or relaxed quality gates'
    rig['revision']=canonical({k:v for k,v in rig.items() if k!='revision'})
    return rig

def validate(base,rig):
    if rig==base:return rig
    if rig.get('revision')!=canonical({k:v for k,v in rig.items() if k!='revision'}):raise ValueError('EFFECTIVE_RIG_HASH')
    expected=effective_rig(base,rig.get('near_sensor'))
    if rig!=expected:raise ValueError('EFFECTIVE_RIG_CALIBRATION_OR_SAMPLING_MISMATCH')
    return rig

def runtime_rig(out,base):
    # All runtime consumers bind the same task mission, independently of old calibration checks.
    return validate(base,read(out.parent/'mission.json')['rig'])

def apply_scene(main,out,cfg):
    main.find('lidar/scan/horizontal/samples').text=str(cfg['rig']['lidar']['horizontal'])
    main.find('lidar/scan/vertical/samples').text=str(cfg['rig']['lidar']['vertical'])
    dump(out/'sensor_manifest.json',cfg['rig'])

def bind_adapter(out,cfg):
    p=out/'common.py';s=p.read_text();start=s.index('def settings():');end=s.index('\n',start)
    s=s[:start]+'def settings():return read('+repr(str(out.parent/'scene/sensor_manifest.json'))+')'+s[end:];p.write_text(s)
    from common import sha
    d=read(out/'adaptation.json');d.update(base_calibration_revision=d['rig_revision'],rig_revision=cfg['rig']['revision']);d['generated']['common.py']=sha(p);dump(out/'adaptation.json',d)
