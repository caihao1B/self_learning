"""Shared calibration and deterministic measurement mathematics; no world access."""
import hashlib,json,math
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[5];BASE=ROOT/'optimization_v2';PKG=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def dump(p,d):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_name(p.name+'.tmp');tmp.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');tmp.replace(p)
def settings():return yaml.safe_load((BASE/'config/sensors.yaml').read_text())
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def set_stamp(m,t):m.header.stamp.sec=int(t);m.header.stamp.nanosec=round((t-int(t))*1e9)
def phase(t,profile):return next((p for p in profile if p['start']<=t<p['end']),{'name':'nominal'})
def ecef(llh):
    lat,lon,h=llh;lat,lon=map(math.radians,[lat,lon]);a=6378137.;e2=6.69437999014e-3;n=a/math.sqrt(1-e2*math.sin(lat)**2)
    return np.array([(n+h)*math.cos(lat)*math.cos(lon),(n+h)*math.cos(lat)*math.sin(lon),(n*(1-e2)+h)*math.sin(lat)])
def geodetic(xyz):
    x,y,z=xyz;p=math.hypot(x,y);a=6378137.;e2=6.69437999014e-3;lat=math.atan2(z,p*(1-e2))
    for _ in range(8):
        n=a/math.sqrt(1-e2*math.sin(lat)**2);h=p/math.cos(lat)-n;lat=math.atan2(z,p*(1-e2*n/(n+h)))
    return np.array([math.degrees(lat),math.degrees(math.atan2(y,x)),h])
def enu_rotation(origin):
    a,b=map(math.radians,origin[:2]);return np.array([[-math.sin(b),math.cos(b),0],[-math.sin(a)*math.cos(b),-math.sin(a)*math.sin(b),math.cos(a)],[math.cos(a)*math.cos(b),math.cos(a)*math.sin(b),math.sin(a)]])
def to_enu(llh,origin):return enu_rotation(origin)@(ecef(llh)-ecef(origin))
def from_enu(xyz,origin):return geodetic(ecef(origin)+enu_rotation(origin).T@xyz)
def config_hash():return hashlib.sha256(b''.join((BASE/'config'/name).read_bytes() for name in ['sensors.yaml','fault_profiles.yaml','data_contract.yaml'])).hexdigest()
