"""Independent spray observer transport edition; preserve older runtime files."""
from common import *
from prepare_r12 import recording_source

def derive(source):
    text=recording_source(source,'scene_spray_r10','R10')
    old='from transport_r08 import configure_transport,transport_environment'
    assert text.count(old)==1
    text=text.replace(old,'from transport_video_r12 import configure_transport,transport_environment',1)
    marker="bound += [Path(__file__),PKG/'scripts/record_r12.py'"
    assert text.count(marker)==1
    return text.replace(marker,"bound += [PKG/'scripts/transport_video_r12.py',PKG/'scripts/prepare_spray_video_r12.py',PKG/'scripts/prepare_r12.py',Path(__file__),PKG/'scripts/record_r12.py'",1)

def build():
    source=PKG/'scripts/runtime_spray_r10.py';text=derive(source.read_text())
    path=edition('R12')/'runtime_spray_video.py';compile(text,str(path),'exec')
    if not path.exists() or path.read_text()!=text:path.write_text(text)
    binding=dict(task='spray',source=str(source.relative_to(ROOT)),source_sha256=sha(source),
                 derived_runtime=str(path.relative_to(ROOT)),derived_sha256=sha(path),
                 changes=['same observer-only derivation','64 MiB session SHM segment for observer image writer only'],
                 scope='No control, physics, perception, map or work admission change; existing recording runtime retained')
    dump(edition('R12')/'spray_video_derivation.json',binding)
    return path,binding
