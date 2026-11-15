"""Capture additional teaching references at the unchanged L01 revision.
Authoring host only. Never imports or executes robot modules.
"""
from pathlib import Path
import hashlib, json, ast

ROOT = Path(__file__).resolve().parents[1]
ROBOT = Path('/home/ceihao/robot_project')
REV = '9aff8337b7794f704c5556a973a5220e73f17808'
TREE = '0e71e12fbf3bfa67295abd9cec071c6edc9bd531'
tree = json.loads((ROBOT/'delivery/r10_completion_20261115/remote_final_tree.json').read_text())
assert tree['sha'] == TREE and not tree.get('truncated')
remote = {e['path']: e['sha'] for e in tree['tree'] if e['type'] == 'blob'}
selections = '''
optimization_v2/ws/src/scenario_world/scripts/generate_scene.py
optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py
optimization_v2/ws/src/sensor_sim_adapter/scripts/common.py
optimization_v2/ws/src/localization_guard/scripts/icp_gate.py
optimization_v2/ws/src/localization_guard/scripts/cloud_filter.py
optimization_v2/ws/src/localization_guard/scripts/input_adapter.py
optimization_v2/ws/src/map_manager/scripts/mapping_export.py
optimization_v2/ws/src/map_manager/src/graph_probe.cpp
optimization_v2/ws/src/gnss_supervisor/src/gnss_supervisor.cpp
optimization_v2/ws/src/terrain_perception/include/terrain_perception/risk.hpp
optimization_v2/ws/src/terrain_perception/src/terrain_node.cpp
optimization_v2/ws/src/object_fusion/scripts/tracker.py
optimization_v2/ws/src/experience_memory/scripts/engine16.py
optimization_v2/ws/src/experience_memory/scripts/features16.py
optimization_v2/ws/src/motion_prediction/scripts/imm15.py
optimization_v2/ws/src/motion_prediction/scripts/trajectron15.py
optimization_v2/ws/src/coverage_partition/scripts/geometry11.py
optimization_v2/ws/src/coverage_partition/scripts/partition11.py
optimization_v2/ws/src/task_platform/scripts/planning_r03.py
optimization_v2/ws/src/task_platform/scripts/rig_r08.py
optimization_v2/ws/src/task_platform/scripts/startup_r08.py
optimization_v2/ws/src/task_platform/scripts/tracking_error_r08.py
optimization_v2/ws/src/task_platform/scripts/obstruction_memory_r10.py
optimization_v2/ws/src/task_platform/r09_backend/self_filter.hpp
optimization_v2/ws/src/task_platform/r08_backend/surface_statistics.hpp
optimization_v2/ws/src/task_platform/r08_nav/observed_prior.cpp
optimization_v2/ws/src/task_platform/r07_nav/exact_goal.cpp
optimization_v2/ws/src/task_platform/scripts/mission_r08.py
optimization_v2/ws/src/task_platform/scripts/prepare_safe_plan_r10.py
optimization_v2/ws/src/task_platform/scripts/safe_plan_r10.py
optimization_v2/ws/src/task_platform/scripts/tracker_r08.py
optimization_v2/ws/src/task_platform/scripts/terrain_r08.py
optimization_v2/ws/src/task_platform/scripts/terrain_r09.py
optimization_v2/config/slam_mapping.yaml
optimization_v2/config/slam_localization.yaml
optimization_v2/config/local_estimator.yaml
optimization_v2/config/icp_odometry.yaml
optimization_v2/config/sensors.yaml
optimization_v2/config/controller_profiles.yaml
optimization_v2/maps/prior/workspaces.geojson
optimization_v2/maps/prior/permanent.geojson
optimization_v2/maps/prior/terrain_coarse.geojson
optimization_v2/maps/frozen/manifest.json
optimization_v2/three_tasks/editions/R05/mowing/sensors.json
optimization_v2/three_tasks/editions/R05/mowing/binding.json
optimization_v2/three_tasks/editions/R09/nav2.yaml
optimization_v2/three_tasks/editions/R10/spray/nav2.yaml
optimization_v2/three_tasks/editions/R10/mowing/mission.json
optimization_v2/three_tasks/editions/R10/spray/mission.json
optimization_v2/ws/src/task_platform/scripts/dispatch.py
'''.strip().splitlines()

entries=[];missing=[]
for path in selections:
    if path not in remote or not (ROBOT/path).is_file():
        missing.append(path);continue
    data=(ROBOT/path).read_bytes()
    blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    if blob != remote[path]:raise ValueError('BASELINE_MISMATCH '+path)
    dest=ROOT/'baseline/l02_source'/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
    row=dict(path=path,local=str(dest.relative_to(ROOT)),sha256=hashlib.sha256(data).hexdigest(),git_blob=blob,bytes=len(data),url=f'https://github.com/caihao1B/ROS2_BT-/blob/{REV}/{path}')
    if path.endswith('.py'):
        row['symbols']=[dict(name=n.name,line=n.lineno,end=n.end_lineno) for n in ast.walk(ast.parse(data)) if isinstance(n,(ast.FunctionDef,ast.ClassDef,ast.AsyncFunctionDef))]
    entries.append(row)
out=dict(commit=REV,tree=TREE,entries=entries,missing_selections=missing,scope='Additional L02 fixed references; L01 manifest unchanged')
(ROOT/'baseline/l02_manifest.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(captured=len(entries),missing=missing),ensure_ascii=False))
