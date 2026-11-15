"""Read-only capture of the declared robot revision for L01 teaching.

The robot workspace is never edited. This reads a previously verified Git tree,
checks its declared SHA against the immutable GitHub commit, and verifies bytes.
Run from the original teaching authoring machine, not a fresh clone.
"""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import ast, collections, hashlib, json, platform, shutil, subprocess, urllib.request

ROOT = Path(__file__).resolve().parents[1]
PROJECT = Path('/home/ceihao/robot_project')
COMMIT = '9aff8337b7794f704c5556a973a5220e73f17808'
TREE = '0e71e12fbf3bfa67295abd9cec071c6edc9bd531'
URL = f'https://github.com/caihao1B/ROS2_BT-/blob/{COMMIT}/'
raw_tree = json.loads((PROJECT/'delivery/r10_completion_20261115/remote_final_tree.json').read_text())
assert raw_tree['sha'] == TREE and not raw_tree.get('truncated')
remote = {e['path']:e for e in raw_tree['tree'] if e['type'] == 'blob'}
entries = []

def capture(path, label, kind='source', snapshot=True):
    p = PROJECT/path
    data = p.read_bytes()
    blob = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    expected = remote.get(path, {}).get('sha')
    local_blob=blob
    if expected != blob:
        if kind == 'source' or expected is None:
            raise RuntimeError(f'BASELINE_MISMATCH {path}: {expected} != {blob}')
        # Earlier publishing normalized documentation links. Preserve the fixed
        # remote revision, and record the mismatch rather than editing the robot.
        raw='https://raw.githubusercontent.com/caihao1B/ROS2_BT-/'+COMMIT+'/'+urllib.parse.quote(path)
        req=urllib.request.Request(raw,headers={'User-Agent':'robot-learning-baseline'})
        with urllib.request.urlopen(req,timeout=45) as response:data=response.read()
        blob=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
        if blob != expected:raise RuntimeError('REMOTE_BLOB_MISMATCH '+path)
    local = ('baseline/source/' if kind == 'source' else 'evidence/')+path
    if snapshot:
        dest = ROOT/local
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
    row = dict(path=path, label=label, kind=kind, bytes=len(data),
               sha256=hashlib.sha256(data).hexdigest(),git_blob=blob,
               url=URL+path, local=local if snapshot else None,
               original_local_git_blob=local_blob,local_matched_frozen_revision=local_blob==blob)
    if p.suffix in ['.py','.cpp','.hpp','.cc','.sh','.xml','.action','.yaml','.md']:
        lines=data.decode('utf-8').splitlines();row['lines']=len(lines)
        if p.suffix == '.py':
            try:
                row['symbols']=[dict(name=n.name,line=n.lineno,end_line=n.end_lineno)
                    for n in ast.walk(ast.parse(data)) if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef))]
            except SyntaxError: row['symbols']=[]
    entries.append(row)
    return row

sources = [
('optimization_v2/env.sh','隔离 ROS 环境'),
('optimization_v2/three_tasks/advance.sh','阶段启动入口'),
('optimization_v2/three_tasks/gui.sh','观测 GUI 入口'),
('optimization_v2/ws/src/task_platform/scripts/dispatch.py','命令分派'),
('optimization_v2/ws/src/task_platform/scripts/common.py','回执检查与会话锁'),
('optimization_v2/ws/src/task_runtime/scripts/common_r1.py','路径与原子 JSON 保存'),
('optimization_v2/ws/src/task_platform/scripts/stage_r12.py','录像案例范围与评估'),
('optimization_v2/ws/src/task_platform/scripts/prepare_spray_video_r12.py','录制运行器确定性派生'),
('optimization_v2/ws/src/task_platform/scripts/runtime_spray_r10.py','点喷进程编排'),
('optimization_v2/ws/src/task_platform/scripts/mission_spray_r10.py','点喷任务状态机'),
('optimization_v2/ws/src/task_platform/scripts/docking_r10.py','静止精停判据'),
('optimization_v2/ws/src/task_platform/scripts/control_spray_r10.py','点喷唯一底盘出口'),
('optimization_v2/ws/src/task_platform/scripts/local_dock_r10.py','有界局部精停路径'),
('optimization_v2/ws/src/task_platform/scripts/precision_supervisor_r10.py','点喷 BT 程序派生'),
('optimization_v2/ws/src/task_platform/r10_backend/precision_progress.hpp','C++ 精停进展尺度'),
('optimization_v2/ws/src/task_platform/r10_backend/correction_filter.hpp','C++ 地图校正滤波'),
('optimization_v2/ws/src/task_platform/r10_backend/stopped_revalidation.inc','停车重验证 prepare 与 commit'),
('optimization_v2/ws/src/task_platform/r08_backend/policy.hpp','C++ 行为树判据'),
('optimization_v2/ws/src/task_platform/r08_backend/common.xml','行为树结构'),
('optimization_v2/ws/src/task_platform/scripts/regional_r08.py','区域观测地图配准'),
('optimization_v2/ws/src/task_platform/scripts/mission_mowing_r10.py','割草逐段执行'),
('optimization_v2/ws/src/task_platform/scripts/recovery_mowing_r10.py','分挡恢复与末端直线'),
('optimization_v2/ws/src/task_platform/r10_backend/safe_route.cpp','OMPL 安全连接'),
('optimization_v2/ws/src/coverage_partition/src/partition_metis.cpp','METIS 工作量分簇'),
('optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp','Fields2Cover 与 OR-Tools'),
('optimization_v2/ws/src/tool_sim/src/action_server.cpp','原始工具动作服务器'),
('optimization_v2/ws/src/task_platform/scripts/tool_build_r09.py','当前工具服务器派生'),
('optimization_v2/ws/src/task_platform/scripts/ledger_spray_r10.py','点喷账本运行适配'),
('optimization_v2/ws/src/work_ledger/scripts/ledger_core.py','账本核心'),
('optimization_v2/ws/src/mission_interfaces/action/ToolTask.action','工具动作协议'),
('optimization_v2/ws/src/task_platform/src/VehicleDrive.cc','车辆 Gazebo 插件'),
('optimization_v2/ws/src/task_platform/CMakeLists.txt','C++ 构建依赖'),
('optimization_v2/three_tasks/monitor/data.py','GUI 只读状态聚合'),
('optimization_v2/three_tasks/monitor/server.py','GUI HTTP 服务'),
('optimization_v2/three_tasks/monitor/ros_observer.py','GUI 传感器只读订阅'),
('optimization_v2/three_tasks/monitor/index.html','GUI 浏览器界面'),
]
for p,label in sources: capture(p,label)
docs=[
('docs/optimization_v2/命令Prompt合集.md','S01 至 S18 原任务计划'),
('docs/three_tasks/实施步骤.md','R01 至 R12 改造计划'),
('docs/three_tasks/R05/TECHNICAL.md','定位方案变更理由'),
('docs/three_tasks/R08/TECHNICAL.md','巡检与跟随依据'),
('docs/three_tasks/R09/TECHNICAL.md','割草几何与反馈修复'),
('docs/three_tasks/R10/GUI_AND_BOUNDED_REPAIR.md','最新修复及未完成范围'),
('docs/three_tasks/R12/LOCAL_REPRODUCTION.md','资源与迁移边界'),
('docs/optimization_v2/S15/RESULTS.md','历史预测实验结论'),
('docs/optimization_v2/S16/RESULTS.md','历史缓存实验结论'),
]
for p,label in docs: capture(p,label,'document')
evidence=[
('validation/R12/spray/mission_result.json','三点点喷结果'),
('validation/R12/spray/mission_events.jsonl','三点任务事件'),
('validation/R12/spray/fault_injection.json','工具 JAM 注入'),
('validation/R12/spray/recording_acceptance.json','三点评估与录像回执'),
('validation/R12/spray/spec.json','实际运行参数与源码绑定'),
('validation/R10/bounded_A1_result.json','完整 A1 结果'),
('validation/R10/bounded_B1_result.json','完整 B1 结果'),
('validation/R10/repair_attempts.json','42 次历史尝试摘要'),
('validation/R10/terminal_live_evidence.json','末端恢复实际事件'),
('validation/GUI/acceptance.json','GUI 自动检查'),
('validation/GUI/local_resource_inventory.json','未随 Git 同步的资源范围'),
]
for s in ['R08','R09','R10','R11','R12']:evidence.append((f'validation/{s}/latest.json',s+' 固定版本回执'))
for p,label in evidence:capture(p,label,'evidence')
for p,label in [('media/gui_overview.png','真实 GUI 总览截图'),('media/gui_sensors.png','真实 GUI 传感器截图')]:
    e=capture(p,label,'image',False);shutil.copyfile(PROJECT/p, ROOT/'assets'/Path(p).name);e['local']='assets/'+Path(p).name

# Compare every published first-party workspace source file, not just tutorial excerpts.
source_rows=[]
for path,e in sorted(remote.items()):
    if not path.startswith('optimization_v2/ws/src/'):continue
    p=PROJECT/path
    b=p.read_bytes() if p.is_file() else None
    match=b is not None and hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()==e['sha']
    source_rows.append(dict(path=path,git_blob=e['sha'],local_matches=match))
assert source_rows and all(x['local_matches'] for x in source_rows)
packages=['ros-jazzy-ros-gz-sim','ros-jazzy-robot-localization','ros-jazzy-rtabmap-ros',
          'ros-jazzy-behaviortree-cpp','ros-jazzy-nav2-controller','ros-jazzy-fields2cover',
          'ros-jazzy-ortools-vendor','libmetis-dev','nlohmann-json3-dev']
r=subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\t${Status}\n',*packages],capture_output=True,text=True)
env=dict(python=platform.python_version(),machine=platform.machine(),os_release=Path('/etc/os-release').read_text(),dpkg_exit=r.returncode,packages=r.stdout.splitlines(),errors=r.stderr.splitlines())
components=subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\t${Status}\n',
    'ros-jazzy-rtabmap','ros-jazzy-rtabmap-odom','ros-jazzy-rtabmap-slam'],capture_output=True,text=True)
env['rtabmap_components']=dict(exit_code=components.returncode,packages=components.stdout.splitlines(),errors=components.stderr.splitlines())
(ROOT/'validation/rtabmap_components.txt').write_text(components.stdout+components.stderr)
manifest=dict(schema_version=1,project_repository='caihao1B/ROS2_BT-',commit=COMMIT,tree=TREE,
    captured_local=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),timezone='Asia/Shanghai',
    source_verification_scope='All published optimization_v2/ws/src blobs compared with local bytes; no new simulation',
    source_files=source_rows,entries=entries,environment=env,
    rights='Source excerpts retain original ownership and notices; no project-wide license grant inferred.')
(ROOT/'baseline/manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
rows=json.loads((PROJECT/'validation/R10/repair_attempts.json').read_text());unique={x['path']:x for x in rows}
unsuccessful=[x for x in unique.values() if not x.get('task_passed_at_execution')]
summary=dict(total_records=len(rows),unique_runs=len(unique),not_passed_at_execution=len(unsuccessful),
    stop_code_counts=dict(collections.Counter(str(x.get('reason') or x.get('terminal') or 'UNSPECIFIED') for x in unsuccessful)),
    warning='Not a random sample or root-cause frequency; includes interrupted/bounded sessions and multiple software versions.')
(ROOT/'evidence/failure_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(source_files=len(source_rows),captured_files=len(entries),all_match=True,dpkg_exit=r.returncode)))
