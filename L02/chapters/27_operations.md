# 第 27 章 三任务人工操作与录像

本章给出当前作者机已经准备好的工程操作卡。它们会启动真实仿真并生成新结果，不是离线练习。一次只运行一个机器人案例，先读清范围，再启动，最后核对任务、工具、停车和录像分别是否通过。

## 启动前的只读检查

```bash
cd /home/ceihao/robot_project
pwd
test -f optimization_v2/maps/frozen/map.db
echo $?
test -x optimization_v2/venvs/scene/bin/python
echo $?
bash optimization_v2/three_tasks/advance.sh R09 check
echo $?
```

最后检查应符合固定基线 R09 PASS。若绑定 changed，不继续拿旧回执当准入。`record` 入口实际依赖 R09，因此 R10 PARTIAL 不自动禁止短录像；这并不意味着可以宣布 R10 已通过。

## 终端一 开监视 GUI

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/gui.sh
```

在同一 Linux 机器浏览器打开 `http://127.0.0.1:8088`。GUI 是只读监视，打开后可能先显示历史运行。核对会话与 live/historical 标识，不把历史完成画面当当前状态。Windows 离线教材与这个实时 GUI 是两个不同页面。

## 终端二 选择一个案例

|案例名|明确范围|重点观察|
|---|---|---|
|spray|前三点，工具 JAM，300 仿真秒上限|导航、精停、卡滞、残余重试、重放|
|inspection|当前声明巡检案例，480 秒上限|路线、路径进展、停车|
|inspection_leader|20 m 距离上限，150 秒|同向目标与保持距离|
|inspection_blocked|20 m 距离上限，120 秒|持久阻塞与有界处理|
|mowing|前五个执行段，420 秒|工作段/连接段、换挡、工具面积|
|mowing_startup_perception_loss|前五段，启动感知故障，420 秒|START_WORK 中断与恢复|
|spray_full_A1|完整 A1，1800 秒|长期定位与完整义务，历史 PARTIAL|
|mowing_full_B1|完整安全执行 B1，1800 秒|完整段与残余，历史 PARTIAL|

先选一个短案例，以下命令是互斥选项，不要全部粘贴同时运行：

```bash
bash optimization_v2/three_tasks/advance.sh R12 record spray
```

```bash
bash optimization_v2/three_tasks/advance.sh R12 record inspection_leader
```

```bash
bash optimization_v2/three_tasks/advance.sh R12 record mowing
```

若需要测试 START_WORK 响应，将末尾案例改为 `mowing_startup_perception_loss`。完整任务另行选择 `spray_full_A1` 或 `mowing_full_B1`，可能耗时明显更久；本轮教材编写没有重跑它们，也没有执行六十次矩阵。

## 看 Gazebo 窗口

已有录制任务运行后，在另一个终端执行：

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R12 view-live
```

该入口寻找当前活动会话并连接对应 GZ_PARTITION 的 GUI。没有活动会话时会拒绝，而不是自己新开一个空世界。额外图形窗口会增加计算负担，观察到 RTF 降低应区分显示开销与算法错误。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### stage_r12.py 第 1 至 18 行

[离线带行号源码](../../baseline/l02_views/ae5ff590266d-stage_r12.py.html#L1) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/stage_r12.py#L1)

```python
"""Truthful separate recordings; partial upstream acceptance never becomes a full release."""
from common import *
import importlib.util
from prepare_r12 import build

CASES=dict(spray=dict(task='spray',case='tool_jam',point_limit=3,seconds=300),spray_full_A1=dict(task='spray',case='nominal',point_limit=0,seconds=1800),inspection=dict(task='inspection',case='nominal',seconds=480),inspection_leader=dict(task='inspection',case='leader',distance_limit=20.,seconds=150),inspection_blocked=dict(task='inspection',case='blocked',distance_limit=20.,seconds=120),mowing=dict(task='mowing',case='nominal',segment_limit=5,seconds=420),mowing_full_B1=dict(task='mowing',case='nominal',segment_limit=0,seconds=1800))

CASES['mowing_startup_perception_loss']=dict(task='mowing',case='startup_perception_loss',segment_limit=5,seconds=420)

def declared_scope_matches(receipt,spec,name):
 expected=CASES[name];fields={'seconds':'sim_seconds','distance_limit':'distance_limit_m'}
 if expected['task']=='mowing' and spec.get('segment_start',0)!=0:return False
 if receipt.get('case')!=name or receipt.get('task')!=expected['task'] or spec.get('task')!=expected['task']:return False
 if not all(spec.get(fields.get(k,k))==v for k,v in expected.items() if k!='task'):return False
 return expected['task']!='inspection' or spec.get('distance_limit_m')==expected.get('distance_limit',0.)

def recheck(receipt):
 """A saved PASS is not enough after code/config or video changes."""
```

案例字典明确点数/段数/时间，完整案例用 limit=0 表达全范围。读取此声明能防止误把默认短例当完整簇。额外启动故障案例在后续赋值加入，AST 练习同时读取。

### stage_r12.py 第 33 至 60 行

[离线带行号源码](../../baseline/l02_views/ae5ff590266d-stage_r12.py.html#L33) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/stage_r12.py#L33)

```python
def record(name):
 require('R09')
 if name not in CASES:raise ValueError('UNKNOWN_RECORDING_CASE')
 spec=CASES[name].copy();task=spec.pop('task');path=edition('R12')/('runtime_'+task+'.py');binding=next(x for x in read(edition('R12')/'runtime_derivation.json')['records'] if x['task']==task)
 if task=='mowing':
  from prepare_mowing_video_r12 import build as build_video_transport
  path,binding=build_video_transport()
 if task=='spray':
  from prepare_spray_video_r12 import build as build_video_transport
  path,binding=build_video_transport()
 if sha(ROOT/binding['source'])!=binding['source_sha256'] or sha(path)!=binding['derived_sha256']:raise ValueError('REBUILD_R12_AFTER_RUNTIME_CHANGE')
 module=importlib.util.spec_from_file_location('r12_'+task,path);runtime=importlib.util.module_from_spec(module);module.loader.exec_module(runtime)
 # Long mowing and held-obstacle recordings previously exceeded image age on this host.
 # Lower only wall-clock pacing; task clock, actions and physical limits remain original.
 out=runtime.run(rtf=.3,**spec) if name=='inspection_blocked' else runtime.run(rtf=.4,**spec) if name=='mowing_full_B1' else runtime.run(rtf=.5,**spec) if name=='spray_full_A1' else runtime.run(**spec)
 if task=='spray':
  from analyze_spray_r10 import analyze
 elif task=='mowing':
  from analyze_mowing_r10 import analyze
 else:
  from analyze_r08 import analyze
 result=analyze(out);video=read(out/'video/recording.json') if (out/'video/recording.json').exists() else {};frames=[json.loads(l) for l in (out/'video/frames.jsonl').read_text().splitlines()] if (out/'video/frames.jsonl').exists() else [];task_result=result.get('task_result',{});events=[json.loads(l) for l in (out/'mission_events.jsonl').read_text().splitlines()];first_action=next((x['t'] for x in events if x['event'] in ['NAV_GOAL','SEGMENT_GOAL','FOLLOW_PATH_SENT','FOLLOW_PATH_GOAL','ROUTE_GOAL']),None)
 gates=[json.loads(l) for l in (out/'control/gate.jsonl').read_text().splitlines()];first_action=next((g['t'] for g in gates if g.get('mode')=='DRIVE' and abs(g.get('v',0))+abs(g.get('w',0))>.003),first_action)
 if name=='mowing_startup_perception_loss':first_action=min([e['t'] for e in events if e['event']=='START_WORK_REQUEST']+([first_action] if first_action is not None else []),default=None)
 coverage=bool(frames and frames[0]['sim_t']<=(first_action if first_action is not None else 15.) and frames[-1]['sim_t']>=task_result.get('t',1e9)-.5)
 active_frames=[f for f in frames if (first_action or 0)<=f['sim_t']<=task_result.get('t',0)]
 max_age=max((f['image_wall_age_s'] for f in active_frames),default=1e9);max_gap=max((b['sim_t']-a['sim_t'] for a,b in zip(active_frames,active_frames[1:])),default=1e9);quality=bool(len(active_frames)>=10 and max_age<=2.5 and max_gap<=1.0)
 receipt=dict(case=name,task=task,path=str(out.relative_to(ROOT)),task_passed=result['passed'],video_valid=video.get('valid',False),mission_timeline_complete=coverage,active_video_quality=quality,max_active_image_age_wall_s=max_age,max_active_frame_gap_sim_s=max_gap,passed=bool(result['passed'] and video.get('valid') and coverage and quality),task_terminal=task_result.get('terminal'),scope='Actual independent recording, retains full or selected original obligation count',sim_seconds=task_result.get('t'),video_seconds=video.get('duration_s'),source_video_sha256=sha(out/'video/full.mp4') if (out/'video/full.mp4').exists() else None,analysis_checks=result.get('checks',{}));dump(out/'recording_acceptance.json',receipt);dump(APP/'results/R12'/(name+'_latest.json'),receipt);return out,result,receipt
```

record 先 require R09，再检查派生源码摘要，按案例调用 runtime，并分别分析任务与视频。最终 passed 是任务、编码、时间线和图像质量交集，video_valid 不能单独升级任务。

<!-- generated-production-end -->

## 运行中按顺序查什么

先看时钟是否前进、定位与感知是否准入；再看目标/段号、BT action 与原因；随后看候选速度、门控、实测速度；工作时看工具健康、请求、有效反馈与账本；异常时看取消、重验证、等待期限和残余。

遇到 HUMAN 或安全停止，不用手动发布速度绕过门控。记录当前会话路径和原因，按状态检查定位/几何/反馈。演示中安全停止可以是一个有效行为片段，但不是全任务成功。

## 怎样停止

在任务终端按一次 Ctrl+C，等待合作退出和最终文件。不要连续强杀所有 Python/Gazebo 进程。任务收尾后再停 GUI 终端；查看 runtime、mission_result 和工具/停车字段。如果运行器需要强制结束某个自有进程，应该在结果中保留事实。

## 怎样找到最新结果

```bash
cd /home/ceihao/robot_project
python3 -m json.tool optimization_v2/three_tasks/results/R12/spray_latest.json
```

指针中的 path 是相对原工程根目录的实际运行目录，case、task、task_passed、video_valid 与 passed 分别说明范围与检查。不要只看文件更新时间。新运行可能更新 latest，历史证据仍有独立路径。

进入该目录后查看 `mission_result.json`、`mission_events.jsonl`、`runtime.json`、`recording_acceptance.json`。完整视频在相应 `video/full.mp4`，是否可作为完整演示还需时间线、图像新鲜度和任务证据检查。缩短视频应保留章节与倍速说明，不把失败剪掉后称完整通过。

## 离线练习

执行 `python3 examples/l02/labs.py 27`，程序用 AST 解析固定源码中的 CASES，不 import 运行器，不启动仿真。它读出上述范围，能帮助你核对文档是否与实际命令一致。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_27.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab27():
    """Read declared recording cases directly from frozen Python AST."""
    import ast
    p=ROOT/'baseline/source/optimization_v2/ws/src/task_platform/scripts/stage_r12.py'
    tree=ast.parse(p.read_text(encoding='utf-8'))
    cases={}
    def literal(node):
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='dict':
            return {k.arg:literal(k.value) for k in node.keywords}
        return ast.literal_eval(node)
    for stmt in tree.body:
        if isinstance(stmt,ast.Assign):
            target=stmt.targets[0]
            if isinstance(target,ast.Name) and target.id=='CASES':cases.update(literal(stmt.value))
            elif isinstance(target,ast.Subscript) and isinstance(target.value,ast.Name) and target.value.id=='CASES':
                cases[ast.literal_eval(target.slice)]=literal(stmt.value)
    assert cases['spray']['point_limit']==3 and cases['mowing']['segment_limit']==5
    return dict(kind='actual source, parsed not imported',cases=cases)
```

```text
{
  "chapter": "27",
  "passed": true,
  "result": {
    "kind": "actual source, parsed not imported",
    "cases": {
      "spray": {
        "task": "spray",
        "case": "tool_jam",
        "point_limit": 3,
        "seconds": 300
      },
      "spray_full_A1": {
        "task": "spray",
        "case": "nominal",
        "point_limit": 0,
        "seconds": 1800
      },
      "inspection": {
        "task": "inspection",
        "case": "nominal",
        "seconds": 480
      },
      "inspection_leader": {
        "task": "inspection",
        "case": "leader",
        "distance_limit": 20.0,
        "seconds": 150
      },
      "inspection_blocked": {
        "task": "inspection",
        "case": "blocked",
        "distance_limit": 20.0,
        "seconds": 120
      },
      "mowing": {
        "task": "mowing",
        "case": "nominal",
        "segment_limit": 5,
        "seconds": 420
      },
      "mowing_full_B1": {
        "task": "mowing",
        "case": "nominal",
        "segment_limit": 0,
        "seconds": 1800
      },
      "mowing_startup_perception_loss": {
        "task": "mowing",
        "case": "startup_perception_loss",
        "segment_limit": 5,
        "seconds": 420
      }
    }
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 故障与自测

启动即报会话锁：先确认是否已有自己的任务；GUI 没切到新任务：检查 active 会话与观察者；视频黑屏：查相机到达与编码器，不用任务成功替代媒体检查；退出码 2：读具体 receipt，而不是重复启动直到碰巧 0。

练习：独立完成一个短案例后写五项记录：命令、会话、选择范围、任务/停车/工具/视频分项结果、与固定历史样本的差异。能提供这五项才算可复现操作，而不是仅“看见机器人动了”。
