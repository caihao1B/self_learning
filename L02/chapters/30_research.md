# 第 30 章 从学习到修改项目

学完教材后，你应能独立提出小范围修改、解释影响、执行可检查实验并保留真实结论。研究的第一步不是增加最复杂模型，而是知道原系统为什么这样设计、哪里已经有证据、哪里仍需要确认。

## 先选一个有边界的问题

例如“点喷静止窗口在某类地标附近频繁拒绝”“割草末端恢复多次触发无进展”“GUI 无法方便比较 raw 与 filtered correction”。每个问题写出用户可见现象、相关模块、输入输出、预期改善和不可放宽条件。

不要同时改传感器、滤波、规划、控制和阈值，再用一段成功视频解释因果。更好的实验是先保持任务和物理约束，修改一个机制，比较同范围结果；需要连锁修改时明确依赖并逐层验证。

## 修改类型决定复验范围

|修改|直接影响|必要检查方向|
|---|---|---|
|文字或 GUI 布局|阅读与展示|链接、数据语义、浏览器|
|传感器外参|点云、定位、地形、工具几何|坐标、时间、资产与短任务|
|车宽/轴距/转角|运动学、足迹、路径、控制|几何扫掠、规划、实际跟踪|
|BT 优先级|选择与取消|条件组合、状态机、故障触发|
|账本规则|完成量与恢复|重复反馈、事务、残余与重启|
|模型/缓存|预测与实时性|质量、成本、超时回退、闭环|

当前基线不可被学习例子悄悄覆盖。教学脚本使用合成数值与临时目录；生产修改应在独立分支/工作目录进行，并保留原回执。用户未来选择允许偏差，也需要明确参数、版本和新的验证范围。

## Git 的最小工作流

以下命令适用于已经是 Git 仓库的独立开发目录，不要在不明目录盲目执行：

```bash
git status --short
git rev-parse HEAD
git switch -c study/one-localization-change
git diff
```

status 显示未提交变化；rev-parse 显示当前提交；switch -c 建立新分支；diff 查看具体修改。提交前只 add 本次明确文件，避免把运行数据库、视频、虚拟环境或凭据加入。不要使用 reset --hard 清理自己尚未理解的工作区。

学习仓库与机器人仓库分别管理。教材完成的提交不能替代机器人功能提交；原机器人更新后，也不应自动修改固定学习基线，而应建立新教材版本或差异章节。

## 实验的四级验证

第一级是输入与纯函数：非有限值、角度边界、时间倒退、工作键冲突。第二级是组件：实际库调用、消息类型、生命周期、取消和退出。第三级是短任务：选择能真正触发修改分支的场景，记录版本与证据。第四级是完整受影响任务与正式对照，确认局部改善没有牺牲其它环节。

测试不要只重复实现公式；应包含反例与失败分支。比如修改精停进展尺度，要验证合法短移动、无授权上下文、距离越界和超时仍生效；修改 START_WORK 取消，要实际在启动阶段触发而非行驶后才触发。

## 预算与统计

给每次实验设仿真和墙钟上限，明确是否允许 PARTIAL 结束。记录种子、任务范围、模型版本、硬件与资源占用。随机控制器或物理接触需要重复试验；三次都成功也只是有限样本，不能轻易声称普遍最优。

消融应移除一个明确机制并保持其它条件可比。例如比较 cache 与 full，必须同时统计特征/查询/维护/推理开销、任务完成、等待和安全回退。不能把更小任务或更慢车速混进对照却仍叫算法加速。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### safe_plan_r10.py 第 53 至 62 行

[离线带行号源码](../../baseline/l02_views/f1fb6bd2d929-safe_plan_r10.py.html#L53) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/safe_plan_r10.py#L53)

```python
def load_checked(path=None):
    path=path or edition('R10')/'mowing_safe_plan.json'; plan=read(path)
    revision=plan.pop('revision'); computed=canonical(plan);plan['revision']=revision
    if computed!=revision:raise ValueError('SAFE_PLAN_REVISION_CHANGED')
    source=edition('R09')/'mission.json'
    if plan['source_sha256']!=sha(source):raise ValueError('REFERENCE_PLAN_CHANGED')
    for field,p in [('backend_sha256',APP/'build/r10_safe_route/r10_safe_route'),('backend_source_sha256',PKG/'r10_backend/safe_route.cpp'),('generator_sha256',PKG/'scripts/prepare_safe_plan_r10.py'),('envelope_source_sha256',PKG/'scripts/planning_envelope_r10.py')]:
        if sha(p)!=plan[field]:raise ValueError('SAFE_PLAN_GENERATOR_CHANGED '+field)
    validate(plan,read(source))
    return plan
```

加载时重算计划 revision，再核对参考计划、后端二进制、后端源码、生成器与包络源码摘要。几何修改会使多个依赖失效，不能只改一个 JSON 的 revision。

### common.py 第 19 至 23 行

[离线带行号源码](../../baseline/l02_views/e9ad013f1962-common.py.html#L19) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/common.py#L19)

```python
def seal(stage,summary,files,sources,next_stage):
 out=APP/'results'/stage;out.mkdir(parents=True,exist_ok=True)
 artifacts={str(Path(p).relative_to(ROOT)):dict(path=str(Path(p).relative_to(ROOT)),sha256=sha(p)) for p in [*files,*sources] if Path(p).is_file()}
 result=dict(stage=stage,automated_status='PASS' if summary['passed'] else 'PARTIAL',summary=summary,artifacts=list(artifacts.values()),next_step=next_stage if summary['passed'] else None,formal_credit=False,original_S17='PARTIAL',original_S18='PARTIAL',created_utc=stamp())
 dump(out/'latest.json',result);return result
```

最终保存的 summary 与制品集合形成新的可追溯证据。next_step 由当前真实结论决定，原正式状态不会因学习或演示完成而自动升级。

<!-- generated-production-end -->

## 可以开展的后续学习项目

一是完善全局定位与精停证据，重点分析完整 A1 的拒绝窗口。二是检查安全 B1 末端与连接路状态机，保留原作业分母。三是独立机械臂/喷头项目实现相同 ToolTask 语义，再联调反馈和停止。四是用真实录制数据评估视觉与预测模型，先建立数据划分和预算，再考虑替换默认链路。

这些是研究方向，不代表教材已实现。当前 R10–R12 与原 S17/S18 的 PARTIAL 保持；离线教材的 PASS 只说明教材内容、链接和例子检查通过。

## 离线依赖闭包练习

执行 `python3 examples/l02/labs.py 30`。从 vehicle 修改沿依赖图找出 plan、sensor_extrinsics、controller、localization、short_trial 与 full_trial。队列遍历和 seen 集合避免重复访问；这是简单依赖传播模型，实际工程还要由 manifest 与代码调用确认。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_30.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab30():
    """Dependency closure identifies artifacts invalidated by a geometry change."""
    graph={'vehicle':['plan','sensor_extrinsics','controller'],
           'plan':['task_version'],'sensor_extrinsics':['localization'],
           'controller':['short_trial'],'localization':['short_trial'],
           'task_version':['short_trial'],'short_trial':['full_trial']}
    todo=deque(['vehicle']);affected=set()
    while todo:
        node=todo.popleft()
        if node in affected:continue
        affected.add(node);todo.extend(graph.get(node,[]))
    assert 'full_trial' in affected and 'sensor_extrinsics' in affected
    return dict(kind='teaching dependency graph',changed='vehicle',recheck=sorted(affected))
```

```text
{
  "chapter": "30",
  "passed": true,
  "result": {
    "kind": "teaching dependency graph",
    "changed": "vehicle",
    "recheck": [
      "controller",
      "full_trial",
      "localization",
      "plan",
      "sensor_extrinsics",
      "short_trial",
      "task_version",
      "vehicle"
    ]
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 写出你的第一份修改说明

用一页说明：具体问题；固定基线；准备修改的文件和机制；为什么预期有效；哪些约束不变；组件与短任务检查；完整任务范围；失败时怎样保留结果。最后列出能够推翻自己假设的观察条件。

自测：如果短案例 PASS，而完整任务仍 PARTIAL，如何写结论？答案是分别说明短例改善与完整任务未完成原因，不把其中一个覆盖另一个。若某分支在短例根本没有触发，不能写“该分支已验证”。

完成这一章意味着具备阅读和实验的方法，不意味着机器人所有历史目标已经达成。下一阶段 L03 将这些内容组织成更完整的学习站，提供导航、搜索、代码阅读和发布验收；教材内容与机器人验收仍分别管理。
