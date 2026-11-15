# 第 20 章 条带 地头与安全执行版本

割草覆盖需要同时解决“工具扫过哪些区域”和“车怎样连接条带”。Fields2Cover 产生条带，OMPL 等后端寻找满足运动学的连接，几何与实时风险检查决定能否执行。路径存在不等于已完成有效作业。

## 幅宽 间距与条带方向

工具有效幅宽决定单次理论覆盖宽度，条带间距通常留一定重叠以抵抗定位和跟踪误差。当前工具有效幅宽 1.6 m，R03 规划条带间距 1.40 m；名义重叠不是最终有效面积保证，还需要真实反馈与几何并集。

条带方向影响数量、长度和地头连接。当前后端用 Fields2Cover 角度枚举与条带数目标，生成整个工作区一致相位的条带，再按分簇区域裁剪。若每簇独立从不同起点排线，边界可能出现缝隙或过度重叠。

## 工具中心与车辆参考点

工具安装在车体某个偏移位置。希望刀盘中心沿某条线工作时，后轴轨迹需要扣除旋转后的工具偏移。直接把刀盘路线当 base_link 路线，会产生沿程偏移，尤其在条带端部造成漏作业或越界。

有效面积应来自工具开启、反馈有效时的工具扫掠并集，再与原目标区域相交。连接段即使经过草地，也不能仅凭几何给工作信用；工具关闭、反馈过期或不确定的部分保留残余。

## Dubins 与 Reeds–Shepp

Dubins 路径连接带方向的两个位姿，限制最小转弯半径且只允许前进。Reeds–Shepp 允许前进与倒车，可在狭小地头减少大圈，但会产生换挡尖点。尖点不是允许车体瞬间改变方向：执行时应分成单挡段，停车、停工具、确认换挡，再继续。

OMPL 提供状态空间与搜索基础；候选仍需检查曲率、端点姿态、足迹、坡面、长度和时间。一个数学曲线可行，但整车扫掠越界，依然不可执行。加速度、转向速率和接触动力学还要在实际运行验证。

## 原 B1 与安全版本

原 B1 有 34 个参考执行段，部分末端车身与 10° 坡面重叠，而当前允许坡度为 5°。用户批准生成新的安全执行路径，保留地图、原工作面积、原路径及 PARTIAL 记录。新版本有 47 个执行段，段数变化包含连接与拆分，不能直接解释为工作面积变大。

`safe_plan_r10.validate()` 检查原工作多边形和面积分母未变、预算未变、通行权限没有扩大、包络一致、连接连续、换挡明确、工作键与原义务一致。连接段不得打开工具或获得作业信用。这是“改变执行方式、保留任务事实”的具体实现。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### planner_backend.cpp 第 20 至 32 行

[离线带行号源码](../../baseline/l02_views/1db7fcd0fc52-planner_backend.cpp.html#L20) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L20)

```cpp
  f2c::sg::BruteForce generator;generator.setStepAngle(M_PI/12);f2c::obj::NSwath objective;
  double angle=j.contains("angle")?j.at("angle").get<double>():generator.computeBestAngle(objective,width,c);
  f2c::decomp::BoustrophedonDecomp decomp;decomp.setSplitAngle(angle);auto cells=decomp.decompose(F2CCells(c));
  // One global line phase over the entire work zone avoids seams at cell/cluster boundaries.
  auto lines=generator.generateSwaths(angle,width,c);f2c::rp::BoustrophedonOrder order;auto sorted=order.genSortedSwaths(lines);
  json rows=json::array();for(const auto& s:sorted){auto a=s.startPoint(),b=s.endPoint();rows.push_back({{"id",s.getId()},{"a",{a.getX(),a.getY()}},{"b",{b.getX(),b.getY()}},{"length",s.length()}});}
  return {{"angle",angle},{"swaths",rows},{"inner_work",geo(inner)},{"inner_motion",geo(inner_motion)},{"decomposition",geo(cells)},{"library","Fields2Cover"},{"whole_zone_headland",true}};
}
json turns(const json& j){
  double radius=j.at("radius"),ds=j.value("ds",.05);need(radius>=2.11&&ds>0&&ds<=.1,"TURN_LIMITS");
  F2CRobot robot(1.04,.9);robot.setMinTurningRadius(radius);robot.setMaxDiffCurv(.25);robot.setCruiseVel(.12);robot.setTurnVel(.12);
  f2c::pp::DubinsCurvesCC cc;cc.setDiscretization(ds);cc.setUsingCache(false);f2c::pp::DubinsCurves simple;simple.setDiscretization(ds);simple.setUsingCache(false);json out=json::array();
  need(j.at("queries").size()<=300,"QUERY_LIMIT");
```

Fields2Cover 先构造工作与通行几何，枚举方向并用整区同相位生成 swaths，再排序。decomposition 用于描述几何分解，不意味着每个簇独立生成不同相位。

### safe_plan_r10.py 第 12 至 50 行

[离线带行号源码](../../baseline/l02_views/f1fb6bd2d929-safe_plan_r10.py.html#L12) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/safe_plan_r10.py#L12)

```python
def validate(plan, original):
    old=original['clusters'][0]; new=plan['cluster']; segments=new['segments']
    old_work=[s for s in old['segments'] if s['kind']=='COVERAGE']
    original_by_id={s['id']:s for s in old_work}
    old_union=union_all([Polygon(s['target_area']) for s in old_work])
    retained=union_all([Polygon(s['target_area']) for s in new['original_work_segments']])
    if new['original_work_segments']!=old_work or old_union.symmetric_difference(retained).area>1e-8:
        raise ValueError('ORIGINAL_WORK_POLYGONS_CHANGED')
    if new['work_area_m2']!=old['work_area_m2'] or new['area_map']!=old['area_map']:
        raise ValueError('ORIGINAL_CLUSTER_AREA_CHANGED')
    if new['budget_s']!=old['budget_s'] or new['estimated_seconds']>old['budget_s']:
        raise ValueError('ORIGINAL_TIME_BUDGET_CHANGED_OR_EXCEEDED')
    prior=shape(plan['navigation_prior']); allowed=prior.buffer(-original['footprint_margin_m'])
    if prior.difference(shape(original['allowed_prior'])).area>1e-8:
        raise ValueError('NAVIGATION_AUTHORITY_EXPANDED')
    nav=yaml.safe_load((edition('R09')/'nav2.yaml').read_text())
    maps=[nav[k][k]['ros__parameters'] for k in ['local_costmap','global_costmap']]
    expected,policy=make_envelope(original['vehicle']['footprint'],max(c['footprint_padding'] for c in maps),max(c['resolution'] for c in maps))
    if plan.get('planning_envelope')!=expected or plan.get('planning_clearance')!=policy:
        raise ValueError('GRID_AND_TRACKING_ENVELOPE_CHANGED')
    cfg=copy.deepcopy(original);cfg['allowed_prior']=plan['navigation_prior']
    previous=None
    for s in segments:
        if previous and (math.dist(previous[:2],s['poses'][0][:2])>1e-6 or yaw_error(previous[2],s['poses'][0][2])>1e-6):
            raise ValueError('DISCONNECTED_EXECUTION_PATH')
        valid,why=validate_candidate(s['poses'],s['poses'][0],s['poses'][-1],cfg,60.)
        if not valid:raise ValueError(why)
        runs=gear_runs(s['poses'])
        if len(runs)!=1 or runs[0]['direction']!=s['direction']:raise ValueError('UNDECLARED_GEAR_CHANGE')
        for p,w in zip(s['poses'],s['poses_world']):
            if math.dist([p[0]+28,p[1]+14,p[2]],w)>1e-7:raise ValueError('FRAME_MISMATCH')
            if not allowed.covers(body_polygon(plan['planning_envelope'],*w)):raise ValueError('PLANNING_ENVELOPE_OUTSIDE_PRIOR')
        if s['kind']=='COVERAGE':
            source=original_by_id[s['work_key']]
            if s['target_area']!=source['target_area'] or s['requested_area_m2']!=source['requested_area_m2'] or s['work_target_pose']!=source['poses'][0]:
                raise ValueError('FEEDBACK_OBLIGATION_CHANGED')
        elif s['tool_on'] or s['target_area'] or s['requested_area_m2']!=0:raise ValueError('CONNECTOR_HAS_WORK_CREDIT')
        previous=s['poses'][-1]
    return dict(passed=True,execution_segments=len(segments),reference_original_segments=len(old['segments']),original_reference_status='PARTIAL',original_work_target_union_m2=old_union.area,original_cluster_area_m2=old['work_area_m2'],original_work_union_unchanged=True)
```

校验原作业多边形与面积、时间预算和通行权限，再检查整条执行路径连续、单挡、曲率与包络。连接段不能带工具或面积信用，工作段保持原 work_key 与 target。

<!-- generated-production-end -->

## 为什么仍可能大圈或找不到路

车体宽、前伸大、半径约 3 m、坡面禁区与狭窄地头共同压缩可行空间。某些末端姿态需要大圈或倒车才能到达。控制器参数不能让几何上不合法的路径合法；反复重规划也不保证改善。

如果搜索在预算内找不到合格连接，应记录搜索条件、拒绝原因和未完成区域，停车请求人工。这个结论是有界搜索失败，不能声称已经证明任何路径都不存在。

## 面积并集练习

执行 `python3 examples/l02/labs.py 20`。三个平行教学区间存在重叠，合并后长度为 10 m，乘 1.4 m 幅宽为 14 m²，原义务 100 m²，残余 86 m²。若直接把区间长度相加会重复计数。真实系统使用二维几何而不是这段一维简化。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_20.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab20():
    """Tool sweep uses unions; shortening a route does not shrink obligation."""
    original=100.; intervals=[(0.,5.),(4.,8.),(10.,12.)]
    merged=[]
    for start,end in sorted(intervals):
        if merged and start<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],end)
        else:merged.append([start,end])
    length=sum(b-a for a,b in merged);width=1.4;area=length*width
    assert length==10
    return dict(kind='parallel-strip teaching union, not polygon solver',confirmed_area_m2=area,
                original_area_m2=original,remaining_m2=original-area,coverage_fraction=area/original)
```

```text
{
  "chapter": "20",
  "passed": true,
  "result": {
    "kind": "parallel-strip teaching union, not polygon solver",
    "confirmed_area_m2": 14.0,
    "original_area_m2": 100.0,
    "remaining_m2": 86.0,
    "coverage_fraction": 0.14
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 操作核查与常见故障

读当前 safe plan 时，先看 reference_original_segments、原工作面积、execution_segments、版本与残余，再看轨迹图。大圈应叠加通行边界、坡面与车体足迹一起判断。只画后轴轨迹会低估扫掠。

漏割边界：查工具偏移、条带间距与裁剪；倒车轨迹抖动：查换挡段和航向；路径接缝瞬转：查位置/姿态连续性；面积异常增加：查重叠并集、连接段工具状态和工作键；末端反复绕圈：查可达性、姿态容差与局部进展判据。

练习：画出一条工作条带和两条工具关闭连接，注明哪一部分进入有效面积。然后解释为什么允许残余不等于允许减少原分母。
