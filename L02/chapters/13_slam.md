# 第 13 章 SLAM 建图 回环与冻结地图

SLAM 同时估计运动和地图；里程计只解决局部连续运动；重定位在已有地图中重新找位置。这三个概念相关但不能互换。项目虽然使用 RTAB-Map 组件，当前全局定位主链路仍包含区域观测地图匹配，不能把所有环节都统称为“RTAB 已全部解决”。

## 位姿图怎样表示历史

位姿图的节点表示关键时刻位姿，边表示相邻运动或回环约束。每条边有测量和不确定性。优化让整个图尽量满足这些约束，而不是只把最后一个点硬拉回起点。

当机器人绕一圈回来，里程计可能累计漂移。回环检测先提出“这里可能见过”，几何验证确认约束是否可靠，再加入图优化。重复农田结构可能造成错误相似，错误回环会把整张图扭曲，因此外观相似不是充分条件。

## 一个可手算的回环

一维四段增量为 1.02、1.03、−0.98、−0.97 m，总和 0.10 m。若假设起终点相同且四段同等可信，最小二乘修正可把 0.10 m 平均分到四条边，每条减 0.025 m，使总和回到零。真实 SE(3) 位姿图有旋转、不同协方差和鲁棒核，不能照搬平均减法，但这个例子解释了“全图调整”的思想。

## RTAB 数据库保存什么

建图数据库可以保存关键帧、描述、传感器数据、图关系和优化结果。SQLite 文件存在不代表内容完整；需要数据库完整性、节点/边、观测来源、导出版本和正常关闭记录。直接复制仍在写入的数据库也可能得到不一致快照。

`mapping_export.py` 在正常停止 RTAB 后，以只读模式检查数据库，调用 graph_probe 和 rtabmap-export 导出观测云及位姿，再验证源数据库摘要未变。地图 manifest 绑定数据库、点云、场景、传感器与采集输入，说明地图来自实际模拟观测而非直接转换 SDF 真值。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### mapping_export.py 第 22 至 38 行

[离线带行号源码](../../baseline/l02_views/ef62d587b8b7-mapping_export.py.html#L22) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/map_manager/scripts/mapping_export.py#L22)

```python
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
```

只读 SQLite integrity_check 先检查数据库；graph_probe 与 rtabmap-export 使用外部真实工具。导出前后数据库摘要必须一致，manifest 把观察来源与所有文件绑定。export 不是把 SDF 世界几何直接当作 SLAM 输出。

<!-- generated-production-end -->

## 冻结为什么必要

规划与定位必须知道自己使用哪一版地图。冻结包含 map_id、revision、frame_epoch、数据库哈希和相关资产。文件名 `map.db` 本身不能唯一识别地图；把另一个数据库覆盖到同名文件，会破坏先前路径与定位证据。

建图模式允许新增信息和改变图，定位模式通常希望使用固定地图。是否真正冻结应看运行参数、写权限和前后摘要，而不是只看界面按钮。教材保留地图 manifest，但体积较大的本地数据库与参考云不一定随 GitHub 提供；新机器复现需要按资源清单迁移。

## 当前全局定位的边界

项目历史中尝试了跨车高的 RTAB 原生全局重定位，但未形成所有条件都通过的结果。当前三任务使用 RTAB 局部 ICP、轮速/IMU 融合、冻结观测云的区域配准和 GNSS 独立监督。区域匹配还有初值与可观测性要求，不应宣传为任何位置都能无先验瞬间定位。

“使用相同视觉方案”在工程上需要分清：共用数据与接口、共用局部估计、共用全局地图，分别有不同难度。高低不同车辆看到的结构不同，传感器外参和遮挡也不同，不能只复制一份配置。

## 操作范围

阅读地图冻结信息：

```bash
cd /home/ceihao/self_learning
python3 -m json.tool baseline/l02_source/optimization_v2/maps/frozen/manifest.json
python3 examples/l02/labs.py 13
```

学习仓库中的 manifest 用于理解引用和版本，不保证其指向的大资源都存在于学习目录。不要为通过一个存在检查创建空 `map.db`。真正建图需要机器人运行环境、原采集流程和足够磁盘，不能用本章的图优化小例子代替。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_13.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab13():
    """One-dimensional loop closure distributed through a four-edge pose graph."""
    odom_edges=[1.02,1.03,-.98,-.97]; drift=sum(odom_edges)
    correction=drift/len(odom_edges); optimized=[x-correction for x in odom_edges]
    assert abs(sum(optimized))<1e-12
    return dict(kind='equal-weight 1D teaching graph',before_closure_m=drift,
                optimized_edges_m=optimized,after_closure_m=sum(optimized))
```

```text
{
  "chapter": "13",
  "passed": true,
  "result": {
    "kind": "equal-weight 1D teaching graph",
    "before_closure_m": 0.10000000000000009,
    "optimized_edges_m": [
      0.995,
      1.005,
      -1.005,
      -0.995
    ],
    "after_closure_m": 0.0
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 常见失败

重启后找不到地图：先查真实路径与资源清单；地图能打开但无法定位：查传感器高度、外参、初值、可见结构与地图版本；回环后跳变：查边的几何验证和原始残差；地图越来越大：查是否误处建图模式及是否持续写数据库；读出不完整：检查进程是否正常关闭及 SQLite 状态。

问：回环次数多是否越好？答：错误回环比没有回环更危险；还需看约束质量和最终误差。问：冻结后地图永远不能更新吗？答：可以建立新版本，重新绑定定位与路径，再验证迁移影响。

练习：画出数据库、观测云、地图元数据、导航栅格、离线计划之间的依赖。改变地图原点时，圈出哪些资产需要重建，不能只改可视化配置。
