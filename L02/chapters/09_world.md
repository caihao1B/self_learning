# 第 09 章 三维世界与先验分层

三维仿真世界承担物理实验场，先验地图承担规划输入。两者相关但不相同：世界可以包含机器人尚未观察到的低障碍、运动目标和坡面细节；先验只暴露任务允许提前知道的部分。若把完整世界答案直接送给在线系统，感知价值便难以评估。

## 世界 工作区 通行区和任务簇

世界是 Gazebo 中的地面、模型、碰撞、灯光和传感器。工作区是需要产生有效作业的区域；通行区是车辆被允许经过的空间，可能包含道路和地头；簇是算法分配的工作义务集合。车辆可以经过不计工作量的连接区域，也可能为了转弯短暂离开工作条带但仍处于合法通行区。

因此“轨迹比作业区大”不一定错误。需要检查是否在授权地头内、是否满足车体扫掠与坡度、工具是否在连接段关闭、是否把非工作区域错误计入覆盖。若轨迹超出通行边界，则是几何或执行问题，不能用阿克曼本来就要大转弯解释一切。

## 不规则区域怎样表示

GeoJSON Polygon 使用一组闭合环表示外边界与洞。坐标单位取决于项目约定；本项目局部地图的数字不能直接当经纬度。多边形有洞时，路径与覆盖不能穿过洞；几何自交或环未闭合可能让面积和裁剪结果异常。

SDF 描述 Gazebo 世界、link、joint、visual、collision 和传感器。visual 决定看起来怎样，collision 决定物理接触。漂亮贴图不会自动改善碰撞几何；过于精细的碰撞网格反而增加计算成本。当前轻量结构与视觉外壳分离，适合在固定物理合同下改善展示。

## 哪些信息可提前知道

工作边界、道路、永久设施和粗地形可作为离线先验。临时放置物、行人、运动障碍、近地细节和当次观测质量需要实时获取。已知坡度可以用于排除明确不可行区域；不确定地形保留为待观测，而不是默认平地。

把未知保留为未知会降低完成率，但能暴露真实传感器覆盖和定位问题。为了演示把未知格直接清空，会使结果失去原先的解释力。合理改进是增加实际观测、调整采样或重新设计任务范围，并记录新版本。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### generate_scene.py 第 18 至 33 行

[离线带行号源码](../../baseline/l02_views/d71792794a9f-generate_scene.py.html#L18) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L18)

```python
def geometries(variant='base',targets=(1650,1140),width=6):
    vertices=[[(8,24),(28,18),(52,22),(60,38),(54,48),(57,66),(38,77),(15,70),(10,52),(19,42)],
              [(108,42),(125,34),(143,41),(152,57),(144,76),(127,80),(112,70),(117,57),(105,52)]]
    zones=[]
    for pts,area,c in zip(vertices,targets,[(34,47),(128,58)]):
        p=Polygon(pts);s=math.sqrt(area/p.area);p=scale(p,s,s,origin='centroid');p=translate(p,c[0]-p.centroid.x,c[1]-p.centroid.y)
        if variant=='notch':p=p.difference(box(c[0]-3,p.bounds[3]-8,c[0]+3,p.bounds[3]+1))
        if variant=='holes':p=p.difference(box(c[0]-11,c[1]+5,c[0]-5,c[1]+9))
        if not (p.is_valid and p.geom_type=='Polygon' and p.area>0):raise ValueError('invalid/disconnected workspace')
        zones.append(p)
    control=[[(49,42),(65,34),(71,32),(82,50)],[(82,50),(93,68),(101,70),(122,58)]]
    points=[]
    for part in control:
        p=np.array(part)
        for t in np.linspace(0,1,81):points.append(((1-t)**3*p[0]+3*(1-t)**2*t*p[1]+3*(1-t)*t*t*p[2]+t**3*p[3]).tolist())
    line=LineString(points);return zones,line,line.buffer(width/2,quad_segs=12)
```

Polygon 从不规则顶点构造，面积缩放使用平方根，因为面积随线性尺度平方变化；translate 放到指定中心。两个三次 Bézier 段生成连接中心线，再 buffer 成道路区域。此函数属于离线世界生成，不能当在线感知入口。

### generate_scene.py 第 58 至 64 行

[离线带行号源码](../../baseline/l02_views/d71792794a9f-generate_scene.py.html#L58) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/scenario_world/scripts/generate_scene.py#L58)

```python
def part(link,name,kind,data,pose,color,collision=True):
    for tag in ['visual','collision'] if collision else ['visual']:
        e=ET.SubElement(link,tag,name=name if tag=='collision' else name+'_visual');text(e,'pose',' '.join(map(str,pose)))
        geom(e,kind,data)
        if tag=='visual':
            m=ET.SubElement(e,'material');text(m,'ambient',color);text(m,'diffuse',color)
    return e
```

同一 geometry 可以分别写 visual 与 collision。collision=False 只改变可见外观，不影响物理接触；这正是理解贴图和碰撞模型分离的入口。

<!-- generated-production-end -->

## 从坡度到通行代价

平面 `z=ax+by+c` 的坡度大小可由 `atan(sqrt(a²+b²))` 计算，a、b 是高程变化与水平距离之比。低矮台阶的高度可能不大，却有陡直边缘；粗糙度高的区域也可能让轮速与姿态估计失真。因此代价不应只由绝对 z 决定。

当前安全路径讨论涉及 5° 通行限制与部分 10° 坡面。地图保持原样，新执行版本裁剪不适合的段，原工作区域分母不变。这种处理暴露残余，便于后续真实机器人或人工接管，而不是通过改分母让覆盖率看起来更高。

## 世界尺寸与三十分钟预算

粗估工作时间为 `T≈路径长度/有效速度 + 点停留 + 转弯换挡 + 预期等待 + 收尾储备`。条带长度可以用面积/有效幅宽近似，但不规则边界和障碍会增加连接成本。等面积簇未必等时间；分簇后还要重新计算路径并检查预算。

学习时先在小案例证明输入、决策、反馈和停止链路，再扩大任务。小案例必须明示选择范围，不能替代正式分母。连续三个簇允许形状不同，只要连通与工作量合同满足即可。

## 实际查看命令

```bash
cd /home/ceihao/self_learning
python3 -m json.tool baseline/l02_source/optimization_v2/maps/prior/workspaces.geojson
python3 examples/l02/labs.py 09
```

`python3 -m json.tool` 使用标准库模块格式化并检查 JSON 语法，不计算几何合法性。后者需要 Shapely 等几何工具进一步检查面积、洞、连通与自交。不能把 JSON 可解析当作地图可通行。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_09.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab09():
    """Cell ownership and occupancy are different layers."""
    cells=[dict(id=i,zone='A',permanent=(i==2),observed=None) for i in range(6)]
    cells[3]['observed']='low_obstacle'
    statuses={c['id']:('BLOCKED_PRIOR' if c['permanent'] else
                     'BLOCKED_OBSERVED' if c['observed'] else 'UNKNOWN_OBSERVATION') for c in cells}
    assert statuses[0]=='UNKNOWN_OBSERVATION' and statuses[3]=='BLOCKED_OBSERVED'
    return dict(kind='synthetic layer example',status=statuses,
                lesson='Permitted prior does not imply currently observed free.')
```

```text
{
  "chapter": "09",
  "passed": true,
  "result": {
    "kind": "synthetic layer example",
    "status": {
      "0": "UNKNOWN_OBSERVATION",
      "1": "UNKNOWN_OBSERVATION",
      "2": "BLOCKED_PRIOR",
      "3": "BLOCKED_OBSERVED",
      "4": "UNKNOWN_OBSERVATION",
      "5": "UNKNOWN_OBSERVATION"
    },
    "lesson": "Permitted prior does not imply currently observed free."
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 常见故障与自测

模型有外观无碰撞：核查 SDF collision；车辆漂浮或陷地：检查碰撞尺寸、惯量、接触参数和初始高度；地图整体偏移：检查地图原点与 world/map 转换；条带落入洞：检查多边形内部环和裁剪逻辑；坡面漏检：检查观测点数、坐标与地面模型。

问：能不能把实时检测的障碍永久写进先验？答：需要定义持续性、证据、版本和更新策略。一次传感器误报不应永久污染地图，动态障碍也不应自动变成永久设施。

练习：分别列出一个固定高障碍、低台阶、移动人和未观察区域在“世界、先验、实时感知、评估”四层的表示。四层完全相同时，应检查是否把真值意外暴露给控制链路。
