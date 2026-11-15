# 第 19 章 点访问序列与工时

点喷的离线问题接近旅行商问题 TSP：给定服务点，选择访问顺序使代价较小。但机器人不能穿墙，实际连接距离与欧氏距离不同，导航成功也不代表该点工具工作完成。

## 点序与连接两层问题

先在允许通行区域中计算点间合法连接代价，再优化访问顺序。若直接用直线距离，两个点看起来很近，中间却可能隔着永久障碍，最终序列会很差。当前 `visibility()` 把服务点与安全多边形顶点组成可见图，并用 Dijkstra 得到点间距离及折线路径。

安全区域由足迹和边距缩进得到；差速原地转向也需要检查整车旋转扫掠。中心点能到不代表原地旋转时四角不碰障碍。保守外接半径会排除一些实际上可行的姿态，但容易建立清楚的安全几何合同。

## TSP 的计算难点

n 个点的排列随 n 阶乘增长，不能对几十上百点直接穷举。最近邻先选当前最近点，速度快但易陷入差序列；2-opt 通过交换两条边减少绕行；OR-Tools RoutingModel 使用启发式与局部搜索，在预算内给出较好解。

当前后端设置 PATH_CHEAPEST_ARC、GREEDY_DESCENT、两秒搜索上限与解数量限制，明确 `optimality_claimed=false`。返回一个序列不代表已证明全局最优。Python 将其与 nearest-neighbor+2-opt 候选按同一代价比较，再保留更小者。

## 整数代价和节点索引

OR-Tools 路由内部索引不是原始点编号，需要 `RoutingIndexManager.IndexToNode` 转换。代价通常用 int64，当前把米乘 1000 后取整相当于毫米尺度。精度太粗会让不同路径变成相同代价；数值太大则增加溢出风险。

闭合 tour 包含最后一点回第一点的边，开路线则不同。阅读报告必须明确终点规则。实际任务还可能从独立入口接入，不能只优化点集环路就忽略起终段。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### planning_r03.py 第 31 至 55 行

[离线带行号源码](../../baseline/l02_views/a2602a00134c-planning_r03.py.html#L31) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/planning_r03.py#L31)

```python
def visibility(nodes,safe):
 corners=[]
 for poly in polygons(safe):
  for ring in [poly.exterior,*poly.interiors]:corners.extend(list(ring.coords)[:-1])
 xy=np.asarray([*nodes,*corners]);adj=[[] for _ in xy];allowed=safe.buffer(1e-8)
 for i in range(len(xy)):
  js=np.arange(i+1,len(xy))
  if not len(js):continue
  lines=shapely.linestrings(np.stack([np.tile(xy[i],(len(js),1)),xy[js]],axis=1));mask=shapely.covers(allowed,lines)
  for j in js[mask]:d=float(np.linalg.norm(xy[i]-xy[j]));adj[i].append((int(j),d));adj[j].append((i,d))
 costs=[];paths={}
 for i in range(len(nodes)):
  dist=[math.inf]*len(xy);dist[i]=0;prev={};q=[(0.,i)]
  while q:
   d,u=heapq.heappop(q)
   if d>dist[u]+1e-10:continue
   for v,w in adj[u]:
    if d+w<dist[v]-1e-10:dist[v]=d+w;prev[v]=u;heapq.heappush(q,(d+w,v))
  if not all(math.isfinite(x) for x in dist[:len(nodes)]):raise ValueError('DISCONNECTED_SERVICE_PATH')
  costs.append(dist[:len(nodes)])
  for j in range(len(nodes)):
   ids=[j]
   while ids[-1]!=i:ids.append(prev[ids[-1]])
   paths[i,j]=xy[list(reversed(ids))].tolist()
 return np.asarray(costs),paths
```

可见图用合法区域 covers 判断整条线，随后对每个服务节点运行 Dijkstra。heapq 保存当前最短候选，prev 反向恢复点间路径。所有服务点相互可达才返回代价矩阵。

### planner_backend.cpp 第 44 至 53 行

[离线带行号源码](../../baseline/l02_views/1db7fcd0fc52-planner_backend.cpp.html#L44) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/coverage_planner/src/planner_backend.cpp#L44)

```cpp
json tsp(const json& j){
  using namespace operations_research;auto cost=j.at("cost").get<std::vector<std::vector<int64_t>>>();int n=cost.size();need(n>=2&&n<=500,"TSP_SIZE");
  for(const auto& row:cost){need(row.size()==static_cast<size_t>(n),"MATRIX_SHAPE");for(auto v:row)need(v>=0&&v<1000000000000LL,"MATRIX_VALUE");}
  RoutingIndexManager manager(n,1,RoutingIndexManager::NodeIndex(0));RoutingModel routing(manager);
  const int cb=routing.RegisterTransitCallback([&](int64_t i,int64_t k){return cost[manager.IndexToNode(i).value()][manager.IndexToNode(k).value()];});routing.SetArcCostEvaluatorOfAllVehicles(cb);
  auto options=DefaultRoutingSearchParameters();options.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);options.set_local_search_metaheuristic(LocalSearchMetaheuristic::GREEDY_DESCENT);
  options.mutable_time_limit()->set_seconds(2);options.set_solution_limit(1000);const Assignment* solution=routing.SolveWithParameters(options);need(solution!=nullptr,"TSP_NO_SOLUTION");
  json order=json::array();for(auto i=routing.Start(0);!routing.IsEnd(i);i=solution->Value(routing.NextVar(i)))order.push_back(manager.IndexToNode(i).value());
  return {{"order",order},{"cost",solution->ObjectiveValue()},{"optimality_claimed",false},{"solver","OR-Tools RoutingModel"}};
}
```

RoutingIndexManager 负责内部索引映射，回调返回整数代价，时间上限限制搜索。optimality_claimed=false 明确不把启发式解当全局证明。

<!-- generated-production-end -->

## 工时不是距离除速度就结束

总时间可拆成移动、转向、精停、工具停留、取消恢复、动态等待与收尾。点密集时每点停留占比很高，提高巡航速度带来的收益有限。假设 56 点每点至少 3 s，仅工具请求基准就有 168 s，还不含定位窗口与连接。

离线估计用于分簇和预算，在线账本用于真实确认。原完整 A1 28/56 的保存结果不能由离线计划中 56 个点都出现来补足。任务失败后续跑应使用相同原工作键与残余量，而不是重新向已完成点喷满剂量。

## 执行为什么主要用 NavigateToPose

每个点是一个服务位姿，在线存在动态代价与定位变化，因此请求 Nav2 导航到点更自然。离线点序提供访问顺序和合法连接参考，Nav2 根据当前环境执行接近。末端仍可能需要局部 FollowPath 精停，之后满足静止观察窗口才开始工具。

过分锁死离线连接会削弱动态避障；完全忽略离线顺序又会丢失覆盖效率与预算。因此上层保留工作义务，执行层允许受约束调整连接，调整不能改变原服务身份。

## 离线练习

执行 `python3 examples/l02/labs.py 19`。四点矩阵允许穷举全部以 0 开始的序列，最短闭环长度为 10 教学米；0.5 m/s 移动需 20 s，再加四点各 3 s 服务，最小示例时间为 32 s，尚未计转弯与等待。这个精确小解可用于理解代价，不能宣称生产 OR-Tools 的大实例也是精确最优。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_19.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab19():
    """Tiny exhaustive TSP checks a closed tour and separates work time."""
    cost=[[0,2,9,3],[2,0,3,8],[9,3,0,2],[3,8,2,0]]
    def length(order):return sum(cost[a][b] for a,b in zip(order,order[1:]+order[:1]))
    routes=[(0,*p) for p in itertools.permutations([1,2,3])]
    best=min(routes,key=length);distance=length(best)
    assert distance==10
    return dict(kind='four-node exact teaching solver; production OR-Tools heuristic',
                tour=best,distance_m=distance,travel_s=distance/.5,
                service_s=4*3,total_lower_bound_s=distance/.5+4*3)
```

```text
{
  "chapter": "19",
  "passed": true,
  "result": {
    "kind": "four-node exact teaching solver; production OR-Tools heuristic",
    "tour": [
      0,
      1,
      2,
      3
    ],
    "distance_m": 10,
    "travel_s": 20.0,
    "service_s": 12,
    "total_lower_bound_s": 32.0
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 故障与自测

点不可达：检查足迹缩进和服务中心位置；序列跨障碍：检查代价是否来自可见图；耗时远超估计：分解导航、精停、工具和恢复事件；漏点：检查索引映射、原义务集合与完成集合；到点后工具拒绝：看位置、停止和工具准入，不能把工具拒绝改为导航成功。

练习：把某两点的代价增大，观察最优顺序变化；再把每点服务时间从 3 s 增至 10 s，观察路线不变但总工时显著增加。说明分簇权重为何需要服务成本。
