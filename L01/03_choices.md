# 技术选择与适用边界

选择技术时先问它解决哪一个问题，再看当前输入和计算条件是否匹配。功能越复杂不一定越有效；本项目使用真实对照结果决定是否启用预测与缓存，也用物理约束限制路径和控制。

## 当前主链路和候选方案

|问题|采用的技术及位置|选择理由|代价或容易失效处|其他方案与当前状态|
|---|---|---|---|---|
|物理与传感器仿真|Gazebo Harmonic、ROS 2 Jazzy、SDF/插件|关节、碰撞、传感器与 ROS 通信可以组成闭环|渲染和大点云消耗算力；物理简化影响外推|Blender 可用于资产制作，当前运行世界在 Gazebo|
|车体运动学|差速与前阿克曼独立几何合同|让路径遵守真实车辆的转弯和倒车条件|车宽、基准点和工具包络都会影响可通行性|不以视觉车型名称推断实际硬件参数|
|局部运动估计|RTAB 原生 ICP 里程计、轮速/IMU、robot_localization EKF|多种观测互补，保持局部连续 odom|结构不足、动态物体、轮滑和时间错位|纯轮速易累积漂移；纯 ICP 也可能给出错误运动|
|全局地图定位|冻结观测云区域匹配、留出几何验证、GNSS 检验|在既有区域资产上获得可检查的地图修正|初值、结构可观测性、全局修正抖动|跨车高 RTAB 原生全局视觉重定位仍有失败记录|
|定位偏移监督|独立 GNSS 创新、时间与杆臂补偿、跳变锁存|让定位结果还需要外部一致性证据|GNSS 拒绝可能阻止恢复；静止单天线航向不足|不能把被检验的信息同时作为唯一检验依据|
|三维地形到二维驾驶|点云地面统计、坡度/阶差/粗糙度、足迹与代价图|地面车辆仍可用二维导航，同时保留立体风险|自车噪点、近地盲区、未知格和配准误差|直接高度阈值便宜但不能充分表达坡面与负障碍|
|工作量分区|METIS 图划分及连通性、负载检查|在连续不规则区域上构造三个可执行子问题|图权重不等于实际任务工时，需要后续路径反算|几何等面积切分不保证工作量或连通性|
|点喷访问顺序|OR-Tools 等点序优化与合法连接|点任务接近 TSP；先后顺序影响路程|欧氏距离近不代表车辆实际容易到达|每点的实际执行采用 NavigateToPose 与工具动作|
|割草条带|Fields2Cover、受约束地头与 OMPL 连接|条带生成与非完整车辆连接分开处理|整车扫掠、坡面、倒车和工具关闭都要检查|完整路径长度之外还需计算有效覆盖与残余|
|导航到点|Nav2 Smac 搜索及精确末端适配|使用成熟全局规划，再处理格心与服务点差异|成功到达导航容差不等于满足喷洒准入|点喷必要时增加局部 odom 精停|
|沿路径控制|当前任务的 RPP 跟踪与车辆门控|解释直观，适合已有路径和明确运动学约束|末端曲率、短路径和定位修正会影响跟踪|MPPI/DWB 配置和阶段实验保留；不能宣称每个当前任务都同时使用|
|行为决策|BehaviorTree.CPP、原因码、候选资格和有界恢复|高优先级条件可反复检查，日志能解释选择|异步取消和任务状态可能不同步|BT 不是单独的最优规划器，也不证明全局无路|
|工具与完成量|ROS ToolTask、C++ 状态机、SQLite 账本、幂等键|将请求、物理反馈、确认量和残余分开|停止不确定、反馈过期、几何异常|真实机械臂和切割机构留作独立项目|
|动态交互|当前巡检的观测几何跟踪、路径投影、间距/TTC 判据|能区分同向跟随、横穿等待和持续受阻|目标关联和可观测性会限制判断|S15 学习型/IMM 候选未获当时默认启用资格|
|经验缓存|历史 S16 对照保留，默认关闭|检索和维护也有成本，需比较净收益|低命中、错误相似和场景分布变化|实验完成不代表缓存有收益|
|监视学习|Python 只读服务、ROS 订阅、HTML/JS|与控制解耦，可同时看数据和决策|历史页只有保存范围，不能凭空回放所有传感器|实时 GUI 与本学习站分别承担运行监视和教学|

这张表由固定版本代码、[R05 说明][localization-doc]、[R08 说明][inspection-doc]、[R09 说明][mowing-doc]、[预测][prediction]及[缓存][cache]实验支持。每一行在后续教材中会展开为输入输出、公式、代码、演示和失败边界。

## 语言和库如何配合

Python 经常位于任务编排和数据处理层，C++ 经常位于仿真插件、原生库与监督组件。这是当前代码的职责分配，不能简单理解为 Python 一定慢或 C++ 一定实时。消息频率、算法复杂度、内存分配、队列和进程调度都会影响延迟。

|语言或文件|本项目可直接阅读的入口|应学习的概念|
|---|---|---|
|Bash|[advance.sh][entry]、[env.sh][env]|当前目录、参数传递、环境继承、source、exec、退出码|
|Python 标准库|[dispatch.py][dispatch]、[runtime][runtime]|argparse、pathlib、json、importlib、subprocess、time、signal|
|Python ROS|[mission_spray_r10.py][mission]|节点、回调、Future、ActionClient、消息与异常收尾|
|Python 数值与几何|局部定位、感知、账本模块|NumPy/SciPy、坐标变换、Shapely 交并与 SQLite 事务|
|C++|[精停进度][precision]、[工具服务][tool-server]、[BT][bt]|const 引用、命名空间、模板取值、智能指针、状态与生命周期|
|CMake 与包清单|[CMakeLists.txt][cmake]|编译标准、头文件、共享库、链接、插件安装位置|
|YAML/JSON/XML/SDF|任务参数、BT 和场景|配置不是算法本身；加载者决定其实际含义|
|HTML/JS|[GUI 页面][gui-page]|HTTP 数据、DOM、Canvas、刷新与历史状态分离|

## 为什么这些库不能全部用一个安装命令概括

ROS 发行版包、系统开发库、Python 虚拟环境包和项目源码编译物属于不同层次。`apt` 安装某个 C++ 开发包，不等于任意 Python 解释器都能 `import` 它；`pip` 安装一个 Python 包，也不会自动生成 Gazebo 能加载的系统插件。

教材的安装章节将按“操作系统 → ROS/Gazebo → 系统库 → Python 环境 → 工作区构建 → 运行资源”排列。先理解依赖边界，才能解释 `ModuleNotFoundError`、找不到共享库和插件版本不匹配为什么需要不同解决方法。

<!-- generated fixed-revision references -->
[env]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/env.sh
[entry]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/three_tasks/advance.sh
[dispatch]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/dispatch.py
[common]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/common.py
[stage12]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/stage_r12.py
[derive]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/prepare_spray_video_r12.py
[runtime]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/runtime_spray_r10.py
[mission]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/mission_spray_r10.py
[control]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/control_spray_r10.py
[docking]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/docking_r10.py
[precision]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/r10_backend/precision_progress.hpp
[precision-builder]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/precision_supervisor_r10.py
[bt]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/r08_backend/policy.hpp
[tool-server]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/tool_sim/src/action_server.cpp
[tool-build]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/tool_build_r09.py
[tool-action]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/mission_interfaces/action/ToolTask.action
[cmake]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/CMakeLists.txt
[gui-page]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/three_tasks/monitor/index.html
[gui-data]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/three_tasks/monitor/data.py
[gui-observer]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/three_tasks/monitor/ros_observer.py
[point-result]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R12/spray/mission_result.json
[point-events]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R12/spray/mission_events.jsonl
[fault]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R12/spray/fault_injection.json
[recording]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R12/spray/recording_acceptance.json
[full-a1]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R10/bounded_A1_result.json
[full-b1]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R10/bounded_B1_result.json
[attempts]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/R10/repair_attempts.json
[historical-plan]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/optimization_v2/命令Prompt合集.md
[r-plan]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/three_tasks/实施步骤.md
[localization-doc]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/three_tasks/R05/TECHNICAL.md
[inspection-doc]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/three_tasks/R08/TECHNICAL.md
[mowing-doc]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/three_tasks/R09/TECHNICAL.md
[repair-doc]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/three_tasks/R10/GUI_AND_BOUNDED_REPAIR.md
[prediction]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/optimization_v2/S15/RESULTS.md
[cache]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/optimization_v2/S16/RESULTS.md
[resources-doc]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/docs/three_tasks/R12/LOCAL_REPRODUCTION.md
[resources]: https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/validation/GUI/local_resource_inventory.json
