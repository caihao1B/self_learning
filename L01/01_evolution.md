# 需求演变与设计理由

项目从农田机器人仿真出发，逐渐形成三种作业、共用环境和感知定位基础、独立任务与反馈核算的结构。理解这条路线，比记住目录中的 S 编号和 R 编号更重要。

## 原始问题与迁移

用户最初在 Windows 虚拟机中的 Ubuntu 24.04 上完成项目，随后改用双系统里的原生 Linux，主要动机是运行速度和图形算力约束。这是用户提供的立项背景，不是本教材测得的虚拟机性能基准。

迁移首先要解决“原项目依赖什么”：模型、世界、插件、地图、Python 环境、编译产物和绝对路径都可能来自不同目录。单纯复制源码不会搬走系统依赖；删除旧目录前必须检查引用。当前活跃工程集中在主文件夹，旧入口与后来隔离的 `optimization_v2` 工作区分别保留。

这段经历形成第一条设计原则：**运行基线先闭合，再扩展算法**。否则错误可能来自缺文件或加载了旧插件，调算法参数会掩盖问题。

## 从简单场景到可观测的复杂环境

最初的优化要求包含三维环境、低矮和高大障碍、移动目标，以及多传感器。随后需求进一步明确为两个连续、不规则工作区，中间由曲线道路连接；工作区由算法划分三个连通簇，避免事先画好三个互不相连的小场地来代替大地图问题。

复杂度的关键不是模型数量，而是**哪些信息提前知道、哪些信息必须现场观测**。永久边界、基础地形和部分固定障碍可成为规划先验；动态目标和当前通行情况需要实时感知。地图上一块空白不能自动变成已经观测到的自由空间。

设计依据在[原 S 任务计划][historical-plan]；当前几何实现与源码导航对应于世界、车辆、分簇及路径模块。30 分钟限制首先是单簇仿真时间预算，不能换算成整套正式测试只需 30 分钟现实时间。

## 两种作业为什么需要不同路径语义

点喷需要到达一系列目标、停车、执行动作并核对反馈，访问顺序与旅行商问题相关。条带割草需要连续通过作业区域，考虑幅宽、地头、转弯半径、倒车和工具启停。两者都叫覆盖，但“覆盖一个点”和“覆盖一块面积”的判定不同。

机械臂、VLA 和真实工具机构被保留为后续独立工作。当前用 `ToolTask` 动作表达作业请求、反馈、取消、停止与残余。这让导航系统能先验证完整接口，又不必假设尚未设计的机械结构已经存在。

由此形成第二条原则：**任务完成量应来自有效反馈**。经过目标、走过路径、发出了工具命令，都不足以独立产生作业信用。

## S01 至 S18 建设的是能力链

|阶段范围|要解决的问题|为什么按这个顺序|
|---|---|---|
|S01–S04|环境、世界、底盘、带误差传感器|先让物理与观测来源可信|
|S05–S08|局部估计、建图、冻结图定位、GNSS 监督|先区分局部连续性和全局一致性|
|S09–S10|通行代价、工具接口与账本|定义可执行条件和可计量结果|
|S11–S14|连通分簇、路径和工时、导航、任务闭环|把几何计划变成真实动作|
|S15–S16|预测与缓存对照|用数据决定是否启用更复杂方案|
|S17–S18|六簇正式验收、可复现交付|验证整条链路，而非累计组件 PASS|

这一表描述计划职责，具体实现和通过范围要查对应版本。后续的 R 系列没有取消原 S17/S18 的标准。

## 为什么后来拆成三个独立任务

长任务和完整矩阵需要很长现实时间。为了让演示、学习与故障定位更可控，项目改成点喷、巡检、割草分别运行，各自使用适合的车辆和任务语义，共用场景与基础处理链。点喷参考 Husky 尺寸、巡检参考 Jackal 尺寸、割草使用 Dino 的前阿克曼几何；名称表示尺寸与模型参考，不等于商品整机认证。

拆分可以缩短一次启动和专项验证的范围，也便于定位某个功能的失败；它不会自动减少“把全部工作和全部故障组合完整测试一遍”的总需求。[R01–R12 计划][r-plan]

## 真实测试怎样推动设计修正

|遇到的事实|调整|保留的边界|学习价值|
|---|---|---|---|
|换车后传感器高度和可见结构变化|独立外参、有效传感器版本、区域观测匹配|不继承旧车辆的全局重定位资格|硬件几何也是算法输入|
|阿克曼地头绕圈大、端点难到达|整车扫掠、曲率约束、分挡停车、末端接近段|不缩车、不让阿克曼原地旋转|路径必须服从运动学|
|原 B1 部分终点与超限坡面冲突|用户批准独立安全执行版本，保留原参考|面积分母、原路径和残余不变|修改问题定义必须显式留痕|
|微小精停动作达不到粗导航进展尺度|为验证后的短动作设置专用进展尺度|超时、碰撞、作业准入继续独立检查|监督器也有适用尺度|
|启动工具时收到取消、反馈到达顺序变化|持续处理 BT 指令，分开停止确认与反馈时钟|不能用取消请求代替物理停止|异步系统需要完整状态机|
|失败现场难以学习|增加只读实时 GUI，展示原因与证据|不引入第二个控制出口|可观测性是工程能力|

## 当前仍在探索的部分

完整 A1 的后半程仍出现定位旋转跳变与再次锁存；完整 B1 停车重定位时出现 GNSS 创新拒绝及等待超时。已有记录缩小了排查范围，但不能把末次快照直接当成已证明根因。详见[当前修复报告][repair-doc]和[问题分类](08_failure_cases.md)。

这些失败可以直接成为教材案例：读者不仅学“如何成功”，还学如何保护旧结果、保存残余、提出可证伪的假设，并用针对性检查区分假设。

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
