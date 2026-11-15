# 完整教材目录与章节验收

教材按学习依赖组织为八部分、三十章。L01 已交付设计资料及第 08 章的详细样章，其余章节的完整教学正文在 L02 编写。表中的实验是教学交付目标，不表示已重新执行所有机器人实验。

## 第一部分 项目问题与学习基础

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|01|项目为什么立项：三种作业、需求变化、环境约束、完成与残余|无|用自己的话区分导航到达和作业完成|
|02|Linux 第一次操作：终端、目录、绝对/相对路径、文件、权限、引号|01|找对项目目录，解释一条带参数命令|
|03|程序怎样运行：进程、环境、解释器、编译、退出码、信号和日志|02|区分 source 与 bash，解释 Ctrl+C 后的收尾|
|04|C++ 与 Python 入门：变量、函数、类、集合、错误处理和接口|03|运行一个结果读取例和一个 C++ 判据例|

## 第二部分 ROS 与完整系统

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|05|ROS 通信：节点、topic、service、action、QoS、生命周期|03–04|解释为什么导航用 action，状态常用 topic|
|06|空间和时间：map/odom/base_link、旋转、单位、时间戳与仿真时钟|04–05|画出 TF 关系，计算一个时间差并判断其意义|
|07|整个系统架构：数据流、权限、进程编排、评估隔离与版本|05–06|从传感器指出影响停车的完整链路|
|08|从启动到三点点喷闭环：命令、Python 编排、C++ 监督与实际证据|02–07；也可先按样章引导阅读|解释 JAM、残余重试、幂等重放及 3/56 范围|

## 第三部分 世界 车辆与传感器

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|09|三维世界与先验：不规则区域、道路、坡度、永久障碍和未知信息|06–07|指出离线规划允许使用的先验与实时感知范围|
|10|差速与阿克曼：轮距、轴距、转角、后轴基准和整车扫掠|04、06|画出最小转弯圆并解释车体外侧空间|
|11|传感器模拟：点云、图像、IMU、轮速、GNSS、外参、噪声和延迟|05–06、10|读取一条消息并解释字段、单位、采样时刻|

## 第四部分 定位与三维感知

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|12|轮速 IMU 与激光里程计：积分、偏置、ICP、EKF 与质量门控|06、11|说明三个输入如何互补以及不能互相证明的情况|
|13|SLAM 建图 回环与冻结地图：位姿图、地图资产和版本|12|区分建图、回环、局部里程计和重定位|
|14|全局定位与 GNSS 监督：区域匹配、杆臂、创新、NIS 和重验证|12–13|解释定位跳变为何触发停车，辨认已证实与待验证根因|
|15|点云预处理与地形：自车过滤、时间补偿、坡面、阶差、盲区|10–12|用一段观测解释假障碍和未知格|
|16|动态障碍与跟随：聚类关联、路径关系、间距、TTC 与预测候选|14–15|区分同向目标与横穿目标，解释跟随准入|
|17|预测和经验缓存实验：候选、训练/测试边界、命中、开销和启用条件|16|根据实际对照判断为什么当前不默认启用|

## 第五部分 规划与导航控制

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|18|连通工作量分簇：图、节点权重、METIS、连通与平衡检查|09–10|说明等面积为何不等工时，检查三个簇是否连通|
|19|点访问序列：TSP、OR-Tools、合法连接与作业时间|18|对比两种点序的路程和完整作业时间|
|20|条带与地头：Fields2Cover、幅宽、OMPL、倒车及坡面裁剪|10、15、18|解释原 B1 与安全执行版本及不变的残余分母|
|21|Nav2 三类动作与规划器：目标、路径、生命周期、取消和代价图|05、14–15、19–20|说清 NavigateToPose 与 FollowPath 的职责|
|22|RPP MPPI DWB 的控制思想：预测/采样/几何跟踪、运动学与精停|10、21|解释控制器为何不能补救几何上不可行的路径|

## 第六部分 决策与有效作业

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|23|可解释 BT：节点状态、优先级、原因码、有界等待和人工锁存|05、16、21|从输入证据推导一次 WAIT/FOLLOW/REPLAN/HUMAN|
|24|任务状态机和恢复：动作 Future、取消确认、精停、重验证及换挡|08、22–23|追踪一条异常如何进入停车与保存残余|
|25|工具动作和账本：模拟反馈、有效剂量/面积、幂等、事务和重启|05、24|证明重放没有重复作业，说明停止不确定为何重要|

## 第七部分 部署与人工实验

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|26|环境搭建：当前机与新机器、apt/pip、CMake/colcon、资源哈希|02–07|列出源码之外还需要的运行资源，核对依赖位置|
|27|三任务人工操作：预检、一个任务一次启动、GUI、停止、结果与录像|08、26|按操作卡独立检查三种短案例|
|28|实时 GUI 与证据阅读：相机/点云、频率、拒绝原因、账本、历史状态|14–16、23–25、27|在 GUI 中分清新鲜、有效、历史和完成状态|

## 第八部分 调试与自主研究

|章|标题与主要内容|前置|完成后应能展示|
|---|---|---|---|
|29|失败案例库：环境、时间、坐标、定位、几何、控制、取消与反馈|前面相关章|按照证据而不是错误名称排查，写出可验证假设|
|30|从学习到修改：影响范围、版本、组件检查、短复验与完整任务|27–29|完成一份修改实验设计并说明哪些历史证据失效|

## 每章统一采用的教学结构

每章先说明一个具体问题，再给直观例子和图，随后补充必要数学、输入输出、时间和坐标约定。代码部分给出真实文件和核心函数、调用者与被调用者、正常与异常分支；最后安排小型练习、项目操作、检查依据和答案。

术语首次出现时同时给中文含义和英文缩写。例如 odometry 译为里程计，描述连续运动估计；并不意味着它只能来自车轮。公式必须说明每个变量的单位。参数分为物理约束、经验设置、实验门限和教学示例值，避免混用。

生产代码不必在两种语言中重复实现。某一步主要调用上游 C++ 库时，会说明调用边界与库的职责，再给标明用途的简化教学例。复杂理论的推导安排在可选深入段落，不阻断第一次理解流程。

## L02 与 L03 的完成条件

L02 应交付三十章实质正文、逐章源码映射、命令说明、可运行的小例子、答案及失败案例。不能只用现有阶段文档的链接代替正文。

L03 将内容制作成完整在线/离线学习站，验证导航、搜索、代码展示、素材、不同窗口尺寸与发布结果。L01 当前的 `index.html` 是阅读预览，不代表这些完整网站功能已经验收。

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
