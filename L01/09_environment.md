# 环境与复现资源

阅读教材只需要浏览器。运行离线 Python 例子需要 Python 3；运行附带的 C++ 示例需要编译器和 nlohmann/json 头文件；运行真实机器人则需要完整 Linux、ROS/Gazebo、工作区构建物与地图资源。这三个层次分别准备。

## 当前作者机的事实

本轮只读核查得到 Ubuntu 24.04.5 LTS、x86_64、Python 3.12.3。项目使用 ROS 2 Jazzy 和 Gazebo Harmonic。基础依赖的具体 Debian 版本记录在 [manifest 的 environment 字段](../baseline/manifest.json)，用于追溯，不承诺软件源未来仍提供相同二进制包。

|组件|已查到的上游版本或状态|用途|
|---|---|---|
|robot_localization|3.8.3|轮速/IMU/激光增量融合|
|RTAB、rtabmap-odom、rtabmap-slam|0.23.13，组件包已安装|激光里程计与既有建图流程|
|BehaviorTree.CPP|4.10.0|任务行为监督|
|Nav2 controller|1.3.13|导航控制框架|
|Fields2Cover|2.1.1|覆盖条带与相关几何|
|OR-Tools vendor|9.9.1|点访问顺序等优化|
|METIS|5.1.0|图划分|
|nlohmann/json|3.11.3|C++ JSON 数据读取|

本机 `ros-jazzy-rtabmap-ros` 集合包查询失败，但三个实际 RTAB 组件包安装成功。这说明“某个包名查不到”和“相关程序没有安装”不是同一个判断。组件补充查询见 [rtabmap_components.txt](../validation/rtabmap_components.txt)。早期 S01 锁文件中的 deferred 状态只描述当时阶段。

## 当前机器上的操作入口

`optimization_v2/env.sh` 清理旧 overlay 变量，再加载 `/opt/ros/jazzy/setup.bash` 和当前工作区。它也重设 PATH、ROS domain 和 Gazebo partition，所以应在任务终端使用，不要假设 Conda 或别的工作区会继续生效。[环境脚本][env]

`advance.sh` 使用项目的 `optimization_v2/venvs/scene/bin/python`，不是从 PATH 随意找到一个 Python。内部部分 ROS 进程仍使用系统 Python，部分算法进程使用项目虚拟环境。依赖必须安装到实际执行该代码的环境。

## 新机器需要准备哪些层次

|层次|要准备什么|怎样检查|
|---|---|---|
|系统|Ubuntu 24.04、兼容图形驱动、显示与可用磁盘|系统版本、基础渲染和传感器实际输出|
|ROS 与 Gazebo|Jazzy、Harmonic、ros_gz 及节点依赖|组件位置、版本、节点启动与桥接|
|系统开发库|编译器、CMake、Eigen、PCL、JSON、METIS 等|头文件、库和项目构建结果|
|Python 环境|系统 ROS Python 与项目 scene/算法环境|每个解释器下单独导入、版本与路径|
|源码构建|工作区包、阶段派生插件和可执行文件|构建日志、安装位置和绑定哈希|
|输入资源|模型、世界、冻结地图、区域观测资产、规划配置|资源清单、哈希及实际加载|
|运行资格|对应版本的前置和检查|有效回执与必要的真实短复验|

L02 将逐条给出安装和构建命令，并解释 apt、pip、venv、CMake 和 colcon 的区别。当前的历史安装脚本含固定版本和本机代理配置，不能不加解释地作为任何新电脑的一键安装器。

## 必须单独保留的地图

冻结数据库路径为 `optimization_v2/maps/frozen/map.db`，大小 150085632 字节，预期 SHA-256 为：

```text
b4d1a2d508ad9dc17205f77ba34ffb8a8e662e2f0f27a8f94119e7147df4bb82
```

数据库没有包含在本次教材或原源码提交里，仍在原 Linux 工程中。还有地图云、位姿图、区域资产和阶段输入等依赖；不能只补一个数据库就宣称新机器部署已完成。[原复现说明][resources-doc]、[资源清单][resources]

## Windows 的学习边界

下载教材后可以离线阅读文字、图、源码快照和精选 JSON。Python 例子用标准库读取随包数据；C++ 例子可先阅读源码和实际输出，编译工具链安装将在 L02 另讲。

Windows 浏览器打开 `127.0.0.1:8088` 会连接 Windows 自己；双系统切到 Windows 时，Linux 里的仿真通常并没有继续运行。学习站也没有 ROS 后端。实时 GUI 需要对应 Linux 会话正在运行，远程查看将另讲网络地址与连接方式。

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
