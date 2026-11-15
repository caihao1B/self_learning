# 新人学习路线与检查方法

先观察一个具体任务，再逐层解释，最后做小修改。第一次学习不必先掌握所有 SLAM 数学，也不必从头阅读八百多个源码文件。

## 路线一 在 Windows 看懂一个任务

打开根目录 `index.html` 或在 GitHub 阅读 README。依次阅读基线、需求演变、架构和样章，遇到不懂的 Linux 语法先查看样章的命令拆解。这里需要浏览器，不需要启动 Linux 仿真。

然后运行附带的 Python 证据读取例，或直接阅读它的保存输出。练习只使用教材中的 JSON/JSONL，能够让你亲手区分“第一次工具动作失败”和“整个三点演示最终成功”。你应能解释 3/3 与 3/56 的分母、重放为什么不重复计量，以及为什么录像通过不能让完整 A1 变成 PASS。

第一轮完成的检查是：不看答案，用纸画出“导航到点—停止—观测稳定—工具动作—反馈账本—下一个点”的流程，并在故障位置添加取消和恢复分支。

## 路线二 在 Linux 复现已有短案例

先检查工程目录、运行资源和 R09 前置。打开实时 GUI，再在另一个终端只运行一个短任务。第一遍尽量不修改参数，先学会读取版本、阶段、数据年龄和拒绝原因。

按顺序观察：三点点喷工具故障恢复；巡检同向目标跟随；五段割草启动故障取消。具体命令见样章与原 GUI 手册。每个案例结束后查看结果文件与最终停车，不用“机器人动起来了”代替检查。

这条路线使用当前已准备的 Linux 机器。新机器先完成第 26 章的资源与构建步骤；不能只复制一条命令就假设环境存在。

## 路线三 阅读实际代码

从小到大阅读：`advance.sh` → `dispatch.py` → `stage_r12.record()` → 运行器的 `launch()` → `mission_spray_r10.main()`。先画调用图，再进入嵌套的 `wait()`、`service()`、`cancel_nav()` 和 `settle_and_redock()`。

看到 `ActionClient` 时，先找动作类型和服务器名称；看到一个订阅回调，先找它写入哪一个状态；看到一个异常，查它在哪里被捕获、是否执行 finally、最终写什么结果。C++ 则从短小的 `progressDelta()` 入门，再读工具状态机和 BT。

不要假设 Python 的状态变量与 C++ 内部状态自动同步。它们通过 ROS 消息通信，有时间戳、队列、延迟和失效条件。

## 路线四 用失败学习调试

先读 3.6 cm 精停与 10 cm 进展判据的尺度矛盾，再读车身中心/后轴旋转、工具反馈时钟排序和大点云传输。每次都写出三项：我观察到了什么；我的解释是什么；用哪一条额外检查可以区分这个解释与另一种解释。

完整 A1 与 B1 的当前未完成问题用于练习提出假设，不安排成“按某个现成补丁必定修好”的题目。它们尚缺完整因果验证，教材答案会保留这个事实。

## 学习记录模板

|记录项|写法示例|
|---|---|
|环境|Windows 阅读 / Linux 实操；操作系统与 Python 版本|
|基线|固定提交和当前任务版本|
|目标|例如理解工具重放，而不是笼统写“学习 ROS”|
|输入|使用哪个案例和证据文件|
|操作|原样记录命令、工作目录和终端编号|
|预期|我认为应该看到什么|
|观察|实际字段、数值、时间和截图|
|解释|事实与推测分开|
|下一步|用哪个最小实验验证推测|

每次只新增一个学习问题。对某个参数做实验前记录原值和单位；运行结束后比较效果，并说明是否影响其他模块的有效版本。学习资料中的源码快照用于阅读和例子，不应直接覆盖机器人工作区。

## 什么时候可以进入下一层

能解释命令的路径与参数，再学习进程和通信；能指出数据的坐标系和时间，再学习定位与融合；能区分目标、路径、速度和最终出口，再学习 BT 与恢复；能解释确认量与残余，再讨论完整覆盖率。

通过这些检查比固定“第几天必须学完”更可靠。三十章的理论深度不同，阅读和实操可按你的基础多次循环。

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
