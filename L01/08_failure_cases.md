# 失败记录与优先学习的问题

机器人仿真中的失败通常是多层共同作用：一个错误位姿可能导致地形投影错位，随后控制器拒绝路径，最后任务表现为等待超时。只记最后的报错会漏掉真正的因果链。

## 样本范围与统计方法

本基线的 R10 修复摘要包含 **42 个不同运行目录**，其中 31 条在记录时没有取得任务 PASS。它们跨越多个代码版本，包含主动停止、时间预算结束和程序错误，不是随机抽取的稳定版本成功率样本。

按摘要里的原始停止字符串计数，`WAIT_DEADLINE` 出现 6 次；`TASK_BOUND_OR_STOP_REQUEST`、`DOCKING_REALIGNMENT_EXHAUSTED`、`ENDPOINT_RECOVERY_EXHAUSTED` 各出现 3 次。完整计数见 [failure_summary.json](../evidence/failure_summary.json)，原始记录见[尝试摘要][attempts]。统计表示这个样本中的停止名称频次，不能说“等待超时就是项目最常见的根因”。

L02 将把案例按环境、观测、几何、控制、状态机与反馈分类，保留重复运行与主动中断标记。尚无证据的根因只列为假设。

## 重点案例

|现象|已知证据与解释|采用的处理|当前教学结论|
|---|---|---|---|
|几厘米精停时无进展上报|原进展要求 10 cm，大于整个短动作位移|专用精停尺度，且仅允许已验证的小范围动作使用|已实现并有组件与运行证据；不代表完整 A1 已完成|
|定位跳变后一直等待|恢复锁存可能晚于最初取消到达；等待期间仍需检查|等待逻辑持续响应定位重验证；prepare/commit 保留版本与路径证明|异步事件不能只检查一次|
|完整 A1 后半程重复锁存|三次旋转差超过正常 0.035 rad 门限；对应平移仍在门限内|保留停止与日志，进一步查旋转估计、采集时刻与静止滤波一致性|当前未解决；不能把 0.10 rad 停车候选门限当成正常跳变门限|
|割草路径终点落在超限坡面|原路径的部分终点与 10° 坡面重叠，允许坡度 5°|独立安全执行版本，保留原目标面积和历史参考|规划先验需要兼容车辆与安全要求|
|阿克曼末端位置接近却朝向失败|短距离纠偏要求过大曲率，换向请求又受保护|曲率预瞄、分挡停车、原终点之前的直线接近候选|末端恢复有成功片段；完整 B1 仍未通过|
|车身附近出现假障碍或未知格|自车回波、误差余量和异步运动补偿会影响真实地面点|先在采集时刻车体系自过滤，再作运动补偿；保留地面例外|不能靠清空未知格获得通行|
|工具开始工作时取消不及时|启动与等待反馈阶段也会收到 BT CANCEL/WAIT|每次 tick 检查，分别确认导航、车体和工具停止|请求取消、收到确认、实测停止是不同事件|
|反馈看似过期或停止未确认|话题到达顺序与采集顺序不同；大点云可能争用传输|有界缓冲、独立工具桥、采样与传输调整|时间戳、接收墙钟和有效期分别检查|
|割草账本几何异常|足迹和边界相切时交集可能含点、线、面|只累计面积部分，异常帧保留拒绝原因|不能用异常帧虚增覆盖，也不能丢掉已有确认量|
|Python 找不到 shapely|实际任务使用的解释器环境缺依赖|核对解释器和其依赖，使用指定虚拟环境|终端中的 python 与脚本指定的 python 可能不同|
|dict 重复传入 t 参数|事件记录的通用字段与调用者字段重名|修正字段命名并保留失败记录|日志代码本身也可能打断任务|
|录像新鲜度失败|文件可以播放但长任务中的图像可能停更|分别检查文件解码、覆盖时间线与活动图像年龄|录像存在不是闭环执行证据|

上表的处理经过不同阶段版本，具体支持范围见[割草修复说明][mowing-doc]与[本轮修复][repair-doc]。一种处理被采用，不能自动说明所有类似现象已经消失。

## 遇到机器人不动时怎样查

先看任务是否仍在运行、仿真时钟是否推进。再看任务阶段与 BT 原因码，然后看定位/感知是否有效、Nav2 是否有候选命令、底盘门控是否允许、实测轮速和车体是否运动。最后检查工具状态与完成量。

这条顺序沿实际链路缩小范围。例如“BT 为 EXECUTE、原始速度非零、门控后速度为零且 reason 为感知过期”优先指向感知准入；此时调 RPP 速度参数通常不能解决数据过期。

若进程已退出，先保存 `mission_result.json`、`mission_events.jsonl`、`runtime.json` 和相关最后一段状态，再看堆栈。重复运行会产生新的目录，应记录明确的旧运行名，避免把新一次成功解释成旧一次失败已经消失。

## 对当前未完成问题提出假设

完整 A1 需要核查旋转跳变与观测时刻、当前里程计和静止滤波之间的关系。完整 B1 的最后记录出现 GNSS 创新约 0.1693 m，超过 0.15 m 门限，恢复候选没有积累。应查“为什么候选没有通过”和“候选验证是否依赖正在被修复的旧校正”，而不是先扩大门限。

学习练习：为每个问题提出两个可能解释，列出各自支持与反对的观测，再设计不需要完整长任务的最小检查。原 42 条尝试和当前结果可以提供线索，但不能代替新的针对性验证。

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
