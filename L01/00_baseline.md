# 学习基线与事实边界

本教材解释的是一套已经有真实仿真、源码和结果记录的农田机器人系统。阅读时最重要的习惯是同时看三个对象：代码版本、运行输入、对应结果。一个阶段曾经通过，不能保证后来修改的版本也通过。

## 固定版本

|对象|本次采用的值|
|---|---|
|机器人仓库|`caihao1B/ROS2_BT-`|
|固定提交|`9aff8337b7794f704c5556a973a5220e73f17808`|
|Git 树|`0e71e12fbf3bfa67295abd9cec071c6edc9bd531`|
|当前运行工程|`/home/ceihao/robot_project`|
|学习资料目录|`/home/ceihao/self_learning`|
|查验范围|固定提交下的 821 个 `optimization_v2/ws/src` 文件与本机字节比对|
|本轮方法|读取代码、冻结资料、核查已有回执、运行小型离线教学例；没有重跑机器人仿真|

Git 提交可以理解为“指向一组固定文件的版本标识”。分支 `main` 会变化，所以教材中的外部源码链接使用完整提交号。SHA-256 用于发现文件变化；它不证明算法正确。教材的 [manifest](../baseline/manifest.json) 同时保留 Git blob 标识、SHA-256、文件用途和源码链接。

一个早期 S 阶段总计划文件在本机与公开提交之间存在字节差异。教材采用固定提交里的版本并记录差异，不修改原工程。历史文档的写法与当前实现不一致时，当前代码及其绑定结果具有更直接的解释力。

## 当前能够确认的范围

|能力或阶段|事实|阅读边界|
|---|---|---|
|R08 巡检|保存的阶段回执 PASS，当前文件校验另见检查结果|包含本阶段绑定的巡检、跟随、停车恢复等案例|
|R09 割草组件|保存的阶段回执 PASS|组件资格不能替代完整 B1|
|三点点喷录像|选中 3 点、完成 3 点；真实 JAM、重试和幂等重放通过|A1 原目标仍为 56 点|
|五段割草启动故障录像|选中 5 段、完成 5 段；取消与停止恢复通过|安全执行计划总计 47 段|
|完整 A1|完成 28/56 点，定位重验证达到上限|完整任务仍未通过|
|完整安全版 B1|完成 19/47 段，等待超时后人工处理|原参考 34 段及其 PARTIAL 记录保留|
|R10、R11、R12|PARTIAL|外观和录像组件存在，上游完整任务条件仍不足|
|原 S17、S18|PARTIAL|未补齐六簇正式矩阵，不能从小规模录像推定完成|
|实时 GUI|14 项单元检查、21 项浏览器检查的记录通过|用户人工体验另列，界面不授予任务 PASS|

主要结果可对照[三点回执][recording]、[A1 摘录][full-a1]、[B1 摘录][full-b1]。阶段完整性检查的本次输出保存在 [stage_checks.json](../validation/stage_checks.json)。

## 三个容易混淆的概念

**SLAM、局部里程计和全局重定位。** SLAM 同时估计运动并构建地图；局部里程计强调连续的短时运动；全局定位需要把当前观测对齐到已有地图。当前三任务链路使用 RTAB 原生局部 ICP 里程计、轮速/IMU 融合，以及冻结观测点云的区域匹配和 GNSS 监督。跨车高的旧 RTAB 数据库原生全局重定位存在失败记录，不能写成全面解决。[定位变更说明][localization-doc]

**实验完成与模型值得启用。** 历史 S15 完成了预测候选的对照，但当时默认选择 `no_prediction`。S16 记录缓存净收益与命中证据不足，默认关闭。这里的实验 PASS 说明按当时范围完成检查，不表示候选模型优于基线；当前巡检跟随还使用几何跟踪与路径关系，不能简单归因于学习型预测器。[预测实验][prediction]、[缓存实验][cache]

**模拟反馈与真实农业效果。** 工具动作来自仿真执行器及反馈，账本按有效数据确认剂量或面积。它适合验证接口、取消和幂等性；喷洒机械臂、真实药液沉积、真实刀具切割效果仍是后续独立项目。

摄像头有实际仿真图像和 GUI 预览，但“图像可显示”本身不证明视觉语义识别通过。讲解某个任务的避障时，应查看它实际接入的几何、跟踪或识别支路，不能因为系统里有摄像头就推定所有决策使用了图像分类。

## 阅读证据的五个问题

先问“这是什么版本”，再问“选择了多少任务”，然后查“实际完成了多少”“靠什么反馈确认”“失败后是否安全停止”。这五个问题比只看一行 PASS 更能说明系统做到了什么。

公开 Git 仓库含源码、配置与精选资料；冻结的 `map.db`、已安装依赖和完整运行资源还在原 Linux 电脑。教材可以在 Windows 阅读，直接下载学习仓库不会获得完整仿真运行环境。资源边界详见[环境说明](09_environment.md)。

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
