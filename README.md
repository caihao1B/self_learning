# 农田机器人项目学习教材

从第一次打开终端，到理解点喷、巡检、割草的观测、决策、控制和作业反馈。本仓库面向刚接触 Linux、ROS 2 和机器人项目的学习者。

**当前交付包含 L01 和 L02 完整三十章教材。** [进入三十章目录](L02/README.md)，或下载后打开根目录 `index.html` 并点击 L02。完整学习网站与发布验收属于 L03。机器人项目的 R10–R12 仍为 PARTIAL；学习资料的完成状态独立记录。L01 文件保留为当时的设计与检查记录。

L02 提供逐章原理、56 段固定生产源码解读、三十个 Python 离线练习、真实 C++ 判据调用、Linux 命令、环境搭建、三任务操作卡和故障分析。阅读入口：[学习路线](L02/learning_routes.md) · [三点闭环](L02/chapters/08_spray_walkthrough.md) · [环境搭建](L02/chapters/26_environment.md) · [人工操作](L02/chapters/27_operations.md) · [检查结果](L02/RESULTS.md)。

## 从这里开始

- Windows：在 GitHub 直接阅读章节；也可用仓库的 **Code → Download ZIP** 下载，解压后双击根目录 `index.html`，进入 L02 三十章阅读版，不需要安装 ROS 或 Python。使用期间保留 `L01`、`L02`、`assets`、`baseline`、`evidence` 等相邻目录。完整网站搜索等功能留到 L03。
- Linux：阅读同一份资料；需要操作机器人时，进入原工程 `/home/ceihao/robot_project`。本学习仓库不是机器人的可运行替代目录。
- 离线练习：使用明确标注的合成数据讲解算法，或读取附带的真实历史结果，不启动仿真。见[三十个练习](examples/l02/labs.py)和[三点历史结果分析](examples/inspect_spray.py)。

|阅读顺序|资料|你将学到什么|
|---|---|---|
|1|[学习基线与事实边界](L01/00_baseline.md)|辨认当前代码、历史证据和仍未完成的功能|
|2|[需求演变与设计理由](L01/01_evolution.md)|理解从原项目到三任务系统的取舍|
|3|[架构和四张关系图](L01/02_architecture.md)|追踪传感器、任务、控制和反馈|
|4|[技术选型与适用边界](L01/03_choices.md)|知道每个库负责什么，为什么选它|
|5|[源码与证据导航](L01/04_code_evidence.md)|从模块找到函数、固定版本代码和结果|
|6|[完整教材目录](L01/05_curriculum.md)|了解后续 30 章的目标、前置和练习|
|7|[新人学习路线](L01/06_learning_routes.md)|按观察、理解、实验、修改的顺序学习|
|8|[详细样章 三点点喷闭环](L01/07_sample_spray.md)|逐层解释命令、Python、C++、工具反馈与故障恢复|
|9|[失败记录与问题分类](L01/08_failure_cases.md)|区分停止原因和根因，学习排查顺序|
|10|[环境与复现资源](L01/09_environment.md)|区分操作系统、依赖、源码、构建物和数据|
|11|[L01 检查结果](L01/RESULTS.md)|查看本次实际验证的范围|

[资料来源与复核方法](L01/10_sources_and_method.md)说明固定版本、标准库资料和检查范围。

机器人源码固定于 [ROS2_BT- 的 9aff8337 提交](https://github.com/caihao1B/ROS2_BT-/commit/9aff8337b7794f704c5556a973a5220e73f17808)。附带源码摘录、机器可读哈希清单与精选证据，方便在 Windows 上对照阅读。详细 provenance 见 [baseline/manifest.json](baseline/manifest.json)。

## 怎样理解这里的状态

`PASS` 表示对应范围的检查通过；`PARTIAL` 表示有内容完成但必要项不足；`HISTORICAL` 表示历史版本结果；`PLANNED` 表示教材或功能尚未交付；`PENDING_USER` 表示需要用户亲自检查的体验。视频文件可播放、任务安全停车、完成全部作业，是三个不同结论。

所有本轮教材日期采用北京时间。原证据保留其 UTC 时间或仿真时间，不改写原始记录。原代码版权与第三方说明保持原有归属，本仓库未推定或补授机器人项目的统一开源许可。

下一步提示词：**执行 L03：基于 L02 三十章内容制作完整在线与离线学习站，完善导航、搜索、代码阅读、素材和发布验收；保留固定学习基线与真实验收状态，同步 self_learning。**
