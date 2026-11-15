# L02 完整三十章教材

固定机器人基线 `9aff8337`。从 Linux 与编程基础，到传感器、定位、规划、BT、工具账本和人工操作。原机器人 R10–R12 仍为 PARTIAL；本教材的完成状态独立记录。

Windows 下载完整仓库后打开根目录 index.html，再进入本目录。每章都有固定源码节选、逐段解释、可运行离线练习、输出与故障排查。真实仿真命令集中在第 08、26、27 章并明确前置。

[先看学习路线](learning_routes.md) · [术语与命令索引](reference.md) · [检查结果](RESULTS.md) · [L01 设计依据](../L01/05_curriculum.md)

## 项目问题与学习基础

|章节|本章代码与练习|
|---|---|
|[第 01 章 项目为什么立项](chapters/01_project.md)|2 段生产源码；练习 01|
|[第 02 章 Linux 第一次操作](chapters/02_linux.md)|1 段生产源码；练习 02|
|[第 03 章 程序怎样运行](chapters/03_process.md)|2 段生产源码；练习 03|
|[第 04 章 C++ 与 Python 入门](chapters/04_languages.md)|2 段生产源码；练习 04|

## ROS 与完整系统

|章节|本章代码与练习|
|---|---|
|[第 05 章 ROS 通信与生命周期](chapters/05_ros.md)|2 段生产源码；练习 05|
|[第 06 章 空间坐标与时间](chapters/06_frames_time.md)|1 段生产源码；练习 06|
|[第 07 章 完整系统架构](chapters/07_architecture.md)|2 段生产源码；练习 07|
|[第 08 章 从启动命令到三点点喷闭环](chapters/08_spray_walkthrough.md)|1 段生产源码；练习 08|

## 世界 车辆与传感器

|章节|本章代码与练习|
|---|---|
|[第 09 章 三维世界与先验分层](chapters/09_world.md)|2 段生产源码；练习 09|
|[第 10 章 差速与阿克曼运动学](chapters/10_kinematics.md)|1 段生产源码；练习 10|
|[第 11 章 传感器模拟与真实误差](chapters/11_sensors.md)|3 段生产源码；练习 11|

## 定位与三维感知

|章节|本章代码与练习|
|---|---|
|[第 12 章 轮速 IMU 与激光局部里程计](chapters/12_odometry.md)|2 段生产源码；练习 12|
|[第 13 章 SLAM 建图 回环与冻结地图](chapters/13_slam.md)|1 段生产源码；练习 13|
|[第 14 章 全局定位与 GNSS 偏移监督](chapters/14_global_localization.md)|2 段生产源码；练习 14|
|[第 15 章 点云预处理与地形代价](chapters/15_terrain.md)|2 段生产源码；练习 15|
|[第 16 章 动态障碍与保持距离跟随](chapters/16_dynamic.md)|2 段生产源码；练习 16|
|[第 17 章 预测与经验缓存实验](chapters/17_prediction_cache.md)|2 段生产源码；练习 17|

## 规划与导航控制

|章节|本章代码与练习|
|---|---|
|[第 18 章 连通工作量分簇](chapters/18_partition.md)|2 段生产源码；练习 18|
|[第 19 章 点访问序列与工时](chapters/19_tsp.md)|2 段生产源码；练习 19|
|[第 20 章 条带 地头与安全执行版本](chapters/20_swaths.md)|2 段生产源码；练习 20|
|[第 21 章 Nav2 动作与规划器](chapters/21_nav2.md)|2 段生产源码；练习 21|
|[第 22 章 RPP MPPI DWB 与精停](chapters/22_controllers.md)|2 段生产源码；练习 22|

## 决策与有效作业

|章节|本章代码与练习|
|---|---|
|[第 23 章 可解释行为树](chapters/23_bt.md)|2 段生产源码；练习 23|
|[第 24 章 任务状态机与恢复](chapters/24_state_machine.md)|2 段生产源码；练习 24|
|[第 25 章 工具动作与有效作业账本](chapters/25_ledger.md)|2 段生产源码；练习 25|

## 部署与人工实验

|章节|本章代码与练习|
|---|---|
|[第 26 章 环境搭建与资源迁移](chapters/26_environment.md)|2 段生产源码；练习 26|
|[第 27 章 三任务人工操作与录像](chapters/27_operations.md)|2 段生产源码；练习 27|
|[第 28 章 实时 GUI 与证据阅读](chapters/28_gui.md)|2 段生产源码；练习 28|

## 调试与自主研究

|章节|本章代码与练习|
|---|---|
|[第 29 章 失败案例与诊断方法](chapters/29_failures.md)|2 段生产源码；练习 29|
|[第 30 章 从学习到修改项目](chapters/30_research.md)|2 段生产源码；练习 30|

