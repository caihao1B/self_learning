# 本机复现与 GitHub 同步范围

当前可运行工程位于 `/home/ceihao/robot_project`。本轮同步第一方源码、配置、技术说明、GUI、紧凑验收记录和展示视频；已安装的 ROS/Gazebo、Python 环境、编译输出和完整原始运行数据仍在本机。GitHub 源码同步不等于可独立启动的运行环境发行包。

## 在当前电脑复现

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/gui.sh
```

打开 `http://127.0.0.1:8088`。在另一终端一次启动一个案例：

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R12 record spray
```

其他案例及其范围见 [GUI 手册](../../../optimization_v2/three_tasks/monitor/README.md) 和 [R12 手工命令](MANUAL.md)。不要在任务运行中重新编译或更改绑定源码。任务检查返回 PARTIAL 时应查看具体缺项，不应修改回执把它改成 PASS。

## 迁移到另一台电脑前

需另行迁移原冻结数据库 `optimization_v2/maps/frozen/map.db`（150085632 字节），以及匹配版本的地图、区域观测资产、依赖和阶段资源。原数据库的 SHA-256：

```text
b4d1a2d508ad9dc17205f77ba34ffb8a8e662e2f0f27a8f94119e7147df4bb82
```

此数据库未纳入本轮 Git 源码提交，不可删除本机副本。`validation/GUI/local_resource_inventory.json` 列出冻结地图清单的期望哈希和本机文件大小；这份清单只描述依赖，不替代文件，也不证明全新系统部署已通过。原冻结地图 `manifest.json` 和运行结束时的数据库完整性检查仍是依据。

手工检查时可在仿真停止后执行：

```bash
cd /home/ceihao/robot_project
sha256sum optimization_v2/maps/frozen/map.db
source optimization_v2/env.sh
bash optimization_v2/three_tasks/advance.sh R08 check
bash optimization_v2/three_tasks/advance.sh R09 check
bash optimization_v2/three_tasks/advance.sh R10 check
bash optimization_v2/three_tasks/advance.sh R12 check
```

原 S17/S18 六簇正式标准和未复验的大规模任务继续保留 PARTIAL。新的 GUI、局部修复和小规模演示分别有自己的证据范围。
