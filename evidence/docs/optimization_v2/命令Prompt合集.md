# 原项目优化改造 18 步命令 Prompt 合集

版本 20261109.1。依据已批准的《不规则作业区自主机器人项目任务书》，将改造分为 **18 步**。本合集提供后续交给执行代理的完整指令、实际应交付的接口、手工检查命令和每步学习材料要求。**计划交付时仅建立步骤索引，18 个机器人改造步骤的初始状态均为 NOT_RUN；后续完成情况以 stepctl status 的有效回执为准。**

工作根目录是 `/home/ceihao/robot_project`。新加卷原目录只保留迁移指引；后续源码、模型和验收输出均放主文件夹。这里“贴近商用”指数据质量、职责隔离、可恢复动作、明确失效和可重复验收，不表示商业认证或任意场景都能成功。

## 怎样使用

以后在项目聊天中直接说：

> 执行第 1 步。按照项目中的命令 Prompt 合集完成实现、自动验证、手工查验入口和技术学习材料。

其他步骤把数字换成 2 至 18。也可以使用“继续第 N 步未完成部分”“复验第 N 步”“只显示第 N 步的手工验收命令”。`执行` 指实施该步；`复验` 指运行已有实现的测试；`check` 只核对已有回执。索引脚本不会自己调用 AI 编程。

以下命令**现在就可使用**，不启动 ROS/Gazebo：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh list
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh prompt 06
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh manual 06
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh status
```

`prompt 06` 显示第 6 步可整段复制的执行指令；`manual 06` 显示该步未来交付后的手工验收命令。`demo/test/open` 在对应阶段执行器/报告交付前会返回 `NOT_READY`，这是正常的未就绪状态。不要为了让状态变绿创建空处理器或伪造回执。

## 现有基础与首步要解决的问题

已核对原生入口为 `/home/ceihao/robot_project/native_20261108/run.sh`，平台为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。现存底盘、传感器、地图合同、导航仲裁、工具反馈/账本、BT 和 MPPI 代码有复用价值，但组件或静态预览可用不等于完整自主任务通过。

当前 `native_20261108/task_bt/run.sh` 仍引用不存在的 `native_20261108/dynamic_avoidance/run.sh`；原生 smoke 依赖项目外但仍在主文件夹的 `/home/ceihao/robot_migration_20261108/scripts/sensor_smoke.py`。另外存在两份 ros2_ws 源码树。因此先做基线闭合与运行复验，再建新系统，不能直接沿用历史报告的状态。

本次缓存查询显示 robot_localization 3.8.3 已安装，RTAB-Map 0.23.13、Fields2Cover 2.1.1、METIS 5.1.0 为候选；后续安装时重新核对并锁版本。候选可见不等于已安装、兼容或已验证。

原入口的两个现有命令如下，需顺序运行；它们不证明自主作业：

```bash
bash /home/ceihao/robot_project/native_20261108/run.sh check --seconds 35
bash /home/ceihao/robot_project/native_20261108/run.sh farm --seconds 180
```

## 任务约束与技术取舍

- 同一地图分次运行差速喷洒车与阿克曼割草车；两车均可巡检曲线连接路。本轮不增加麦克纳姆第三车型。
- 两区均为连续不规则多边形，算法各分三个相邻连通簇，不人为隔离、不固定相同形状；地图首轮面积只是候选，最终由可行路径和工时反算。
- 第 11 步只验几何和初估负载；第 12 步才用 TSP/条带真实路径完成工时闭环；第 14、17 步再用闭环仿真验证实际耗时。三种结果不混用。
- 局部里程计、三维建图、冻结图重定位、GNSS/偏移监督拆成四步，各有轨迹、地图或故障演示。感知避障使用同源传感器的独立处理支路。
- 作业抽象为带反馈的点服务/连续工具动作，保留几何和有效作业量；机械臂、近景作业视觉和 VLA 留到后续独立项目。
- 动态预测和经验缓存必须做对照；某个开源学习模型或记忆机制没有净收益时保留实测结论并默认禁用。实验完成与算法优越是两个不同判断。
- 30 分钟指单簇的仿真时钟上限，不是完整六簇、建图或所有测试的现实总耗时。初始正常计划目标 24 分钟，超时/长期封堵必须保存残余并安全结束。

## 步骤总览

| 步骤 | 目标 | 看得见的交付 | 前置 |
|---|---|---|---|
| [S01](prompts/S01.md) | 原项目基线复验与隔离开发环境 | 原模型画面、依赖/路径状态表 | 无 |
| [S02](prompts/S02.md) | 连续不规则三维世界与先验分层 | 完整 3D 场地、先验与未知层 | S01 |
| [S03](prompts/S03.md) | 差速与阿克曼轻量底盘 | 两车实际轨迹与轮速/转角 | S02 |
| [S04](prompts/S04.md) | 多传感器模拟与真实误差注入 | 点云、双目、IMU/轮速/GNSS 曲线 | S03 |
| [S05](prompts/S05.md) | 轮速 IMU 与激光局部里程计 | 三种里程计轨迹及轮滑对比 | S04 |
| [S06](prompts/S06.md) | 三维 SLAM 建图与回环 | 增长的三维地图、回环前后对比 | S05 |
| [S07](prompts/S07.md) | 冻结地图与重启重定位 | 重启重定位轨迹与地图版本拒绝 | S06 |
| [S08](prompts/S08.md) | GNSS 配准与定位偏移监督 | GNSS/地图定位对比及状态时间轴 | S07 |
| [S09](prompts/S09.md) | 三维感知融合与二维通行代价 | 地形色图、低障碍、目标框与代价图 | S08 |
| [S10](prompts/S10.md) | 喷洒割草抽象动作与有效作业账本 | 点状态、有效作业足迹与残余 | S08, S09 |
| [S11](prompts/S11.md) | 按工作量自动划分三个连通簇 | 整区自动三色分簇、邻接/负载图 | S02, S03, S10 |
| [S12](prompts/S12.md) | 点服务和条带离线路径与工时闭环 | 点序列、条带/地头动画、工时构成 | S11 |
| [S13](prompts/S13.md) | Nav2 控制器与三类导航动作 | 两车闭环轨迹及控制器对比 | S08, S09, S12 |
| [S14](prompts/S14.md) | 可解释行为树与完整作业闭环 | BT 高亮、理由时间轴、完整簇结果 | S10, S13 |
| [S15](prompts/S15.md) | 动态障碍多模态预测与风险接入 | 多模态未来轨迹与闭环 A/B | S14 |
| [S16](prompts/S16.md) | 经验检索缓存与算力收益对照 | 三组计算开销、命中与错误匹配 | S15 |
| [S17](prompts/S17.md) | 六簇任务与故障消融验收 | 六簇验收总览、失败回放/残余 | S14, S15, S16 |
| [S18](prompts/S18.md) | 可复现交付与学习材料总验收 | 三个最终入口、依赖/体积/学习索引 | S17 |

## 每一步共同遵守的执行要求

1. 先阅读 `/home/ceihao/robot_project/AGENTS.md`、本合集和本步 prompt。核对前置回执及实际源码，不假设文档里的历史 PASS 仍有效。用户说“执行第 N 步”即要求完成该步实现、验证、材料和证据；若前置缺失，先明确缺口并完成必要前置，不越过依赖直接宣布完成，也不自动扩展到不相关后续步骤。
2. 唯一实现工作区为 `/home/ceihao/robot_project/optimization_v2/ws`；参考原代码，复用确有价值的模块。建立轻量适配，避免同时维护两套相同实现。原 dino、原农田和已有有效输入保留。每步冻结实际依赖版本，只有确有需要才安装组件，禁止全局 sudo pip 或无依据升级驱动。
3. 保持唯一 TF、导航动作、底盘和工具出口；原停止、时间和反馈门限不可通过放宽使测试变绿。新参数须有推导和测试。真实姿态和未来轨迹只在传感器仿真器/独立评分器使用，在线算法只接收带误差的观测和授权先验。
4. 必须交付本步 `optimization_v2/stages/SNN/run.sh`，支持本索引调用的 `demo/test/stop`、`--case`、`--seed`、`--sim-seconds`、`--wall-timeout`，以及本步需要的 `--suite/--resume/--shard`。`demo` 默认打开所需 GUI，`test` 默认有界无界面执行；离线案例说明无仿真时钟。所有列出的 case 必须真实存在。启动时使用自己的进程组和 PID/会话记录，收到中断先关工具、停车、保存再退出，不以全局 pkill 收尾。
5. 单个正式簇连同收尾最多 1800 仿真秒，正常计划 ≤1440 秒。组件演示按本步命令限制，现实超时独立设置。暂停时现实 watchdog 仍有效；达到时间预算应提前留出停车/保存余量。建图可分段，整套批量验收可远大于 30 分钟现实时间。
6. 自动验收必须同时含正常与失败路径，产出真实图表/可视页面/必要截图和原始指标。GUI 进程启动、文件存在或返回 0 都不能单独证明功能完成。人工未查看时记录 `NOT_CHECKED`，不得替用户点击确认；也不把人工查看设成用户未要求的继续授权门槛。
7. 每步生成 `docs/optimization_v2/SNN/TECHNICAL.md`（技术选择、原理、输入输出、坐标单位、公式、默认参数、优缺点、失败边界）、`CODE_WALKTHROUGH.md`（真实调用链、核心函数与绝对路径/行号、生产代码和上游模块职责）、`MANUAL_CHECK.md`（实际跑过的复制命令、终端数量、可见现象、异常现象、停止恢复）、`RESULTS.md`（命令、退出码、版本/种子、指标、PASS/FAIL/NOT_RUN、已知限制）。本步教程必须有对应实际数据，不能只给泛用科普。
8. 同时交付 `optimization_v2/lessons/SNN/lesson.cpp`、`lesson.py`、`README.md`：一个简小但真实可运行的本步算法/数据流例子，各有正常和边界输入、构建/运行命令与输出解释。生产实现不必强行写两份语言；没有自写 C++ 时写清上游 C++ 的职责，另一语言只作为标注清楚的教学等价例，不冒充生产代码。自写纯配置步骤也需讲解实际读取/状态流。
9. 输出 `optimization_v2/results/SNN/<run_id>/index.html`，离线即可查看实际轨迹/地图/曲线/原因和证据链接；`latest.json` 按执行器合同记录结果、检查项、源码/配置/证据哈希、前置回执哈希。执行失败必须更新失败结果，不能让上一次 PASS 冒充本轮通过。`stepctl check` 仅核对已生成回执，不代替重新跑测试。
10. 默认顺序 01→18。修改公共代码后按依赖图重验受影响步骤；原始证据与计划版本匹配才有效。后续步骤“使用”前序接口通常不改前序源码；确需改动时记录影响，避免静默使整条证据链失效。正常结束后给出本次改动、实测结果、手工验收命令、学习材料链接和剩余事项。

## 任务书目标追踪

| 任务书目标 | 主实现步骤 | 最终验收 |
|---|---|---|
| R01 连续场地、两车与曲线通路 | 02、03、13 | 17 |
| R02 算法三分簇、几何完整 | 11、12 | 17 |
| R03 曲率净空、工作量与 24 分钟计划 | 03、12、13 | 17 |
| R04 传感器定位、偏移与退化 | 04 至 08 | 17 |
| R05 工具反馈、取消、幂等恢复 | 10、14 | 17 |
| R06 点服务 100%、条带有效覆盖 ≥95%、巡检记录 | 12 至 14 | 17 |
| R07 动态障碍、30 分钟内退出、进度保护 | 09、14 至 16 | 17 |
| R08 版本、种子、真值隔离、唯一所有权 | 01 至 18 | 17、18 |

三维环境、传感器融合、建图定位、预测、BT、控制、离线/在线覆盖比较与作业接口都有明确实施与验收落点。阈值是任务书的初始验收目标，不能写成当前已达到的指标。若固定三个簇与工时无法同时满足，应给出不可行原因并显式调整任务配置，不得隐藏未覆盖区域。

## 执行器与证据说明

`optimization_v2/stages/README.md` 定义未来执行器及回执格式；`optimization_v2/plan/steps.json` 为步骤和检查项索引。`NOT_RUN` 表示没有本轮回执，`FAIL/PARTIAL` 表示未全部满足，`STALE` 表示源码/配置/前置证据已变化，`INVALID` 表示回执损坏或不完整。`PASS` 只表示自动证据及一致性检查通过，人工状态另列。哈希检查不是对算法正确性的数学证明，不能替代真实测试和人工查看。

所有步骤的完整 prompt 如下；独立文件同时保存在 `docs/optimization_v2/prompts/S01.md` 至 `S18.md`。复制单步时保留其前置、验收与共同要求引用。

## 执行第 1 步 原项目基线复验与隔离开发环境

请基于原项目实施 S01，目标是：找到真正运行的源码和依赖，保留可复现的原项目入口，建立新的隔离工作区。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：无。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 审计原生入口、两个 ros2_ws 源码树、模型资源、符号链接和路径引用；用文件哈希比较副本，给出唯一修改和构建位置，不凭目录名称判断哪一份有效。

2. 原 run.sh 的 sensor_smoke.py 来自主项目外的 robot_migration_20261108；将仍必要的辅助代码及许可纳入受控目录。task_bt/run.sh 引用的 dynamic_avoidance/run.sh 已缺失：列出影响，修复必要入口或建立有测试的新适配层，不能把旧 BT 文档当运行证据。

3. 建立 optimization_v2/ws/src、env.sh、统一有界启动和停止工具。新环境使用 ROS_DOMAIN_ID=119、独立 GZ_PARTITION；新旧仿真仍共用会话互斥锁，禁止两个实例抢资源。保留原生环境 118 的用法。

4. 采集系统、CPU/GPU/RAM、ROS、Gazebo 和现有组件版本；按后续阶段需要列依赖与锁定策略。RTAB-Map、METIS、Fields2Cover 安装状态逐项核实；F2C 2.1.1 与拟用适配分支的 2.0.0 接口差异做最小编译样例后锁定。不要为列依赖就安装整套深度学习栈。

5. 复验静止农田、模型、传感器 smoke 和适用的原有组件测试；记录当前原始模型的运动限制。为新底盘建立后续参数表，不宣称本步已完成自主作业。安装缺失必要依赖时使用系统包或隔离环境；需要密码时在本机终端认证。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/config/baseline_manifest.json`

- `/home/ceihao/robot_project/optimization_v2/config/dependency_lock.yaml`

- `/home/ceihao/robot_project/optimization_v2/env.sh`

- `/home/ceihao/robot_project/optimization_v2/ws/src`

- `/home/ceihao/robot_project/optimization_v2/stages/S01/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

浏览器查看模型缩略图、传感器频率、依赖状态和外部路径清单；Gazebo 能显示原农田及机器人。缺失的旧 BT 入口必须在报告中明确标红或给出修复验证。

### 必须通过的检查

- **S01-C01**：原生静态传感器 smoke 和所选组件回归实际执行成功；每项有命令、退出码和新证据。

- **S01-C02**：新工作区可在干净 shell 启动；不会加载旧 VM overlay；所有必需项目资源在主文件夹有可追溯来源。

- **S01-C03**：模型和必要辅助脚本的路径闭合；旧 BT 缺失依赖已修复并验证，或明确隔离成不再用于后续的旧入口且替代方案已验证。

- **S01-C04**：唯一源码/构建规则、版本锁和停止机制可用；同时启动第二个仿真实例被拒绝；不杀无关进程。

### 本步技术与代码教学重点

**算法原理**：可复现构建、依赖闭包、源码与构建产物分离、互斥与失败传播。

**C++ 核心逻辑**：阅读现有 task_bt/engine.cpp 的入口与状态传播；用最小 C++ 示例展示配置检查失败如何返回非零退出码。

**Python 核心逻辑**：解释环境装载、依赖清单、哈希和 subprocess 超时收尾；用示例验证缺失文件不能通过准入。

**现有代码参考**：native_20261108/run.sh；native_20261108/scripts/env.sh；native_20261108/task_bt/engine.cpp；ros2_ws/src 和 native_20261108/ros2_ws/src。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 1 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `legacy_farm`**：看到原农田和静止机器人；这不是自主覆盖演示。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S01 --case legacy_farm --seed 41 --sim-seconds 90 --wall-timeout 240
```

时限：最多 90 仿真秒，外层现实超时 240 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `dependency_audit`**：报告列出每个缺失项、处理结果和源码路径。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S01 --case dependency_audit --seed 41 --sim-seconds 30 --wall-timeout 180
```

时限：最多 30 仿真秒，外层现实超时 180 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S01 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1200
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S01
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S01
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S01 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S01
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S01/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S01/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S01/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 2 步 连续不规则三维世界与先验分层

请基于原项目实施 S02，目标是：先看到完整工作场地；区域内部不预置三个独立田块，簇由第 11 步算法产生。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S01。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 用参数化几何生成 Gazebo Harmonic SDF，默认世界约 160×112 m；喷洒区约 1650 m²、割草区约 1140 m²作为首轮候选，允许第 12 步根据工时调整。两区各为连续凹多边形，包含基础、凹口、含洞三个变体。

2. 连接路采用约 6 m 宽曲线通道，保存中心线、边界、检查点和坡度；在第 3 步用真实底盘净空复核，不靠目视宣称两车可通行。场景构造优先 SDF/网格，Blender 仅作可选模型编辑工具。

3. 加入 0.08、0.15、0.30 m 低障碍、1.5–3 m 高障碍、若干不同坡度地形、行人和移动设备。移动视觉模型配匹配的物理碰撞体，使用原项目的临时雷达/相机探针验证可见性与碰撞检测；本步不提前依赖第 4 步整套传感器。

4. 分开发布可用先验与评估真值：先验只含边界、禁行大结构、道路和约定已有的粗地形；临时低障碍、移动者和细地形只经传感器进入在线算法。配置文件和话题也隔离，不能只把名称改成 observation。

5. 输出统一地图原点、单位、版本、种子和资源清单。簇标识在物理世界中不存在；三簇边界不得变成沟渠、围墙或额外地头。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/assets/worlds/irregular_farm.sdf`

- `/home/ceihao/robot_project/optimization_v2/config/scenes.yaml`

- `/home/ceihao/robot_project/optimization_v2/maps/prior/workspaces.geojson`

- `/home/ceihao/robot_project/optimization_v2/maps/prior/connector.geojson`

- `/home/ceihao/robot_project/optimization_v2/evaluation/truth_manifest.json`

- `/home/ceihao/robot_project/optimization_v2/stages/S02/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

Gazebo 俯视与地面视角看到两个连续不规则区域、曲线连接路、坡地和运动物体；HTML 显示“规划先验/传感器待发现/仅评估真值”三层可分别开关的图。

### 必须通过的检查

- **S02-C01**：两个工作区各自连通、边界合法；不存在预划分的三个物理小田块；三个几何变体均可加载。

- **S02-C02**：两区之间存在曲线通路；世界坐标、先验坐标及可视叠加对齐；面积和尺寸从文件计算。

- **S02-C03**：动态物体在固定种子下运动可重现；物理碰撞体与视觉轮廓对齐；雷达命中及相机成像有观测证据。

- **S02-C04**：在线节点配置不读取真值目录或订阅真值话题；仅独立评估进程读取精确障碍/位姿。

### 本步技术与代码教学重点

**算法原理**：三维仿真与二维任务边界、可观测性、数据泄漏、物理模型与渲染模型区别。

**C++ 核心逻辑**：解析 SDF 的 visual/collision 与坐标变换；若无自写 C++ 插件，明确上游 Gazebo 负责物理，学习示例只计算几何包络。

**Python 核心逻辑**：解释多边形生成、种子化物体运动、资源导出与先验/真值分层。

**现有代码参考**：原农田模型与 native_20261108/farm_sensor_smoke.sdf；许可允许时复用模型，避免拷贝整个旧输出目录。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 2 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `world_overview`**：拖动 Gazebo 视角，确认连续区域与连接路，观察行人、移动设备和低障碍。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case world_overview --seed 41 --sim-seconds 180 --wall-timeout 360
```

时限：最多 180 仿真秒，外层现实超时 360 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `prior_vs_unknown`**：浏览器对照先验与场景：临时低障碍不得提前出现在规划先验。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S02 --case prior_vs_unknown --seed 41 --sim-seconds 120 --wall-timeout 300
```

时限：最多 120 仿真秒，外层现实超时 300 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S02 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1200
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S02
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S02
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S02 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S02
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S02/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S02/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S02/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 3 步 差速与阿克曼轻量底盘

请基于原项目实施 S03，目标是：在同一地图分次运行两种固定底盘，得到实际可用的速度、转弯和工具几何参数。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S02。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 为喷洒建立简化差速车，为割草建立真实转向关节的阿克曼车。减少装饰网格，保留正确质量、惯量、碰撞体、轮径、轮距、轴距和轮胎摩擦。原 dino 保留为参考入口。

2. 接入当前版本可用的 Gazebo 驱动系统或 ros2_control，先验证插件接口。差速计算左右轮速；阿克曼从 v、ω 得到曲率/转角并限制转弯半径，低速零除和转向速率需处理。

3. 建立唯一底盘命令仲裁与 watchdog。分级扫速确认停车距离、横向偏差和稳定性后才提高速度；不得通过放宽现有停车门限换取高速通过。

4. 两车分别完成直线、圆弧、S 弯、坡面和急停实验，测试曲线连接路净空及是否会刮碰；记录低速作业速度和转场速度可用区间。

5. 此时定义虚拟工具接口所需外形：喷洒作用半径、停靠误差、服务时长、割草有效宽度和挂载包络。先锁定用于规划的参数，数值由模型验证，不等待机械臂实现。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/robot_description`

- `/home/ceihao/robot_project/optimization_v2/ws/src/chassis_bridge`

- `/home/ceihao/robot_project/optimization_v2/config/vehicles/diff.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/vehicles/ackermann.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/tool_profiles.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S03/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

Gazebo 能分别看到两车走直线、圆和曲线连接路；报告叠加目标轨迹、实际轨迹、轮速/转角、停车距离和各速度下结果。

### 必须通过的检查

- **S03-C01**：差速和阿克曼各自运动学闭环通过；阿克曼不能靠原地旋转或侧滑假装转弯。

- **S03-C02**：两车在连接路实际无碰撞通行；轨迹与转向/轮速曲线有时间对齐记录。

- **S03-C03**：失联、超时、取消、仿真暂停及复位后均进入安全状态；现有停车与反馈门限保留并复验。

- **S03-C04**：选定速度、最小转弯半径、footprint 和工具宽度已固化且有版本；尚未测试的更高速度不得列为可用。

### 本步技术与代码教学重点

**算法原理**：运动学与动力学差别、曲率约束、离散控制、制动与 watchdog。

**C++ 核心逻辑**：驱动桥接、关节命令限幅、单一出口及控制周期；说明哪些来自上游驱动。

**Python 核心逻辑**：复用/对照 kinematics.py 推导差速和阿克曼映射，分析扫速实验及停车曲线。

**现有代码参考**：ros2_ws/src/dino_control/scripts/kinematics.py、drive_node.py、stationary_gate.py；field95 现有停止和所有权测试。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 3 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `diff_connector`**：差速车走曲线道路，终点自动停车；看轨迹误差和轮速。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S03 --case diff_connector --seed 41 --sim-seconds 240 --wall-timeout 480
```

时限：最多 240 仿真秒，外层现实超时 480 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `ackermann_connector`**：阿克曼车经过同一路径，前轮转角随曲率改变，不能原地转向。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S03 --case ackermann_connector --seed 41 --sim-seconds 240 --wall-timeout 480
```

时限：最多 240 仿真秒，外层现实超时 480 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `watchdog_stop`**：演示主动中断命令后停车，报告显示停车时间与距离。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S03 --case watchdog_stop --seed 41 --sim-seconds 90 --wall-timeout 240
```

时限：最多 90 仿真秒，外层现实超时 240 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S03 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S03
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S03
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S03 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S03
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S03/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S03/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S03/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 4 步 多传感器模拟与真实误差注入

请基于原项目实施 S04，目标是：让三维雷达、双目、IMU、轮速和 GNSS 都产生可看的观测，并具备合理误差与失效模式。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S03。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 安装初始配置为 32×1024 点 10 Hz 的 3D 雷达、640×360 15 Hz 双目、200 Hz IMU、100 Hz 轮速、5 Hz GNSS；双目含 CameraInfo/标定基线。按实测资源下调时修改版本并记录，不能悄悄降低验收负载。

2. 建立传感器坐标树、时间戳和 QoS 表；确认真实点云有多条垂直扫描线，不能把 2D scan 拉伸成 3D。轮速测量来自关节运动，不使用世界位姿求导。

3. 在传感器接口注入可配置噪声、偏置、时延和掉帧；IMU 包含偏置漂移，轮速支持滑移，GNSS 支持 fixed/float/dropout 和错误固定解。模拟 GNSS 观测可以由物理真值生成带噪观测，但精确位姿接口只给评分器。

4. 保存传感器标定和消息 schema；原始数据同时分流给定位与感知，分别同步/降采样，避免每个节点自建一套不一致的时基。

5. 提供 RViz 面板和短时记录，测 CPU/GPU/RTF/消息丢失。使用内存/磁盘上限，不生成无界 rosbag。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/sensor_sim_adapter`

- `/home/ceihao/robot_project/optimization_v2/config/sensors.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/fault_profiles.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/rviz/sensors.rviz`

- `/home/ceihao/robot_project/optimization_v2/config/data_contract.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S04/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

同屏查看有高度的点云、左右图像、IMU/轮速时间曲线及 GNSS 轨迹；故障演示中 GNSS 轨迹偏移或中断，诊断栏同时变化。

### 必须通过的检查

- **S04-C01**：五类观测均存在，frame_id、单位、时间戳、协方差、相机标定有效；双目左右同步误差被测量。

- **S04-C02**：稳态发布频率初始目标在配置值 ±10% 内；不满足时明确 FAIL/降配后重验，记录对应 RTF 和掉帧比例。

- **S04-C03**：时延、掉帧、轮滑及 GNSS 错误固定解故障真实改变观测；不是只修改状态标签。

- **S04-C04**：在线模块不能访问精确世界位姿；传感器生成器与独立评分器的真值权限边界有配置和运行审计。

### 本步技术与代码教学重点

**算法原理**：测量模型 z=h(x)+噪声、偏置与白噪声、误差相关性、时间同步与标定。

**C++ 核心逻辑**：PointCloud2/Image/Imu 的消息结构、时间戳和转换；上游传感器插件与自写适配器边界。

**Python 核心逻辑**：带种子的噪声/时延模型、频率与同步统计、有限 rosbag 管理。

**现有代码参考**：ros2_ws/src/dino_sensors/scripts/estimator_io.py、sensor_math.py、scan_filter.py；现有传感器质量测试。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 4 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `sensor_panel`**：RViz 展示三维点云和双目；面板显示 IMU、轮速、GNSS 频率。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S04 --case sensor_panel --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `sensor_faults`**：依次出现 GNSS 中断、偏置和相机掉帧，观测与诊断同时有变化。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S04 --case sensor_faults --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S04 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S04
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S04
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S04 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S04
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S04/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S04/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S04/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 5 步 轮速 IMU 与激光局部里程计

请基于原项目实施 S05，目标是：得到连续 odom 轨迹，并能直观看到轮滑、IMU 偏置或 ICP 退化的影响。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S04。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 采用 robot_localization 局部估计器，使用轮速、IMU 和验证过的纯雷达 ICP 增量；首先验证当前 RTAB-Map ICP 前端，不同时默认启动另一套 ICP。

2. 明确每个输入融合的量及协方差，避免将轮速积分位姿和同源速度当独立测量重复加权；纯 ICP 前端不得再次偷偷融合 IMU/轮速后作为独立观测回灌。

3. 唯一局部估计器发布 odom→base_link；其他节点禁发同一 TF。global/map 修正不得反馈到本步局部滤波造成跳变。

4. 对坡地保留测得的三维姿态，单独派生规划平面位姿；验证重力补偿和坐标约定，不能盲设 two_d_mode 把坡度信息抹掉。

5. 运行闭环线路及轮滑/特征不足案例，输出轮速单独、轮速加 IMU、多源融合三条轨迹，基于独立真值评分而非把真值喂给滤波器。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/config/local_estimator.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/icp_odometry.yaml`

- `/home/ceihao/robot_project/optimization_v2/ws/src/localization_guard`

- `/home/ceihao/robot_project/optimization_v2/config/tf_ownership.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S05/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

RViz 和报告叠加三条里程计轨迹；轮滑处轮速轨迹偏离，融合估计应给出自己的误差与退化状态；能看到 odom 连续而不是被强行拉回真值。

### 必须通过的检查

- **S05-C01**：全程 odom→base_link 只有一个发布者；无 TF 环和时戳倒退；重置时显式新建 epoch。

- **S05-C02**：协方差有限且合法，滤波创新与输入质量可查；重复源被配置检查拒绝。

- **S05-C03**：在固定轮滑用例中融合相对轮速基线的误差和连续性有实测对比；若融合更差先修正原因，不能只凭节点启动通过。

- **S05-C04**：ICP 退化或 IMU 失联时诊断和降速/停止接口触发；独立真值仅用于评分。

### 本步技术与代码教学重点

**算法原理**：误差状态估计、创新门控、可观测性、ICP 退化、相关信息重复融合。

**C++ 核心逻辑**：解释上游滤波器预测/更新、ICP 配准残差与自写健康监督代码，不复制整套 EKF。

**Python 核心逻辑**：轨迹对齐、ATE/RPE 计算、时间插值及轮滑故障注入；教学例对比不同协方差的影响。

**现有代码参考**：现有 estimator_io.py、scan_quality.py 与 navigation 的 TF 故障样例。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 5 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `odometry_loop`**：看三种轨迹闭环误差及 odom 连续性。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S05 --case odometry_loop --seed 41 --sim-seconds 240 --wall-timeout 540
```

时限：最多 240 仿真秒，外层现实超时 540 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `slip_and_icp_loss`**：模拟轮滑与稀疏特征，观察输入创新、退化标志和停止请求。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S05 --case slip_and_icp_loss --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S05 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1500
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S05
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S05
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S05 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S05
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S05/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S05/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S05/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 6 步 三维 SLAM 建图与回环

请基于原项目实施 S06，目标是：从传感器数据真正建立三维地图，展示经过回环后地图和轨迹的变化。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S05。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 将 RTAB-Map 配置为 MAPPING，接收局部里程计和相机/三维点云；相机用于地点识别，雷达用于几何约束，按锁定版本验证同步接口。

2. 设计低速可重复采集线路，穿过两区、连接路和可识别固定结构，设置回访段；可分段建图，不能把单簇 30 分钟限制误当整个建图最多 30 分钟。

3. 实现带操作员可见状态的采集、暂停、继续和正常保存；保持 TF 唯一发布，并区分建图模式与作业模式。

4. 过滤明显动态物体、保留足够稳定结构；对农田重复纹理/低纹理记录回环失败案例，不能用 Gazebo 精确位姿替代配准。

5. 导出数据库、三维点云、位姿图和回环前后对照；质量不足时调整路线/参数再采集，不把原始 SDF 导出当 SLAM 地图。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/config/slam_mapping.yaml`

- `/home/ceihao/robot_project/optimization_v2/maps/mapping_sessions`

- `/home/ceihao/robot_project/optimization_v2/ws/src/map_manager`

- `/home/ceihao/robot_project/optimization_v2/config/routes/mapping_routes.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S06/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

RViz 显示持续增长的三维地图，能看到树干、障碍和坡地高度；报告可切换回环前/后轨迹并标出回环约束。

### 必须通过的检查

- **S06-C01**：数据库由本轮传感器采集生成，可重开；点云具有真实高度信息，地图不来自直接复制世界模型。

- **S06-C02**：至少一条有效回访路线产生经检查的回环约束；错误回环测试被拒绝或触发诊断。

- **S06-C03**：输出可量化的地图重影/对齐误差和轨迹误差；动态物体残留有标注，不虚称完整消除。

- **S06-C04**：采集可正常停止/保存/恢复，地图与传感器配置、标定、版本绑定。

### 本步技术与代码教学重点

**算法原理**：前端里程计与后端图优化、地点识别、回环验证、漂移与动态残影。

**C++ 核心逻辑**：RTAB-Map 的配准约束与图优化调用链；自写 map_manager 生命周期和保存服务。

**Python 核心逻辑**：采集路线、地图导出和回环前后误差统计；小型位姿图教学例。

**现有代码参考**：ros2_ws/src/dino_slam/scripts/map_contract.py、export_map.py、bounded_launch.py 的契约和收尾方式。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 6 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `mapping_loop`**：看地图从无到有和回访后的约束；旋转 RViz 视角确认三维高度。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S06 --case mapping_loop --seed 41 --sim-seconds 600 --wall-timeout 1200
```

时限：最多 600 仿真秒，外层现实超时 1200 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `loop_comparison`**：打开实际采集数据的回环前后对照与位姿图，不重新跑完整建图。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S06 --case loop_comparison --seed 41 --sim-seconds 120 --wall-timeout 300
```

时限：最多 120 仿真秒，外层现实超时 300 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S06 --suite acceptance --seed 41 --sim-seconds 900 --wall-timeout 3600
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S06
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S06
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S06 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S06
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S06/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S06/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S06/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 7 步 冻结地图与重启重定位

请基于原项目实施 S07，目标是：保存后关闭系统，再从不同初始位置启动并在冻结地图中恢复定位。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S06。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 导出三维地图、供导航使用的二维占据层/粗地形、坐标原点与文件校验值；定义地图 manifest 与校准版本。地图构建和任务先验边界分开管理。

2. 切换到 LOCALIZATION，冻结长期地图；临时障碍留在局部层。RTAB-Map 输出地图定位观测，全局纠偏器成为作业模式 map→odom 的唯一发布者。

3. 实现初始化、定位中、可靠、退化、丢失状态。未可靠定位时不能进入自动作业，允许低速受控重定位流程但不默认盲目运动。

4. 完成关闭重开、不同初始位置、遮挡、错误初始位姿和地图版本变化试验；重定位成功要看传感器匹配和误差，不只看 TF 存在。

5. 建立地图版本准入：本步用计划契约 fixture 验证旧地图 ID 被拒绝；第 12 步产生真实计划后再做端到端重验。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/maps/frozen/manifest.json`

- `/home/ceihao/robot_project/optimization_v2/maps/frozen/map.db`

- `/home/ceihao/robot_project/optimization_v2/config/slam_localization.yaml`

- `/home/ceihao/robot_project/optimization_v2/ws/src/global_pose_manager`

- `/home/ceihao/robot_project/optimization_v2/config/map_contract.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S07/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

重启后地图保持不变，机器人轨迹重新对齐；界面显示 INITIALIZING→LOCALIZED。注入错误地图版本时出现明确拒绝原因。

### 必须通过的检查

- **S07-C01**：至少三个规定初始位姿可重定位；导出位置/航向误差和恢复耗时，不能以真值直接初始化成功冒充重定位。

- **S07-C02**：冻结地图文件不被在线定位修改；map→odom 与 odom→base_link 各只有唯一发布者。

- **S07-C03**：定位丢失时动作准入关闭，恢复有质量门槛和迟滞，不能一帧成功立即全速运行。

- **S07-C04**：地图/标定/计划版本不一致时拒绝执行；契约样例与后来真实计划复验事项写清。

### 本步技术与代码教学重点

**算法原理**：建图与定位分离、地图版本、全局校正与局部连续性、重定位置信度。

**C++ 核心逻辑**：全局观测转 map→odom、生命周期与准入状态机；上游定位与自写所有权逻辑分清。

**Python 核心逻辑**：地图导出、manifest 哈希、重启用例驱动与恢复时间统计。

**现有代码参考**：map_contract.py 与原有 TF identity、binding guard 测试。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 7 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `restart_relocalize`**：演示停止定位节点、重启、换初始位姿后恢复；看状态与轨迹。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S07 --case restart_relocalize --seed 41 --sim-seconds 300 --wall-timeout 660
```

时限：最多 300 仿真秒，外层现实超时 660 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `wrong_map`**：加载错误版本 fixture，应显示版本不匹配且禁止开始任务。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S07 --case wrong_map --seed 41 --sim-seconds 90 --wall-timeout 240
```

时限：最多 90 仿真秒，外层现实超时 240 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S07 --suite acceptance --seed 41 --sim-seconds 600 --wall-timeout 1800
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S07
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S07
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S07 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S07
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S07/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S07/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S07/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 8 步 GNSS 配准与定位偏移监督

请基于原项目实施 S08，目标是：在卫星定位与地图定位之间做有质量依据的选择，能解释偏移和故障时为何停或降级。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S07。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 定义 geodetic→ENU→map 的配准、datum 和航向来源；经验证再使用 navsat_transform 或等价适配，不把经纬度直接当米。

2. 全局纠偏器融合/选择 RTAB-Map 地图观测与 GNSS，记录相关性假设。GNSS 不同时作为 RTAB-Map 强先验再作为独立检查重复使用。

3. 加入 GNSS fixed/float/错误 fixed、协方差、创新残差、地图匹配质量和时间新鲜度检查。状态切换使用迟滞和连续证据。

4. 在位置误差、航向误差或时间异常超过已批准限制时请求降速/停机；恢复前重新验证，不靠持续拉扯 TF 掩盖跳变。

5. 做正常、GNSS 遮挡、多径偏移、地图定位退化、两者同时失效的对照；显示真值误差只在评估视图。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/config/gnss_alignment.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/localization_supervisor.yaml`

- `/home/ceihao/robot_project/optimization_v2/ws/src/global_pose_manager`

- `/home/ceihao/robot_project/optimization_v2/config/fault_cases/localization.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S08/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

报告叠加 GNSS、地图定位、最终轨迹，时间轴显示信任源、创新残差与状态；错误 fixed 场景不能继续显示“可靠”并全速行驶。

### 必须通过的检查

- **S08-C01**：基础场景位置误差 P95 ≤0.15 m、航向误差 P95 ≤0.05 rad 的任务书初始目标达标；未达标明确 FAIL 并分析。

- **S08-C02**：固定解标签不能绕过创新检验；GNSS 遮挡、虚假固定解、地图失配及双失效均触发预期状态。

- **S08-C03**：map→odom 唯一发布，全局观测不回灌局部 EKF；坐标转换与协方差变换有已知样例校验。

- **S08-C04**：状态/阈值变化、停车请求与恢复依据可在同一时间轴查验；保留原跳变和停止门限。

### 本步技术与代码教学重点

**算法原理**：坐标配准、创新检验、协方差、可靠性监督与冗余源相关性。

**C++ 核心逻辑**：质量门控、残差检验、单一 TF 发布、状态迟滞的核心回调。

**Python 核心逻辑**：ENU 配准样例、误差统计、故障时序生成及状态解释图。

**现有代码参考**：原 estimator guard、scan quality、tf_fault_proxy.py 和定位切换测试。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 8 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `gnss_outage`**：GNSS 丢失后地图定位接管；观察质量变化和恢复迟滞。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S08 --case gnss_outage --seed 41 --sim-seconds 240 --wall-timeout 540
```

时限：最多 240 仿真秒，外层现实超时 540 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `false_fixed_and_loss`**：错误固定解被拒；双源失效进入停车状态。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S08 --case false_fixed_and_loss --seed 41 --sim-seconds 240 --wall-timeout 540
```

时限：最多 240 仿真秒，外层现实超时 540 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S08 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S08
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S08
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S08 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S08
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S08/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S08/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S08/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 9 步 三维感知融合与二维通行代价

请基于原项目实施 S09，目标是：将同一批三维传感器观测转成车辆能使用的地形、低障碍、动态目标及二维代价。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S08。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 建立点云时间校正、体素降采样、地面/非地面分离、高程/坡度/台阶估计；基于底盘离地间隙、倾斜稳定性和工具包络判断可通行，不使用统一高度阈值代替全部车辆能力。

2. 双目/图像用于行人、设备等类别和几何互证；采用先验证可部署的轻量开源检测器，记录许可证、模型来源和本机延迟。几何避障不得等分类结果到来才停车。

3. 将图像框按标定和时间与点云关联，输出具有不确定度的目标状态、尺寸、类别及稳定轨迹 ID；ID 只表示当前轨迹关联，不等于永久身份。

4. 向 Nav2 提供高程风险转换的 2D costmap/voxel 投影和实时障碍层，设置清除、持久时间和遮挡规则。定位读取稳定特征，避障保留临时物体，两条支路独立参数。

5. 验证三个高度低障碍、坡面、遮挡行人和突然移动目标；标注超出传感器可见范围的情况，保持减速/停车兜底。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/terrain_perception`

- `/home/ceihao/robot_project/optimization_v2/ws/src/object_fusion`

- `/home/ceihao/robot_project/optimization_v2/config/traversability.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/costmap.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/detector_lock.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S09/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

RViz 同时显示原始点云、地面色图、目标框/轨迹和二维 costmap；临时低障碍被感知后才出现在在线代价层，坡度风险随底盘配置改变。

### 必须通过的检查

- **S09-C01**：0.08/0.15/0.30 m 可见测试物均有明确检测与可通行判定结果；不可见/漏检不能记成正确检测。

- **S09-C02**：坡度、台阶和工具包络按两底盘配置转换；边界/高障碍/动态障碍能阻止危险通行。

- **S09-C03**：目标消息有时戳、位置、速度、尺寸、类别、不确定度与关联 ID；遮挡再现和误关联有固定测试。

- **S09-C04**：移除临时物体后代价按规则清除，不残留永久幽灵障碍；相机失效时几何避障继续或明确降级。

### 本步技术与代码教学重点

**算法原理**：2.5D 高程与 2D 导航、几何和语义融合、关联与遮挡、延迟和不确定度。

**C++ 核心逻辑**：点云处理、地形风险层、时序关联回调和 Nav2 层更新；说明上游 PCL/代价层调用。

**Python 核心逻辑**：模型推理适配、标定投影、检测/关联评估和数据回放。

**现有代码参考**：原 scan_filter.py、scan_quality.py 的质量门控；避免照搬只适合 2D scan 的障碍规则。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 9 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `low_obstacles_and_slope`**：看点云和代价图中低障碍、坡地的变化；切换底盘配置比较通行判断。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S09 --case low_obstacles_and_slope --seed 41 --sim-seconds 240 --wall-timeout 540
```

时限：最多 240 仿真秒，外层现实超时 540 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `occluded_pedestrian`**：行人从遮挡后出现，看到框、点云、轨迹 ID 与代价更新。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S09 --case occluded_pedestrian --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S09 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S09
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S09
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S09 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S09
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S09/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S09/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S09/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 10 步 喷洒割草抽象动作与有效作业账本

请基于原项目实施 S10，目标是：不建机械臂，通过带反馈的抽象动作实现真实的成功、部分完成、失败与恢复语义。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S08, S09。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 定义统一任务 action：POINT_SERVICE、START_WORK、STOP_WORK，包含 task_id、map/plan/tool 版本、目标、幂等键、时限和取消。反馈包含进度、有效作业量、健康度；最终结果含残余和错误码。

2. 喷洒仅在到位、速度足够低、定位可靠及目标满足条件时开始；模拟作用范围与驻留过程。割草仅在许可速度/工具状态/路径段上累计有效条带，转场和地头非工作段不能误计。

3. 工具动作适配独立于机械臂实现，后续机械臂/近景相机/VLA 只替换 action server 或内部适配。仍要使用第 3 步虚拟工具几何，不能省略宽度、包络和反馈延迟。

4. 复用现有实测账本/反馈思想，按确认反馈记账；实现重复请求、取消、半完成、失联、重启恢复。独立评分器用仿真物理状态评估实际作业，在线账本只接收工具观测反馈。

5. 预留机械臂隔离接口规范、错误码、版本和能力查询；提供模拟服务器与客户端，不安装 MoveIt/VLA 来扩大本轮范围。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/mission_interfaces/action`

- `/home/ceihao/robot_project/optimization_v2/ws/src/tool_sim`

- `/home/ceihao/robot_project/optimization_v2/ws/src/work_ledger`

- `/home/ceihao/robot_project/optimization_v2/config/tool_contract.yaml`

- `/home/ceihao/robot_project/optimization_v2/contracts/future_tool_adapter.md`

- `/home/ceihao/robot_project/optimization_v2/stages/S10/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

喷洒点从待执行变为进行中、成功或部分；割草足迹逐步着色；故障后能看到未完成点/残余条带，重复请求不会重复计数。

### 必须通过的检查

- **S10-C01**：成功、部分、失败、取消、失联、重复请求六种状态均有可运行用例和明确终态。

- **S10-C02**：动作受理不等于完成；位置/速度/定位/工具健康不满足时不能累积有效工作量。

- **S10-C03**：重复幂等键不重复执行已确认工作；重启后已完成和未完成内容可恢复。

- **S10-C04**：独立评分与在线账本分离；反馈时间戳/版本错误被拒绝；接口无需依赖真实机械臂。

### 本步技术与代码教学重点

**算法原理**：请求/执行/完成区分、幂等性、持久化事务、反馈驱动作业计量。

**C++ 核心逻辑**：ROS action 服务端生命周期、取消和互斥；工具准入与唯一命令出口。

**Python 核心逻辑**：账本事务、幂等键、区域并集与残余计算；模拟故障客户端。

**现有代码参考**：field95/measured_ledger.py、tool_ingress.py、feedback.py、protocol.py 与恢复/工具交接测试。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 10 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `point_service_states`**：看喷洒驻留、成功与部分状态；模拟失联后剩余任务保留。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S10 --case point_service_states --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `continuous_work_and_retry`**：看割草足迹和重复请求，已完成面积不能二次累加。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S10 --case continuous_work_and_retry --seed 41 --sim-seconds 240 --wall-timeout 540
```

时限：最多 240 仿真秒，外层现实超时 540 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S10 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1500
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S10
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S10
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S10 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S10
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S10/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S10/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S10/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 11 步 按工作量自动划分三个连通簇

请基于原项目实施 S11，目标是：从整个不规则工作区计算三个相邻连通簇，先证明几何完整性，再由下一步验证真实工时。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S02, S03, S10。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 将喷洒区可执行栅格/候选点、割草区可作业单元转为邻接图；明确孔洞、永久禁区和机体/工具边界处理。每个任务单元只能归属一次。

2. 节点权重表示初估服务/行驶/转弯工作量；边权表示不希望切开的共享边或连续条带关系，不把行驶时间直接当切割惩罚。使用固定种子的 METIS k-way 候选三分。

3. 修复并验证图连通和几何连通，保证每区恰好三个非空簇且簇邻接图连通；不画隔离带、不在簇界新增地头、不预设统一形状。

4. 对基础、凹口、含洞变体做几何检查；同时实现简单 K-means 加连通修复作参照，保留 DARP 接口和适用假设。比较最大负载、切边、连通性和计算时间。

5. 输出状态 PROVISIONAL 与初估工时，明确最终工时尚未验收；第 12 步真实路线回算后再调整边界。k 在配置中可扩展，但本实验固定 3，不可行时不能悄改为 4。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/coverage_partition`

- `/home/ceihao/robot_project/optimization_v2/config/partition.yaml`

- `/home/ceihao/robot_project/optimization_v2/plans/candidate/zone_a_clusters.geojson`

- `/home/ceihao/robot_project/optimization_v2/plans/candidate/zone_b_clusters.geojson`

- `/home/ceihao/robot_project/optimization_v2/plans/candidate/partition_manifest.json`

- `/home/ceihao/robot_project/optimization_v2/stages/S11/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

浏览器从未划分的完整区域一键切换为三色分簇，显示邻接边、每簇工作量和检查结果；换种子/几何变体可看边界由算法变化。

### 必须通过的检查

- **S11-C01**：每区恰好三个非空簇，各自图/几何连通，簇邻接图连通；边界只是逻辑标识。

- **S11-C02**：相对可作业域的并集差面积比例 ≤10⁻⁶；重叠和遗漏分别检查；所有点/单元唯一归属。

- **S11-C03**：基础、凹口、含洞三类几何有结果；固定种子可复现；不存在手写三块坐标假装算法分区。

- **S11-C04**：METIS 与 K-means 修复对照可运行；报告初估耗时而非声称已满足 24/30 分钟。

### 本步技术与代码教学重点

**算法原理**：图划分与聚类区别、负载平衡、连通约束、切边代价和可行性检验。

**C++ 核心逻辑**：METIS 调用的 CSR 图、节点/边权与结果校验；若绑定在 Python 中，C++ 给真实上游调用说明和最小样例。

**Python 核心逻辑**：Shapely 几何、任务单元图、连通修复、面积完整性和交互图输出。

**现有代码参考**：field95/planning.py、plan_cli.py 及本地主文件夹内已迁移的离线研究代码；使用前核实路径，不能重新依赖新加卷。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 11 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `partition_base`**：显示两区的自动三色分簇、面积、邻接和初估负载。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S11 --case partition_base --seed 41 --sim-seconds 120 --wall-timeout 300
```

时限：最多 120 仿真秒，外层现实超时 300 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `partition_variants`**：切换凹口/含洞与种子，观察算法生成的不同边界及连通检查。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S11 --case partition_variants --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S11 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1200
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S11
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S11
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S11 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S11
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S11/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S11/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S11/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 12 步 点服务和条带离线路径与工时闭环

请基于原项目实施 S12，目标是：生成真正可执行的两类覆盖路线，并用实际几何、转弯和工具时间修正三簇边界。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S11。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 喷洒按可达栅格中心建立任务点，使用障碍感知的路径代价矩阵求 TSP 顺序；NN+2-opt 作基线，OR-Tools 作候选。不能用欧氏距离穿越禁区或孔洞。不可达点明确报出。

2. 割草使用已验证版本的 Fields2Cover 生成方向、条带、整区地头和连接曲线；处理凹多边形、孔洞、工具宽度和阿克曼最小半径。簇界不新增物理地头，允许安全转场但工具关闭。

3. 将计划分为工作段、转场段、地头段、连接路和停靠动作，计算每段长度、曲率、限速与加减速，累计驻留、工具切换和安全余量。默认名义计划每簇 ≤1440 仿真秒，为等待/恢复留余量。

4. 把真实路径时间反馈给第 11 步划分器，做有界边界交换/重分区，重新验证几何完整和三簇约束。不满足时返回 INFEASIBLE 与原因，允许显式改区域尺寸/任务密度并重新版本化，不能删点或缩小评分分母。

5. 导出版本绑定的 VALIDATED 计划；过期地图/车辆/工具版本拒绝准入。设计离线全局计划+在线局部修补作为主线，并定义纯在线覆盖对照，最终优劣留给第 17 步数据。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/coverage_planner`

- `/home/ceihao/robot_project/optimization_v2/plans/validated/mission_plan.json`

- `/home/ceihao/robot_project/optimization_v2/plans/validated/routes.geojson`

- `/home/ceihao/robot_project/optimization_v2/plans/validated/time_budget.csv`

- `/home/ceihao/robot_project/optimization_v2/config/planning.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S12/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

浏览器能逐段播放喷洒访问顺序和割草条带/转弯；侧栏显示每簇工时构成、最小净空和曲率；不可行案例显示失败位置和缺口。

### 必须通过的检查

- **S12-C01**：喷洒全部可服务点恰好有计划，割草的外边界/地头/边角和残余均被显式计算；不可达不冒充完成。

- **S12-C02**：每段碰撞净空、工具包络、曲率、速度及加减速可行；禁用对阿克曼不可行的原地掉头。

- **S12-C03**：三簇的名义计划各 ≤1440 仿真秒，计划生成有迭代上限；极端配置能正确返回不可行。

- **S12-C04**：实际路线回算后的簇仍满足 S11 几何约束；地图/机器人/工具/计划哈希绑定，改变任一关键版本时拒绝旧计划。

### 本步技术与代码教学重点

**算法原理**：覆盖与访问顺序、运动约束路径、时间负载闭环、离线主计划与在线修补。

**C++ 核心逻辑**：Fields2Cover 的边界→地头→条带→连接路径调用链与曲率/净空检查。

**Python 核心逻辑**：TSP 代价矩阵、OR-Tools/2-opt、时间预算和边界优化循环。

**现有代码参考**：field95/planning.py、plan_cli.py、test_offline.py、test_strips.py；原离线路径研究只借鉴算法，不复制历史日志。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 12 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `plan_animation`**：播放两区路线，看点停靠与连续条带/地头；查看每簇工时条形图。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S12 --case plan_animation --seed 41 --sim-seconds 240 --wall-timeout 540
```

时限：最多 240 仿真秒，外层现实超时 540 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `infeasible_and_stale`**：过大任务或过小转弯空间应报不可行；旧地图计划应被拒绝。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S12 --case infeasible_and_stale --seed 41 --sim-seconds 120 --wall-timeout 300
```

时限：最多 120 仿真秒，外层现实超时 300 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S12 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 1800
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S12
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S12
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S12 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S12
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S12/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S12/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S12/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 13 步 Nav2 控制器与三类导航动作

请基于原项目实施 S13，目标是：让两车真正执行规划轨迹，并区分点到点转场、连续覆盖和道路巡检。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S08, S09, S12。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 差速默认 DWB，阿克曼默认 RPP；加入符合各底盘运动模型的 MPPI 对照配置。检查当前 Nav2 版本的模型/参数，禁止把差速模型直接套到阿克曼。

2. 喷洒到点使用 NavigateToPose，由全局规划/局部控制到达服务位姿；割草与连接路精确巡检使用分段 FollowPath，带路径有效性监测。FollowPath 本身不负责全局重新规划，交由任务层决定。

3. 控制前检查计划、定位、代价图、工具状态和命令租约；保留唯一底盘出口，速度平滑/限幅与独立安全停止不会产生第二命令源。

4. 用同路线/速度/种子比较跟踪误差、停车、耗时和资源，分别为两底盘选择默认值。动态障碍先用现有几何感知的保守停/绕基线，不假装已接入第 15 步预测。

5. 做取消、抢占、换控制器、定位失效和连接路巡检；暂停覆盖时保留路径进度，不能盲目重发整条路径造成重复作业。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/config/nav2_diff.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/nav2_ackermann.yaml`

- `/home/ceihao/robot_project/optimization_v2/ws/src/navigation_adapter`

- `/home/ceihao/robot_project/optimization_v2/config/inspection_route.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/controller_profiles.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S13/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

Gazebo/RViz 看两车分别跟踪覆盖线和连接路；报告对比 DWB/RPP/MPPI 的轨迹、速度/角速度/转角、误差与控制耗时。

### 必须通过的检查

- **S13-C01**：差速 DWB、阿克曼 RPP 及适配 MPPI 各有闭环测试；输出符合对应底盘曲率/速度限制。

- **S13-C02**：NavigateToPose 和 FollowPath 的到点、取消、失败、重启语义可查；动作和底盘所有权唯一。

- **S13-C03**：任务书及原项目适用跟踪/停止门限通过；建议 20 Hz 控制、P95 计算时延 <40 ms 的目标单独实测，不用降速掩盖超时。

- **S13-C04**：两车连接路巡检均可完成规定路段/检查点并保存观测记录；异常时停止且保留进度。

### 本步技术与代码教学重点

**算法原理**：路径跟踪与目标导航差别、采样控制、纯追踪、约束模型预测与控制时限。

**C++ 核心逻辑**：上游 DWB/RPP/MPPI 核心代价与运动模型、自写导航适配与命令仲裁；避免无必要重写控制器。

**Python 核心逻辑**：动作客户端、路线切片、控制器对照评估、进度与检查点记录。

**现有代码参考**：ros2_ws/src/dino_navigation/scripts/arbiter.py、execute_path.py；field95/mppi_adapter/controller.cpp 和 hard_optimizer.hpp。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 13 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `diff_point_and_inspect`**：差速车到点再巡检连接路，看动作切换和检查点记录。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S13 --case diff_point_and_inspect --seed 41 --sim-seconds 420 --wall-timeout 900
```

时限：最多 420 仿真秒，外层现实超时 900 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `ackermann_follow_path`**：阿克曼车按条带曲线行驶，显示曲率和转角；取消后安全停车。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S13 --case ackermann_follow_path --seed 41 --sim-seconds 420 --wall-timeout 900
```

时限：最多 420 仿真秒，外层现实超时 900 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S13 --suite acceptance --seed 41 --sim-seconds 600 --wall-timeout 3600
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S13
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S13
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S13 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S13
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S13/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S13/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S13/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 14 步 可解释行为树与完整作业闭环

请基于原项目实施 S14，目标是：把定位、导航、工具和账本组成能解释等待、重规划和换簇理由的完整任务。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S10, S13。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 建立分层 BT：任务/簇调度层、导航与工具执行层、独立安全监督。复用 BehaviorTree.CPP，但把条件输入、黑板项和动作所有权显式列出。

2. 等待、局部绕行、局部补扫、全局重规划、延后点/簇使用可解释条件与成本：阻塞持续时间、剩余预算、预测/观测风险、绕行成本和工具状态；设置迟滞、最大重试与总预算。第 15 步预测未接入时明确使用当前观测基线。

3. 每次决策输出 reason_code、证据值、阈值、被选/被拒行动及成本，生成时间轴和 BT 活跃节点视图；不能只写 Replan/Wait 文本标签。

4. 执行点服务到达→准入→驻留→反馈入账→下一点；割草按段启停工具并记录残余；巡检连接路时停止作业工具。跨簇调度要保留原簇未完成状态。

5. 先跑几分钟微型冒烟，再分别跑一个完整喷洒簇和割草簇；短片段标记 SMOKE，不代替完整簇验收。剩余预算不足时受控停止并导出 PARTIAL，单次运行不超过 1800 仿真秒。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/behavior_trees/mission.xml`

- `/home/ceihao/robot_project/optimization_v2/ws/src/mission_bt`

- `/home/ceihao/robot_project/optimization_v2/config/decision_policy.yaml`

- `/home/ceihao/robot_project/optimization_v2/contracts/decision_reason_codes.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/nominal_missions.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S14/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

看 BT 高亮与车辆动作同步：短暂封堵等待、持续封堵重新规划、工具故障暂停；时间轴解释每次选择，地图显示实际完成及剩余。

### 必须通过的检查

- **S14-C01**：等待/重规划/补扫/延后/停止均有触发与退出条件、迟滞和重试上限；无无限恢复循环。

- **S14-C02**：每次关键决策都有输入证据和 reason_code，数值与日志能对应，取消/抢占时唯一动作及工具所有权保持。

- **S14-C03**：完整正常喷洒簇反馈确认完成率 100%；完整正常割草簇有效覆盖 ≥95% 且残余全部导出；每簇 ≤1800 仿真秒。

- **S14-C04**：超预算/持续封堵/工具失联正确输出 PARTIAL 或失败；未完成不能靠换簇或缩小分母记成完成。

### 本步技术与代码教学重点

**算法原理**：分层行为树、反应式监督、行动成本和效用、迟滞与有界恢复。

**C++ 核心逻辑**：BT 异步 Action、Condition、halt/取消、黑板与事件发布；关键决策 guard 和预算检查。

**Python 核心逻辑**：任务调度、成本比较、账本恢复和可解释时间轴可视化。

**现有代码参考**：native_20261108/task_bt/engine.cpp 的结构思想与 field95 的 wait、dispatch、handoff、persistence 用例；不调用缺失旧启动器。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 14 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `decision_story`**：看等待、绕行/重规划和停止的原因时间轴；此案例只是 SMOKE。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case decision_story --seed 41 --sim-seconds 300 --wall-timeout 720
```

时限：最多 300 仿真秒，外层现实超时 720 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `nominal_spray_cluster`**：完整喷洒簇跑完，核对所有点及仿真耗时。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case nominal_spray_cluster --seed 41 --sim-seconds 1800 --wall-timeout 2700
```

时限：最多 1800 仿真秒，外层现实超时 2700 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `nominal_mow_cluster`**：完整割草簇跑完，查看有效覆盖热图和残余。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S14 --case nominal_mow_cluster --seed 41 --sim-seconds 1800 --wall-timeout 2700
```

时限：最多 1800 仿真秒，外层现实超时 2700 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S14 --suite acceptance --seed 41 --sim-seconds 1800 --wall-timeout 7200
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S14
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S14
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S14 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S14
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S14/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S14/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S14/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 15 步 动态障碍多模态预测与风险接入

请基于原项目实施 S15，目标是：展示多个合理未来轨迹及其不确定度，并验证预测是否实际改善决策。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S14。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 先使用跟踪结果建立 IMM 多运动模式基线（如匀速/加速/转弯），再接入任务书指定的 Trajectron++ 候选适配；固定代码/权重/许可和隔离依赖，不能仅改名包装 EKF 恒速。

2. 以在线检测关联轨迹、邻居和地图可用特征输入预测，输出 2–4 秒可配置时域的多模态轨迹概率/协方差；不能读世界脚本中未来路径。

3. 构建仿真观测数据集，按场景/轨迹拆分训练、验证、测试，隔离未来标签。评估转弯、停走、交互让行和遮挡，先检查预训练模型域差异。

4. 将未来占据和冲突风险交给 BT 风险/等待评估与速度限制；未验证时不直接声称原生 Nav2 costmap 可表达完整时间维，必要时使用有明确接口的预测风险监督器。安全停车独立于预测。

5. 对照无预测、IMM、学习模型的 ADE/FDE、概率校准、风险漏报、端到端等待/重规划/完成率及算力。学习模型未过门槛可保持 IMM 默认，并提交真实失败/禁用结论；学习模型适配和可复现实验仍须完成。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/motion_prediction`

- `/home/ceihao/robot_project/optimization_v2/config/prediction.yaml`

- `/home/ceihao/robot_project/optimization_v2/config/model_lock.yaml`

- `/home/ceihao/robot_project/optimization_v2/datasets/prediction/manifest.json`

- `/home/ceihao/robot_project/optimization_v2/ws/src/prediction_risk_guard`

- `/home/ceihao/robot_project/optimization_v2/stages/S15/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

RViz 显示当前检测轨迹、未来多条预测及概率；行人突然转弯时预测分布变化。报告可切换 IMM/学习模型并显示误差、等待和算力差异。

### 必须通过的检查

- **S15-C01**：预测输入来自在线轨迹；未来标签仅在离线训练/独立评分中使用；训练测试分场景无泄漏。

- **S15-C02**：IMM 与学习模型适配均完成可重复推理测试，转弯/停走案例输出非单一恒速未来及不确定度。

- **S15-C03**：闭环 A/B 记录安全、任务效率和算力；默认启用哪个模型由预注册门槛决定，未达标模型标记 disabled。

- **S15-C04**：预测超时/失联/大不确定度退回安全基线；预测不能绕过当前障碍或独立停车条件。

### 本步技术与代码教学重点

**算法原理**：多模型与多模态预测、交互条件、预测概率、域差异与风险而非盲从均值。

**C++ 核心逻辑**：风险走廊/时空相交检查、超时 guard 与 BT 输入；上游推理 C++ 不存在时不杜撰。

**Python 核心逻辑**：轨迹预处理、IMM、Trajectron++ 推理适配、数据划分和 ADE/FDE 评估。

**现有代码参考**：第 9 步轨迹接口和第 14 步 reason_code；现有研究数据仅在许可及 schema 符合时复用。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 15 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `prediction_turning`**：看行人转弯的多条未来轨迹，比较 IMM 和学习模型。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S15 --case prediction_turning --seed 41 --sim-seconds 240 --wall-timeout 600
```

时限：最多 240 仿真秒，外层现实超时 600 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `prediction_timeout`**：预测失联时保持感知和安全停车，BT 给出降级原因。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S15 --case prediction_timeout --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S15 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 2400
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S15
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S15
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S15 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S15
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S15/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S15/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S15/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 16 步 经验检索缓存与算力收益对照

请基于原项目实施 S16，目标是：检验重复特征场景是否值得使用经验库，而不是预设“记住物体一定更省算力”。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S15。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 构造尺寸、类别、速度历史、运动模式、道路关系、邻居和定位/预测质量等上下文特征；缓存行为原型/模型选择或计算触发策略，不缓存过期绝对位置，也不把轨迹 ID 当永久身份。

2. 使用 SQLite 有界经验库，加入版本、时间衰减、TTL、相似度门槛和检索失败回退；地图/模型版本变化或分布外输入不能强行命中。

3. 命中后仍做实时几何/风险检验。缓存不能直接强迫车辆等待/通过，最终决策仍需当前证据和独立安全层。

4. 公平比较三组：每帧完整预测、相同事件触发但无缓存、事件触发加缓存。冻结评估数据/库快照、统一触发与种子，单独计特征提取、查询和维护开销。

5. 输出准确性、缓存命中、误命中、预测计算量、CPU/GPU、延迟、任务收益；无净收益或风险退化则默认关闭。保留小规模数据比对功能，不保留无限历史原始视频。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/ws/src/experience_memory`

- `/home/ceihao/robot_project/optimization_v2/config/memory.yaml`

- `/home/ceihao/robot_project/optimization_v2/contracts/experience_schema.sql`

- `/home/ceihao/robot_project/optimization_v2/evaluation/memory_ablation.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S16/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

报告显示三组相同场景的计算时间和任务效果，逐次检索可看命中特征及被拒原因；相似外形但反向运动必须能被区分或拒绝。

### 必须通过的检查

- **S16-C01**：三组公平对照实际运行，含检索/维护开销，报告净收益而非只统计少调用几次模型。

- **S16-C02**：过期、地图/模型版本变化、错误相似物和分布外输入能正确失效/回退；缓存不能绕过安全检查。

- **S16-C03**：在线库初始上限 100 MB，按可审计策略淘汰；记录增长与重启恢复结果。

- **S16-C04**：实验输出 enable/disable 建议及证据；若无收益可判本步实验完成，但必须默认禁用，不能宣称算法有效。

### 本步技术与代码教学重点

**算法原理**：记忆与实时估计边界、事件触发计算、缓存失效、消融实验和净收益。

**C++ 核心逻辑**：在线匹配准入、当前风险复核与超时回退；解释是否采用 Python 服务而非硬加 C++ 数据库层。

**Python 核心逻辑**：SQLite 事务、特征检索、TTL、冻结快照、配对对照与性能统计。

**现有代码参考**：第 15 步预测接口，第 14 步决策解释，field95 持久化/幂等设计。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 16 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `memory_three_way`**：看三组总耗时、命中率和风险指标，检查净收益是否成立。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S16 --case memory_three_way --seed 41 --sim-seconds 300 --wall-timeout 900
```

时限：最多 300 仿真秒，外层现实超时 900 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `memory_wrong_match`**：相似外形不同方向/版本的物体被拒用，系统重新预测。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S16 --case memory_wrong_match --seed 41 --sim-seconds 180 --wall-timeout 420
```

时限：最多 180 仿真秒，外层现实超时 420 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S16 --suite acceptance --seed 41 --sim-seconds 300 --wall-timeout 2400
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S16
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S16
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S16 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S16
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S16/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S16/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S16/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 17 步 六簇任务与故障消融验收

请基于原项目实施 S17，目标是：证明全部六簇、两种底盘和连接路在规定场景中能工作，并公开失败与残余。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S14, S15, S16。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 建立版本固定的验证矩阵：两区各三簇、两底盘分别运行、基础/凹口/含洞几何及规定故障。主场景六簇每簇至少 10 个固定种子；变体先独立验证几何与可执行性，再列清闭环抽样范围。

2. 每次正式簇运行最多 1800 仿真秒，正常名义计划 ≤1440 秒；最长封堵不可完成时按 PARTIAL 验收安全退场。整套 60 次以上的测试不要求 30 分钟内完成，提供分批、恢复和进度清单。

3. 故障包括动态横穿/长期阻塞、低障碍、坡度风险、GNSS 假 fixed/遮挡、定位丢失、传感器延迟、工具失联、任务重启。正常成功率和故障安全结果分开报告。

4. 执行分簇对照（K-means 修复、DARP 适配、METIS+时间闭环）、控制器对照、定位输入消融、预测与记忆对照，以及离线+在线修补与纯在线覆盖对照。共用输入/种子/速度/安全限制，不冒用不同任务比较。

5. 为每次运行生成完成/残余地图、误差、仿真与现实耗时、碰撞和资源指标，汇总分位数与区间；输出失败案例短回放。严禁只挑最好的一次作为验收。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/evaluation/acceptance_matrix.yaml`

- `/home/ceihao/robot_project/optimization_v2/evaluation/seeds.json`

- `/home/ceihao/robot_project/optimization_v2/ws/src/acceptance_runner`

- `/home/ceihao/robot_project/optimization_v2/evaluation/checkpoints`

- `/home/ceihao/robot_project/optimization_v2/evaluation/acceptance_summary.json`

- `/home/ceihao/robot_project/optimization_v2/stages/S17/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

总览页六簇逐个显示完成率、用时、定位误差、碰撞和 PASS/PARTIAL/FAIL；点击失败可看回放/轨迹、残余和原因，支持按种子/算法筛选。

### 必须通过的检查

- **S17-C01**：R01–R08 逐项追踪到执行证据；主场景六簇各至少 10 个固定种子无缺项，未运行不能写 PASS。

- **S17-C02**：正常喷洒 100%、割草 ≥95% 且残余完整；基础定位 P95 目标、运动学与停止门限通过；固定验收用例零碰撞。

- **S17-C03**：每次运行 ≤1800 仿真秒（包含退出/停止预算）；异常按期停止保存进度，PARTIAL 与失败不可混成完整成功。

- **S17-C04**：各类消融有同条件配对结果和局限；纯在线覆盖/实验性模型若无优势可拒用但必须有数据。

- **S17-C05**：批次中断恢复不重复计成绩；训练/调参/正式测试分离；资源指标与真实硬件、RTF 同时记录。

### 本步技术与代码教学重点

**算法原理**：验收与探索实验分离、消融和公平对照、分位数/区间、失败分析。

**C++ 核心逻辑**：运行监控、独立碰撞/时钟评估与契约校验；查看关键安全回调实码。

**Python 核心逻辑**：批量实验、断点续跑、配对统计、R01–R08 追踪与图表生成。

**现有代码参考**：field95/run_tests.py 与原有回归输入，前 16 步证据；历史报告不能替代本轮正式运行。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 17 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `acceptance_preview`**：快速故障样例与六簇总览；明确此 SMOKE 不能替代正式种子矩阵。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S17 --case acceptance_preview --seed 41 --sim-seconds 300 --wall-timeout 720
```

时限：最多 300 仿真秒，外层现实超时 720 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `resume_partial`**：制造暂停/重启，地图显示已完成与残余，恢复后不重复累积。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S17 --case resume_partial --seed 41 --sim-seconds 300 --wall-timeout 720
```

时限：最多 300 仿真秒，外层现实超时 720 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S17 --suite formal --seed 41 --sim-seconds 1800 --wall-timeout 21600 --shard 1/10 --resume
```

上面只执行正式矩阵第 1/10 分片（每分片约六个正式簇）；`--resume` 跳过版本匹配的已完成案例。单分片成功不等于本步 PASS。执行本步的代理须依次调度 1/10 至 10/10 及规定消融，汇总全部必需项后才发 PASS。以下命令可手工分批继续，跑到哪一片都能从检查点恢复：

```bash
for shard in 2 3 4 5 6 7 8 9 10; do
  bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S17 --suite formal --seed 41 --sim-seconds 1800 --wall-timeout 21600 --shard "$shard/10" --resume || break
done
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S17 --suite ablations --seed 41 --sim-seconds 1800 --wall-timeout 21600 --resume
```

上限是每次调用的现实秒数；超时保留检查点并报告 PARTIAL。`check S17` 必须在矩阵与消融全完成后才显示 PASS。


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S17
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S17
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S17 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S17
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S17/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S17/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S17/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。

## 执行第 18 步 可复现交付与学习材料总验收

请基于原项目实施 S18，目标是：整理成可启动、可恢复、可学习且体积受控的最终项目，确认没有重新依赖新加卷。

先读 `/home/ceihao/robot_project/AGENTS.md`、`/home/ceihao/robot_project/docs/optimization_v2/命令Prompt合集.md` 的共同要求，以及批准的 `/home/ceihao/robot_project/reports/不规则作业区自主机器人项目任务书_20261109.docx`。本步前置：S17。检查实际完成状态，不把本 prompt 当作实现已存在。

本次请直接完成实现、必要依赖、正常/失败测试、直观证据与学习材料；不要只回复方案。按共同要求处理前置缺口，不实现无关后续步骤。

### 实施内容

1. 固定最终依赖/模型/地图/计划/参数版本，提供差速喷洒、阿克曼割草、连接路巡检三个入口及命令行帮助，写一页快速开始。

2. 在干净 shell 和隔离测试目录完成安装/构建/启动/停止/重启恢复；检索所有绝对路径、符号链接和运行资源，验证主文件夹依赖闭合。与系统安装的软件依赖区别列出。

3. 按用户空间目标保留源码、许可、锁文件、模型、必要地图/数据库、准入输入、最新证据及简短失败样例。建立原始数据环形记录（初始每案例 1 GB）和经验库 100 MB 上限；删除旧输出前做引用检查，不能删被回执绑定的验收证据。

4. 汇总每步 TECHNICAL/CODE_WALKTHROUGH/MANUAL_CHECK/RESULTS 与 C++/Python 可运行教学例；检查文件链接、版本与代码行号，并按真实生产/上游/教学示例分类。

5. 完成最终回归与变更影响分析。若修正前序代码导致回执过期，先重验受影响步骤再更新最终状态；不为最后交付虚构前序 PASS。记录未启用模型/缓存的实测原因和将来机械臂接口。

### 预计交付位置

这些是本步应创建/完善的交付路径，不代表现在已有：

- `/home/ceihao/robot_project/optimization_v2/README.md`

- `/home/ceihao/robot_project/optimization_v2/run.sh`

- `/home/ceihao/robot_project/optimization_v2/config/release_manifest.json`

- `/home/ceihao/robot_project/docs/optimization_v2/快速开始.md`

- `/home/ceihao/robot_project/docs/optimization_v2/学习导航.md`

- `/home/ceihao/robot_project/optimization_v2/config/storage_policy.yaml`

- `/home/ceihao/robot_project/optimization_v2/stages/S18/run.sh` 及共同要求规定的四份文档、两种语言教学例、可视报告和真实回执。

### 人工应看到的结果

打开总览，三个任务入口、六簇状态、资源占用和 18 步学习材料可点击；干净启动后能运行选定任务并恢复一个中断作业。

### 必须通过的检查

- **S18-C01**：新工作区与运行资源完全位于主文件夹；没有新加卷运行依赖，干净 shell 与安装说明复现成功。

- **S18-C02**：三个任务入口和停止/恢复命令有真实运行记录；最终必要回归通过。

- **S18-C03**：18 步必备文档、源码讲解、C++/Python 教学例与证据链接完整；前序回执有效或明确列出未完成，不能在存在缺口时宣称全部完成。

- **S18-C04**：存储策略实测有界，必要模型/地图/锁文件/许可/证据未删除；体积清单和可重建项说明可查。

### 本步技术与代码教学重点

**算法原理**：可复现交付、版本契约、回归影响、可维护性与证据生命周期。

**C++ 核心逻辑**：最终模块调用图与 C++ 可执行文件/插件职责；示例编译命令和实际依赖。

**Python 核心逻辑**：启动编排、配置校验、交付审计、恢复与存储维护逻辑。

**现有代码参考**：前 17 步有效交付；保留原 native_20261108 入口，不把其运行成功当新系统验收。 参考文件需先检查实际存在、依赖和测试，不能照抄历史结论。

### 手工验收命令与解释

以下 demo/test 在第 18 步执行器交付后使用。若对应阶段尚无执行器，会显示 NOT_READY；是否已实现以当前步骤状态和实际文件为准。

命令逐条执行，不并行运行两辆车或两个仿真。新开一个普通终端即可，执行器负责环境；若主终端被演示占用，停止命令放第二终端。


**演示 `release_smoke`**：从最终入口完成短任务、停止和恢复；查看帮助与配置版本。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S18 --case release_smoke --seed 41 --sim-seconds 300 --wall-timeout 720
```

时限：最多 300 仿真秒，外层现实超时 720 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**演示 `delivery_audit`**：总览检查依赖闭合、文件体积、18 步状态和学习材料。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh demo S18 --case delivery_audit --seed 41 --sim-seconds 120 --wall-timeout 360
```

时限：最多 120 仿真秒，外层现实超时 360 秒。离线绘图/规划案例只使用现实超时；不能把这个上限当作实测任务完成时间。


**自动复验**：运行本步必需检查及失败用例，生成本轮指标和回执。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh test S18 --suite acceptance --seed 41 --sim-seconds 1800 --wall-timeout 7200
```


**查看本轮证据及状态**：`check` 校验已存在的回执/文件版本，`open` 打开浏览器中的实测可视页面。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh check S18
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S18
```

没有桌面或浏览器时，用下面命令取得报告路径：

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh open S18 --print-path
```


**停止本步自己启动的进程**：关闭工具、停车、保存未完成进度后退出；其他 ROS 会话不应受影响。

```bash
bash /home/ceihao/robot_project/optimization_v2/stepctl.sh stop S18
```


**阅读本步学习材料**：执行完成后下列文件存在；`CODE_WALKTHROUGH.md` 必须给出对应源码的函数和行号，教学示例的编译/运行命令放在 lessons 的 README。

```bash
cat /home/ceihao/robot_project/docs/optimization_v2/S18/TECHNICAL.md
cat /home/ceihao/robot_project/docs/optimization_v2/S18/CODE_WALKTHROUGH.md
cat /home/ceihao/robot_project/optimization_v2/lessons/S18/README.md
```

### 完成时的回复要求

列出本轮实现及关键文件、实际执行的检查与数值、未通过或未运行项；给我上述可直接复制的验收命令与可视结果入口，并链接四份学习材料。报告中区分自动通过、人工尚未查看及实验方案被数据否定。不要用下一步计划代替本步结果。 最后必须判断“可以执行下一步”或“暂不能执行下一步”，依据本步有效回执及下一步前置说明原因；不自动实施下一步。
