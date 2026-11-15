# 术语 命令与依赖索引

本页用于读章时快速返回概念。参数与源码以固定基线为准；官方资料用于理解上游概念，不能把更新版默认值直接覆盖当前项目。

## 常用术语

|术语|含义|章节|
|---|---|---|
|obligation 工作义务|原任务必须交代的点、面积或路径范围|01、25|
|residual 残余|未被有效反馈确认的原工作|01、20、25|
|shell|解释终端命令的程序|02、03|
|process 进程|正在执行的程序实例|03|
|overlay|叠加在基础 ROS 环境上的工作区|03、26|
|RAII|通过对象生命周期管理资源|04|
|Future|尚未或已经完成的异步结果容器|05、24|
|QoS|通信交付质量约定|05|
|lifecycle|节点配置、激活、退出状态管理|05、21|
|TF|随时间描述坐标系关系|06|
|ENU|局部东、北、天坐标|06、14|
|lever arm 杆臂|天线/传感器相对车体参考点偏移|06、14|
|epoch|区分时间重置或会话代际|06、25|
|RTF|仿真时长与现实时长的比值|03、06|
|footprint 足迹|车体投影的碰撞/通行几何|10、20|
|curvature 曲率|单位行进距离的方向变化，单位 1/m|10、22|
|PointCloud2|带字段布局的点云消息|11|
|extrinsic 外参|传感器与车体之间刚体变换|11、15|
|bias 偏置|非零均值测量误差|11、12|
|covariance 协方差|不确定性及变量相关性|12、14|
|ICP|用点云对应关系估计刚体运动|12|
|EKF|非线性模型局部线性化的卡尔曼滤波|12|
|SLAM|同时定位与建图|13|
|loop closure 回环|重新观察旧区域形成全局约束|13|
|NIS|按创新协方差归一的残差平方|14|
|voxel 体素|三维空间离散单元|15|
|TTC|保持当前相对运动时的碰撞时间估计|16|
|IMM|交互多运动模型估计|17|
|ADE/FDE|平均/末端未来位置误差|17|
|CSR|压缩稀疏行图表示|18|
|METIS|图划分库|18|
|TSP|旅行商访问顺序优化|19|
|swath 条带|工具连续工作的覆盖线/带|20|
|headland 地头|用于转弯连接的区域|20|
|Reeds–Shepp|允许前后运动的有界曲率连接|20|
|RPP|受约束纯追踪控制|22|
|DWB|动态窗口轨迹采样与评分控制|22|
|MPPI|预测控制序列采样与加权更新|22|
|BT tick|一次行为树评估推进|23|
|latch 锁存|满足明确复位前持续保留停止状态|23|
|idempotency 幂等|重复同一请求不重复产生效果|25|
|transaction 事务|相关存储修改共同提交或回滚|25|
|WAL|SQLite 写前日志模式|25|
|hash 摘要|检查文件内容身份|01、30|

## 命令按影响分类

|命令|作用|是否改变状态|
|---|---|---|
|pwd、ls、rg、sed -n|目录与文本观察|通常只读|
|test、sha256sum|存在/权限/内容检查|只读|
|source env.sh|加载当前 shell 环境|改变 shell 环境，不安装包|
|python3 examples/l02/labs.py NN|离线练习|仅临时内存/临时目录，不控制机器人|
|g++ ... -o build/...|编译教学 C++|写学习目录 build|
|apt install|安装系统依赖|改变系统，见第 26 章前置|
|python -m pip install|安装指定解释器环境依赖|改变该环境|
|colcon build|构建工作区|写 build/install/log，影响运行版本|
|advance.sh R09 check|检查现存回执和摘要|只读检查|
|gui.sh|启动只读监视服务|创建监视进程/状态文件|
|advance.sh R12 record CASE|运行真实仿真并录制|创建任务进程和新结果|
|Ctrl+C|请求前台进程组中断|需要等待合作收尾|

## 库与职责

|库/工具|语言边界|本项目职责|
|---|---|---|
|rclpy / rclcpp|Python / C++|ROS 通信与节点|
|Nav2|主要 C++，通过 action 调用|规划、路径控制与生命周期|
|RTAB-Map|C++ 节点与工具|局部 ICP、既有建图流程|
|robot_localization|C++ 节点|多状态局部融合|
|NumPy / SciPy|Python 接口与编译运算|数组、线代、邻近与分配|
|Shapely|Python 与 GEOS|多边形、扫掠、面积并集|
|METIS|C API / C++ 包装|连通工作量图划分|
|OR-Tools|C++ 后端|点访问顺序启发式|
|Fields2Cover|C++ 后端|条带与几何生成|
|OMPL|C++ 后端|非完整车辆连接候选|
|BehaviorTree.CPP|C++ 与 XML|条件组合与可解释监督|
|Eigen|C++ 模板库|变换、矩阵与四元数|
|nlohmann/json|C++ 头文件库|配置与结构化数据|
|SQLite / sqlite3|C 引擎 / Python 接口|反馈账本与事务|
|Gazebo gz-sim8|C++ 插件与 SDF|物理、传感器与关节执行|

## 官方概念资料

ROS 官方 Jazzy [安装源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)和 [QoS 说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)用于核对平台与通信兼容性。Python 3.12 [argparse](https://docs.python.org/3.12/library/argparse.html)、[subprocess](https://docs.python.org/3.12/library/subprocess.html)和 [sqlite3](https://docs.python.org/3.12/library/sqlite3.html)解释命令参数、进程与数据库接口。Nav2 [Jazzy RPP 源码说明](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/README.md)用于对照控制器职责。

生产参数、当前启用方案和验收状态仍以各章固定项目源码及 evidence 为依据。官方 rolling 文档可能包含基线没有的新功能，不在本教材中自动升级。

## 不随学习仓库提供的内容

完整机器人运行环境、原冻结数据库、大型参考云、所有原始运行和长视频不因教材可读而自动具备。资源清单见 [L01 环境说明](../L01/09_environment.md) 与 [原项目迁移边界](../evidence/docs/three_tasks/R12/LOCAL_REPRODUCTION.md)。当前 Windows 原生阅读与新机器完整仿真需要你分别人工确认；Linux 浏览器自动检查不能代替二者。
