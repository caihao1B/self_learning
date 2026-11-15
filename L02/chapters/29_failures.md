# 第 29 章 失败案例与诊断方法

工程调试不是看见一个异常名就修改相关参数。正确顺序是固定失败版本、限定任务范围、定位第一处异常、提出可检验假设、做最小修复、复验影响范围。本章总结现存记录中值得重点学习的问题，也保留尚未确定的根因。

## 哪些问题最常见

L01 汇总的 42 条独立历史尝试中，31 条在执行时未通过，包含有界停止和人工中断。记录中的 WAIT_DEADLINE 出现 6 次，部分精停重对齐耗尽和末端恢复耗尽各出现多次。这个样本不是随机抽样，跨越多个代码版本，因此不能推出“系统有多少百分比时间因某根因失败”。

停止码通常是最后保护动作，根因可能更早。WAIT_DEADLINE 可能等待的是工具、定位、动作结果或观察恢复；LOCALIZATION_REVALIDATION_BOUND 表示重验证耗尽，不自动证明 GNSS 硬件有问题。

## 按层建立排查表

|表现|先看证据|可能原因|合理下一步|
|---|---|---|---|
|启动 ImportError|解释器路径、模块位置|错误 venv、依赖缺失|固定解释器并检查导入|
|有 topic 无数据|QoS、domain、发布频率|不兼容、错域、未激活|检查通信合同|
|定位跳变|raw correction、stamp、GNSS|外参/基准、时差、配准歧义|区分几何与时间假设|
|静止精停反复失败|窗口 median/latest/p95|全局抖动、目标偏差、采样间断|保留门限分析原始窗口|
|阿克曼绕大圈|足迹、曲率、地头、坡面|几何约束或目标姿态|验证合法连接与残余|
|START_WORK 不响应 WAIT|阶段事件、BT 时间|启动等待未处理决策|修复等待循环与取消|
|工具量不增长|ledger 拒绝原因|反馈断档、未停稳、无效定位|先修反馈与准入|
|重复执行|工作键、指纹、attempt|身份变化或重启不确定|查询账本并恢复|
|录像旧帧|采集/接收年龄、编码结果|传输或显示落后|检查媒体链路|
|任务结束进程异常|runtime 退出列表|清理或插件崩溃|单独记录并检查收尾|

## 案例一 精停与全局抖动

现象是导航接近目标，但静止窗口或重对齐无法稳定通过。处理链包含原始 map→odom 修正检查、修正滤波、静止窗口、中位与最新距离、局部 odom 精停以及有界恢复。它们解决不同问题：滤波降低合格修正的高频抖动，局部精停提供连续参考，门限保留作业要求。

不能因为最终三点成功，就认为完整 A1 定位问题全部解决。保存的完整 A1 仍在 28/56 后因重验证上限停止。学习时把成功子例与失败长例并排，看触发分支和时间范围差异。

## 案例二 坡面与末端不可兼容

原 B1 的部分末端与 10° 坡面重叠，允许条件为 5°。继续要求逐个原末端到达会与通行合同冲突。经用户批准，建立安全执行版本，保留原工作多边形、面积分母和原 PARTIAL。修复位置是计划版本与几何合同，不是改控制器让车硬开。

新安全版本仍有完整任务未通过记录，说明几何改进不等于全部动态和执行问题已经解决。后续应继续区分连接可行性、观测风险和状态机，不把所有剩余失败都归成同一坡面问题。

## 案例三 工具启动与取消

START_WORK 期间等待工作反馈，如果不持续检查 BT，就可能在感知失效时仍等待或继续工具启动。修复让启动等待可中断，调用统一 stop_all，确认车和工具停止，再决定恢复。这是所有异步前置阶段都应检查的模式。

## 案例四 时间戳被误用

工具反馈可能比对应时钟回调先到。若立即按旧时钟拒绝，或把反馈重写为当前时间，会分别造成假失效或隐藏延迟。派生工具服务器保留原采集时间，把到达数据暂存到有界队列，等时钟推进后处理，并继续使用真实到达年龄。

## 案例五 假障碍和自车返回

高密度射线、体素等权与自车扩展区域曾影响近地风险。改进按真实射线支持统计表面、保留低台阶并区分物理车体与不确定环。修复必须保留真实低障碍测试，不能只用“这次终于能走”证明过滤正确。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### tool_build_r09.py 第 5 至 36 行

[离线带行号源码](../../baseline/l02_views/ecf3df28b853-tool_build_r09.py.html#L5) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/tool_build_r09.py#L5)

```python
def generate_tool(out):
 s=(BASE/'ws/src/tool_sim/src/action_server.cpp').read_text()
 s='#include <deque>\n'+s
 s=s.replace(' bool fresh(){',' std::deque<std::pair<J,Wall::time_point>> incoming_;\n bool fresh(){')
 start=s.index('try{auto d=J::parse(m->data);double t=d.at("t");bool ordered=')
 end=s.index(');\n  ledger_sub_',start)
 original=s[start:end]
 assert original.endswith('}}')
 body=original[len('try{auto d=J::parse(m->data);'): -len('catch(...){stopped_since_=-1;}}')]
 assert body.endswith('}')
 body=body[:-1].replace('feedback_wall_=Wall::now();','feedback_wall_=arrival;')
 helper=' void consume(J d,Wall::time_point arrival){'+body+'}\n void drain(){while(!incoming_.empty()&&incoming_.front().first.value("t",1e99)<=t_+.001){auto p=incoming_.front();incoming_.pop_front();if(p.first.value("t",-1.)>last_sensor_)consume(p.first,p.second);}}\n'
 s=s[:start]+'try{auto d=J::parse(m->data);incoming_.push_back({d,Wall::now()});if(incoming_.size()>30)incoming_.pop_front();drain();}catch(...){stopped_since_=-1;}}'+s[end:]
 s=s.replace(' void tick(){',helper+' void tick(){\n  drain();')
 s=s.replace('t_=t;});','t_=t;drain();});')
 s=s.replace('begin_stop("LOST","FEEDBACK_LOST",true);','{RCLCPP_ERROR(get_logger(), "Feedback freshness failure sim=%.6f sensor=%.6f wall_age=%.6f queued=%zu",t_,feedback_.value("t",-100.),age(feedback_wall_),incoming_.size());begin_stop("LOST","FEEDBACK_LOST",true);}')
 put(out/'tool_action_server.cpp',s)
 plugin=(BASE/'ws/src/tool_sim/src/ToolActuator.cc').read_text().replace('t-last_pub_>=.049','t-last_pub_>=cfg_.value("feedback_period_s",.049)')
 put(out/'ToolActuator.cc',plugin)
 return """\nset(Protobuf_PROTOC_EXECUTABLE /usr/bin/protoc CACHE FILEPATH "Gazebo ABI" FORCE)
find_package(Protobuf MODULE REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
add_library(s10_tool_r09 SHARED ToolActuator.cc)
target_link_libraries(s10_tool_r09 gz-sim8::gz-sim8 gz-plugin2::gz-plugin2 gz-transport13::gz-transport13 nlohmann_json::nlohmann_json)
find_package(rclcpp_action REQUIRED)
find_package(mission_interfaces REQUIRED)
add_executable(r09_tool_action_server tool_action_server.cpp)
ament_target_dependencies(r09_tool_action_server rclcpp rclcpp_action mission_interfaces std_msgs rosgraph_msgs)
target_link_libraries(r09_tool_action_server nlohmann_json::nlohmann_json)
"""
```

派生器定位原 C++ 接收逻辑，将反馈入队并在时钟追上后消费。保留 arrival 用于墙钟年龄，不重新打采集戳。字符串断言用于发现上游源码变化，不能静默生成错误派生版本。

### common.py 第 8 至 12 行

[离线带行号源码](../../baseline/l02_views/e9ad013f1962-common.py.html#L8) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/common.py#L8)

```python
def check(stage):
 p=APP/'results'/stage/'latest.json'
 if not p.exists():return dict(passed=False,stage=stage,reason='NOT_RUN')
 d=read(p);bad=[x['path'] for x in d['artifacts'] if not (ROOT/x['path']).is_file() or sha(ROOT/x['path'])!=x['sha256']]
 return dict(passed=d['automated_status']=='PASS' and not bad,stage=stage,changed=bad,status=d['automated_status'])
```

失败复验前先查内容是否仍匹配回执。某阶段 status 为 PASS 但 bound 文件 changed，也不能复用为当前证据。这个短函数是版本问题与行为问题分开的入口。

<!-- generated-production-end -->

## 诊断记录模板

每次实验写：固定提交与制品哈希；任务/选择范围；最早异常时间；当时输入及年龄；触发判据；两个候选根因；能区分二者的最小检查；修改内容；复验结果与残余。没有独立证据时使用“假设”，不要写成已确诊。

只改变一个主要因素更容易解释，但某些修复涉及坐标、计划和外参的连锁更新，必须明确列出依赖范围。组件检查通过后先跑有界短例，证明新分支确实触发，再跑受影响完整任务。

## 离线分析

执行 `python3 examples/l02/labs.py 29` 查看真实失败分类。把出现次数最多的停止码列出来，然后为每种写出需要进一步读取的文件。不要把频次表标题写成“根因概率”。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_29.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab29():
    """Inspect the shipped failure summary without inventing cause counts."""
    p=E/'failure_summary.json'
    d=json.loads(p.read_text(encoding='utf-8'))
    return dict(kind='historical classified attempts',summary=d,
                limitation='Stop-code frequency is not an independently diagnosed root-cause frequency.')
```

```text
{
  "chapter": "29",
  "passed": true,
  "result": {
    "kind": "historical classified attempts",
    "summary": {
      "total_records": 42,
      "unique_runs": 42,
      "not_passed_at_execution": 31,
      "stop_code_counts": {
        "TimeoutError('WAIT_DEADLINE')": 6,
        "RuntimeError('TOOL_STOP_UNCONFIRMED')": 1,
        "TimeoutError('ENDPOINT_RECOVERY_BUDGET')": 1,
        "RuntimeError('ENDPOINT_RECOVERY_MANUAL_LATCH_RETAINED')": 1,
        "RuntimeError('ALTERNATE_ROUTE_REQUIRES_COVERAGE_RESEQUENCING_RETAIN_RESIDUAL')": 2,
        "RuntimeError('MANUAL_OR_SAFETY_HOLD MOTION_WITHOUT_OBSERVED_PROGRESS')": 1,
        "UNSPECIFIED": 1,
        "RuntimeError('REDOCK_INTERRUPTED_OR_TIMEOUT')": 1,
        "TimeoutError('TASK_BOUND_OR_STOP_REQUEST')": 3,
        "RuntimeError('TOOL_STOP_UNCERTAIN')": 1,
        "RuntimeError('DOCKING_REALIGNMENT_EXHAUSTED')": 3,
        "ModuleNotFoundError(\"No module named 'shapely'\")": 1,
        "RuntimeError('TOOL_ADMISSION_FAILED')": 1,
        "RuntimeError('SEGMENT_RETRY_BOUND')": 1,
        "InterruptedError('SESSION_STOP')": 1,
        "RuntimeError('ENDPOINT_RECOVERY_EXHAUSTED')": 3,
        "TypeError(\"dict() got multiple values for keyword argument 't'\")": 1,
        "RuntimeError('DOCKING_MANUAL_OR_SAFETY_HOLD MOTION_WITHOUT_OBSERVED_PROGRESS')": 1,
        "RuntimeError('LOCALIZATION_REVALIDATION_BOUND')": 1
      },
      "warning": "Not a random sample or root-cause frequency; includes interrupted/bounded sessions and multiple software versions."
    },
    "limitation": "Stop-code frequency is not an independently diagnosed root-cause frequency."
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 自测

问：一次成功能否删除全部历史失败？答：不能用删除来消除未解释的行为；可整理精简证据，但需要保留与当前判断有关的版本和残余。

问：如何证明修复了 START_WORK 中断？答：需要在该阶段实际注入或观察中断，记录取消、停车、工具停止及后续行为；仅静态看到 if 语句不够。

练习：选择 WAIT_DEADLINE，提出“消息没有到”和“消息到了但不满足条件”两个假设，分别写一个只读观察方法。能区分二者，才有针对性修复方向。
