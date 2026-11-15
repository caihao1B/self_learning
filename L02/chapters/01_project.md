# 第 01 章 项目为什么立项

你面对的是一个农田机器人系统：它需要在有坡面、道路、静态物体和运动物体的环境中完成任务，并留下可以检查的结果。立项时首先要回答“什么算完成”，然后才能选择地图、底盘、传感器和算法。否则容易出现车已经走完轨迹，报告也显示成功，但工具没有真正工作的情况。

本章不要求编程基础。读完后，你应能画出三种任务的输入和输出，并解释为什么一次漂亮的视频不能代表全地图验收通过。

## 三种任务分别解决什么问题

点喷的对象是离散服务点。任务需要知道点的位置、请求量、允许位置误差、工具是否就绪和已确认多少。路线优化决定先去哪个点，导航负责接近，精停负责满足作业条件，工具动作负责执行，账本负责避免漏记与重复记。少了其中任意一步，都可能把“到点”误当成“完成喷洒”。

巡检的对象是一条连接路径。重点是沿着路线稳定通行，遇到横穿目标可以等待，遇到同向移动目标可以在有条件时保持距离，持续受阻时尝试合法替代路径，耗尽有界恢复后请求人工。巡检完成量不能用喷洒剂量衡量，它关心路径进展、偏离、风险和结束停车。

割草的对象是一块区域。离线规划产生覆盖条带和连接段；执行器需要区分工具开启的工作段、工具关闭的转弯段、换挡停车和未完成残余。车辆经过的面积与工具实际确认的有效面积也不相同，重叠区域不能无限累加。

|任务|主要几何对象|执行动作|完成证据|
|---|---|---|---|
|点喷|服务点及允许姿态|导航到点、精停、POINT_SERVICE|每个工作键的有效反馈和残余|
|巡检|有方向的连接路线|沿路径行驶、等待、跟随、恢复|有效进展、任务结果、停车确认|
|割草|条带、地头、原作业区域|FollowPath、START_WORK、STOP_WORK|工具反馈形成的有效面积与残余|

三个任务独立启动，可以使用各自适合的固定底盘。这样减少一次测试牵涉的模块，缩短发现问题的反馈时间。它不会自动减少同样作业总量所需要的物理时间：把三小时切成三段，不会使它变成一小时。真正节省的是排错、重跑和演示准备的时间。

## 从需求变化理解设计选择

最初系统从虚拟机迁移到原生 Linux，原因是仿真运行与图形资源限制。迁移阶段首先关心模型、世界、依赖和入口是否完整；之后才逐步加入多传感器、定位、感知、规划与行为树。这个顺序体现了工程依赖：如果仿真时钟和传感器消息都不可靠，控制器参数再复杂也无法补救。

覆盖问题随后从规则小地图扩展到连续不规则工作区。用户要求先有完整工作区，再通过算法分成三个连通簇，而不是提前画三个互不相连的小岛。这样分簇才真正承担大地图任务分解的作用。簇边界主要是任务组织和计账边界，不意味着车辆必须把每条边界当围墙。

当完整六簇和大量故障矩阵的现实运行时间过长时，演示任务获得了独立版本。正式地图、原作业分母和验收标准仍保留。之后进一步拆成点喷、巡检、割草三个入口，并修复定位抖动、阿克曼连接和工具取消。三点点喷是可学习的闭环样本，不是把 A1 的 56 个义务永久改成 3 个。

## 需求怎样转成可执行的约束

“贴近真实”需要拆成能检查的要求。传感器有采集时间、误差和外参；车辆通过关节执行，而不是直接设置世界位姿；规划考虑整车包络和曲率；在线控制只使用授权先验与观测；工具完成量需要反馈；无法可靠恢复时允许停车并保留残余。

“最多三十分钟”也要说明时钟。任务预算是仿真时间，计算机消耗的是墙钟时间。若平均实时因子 RTF 为 0.4，1800 仿真秒大约需要 4500 现实秒，再加初始化和收尾。机器变快可能缩短墙钟，却不会改变作业的仿真时长分母。第 03、06 章会详细解释。

“允许偏差”应落实为明确版本的容差或残余政策。它不能被解释为随意忽略定位拒绝、把障碍删掉或关闭碰撞检查。原 B1 的部分末端与允许坡面约束冲突后，采用新的安全执行版本，并保留原目标和残余，就是把授权变成可审查设计的例子。

## 当前完成到了哪里

学习基线固定于 `9aff8337b7794f704c5556a973a5220e73f17808`。R08、R09 的现存回执检查为 PASS；R10、R11、R12 保持 PARTIAL。三点点喷记录完成 3/3，原 A1 仍有 56 个目标；完整 A1 的保存结果完成 28/56，因定位重验证有界停止；安全执行版完整 B1 保存结果完成 19/47，进入人工请求/等待期限相关停止。段数与有效面积不是同一个指标。

这些状态可以同时成立：一个子案例可以通过，一个大阶段仍可以未完成。失败后的安全停车也有价值，但不能据此宣布任务完成。更不能把停止原因名称直接当成定位问题的最终根因。

## 用集合而不是印象理解残余

设原工作义务集合为 U，本次选中集合为 S，反馈确认集合为 C。必须满足 C 是 S 的子集，S 是 U 的子集。样章中 |U|=56，|S|=3，|C|=3。本次子任务完成率是 3/3；对原任务的已确认比例至多是 3/56，并且还要检查历史账本是否属于同一版本与会话。

如果一个点只完成一部分剂量，不能把它当作普通布尔值“完成/未完成”而丢掉数量。更准确的表达是请求量 q、确认量 c、剩余量 max(0,q−c)，以及反馈是否存在不确定性。割草的面积还要取几何并集，避免重复计账。

## 代码入口与阅读任务

`common.check()` 同时检查阶段状态和绑定文件哈希；`seal()` 将 summary 与相关文件封存成回执。Python 字典用于表示具名字段，列表存多项制品，SHA-256 把文件内容映射成摘要。哈希能够检查“是不是同一份内容”，不能证明算法本身正确。

`stage_r12.CASES` 是实际录像范围的入口。看到 `point_limit=3` 应理解为该案例的选择范围，而不是点喷规划器只能处理三个点。`seconds=300` 是案例仿真预算，也不是保证一定跑满五分钟。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### common.py 第 8 至 12 行

[离线带行号源码](../../baseline/l02_views/e9ad013f1962-common.py.html#L8) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/common.py#L8)

```python
def check(stage):
 p=APP/'results'/stage/'latest.json'
 if not p.exists():return dict(passed=False,stage=stage,reason='NOT_RUN')
 d=read(p);bad=[x['path'] for x in d['artifacts'] if not (ROOT/x['path']).is_file() or sha(ROOT/x['path'])!=x['sha256']]
 return dict(passed=d['automated_status']=='PASS' and not bad,stage=stage,changed=bad,status=d['automated_status'])
```

先读路径拼接，再看 artifacts 列表推导式。任一文件不存在或摘要变化进入 bad；最后 passed 还要求原 automated_status 是 PASS。这个函数返回新判断，不改旧回执。调用者 require 在不满足时抛错，因此结果为 PARTIAL 与文件变更是两个独立失败维度。

### stage_r12.py 第 1 至 18 行

[离线带行号源码](../../baseline/l02_views/ae5ff590266d-stage_r12.py.html#L1) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/stage_r12.py#L1)

```python
"""Truthful separate recordings; partial upstream acceptance never becomes a full release."""
from common import *
import importlib.util
from prepare_r12 import build

CASES=dict(spray=dict(task='spray',case='tool_jam',point_limit=3,seconds=300),spray_full_A1=dict(task='spray',case='nominal',point_limit=0,seconds=1800),inspection=dict(task='inspection',case='nominal',seconds=480),inspection_leader=dict(task='inspection',case='leader',distance_limit=20.,seconds=150),inspection_blocked=dict(task='inspection',case='blocked',distance_limit=20.,seconds=120),mowing=dict(task='mowing',case='nominal',segment_limit=5,seconds=420),mowing_full_B1=dict(task='mowing',case='nominal',segment_limit=0,seconds=1800))

CASES['mowing_startup_perception_loss']=dict(task='mowing',case='startup_perception_loss',segment_limit=5,seconds=420)

def declared_scope_matches(receipt,spec,name):
 expected=CASES[name];fields={'seconds':'sim_seconds','distance_limit':'distance_limit_m'}
 if expected['task']=='mowing' and spec.get('segment_start',0)!=0:return False
 if receipt.get('case')!=name or receipt.get('task')!=expected['task'] or spec.get('task')!=expected['task']:return False
 if not all(spec.get(fields.get(k,k))==v for k,v in expected.items() if k!='task'):return False
 return expected['task']!='inspection' or spec.get('distance_limit_m')==expected.get('distance_limit',0.)

def recheck(receipt):
 """A saved PASS is not enough after code/config or video changes."""
```

CASES 是普通字典，spec 字段声明任务范围。declared_scope_matches 将外部回执、实际 spec 与声明逐项比较，防止三点录像被误标为完整任务。这里没有执行模型推理，只是在核对事实合同。

<!-- generated-production-end -->

## 操作和观察

在学习目录执行 `python3 examples/l02/labs.py 01`，它只读取附带的历史结果。Windows 可改成 `py -3 examples\l02\labs.py 01`。输出较长，先找到任务状态、目标数量、完成数量、停止字段，再对照第 08 章的事件链。读取旧文件不会重新控制机器人。

当前原工程的只读检查命令为：

```bash
cd /home/ceihao/robot_project
bash optimization_v2/three_tasks/advance.sh R10 check
echo $?
```

基线预期退出码为 2，报告 PARTIAL；这是已知状态，不应通过编辑结果文件改成 0。若出现 `changed`，说明当前文件与回执绑定已经不同，需要先辨认版本。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_01.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab01():
    """Historical scope: a subset has a different denominator."""
    r = read('evidence/validation/R12/spray/mission_result.json')
    # Field names are inspected rather than inferred from the video title.
    return dict(kind='historical evidence', result=r,
                lesson='Read selected count and original obligation separately.')
```

```text
{
  "chapter": "01",
  "passed": true,
  "result": {
    "kind": "historical evidence",
    "result": {
      "ownership": {
        "/tf": [
          {
            "node": "s05_local_odometry",
            "namespace": "/",
            "gid": [
              1,
              15,
              245,
              121,
              239,
              153,
              230,
              235,
              0,
              0,
              0,
              0,
              0,
              0,
              19,
              3
            ]
          },
          {
            "node": "s08_global_pose_manager",
            "namespace": "/",
            "gid": [
              1,
              15,
              245,
              121,
              255,
              153,
              53,
              174,
              0,
              0,
              0,
              0,
              0,
              0,
              23,
              3
            ]
          }
        ],
        "/s03/actuator/request": [
          {
            "node": "r07_actuator_gate",
            "namespace": "/",
            "gid": [
              1,
              15,
              245,
              121,
              3,
              154,
              53,
              249,
              0,
              0,
              0,
              0,
              0,
              0,
              20,
              3
            ]
          }
        ],
        "/s10/tool/command": [
          {
            "node": "s10_tool_action_server",
            "namespace": "/",
            "gid": [
              1,
              15,
              245,
              121,
              9,
              154,
              53,
              227,
              0,
              0,
              0,
              0,
              0,
              0,
              20,
              3
            ]
          }
        ]
      },
      "final_gate": {
        "raw_nav_v": 0.0,
        "raw_nav_w": 0.0,
        "t": 92.76,
        "allowed": false,
        "mode": "STOP",
        "v": 0.0,
        "w": 0.0,
        "reasons": [
          "NAV_COMMAND_STALE"
        ],
        "stopped": true,
        "tool_stopped": true,
        "nav_idle": true,
        "progress_m": 7.077378701575697
      },
      "passed": true,
      "error": null,
      "cluster": "A1",
      "selected_points": 3,
      "original_points": 56,
      "completed": [
        {
          "point": "point_7_14",
          "state": "SUCCEEDED",
          "reason": "CONFIRMED_WORK",
          "confirmed": 3.043999999999997,
          "remaining": 0.0,
          "uncertainty": false,
          "replayed": false,
          "meter_before": 0.0,
          "meter_after": 3.2337
        },
        {
          "point": "point_8_14",
          "state": "SUCCEEDED",
          "reason": "CONFIRMED_WORK",
          "confirmed": 3.073999999999998,
          "remaining": 0.0,
          "uncertainty": false,
          "replayed": false,
          "meter_before": 3.3613,
          "meter_after": 6.621
        },
        {
          "point": "point_8_15",
          "state": "SUCCEEDED",
          "reason": "CONFIRMED_WORK",
          "confirmed": 3.0519999999999925,
          "remaining": 0.0,
          "uncertainty": false,
          "replayed": false,
          "meter_before": 6.621,
          "meter_after": 9.8647
        }
      ],
      "failed": [],
      "not_selected": [
        "point_9_15",
        "point_9_14",
        "point_10_14",
        "point_10_15",
        "point_10_16",
        "point_9_16",
        "point_8_16",
        "point_8_17",
        "point_9_17",
        "point_10_17",
        "point_10_18",
        "point_9_18",
        "point_8_18",
        "point_8_19",
        "point_9_19",
        "point_10_19",
        "point_11_19",
        "point_12_20",
        "point_11_20",
        "point_10_20",
        "point_9_20",
        "point_8_20",
        "point_8_21",
        "point_9_21",
        "point_10_21",
        "point_11_21",
        "point_12_21",
        "point_12_22",
        "point_12_23",
        "point_11_23",
        "point_11_22",
        "point_10_22",
        "point_9_22",
        "point
[显示节选；完整结构请打开本章输出文件]
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 常见误解与自测答案

问：三个案例录像都有了，是否就可以写全系统通过？答：还要看每个录像范围、运行版本、有效反馈、完整任务和阶段准入条件；录像文件存在只说明媒体存在。

问：长期障碍挡住一部分区域，怎样合理结束？答：保留原工作分母，记录已确认量、残余区域、阻塞证据和有界恢复结果，确认车与工具停止，再向人工说明。它可以是有效演示，也可以是任务 PARTIAL。

练习：用一张纸写出“请求、决策、执行、观测、确认”五列，把一个服务点从导航到工具结束填进去。检查标准是五列不能都用同一个 `success=true` 替代。完成后进入第 02 章，学习怎样找到保存这些事实的文件。
