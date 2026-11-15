# 第 22 章 RPP MPPI DWB 与精停

控制器把当前位姿、路径与约束变成速度候选。当前三任务主链路使用 RPP，MPPI/DWB 的配置与早期对照保留。理解它们的思想有助于判断问题该改路径、状态估计还是控制参数。

## Pure Pursuit 与 RPP

Pure Pursuit 在车体坐标中选择前视点 (x,y)，用通过该点的圆得到 `κ=2y/(x²+y²)`，再计算 `ω=vκ`。前视距离短通常跟得紧，但更敏感；前视距离长更平滑，但可能切角与末端滞后。

RPP 在几何跟踪之外根据曲率、接近目标和碰撞等条件调节速度。它轻量且容易解释，但不负责凭空生成绕障通路。路径本身不可行时，降低速度只能减慢失败，不能创造转弯空间。

当前割草 RPP 禁止 rotate_to_heading，允许 reversing；这是当前 R09/R10 配置，早期 S13 的限制不同。阿克曼要倒车时还需要任务执行器按单挡段组织与实测停车，不能只改一个 allow_reversing 布尔值。

## DWB 的动态窗口

DWB 在当前可达速度范围采样 v、ω，按运动模型积分一段轨迹，再由多个 critic 评价障碍、路径、目标和振荡等代价，选出较好的候选。加减速度限制决定短时间能达到哪些速度。

优点是评价项清楚，适合差速基线；代价是采样分辨率、权重和短视性可能造成量化停滞或局部困境。增加速度样本会增加计算量，不一定解决目标附近的几何或坐标问题。

## MPPI 的预测采样

MPPI 对未来一串控制输入加入随机扰动，滚动预测多条轨迹，用指数代价权重更新控制序列，执行第一段后重新估计。与只选择一个固定速度候选相比，它处理时间序列和多项代价更灵活，也更消耗算力。

预测时域、时间步、batch 数、温度和 motion model 共同决定行为。模型必须符合差速或阿克曼约束。随机采样还要求多次独立运行比较稳定性，不能用一次成功证明优于 RPP。

|方法|主要计算|适合学习的切入点|主要局限|
|---|---|---|---|
|RPP|前视几何与速度调节|曲率、前视点、碰撞检查|依赖可执行路径|
|DWB|速度网格与轨迹评分|候选、critic、加速度窗口|量化与局部停滞|
|MPPI|控制序列随机采样与加权|预测时域、模型、代价|算力与参数耦合|

## 精停不是普通导航容差

喷洒需要满足厘米级稳定窗口。普通无进展监督可能要求累计移动 0.10 m，但合法精停只需 0.036 m；沿用同一进展尺度会把有意义的小调整判断为无进展。当前 `progressDelta` 仅在 VALIDATED_LOCAL_DOCKING 且距离位于明确范围时缩小尺度。

对 0.036 m 请求，四分之一为 0.009 m；距离超出范围则回退普通值。原超时、紧急优先级和工作准入不因此取消。精停小运动还需要独立合法路径与观测，不是对所有停滞放宽监督。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### precision_progress.hpp 第 1 至 17 行

[离线带行号源码](../../baseline/l02_views/3b34eb3bcafc-precision_progress.hpp.html#L1) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/r10_backend/precision_progress.hpp#L1)

```cpp
#pragma once
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
namespace r10 {
// Match progress scale to an explicitly validated, bounded precision motion.
// The 10 s timeout, emergency priority and original work admission stay intact.
inline double progressDelta(const nlohmann::json& cfg,const nlohmann::json& e) {
  const double coarse=cfg.at("progress_delta_m").get<double>();
  if (!cfg.contains("precision_progress_policy") ||
      e.value("motion_scope",std::string())!="VALIDATED_LOCAL_DOCKING" ||
      !e.contains("expected_translation_m") || !e["expected_translation_m"].is_number()) return coarse;
  const double distance=e["expected_translation_m"].get<double>();
  if (!std::isfinite(distance) || distance<=.008 || distance>.15) return coarse;
  return std::min(coarse,std::max(.005,.25*distance));
}
}
```

这个短函数是完整生产判据，可以由教材 C++ 例直接调用。所有合法性检查失败时回退粗尺度，只有受界限保护的局部精停才缩小进展阈值；未修改超时与安全条件。

### nav2.yaml 第 88 至 114 行

[离线带行号源码](../../baseline/l02_views/1f182bcfa0bc-nav2.yaml.html#L88) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/three_tasks/editions/R09/nav2.yaml#L88)

```yaml
      transform_tolerance: 0.15
      min_approach_linear_velocity: 0.02
      approach_velocity_scaling_dist: 0.5
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 2.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 3.0983388881649963
      regulated_linear_scaling_min_speed: 0.04
      use_rotate_to_heading: false
      allow_reversing: true
      max_angular_accel: 0.5
      max_robot_pose_search_dist: 8.0
      rotate_to_heading_angular_vel: 0.35
      rotate_to_heading_min_angle: 0.45
      use_fixed_curvature_lookahead: true
      curvature_lookahead_dist: 0.8
      interpolate_curvature_after_goal: true
    MPPI:
      plugin: s13::MeasuredController
      implementation: nav2_mppi_controller::MPPIController
      time_steps: 100
      model_dt: 0.05
      batch_size: 750
      iteration_count: 1
      vx_std: 0.05
      vy_std: 0.0
```

RPP 参数说明前视、接近降速、碰撞检查与反向行驶。use_rotate_to_heading=false 适配阿克曼不能原地旋转；allow_reversing=true 还需外部单挡段和停车流程配合。

<!-- generated-production-end -->

## 停车距离和延迟

理想恒减速度模型 `d=v²/(2a)`，如果感知/处理/执行总延迟 τ，还要加 `vτ`。0.5 m/s、减速度 0.4 m/s²、延迟 0.2 s 时约 0.4125 m。真实地面还受附着与执行器限制，所以这个计算用于理解，不能作为实机安全距离证明。

## 运行练习

```bash
cd /home/ceihao/self_learning
python3 examples/l02/labs.py 22
./build/precision_demo
```

C++ 示例先按第 04 章编译。Python 计算前视点曲率和延迟停车距离；C++ 直接调用固定生产判据。二者都不向机器人发送速度。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_22.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab22():
    """Pure pursuit and actuator speed-curvature limits."""
    x=1.;y=.2;v=.5;curvature=2*y/(x*x+y*y);omega=v*curvature
    stop_distance=v*v/(2*.4)+v*.2
    assert math.isfinite(omega) and stop_distance>.3
    return dict(kind='synthetic control calculation',curvature_1_per_m=curvature,
                angular_velocity_rad_s=omega,stop_distance_with_delay_m=stop_distance)
```

```text
{
  "chapter": "22",
  "passed": true,
  "result": {
    "kind": "synthetic control calculation",
    "curvature_1_per_m": 0.38461538461538464,
    "angular_velocity_rad_s": 0.19230769230769232,
    "stop_distance_with_delay_m": 0.4125
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 排错与自测

轨迹切角：查前视距离、路径曲率和速度；低速抖动：查定位修正、短路径、速度量化与时间对齐；车不响应角速度：查阿克曼 v≈0、曲率限制与转向关节；不停绕圈：先证明末端姿态可达；CPU 高：分开测控制器计算与传感器处理。

问：把控制器从 RPP 换成 MPPI 是否必然解决割草大圈？答：不会改变轴距、足迹和通行边界。先有合法路径，再比较跟踪性能与开销。

练习：保持前视点方向不变，把距离放大两倍，计算曲率变化；再把速度加倍，看角速度与停车距离怎样变化。说明提速不能只改一个 max_speed。
