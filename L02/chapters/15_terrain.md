# 第 15 章 点云预处理与地形代价

车辆在地面上行驶，导航可以使用二维位置与航向，但危险并不只有二维墙壁。低矮物、台阶、坡面、粗糙地面和盲区需要从三维观测判断，再转成车辆可用的二维代价。本章解释这条降维链路及其容易产生假障碍的地方。

## 预处理不是越多越好

先拒绝 NaN、Inf、范围外点，再按正确外参变换，去掉自车返回，进行适度空间降采样。每一步都应说明删除什么、为什么删，以及是否会同时删除真实低障碍。把靠近机器人所有点一律删掉，会得到漂亮但不安全的空白区域。

体素过滤把空间划成小立方格，用代表点降低计算量。它能控制密度，却改变统计权重：一块地面有很多独立射线，一个噪声点也可能占一个体素。若直接按占据体素等权统计，高处孤立噪声可能被放大。当前 R08 的表面统计使用实际射线支持来处理这类问题。

## 地面高度 坡度 阶差 粗糙度

地面模型给出局部预期 floor(x,y)，点高度减 floor 得到相对高度。邻域平面拟合可估坡度，分位数差可描述高低变化，拟合残差可描述粗糙度。三者含义不同：平滑斜坡粗糙度低，但坡度可能超限；低台阶平均坡度可能被大邻域稀释，却仍是碰撞风险。

均值容易受离群点影响，中位数与分位数更稳健，但也可能把少量真实台阶吞掉。因此当前代码在低/高分位统计外寻找有空间支持、与噪声置信区间分离的高度模式。强几何危险仍保留即时上通道策略，不等很多帧平均后才响应。

## 自车过滤中的边界

R09 `selfReturn` 区分物理车体内部与扩展不确定环。车体内部返回应剔除；扩展环中较高点可能是车体误差返回，但低地面/台阶不应都删掉。代码对扩展环采用不同高度条件，且明确“不合成自由格”。

这能修复部分假障碍，但不是识别所有近地物体的万能规则。外参、车体形状、雷达高度、姿态和碰撞包络变化后，必须重新验证过滤范围。软件认为空旷而真实车体遮挡的区域仍应保持未知。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### surface_statistics.hpp 第 1 至 25 行

[离线带行号源码](../../baseline/l02_views/2df4a08c3115-surface_statistics.hpp.html#L1) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/r08_backend/surface_statistics.hpp#L1)

```cpp
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
namespace r08 {
struct SurfaceStatistics {double ground=0,top=0;int supported_modes=0;};
inline SurfaceStatistics surfaceStatistics(const std::vector<Eigen::Vector3d>& rays,
 double voxel_ground,double voxel_top,double floor,double sigma=.012,double strong=.045) {
 if(rays.empty())return {voxel_ground,voxel_top,0};
 std::vector<double> z;for(const auto&p:rays)z.push_back(p.z());std::sort(z.begin(),z.end());
 SurfaceStatistics r{z[size_t(.25*(z.size()-1))]-floor,z[size_t(.8*(z.size()-1))]-floor,0};
 // Strong geometric hazards keep the existing immediate upper-channel policy.
 if(voxel_top>=strong||voxel_top<=-strong)r.top=voxel_top;
 // A small, coherent raised surface must not disappear into many floor rays.
 // Require spatial support and a confidence-separated mode; random isolated
 // range-noise bins must not receive the same weight as hundreds of floor rays.
 std::map<std::pair<int,int>,std::vector<double>> high,low;
 for(const auto&p:rays){double h=p.z()-floor;auto key=std::make_pair(int(std::floor(p.x()/.04)),int(std::floor(p.y()/.04)));if(h>r.ground+2*sigma)high[key].push_back(h);if(h<r.top-2*sigma)low[key].push_back(h);}
 for(const auto&[key,a]:high)if(a.size()>=3){double mean=0;for(double v:a)mean+=v;mean/=a.size();if(mean-2*sigma/std::sqrt(a.size())>r.ground+2*sigma){r.top=std::max(r.top,mean);++r.supported_modes;}}
 for(const auto&[key,a]:low)if(a.size()>=3){double mean=0;for(double v:a)mean+=v;mean/=a.size();if(mean+2*sigma/std::sqrt(a.size())<r.top-2*sigma){r.ground=std::min(r.ground,mean);++r.supported_modes;}}
 return r;
}
}
```

z 分位统计来自射线而非简单体素计数。high/low 按小平面格收集独立支持，至少三个返回且置信区间分离才保留小表面模式。strong 危险通道优先，不被平均稀释。

### self_filter.hpp 第 1 至 13 行

[离线带行号源码](../../baseline/l02_views/fcb035de32d1-self_filter.hpp.html#L1) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/r09_backend/self_filter.hpp#L1)

```cpp
#pragma once
#include <array>
namespace r09 {
// Keep real low ground/steps in the uncertainty annulus. The physical body
// still removes its own returns; this never synthesizes free-space cells.
inline bool selfReturn(double x,double y,double z,const std::array<double,4>& expanded,
                       double margin,double ceiling) {
 const bool padded=x>expanded[0]&&x<expanded[1]&&y>expanded[2]&&y<expanded[3];
 const bool physical=x>expanded[0]+margin&&x<expanded[1]-margin&&
                     y>expanded[2]+margin&&y<expanded[3]-margin;
 return z<ceiling && (physical || (padded && z>.08));
}
}
```

physical 是真实车体内部，padded 是带余量区域。外环只删除满足高度条件的返回，低地面仍保留。这个函数是过滤判据，不会给缺失区域制造自由空间。

<!-- generated-production-end -->

## 时间补偿的适用条件

跨帧累积需要把点变换到同一时刻或共同 odom 坐标；定位修正改变时还要避免历史点云跳动。当前主雷达是无逐点时间的瞬时扫描模拟，因此讨论的是帧级时刻一致性和累积，不宣称已经验证真实旋转扫描的逐点去畸变。

对动态物体长时间累积会留下拖影。地形长期统计与动态目标短期风险最好分开处理，分别使用新鲜度与清除逻辑。一个已离开的行人不能永久变成静态墙；一个暂时没看到的低障碍也不能立即视为消失。

## 从三维风险到二维栅格

栅格至少需要区分已观测自由、障碍、未知。Nav2 代价还有膨胀等连续值，不应把所有非零格都解释成实体墙。车辆足迹在栅格上检查的是整个多边形，不只是中心点。

地图分辨率越粗，窄通道与车体边缘越容易受离散化影响；越细，计算与存储成本越高。规划包络需要考虑足迹 padding、栅格尺寸和跟踪误差，不能离线刚好贴边、在线再怪控制器“胆小”。

## 小例子与命令

执行 `python3 examples/l02/labs.py 15`。一组近零高度混入 1.5 m 离群点，均值明显偏高而中位数仍接近地面；另一个 0.10 m/1 m 高差例子坡度约 5.71°，超过 5°。这些是教学数据，不是对某块实际地图重新测量。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_15.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab15():
    """Robust statistics do not create missing observations."""
    heights=[.00,.01,.02,.01,.00,1.5]
    median=statistics.median(heights); mean=statistics.mean(heights)
    slope=math.degrees(math.atan2(.10,1.0))
    assert median<mean and slope>5
    return dict(kind='synthetic surface samples',median_m=median,mean_m=mean,
                slope_deg=slope,passes_five_deg=False,empty_cell='UNKNOWN')
```

```text
{
  "chapter": "15",
  "passed": true,
  "result": {
    "kind": "synthetic surface samples",
    "median_m": 0.01,
    "mean_m": 0.25666666666666665,
    "slope_deg": 5.710593137499643,
    "passes_five_deg": false,
    "empty_cell": "UNKNOWN"
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 常见故障排查

在当前 GUI 中同时查看点云、地形状态、拒绝原因和定位。若假障碍只在转弯出现，重点检查姿态外参、时间差与自车过滤；若所有地方都未知，检查点云输入和观测累积；若坡面变成直墙，检查 floor 模型和坐标转换。

## 自测

问：没有点是否说明没有障碍？答：可能是遮挡、量程、丢包或视场盲区，应先保持未知。

问：把障碍高度门限调高能否解决所有停车？答：可能掩盖真实低障碍。应先解释误报来源，再做有证据的参数修改并复验。

练习：画出一个车体、自车扩展环、低台阶和雷达盲区，分别指出哪些点应该删除、保留或标记未知。给每种选择写一个能反驳它的观测条件。
