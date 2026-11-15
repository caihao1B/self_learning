# 第 04 章 C++ 与 Python 入门

这个项目不是把所有算法用两种语言各写一遍。Python 擅长连接模块、读取配置、组织实验和处理数据；C++ 常用于 ROS 原生组件、Gazebo 插件、几何后端和高频监督。学习的重点是知道数据怎样跨越边界，以及谁负责校验。

## 从变量和函数开始

变量给一个值起名字。Python 中 `distance = 0.03` 在运行时绑定对象；C++ 中 `double distance = 0.03;` 同时声明静态类型。两者都不会自动知道 0.03 是米还是厘米，所以变量名、接口和文档必须保留单位。

函数把输入映射到输出。例如 `wrap(angle)` 把角度归一到主值范围，调用者传入弧度，得到弧度。类把状态和操作放在一起，例如 C++ `Policy` 持有等待开始时间、人工锁存和配置；Python 任务则常用字典保存最近消息。

|需要表达的内容|Python|C++|本项目中的例子|
|---|---|---|---|
|具名状态|dict|struct 或 JSON 对象|定位 valid、reason、t|
|有序集合|list、deque|vector、deque|路径点与观测窗口|
|缺失值|None|optional 或显式状态|还没有定位、还没有动作结果|
|错误|raise / try / except|throw / try / catch|版本不一致、路径非法|
|资源释放|with、finally|RAII、析构函数|文件、锁、进程句柄|

Python 的 `bool` 是整数类型的子类，`isinstance(True, int)` 为真；输入验证如果只看数字，可能把布尔值误当距离。NaN 则可能让比较表达式出现意外结果。实际 C++ 判据使用 `std::isfinite`，教学例子也明确拒绝非有限值。

## 可变对象和复制

Python 字典赋值通常复制的是引用，不是整个嵌套结构。`b = a` 后修改 b，可能改变 a；`dict.copy()` 只做浅复制，嵌套字典仍共享；`copy.deepcopy()` 才递归复制。当前传感器派生器 `effective_rig()` 使用 deepcopy，避免在修改采样密度时污染已冻结基础标定。

C++ 引用 `const J&` 可以避免复制并禁止经该引用修改；按值参数会产生复制或移动；`std::move` 允许资源转移，不代表每次都必然移动。不能在转移后假设对象还保留原内容。`std::vector` 扩容也可能使指向旧元素的指针或引用失效。

## 看懂回调和 lambda

ROS 消息到来时执行的函数叫回调。Python 的 `lambda m, k=key: ...` 把当次循环的 key 放在默认参数里，避免所有回调最终引用同一个末尾变量。C++ lambda 的 `[this]` 捕获对象指针，`[&]` 按引用捕获；如果回调比被引用对象活得更久，就可能出现悬空引用。

任务代码中 `nonlocal active_nav` 表示修改外层函数的变量。它与 `global` 不同，作用范围是封闭函数。它也不是线程锁；多线程读写共享状态仍需要同步。C++ 插件通过 `std::mutex` 和 `lock_guard` 保护请求数据，以便传输回调和仿真更新之间安全交接。

## 语言边界怎样连接

第一种边界是 ROS 消息/action：Python 客户端与 C++ 服务器通过生成的类型通信。双方不用链接同一份业务源码，但需要一致的接口定义与语义。

第二种边界是子进程 JSON：Python 构造 CSR 图或代价矩阵，C++ 程序调用 METIS、Fields2Cover、OR-Tools，结果再以 JSON 返回。这样可以把成熟原生库包装成清楚的输入输出；代价是序列化开销和额外错误处理。

第三种边界是 Python 调用 NumPy、SciPy 等库。你写的外层是 Python，矩阵和邻近搜索常在编译代码中执行。不能只按文件扩展名推断运行性能。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### rig_r08.py 第 5 至 12 行

[离线带行号源码](../../baseline/l02_views/faf89efd4348-rig_r08.py.html#L5) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/rig_r08.py#L5)

```python
def effective_rig(base,near):
    rig=copy.deepcopy(base)
    rig.update(schema='R08-EFFECTIVE-SENSORS/1',base_calibration_revision=base['revision'],near_sensor=copy.deepcopy(near))
    rig['lidar'].update(horizontal=2048,vertical=128)
    rig['branches']['perception_sampling']=dict(max_rings=64,max_azimuth=512,near_channels='all original independent rays',localization='full 128 x 2048')
    rig['sampling_change_reason']='Actual extra simulator rays for sparse frozen-map landmarks; no repeated-point inflation or relaxed quality gates'
    rig['revision']=canonical({k:v for k,v in rig.items() if k!='revision'})
    return rig
```

deepcopy 保留原标定字典。update 只改变明确的采样与版本信息；canonical 对剔除旧 revision 后的内容计算新标识。函数返回派生配置，不直接移动任何车体。

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

const 引用避免修改输入；at 在必需字段缺失时失败；value 为可选字段提供默认。检查授权 scope、数字类型、有限性和距离范围后，才返回受界限约束的精停进展尺度。

<!-- generated-production-end -->

## 编译并运行一个真实 C++ 判据例子

当前 Linux 已有 g++ 与 nlohmann/json 时：

```bash
cd /home/ceihao/self_learning
mkdir -p build
g++ -std=c++17 -Wall -Wextra -pedantic examples/precision_demo.cpp -o build/precision_demo
./build/precision_demo
```

`-std=c++17` 选择语言标准；`-Wall -Wextra -pedantic` 开启常用诊断；`-o` 指定输出文件。`./` 表示执行当前目录中的相对路径，避免与 PATH 中同名程序混淆。这个例子包含固定生产头文件，实际调用 `progressDelta`，不启动 ROS。

输出的普通进展尺度为 0.100 m，合法精停示例为 0.009 m，越界精停请求回退为 0.100 m。小尺度只在合格精停上下文使用，不能把它理解为普遍放松无进展监督。

## Python 练习

```bash
python3 examples/l02/labs.py 04
```

先读函数的参数检查，再看正常返回，最后看异常。`try` 只包住预期可能失败的操作，捕获明确异常类型；不要把所有异常都吞掉然后继续给机器人发命令。输出应有一个正常距离及四个被拒绝的输入。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_04.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab04():
    """Validation prevents bool, NaN and absent data masquerading as a number."""
    def distance(x):
        if isinstance(x, bool) or not isinstance(x, (int, float)):
            raise TypeError('numeric distance required')
        if not math.isfinite(x) or x < 0:
            raise ValueError('finite nonnegative distance required')
        return x
    rejected = []
    for value in [None, True, float('nan'), -1]:
        try: distance(value)
        except (TypeError, ValueError): rejected.append(repr(value))
    assert len(rejected) == 4
    return dict(valid_m=distance(.03), rejected=rejected)
```

```text
{
  "chapter": "04",
  "passed": true,
  "result": {
    "valid_m": 0.03,
    "rejected": [
      "None",
      "True",
      "nan",
      "-1"
    ]
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 库、模块与构建文件

Python `import json` 导入标准库；`import numpy` 需要对应环境安装 NumPy；`from sensor_msgs.msg import Imu` 依赖 ROS 消息生成包。C++ `#include` 让编译器看到声明，`target_link_libraries` 让链接阶段找到实现。头文件找得到不等于库能成功链接。

`package.xml` 声明 ROS 包依赖，`CMakeLists.txt` 描述构建目标，colcon 在工作区按依赖顺序组织多个包。第 26 章会区分系统包、Python 虚拟环境和项目构建物，避免把所有安装问题都归结为 pip。

## 排错和自测

Python `KeyError` 表示键不存在，先检查输入 schema 与版本；`TypeError` 往往是参数或类型不匹配；`ImportError` 要检查解释器与搜索路径。C++ 编译错误、链接错误、运行期异常是不同阶段，分别看第一条有效诊断、缺失符号或异常消息。

问：为什么不用 Python 把 C++ 算法完整重写一遍学习？答：可以写小型原理例子，但生产算法有大量边界和工程细节。教材明确标出教学模型与实际调用边界，避免把简化例子误装进运行链路。

练习：给 `lab04.distance` 增加单位说明，分别传入 0、0.03、NaN 和字符串。解释为何 0 合法而负数非法；这只是该函数的距离合同，不是所有物理量都不能为负。
