# 第 11 章 传感器模拟与真实误差

仿真传感器的价值在于提供带采样、坐标与误差的数据，而不是直接给任务一个正确位置。本章逐一说明点云、图像、IMU、轮速和 GNSS 的输入语义，并解释为什么不能为了“更真实”随意添加不可核对的噪声。

## 点云中的每个字段

PointCloud2 的 x/y/z 表示传感器坐标中的点位置；fields 记录字段名、偏移和类型；point_step 是每点字节数，row_step 是每行字节数；width/height 描述组织结构。本项目适配器保留 ring 束编号，按原始组织栅格生成，不凭空给每个点编造采集时间。

早期 S04 是 32×1024、10 Hz；当前 R08 派生 rig 把主雷达采样改为 128×2048，以增加稀疏地标的真实模拟射线。R10 mowing 中雷达位置为 [1.1,0,1.5] m。更高采样并不意味着所有原始点都进入 ICP；定位和感知后续仍有各自筛选、降采样和预算。

当前 scan_model 是 `instantaneous_no_point_times`。它不模拟真实旋转雷达逐点扫描畸变，因此不能声称已经验证逐点 deskew。后续接真实旋转雷达时需要驱动提供点时间、扫描模型和高频姿态插值。

## 双目图像

当前 rig 的双目分辨率为 640×360、15 Hz、基线 0.12 m。理想针孔模型下，深度 `Z=fB/d`，f 是像素焦距，B 是左右相机距离，d 是视差像素。视差很小时深度对像素误差特别敏感，所以远处深度通常更不稳定。

图像字段还包括 encoding、step 与 optical frame。RGB 与 BGR 通道顺序不同，不能仅按三字节直接显示。双目配对要求采集时间接近，不能随意把最近左图与很旧右图组成一对。存在图像流不代表当前任务已经把每个物体可靠分类为行人。

## IMU 的测量与缺失项

陀螺仪测角速度，单位 rad/s；加速度计测比力，单位 m/s²，涉及重力与姿态约定。仿真误差包含固定偏置、白噪声和随机游走。随机游走增量按 `sigma_walk*sqrt(dt)` 缩放，而不是每帧加同样方差，否则采样率变化会改变物理误差模型。

当前原始适配 IMU 不提供可信姿态，orientation covariance 首项为 −1 表示该项不可用。四元数填成单位值不意味着已测得绝对方向。后续姿态输入若由预处理得到，必须说明来源，不能把这个占位值当作真实无误差姿态融合。

## 轮速与 GNSS

轮速来自实际关节反馈，加编码器量化、噪声和有效尺度滑移模型。关节转了不保证车体平移同样距离，地面滑移与转向接触都会影响里程计。该误差模型仍比真实轮胎土壤作用简化，需要保留外推边界。

GNSS 数据使用经纬高、状态与协方差；fixed 和 float 模式有不同噪声，且加入相关误差。固定 rig 中给出的厘米级 sigma 是模拟条件，不等于真实环境随时具备 RTK 固定解。单天线静止点也不能可靠给出航向。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### adapter.py 第 48 至 51 行

[离线带行号源码](../../baseline/l02_views/14de986abb94-adapter.py.html#L48) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L48)

```python
    def enqueue(self,k,m,t,size=500):
        f=phase(t,self.profile);delay=float(f.get('delay_s',0));self.seq+=1
        if self.bytes+size>self.c['limits']['queue_bytes']:self.drops['queue_'+k]+=1;return
        heapq.heappush(self.queue,(t+delay,self.seq,k,m,size,t));self.bytes+=size;self.highwater=max(self.bytes,self.highwater);self.flush()
```

延迟队列保存到期时间、序号、消息、大小和采集时间；bytes 约束内存，heapq 先取最早到期项。入队后立即 flush 允许零延迟数据及时发出。

### adapter.py 第 52 至 59 行

[离线带行号源码](../../baseline/l02_views/14de986abb94-adapter.py.html#L52) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L52)

```python
    def flush(self):
        while self.queue and self.queue[0][0]<=self.t+1e-9:
            due,seq,key,m,size,t=heapq.heappop(self.queue);self.bytes-=size
            # A transport recovery can overtake delayed samples. Never time-travel online.
            if t<=self.last_published.get(key,-1):
                self.drops['late_'+key]+=1;self.audit(key,t,event='late_discard',delivery_sim=self.t);continue
            self.last_published[key]=t;self.pub[key].publish(m);self.counts[key]+=1
            self.audit(key,t,event='published',delivery_sim=self.t,delay_sim=self.t-t)
```

flush 不重写消息 header。若某采集时刻不比已发布的新，则记录 late_discard。传输恢复后的旧包不能倒退在线时间；丢弃统计是质量证据的一部分。

### adapter.py 第 80 至 88 行

[离线带行号源码](../../baseline/l02_views/14de986abb94-adapter.py.html#L80) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/sensor_sim_adapter/scripts/adapter.py#L80)

```python
    def imu(self,m):
        t=stamp(m);c=self.c['imu'];rng=self.rng['imu'];dt=max(0,min(.1,t-self.last.get('imu',t)));self.last['imu']=t;f=phase(t,self.profile)
        self.gyro_bias+=rng.normal(0,c['gyro_walk']*math.sqrt(dt),3);self.accel_bias+=rng.normal(0,c['accel_walk']*math.sqrt(dt),3)
        angular=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);accel=np.array([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]);raw_z=float(angular[2])
        angular+=self.gyro_bias+rng.normal(0,c['gyro_sigma'],3);angular[2]+=f.get('gyro_z_bias_rad_s',0);accel+=self.accel_bias+rng.normal(0,c['accel_sigma'],3)
        for i,k in enumerate(['x','y','z']):setattr(m.angular_velocity,k,float(angular[i]));setattr(m.linear_acceleration,k,float(accel[i]))
        m.header.frame_id='imu_link';m.orientation.x=m.orientation.y=m.orientation.z=0.;m.orientation.w=1.;m.orientation_covariance=[-1.,0.,0.,0.,0.,0.,0.,0.,0.]
        m.angular_velocity_covariance=[c['gyro_sigma']**2 if i in [0,4,8] else 0. for i in range(9)];m.linear_acceleration_covariance=[c['accel_sigma']**2 if i in [0,4,8] else 0. for i in range(9)]
        self.audit('imu',t,event='generated',raw_z=raw_z,observed_z=float(angular[2]),bias=self.gyro_bias.tolist());self.enqueue('imu',m,t)
```

随机游走按 sqrt(dt) 增长，角速度与加速度分别加偏置和白噪声。姿态被声明不可用，协方差用 sigma 平方。代码末尾继续走统一延迟队列，而不是绕开时间模型。

<!-- generated-production-end -->

## 延迟和丢包怎样注入

适配器以采集时刻 t 和延迟 delay 构造发送时刻 `t+delay`，保留消息原始 stamp。优先队列 `heapq` 按到期时间发送。网络恢复时新包可能超过旧延迟包，代码使用每类传感器的 last_published 丢弃时间倒退的到达。

随机数按传感器分流种子，便于控制实验复现。相同种子能控制噪声抽样，却不能保证整个多进程物理系统逐位完全一致；调度和数值误差也会改变运行。队列还有字节限制，防止无限堆积旧点云导致内存失控。

## 数据分支的设计理由

定位更需要稳定几何，感知更关心低障碍与动态目标，GUI 更需要有界展示。若把一个消费支路卡顿传给全部传感器，就会让控制和展示互相拖累。当前代码分流 motion/vision 等进程，并记录到达年龄与丢弃原因。

## 实验与手工观察

执行 `python3 examples/l02/labs.py 11`，四个教学包中 1.1 s 的包延迟到达，被较新的包越过后拒绝。观察 accepted 与 rejected 中的 stamp 和 arrival，它们刻意不相同。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_11.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab11():
    """Delayed packets keep acquisition stamps; old arrivals can be rejected."""
    packets=[(1.0,.0),(1.1,.4),(1.2,.0),(1.3,.0)]
    queue=[(t+delay,t) for t,delay in packets];heapq.heapify(queue)
    last=-math.inf; accepted=[]; rejected=[]
    while queue:
        arrival,stamp=heapq.heappop(queue)
        if stamp<=last:rejected.append(dict(stamp=stamp,arrival=arrival));continue
        last=stamp;accepted.append(dict(stamp=stamp,arrival=arrival))
    assert [p['stamp'] for p in rejected]==[1.1]
    return dict(kind='synthetic timing',accepted=accepted,rejected=rejected)
```

```text
{
  "chapter": "11",
  "passed": true,
  "result": {
    "kind": "synthetic timing",
    "accepted": [
      {
        "stamp": 1.0,
        "arrival": 1.0
      },
      {
        "stamp": 1.2,
        "arrival": 1.2
      },
      {
        "stamp": 1.3,
        "arrival": 1.3
      }
    ],
    "rejected": [
      {
        "stamp": 1.1,
        "arrival": 1.5
      }
    ]
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

现场先用 `ros2 topic list -t` 确定名称，再查看 header、frame_id、width、height、encoding 与 covariance。不要直接在终端输出完整高密度点云来判断质量；GUI 和质量摘要更合适。采样率不足应同时查生产、桥接、适配、订阅与处理耗时。

## 自测与常见故障

图像颜色异常先查 encoding；点云倾斜先查外参和 frame；IMU 航向异常先查单位、坐标与偏置；GNSS 地图整体错位先查基准和杆臂；消息延迟持续增长先查队列和计算预算。

问：给点云补一个 `time` 字段是否等于支持真实扫描去畸变？答：只有时间来自实际扫描模型或驱动才有意义，伪造时间会给算法错误运动补偿。

练习：为五种传感器各写“测量量、单位、坐标、采集频率、缺失/无效表示、主要误差”六项。先理解数据合同，再谈融合算法。
