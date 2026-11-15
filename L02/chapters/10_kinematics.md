# 第 10 章 差速与阿克曼运动学

车辆能否转进下一条条带，首先是几何问题。控制器只能在物理能力内跟踪路径，不能把前轮转向车辆变成能原地旋转的差速车。本章从后轴参考点、轮距、轴距和车体扫掠解释为什么割草轨迹可能出现大圈。

## 差速模型

设左右轮线速度为 vL、vR，轮距为 T，则车体前向速度 `v=(vR+vL)/2`，角速度 `ω=(vR−vL)/T`。轮半径为 r 时，关节角速度分别为 `(v−Tω/2)/r` 与 `(v+Tω/2)/r`。v 的单位为 m/s，ω 和轮速单位为 rad/s。

当 v=0、ω 非零时，两侧轮反向转动，理论上可以原地旋转。四轮滑移差速还存在接触滑移，因此有效轮距可能不同于几何轮距。项目保留标定值与几何值的区别，不能只测模型宽度就认为转角积分一定准确。

## 阿克曼与自行车模型

自行车模型把左右前轮等效为一个转向轮。轴距 L、等效前轮转角 δ、后轴中心速度 v 对应曲率 `κ=tanδ/L`，转弯半径 `R=1/|κ|`，角速度 `ω=vκ`。速度接近零时不能任意使用 w/v；当前执行器也明确禁止阿克曼零速原地旋转。

真正左右前轮转角不同。若轮距为 T，给定曲率 κ，内外轮可按 `atan(Lκ/(1∓Tκ/2))` 求出。内轮通常转得更多，关节极限应逐轮满足。把 45° 的单个关节限制直接当作自行车等效转角，可能高估实际曲率能力。

## 当前割草车的真实配置

固定 R10 mowing mission 中，轴距 1.638 m、轮距 1.8 m、车体声明外长 2.78 m、外宽 2.77 m，最大曲率约 0.333518 1/m，对应后轴基准最小半径约 2.99834 m。足迹相对后轴地面投影从 x=−0.571 到 2.209 m，y=±1.385 m。工具有效幅宽为 1.6 m。

这些参数说明它是项目 Dino 几何的前阿克曼简化适配，配置明确 `hardware_equivalence=false`。它不是小型零转向草坪机，也不能据模型名宣称与某个商用机硬件完全一致。当前动力学是 Gazebo 刚体接触与受限关节速度执行，并未验证实机电机扭矩、功率、轮胎和农业地面性能。

## 后轴圆与车体外圈不同

后轴中心半径 R≈3 m，不代表整车只需 6 m 宽空间。粗略考虑最外前角，外圈半径可估为 `sqrt((R+半宽)²+前悬参考距离²)`。代入当前足迹约得到 4.91 m；完整连续转弯扫掠直径可接近 9.8 m，再加安全边距。这个计算是包络直觉，不替代完整多边形扫掠验证。

如果条带间距只有约 1.4 m，简单前进 U 形转弯很难直接连接相邻条带。可选择跳条、较大的合法地头、Reeds–Shepp 倒车组合或重新排列方向。路径超出工作条带并不自动不合理，但必须始终在允许通行区域，不能穿越坡面禁区。

## 实际执行器做了什么

`VehicleDrive.cc` 首先检查会话、版本、epoch、sequence 和时间戳，之后才使用请求速度。它限速、限加减速、检查转向跟踪误差，并把速度映射到关节命令。执行器没有直接把底盘位姿设置到规划点；车辆运动来自物理关节与接触。

转向轮尚未对准时，代码把车速降到低速，并在误差持续过久时锁存故障。这解释了为什么控制器请求 0.5 m/s，实际速度仍可能只有 0.1 m/s。不同层的速度限制需要逐层排查。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### VehicleDrive.cc 第 80 至 114 行

[离线带行号源码](../../baseline/l02_views/7e4ef90fa8ef-VehicleDrive.cc.html#L80) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/src/VehicleDrive.cc#L80)

```cpp
       lastSequence=request.at("sequence");lastRequestSim=t;
       std::string mode=request.at("mode");
       if(mode=="STOP")latch("STOP_REQUEST");
       else if(mode=="ARM" && !info.paused && stopped && std::abs(v)<.001 && std::abs(w)<.001){latched=false;reason="ARMED";}
       else if(mode!="DRIVE" && mode!="ARM")latch("INVALID_MODE");
     }catch(const std::exception &){latch("LEASE_REJECTED");}
   }
   if(age>c["safety"]["actuator_wall_s"].get<double>() || t-lastRequestSim>c["safety"]["actuator_sim_s"].get<double>())latch("ACTUATOR_WATCHDOG");
   if(!valid)latch("ENCODER_MISSING");
   double v=0,w=0;
   if(!latched && !info.paused && request.is_object() && request.value("mode",std::string())=="DRIVE"){
     v=request.value("v",0.);w=request.value("w",0.);
     v=std::clamp(v,-c["limits"]["reverse"].get<double>(),c["limits"]["max_speed"].get<double>());
     w=std::clamp(w,-c["limits"]["yaw_rate"].get<double>(),c["limits"]["yaw_rate"].get<double>());
   }
   if(ack && std::abs(v)<.001){v=w=0;}
   if(!info.paused && dt>0 && dt<.1 && valid){
     double k=heldK,error=0;
     if(ack && v!=0)k=std::clamp(w/v,-c["steering"]["max_curvature"].get<double>(),c["steering"]["max_curvature"].get<double>());
     if(ack){heldK=k;double al=atan(L*k/(1-T*k/2)),ar=atan(L*k/(1+T*k/2));error=std::max(abs(al-q[2]),abs(ar-q[3]));
       if(error>c["safety"]["alignment_error_rad"].get<double>()){
         v=std::clamp(v,-c["safety"]["alignment_speed_mps"].get<double>(),c["safety"]["alignment_speed_mps"].get<double>());
         if(trackingSince<0)trackingSince=t;
         if(t-trackingSince>c["safety"]["steering_tracking_timeout_s"].get<double>()){latch("STEERING_TRACKING_FAULT");v=0;}
       }else trackingSince=-1;
       const double gain=c["steering"]["gain"],rate=c["steering"]["rate_rad_s"];
       command[2]=std::clamp(gain*(al-q[2]),-rate,rate);command[3]=std::clamp(gain*(ar-q[3]),-rate,rate);
     }
     if(speed*v<0)v=0;
     const double rate=(abs(v)<abs(speed))?c["limits"]["deceleration"].get<double>():c["limits"]["acceleration"].get<double>();
     speed+=std::clamp(v-speed,-rate*dt,rate*dt);
     const double wrate=c["limits"]["yaw_acceleration"];yaw+=std::clamp(w-yaw,-wrate*dt,wrate*dt);
     if(ack){double a=tan(q[2]),b=tan(q[3]);k=.5*(a/(L+T*a/2)+b/(L-T*b/2));yaw=speed*k;}
     command[0]=(speed-driveTrack*yaw/2)/r;command[1]=(speed+driveTrack*yaw/2)/r;
     if(!ack){command[2]=command[0];command[3]=command[1];}
```

v,w 先限幅，阿克曼零速不允许转圈。k=w/v 被最大曲率限制，左右转角按轴距与轮距分别计算；转向误差大时降低车速。最后用反馈转角计算实际曲率，再映射四个关节命令。

<!-- generated-production-end -->

## 离线练习与图解任务

`python3 examples/l02/labs.py 10` 使用另一个明确标注的教学尺寸，计算后轴圆、外侧前角圆与左右轮速。它不覆盖当前 Dino 参数，也不参与控制。把输出画成同心参考图，观察外圈为何更大。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_10.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab10():
    """Rear-axle bicycle radius and the larger outer front corner sweep."""
    L=1.2; delta=math.radians(30); half_width=.5; front=1.5
    radius=L/math.tan(delta)
    outer=math.hypot(radius+half_width,front)
    v=.5; omega=v/radius; r=.2; track=1.
    left=(v-track*omega/2)/r; right=(v+track*omega/2)/r
    assert outer>radius and right>left
    return dict(kind='synthetic dimensions, not Dino calibration',rear_radius_m=radius,
                outer_front_sweep_m=outer,omega_rad_s=omega,wheel_rad_s=[left,right])
```

```text
{
  "chapter": "10",
  "passed": true,
  "result": {
    "kind": "synthetic dimensions, not Dino calibration",
    "rear_radius_m": 2.078460969082653,
    "outer_front_sweep_m": 2.983028824715352,
    "omega_rad_s": 0.24056261216234404,
    "wheel_rad_s": [
      1.8985934695941398,
      3.10140653040586
    ]
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

再从固定 mission 文件中找到 `physical_contract` 与 `footprint`：

```bash
cd /home/ceihao/self_learning
rg -n 'wheelbase|max_curvature|nominal_bicycle_radius|footprint|hardware_equivalence' baseline/l02_source/optimization_v2/three_tasks/editions/R10/mowing/mission.json
```

## 排错与自测

转弯半径很大：先核查参考点、最大曲率、实际关节限位和足迹，而不是先调 RPP 前视距离。轨迹绕远：检查是否是障碍、坡面、末端姿态、前进约束或地头空间导致；一个有限时间规划器找不到短路，不等于数学上没有路径。

车体侧面撞边：检查规划是否只验证中心线，是否忽略前角扫掠或工具包络。倒车后方向异常：检查路径切分、航向定义和换挡时是否确认停车。

问：缩小 visual 模型能让车转小圈吗？答：不会。必须修改物理合同、碰撞、足迹、传感器和路径，并重新验证；只改外观不能改变运动学。

练习：解释“最小转弯半径 3 m”包含哪个参考点、哪种运动模型、哪个方向约束。无法回答这些限定时，单个半径数字不足以判断场景是否合理。
