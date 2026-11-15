# 第 17 章 预测与经验缓存实验

复杂模型是否值得启用，需要同时看预测质量、实时预算和闭环任务结果。项目保留了 IMM、学习型候选与经验缓存实验，但当前默认不启用这些候选。历史实验 PASS 表示按当时实验合同完成检查，不代表所有方法都比基线更好。

## 为什么恒速模型会不足

恒速模型适合短时平稳运动，遇到起停、转弯和交互容易偏离。增加复杂度有两条路线：用多种物理模型表达不同运动模式，或使用学习模型从历史与邻居关系预测多种未来。两者都应输出不确定性，而不是只给一条看似确定的线。

IMM 即交互多模型滤波。当前候选包含 CV、CA、左转、右转，状态为 [x,y,vx,vy,ax,ay]。每步先按模式转移概率混合上一估计，再分别预测与观测更新，用观测似然更新模式概率。未来模式分开保留，避免把左转与右转平均成一条不存在的直线。

## 学习型候选的边界

Trajectron 候选使用观测历史与交互特征。模型输入必须是当前及过去可获得信息，不能把仿真脚本未来位置或评估标签喂给在线预测。训练集、调参与测试数据需要分开；同一条轨迹切片混进训练和测试会夸大泛化能力。

类别识别不确定时，不能假装目标一定是行人。部署还需要权重版本、预处理、设备、推理耗时、超时回退与授权许可。模型源码可用不等于在当前车载算力与农田分布上有效。

## 怎样评价预测

ADE 是多个未来时刻的平均位置误差，FDE 是最后时刻误差，单位通常为米。多模态模型还要说明取最优候选还是概率加权；只报告最优候选会掩盖模式选择问题。闭环还需要看漏报冲突、误停、任务完成、停车与计算延迟。

没有碰撞样本不能推导漏报率为零。若可评估 eligible 数为 0，比例应为空或未定义，而不是写 0%。同样，离线误差下降不保证闭环更快，模型可能让系统更保守或引入额外等待。

## 当前缓存究竟缓存什么

`MemoryEngine` 缓存帮助选择模型，并不直接重放旧预测。命中后仍使用当前状态的 IMM 得到新预测，且安全门控继续检查当前冲突、时效性和不确定性。特征包含上下文，不能只按尺寸或颜色认定行为相同。

三组对照为 full、event、cache：full 对合格人类目标调用学习模型；event 根据事件触发减少调用；cache 在事件与类别满足时查询经验，命中则使用新鲜轻量模型。比较缓存时必须把特征、查询、维护、验证、观测和推理成本一起算入。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### imm15.py 第 13 至 30 行

[离线带行号源码](../../baseline/l02_views/8092e8e61ace-imm15.py.html#L13) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/motion_prediction/scripts/imm15.py#L13)

```python
def transition(mode, dt, turn_rate=.4):
    f = np.zeros((6, 6))
    f[:2, :2] = np.eye(2)
    if mode in ('CV', 'CA'):
        f[:2, 2:4] = np.eye(2)*dt
        f[2:4, 2:4] = np.eye(2)
        if mode == 'CA':
            f[:2, 4:] = np.eye(2)*(.5*dt*dt)
            f[2:4, 4:] = np.eye(2)*dt
            f[4:, 4:] = np.eye(2)
    else:
        omega = turn_rate if mode == 'TURN_LEFT' else -turn_rate
        s, c = math.sin(omega*dt), math.cos(omega*dt)
        f[:2, 2:4] = np.array([[s, c-1], [1-c, s]])/omega
        rotation = np.array([[c, -s], [s, c]])
        f[2:4, 2:4] = rotation
        f[4:, 2:4] = omega*np.array([[0, -1], [1, 0]])@rotation
    return f
```

同一个六维状态在 CV、CA 与左右转弯下使用不同 F。CA 包含 dt²/2 和加速度积分；转弯矩阵使用 sin/cos 表达协调转动。这是运动模型候选，不包含未来标签。

### engine16.py 第 15 至 43 行

[离线带行号源码](../../baseline/l02_views/104359d4742a-engine16.py.html#L15) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/experience_memory/scripts/engine16.py#L15)

```python
 def predict(self,frames,now=None):
  start=time.perf_counter();cpu=time.process_time();observe=time.perf_counter()
  for frame in frames:self.base.observe(frame)
  observe_ms=(time.perf_counter()-observe)*1000;frame=frames[-1]
  feature_start=time.perf_counter();fs=self.base.history.features(frame['t']);items=[]
  for f in fs:
   if frame['t']-f['t']>.6:continue
   r=self.base.history.tracks[f['id']][-1]
   if f['classification']!='person' and np.linalg.norm(r['velocity'])<.05:continue
   if len(items)>=6:break
   x=context(f,fs,frame);event,reasons=self.trigger.event(f['id'],frame['t'],x);items.append((f,x,event,reasons))
  for key in list(self.trigger.last):
   if key not in self.base.history.tracks:del self.trigger.last[key]
  feature_ms=(time.perf_counter()-feature_start)*1000
  predictions=[];decisions=[];infer_ms=query_ms=maintenance_ms=0.;learned_calls=imm_calls=0
  for f,x,event,reasons in items:
   answer=dict(hit=False,reason='NOT_QUERIED',query_ms=0.,maintenance_ms=0.)
   if self.group=='cache' and event and f['classification']=='person':answer=self.store.lookup(x,time.time() if now is None else now)
   full=f['classification']=='person' and (self.group=='full' or (event and not answer['hit']))
   ts=time.perf_counter()
   if full:prediction=self.learned.predict(f,fs,3.2);learned_calls+=1
   else:prediction=self.base.filters[f['id']].predict(3.2);imm_calls+=1
   infer_ms+=(time.perf_counter()-ts)*1000
   prediction.update(id=f['id'],size=f['size'],classification=f['classification'],observed_age_s=frame['t']-f['t'],candidate_status='LEARNED_PERSON' if full else 'IMM_CURRENT_STATE')
   predictions.append(prediction);query_ms+=answer['query_ms'];maintenance_ms+=answer['maintenance_ms']
   decisions.append(dict(id=f['id'],t=f['t'],event=event,event_reasons=reasons,selected='learned' if full else 'fresh_imm',context=x,lookup=answer))
  return dict(t=frame['t'],frame='odom',method=self.group,status='OK',forecasts=predictions,future_labels_used=False,
   memory=dict(decisions=decisions,observe_ms=observe_ms,feature_ms=feature_ms,query_ms=query_ms,maintenance_ms=maintenance_ms,inference_ms=infer_ms,learned_calls=learned_calls,imm_calls=imm_calls),
   wall_ms=(time.perf_counter()-start)*1000,cpu_ms=(time.process_time()-cpu)*1000)
```

observe 消费当下历史，context/Trigger 决定事件，lookup 仅影响是否调用学习模型。命中后仍调用当前 IMM predict；wall/cpu 与 feature/query/maintenance/inference 分项累计，防止只统计节省的调用数。

<!-- generated-production-end -->

## 净收益的计算

设完整推理成本 C，命中率 h，额外特征/查询/验证/维护总成本 O，则简化平均成本为 `O+(1−h)C`。只有 `hC>O` 才可能节省计算；还未考虑错误命中造成额外等待和重规划。

教学例中 C=8 ms、O=2.7 ms，需要命中率超过 33.75% 才在这个成本模型下有收益。真实 S16 应读实测结果，不能把这个数当项目实测加速率。缓存减少调用数，却可能增加 CPU、延迟或任务时间。

## 真实结论怎样阅读

历史 S15/S16 结果保留在 evidence 文档。它们包含场景、任务范围、是否完成、候选状态与开销。某些实验允许有界 PARTIAL 仍完成实验合同，所以必须分别读 experiment passed 与 mission completed。当前默认关闭的选择，是承认实测未支持无条件部署，而不是否定预测研究价值。

阅读 [历史 S15](../../evidence/docs/optimization_v2/S15/RESULTS.md) 与 [历史 S16](../../evidence/docs/optimization_v2/S16/RESULTS.md) 时，先看摘要和 scope，再看比较表，不从某一行较小 latency 推断整体胜出。

## 练习与排错

执行 `python3 examples/l02/labs.py 17`，比较 10%、40%、80% 命中率。保持 C 不变增加查询成本，观察收益翻转。这里计算几乎瞬间完成，不需要启动模型或仿真。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_17.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab17():
    """Cache break-even includes feature, query, validation and maintenance cost."""
    inference=8.; feature=1.; query=.5; verify=1.; maintenance=.2
    overhead=feature+query+verify+maintenance
    break_even=overhead/inference
    rows=[dict(hit_rate=h,cost_ms=overhead+(1-h)*inference,
               saving_ms=inference-overhead-(1-h)*inference) for h in [.1,.4,.8]]
    assert rows[0]['saving_ms']<0 and rows[-1]['saving_ms']>0
    return dict(kind='synthetic costs, not measured S16 speedup',break_even=break_even,cases=rows)
```

```text
{
  "chapter": "17",
  "passed": true,
  "result": {
    "kind": "synthetic costs, not measured S16 speedup",
    "break_even": 0.3375,
    "cases": [
      {
        "hit_rate": 0.1,
        "cost_ms": 9.9,
        "saving_ms": -1.9000000000000004
      },
      {
        "hit_rate": 0.4,
        "cost_ms": 7.5,
        "saving_ms": 0.5
      },
      {
        "hit_rate": 0.8,
        "cost_ms": 4.3,
        "saving_ms": 3.7
      }
    ]
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

常见错误包括历史数据泄漏、缓存版本不匹配、把过期预测当新观测、仅统计命中率、不计维护成本、没有超时回退。修复顺序是恢复时间与版本边界，再比较完整开销和任务结果。

自测：一种模型 ADE 更好但 p95 推理时间超过控制预算，应如何部署？答案可能是降低触发频率、异步候选、降级回退或继续关闭；不能让过期预测取得执行权限。最后写出一份至少包含质量、成本和闭环影响的对照表。
