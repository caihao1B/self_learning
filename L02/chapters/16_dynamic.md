# 第 16 章 动态障碍与保持距离跟随

这里的“跟随”指跟随前方同向移动障碍物并保持距离，不是单纯跟踪预设路径。要做出这个决定，系统需要知道目标是否可靠、是否在当前路径上、是否同向、间距是否足够，以及前方路径是否仍有效。

## 检测 关联与轨迹

检测在一帧中找出候选物体，关联把连续帧中的候选连接成同一轨迹。当前几何 Tracker 保存位置、速度、协方差、尺寸、观测时间、命中次数和状态。它使用短时恒速预测辅助关联，并通过距离、NIS 和尺寸差筛选匹配。

候选匹配形成代价矩阵，SciPy `linear_sum_assignment` 求一对一分配。若两个选择代价太接近，代码把它视作歧义而不是强行确定身份。轨迹会经历 TENTATIVE、OBSERVED、COASTING 或 AMBIGUOUS_COAST；短时间未见仍可外推，但不能当作新鲜真实观测。

这是追踪层的短时运动模型，与 S15 未来行为预测候选分开。使用恒速关联不等于整个系统声称所有未来障碍都恒速。

## 为什么投影到实际执行路径

世界中“在机器人前面”不足以判断同路。弯道上邻近路径可能在车体正前方；另一条平行条带的物体也可能很近。当前 `tracker_r08.py` 订阅 active_path，把目标与车体投影到该路径的弧长坐标。

弧长差给出沿路径前后关系，路径切线给出同向速度，法向分量给出横穿倾向。净间距还要减目标半径与车体前伸，不能用两个中心之间距离直接代替保险距离。

## TTC 与距离

碰撞时间 TTC 的一维近似是 `gap/(v_ego−v_leader)`，仅当闭合速度为正且保持当前运动时成立。前车更快时没有有限追撞 TTC，但横穿和侧向风险仍可能存在。TTC 大不代表足迹不相交，也不能忽略急停距离和延迟。

当前 BT 的 leader 判据还要求 blocked、on_path、same_direction、confidence≥0.75、speed>0.05、最小间距、最小 TTC 与 path_valid。满足条件才进入 FOLLOW，速度上限由前车速度和间距误差调节，并继续受底盘独立门控限制。

<!-- generated-production-start -->
## 固定生产代码逐段阅读

下面是固定提交中的原样节选。完整文件有离线行号页；片段可能省略上下文，不能单独当成可运行脚本。读完每段后，回到调用者确认它在当前任务中如何使用。

### tracker.py 第 6 至 36 行

[离线带行号源码](../../baseline/l02_views/6501f9350ae5-tracker.py.html#L6) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/object_fusion/scripts/tracker.py#L6)

```python
 def update(self,t,observations):
  if self.last is not None and t<=self.last:raise ValueError('nonmonotonic tracking time')
  self.last=t
  for tr in self.tracks.values():
   dt=min(2.,t-tr['t']);F=np.eye(4);F[0,2]=F[1,3]=dt;G=np.array([[dt*dt/2,0],[0,dt*dt/2],[dt,0],[0,dt]]);tr['x']=F@tr['x'];tr['P']=F@tr['P']@F.T+G@G.T*.8;tr['t']=t;tr['state']='COASTING'
  ids=list(self.tracks);cost=np.full((len(ids),len(observations)),1e5);H=np.eye(4)[:2]
  for i,k in enumerate(ids):
   tr=self.tracks[k]
   for j,o in enumerate(observations):
    delta=np.array(o['position'][:2])-tr['x'][:2];S=tr['P'][:2,:2]+np.array(o['covariance']).reshape(3,3)[:2,:2];nis=float(delta@np.linalg.solve(S,delta));size=np.linalg.norm(np.array(o['size'])-tr['size'])
    if np.linalg.norm(delta)<1.5 and nis<9.21 and size<1.0:cost[i,j]=nis+size
  matches=[];ambiguous=set()
  if cost.size:
   rr,cc=linear_sum_assignment(cost)
   for i,j in zip(rr,cc):
    if cost[i,j]>=1e5:continue
    alternatives=np.sort(cost[:,j]);row=np.sort(cost[i]);uncertain=(len(alternatives)>1 and alternatives[1]-alternatives[0]<.5) or (len(row)>1 and row[1]-row[0]<.5)
    if uncertain:ambiguous.add(j);self.tracks[ids[i]]['state']='AMBIGUOUS_COAST';continue
    matches.append((ids[i],j))
  used=set()
  for k,j in matches:
   tr=self.tracks[k];o=observations[j];R=np.array(o['covariance']).reshape(3,3)[:2,:2];K=tr['P']@H.T@np.linalg.inv(H@tr['P']@H.T+R);tr['x']+=K@(np.array(o['position'][:2])-H@tr['x']);I=np.eye(4)-K@H;tr['P']=I@tr['P']@I.T+K@R@K.T;tr['size']=.7*tr['size']+.3*np.array(o['size']);tr['obs']=o;tr['last_seen']=t;tr['state']='OBSERVED';tr['hits']+=1;used.add(j)
  for j,o in enumerate(observations):
   if j in used or j in ambiguous:continue
   k=self.next_id;self.next_id+=1;self.tracks[k]=dict(id=k,t=t,last_seen=t,x=np.array([*o['position'][:2],0.,0.]),P=np.diag([.0036,.0036,.5,.5]),size=np.array(o['size']),obs=o,state='TENTATIVE',hits=1)
  for k in list(self.tracks):
   if t-self.tracks[k]['last_seen']>1.2:del self.tracks[k]
  result=[]
  for k,tr in self.tracks.items():
   o=tr['obs'];age=t-tr['last_seen'];result.append(dict(id=k,t=t,observation_t=tr['last_seen'],frame='odom',position=[*tr['x'][:2].tolist(),o['position'][2]],velocity=tr['x'][2:].tolist(),size=tr['size'].tolist(),covariance=tr['P'].reshape(-1).tolist(),classification=o.get('classification','unknown_obstacle') if age<.6 else 'unknown_obstacle',confidence=o.get('confidence',0.) if age<.6 else 0.,state=tr['state'],confirmed=tr['hits']>=3,observed_age_s=age,semantic=o.get('semantic',{})))
  return result,[int(j) for j in sorted(ambiguous)]
```

先做短时状态预测，再构造距离/NIS/尺寸代价；匈牙利分配后仍检查歧义。匹配成功更新状态与命中次数，超过丢失时间删除，输出时保留 observed_age 与 confirmed。

### tracker_r08.py 第 26 至 46 行

[离线带行号源码](../../baseline/l02_views/e0c967e86c06-tracker_r08.py.html#L26) · [固定 Git 提交](https://github.com/caihao1B/ROS2_BT-/blob/9aff8337b7794f704c5556a973a5220e73f17808/optimization_v2/ws/src/task_platform/scripts/tracker_r08.py#L26)

```python
 def geometry(m):
  d=json.loads(m.data);t=d['t']
  if tracker.last is not None and t<=tracker.last:return
  tracks,ambiguous=tracker.update(t,d['objects']);recent_motion=motion_memory.update(t,tracks);loc=st['loc'];result=dict(t=t,valid=False,on_path=False,tracks=tracks,ambiguous=ambiguous,path_valid=False,path_epoch=st['path_epoch'],source='CURRENT_PCL_GEOMETRY_AND_MEASURED_ODOMETRY',truth_used=False,predicted_conflict=False,crossing_evidence=[])
  if loc and loc.get('valid') and abs(t-loc['t'])<.3:
   tr=loc['transform'];co,si=math.cos(tr[2]),math.sin(tr[2]);R=np.array([[co,-si],[si,co]]);ego=np.array(loc['pose'][:2]);original_line=LineString(st['path'][:,:2]);end=st['path'][-1,:2];delta=end-st['path'][-2,:2];tangent_end=delta/max(1e-6,np.linalg.norm(delta));line=LineString(np.vstack([st['path'][:,:2],end+7*tangent_end]));ego_arc=line.project(Point(ego));candidates=[];activity_eligible=set()
   for track in tracks:
    pos=R@np.array(track['position'][:2])+tr[:2];obj_arc=line.project(Point(pos));along=obj_arc-ego_arc;radius=max(track['size'][:2])/2;lateral=line.distance(Point(pos));gap=along-radius-max(x for x,y in cfg['vehicle']['footprint'])
    if track['confirmed'] and track['state']=='OBSERVED' and track['observed_age_s']<.25:
     rotation=np.zeros((4,4));rotation[:2,:2]=R;rotation[2:,2:]=R;C=rotation@np.array(track['covariance']).reshape(4,4)@rotation.T;future=crossing(st['path'],ego,max(.15,loc.get('v',0)),pos,R@np.array(track['velocity']),C,track['size'],cfg['vehicle']['footprint'],recent_motion=track['id'] in recent_motion)
     future['recent_observed_motion']=track['id'] in recent_motion
     if track['id'] in recent_motion and future.get('reason')!='SAME_DIRECTION_LEADER_BRANCH':activity_eligible.add(track['id'])
     if future['conflict']:result['predicted_conflict']=True;result['crossing_evidence'].append(dict(id=track['id'],**future))
    if 0<along<7 and lateral<max(abs(y) for x,y in cfg['vehicle']['footprint'])+.1+radius and track['confirmed']:
     a0=np.asarray(line.interpolate(max(0,obj_arc-.1)).coords[0]);b0=np.asarray(line.interpolate(min(line.length,obj_arc+.1)).coords[0]);tangent=(b0-a0)/max(1e-6,np.linalg.norm(b0-a0));normal=np.array([-tangent[1],tangent[0]]);velocity=R@np.array(track['velocity']);speed=float(velocity@tangent);side=float(velocity@normal);hits=tracker.tracks[track['id']]['hits'];beyond_goal=obj_arc>original_line.length+.1
     if beyond_goal and not (speed>.05 and abs(side)<.15 or track['id']==st.get('last_leader')):continue
     confidence=min(.98,.5+.08*hits) if track['observed_age_s']<.25 and track['state']=='OBSERVED' else 0.;candidates.append(dict(id=track['id'],gap=gap,lateral=lateral,speed=speed,same_direction=speed>.05 and abs(side)<.15,confidence=confidence,ttc=gap/max(.001,loc.get('v',0)-speed),state=track['state']))
   envelope=activity.update(t,tracks,activity_eligible);odom_path=(st['path'][:,:2]-np.asarray(tr[:2]))@R;dynamic=route_conflict(envelope,odom_path,loc['odom'][:2],cfg['vehicle']['footprint']);result.update(activity_evidence=dynamic,activity_clear_ahead_m=dynamic['clear_ahead_m']);result['predicted_conflict']=result['predicted_conflict'] or dynamic['conflict']
   grid=st['grid'];evidence=dict(valid=False,reason='OBSERVATION_STALE')
   if grid and abs(t-(grid.header.stamp.sec+grid.header.stamp.nanosec*1e-9))<.3 and grid.header.frame_id=='odom':
    cost=apply_grid(np.asarray(grid.data).reshape(grid.info.height,grid.info.width),[grid.info.origin.position.x,grid.info.origin.position.y],grid.info.resolution,envelope);combined=copy.deepcopy(grid);combined.data=cost.astype('int8').ravel().tolist();riskpub.publish(combined);evidence=validate(cost,[grid.info.origin.position.x,grid.info.origin.position.y],grid.info.resolution,loc['odom'],loc['pose'],tr,st['path'],cfg['vehicle']['footprint'])
```

把轨迹转到 map，投影到当前路径弧长；沿向速度与横向速度决定同向关系。gap 扣除目标大小和车体前伸；只有新鲜观测与有效路径才形成可用证据。

<!-- generated-production-end -->

## 横穿为何采用不同处理

横穿目标的速度沿法向，未来可能占据路径。当前实现结合轨迹协方差、尺寸、当前路径与最近观测活动判断冲突，并对短期活动区域保持记忆。这个记忆用于避免目标短暂遮挡后被当作完全消失，不是复用一条陈旧的未来轨迹。

等待应有清除迟滞：目标刚离开一帧不立即高速恢复，连续证据足够再继续。等待也有期限；持续阻塞后尝试重规划、延期其它义务或请求人工。有限搜索失败只证明本次搜索没找到可接受路径，不证明整个连续空间绝对无路。

## 演示时应该看哪些变化

正常跟随：leader 身份与观测状态稳定，净间距合理，BT FOLLOW，速度上限随间距变化，轨迹持续前进。横穿：冲突证据出现，WAIT/STOP，实测速度下降，清除条件满足后恢复。持久阻塞：等待时间增加、候选替代路径及拒绝原因可见，最终可能 HUMAN 并保留残余。

仅视频中“车慢了”无法区分跟随成功、定位丢失或控制器卡住，应把对应原因码和输入一起记录。

## 练习

执行 `python3 examples/l02/labs.py 16`。教学间距 2 m、车速 0.6 m/s、前车 0.3 m/s，TTC≈6.67 s。把前车改为横向 0.4 m/s，不能继续用同向跟随判据。生产阈值应读任务配置，练习中的 1 m/3 s 不写回工程。

<!-- generated-lab-start -->
### 本章练习核心代码与实际输出

[完整教学脚本](../../examples/l02/labs.py) · [本章完整输出](../../validation/L02/lab_16.json)

代码只使用 Python 标准库；合成输入与历史记录在输出 kind 中区分。公共 ROOT、read、wrap、connected 等辅助函数在完整脚本中定义。

```python
def lab16():
    """TTC uses closing speed; following also needs geometry and freshness."""
    gap=2.; ego=.6; leader=.3
    closing=ego-leader; ttc=gap/closing if closing>0 else None
    same_direction=True; observed=True; path_valid=True
    teaching_allowed=all([same_direction,observed,path_valid,gap>=1.,ttc is None or ttc>=3.])
    assert teaching_allowed
    return dict(kind='synthetic thresholds',gap_m=gap,TTC_s=ttc,
                following_candidate=teaching_allowed,crossing_velocity_mps=[0.,.4],
                lesson='A transverse mover must not reuse this following decision.')
```

```text
{
  "chapter": "16",
  "passed": true,
  "result": {
    "kind": "synthetic thresholds",
    "gap_m": 2.0,
    "TTC_s": 6.666666666666667,
    "following_candidate": true,
    "crossing_velocity_mps": [
      0.0,
      0.4
    ],
    "lesson": "A transverse mover must not reuse this following decision."
  }
}
```

本章例子可以在 Windows 使用 `py -3` 或在 Linux 使用 `python3` 重跑。它验证局部概念或读取历史证据，不对机器人全任务提供新通过结论。
<!-- generated-lab-end -->

## 常见失败与答案

目标 ID 频繁变化：查聚类、遮挡和关联歧义；停止目标被误当同向：查速度置信度和观测年龄；弯道跟错对象：查投影的是当前 active_path 还是过时路线；跟随触发无进展：查前车实际速度、任务进展定义与有界监督。

问：记录相同尺寸物体后，下次可以不检测吗？答：不能。当前位置、速度与关系仍要重新观察，相同尺寸不保证相同行为。经验最多帮助选择模型或策略，并继续通过安全检查。

练习：为同向、横穿、停止、遮挡四种情况分别写出必需输入和允许行为，特别注明哪些字段不能仅靠一帧得到。
