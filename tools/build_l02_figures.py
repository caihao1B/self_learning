"""Small explanatory engineering diagrams; no synthetic simulation images."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
STYLE='''graph [bgcolor="#f7fafb",pad="0.3",nodesep="0.4",ranksep="0.5",fontname="Noto Sans CJK SC"];
node [shape=box,style="rounded,filled",fillcolor="#e6f3ef",color="#478276",fontname="Noto Sans CJK SC",fontsize=15,margin=".2,.15"];
edge [color="#5d8189",fontname="Noto Sans CJK SC",fontsize=12];'''
FIGURES={
'frames':('06_frames_time.md','坐标责任与采集时间示意', '''rankdir=TB;
map[label="map 全局地图"];odom[label="odom 连续局部运动"];base[label="base_link 车体参考"];sensor[label="lidar / camera / IMU / GNSS"];
map->odom[label="全局修正"];odom->base[label="局部里程计"];base->sensor[label="固定外参 / 杆臂"];
capture[label="采集时间 t"];arrival[label="接收时间 t + 延迟"];use[label="按采集时刻配对与变换"];
capture->arrival;arrival->use; sensor->capture[style=dashed];'''),
'fusion':('12_odometry.md','融合输入与质量门控示意', '''rankdir=LR;
wheel[label="轮速 / 编码器"];imu[label="IMU"];cloud[label="点云"];icp[label="RTAB ICP"];gate[label="配准质量 / 时间 / 几何"];ekf[label="EKF\\n预测 + 校正"];odom[label="连续局部 odom"];health[label="健康与失效状态"];
wheel->ekf;imu->ekf;cloud->icp;icp->gate;gate->ekf[label="合格增量"];ekf->odom;gate->health;ekf->health;'''),
'planning':('18_partition.md','从整区到可执行簇的闭环', '''rankdir=TB;
zone[label="连续不规则工作区 + 授权通行区"];network[label="单元图 + 工作量权重"];metis[label="METIS 三簇"];verify[label="独立连通 / 覆盖 / 负载检查"];paths[label="点序或条带 + 合法连接"];time[label="反算工时 + 储备"];execute[label="版本化计划 / 在线执行"];
zone->network->metis->verify->paths->time->execute;time->network[label="超预算时调整",style=dashed];'''),
'bt':('23_bt.md','行为选择的优先级与执行边界', '''rankdir=TB;
e[label="新鲜观测 + 版本 + 任务 + 预算"];safety[label="紧急 / 错版本 / 过期 → STOP"];loc[label="定位不合格 → WAIT"];latch[label="人工锁存 → HUMAN"];obstacle[label="障碍候选资格\\nWAIT / FOLLOW / DETOUR / REPLAN / DEFER"];choice[label="动作 + 原因 + 未选候选"];mission[label="任务状态机与 ROS action"];gate[label="独立底盘 / 工具门控"];
e->safety->loc->latch->obstacle->choice->mission->gate;'''),
'cancel':('24_state_machine.md','取消与换挡握手示意', '''rankdir=TB;
a[label="BT 中断 / 退出请求"];b[label="取消活动导航 + 关闭工具"];c[label="等待 action 结果 / 导航 idle"];d[label="实测车停 + 工具停"];e[label="新鲜证据 / 合法恢复路径"];f[label="选择挡位并重新准入"];g[label="下一个目标"];u[label="超时或停止不确定\\n保留残余 / 人工处理",fillcolor="#fff0d9"];
a->b->c->d->e->f->g;b->u[style=dashed];c->u[style=dashed];d->u[style=dashed];'''),
'ledger':('25_ledger.md','有效反馈到账本的判定链', '''rankdir=TB;
goal[label="工作键 + 原目标 + 版本"];reserve[label="预留任务 / 检查重复与冲突"];feedback[label="当前工具反馈 + 同时刻上下文"];guard[label="身份 / 时间 / 健康 / 位置 / 连续窗口"];credit[label="剂量积分 或 工具面积并集"];tx[label="同一事务写事件与确认量"];residual[label="原义务 − 已确认 = 残余"];replay[label="已完成工作键重放\\n返回原结果，不再次工作"];
goal->reserve->feedback->guard->credit->tx->residual;reserve->replay[label="已成功"];'''),
}
for name,(chapter,caption,body) in FIGURES.items():
    dot=ROOT/'assets'/f'l02_{name}.dot';svg=dot.with_suffix('.svg')
    dot.write_text('digraph G {\n'+STYLE+body+'\n}')
    subprocess.run(['dot','-Tsvg',str(dot),'-o',str(svg)],check=True)
    p=ROOT/'L02/chapters'/chapter;text=p.read_text()
    tag='<!-- l02-figure-'+name+' -->'
    if tag not in text:
        first=text.index('\n## ')
        text=text[:first]+'\n\n'+tag+'\n!['+caption+'](../../assets/'+svg.name+')\n\n'+text[first:]
        p.write_text(text)
print('Created',len(FIGURES),'explanatory figures')
