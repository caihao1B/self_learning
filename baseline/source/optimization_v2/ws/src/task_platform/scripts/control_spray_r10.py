"""Only task actuator request owner. Independently gates Nav2 against observed state."""
import argparse,json,time,signal,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--config',type=Path,required=True);ap.add_argument('--session',required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=json.loads(a.config.read_text());v=c['vehicle'];rclpy.init();n=rclpy.create_node('r07_actuator_gate',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);st=dict(t=0.,stop=False,enc=None,odom=None,loc=None,safety=None,tool=None,worker=None,bt=None,cmd=None,cmd_wall=0.,clock_wall=time.monotonic());seq=0;contextseq=0;progress=0.;lastpose=None
    cmdpub=n.create_publisher(String,'/s03/actuator/request',10);btpub=n.create_publisher(String,'/r06/context',10);statepub=n.create_publisher(String,'/r07/gate',10);log=(a.out/'gate.jsonl').open('w')
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t!=st['t']:st['clock_wall']=time.monotonic()
        if t<st['t']:st['stop']=True
        st['t']=t
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
    for topic,key in [('/s03/encoders','enc'),('/r07/localization','loc'),('/r07/safety','safety'),('/s10/action/state','tool'),('/r07/mission','worker'),('/r06/directive','bt')]:
        n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20 if key=='enc' else 10)
    n.create_subscription(Odometry,'/s05/odometry',lambda m:st.update(odom=dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,v=m.twist.twist.linear.x,w=m.twist.twist.angular.z)),30)
    n.create_subscription(Twist,'/r07/nav_cmd_vel',lambda m:st.update(cmd=m,cmd_wall=time.monotonic()),10)
    def fresh(x,age):return bool(x and -.03<=st['t']-x.get('t',-100)<age)
    def tick():
        nonlocal seq,contextseq,progress,lastpose
        t=st['t'];loc=st['loc'] or {};s=st['safety'] or {};tool=st['tool'] or {};worker=st['worker'] or {};bt=st['bt'] or {};enc=st['enc'];now=time.monotonic()
        localized=fresh(loc,.15) and loc.get('valid',False);healthy=fresh(s,.45) and s.get('valid',False)
        odom=st['odom'] or {};stopped=fresh(odom,.15) and abs(odom.get('v',100))<v['safety']['stopped_v_mps'] and abs(odom.get('w',100))<v['safety']['stopped_w_rad_s']
        tool_stopped=fresh(tool,.15) and tool.get('stopped_confirmed',False) and not tool.get('active',True)
        workerfresh=fresh(worker,.2);nav_idle=workerfresh and worker.get('nav_idle',False)
        if localized:
            p=loc['odom']
            if lastpose is not None:progress+=min(.1,math.dist(p[:2],lastpose[:2])+.25*abs(math.atan2(math.sin(p[2]-lastpose[2]),math.cos(p[2]-lastpose[2]))))
            lastpose=p
        blocked=healthy and s.get('max_speed',0)<=0 or bool(worker.get('nav_failed'))
        contextseq+=1;context=dict(stamp=t,sequence=contextseq,session=a.session,task_version=c['version'],vehicle_hash=v['config_hash'],rig_revision=c['rig']['revision'],localized=localized,sensors_healthy=healthy,stopped=stopped,tool_stopped=tool_stopped,nav_idle=nav_idle,remaining_s=max(0,c['task_budget_s']-t),next_min_s=3.,blocked=blocked,complete=worker.get('complete',False),commanded_motion=workerfresh and worker.get('phase')=='NAVIGATING',progress_m=progress,replans=worker.get('replans',0),deferrals=worker.get('deferrals',0),alternative=worker.get('alternative',False),detour_valid=False,worker_fresh=workerfresh,loc_age=t-loc.get('t',-100),sensor_age=t-s.get('t',-100),reference_valid=loc.get('valid',False))
        distance=worker.get('precision_translation_m')
        if workerfresh and worker.get('phase')=='NAVIGATING' and isinstance(distance,(int,float)) and math.isfinite(distance) and .008<distance<=.15:
            context.update(motion_scope='VALIDATED_LOCAL_DOCKING',expected_translation_m=distance)
        btpub.publish(String(data=json.dumps(context)))
        reasons=[]
        for ok,reason in [(localized,'LOCALIZATION'),(healthy,'PERCEPTION'),(fresh(bt,.15),'BT_STALE'),(workerfresh,'TASK_STALE'),(now-st['clock_wall']<.2,'CLOCK'),(now-st['cmd_wall']<.2,'NAV_COMMAND_STALE'),(tool_stopped,'TOOL_NOT_STOPPED'),(not blocked,'OBSTRUCTED')]:
            if not ok:reasons.append(reason)
        allowed=not reasons and bt.get('action') in ['EXECUTE','FOLLOW','DETOUR'] and worker.get('phase')=='NAVIGATING'
        vv=ww=0.;mode='STOP'
        if allowed and st['cmd']:
            vv=st['cmd'].linear.x;ww=st['cmd'].angular.z
            if not math.isfinite(vv+ww) or vv<-.001 or abs(vv)>c['speed_cap_mps']+.05 or abs(ww)>c['yaw_cap_rps']+.05:allowed=False;reasons.append('NAV_COMMAND_RANGE')
            else:
                cap=min(c['speed_cap_mps'],loc.get('max_speed',0),s['max_speed'],bt.get('speed_cap_mps',0));vv=min(max(0,vv),cap);ww=max(-c['yaw_cap_rps'],min(c['yaw_cap_rps'],ww));mode='DRIVE' if enc and enc['armed'] else 'ARM'
        if not allowed or mode!='DRIVE':vv=ww=0.
        if enc:
            seq+=1;cmdpub.publish(String(data=json.dumps(dict(session=a.session,revision=v['config_hash'],epoch=enc['epoch'],sequence=seq,stamp=t,mode=mode,v=vv,w=ww))))
        d=dict(raw_nav_v=st['cmd'].linear.x if st['cmd'] else None,raw_nav_w=st['cmd'].angular.z if st['cmd'] else None,t=t,allowed=allowed,mode=mode,v=vv,w=ww,reasons=reasons,stopped=stopped,tool_stopped=tool_stopped,nav_idle=nav_idle,progress_m=progress);statepub.publish(String(data=json.dumps(d)));log.write(json.dumps(d)+'\n');log.flush()
    n.create_timer(.03,tick)
    def stop(s,f):st['stop']=True
    for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
    (a.out/'ready').write_text('sole leased actuator request publisher')
    try:
        while not st['stop']:rclpy.spin_once(n,timeout_sec=.005)
    finally:
        st['worker']=None;tick();log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
