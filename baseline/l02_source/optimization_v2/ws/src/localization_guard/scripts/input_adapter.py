"""Observed IMU tilt, stationary bias calibration, wheel covariance and epoch handling."""
import argparse,copy,signal,time
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from support import *
class Inputs:
    def __init__(self,out):
        self.c=policy();self.n=rclpy.create_node('s05_input_adapter');self.out=out;out.mkdir(parents=True,exist_ok=True);self.f=(out/'inputs.jsonl').open('w');self.t=0.;self.epoch=0;self.reset_state()
        self.wpub=self.n.create_publisher(TwistWithCovarianceStamped,'/s05/inputs/wheel',20);self.bpub=self.n.create_publisher(TwistWithCovarianceStamped,'/s05/inputs/wheel_baseline',20);self.ipub=self.n.create_publisher(Imu,'/s05/inputs/imu',40);self.gpub=self.n.create_publisher(Imu,'/s05/imu/gravity_removed',10);self.opub=self.n.create_publisher(Odometry,'/s05/wheel_only',20)
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.epub=self.n.create_publisher(String,'/s05/epoch',q)
        self.clients=[self.n.create_client(Empty,x) for x in ['/s05/fused/reset','/s05/wheel_imu/reset','/s05/icp/s05_icp/reset_odom']]
        self.n.create_service(Empty,'/s05/reset_epoch',self.request_reset);self.n.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data);self.n.create_subscription(Imu,'/s04/localization/imu',self.imu,qos_profile_sensor_data);self.n.create_subscription(TwistWithCovarianceStamped,'/s04/localization/wheel',self.wheel,qos_profile_sensor_data);self.n.create_subscription(String,'/s05/icp/quality',self.icp,10)
        (out/'ready').write_text('observation inputs only')
    def reset_state(self):
        self.epoch_start=self.t;self.last_imu=None;self.last_wheel=None;self.last_pub=-1.;self.xyz=np.zeros(3);self.yaw=0.;self.tilt=np.zeros(2);self.bias_sum=np.zeros(3);self.bias_n=0;self.bias=np.zeros(3);self.accel=0.;self.v=0.;self.icp_data={};self.mismatch_since=None;self.recovery_since=None;self.slipping=False;self.wheel_history=[]
    def reset(self,reason):
        self.epoch+=1;self.reset_state();msg=String();msg.data=json.dumps(dict(epoch=self.epoch,t=self.t,reason=reason));self.epub.publish(msg);self.f.write(msg.data+'\n')
        for c in self.clients:
            ready=c.service_is_ready();self.f.write(json.dumps(dict(key='reset_request',epoch=self.epoch,service=c.srv_name,ready=ready))+'\n')
            if ready:
                future=c.call_async(Empty.Request())
                future.add_done_callback(lambda future,name=c.srv_name,ep=self.epoch:self.f.write(json.dumps(dict(key='reset_ack',epoch=ep,service=name,success=future.exception() is None))+'\n'))
    def request_reset(self,request,response):self.reset('EXPLICIT_RESET_STOP_AND_NEW_ORIGIN');return response
    def clock(self,m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<self.t:self.t=t;self.reset('CLOCK_REWIND_NEW_EPOCH')
        self.t=t
    def icp(self,m):self.icp_data=json.loads(m.data)
    def imu(self,m):
        t=stamp(m)
        if t>self.t+.25:return
        dt=0.005 if self.last_imu is None else t-self.last_imu
        if dt<=0 or dt>.5:self.last_imu=t;return
        self.last_imu=t;c=self.c['imu'];a=np.array([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]);w=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
        if 1<t-self.epoch_start<4 and abs(self.v)<.01:self.bias_sum+=w;self.bias_n+=1;self.bias=self.bias_sum/self.bias_n
        gyro=w-self.bias;norm=float(np.linalg.norm(a));valid_tilt=abs(norm-c['gravity_mps2'])<c['gravity_norm_tolerance'] and abs(self.accel)<c['max_wheel_accel_for_tilt']
        rp=np.array([math.atan2(a[1],a[2]),math.atan2(-a[0],math.hypot(a[1],a[2]))])
        if valid_tilt:self.tilt+=(1-math.exp(-dt/.25))*(rp-self.tilt)
        out=copy.deepcopy(m);q=Rotation.from_euler('xyz',[*self.tilt,0.]).as_quat();out.orientation.x,out.orientation.y,out.orientation.z,out.orientation.w=q.tolist();out.orientation_covariance=[c['tilt_variance'] if i in [0,4] else 1e6 if i==8 else 0. for i in range(9)]
        if not valid_tilt:out.orientation_covariance[0]=-1.
        out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z=gyro.tolist();out.angular_velocity_covariance=[c['gyro_variance'] if i in [0,4,8] else 0. for i in range(9)];self.ipub.publish(out)
        gravity=Rotation.from_quat(q).inv().apply([0,0,c['gravity_mps2']]);free=a-gravity;clean=copy.deepcopy(out);clean.linear_acceleration.x,clean.linear_acceleration.y,clean.linear_acceleration.z=free.tolist();self.gpub.publish(clean)
        if round(t*200)%10==0:self.f.write(json.dumps(dict(key='imu',t=t,epoch=self.epoch,tilt=self.tilt.tolist(),gravity_free=free.tolist(),bias=self.bias.tolist(),tilt_valid=valid_tilt,gyro=gyro.tolist()))+'\n')
    def wheel(self,m):
        t=stamp(m)
        if t>self.t+.25:return
        v=m.twist.twist.linear.x;w=m.twist.twist.angular.z
        if self.last_wheel is not None:
            dt=t-self.last_wheel
            if dt<=0 or dt>.5:self.last_wheel=t;return
            self.accel=.85*self.accel+.15*(v-self.v)/dt;h=self.yaw+w*dt/2;self.xyz[:2]+=v*dt*np.array([math.cos(h),math.sin(h)]);self.yaw+=w*dt
        self.last_wheel=t;self.v=v
        self.wheel_history.append((t,v));self.wheel_history=[x for x in self.wheel_history if x[0]>=t-2.]
        o=Odometry();o.header=copy.deepcopy(m.header);o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.position.x,o.pose.pose.position.y,o.pose.pose.position.z=self.xyz.tolist();o.pose.pose.orientation.z=math.sin(self.yaw/2);o.pose.pose.orientation.w=math.cos(self.yaw/2);o.twist=copy.deepcopy(m.twist)
        for i in range(6):o.pose.covariance[7*i]=.01+max(t,0)*.001
        self.opub.publish(o)
        if t-self.last_pub<.019:return
        self.last_pub=t;c=self.c['wheel'];iv=self.icp_data.get('v');fresh=self.icp_data.get('valid',False) and iv is not None and 0<=t-self.icp_data.get('t',-100)<.5;aligned=[value for ts,value in self.wheel_history if self.icp_data.get('t',-100)-.8<=ts<=self.icp_data.get('t',-100)];residual=float(np.mean(aligned))-iv if fresh and aligned else None;fresh=fresh and residual is not None
        if fresh and abs(residual)>c['mismatch_mps']:
            self.recovery_since=None
            if self.mismatch_since is None:self.mismatch_since=t
            if t-self.mismatch_since>=c['mismatch_hold_s']:self.slipping=True
        elif fresh:self.mismatch_since=None;self.recovery_since=None;self.slipping=False;self.wheel_history=[]
        out=copy.deepcopy(m);out.twist.twist.linear.y=out.twist.twist.linear.z=0.;out.twist.covariance=[0.]*36
        for i,value in enumerate([c['velocity_variance'],c['rolling_constraint_variance'],c['rolling_constraint_variance'],1e6,1e6,1e6]):out.twist.covariance[7*i]=value
        self.bpub.publish(out)
        if self.slipping:out.twist.covariance[0]=c['degraded_variance']
        self.wpub.publish(out);self.f.write(json.dumps(dict(key='wheel',t=t,epoch=self.epoch,v=v,w=w,icp_v=iv if fresh else None,innovation_v=residual,innovation_kind='independent_wheel_minus_lidar_velocity',nis=None if residual is None else residual**2/(c['velocity_variance']+.02),slip_suspected=self.slipping,variance=out.twist.covariance[0]))+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();rclpy.init();obj=Inputs(a.out);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop[0]:rclpy.spin_once(obj.n,timeout_sec=.01)
    finally:obj.f.close();dump(a.out/'result.json',dict(epoch=obj.epoch,stationary_bias_samples=obj.bias_n,gyro_bias_estimate=obj.bias.tolist()));obj.n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
