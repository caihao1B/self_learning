"""Trusted sensor generator: raw domain 121 -> explicit noisy observation domain 119.
Only this process has both contexts. It never publishes raw inputs or world poses.
"""
import argparse,copy,heapq,json,math,signal,time
from collections import Counter
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField,Image,Imu,NavSatFix,JointState
from geometry_msgs.msg import TwistWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
from common import *
import sys
sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'))
from kinematics import twist
TYPES={'points':PointCloud2,'left':Image,'right':Image,'imu':Imu,'wheel':TwistWithCovarianceStamped,'joints':JointState,'gnss':NavSatFix,'quality':String}
TOPICS={'points':'lidar/points','left':'stereo/left/image_rect','right':'stereo/right/image_rect','imu':'imu/data','wheel':'wheel/twist','joints':'wheel/joint_states','gnss':'gnss/fix','quality':'gnss/quality'}
class Adapter:
    def __init__(self,vehicle,out,seed,profile,streams='motion'):
        self.c=settings();self.vehicle=read(vehicle);self.out=out;out.mkdir(parents=True,exist_ok=True);self.log=(out/'generation.jsonl').open('w');self.t=0.;self.queue=[];self.bytes=0;self.highwater=0;self.seq=0;self.epoch=0;self.counts=Counter();self.drops=Counter();self.last={};self.profile=profile
        self.rng={k:np.random.default_rng(np.random.SeedSequence([seed,i])) for i,k in enumerate(['points','left','right','imu','wheel','gnss'])}
        self.gyro_bias=np.array(self.c['imu']['gyro_bias']);self.accel_bias=np.array(self.c['imu']['accel_bias']);self.gnss_bias=np.zeros(3);self.last_published={}
        self.raw_context=Context();self.online_context=Context();rclpy.init(context=self.raw_context,domain_id=121);rclpy.init(context=self.online_context,domain_id=119)
        self.raw=rclpy.create_node('s04_generator_raw_'+streams,context=self.raw_context);self.online=rclpy.create_node('s04_generator_output_'+streams,context=self.online_context)
        active={'points','left','right'} if streams=='vision' else {'imu','wheel','joints','gnss','quality'}
        self.pub={}
        for k,v in TYPES.items():
            if k not in active:continue
            qos=QoSProfile(depth=3 if k in ['left','right'] else 10,reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub[k]=self.online.create_publisher(v,'/s04/'+TOPICS[k],qos)
        self.raw.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data)
        for key,typ,topic,callback in [('points',PointCloud2,'lidar/points',self.cloud),('left',Image,'left',lambda m:self.camera(m,'left')),('right',Image,'right',lambda m:self.camera(m,'right')),('imu',Imu,'imu',self.imu),('gnss',NavSatFix,'gnss',self.gnss),('wheel',String,'joints',self.wheel)]:
            if key in active:
                qos=QoSProfile(depth=3 if key in ['left','right'] else 20,reliability=ReliabilityPolicy.BEST_EFFORT)
                self.raw.create_subscription(typ,'/s04/raw/'+topic,callback,qos)
        self.executor=SingleThreadedExecutor(context=self.raw_context);self.executor.add_node(self.raw)
        (out/'ready').write_text('raw 121 -> observations 119')
    def clock(self,m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<self.t:
            self.queue.clear();self.bytes=0;self.last.clear();self.last_published.clear();self.epoch+=1
        self.t=t;self.flush()
    def audit(self,k,t,**data):self.log.write(json.dumps(dict(sensor=k,t=t,epoch=self.epoch,**data),separators=(',',':'))+'\n')
    def enqueue(self,k,m,t,size=500):
        f=phase(t,self.profile);delay=float(f.get('delay_s',0));self.seq+=1
        if self.bytes+size>self.c['limits']['queue_bytes']:self.drops['queue_'+k]+=1;return
        heapq.heappush(self.queue,(t+delay,self.seq,k,m,size,t));self.bytes+=size;self.highwater=max(self.bytes,self.highwater);self.flush()
    def flush(self):
        while self.queue and self.queue[0][0]<=self.t+1e-9:
            due,seq,key,m,size,t=heapq.heappop(self.queue);self.bytes-=size
            # A transport recovery can overtake delayed samples. Never time-travel online.
            if t<=self.last_published.get(key,-1):
                self.drops['late_'+key]+=1;self.audit(key,t,event='late_discard',delivery_sim=self.t);continue
            self.last_published[key]=t;self.pub[key].publish(m);self.counts[key]+=1
            self.audit(key,t,event='published',delivery_sim=self.t,delay_sim=self.t-t)
    def cloud(self,m):
        t=stamp(m);rng=self.rng['points'];xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=False).reshape(-1,3).astype(np.float32);n=len(xyz);distance=np.linalg.norm(xyz,axis=1);valid=np.isfinite(xyz).all(axis=1)&(distance>0)
        before=xyz[valid][:100].copy();noise=rng.normal(0,self.c['lidar']['sigma_m'],valid.sum());xyz[valid]*=(1+noise/distance[valid])[:,None]
        data=np.zeros(n,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('ring','<u2')])
        for i,k in enumerate(['x','y','z']):data[k]=xyz[:,i]
        # Beam IDs from the native organized raster; never invent point acquisition times.
        data['ring']=np.arange(n)//self.c['lidar']['horizontal'];data['intensity']=0.
        out=PointCloud2();out.header=copy.deepcopy(m.header);out.header.frame_id='lidar_link';out.height=self.c['lidar']['vertical'];out.width=self.c['lidar']['horizontal'];out.is_bigendian=False;out.point_step=18;out.row_step=out.width*18;out.is_dense=False
        out.fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z','intensity'])]+[PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1)];out.data=data.tobytes()
        if n!=out.width*out.height:raise ValueError('Native 3D lidar raster does not match frozen configuration')
        self.audit('points',t,event='generated',native_shape=[m.height,m.width],finite=int(valid.sum()),range_noise_std=float(np.std(noise)),sample_changed=bool(len(before) and np.any(before!=xyz[valid][:100])))
        self.enqueue('points',out,t,len(out.data))
    def camera(self,m,key):
        t=stamp(m);rng=self.rng[key];f=phase(t,self.profile)
        if rng.random()<f.get('image_drop_probability',0):self.drops[key]+=1;self.audit(key,t,event='injected_drop');return
        if m.encoding not in ['rgb8','bgr8']:raise ValueError('unsupported image encoding '+m.encoding)
        raw=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':raw=raw[:,:,::-1]
        noisy=np.clip(raw.astype(np.float32)+rng.normal(0,self.c['stereo']['pixel_sigma'],raw.shape),0,255).astype(np.uint8)
        m.data=noisy.tobytes();m.encoding='rgb8';m.step=m.width*3;m.header.frame_id=key+'_optical';self.enqueue(key,m,t,len(m.data))
    def imu(self,m):
        t=stamp(m);c=self.c['imu'];rng=self.rng['imu'];dt=max(0,min(.1,t-self.last.get('imu',t)));self.last['imu']=t;f=phase(t,self.profile)
        self.gyro_bias+=rng.normal(0,c['gyro_walk']*math.sqrt(dt),3);self.accel_bias+=rng.normal(0,c['accel_walk']*math.sqrt(dt),3)
        angular=np.array([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);accel=np.array([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]);raw_z=float(angular[2])
        angular+=self.gyro_bias+rng.normal(0,c['gyro_sigma'],3);angular[2]+=f.get('gyro_z_bias_rad_s',0);accel+=self.accel_bias+rng.normal(0,c['accel_sigma'],3)
        for i,k in enumerate(['x','y','z']):setattr(m.angular_velocity,k,float(angular[i]));setattr(m.linear_acceleration,k,float(accel[i]))
        m.header.frame_id='imu_link';m.orientation.x=m.orientation.y=m.orientation.z=0.;m.orientation.w=1.;m.orientation_covariance=[-1.,0.,0.,0.,0.,0.,0.,0.,0.]
        m.angular_velocity_covariance=[c['gyro_sigma']**2 if i in [0,4,8] else 0. for i in range(9)];m.linear_acceleration_covariance=[c['accel_sigma']**2 if i in [0,4,8] else 0. for i in range(9)]
        self.audit('imu',t,event='generated',raw_z=raw_z,observed_z=float(angular[2]),bias=self.gyro_bias.tolist());self.enqueue('imu',m,t)
    def wheel(self,msg):
        d=json.loads(msg.data);t=d['t'];c=self.c['wheel'];rng=self.rng['wheel'];resolution=2*math.pi/c['ticks_per_rev'];q=np.round(np.array(d['q'])/resolution)*resolution;v,w=twist(d['velocity'],q,self.vehicle);raw_v=v;f=phase(t,self.profile)
        v=v*(1+f.get('wheel_scale_error',0))+rng.normal(0,c['velocity_sigma']);w+=rng.normal(0,c['angular_sigma'])
        out=TwistWithCovarianceStamped();set_stamp(out,t);out.header.frame_id='base_link';out.twist.twist.linear.x=float(v);out.twist.twist.angular.z=float(w)
        for i,value in enumerate([c['velocity_sigma']**2,1e6,1e6,1e6,1e6,c['angular_sigma']**2]):out.twist.covariance[7*i]=value
        j=JointState();j.header=copy.deepcopy(out.header);j.name=['rear_left_joint','rear_right_joint']+(['steer_left_joint','steer_right_joint'] if len(q)==4 else []);j.position=q.tolist();j.velocity=d['velocity']
        self.audit('wheel',t,event='generated',shaft_v=raw_v,observed_v=float(v));self.enqueue('wheel',out,t);self.enqueue('joints',j,t)
    def gnss(self,m):
        t=stamp(m);c=self.c['gnss'];rng=self.rng['gnss'];f=phase(t,self.profile);mode=f.get('gnss_mode','fixed')
        raw=to_enu([m.latitude,m.longitude,m.altitude],c['origin']);dt=max(0,min(1,t-self.last.get('gnss',t)));self.last['gnss']=t
        alpha=math.exp(-dt/c['correlation_tau_s']);self.gnss_bias=alpha*self.gnss_bias+math.sqrt(1-alpha*alpha)*rng.normal(0,c['correlated_sigma_m'],3)
        quality=String();quality.data=json.dumps(dict(stamp=t,reported_mode='no_fix' if mode=='dropout' else 'float' if mode=='float' else 'fixed'));self.enqueue('quality',quality,t)
        if mode=='dropout':self.drops['gnss']+=1;self.audit('gnss',t,event='injected_drop');return
        sigma=np.array(c['float_sigma_m'] if mode=='float' else c['fixed_sigma_m']);error=rng.normal(0,sigma,3)+self.gnss_bias+np.array(f.get('gnss_bias_enu_m',[0,0,0]));llh=from_enu(raw+error,c['origin']);m.latitude,m.longitude,m.altitude=map(float,llh);m.header.frame_id='gnss_link';m.status.status=2 if mode in ['fixed','false_fixed'] else 0;m.status.service=1
        m.position_covariance_type=2;m.position_covariance=[float(sigma[i//4]**2+c['correlated_sigma_m']**2) if i in [0,4,8] else 0. for i in range(9)]
        self.audit('gnss',t,event='generated',raw_enu=raw.tolist(),observed_enu=(raw+error).tolist(),error_enu=error.tolist(),mode_injected=mode,reported_mode='fixed' if mode=='false_fixed' else mode,covariance=list(m.position_covariance));self.enqueue('gnss',m,t)
    def finish(self):
        self.log.close();dump(self.out/'result.json',dict(counts=dict(self.counts),injected_or_queue_drops=dict(self.drops),queue_highwater_bytes=self.highwater,pending_discarded=len(self.queue),domains=[121,119],raw_topics=[t for t,_ in self.raw.get_topic_names_and_types()],outputs=TOPICS,epoch=self.epoch,config_hash=config_hash()))
        self.executor.shutdown();self.raw.destroy_node();self.online.destroy_node();self.raw_context.shutdown();self.online_context.shutdown()
def main():
    p=argparse.ArgumentParser();p.add_argument('--vehicle',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--profile',choices=['nominal','sensor_faults'],required=True);p.add_argument('--streams',choices=['motion','vision'],required=True);a=p.parse_args()
    profile=yaml.safe_load((BASE/'config/fault_profiles.yaml').read_text())[a.profile];node=Adapter(a.vehicle,a.out,a.seed,profile,a.streams);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop[0]:node.executor.spin_once(timeout_sec=.01)
    finally:node.finish()
if __name__=='__main__':main()
