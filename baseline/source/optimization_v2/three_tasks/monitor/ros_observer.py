"""Optional sensor subscriber. Never publishes TF, goals, commands or tool requests."""
import argparse,json,math,os,signal,time,struct
from collections import deque
from pathlib import Path
from data import Evidence,age,json_value

def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,Image,PointCloud2,NavSatFix
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    ap=argparse.ArgumentParser();ap.add_argument('--app',type=Path,required=True);a=ap.parse_args()
    evidence=Evidence(a.app);out=a.app/'monitor_state';out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('task_readonly_monitor',enable_rosout=False,start_parameter_services=False)
    st=dict(run=None,t=0.,channels={},camera_run=None);counts={};stop=[False];last_write=0.;last_camera=0.;last_scan=0.;last_cloud=0.;cloud_sample=[];cloud_sample_t=None
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def vector(v):return [v.x,v.y,v.z]
    def observe(key,m,data):
        if st['run'] is None:return
        t=stamp(m);history=counts.setdefault(key,deque(maxlen=500));history.append(t)
        while len(history)>2 and history[-1]-history[0]>2:history.popleft()
        hz=(len(history)-1)/(history[-1]-history[0]) if len(history)>1 and history[-1]>history[0] else None
        st['channels'][key]=dict(t=t,received_wall=time.time(),frame=m.header.frame_id,observed_sim_hz=hz,**data)
    def imu(m):observe('imu',m,dict(gyro=vector(m.angular_velocity),acceleration=vector(m.linear_acceleration),orientation_available=m.orientation_covariance[0]>=0))
    def wheel(m):observe('wheel',m,dict(v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,variance=m.twist.covariance[0]))
    def gnss(m):observe('gnss',m,dict(status=m.status.status,service=m.status.service,latitude=m.latitude,longitude=m.longitude,altitude=m.altitude,covariance=list(m.position_covariance)))
    def odom(m):observe('odom',m,dict(v=m.twist.twist.linear.x,w=m.twist.twist.angular.z,position=vector(m.pose.pose.position)))
    def cloud(m):
        nonlocal last_cloud,cloud_sample,cloud_sample_t
        if time.monotonic()-last_cloud>.5:
            last_cloud=time.monotonic();cloud_sample=[];cloud_sample_t=stamp(m);fields={f.name:f for f in m.fields}
            if all(k in fields and fields[k].datatype==7 for k in ['x','y','z']) and m.point_step:
                reader=struct.Struct('>f' if m.is_bigendian else '<f');buf=memoryview(m.data)
                total=m.width*m.height;stride=max(1,math.ceil(total/512))
                for i in range(0,total,stride):
                    offset=(i//m.width)*m.row_step+(i%m.width)*m.point_step
                    xyz=[reader.unpack_from(buf,offset+fields[k].offset)[0] for k in ['x','y','z']]
                    if all(math.isfinite(x) for x in xyz) and .15<math.sqrt(sum(x*x for x in xyz))<55:cloud_sample.append(xyz)
        observe('lidar',m,dict(width=m.width,height=m.height,points=m.width*m.height,point_step=m.point_step,fields=[f.name for f in m.fields],sample_xyz=cloud_sample,sample_t=cloud_sample_t,sample_scope='At most 512 observed points in lidar frame; not a free-space or coverage claim'))
    def camera(m):
        nonlocal last_camera
        observe('camera',m,dict(width=m.width,height=m.height,encoding=m.encoding,source='/s04/perception/left'))
        now=time.monotonic()
        if now-last_camera<1 or st['run'] is None:return
        last_camera=now
        try:
            from PIL import Image as PILImage
            channels={'rgb8':('RGB','RGB'),'bgr8':('RGB','BGR'),'rgba8':('RGBA','RGBA'),'mono8':('L','L')}
            if m.encoding not in channels:return
            mode,raw=channels[m.encoding];pic=PILImage.frombytes(mode,(m.width,m.height),bytes(m.data),'raw',raw,m.step,1).convert('RGB')
            pic.thumbnail((480,270));temp=out/'camera.tmp';pic.save(temp,format='JPEG',quality=80);temp.replace(out/'camera.jpg')
            st['camera_run']=st['run'];st['camera_t']=stamp(m)
        except Exception as ex:st['camera_error']=type(ex).__name__+': '+str(ex)
    def clock(m):
        t=m.clock.sec+m.clock.nanosec*1e-9
        if t<st['t']-.1:st['channels']={};counts.clear();st['camera_run']=None
        st['t']=t
    topics=[(Imu,'/s04/localization/imu',imu),(TwistWithCovarianceStamped,'/s04/localization/wheel',wheel),
            (NavSatFix,'/s04/localization/gnss',gnss),(PointCloud2,'/s04/localization/points',cloud),
            (Image,'/s04/perception/left',camera),(Odometry,'/s05/odometry',odom),(Clock,'/clock',clock)]
    for typ,topic,cb in topics:node.create_subscription(typ,topic,cb,qos_profile_sensor_data)
    def end(*_):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end)
    try:
        while not stop[0]:
            rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
            if now-last_scan>1:
                p,rid=evidence.resolve();active=bool(p and not (p/'runtime.json').exists() and (age(p/'live.json') or 1e9)<8)
                chosen=rid if active else None
                if chosen!=st['run']:
                    st.update(run=chosen,channels={},camera_run=None);counts.clear();cloud_sample=[];cloud_sample_t=None;last_cloud=0.
                last_scan=now
            if now-last_write>.5:
                temp=out/'live.tmp';temp.write_text(json.dumps(json_value(dict(st,wall=time.time(),pid=os.getpid(),readonly=True,domain=119,topics=[t for _,t,_ in topics])),allow_nan=False));temp.replace(out/'live.json');last_write=now
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
