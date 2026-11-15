"""Finite/self/range/voxel filter and observed surface-normal observability."""
import argparse,signal,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_msgs.msg import Header
from scipy.spatial import cKDTree
from support import *
def geometry(xyz):
    if len(xyz)<30:return [0.,0.,0.]
    sample=xyz[::max(1,len(xyz)//1200)];tree=cKDTree(xyz);_,ids=tree.query(sample,k=min(12,len(xyz)));neighbors=xyz[ids];d=neighbors-neighbors.mean(axis=1,keepdims=True);cov=np.einsum('nki,nkj->nij',d,d)/d.shape[1];_,vectors=np.linalg.eigh(cov);normals=vectors[:,:,0]
    return np.linalg.eigvalsh(normals.T@normals/len(normals)).tolist()
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);log=(a.out/'geometry.jsonl').open('w');c=policy()['cloud'];rclpy.init();n=rclpy.create_node('s05_cloud_filter');pub=n.create_publisher(PointCloud2,'/s05/icp/cloud',qos_profile_sensor_data);quality=n.create_publisher(String,'/s05/cloud_quality',10)
    last=[-100.]
    def cloud(m):
        t=stamp(m)
        if t<last[0]:last[0]=-100.
        if t<c['settle_s'] or t-last[0]<1/c['frontend_hz']-1e-6:return
        last[0]=t
        start=time.monotonic();xyz=point_cloud2.read_points_numpy(m,field_names=['x','y','z'],skip_nans=False).reshape(-1,3);dist=np.linalg.norm(xyz,axis=1);xyz=xyz[np.isfinite(xyz).all(axis=1)&(dist>c['range_min_m'])&(dist<c['range_max_m'])];xyz=xyz+np.array([.1,0,.78],np.float32)
        self_mask=(xyz[:,0]>-.65)&(xyz[:,0]<1.5)&(np.abs(xyz[:,1])<.65)&(xyz[:,2]<1.0);xyz=xyz[~self_mask]
        if len(xyz):_,ids=np.unique(np.floor(xyz/c['voxel_m']).astype(np.int32),axis=0,return_index=True);xyz=xyz[np.sort(ids)]
        # Ground supports z/roll/pitch, but must not dominate walls/corners in ICP.
        ground=np.abs(xyz[:,2])<c['ground_band_m'];keep=~ground;keep[np.flatnonzero(ground)[::c['ground_stride']]]=True;xyz=xyz[keep]
        if len(xyz)>c['max_points']:xyz=xyz[np.linspace(0,len(xyz)-1,c['max_points'],dtype=int)]
        eig=geometry(xyz);good=len(xyz)>=c['min_points'] and eig[0]>=c['normal_eigen_min'];d=dict(t=stamp(m),points=len(xyz),normal_eigenvalues=eig,valid=good,processing_s=time.monotonic()-start);msg=String();msg.data=json.dumps(d);quality.publish(msg);log.write(json.dumps(d)+'\n')
        h=Header();h.stamp=m.header.stamp;h.frame_id='base_link';pub.publish(point_cloud2.create_cloud_xyz32(h,xyz.astype(np.float32)))
    n.create_subscription(PointCloud2,'/s04/localization/points',cloud,qos_profile_sensor_data);stop=[False]
    def end(s,f):stop[0]=True
    signal.signal(signal.SIGINT,end);signal.signal(signal.SIGTERM,end);(a.out/'ready').write_text('observed clouds only')
    try:
        while not stop[0]:rclpy.spin_once(n,timeout_sec=.02)
    finally:log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
