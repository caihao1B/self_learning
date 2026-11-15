// Observed geometry only. Raw world geometry and actor scripts are not inputs.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <terrain_perception/risk.hpp>
#include <fstream>
#include <deque>
#include <array>
#include <map>
using J=nlohmann::json;using V=Eigen::Vector3d;using T=Eigen::Isometry3d;
T trans(const nav_msgs::msg::Odometry&m){auto&p=m.pose.pose;T t=T::Identity();t.translation()<<p.position.x,p.position.y,p.position.z;t.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).normalized().toRotationMatrix();return t;}
struct Cell {double t=-100,h=0,top=0,slope=0,rough=0,step=0;int points=0;std::array<int,2>cost{{-1,-1}},clear{{0,0}};};
class Terrain:public rclcpp::Node{
 J cfg_;double res_,half_,ox_,oy_;int n_;std::vector<Cell>cells_;std::array<s09::Profile,2>profiles_;std::deque<nav_msgs::msg::Odometry> odom_;double last_=-1,last_details_=-1;bool epoch_bad_=false;
 rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr objects_,details_,health_;std::array<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr,2>grids_;
 double val(const char*k){return cfg_.at(k).get<double>();}
 int index(double x,double y){int a=int(std::floor((x-ox_)/res_)),b=int(std::floor((y-oy_)/res_));return a>=0&&b>=0&&a<n_&&b<n_?b*n_+a:-1;}
 V center(int i,double z=0){return V((i%n_+.5)*res_+ox_,(i/n_+.5)*res_+oy_,z);}
 void string(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p,const J&d){std_msgs::msg::String m;m.data=d.dump();p->publish(m);}
 void fail(double t,const std::string&reason){string(health_,{{"t",t},{"valid",false},{"reason",reason},{"max_speed",0.}});}
 void cloud(const sensor_msgs::msg::PointCloud2&m){
  auto begin=std::chrono::steady_clock::now();double t=rclcpp::Time(m.header.stamp).seconds();
  if(t<=last_){if(t<last_)epoch_bad_=true;fail(t,"NONMONOTONIC_SENSOR_TIME");return;}last_=t;
  if(epoch_bad_||m.header.frame_id!="lidar_link"||now().seconds()-t>val("max_cloud_age_s")||t>now().seconds()+.05||odom_.empty()){fail(t,"TIME_FRAME_OR_ODOMETRY_UNAVAILABLE");return;}
  auto it=std::min_element(odom_.begin(),odom_.end(),[t](auto&a,auto&b){return std::abs(rclcpp::Time(a.header.stamp).seconds()-t)<std::abs(rclcpp::Time(b.header.stamp).seconds()-t);});double dt=std::abs(rclcpp::Time(it->header.stamp).seconds()-t);
  if(dt>val("max_odom_delta_s")){fail(t,"ODOMETRY_EPOCH_MISMATCH");return;}T o=trans(*it);if(!o.matrix().allFinite()){fail(t,"INVALID_ODOMETRY");return;}
  int sx=std::lround((o.translation().x()-(ox_+half_))/res_),sy=std::lround((o.translation().y()-(oy_+half_))/res_);
  if(std::abs(sx)>10||std::abs(sy)>10){std::vector<Cell> shifted(cells_.size());for(int y=0;y<n_;y++)for(int x=0;x<n_;x++){int px=x+sx,py=y+sy;if(px>=0&&py>=0&&px<n_&&py<n_)shifted[y*n_+x]=cells_[py*n_+px];}cells_.swap(shifted);ox_+=sx*res_;oy_+=sy*res_;}
  pcl::PointCloud<pcl::PointXYZ> input;pcl::fromROSMsg(m,input);auto clean=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for(auto&p:input){V v(p.x,p.y,p.z);if(!v.allFinite()||v.norm()<val("min_range_m")||v.norm()>val("range_m"))continue;v+=V(.1,0,.78);if(v.x()>-.65&&v.x()<1.55&&std::abs(v.y())<.7&&v.z()<1.1)continue;v=o*v;if(index(v.x(),v.y())<0)continue;clean->push_back(pcl::PointXYZ(v.x(),v.y(),v.z()));}
  pcl::VoxelGrid<pcl::PointXYZ> vox;vox.setInputCloud(clean);vox.setLeafSize(val("voxel_m"),val("voxel_m"),val("voxel_m"));auto filtered=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();vox.filter(*filtered);
  auto low=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();for(auto&p:*filtered)if(p.z<o.translation().z()+.35&&p.z>o.translation().z()-.6)low->push_back(p);
  pcl::SACSegmentation<pcl::PointXYZ> seg;seg.setOptimizeCoefficients(true);seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);seg.setAxis(Eigen::Vector3f(0,0,1));seg.setEpsAngle(val("ground_max_angle_deg")*M_PI/180);seg.setMethodType(pcl::SAC_RANSAC);seg.setMaxIterations(80);seg.setDistanceThreshold(val("ground_distance_m"));seg.setInputCloud(low);pcl::PointIndices inliers;pcl::ModelCoefficients plane;seg.segment(inliers,plane);
  if(inliers.indices.size()<size_t(cfg_["min_ground_points"].get<int>())||plane.values.size()!=4||std::abs(plane.values[2])<.7){fail(t,"GROUND_REFERENCE_UNVERIFIED");return;}
  auto floor=[&](double x,double y){return -(plane.values[0]*x+plane.values[1]*y+plane.values[3])/plane.values[2];};
  std::map<int,std::vector<double>> bins;for(auto&p:*filtered){int i=index(p.x,p.y);if(i>=0)bins[i].push_back(p.z);}
  std::map<int,double> h;for(auto&[i,z]:bins){std::sort(z.begin(),z.end());h[i]=z[size_t(.25*(z.size()-1))];}
  J rows=J::array();int lethal=0;double closest=100.;
  for(auto&[i,z]:bins){auto pos=center(i);double ground=floor(pos.x(),pos.y()),height=z[size_t(.8*(z.size()-1))]-ground;std::vector<V> near;
   for(int dy=-3;dy<=3;dy++)for(int dx=-3;dx<=3;dx++){int x=i%n_+dx,y=i/n_+dy;if(x<0||y<0||x>=n_||y>=n_)continue;int j=y*n_+x;if(h.count(j)&&std::abs(h[j]-h[i])<.25)near.push_back(center(j,h[j]));}
   V mean=V::Zero(),normal=V(0,0,1);double rough=.1,slope=0,step=0;bool planar=false;
   if(near.size()>=8){for(auto&p:near)mean+=p;mean/=near.size();Eigen::Matrix3d C=Eigen::Matrix3d::Zero();for(auto&p:near)C+=(p-mean)*(p-mean).transpose();Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(C/near.size());normal=e.eigenvectors().col(0);if(normal.z()<0)normal=-normal;rough=std::sqrt(std::max(0.,e.eigenvalues()[0]));slope=std::acos(std::clamp(normal.z(),0.,1.))*180/M_PI;planar=rough<.02&&slope<15.;
    if(normal.z()>.5)for(auto&p:near){double predicted=h[i]-(normal.x()*(p.x()-pos.x())+normal.y()*(p.y()-pos.y()))/normal.z();step=std::max(step,std::abs(p.z()-predicted));}
   }else {rough=.01;step=std::max(0.,height);}
   // A positive, nearly horizontal island is a step, not a new ground datum.
   if(height>.045&&slope<1.5)step=std::max(step,height);
   Cell&c=cells_[i];c.t=t;c.h=h[i]-ground;c.top=height;c.slope=slope;c.rough=rough;c.step=step;c.points=z.size();
   for(int k=0;k<2;k++){int cost=s09::risk(slope,step,rough,height,planar,profiles_[k]);if(s09::clear(c.cost[k],cost,c.clear[k],cfg_["clear_frames"].get<int>()))c.cost[k]=cost;}
   if(c.cost[0]==100){lethal++;V body=o.inverse()*center(i,h[i]);if(body.x()>0&&std::abs(body.y())<.7)closest=std::min(closest,body.x());}
   if(t-last_details_>=.95)rows.push_back({pos.x(),pos.y(),c.h,c.top,slope,rough,step,c.points,c.cost[0],c.cost[1]});
  }
  for(int k=0;k<2;k++){nav_msgs::msg::OccupancyGrid grid;grid.header=m.header;grid.header.frame_id="odom";grid.info.resolution=res_;grid.info.width=grid.info.height=n_;grid.info.origin.position.x=ox_;grid.info.origin.position.y=oy_;grid.info.origin.orientation.w=1.;grid.data.resize(cells_.size());for(size_t i=0;i<cells_.size();i++)grid.data[i]=(t-cells_[i].t>val("unknown_after_s"))?-1:cells_[i].cost[k];grids_[k]->publish(grid);}
  auto elevated=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();pcl::PointCloud<pcl::PointXYZRGB> colored;
  for(auto&p:*filtered){int i=index(p.x,p.y);pcl::PointXYZRGB q;q.x=p.x;q.y=p.y;q.z=p.z;double ht=p.z-floor(p.x,p.y);q.r=i>=0&&cells_[i].cost[0]==100?240:30;q.g=i>=0&&cells_[i].cost[0]<100?190:70;q.b=ht>.4?170:60;colored.push_back(q);if(ht>.16)elevated->push_back(p);}
  sensor_msgs::msg::PointCloud2 pc;pcl::toROSMsg(colored,pc);pc.header=m.header;pc.header.frame_id="odom";color_pub_->publish(pc);
  J clusters=J::array();if(elevated->size()>8){auto tree=std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();tree->setInputCloud(elevated);pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;ec.setClusterTolerance(.3);ec.setMinClusterSize(8);ec.setMaxClusterSize(12000);ec.setSearchMethod(tree);ec.setInputCloud(elevated);std::vector<pcl::PointIndices> groups;ec.extract(groups);
   for(auto&g:groups){V lo=V::Constant(1e6),hi=V::Constant(-1e6),sum=V::Zero();for(int i:g.indices){auto&p=(*elevated)[i];V v(p.x,p.y,p.z);lo=lo.cwiseMin(v);hi=hi.cwiseMax(v);sum+=v;}V size=hi-lo,ctr=(hi+lo)/2;if(size.x()>3.||size.y()>3.||size.z()<.25)continue;double base=floor(ctr.x(),ctr.y());clusters.push_back({{"position",{ctr.x(),ctr.y(),ctr.z()}},{"size",{size.x(),size.y(),hi.z()-base}},{"minimum",{lo.x(),lo.y(),base}},{"maximum",{hi.x(),hi.y(),hi.z()}},{"points",g.indices.size()},{"covariance",{.0036,0,0,0,.0036,0,0,0,.01}}});}}
  string(objects_,{{"t",t},{"frame","odom"},{"objects",clusters},{"source","PCL_EUCLIDEAN_OBSERVED_POINTS"}});
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();string(health_,{{"t",t},{"valid",true},{"reason",closest<1.4?"GEOMETRIC_PROTECTIVE_STOP":"OBSERVED_GEOMETRY_AVAILABLE"},{"max_speed",closest<1.4?0.:.12},{"closest_front_m",closest},{"input_points",input.size()},{"filtered_points",filtered->size()},{"ground_inliers",inliers.indices.size()},{"compute_ms",ms},{"odom_delta_s",dt},{"instantaneous_scan",true}});
  if(t-last_details_>=.95){last_details_=t;string(details_,{{"t",t},{"frame","odom"},{"columns",{"x","y","ground_relative_m","top_relative_m","slope_deg","roughness_m","step_m","points","diff_cost","ackermann_cost"}},{"cells",rows},{"ground_plane",plane.values},{"lethal_cells",lethal},{"compute_ms",ms}});}
 }
public:
 Terrain():Node("s09_terrain"){
  std::ifstream f(declare_parameter<std::string>("config_file",""));f>>cfg_;res_=val("resolution_m");half_=val("width_m")/2;n_=std::lround(2*half_/res_);ox_=oy_=-half_;cells_.resize(n_*n_);int k=0;for(std::string name:{"diff","ackermann"}){auto p=cfg_["profiles"][name];profiles_[k]={p["max_step_m"],p["max_slope_deg"],p["max_roughness_m"],p["clearance_m"],p["tool_width_m"],val("terrain_slope_sigma_deg"),val("terrain_height_sigma_m"),val("obstacle_min_height_m")};grids_[k++]=create_publisher<nav_msgs::msg::OccupancyGrid>("/s09/terrain/"+name,rclcpp::QoS(2).reliable());}
  color_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/s09/terrain/colored",2);objects_=create_publisher<std_msgs::msg::String>("/s09/geometry",10);details_=create_publisher<std_msgs::msg::String>("/s09/terrain/cells",5);health_=create_publisher<std_msgs::msg::String>("/s09/terrain/health",10);
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",100,[this](nav_msgs::msg::Odometry::SharedPtr m){odom_.push_back(*m);while(odom_.size()>250)odom_.pop_front();});
  cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/s04/perception/points",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::SharedPtr m){try{cloud(*m);}catch(const std::exception&e){fail(now().seconds(),std::string("PROCESSING_EXCEPTION: ")+e.what());}});
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Terrain>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}rclcpp::shutdown();}
