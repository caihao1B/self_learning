// Global reference selector; independent GNSS consistency, not a second local EKF.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <deque>
#include <chrono>
using J=nlohmann::json;using T=Eigen::Isometry3d;using V=Eigen::Vector3d;using Wall=std::chrono::steady_clock;
T pose(const geometry_msgs::msg::Pose&p){T t=T::Identity();t.translation()<<p.position.x,p.position.y,p.position.z;t.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).normalized().toRotationMatrix();return t;}
T decode(const J&p){T t=T::Identity();t.translation()<<p[0].get<double>(),p[1].get<double>(),p[2].get<double>();Eigen::Quaterniond q(p[6].get<double>(),p[3].get<double>(),p[4].get<double>(),p[5].get<double>());if(!t.matrix().allFinite()||!q.coeffs().allFinite()||std::abs(q.norm()-1)>.01)throw std::runtime_error("invalid pose");t.linear()=q.normalized().toRotationMatrix();return t;}
double angle(const T&a,const T&b){return Eigen::AngleAxisd(a.linear().transpose()*b.linear()).angle();}
J values(const T&t){Eigen::Quaterniond q(t.linear());return {t.translation().x(),t.translation().y(),t.translation().z(),q.x(),q.y(),q.z(),q.w()};}
class Supervisor:public rclcpp::Node{
 J cfg_;std::deque<std::pair<double,T>> odoms_;T correction_=T::Identity(),candidate_=T::Identity(),map_=T::Identity();V gps_=V::Zero();Eigen::Matrix2d gps_cov_=Eigen::Matrix2d::Identity();
 bool have_=false,contract_=false,clock_fault_=false,hold_=false,allow_=false,map_bad_=false;double odom_t_=-100,contract_t_=-100,map_t_=-100,gps_t_=-100,heading_t_=-100,last_map_=-100,last_gps_=-100,first_=0,last_clock_=0,v_=0,w_=0,residual_=0,nis_=0;double map_first_=0,last_recovery_obs_=-100;int map_count_=0;int count_=0,gps_count_=0,jumps_=0;std::string state_="INITIALIZING",reason_="WAITING_FOR_MAP_HEADING",source_="NONE",gps_reason_="NO_FIX",map_reason_="NO_MATCH";
 Wall::time_point odom_wall_=Wall::now(),contract_wall_=Wall::now(),clock_wall_=Wall::now();
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_,gps_sub_,contract_sub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;std::shared_ptr<tf2_ros::TransformBroadcaster>tf_;rclcpp::TimerBase::SharedPtr timer_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ack_;
 double c(const char*k){return cfg_.at(k).get<double>();}
 bool stopped(){return std::abs(v_)<c("stopped_v")&&std::abs(w_)<c("stopped_w");}
 bool lookup(double t,T&o){if(odoms_.empty())return false;auto n=std::min_element(odoms_.begin(),odoms_.end(),[t](auto&a,auto&b){return std::abs(a.first-t)<std::abs(b.first-t);});if(std::abs(n->first-t)>.05)return false;o=n->second;return true;}
 bool fresh(double t,double age){return now().seconds()>=t-.05&&now().seconds()-t<=age;}
 bool jump(const T&t){return have_&&((t.translation()-correction_.translation()).norm()>c("map_jump_translation_m")||angle(t,correction_)>c("map_jump_yaw_rad"));}
 void mapEvidence(const std_msgs::msg::String&m){try{
  auto e=J::parse(m.data);double t=e.at("t");if(t<=last_map_||!fresh(t,c("max_observation_age_s"))||!contract_||e.at("map_id")!=cfg_.at("map_id"))return;last_map_=t;
  if(!e.at("valid").get<bool>()){map_reason_=e.value("reason","MAP_QUALITY");return;}
  T o;if(!lookup(t,o))return;T next=decode(e.at("pose"))*o.inverse();
  if(jump(next)){map_bad_=true;map_t_=-100;map_reason_="MAP_JUMP_REJECTED";if(!hold_){++jumps_;hold_=true;count_=0;}return;}
  // GNSS is not fused into RTAB; disagreement is observable and not hidden in TF.
  if(gps_count_>=3&&fresh(gps_t_,c("gnss_age_s"))&&(next.translation()-gps_).head<2>().norm()>c("innovation_m")){map_bad_=true;map_t_=-100;map_reason_="MAP_GNSS_CONFLICT";return;}
  map_=next;map_t_=t;map_bad_=false;map_reason_="GEOMETRIC_MATCH";heading_t_=t;
  if(!have_){if(map_count_==0||(next.translation()-candidate_.translation()).norm()>c("stable_translation_m")||angle(next,candidate_)>c("stable_yaw_rad")){map_count_=1;map_first_=t;}else ++map_count_;candidate_=next;
   if(map_count_>=cfg_["confirmation_count"].get<int>()&&t-map_first_>=c("confirmation_span_s")&&gps_count_>=3&&fresh(gps_t_,c("gnss_age_s"))&&stopped()){correction_=next;have_=true;count_=0;}}
 }catch(...){map_reason_="MALFORMED_MAP";map_t_=-100;}}
 void gpsEvidence(const std_msgs::msg::String&m){try{
  auto e=J::parse(m.data);double t=e.at("t");if(t<=last_gps_||!fresh(t,c("gnss_age_s"))){gps_reason_="STALE_OR_REORDERED";return;}last_gps_=t;
  if(!e.at("valid").get<bool>()||e.at("mode")!="fixed"){gps_reason_=e.value("mode","INVALID");gps_t_=-100;gps_count_=0;return;}
  auto cv=e.at("covariance").get<std::vector<double>>();Eigen::Matrix3d C;for(int i=0;i<9;i++)C(i/3,i%3)=cv.at(i);Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(C);
  if(!C.allFinite()||!C.isApprox(C.transpose(),1e-9)||eig.eigenvalues().minCoeff()<=0||C.topLeftCorner<2,2>().diagonal().maxCoeff()>c("gnss_variance_max_m2")){gps_reason_="COVARIANCE_REJECTED";gps_count_=0;gps_t_=-100;return;}
  T o;if(!lookup(t,o))return;T reference=have_?correction_:candidate_;if(!have_&&map_count_==0){gps_reason_="NO_VERIFIED_HEADING";return;}
  V antenna(e["antenna"][0],e["antenna"][1],e["antenna"][2]);V lever(e["antenna_base_m"][0],e["antenna_base_m"][1],e["antenna_base_m"][2]);T predicted=reference*o;V base=antenna-predicted.linear()*lever;Eigen::Vector2d innovation=(base-predicted.translation()).head<2>();residual_=innovation.norm();Eigen::Matrix2d S=C.topLeftCorner<2,2>()+Eigen::Matrix2d::Identity()*c("reference_variance_m2");nis_=innovation.dot(S.ldlt().solve(innovation));
  if(!base.allFinite()||residual_>c("innovation_m")||nis_>c("nis_limit")){gps_reason_="INNOVATION_REJECTED_FIXED";gps_count_=0;gps_t_=-100;return;}
  gps_=base-reference.linear()*o.translation();gps_cov_=C.topLeftCorner<2,2>();gps_t_=t;++gps_count_;gps_reason_="FIXED_AND_INNOVATION_VALID";
 }catch(...){gps_reason_="MALFORMED_GNSS";gps_t_=-100;gps_count_=0;}}
 void tick(){double t=now().seconds();auto wall=Wall::now();if(t<last_clock_)clock_fault_=true;if(t>last_clock_)clock_wall_=wall;last_clock_=t;
  bool local=fresh(odom_t_,.25)&&std::chrono::duration<double>(wall-odom_wall_).count()<.5&&std::chrono::duration<double>(wall-clock_wall_).count()<.6;
  bool contract=contract_&&fresh(contract_t_,.6)&&std::chrono::duration<double>(wall-contract_wall_).count()<.8;
  bool map=fresh(map_t_,c("max_observation_age_s")),gps=fresh(gps_t_,c("gnss_age_s"))&&gps_count_>=3,heading=fresh(heading_t_,c("heading_hold_s"));
  std::string desired="STOP";source_="NONE";
  if(clock_fault_)reason_="CLOCK_REWIND_NEW_SESSION_REQUIRED";
  else if(!contract)reason_="MAP_CONTRACT_INVALID_OR_STALE";
  else if(!local)reason_="LOCAL_ODOM_OR_CLOCK_STALE";
  else if(!have_)reason_="WAITING_FOR_CONFIRMED_MAP_HEADING";
  else if(map){source_="MAP";desired=gps?"TRUSTED_MAP":"MAP_ONLY";reason_=gps?"MAP_AND_GNSS_AGREE":"GNSS_"+gps_reason_;}
  else if(gps&&heading){source_="GNSS_POSITION_LOCAL_HEADING";desired="GNSS_ONLY";reason_=map_bad_?map_reason_:"MAP_STALE_BOUNDED_HEADING";}
  else reason_=!heading?"ABSOLUTE_HEADING_EXPIRED":"BOTH_GLOBAL_SOURCES_UNAVAILABLE";
  // Recovery requires consecutive time, and physical standstill. Source downgrade remains bounded.
  bool ready=desired!="STOP";
  if(!ready){allow_=false;count_=0;state_=have_?"STOPPED":"INITIALIZING";}
  else if(!allow_){double obs=std::max(map_t_,gps_t_);if(obs>last_recovery_obs_){if(count_==0)first_=t;++count_;last_recovery_obs_=obs;}state_="CONFIRMING";if(t-first_>=c("confirmation_span_s")&&count_>=cfg_["confirmation_count"].get<int>()&&stopped()){allow_=true;state_=desired;}}
  else state_=desired;
  if(allow_&&ready){T next=correction_;if(map)next=map_;else {next.translation().head<2>()=.7*correction_.translation().head<2>()+.3*gps_.head<2>();}
   if(jump(next)){allow_=false;hold_=true;state_="STOPPED";reason_="GLOBAL_JUMP_HOLD";++jumps_;}else correction_=next;}
  if(hold_){allow_=false;state_="STOPPED";reason_="PLAN_REVALIDATION_REQUIRED";}
  if(have_&&!odoms_.empty()&&local){Eigen::Quaterniond q(correction_.linear());geometry_msgs::msg::TransformStamped tr;tr.header.stamp=rclcpp::Time(int64_t(odom_t_*1e9));tr.header.frame_id="map";tr.child_frame_id="odom";tr.transform.translation.x=correction_.translation().x();tr.transform.translation.y=correction_.translation().y();tr.transform.translation.z=correction_.translation().z();tr.transform.rotation.x=q.x();tr.transform.rotation.y=q.y();tr.transform.rotation.z=q.z();tr.transform.rotation.w=q.w();tf_->sendTransform(tr);
   T g=correction_*odoms_.back().second;Eigen::Quaterniond gq(g.linear());geometry_msgs::msg::PoseWithCovarianceStamped p;p.header=tr.header;p.pose.pose.position.x=g.translation().x();p.pose.pose.position.y=g.translation().y();p.pose.pose.position.z=g.translation().z();p.pose.pose.orientation.x=gq.x();p.pose.pose.orientation.y=gq.y();p.pose.pose.orientation.z=gq.z();p.pose.pose.orientation.w=gq.w();for(int k:{0,7,14,21,28,35})p.pose.covariance[k]=allow_?(k<21?.01:.001+std::max(0.,t-heading_t_)*.0001):9999.;pose_pub_->publish(p);}
  J s={{"t",t},{"state",state_},{"reason",reason_},{"source",source_},{"navigation_allowed",allow_},{"max_speed",allow_?(state_=="TRUSTED_MAP"?c("nominal_speed"):c("degraded_speed")):0.},{"gnss_reason",gps_reason_},{"map_reason",map_reason_},{"innovation_m",residual_},{"nis",nis_},{"innovation_limit_m",c("innovation_m")},{"nis_limit",c("nis_limit")},{"map_age",t-map_t_},{"gnss_age",t-gps_t_},{"heading_age",t-heading_t_},{"confirmation_count",count_},{"gps_count",gps_count_},{"jump_count",jumps_},{"plan_revalidation_required",hold_},{"has_transform",have_},{"correction",values(correction_)},{"v",v_},{"w",w_}};std_msgs::msg::String msg;msg.data=s.dump();status_->publish(msg);
 }
public:
 Supervisor():Node("s08_global_pose_manager"){
  std::ifstream f(declare_parameter<std::string>("policy_file",""));f>>cfg_;status_=create_publisher<std_msgs::msg::String>("/s08/status",20);pose_pub_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/s08/pose",20);tf_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/s05/odometry",100,[this](nav_msgs::msg::Odometry::SharedPtr m){odom_t_=rclcpp::Time(m->header.stamp).seconds();odom_wall_=Wall::now();v_=m->twist.twist.linear.x;w_=m->twist.twist.angular.z;odoms_.emplace_back(odom_t_,pose(m->pose.pose));while(odoms_.size()>300)odoms_.pop_front();});
  contract_sub_=create_subscription<std_msgs::msg::String>("/s08/contract",20,[this](std_msgs::msg::String::SharedPtr m){try{auto j=J::parse(m->data);contract_=j.at("valid").get<bool>()&&j.at("map_id")==cfg_.at("map_id");contract_t_=j.at("t");contract_wall_=Wall::now();}catch(...){contract_=false;}});
  map_sub_=create_subscription<std_msgs::msg::String>("/s08/map",20,[this](std_msgs::msg::String::SharedPtr m){mapEvidence(*m);});gps_sub_=create_subscription<std_msgs::msg::String>("/s08/gnss",20,[this](std_msgs::msg::String::SharedPtr m){gpsEvidence(*m);});ack_=create_service<std_srvs::srv::Trigger>("/s08/acknowledge_diagnostic_hold",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){r->success=cfg_.value("diagnostic_fixture",false)&&stopped()&&count_>=3&&now().seconds()-first_>=c("confirmation_span_s")&&fresh(map_t_,c("max_observation_age_s"))&&fresh(gps_t_,c("gnss_age_s"));if(r->success)hold_=false;r->message=r->success?"Diagnostic fixture only; no active work plan":"Not revalidated and stopped";});timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Supervisor>());}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}rclcpp::shutdown();}
