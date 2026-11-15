// Preserve the requested service coordinate after the grid search. Apache-2.0.
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
namespace r07 {
class ExactGoalPlanner : public nav2_smac_planner::SmacPlanner2D {
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& p,std::string n,std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> m) override {
  map_=m;SmacPlanner2D::configure(p,n,tf,m);
 }
 nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& s,const geometry_msgs::msg::PoseStamped& g,std::function<bool()> cancel) override {
  auto path=SmacPlanner2D::createPlan(s,g,cancel);
  if(path.poses.empty())throw nav2_core::NoValidPathCouldBeFound("Empty grid path");
  auto* cm=map_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cm->getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> check(cm);
  auto clear=[&](const auto& from,const auto& to){
   double dx=to.pose.position.x-from.pose.position.x,dy=to.pose.position.y-from.pose.position.y;
   double a=tf2::getYaw(from.pose.orientation),b=tf2::getYaw(to.pose.orientation),dh=std::atan2(std::sin(b-a),std::cos(b-a));
   int count=std::max(1,int(std::ceil(std::max(std::hypot(dx,dy)/.02,std::abs(dh)/.03))));
   for(int i=0;i<=count;++i){
    if(cancel())throw nav2_core::PlannerCancelled("Terminal check cancelled");
    double u=double(i)/count,cost=check.footprintCostAtPose(from.pose.position.x+u*dx,from.pose.position.y+u*dy,a+u*dh,map_->getRobotFootprint());
    if(cost<0||cost>=253)return false;
   }
   return true;
  };
  auto end=g;end.header=path.header;auto begin=s;begin.header=path.header;
  if(std::hypot(g.pose.position.x-s.pose.position.x,g.pose.position.y-s.pose.position.y)<4.0 && clear(begin,end)){
   path.poses.clear();double dx=end.pose.position.x-begin.pose.position.x,dy=end.pose.position.y-begin.pose.position.y;
   int samples=std::max(1,int(std::ceil(std::hypot(dx,dy)/.05)));
   const double heading=std::atan2(dy,dx);
   for(int i=0;i<=samples;++i){auto p=begin;double u=double(i)/samples;p.pose.position.x+=u*dx;p.pose.position.y+=u*dy;p.pose.orientation.x=p.pose.orientation.y=0;p.pose.orientation.z=std::sin(heading/2);p.pose.orientation.w=std::cos(heading/2);path.poses.push_back(p);}
   path.poses.back()=end;return path;
  }
  if(path.poses.size()<2)throw nav2_core::NoValidPathCouldBeFound("Unverified same-cell terminal");
  if(!clear(path.poses[path.poses.size()-2],end))throw nav2_core::GoalOccupied("Exact service terminal footprint blocked");
  path.poses.back()=end;
  if(clear(begin,path.poses[1]))path.poses.front()=begin;
  return path;
 }
};
}
PLUGINLIB_EXPORT_CLASS(r07::ExactGoalPlanner,nav2_core::GlobalPlanner)
