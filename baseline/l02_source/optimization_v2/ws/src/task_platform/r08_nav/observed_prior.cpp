#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <mutex>
namespace r08 {
// This layer creates PRIOR candidates only. The independent local observed
// layer still treats ALL unknown execution cells as occupied.
class ObservedPriorLayer: public nav2_costmap_2d::CostmapLayer {
 std::mutex mutex_;nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;double max_age_=.6;
 public:
 void onInitialize() override {
  auto node=node_.lock();if(!node)throw std::runtime_error("missing node");
  declareParameter("topic",rclcpp::ParameterValue("/r07/risk_local"));declareParameter("max_age",rclcpp::ParameterValue(.6));
  std::string topic;node->get_parameter(name_+".topic",topic);node->get_parameter(name_+".max_age",max_age_);
  sub_=node->create_subscription<nav_msgs::msg::OccupancyGrid>(topic,2,[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){std::lock_guard<std::mutex> l(mutex_);grid_=m;});enabled_=true;current_=false;
 }
 void updateBounds(double,double,double,double* x0,double* y0,double* x1,double* y1) override {
  // Reapply the static layer over the old as well as new observed footprint;
  // otherwise a moved object's former position could remain occupied forever.
  auto c=layered_costmap_->getCostmap();*x0=std::min(*x0,c->getOriginX());*y0=std::min(*y0,c->getOriginY());*x1=std::max(*x1,c->getOriginX()+c->getSizeInMetersX());*y1=std::max(*y1,c->getOriginY()+c->getSizeInMetersY());
 }
 void updateCosts(nav2_costmap_2d::Costmap2D& master,int x0,int y0,int x1,int y1) override {
  std::lock_guard<std::mutex> l(mutex_);auto n=node_.lock();current_=false;
  if(!grid_||grid_->info.resolution<=0||grid_->data.size()!=size_t(grid_->info.width)*grid_->info.height)return;
  double age=(n->now()-rclcpp::Time(grid_->header.stamp)).seconds();if(age<-.05||age>max_age_)return;
  geometry_msgs::msg::TransformStamped t;
  try{t=tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),grid_->header.frame_id,rclcpp::Time(grid_->header.stamp),rclcpp::Duration::from_seconds(.02));}catch(const tf2::TransformException&){return;}
  double h=tf2::getYaw(t.transform.rotation),c=std::cos(h),s=std::sin(h),tx=t.transform.translation.x,ty=t.transform.translation.y;current_=true;
  for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){double mx,my;master.mapToWorld(x,y,mx,my);mx-=tx;my-=ty;double ox=c*mx+s*my,oy=-s*mx+c*my;int i=std::floor((ox-grid_->info.origin.position.x)/grid_->info.resolution),j=std::floor((oy-grid_->info.origin.position.y)/grid_->info.resolution);if(i<0||j<0||i>=int(grid_->info.width)||j>=int(grid_->info.height))continue;int v=grid_->data[j*grid_->info.width+i];if(v<=0)continue;auto old=master.getCost(x,y);if(old==nav2_costmap_2d::NO_INFORMATION)continue;unsigned char cost=v>=100?254:static_cast<unsigned char>(v*2.5);master.setCost(x,y,std::max(old,cost));}
 }
 void reset() override{std::lock_guard<std::mutex> l(mutex_);grid_.reset();current_=false;}
 bool isClearable() override{return false;}
};
}
PLUGINLIB_EXPORT_CLASS(r08::ObservedPriorLayer,nav2_costmap_2d::Layer)
