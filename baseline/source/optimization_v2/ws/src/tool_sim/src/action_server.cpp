#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <mission_interfaces/action/tool_task.hpp>
#include <mission_interfaces/srv/ledger_command.hpp>
#include <mission_interfaces/srv/tool_capabilities.hpp>
using J=nlohmann::json;using Task=mission_interfaces::action::ToolTask;using Handle=rclcpp_action::ServerGoalHandle<Task>;using Service=mission_interfaces::srv::LedgerCommand;using Wall=std::chrono::steady_clock;
// Single-threaded executor + asynchronous ledger calls: no detached goal worker, no second tool owner.
class ToolServer:public rclcpp::Node {
 J cfg_,feedback_,ledger_;std::string boot_,key_,attempt_,phase_="IDLE",terminal_,reason_;double t_=0,start_=0,stop_t_=0,stopped_since_=-1,working_since_=-1,last_sensor_=-1;uint64_t sequence_=0;
 Wall::time_point clock_wall_=Wall::now(),feedback_wall_=Wall::now(),ledger_wall_=Wall::now(),phase_wall_=Wall::now();bool rewound_=false,pending_=false,reserved_=false,uncertain_=false;
 std::shared_ptr<Handle> goal_,stop_goal_;rclcpp_action::Server<Task>::SharedPtr server_;rclcpp::Client<Service>::SharedPtr client_;
 rclcpp::Service<mission_interfaces::srv::ToolCapabilities>::SharedPtr capabilities_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_,state_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fb_sub_,ledger_sub_;rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;rclcpp::TimerBase::SharedPtr timer_;
 static double age(Wall::time_point p){return std::chrono::duration<double>(Wall::now()-p).count();}
 bool fresh(){return feedback_.is_object()&&age(feedback_wall_)<=cfg_["feedback_age_wall_s"].get<double>()&&t_-feedback_.value("t",-100.)>=-.001&&t_-feedback_.value("t",-100.)<=cfg_["feedback_age_sim_s"].get<double>()&&feedback_.value("tool_revision","")==cfg_["tool_revision"]&&feedback_.value("sensor_epoch","")==cfg_["session"];}
 bool stopped(){return fresh()&&stopped_since_>=0&&last_sensor_-stopped_since_>=cfg_["stopped_confirm_sim_s"].get<double>();}
 void send(bool on){J d={{"session",cfg_["session"]},{"tool_revision",cfg_["tool_revision"]},{"boot",boot_},{"sequence",++sequence_},{"key",key_},{"attempt",attempt_},{"t",t_},{"mode",on?"ON":"OFF"}};std_msgs::msg::String m;m.data=d.dump();command_->publish(m);}
 J encode(const Task::Goal&g){J a=J::array();for(auto&p:g.target_area.points)a.push_back({p.x,p.y});return {{"operation",g.operation},{"task_id",g.task_id},{"idempotency_key",g.idempotency_key},{"map_id",g.map_id},{"map_revision",g.map_revision},{"frame_epoch",g.frame_epoch},{"plan_revision",g.plan_revision},{"tool_revision",g.tool_revision},{"segment_id",g.segment_id},{"frame",g.target.header.frame_id},{"target",{g.target.pose.position.x,g.target.pose.position.y,g.target.pose.position.z}},{"area",a},{"requested_quantity",g.requested_quantity},{"timeout_sim_s",g.timeout_sim_s}};}
 void request(J data,std::function<void(J)> done){pending_=true;auto r=std::make_shared<Service::Request>();r->request_json=data.dump();client_->async_send_request(r,[this,done](rclcpp::Client<Service>::SharedFuture f){pending_=false;try{done(J::parse(f.get()->response_json));}catch(const std::exception&e){begin_stop("FAILED",std::string("LEDGER_ERROR:")+e.what(),true);}});}
 void complete(std::shared_ptr<Handle> h,const J&d,const std::string&fallback="FAILED"){
  auto r=std::make_shared<Task::Result>();r->state=d.value("state",fallback);r->error_code=d.value("error",d.value("reason",std::string("")));r->reason=d.value("reason",r->error_code);r->confirmed_quantity=d.value("confirmed_quantity",0.);r->remaining_quantity=d.value("remaining_quantity",0.);r->residual_geojson=d.value("residual_geojson",std::string("{}"));r->uncertainty_pending=d.value("uncertainty_pending",false);r->replayed=d.value("replayed",false);
  if(h->is_canceling())h->canceled(r);else if(r->state=="SUCCEEDED"||r->state=="STOPPED")h->succeed(r);else h->abort(r);
 }
 void begin_stop(std::string terminal,std::string reason,bool uncertain=false){if(phase_=="STOPPING"||phase_=="FINISHING")return;terminal_=terminal;reason_=reason;uncertain_=uncertain;phase_="STOPPING";stop_t_=t_;phase_wall_=Wall::now();send(false);}
 void accept(std::shared_ptr<Handle> h){
  if(h->get_goal()->operation==Task::Goal::STOP_WORK){stop_goal_=h;if(goal_)begin_stop("CANCELLED","STOP_WORK");else{phase_="STOPPING";terminal_="STOPPED";reason_="STOP_WORK";stop_t_=t_;phase_wall_=Wall::now();}return;}
  goal_=h;key_=h->get_goal()->idempotency_key;attempt_=boot_+"-"+std::to_string(sequence_);phase_="RESERVING";start_=t_;phase_wall_=Wall::now();reserved_=false;uncertain_=false;
  request({{"op","reserve"},{"goal",encode(*h->get_goal())},{"attempt",attempt_}},[this,h](J d){
   if(goal_!=h||(phase_!="RESERVING"&&phase_!="STOPPING"))return;
   if(!d.value("ok",false)||d.value("replayed",false)){complete(goal_,d);goal_.reset();phase_=stop_goal_?"STOPPING":"IDLE";return;}
   reserved_=true;if(phase_!="STOPPING")phase_="ADMISSION";phase_wall_=Wall::now();ledger_=d;
  });
 }
 void tick(){
  bool allow=false;
  if(goal_&&phase_!="STOPPING"&&phase_!="FINISHING"&&phase_!="RESERVING"){
   if(goal_->is_canceling())begin_stop("CANCELLED","USER_CANCEL");
   else if(rewound_||age(clock_wall_)>cfg_["clock_wall_s"].get<double>())begin_stop("LOST","CLOCK_WATCHDOG",true);
   else if(t_-start_>goal_->get_goal()->timeout_sim_s)begin_stop(ledger_.value("confirmed_quantity",0.)>0?"PARTIAL":"FAILED","DEADLINE");
   else if(!fresh()&&t_-start_>1.)begin_stop("LOST","FEEDBACK_LOST",true);
   else if(fresh()&&!feedback_.value("healthy",false))begin_stop(ledger_.value("confirmed_quantity",0.)>0?"PARTIAL":"FAILED","TOOL_HEALTH");
   else if(phase_=="HOLDING"){
    if(stopped()&&ledger_.value("start_guard",std::string("PERCEPTION"))!="PERCEPTION"){phase_="ADMISSION";phase_wall_=Wall::now();}
   }else if(phase_=="ADMISSION"){
    if(ledger_.value("start_valid",false)&&ledger_.value("attempt","")==attempt_&&age(ledger_wall_)<.2&&stopped()&&feedback_.value("boot","")==boot_&&feedback_.value("attempt","")==attempt_){phase_="WORKING";phase_wall_=Wall::now();allow=true;}
    else if(age(phase_wall_)>2.4){if(ledger_.value("start_guard",std::string(""))=="PERCEPTION")phase_="HOLDING";else begin_stop("FAILED",ledger_.value("start_guard",std::string("STOP_NOT_CONFIRMED")));}
   }else if(phase_=="WORKING"){
    double fraction=goal_->get_goal()->operation==Task::Goal::POINT_SERVICE?1.:cfg_["completion_fraction"].get<double>();
    if(ledger_.value("confirmed_quantity",0.)>=goal_->get_goal()->requested_quantity*fraction)begin_stop("SUCCEEDED","CONFIRMED_WORK");
    else if(age(ledger_wall_)>.25)begin_stop("LOST","LEDGER_STALE",true);
    else if(!ledger_.value("start_valid",false)){if(ledger_.value("start_guard",std::string(""))=="PERCEPTION"||(goal_->get_goal()->operation==2&&ledger_.value("start_guard",std::string(""))=="WORK_SPEED")){phase_="HOLDING";phase_wall_=Wall::now();}else begin_stop(ledger_.value("confirmed_quantity",0.)>0?"PARTIAL":"FAILED",ledger_.value("start_guard",std::string("INVALID_CONTEXT")));}
    else allow=fresh();
   }
  }
  if(phase_=="RESERVING"&&age(phase_wall_)>2.){send(false);complete(goal_,{{"state","LOST"},{"error","LEDGER_RESERVATION_TIMEOUT"},{"uncertainty_pending",true}});goal_.reset();pending_=false;phase_="IDLE";return;} // never actuate while transaction is unresolved
  if(phase_=="STOPPING"&&!pending_){
   bool confirmed=stopped();bool timeout=t_-stop_t_>cfg_["stop_timeout_s"].get<double>()||age(phase_wall_)>3.;
   if(confirmed||timeout){
    if(!confirmed){uncertain_=true;terminal_="LOST";reason_+="|STOP_UNCONFIRMED";}
    phase_="FINISHING";
    if(goal_&&reserved_)request({{"op","finish"},{"key",key_},{"state",terminal_},{"reason",reason_},{"uncertain",uncertain_}},[this](J d){complete(goal_,d);goal_.reset();reserved_=false;if(stop_goal_){complete(stop_goal_,{{"state",uncertain_?"LOST":"STOPPED"},{"reason",reason_},{"uncertainty_pending",uncertain_}});stop_goal_.reset();}phase_="IDLE";});
    else{if(goal_){complete(goal_,{{"state",terminal_},{"reason",reason_}});goal_.reset();}if(stop_goal_){complete(stop_goal_,{{"state",confirmed?"STOPPED":"LOST"},{"uncertainty_pending",!confirmed}});stop_goal_.reset();}phase_="IDLE";}
   }
  }
  send(allow&&phase_=="WORKING"&&!rewound_&&age(clock_wall_)<=cfg_["clock_wall_s"].get<double>());
  J st={{"t",t_},{"key",key_},{"phase",phase_},{"reason",reason_},{"stopped_confirmed",stopped()},{"working_confirmed",fresh()&&working_since_>=0&&last_sensor_-working_since_>=cfg_["working_confirm_sim_s"].get<double>()},{"feedback_fresh",fresh()},{"boot",boot_},{"active",bool(goal_)},{"tool_on_requested",allow&&phase_=="WORKING"}};std_msgs::msg::String sm;sm.data=st.dump();state_->publish(sm);
  if(goal_){auto f=std::make_shared<Task::Feedback>();f->phase=phase_;f->reason=ledger_.value("start_guard",std::string(""));f->confirmed_quantity=ledger_.value("confirmed_quantity",0.);f->progress=std::clamp(f->confirmed_quantity/std::max(.00001,goal_->get_goal()->requested_quantity),0.,1.);f->health=fresh()&&feedback_.value("healthy",false)?1.:0.;f->elapsed_sim_s=t_-start_;goal_->publish_feedback(f);}
 }
public:
 ToolServer():Node("s10_tool_action_server"){
  auto file=declare_parameter<std::string>("config_file","");std::ifstream in(file);in>>cfg_;boot_=std::to_string(Wall::now().time_since_epoch().count());
  command_=create_publisher<std_msgs::msg::String>("/s10/tool/command",10);state_=create_publisher<std_msgs::msg::String>("/s10/action/state",10);client_=create_client<Service>("/s10/ledger/command");
  clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",rclcpp::SensorDataQoS(),[this](rosgraph_msgs::msg::Clock::ConstSharedPtr m){double t=m->clock.sec+m->clock.nanosec*1e-9;if(t<t_)rewound_=true;if(t!=t_)clock_wall_=Wall::now();t_=t;});
  fb_sub_=create_subscription<std_msgs::msg::String>("/s10/tool/feedback",30,[this](std_msgs::msg::String::ConstSharedPtr m){try{auto d=J::parse(m->data);double t=d.at("t");bool ordered=t>last_sensor_&&t-last_sensor_<=cfg_["feedback_gap_s"].get<double>();feedback_=d;feedback_wall_=Wall::now();if(!fresh()){stopped_since_=-1;working_since_=-1;return;}if(std::abs(d.value("rpm_rad_s",100.))<=cfg_["stopped_max_rad_s"].get<double>()){if(!ordered||stopped_since_<0)stopped_since_=t;}else stopped_since_=-1;if(d.value("healthy",false)&&d.value("request_on",false)&&std::abs(d.value("rpm_rad_s",0.))>=cfg_["working_min_rad_s"].get<double>()&&d.value("boot","")==boot_&&d.value("attempt","")==attempt_){if(!ordered||working_since_<0)working_since_=t;}else working_since_=-1;last_sensor_=t;}catch(...){stopped_since_=-1;}});
  ledger_sub_=create_subscription<std_msgs::msg::String>("/s10/ledger/state",20,[this](std_msgs::msg::String::ConstSharedPtr m){try{auto d=J::parse(m->data);if(d.value("key","")==key_&&d.value("attempt","")==attempt_){ledger_=d;ledger_wall_=Wall::now();}}catch(...){}});
  capabilities_=create_service<mission_interfaces::srv::ToolCapabilities>("/s10/tool/capabilities",[this](const std::shared_ptr<mission_interfaces::srv::ToolCapabilities::Request>,std::shared_ptr<mission_interfaces::srv::ToolCapabilities::Response> r){r->capabilities_json=cfg_.dump();});
  server_=rclcpp_action::create_server<Task>(this,"/s10/tool_task",[this](const rclcpp_action::GoalUUID&,std::shared_ptr<const Task::Goal> g){if(g->operation==Task::Goal::STOP_WORK)return stop_goal_?rclcpp_action::GoalResponse::REJECT:rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;if(goal_||stop_goal_||!client_->service_is_ready()||g->operation<1||g->operation>2||!std::isfinite(g->timeout_sim_s)||g->timeout_sim_s<=0)return rclcpp_action::GoalResponse::REJECT;return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},[](std::shared_ptr<Handle>){return rclcpp_action::CancelResponse::ACCEPT;},[this](std::shared_ptr<Handle> h){accept(h);});
  timer_=create_wall_timer(std::chrono::milliseconds(40),[this]{tick();});
 }
 ~ToolServer(){send(false);}
};
int main(int argc,char**argv){rclcpp::init(argc,argv);auto node=std::make_shared<ToolServer>();rclcpp::spin(node);node.reset();rclcpp::shutdown();}
