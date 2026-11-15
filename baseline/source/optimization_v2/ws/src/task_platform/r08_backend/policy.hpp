#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
namespace r06 {
using J=nlohmann::json;
class Policy {
 public:
 J cfg,e=J::object(),d=J::object();double blocked_since=-1,clear_since=-1,progress_since=-1,last_progress=0;bool manual=false;std::string manual_reason;
 explicit Policy(J config):cfg(std::move(config)){}
 double num(const char*k,double fallback=0)const {return e.contains(k)&&e[k].is_number()&&std::isfinite(e[k].get<double>())?e[k].get<double>():fallback;}
 bool yes(const char*k,bool fallback=false)const{return e.contains(k)&&e[k].is_boolean()?e[k].get<bool>():fallback;}
 double duration()const{return blocked_since<0?0:num("t")-blocked_since;}
 bool fresh()const{return !yes("stale",true);}
 void prepare(J context,double t){
  e=std::move(context);e["t"]=t;
  if(fresh()){
   if(yes("blocked")){if(blocked_since<0)blocked_since=t;clear_since=-1;}
   else {if(clear_since<0)clear_since=t;if(t-clear_since>=cfg.at("clear_hold_s").get<double>())blocked_since=-1;}
   if(leader())blocked_since=t; // Following is progress, not a persistent stationary obstruction.
   if(yes("commanded_motion")&&yes("localized")&&(!yes("blocked")||leader())){
    if(progress_since<0||num("progress_m")-last_progress>=cfg.at("progress_delta_m").get<double>()){progress_since=t;last_progress=num("progress_m");}
    if(t-progress_since>=cfg.at("progress_timeout_s").get<double>()){manual=true;manual_reason="MOTION_WITHOUT_OBSERVED_PROGRESS";}
   }else progress_since=-1;
  }
  d=J::object();
 }
 bool emergency()const{return yes("unsafe")||num("risk",0)>=1.||yes("collision");}
 bool version_bad()const{return !yes("bindings_valid");}
 bool localized_bad()const{return !yes("localized")||!yes("sensors_healthy");}
 bool budget_bad()const{return num("remaining_s")<=cfg.at("stop_reserve_s").get<double>()+num("next_min_s");}
 bool blocked()const{return yes("blocked")||blocked_since>=0;}
 bool leader()const{return yes("blocked")&&yes("leader_on_path")&&yes("leader_same_direction")&&num("leader_confidence")>=.75&&num("leader_speed")>.05&&num("clearance_m")>=cfg.at("follow_min_gap_m").get<double>()&&num("ttc_s",0)>=cfg.at("follow_min_ttc_s").get<double>()&&yes("path_valid");}
 bool short_wait()const{return duration()<cfg.at("short_wait_s").get<double>()||(!yes("blocked")&&blocked_since>=0);}
 bool detour()const{return yes("detour_valid")&&num("detour_s",1e6)+cfg.at("stop_reserve_s").get<double>()<num("remaining_s")&&num("detour_s",1e6)<cfg.at("persistent_block_s").get<double>()-duration();}
 bool replan()const{return duration()>=cfg.at("persistent_block_s").get<double>()&&num("replans")<cfg.at("max_replans").get<int>()&&num("remaining_s")>num("replan_s",10)+cfg.at("stop_reserve_s").get<double>();}
 bool defer()const{return duration()>=cfg.at("persistent_block_s").get<double>()&&yes("alternative")&&num("deferrals")<cfg.at("max_deferrals").get<int>();}
 bool exhausted()const{if(leader())return false;return duration()>=cfg.at("max_block_s").get<double>()||(duration()>=cfg.at("persistent_block_s").get<double>()&&!replan()&&!defer());}
 void choose(const std::string&action,const std::string&reason){
  double cap=cfg.at("speed_cap_mps");
  if(action=="FOLLOW")cap=std::max(0.,std::min(cap,num("leader_speed")+.35*(num("clearance_m")-cfg.at("follow_target_gap_m").get<double>())));
  if(action=="HUMAN"){manual=true;manual_reason=reason;}
  J alternatives=J::array({{{"action","WAIT"},{"eligible",!exhausted()},{"reason","bounded observation wait with clear hysteresis"}},{{"action","FOLLOW"},{"eligible",leader()},{"reason","same-direction tracked leader, clearance and TTC bounds"}},{{"action","DETOUR"},{"eligible",detour()},{"reason","only a validated live footprint path with sufficient budget"}},{{"action","REPLAN"},{"eligible",replan()},{"reason","persistent blockage and remaining planner attempts"}},{{"action","DEFER"},{"eligible",defer()},{"reason","another obligation exists; retain residual and original denominator"}}});
  for(auto& c:alternatives)c["selected"]=c["action"]==action;
  d={{"t",num("t")},{"action",action},{"reason_code",reason},{"speed_cap_mps",cap},{"blocked_duration_s",duration()},{"manual_latched",manual},{"evidence",e},{"thresholds",cfg},{"alternatives",alternatives},{"infeasibility_claim","No global no-path proof; bounded search results only"},{"work_credit",false}};
 }
 bool reset(){if(!yes("stopped")||!yes("tool_stopped")||!yes("nav_idle")||!fresh()||emergency()||version_bad()||localized_bad())return false;manual=false;manual_reason.clear();blocked_since=-1;clear_since=-1;progress_since=-1;return true;}
 void register_nodes(BT::BehaviorTreeFactory& f){
  auto condition=[&](const std::string& name,auto fn){f.registerSimpleCondition(name,[this,fn](BT::TreeNode&){return (this->*fn)()?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});};
  condition("Emergency",&Policy::emergency);condition("WrongVersion",&Policy::version_bad);condition("LocalizationMissing",&Policy::localized_bad);condition("BudgetLow",&Policy::budget_bad);condition("Blocked",&Policy::blocked);condition("LeaderSuitable",&Policy::leader);condition("ShortWait",&Policy::short_wait);condition("ValidDetour",&Policy::detour);condition("CanReplan",&Policy::replan);condition("CanDefer",&Policy::defer);condition("BlockExhausted",&Policy::exhausted);
  f.registerSimpleCondition("InputStale",[this](BT::TreeNode&){return fresh()?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS;});
  f.registerSimpleCondition("ManualHold",[this](BT::TreeNode&){return manual?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
  f.registerSimpleCondition("CompleteFeedback",[this](BT::TreeNode&){return yes("complete")?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
  f.registerSimpleCondition("ActiveObservedPatch",[this](BT::TreeNode&){return yes("patch_active")&&yes("path_valid")?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
  auto action=[&](std::string node,std::string command,std::string reason){f.registerSimpleAction(node,[this,command,reason](BT::TreeNode&){choose(command,reason=="LATCHED"?manual_reason:reason);return BT::NodeStatus::SUCCESS;});};
  action("ExecuteObservedPatch","DETOUR","REPLANNED_PATH_CURRENT_PREFIX_VALID");
  action("EmergencyStop","STOP","SAFETY_STOP");action("VersionStop","STOP","VERSION_MISMATCH");action("StaleStop","STOP","INPUT_STALE");action("LocalizationWait","WAIT","LOCALIZATION_ADMISSION_WAIT");action("BudgetStop","STOP","BUDGET_RESERVE_STOP");action("ReportLatched","HUMAN","LATCHED");action("ReportBlocked","HUMAN","BOUNDED_REPLAN_EXHAUSTED");action("WaitClear","WAIT","SHORT_OBSTRUCTION_OR_CLEAR_HYSTERESIS");action("WaitBounded","WAIT","PERSISTENCE_THRESHOLD_NOT_REACHED");action("FollowLeader","FOLLOW","SAME_DIRECTION_CLEARANCE_CONTROL");action("UseDetour","DETOUR","VALIDATED_DETOUR_COST_BENEFIT");action("Replan","REPLAN","PERSISTENT_BLOCK_GLOBAL_QUERY");action("Defer","DEFER","TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL");action("Complete","COMPLETE","FEEDBACK_CONFIRMED_COMPLETE");action("Execute","EXECUTE","NEXT_OBLIGATION");
 }
};
}
