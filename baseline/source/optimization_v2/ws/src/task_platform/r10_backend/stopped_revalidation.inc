// Included inside the isolated R10 Supervisor; original driving guards stay active.
T recovery_candidate_=T::Identity();
J rejected_map_=J::object(),recovery_worker_=J::object(),recovery_gate_=J::object(),recovery_safety_=J::object(),plan_proof_=J::object();
double recovery_t_=-100,recovery_first_=0,prepared_t_=-100,matched_odom_t_=-100;
int recovery_count_=0,recovery_epoch_=0;bool prepared_=false;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_worker_sub_,recovery_gate_sub_,recovery_safety_sub_,plan_proof_sub_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_recovery_,commit_recovery_;
bool recoveryStopped(){
 const auto wall=Wall::now();
 return !clock_fault_&&std::chrono::duration<double>(wall-clock_wall_).count()<.6&&
 std::chrono::duration<double>(wall-odom_wall_).count()<.5&&std::chrono::duration<double>(wall-contract_wall_).count()<.8&&
 contract_&&fresh(contract_t_,.6)&&fresh(odom_t_,.15)&&stopped()&&
 fresh(recovery_gate_.value("t",-100.),.15)&&recovery_gate_.value("stopped",false)&&recovery_gate_.value("tool_stopped",false)&&
 fresh(recovery_worker_.value("t",-100.),.2)&&recovery_worker_.value("nav_idle",false)&&recovery_worker_.value("phase","")=="RELOCALIZING"&&
 recovery_worker_.value("task_version","")==cfg_.value("plan_revision",std::string(""));
}
bool recoveryConsistent(const T& next){
 return gps_count_>=3&&fresh(gps_t_,c("gnss_age_s"))&&
 r08::antennaDisagreement(next,gps_odom_,gps_lever_,gps_antenna_)<=c("innovation_m");
}
void observeRecovery(const T& next,double t){
 if(!hold_||!recoveryStopped()||!recoveryConsistent(next)){recovery_count_=0;return;}
 const V here=odoms_.back().second.translation();
 if(r08::correctionDisplacement(next,correction_,here)>c("map_jump_translation_m")||angle(next,correction_)>cfg_.value("stopped_recovery_max_yaw_rad",.10)){recovery_count_=0;return;}
 if(recovery_count_==0||t-recovery_t_>c("max_observation_age_s")||r08::correctionDisplacement(next,recovery_candidate_,here)>c("stable_translation_m")||angle(next,recovery_candidate_)>c("stable_yaw_rad")){
  recovery_count_=0;recovery_first_=t;
 }
 recovery_candidate_=next;recovery_t_=t;++recovery_count_;
}
bool recoveryReady(){
 return hold_&&recoveryStopped()&&recovery_count_>=cfg_["confirmation_count"].get<int>()&&
 recovery_t_-recovery_first_>=c("confirmation_span_s")&&fresh(recovery_t_,c("max_observation_age_s"))&&recoveryConsistent(recovery_candidate_);
}
bool previewValid(){
 return prepared_&&recoveryReady()&&r08::correctionDisplacement(recovery_candidate_,correction_,odoms_.back().second.translation())<=c("stable_translation_m")&&angle(recovery_candidate_,correction_)<=c("stable_yaw_rad");
}
void recordRejected(const T& next,const T& paired,const J& evidence){
 rejected_map_={{"capture_t",evidence.at("t")},{"received_t",now().seconds()},{"matched_odom_t",matched_odom_t_},
 {"map_pose",evidence.at("pose")},{"paired_odom",values(paired)},{"candidate",values(next)},{"previous",values(correction_)},
 {"displacement_m",r08::correctionDisplacement(next,correction_,odoms_.back().second.translation())},{"angle_rad",angle(next,correction_)}};
}
void setupRecovery(){
 auto sub=[this](const std::string& topic,J& target){return create_subscription<std_msgs::msg::String>(topic,20,[&target](std_msgs::msg::String::SharedPtr m){try{target=J::parse(m->data);}catch(...){target=J::object();}});};
 recovery_worker_sub_=sub("/r07/mission",recovery_worker_);recovery_gate_sub_=sub("/r07/gate",recovery_gate_);recovery_safety_sub_=sub("/r07/safety",recovery_safety_);plan_proof_sub_=sub("/r10/localization/plan_proof",plan_proof_);
 prepare_recovery_=create_service<std_srvs::srv::Trigger>("/r10/localization/prepare_revalidation",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
  r->success=recoveryReady();
  if(r->success){correction_=recovery_candidate_;map_=recovery_candidate_;map_t_=heading_t_=recovery_t_;map_bad_=false;prepared_=true;prepared_t_=now().seconds();++recovery_epoch_;plan_proof_=J::object();allow_=false;count_=0;}
  r->message=J({{"reason",r->success?"STOPPED_CANDIDATE_PREPARED_HOLD_RETAINED":"CANDIDATE_NOT_STABLE_OR_NOT_STOPPED"},{"epoch",recovery_epoch_},{"t",prepared_t_}}).dump();
 });
 commit_recovery_=create_service<std_srvs::srv::Trigger>("/r10/localization/commit_revalidation",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
  const auto& p=plan_proof_;
  r->success=previewValid()&&fresh(p.value("t",-100.),.4)&&p.value("t",-100.)>prepared_t_&&
   p.value("epoch",-1)==recovery_epoch_&&p.value("plan_revision","")==cfg_.value("plan_revision",std::string(""))&&p.value("map_revision","")==cfg_.value("map_revision",std::string(""))&&
   p.value("path_valid",false)&&p.value("footprint_valid",false)&&p.value("path_sha256",std::string("")).size()==64&&
   fresh(recovery_safety_.value("t",-100.),.45)&&recovery_safety_.value("valid",false)&&recovery_safety_.value("localization_epoch",-1)==recovery_epoch_&&recovery_safety_.value("t",-100.)>prepared_t_;
  if(r->success){hold_=false;prepared_=false;allow_=false;count_=0;recovery_count_=0;map_reason_="STOPPED_PLAN_REVALIDATED";}
  r->message=r->success?"PLAN_REVALIDATED_WAIT_FOR_ORIGINAL_CONFIRMATION":"REVALIDATION_PROOF_MISSING_STALE_OR_MISMATCHED";
 });
}
