// R02: extends the project S03 actuator with four-wheel skid drive.
// Original lease, watchdog, stop and truth separation remain intact.
// Original implementation, following Gazebo joint velocity actuator interfaces.
// Never assigns a chassis pose, linear velocity, angular velocity, or truth odometry.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
namespace r02 {
using Json=nlohmann::json;
class WheelDrive final : public gz::sim::System, public gz::sim::ISystemConfigure,
 public gz::sim::ISystemPreUpdate,public gz::sim::ISystemPostUpdate,public gz::sim::ISystemReset {
 using Wall=std::chrono::steady_clock;
 Json c,pending; std::mutex mutex;gz::transport::Node node;
 gz::transport::Node::Publisher feedback,truth;
 std::vector<gz::sim::Entity> joints;gz::sim::Entity body=0;
 uint64_t received=0,seen=0,lastSequence=0,epoch=0,resets=0;
 Wall::time_point receipt=Wall::now(),lastPublish=Wall::now();
 double lastRequestSim=-1e6,lastSim=-1,lastTruth=-1,speed=0,yaw=0,heldK=0,trackingSince=-1;
 bool latched=true,wasPaused=true;std::string reason="STARTUP";
 std::vector<double> command;std::string session,revision;bool ack=false;
 void latch(const std::string &r){if(!latched || reason=="STARTUP")reason=r;latched=true;}
 void Request(const gz::msgs::StringMsg &m){
   std::lock_guard<std::mutex> lock(mutex);
   try{pending=Json::parse(m.data());}catch(...){pending=Json();}
   receipt=Wall::now();++received;
 }
 bool read(const gz::sim::EntityComponentManager &ecm,std::vector<double>&q,std::vector<double>&v) const {
   for(auto j:joints){
     auto p=ecm.Component<gz::sim::components::JointPosition>(j);auto w=ecm.Component<gz::sim::components::JointVelocity>(j);
     if(!p||!w||p->Data().empty()||w->Data().empty()||!std::isfinite(p->Data()[0])||!std::isfinite(w->Data()[0]))return false;
     q.push_back(p->Data()[0]);v.push_back(w->Data()[0]);
   }return true;
 }
 public:
 void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
   c=Json::parse(sdf->Get<std::string>("config"));session=sdf->Get<std::string>("session");revision=c.at("config_hash");ack=c.at("kind")=="ackermann";
   gz::sim::Model m(entity);body=m.LinkByName(ecm,"base_link");
   std::vector<std::string> names={"rear_left_joint","rear_right_joint"};if(ack){names.push_back("steer_left_joint");names.push_back("steer_right_joint");}else{names.push_back("front_left_joint");names.push_back("front_right_joint");}
   for(auto &name:names){auto j=m.JointByName(ecm,name);if(!j)throw std::runtime_error("missing physical joint "+name);
     joints.push_back(j);ecm.CreateComponent(j,gz::sim::components::JointPosition());ecm.CreateComponent(j,gz::sim::components::JointVelocity());}
   command.resize(joints.size(),0);
   node.Subscribe("/s03/actuator/request",&WheelDrive::Request,this);
   feedback=node.Advertise<gz::msgs::StringMsg>("/s03/encoders");truth=node.Advertise<gz::msgs::StringMsg>("/s03/evaluation/pose");
 }
 void Reset(const gz::sim::UpdateInfo &,gz::sim::EntityComponentManager &ecm) override {
   ++epoch;++resets;lastSequence=0;lastRequestSim=-1e6;lastSim=-1;lastTruth=-1;latched=true;reason="RESET_REQUIRES_ARM";speed=yaw=heldK=0;trackingSince=-1;
   for(auto j:joints)ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(j,{0});
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
   const double t=std::chrono::duration<double>(info.simTime).count();
   const double dt=std::chrono::duration<double>(info.dt).count();
   if(info.paused&&!wasPaused){++epoch;lastSequence=0;latched=true;reason="PAUSED_REQUIRES_ARM";speed=yaw=0;}
   wasPaused=info.paused;
   if(lastSim>=0 && t<lastSim){++epoch;latched=true;reason="CLOCK_REWIND_REQUIRES_ARM";speed=yaw=0;}
   lastSim=t;
   std::vector<double> q,vel;bool valid=read(ecm,q,vel);
   const double r=c["wheel"]["radius"],T=c.contains("calibration")?c["calibration"].value("effective_track_m",c["wheel"]["track"].get<double>()):c["wheel"]["track"].get<double>(),L=c["wheelbase"];
   const double driveTrack=c.contains("calibration")?c["calibration"].value("drive_track_m",T):T;
   bool stopped=valid && std::abs(vel[0]*r)<.01 && std::abs(vel[1]*r)<.01;
   Json request;uint64_t seq;double age;
   {std::lock_guard<std::mutex> lock(mutex);request=pending;seq=received;age=std::chrono::duration<double>(Wall::now()-receipt).count();}
   if(seq!=seen){seen=seq;
     try {
       if(request.at("session")!=session || request.at("revision")!=revision || request.at("epoch").get<uint64_t>()!=epoch || request.at("sequence").get<uint64_t>()<=lastSequence)throw std::runtime_error("LEASE_ID_OR_REPLAY");
       double stamp=request.at("stamp"),v=request.at("v"),w=request.at("w");
       if(!std::isfinite(stamp)||!std::isfinite(v)||!std::isfinite(w)||t-stamp<-.03||t-stamp>.2)throw std::runtime_error("INVALID_OR_STALE_STAMP");
       lastSequence=request.at("sequence");lastRequestSim=t;
       std::string mode=request.at("mode");
       if(mode=="STOP")latch("STOP_REQUEST");
       else if(mode=="ARM" && !info.paused && stopped && std::abs(v)<.001 && std::abs(w)<.001){latched=false;reason="ARMED";}
       else if(mode!="DRIVE" && mode!="ARM")latch("INVALID_MODE");
     }catch(const std::exception &){latch("LEASE_REJECTED");}
   }
   if(age>c["safety"]["actuator_wall_s"].get<double>() || t-lastRequestSim>c["safety"]["actuator_sim_s"].get<double>())latch("ACTUATOR_WATCHDOG");
   if(!valid)latch("ENCODER_MISSING");
   double v=0,w=0;
   if(!latched && !info.paused && request.is_object() && request.value("mode",std::string())=="DRIVE"){
     v=request.value("v",0.);w=request.value("w",0.);
     v=std::clamp(v,-c["limits"]["reverse"].get<double>(),c["limits"]["max_speed"].get<double>());
     w=std::clamp(w,-c["limits"]["yaw_rate"].get<double>(),c["limits"]["yaw_rate"].get<double>());
   }
   if(ack && std::abs(v)<.001){v=w=0;}
   if(!info.paused && dt>0 && dt<.1 && valid){
     double k=heldK,error=0;
     if(ack && v!=0)k=std::clamp(w/v,-c["steering"]["max_curvature"].get<double>(),c["steering"]["max_curvature"].get<double>());
     if(ack){heldK=k;double al=atan(L*k/(1-T*k/2)),ar=atan(L*k/(1+T*k/2));error=std::max(abs(al-q[2]),abs(ar-q[3]));
       if(error>c["safety"]["alignment_error_rad"].get<double>()){
         v=std::clamp(v,-c["safety"]["alignment_speed_mps"].get<double>(),c["safety"]["alignment_speed_mps"].get<double>());
         if(trackingSince<0)trackingSince=t;
         if(t-trackingSince>c["safety"]["steering_tracking_timeout_s"].get<double>()){latch("STEERING_TRACKING_FAULT");v=0;}
       }else trackingSince=-1;
       const double gain=c["steering"]["gain"],rate=c["steering"]["rate_rad_s"];
       command[2]=std::clamp(gain*(al-q[2]),-rate,rate);command[3]=std::clamp(gain*(ar-q[3]),-rate,rate);
     }
     if(speed*v<0)v=0;
     const double rate=(abs(v)<abs(speed))?c["limits"]["deceleration"].get<double>():c["limits"]["acceleration"].get<double>();
     speed+=std::clamp(v-speed,-rate*dt,rate*dt);
     const double wrate=c["limits"]["yaw_acceleration"];yaw+=std::clamp(w-yaw,-wrate*dt,wrate*dt);
     if(ack){double a=tan(q[2]),b=tan(q[3]);k=.5*(a/(L+T*a/2)+b/(L-T*b/2));yaw=speed*k;}
     command[0]=(speed-driveTrack*yaw/2)/r;command[1]=(speed+driveTrack*yaw/2)/r;
     if(!ack){command[2]=command[0];command[3]=command[1];}
   }else if(info.paused || !valid){std::fill(command.begin(),command.end(),0);}
   for(size_t i=0;i<joints.size();++i)ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joints[i],{command[i]});
   if(std::chrono::duration<double>(Wall::now()-lastPublish).count()>=.02){
     lastPublish=Wall::now();Json d={{"t",t},{"epoch",epoch},{"resets",resets},{"q",q},{"velocity",vel},{"command",command},{"armed",!latched},{"reason",reason},{"paused",info.paused},{"limited_v",speed},{"limited_w",yaw},{"valid",valid}};
     gz::msgs::StringMsg m;m.set_data(d.dump());feedback.Publish(m);
   }
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
   const double t=std::chrono::duration<double>(info.simTime).count();if(info.paused||t-lastTruth<.019)return;lastTruth=t;
   const auto p=gz::sim::worldPose(body,ecm);Json d={{"t",t},{"epoch",epoch},{"resets",resets},{"xyz",{p.X(),p.Y(),p.Z()}},{"rpy",{p.Rot().Roll(),p.Rot().Pitch(),p.Rot().Yaw()}}};
   gz::msgs::StringMsg m;m.set_data(d.dump());truth.Publish(m);
 }
};
}
GZ_ADD_PLUGIN(r02::WheelDrive,gz::sim::System,r02::WheelDrive::ISystemConfigure,r02::WheelDrive::ISystemPreUpdate,r02::WheelDrive::ISystemPostUpdate,r02::WheelDrive::ISystemReset)
