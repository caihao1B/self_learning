"""Isolated native ToolTask receiver: wait for clock epoch, never restamp feedback."""
from common import *
def put(p,s):
 if not p.exists() or p.read_text()!=s:p.write_text(s)
def generate_tool(out):
 s=(BASE/'ws/src/tool_sim/src/action_server.cpp').read_text()
 s='#include <deque>\n'+s
 s=s.replace(' bool fresh(){',' std::deque<std::pair<J,Wall::time_point>> incoming_;\n bool fresh(){')
 start=s.index('try{auto d=J::parse(m->data);double t=d.at("t");bool ordered=')
 end=s.index(');\n  ledger_sub_',start)
 original=s[start:end]
 assert original.endswith('}}')
 body=original[len('try{auto d=J::parse(m->data);'): -len('catch(...){stopped_since_=-1;}}')]
 assert body.endswith('}')
 body=body[:-1].replace('feedback_wall_=Wall::now();','feedback_wall_=arrival;')
 helper=' void consume(J d,Wall::time_point arrival){'+body+'}\n void drain(){while(!incoming_.empty()&&incoming_.front().first.value("t",1e99)<=t_+.001){auto p=incoming_.front();incoming_.pop_front();if(p.first.value("t",-1.)>last_sensor_)consume(p.first,p.second);}}\n'
 s=s[:start]+'try{auto d=J::parse(m->data);incoming_.push_back({d,Wall::now()});if(incoming_.size()>30)incoming_.pop_front();drain();}catch(...){stopped_since_=-1;}}'+s[end:]
 s=s.replace(' void tick(){',helper+' void tick(){\n  drain();')
 s=s.replace('t_=t;});','t_=t;drain();});')
 s=s.replace('begin_stop("LOST","FEEDBACK_LOST",true);','{RCLCPP_ERROR(get_logger(), "Feedback freshness failure sim=%.6f sensor=%.6f wall_age=%.6f queued=%zu",t_,feedback_.value("t",-100.),age(feedback_wall_),incoming_.size());begin_stop("LOST","FEEDBACK_LOST",true);}')
 put(out/'tool_action_server.cpp',s)
 plugin=(BASE/'ws/src/tool_sim/src/ToolActuator.cc').read_text().replace('t-last_pub_>=.049','t-last_pub_>=cfg_.value("feedback_period_s",.049)')
 put(out/'ToolActuator.cc',plugin)
 return """\nset(Protobuf_PROTOC_EXECUTABLE /usr/bin/protoc CACHE FILEPATH "Gazebo ABI" FORCE)
find_package(Protobuf MODULE REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
add_library(s10_tool_r09 SHARED ToolActuator.cc)
target_link_libraries(s10_tool_r09 gz-sim8::gz-sim8 gz-plugin2::gz-plugin2 gz-transport13::gz-transport13 nlohmann_json::nlohmann_json)
find_package(rclcpp_action REQUIRED)
find_package(mission_interfaces REQUIRED)
add_executable(r09_tool_action_server tool_action_server.cpp)
ament_target_dependencies(r09_tool_action_server rclcpp rclcpp_action mission_interfaces std_msgs rosgraph_msgs)
target_link_libraries(r09_tool_action_server nlohmann_json::nlohmann_json)
"""
