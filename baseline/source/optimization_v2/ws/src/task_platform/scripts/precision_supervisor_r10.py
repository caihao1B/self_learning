"""Independent spray BT build; protected inspection/mowing supervisor is unchanged."""
from common import *

def derive(policy):
 marker='num("progress_m")-last_progress>=cfg.at("progress_delta_m").get<double>()'
 assert policy.count(marker)==1
 policy=policy.replace(marker,'num("progress_m")-last_progress>=r10::progressDelta(cfg,e)')
 marker='e=std::move(context);e["t"]=t;'
 assert policy.count(marker)==1
 policy=policy.replace(marker,marker+'e["effective_progress_delta_m"]=r10::progressDelta(cfg,e);')
 return '#include "'+str(PKG/'r10_backend/precision_progress.hpp')+'"\n'+policy

def build():
 out=APP/'build/r10_spray_bt_source';out.mkdir(parents=True,exist_ok=True)
 files={'policy.hpp':derive((PKG/'r08_backend/policy.hpp').read_text()),
        'supervisor.cpp':(APP/'build/r08_source/supervisor.cpp').read_text(),
        'original_policy_test.cpp':(PKG/'r08_backend/policy_test.cpp').read_text(),
        'precision_progress_test.cpp':(PKG/'r10_backend/precision_progress_test.cpp').read_text()}
 files['CMakeLists.txt']='''cmake_minimum_required(VERSION 3.16)
project(r10_spray_bt)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O2 -Wall -Wextra)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp 4.10.0 EXACT REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(r10_spray_supervisor supervisor.cpp)
ament_target_dependencies(r10_spray_supervisor rclcpp std_msgs std_srvs behaviortree_cpp)
target_link_libraries(r10_spray_supervisor nlohmann_json::nlohmann_json)
foreach(name original_policy_test precision_progress_test)
 add_executable(${name} ${name}.cpp)
 ament_target_dependencies(${name} behaviortree_cpp)
 target_link_libraries(${name} nlohmann_json::nlohmann_json)
endforeach()
'''
 for name,text in files.items():
  target=out/name
  if not target.exists() or target.read_text()!=text:target.write_text(text)
 subprocess.run(['cmake','-S',str(out),'-B',str(APP/'build/r10_spray_bt')],check=True)
 subprocess.run(['cmake','--build',str(APP/'build/r10_spray_bt'),'--parallel','1'],check=True)
 return APP/'build/r10_spray_bt/r10_spray_supervisor'
