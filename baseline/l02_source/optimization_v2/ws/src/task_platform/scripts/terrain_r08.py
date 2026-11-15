"""Keep low-surface statistics weighted by measured rays, not occupied 3-D voxels."""
from common import *

def cooperative_main(source):
 header='#include "'+str(PKG/'r08_backend/cooperative_shutdown.hpp')+'"\n'
 assert source.count('rclcpp::init(argc,argv);')==1
 return header+source.replace('rclcpp::init(argc,argv);','r08::initialize(argc,argv);').replace('rclcpp::spin(', 'r08::spin(')

def build_terrain():
 out=APP/'build/r08_source';out.mkdir(parents=True,exist_ok=True)
 s=(APP/'build/r07_source/terrain.cpp').read_text()
 s='#include "'+str(PKG/'r08_backend/surface_statistics.hpp')+'"\n'+s
 marker='  std::map<int,std::vector<double>> bins;'
 assert s.count(marker)==1
 s=s.replace(marker,'  std::map<int,std::vector<V>> ray_bins;for(auto&p:*clean){int i=index(p.x,p.y);if(i>=0&&s14::inCollisionVolume(p.z-floor(p.x,p.y),val("collision_ceiling_m")))ray_bins[i].push_back(V(p.x,p.y,p.z));}\n'+marker)
 old='auto obs=r07::groundObservation(cells_[i].observations,t,z[size_t(.25*(z.size()-1))]-f,z[size_t(.8*(z.size()-1))]-f,val("obstacle_min_height_m"))'
 new='auto stat=r08::surfaceStatistics(ray_bins[i],z[size_t(.25*(z.size()-1))]-f,z[size_t(.8*(z.size()-1))]-f,f,val("terrain_height_sigma_m"),val("obstacle_min_height_m"));auto obs=r07::groundObservation(cells_[i].observations,t,stat.ground,stat.top,val("obstacle_min_height_m"))'
 assert s.count(old)==1;s=s.replace(old,new);s=s.replace('std::abs(body.y())<.7','std::abs(body.y())<val("protective_half_width_m")').replace('closest<1.4','closest<val("protective_front_m")');(out/'terrain.cpp').write_text(cooperative_main(s))
 cmake=(APP/'build/r07_source/CMakeLists.txt').read_text().split('add_executable(r07_supervisor')[0].replace('r07_terrain','r08_terrain');(out/'CMakeLists.txt').write_text(cmake+'\nadd_executable(r08_policy_test '+str(PKG/'r08_backend/policy_test.cpp')+')\nament_target_dependencies(r08_policy_test behaviortree_cpp)\ntarget_link_libraries(r08_policy_test nlohmann_json::nlohmann_json)\n')
 (out/'CMakeLists.txt').write_text((out/'CMakeLists.txt').read_text()+'\nadd_executable(r08_shutdown_probe '+str(PKG/'r08_backend/shutdown_probe.cpp')+')\nament_target_dependencies(r08_shutdown_probe rclcpp)\n')
 subprocess.run(['cmake','-S',str(out),'-B',str(APP/'build/r08')],check=True)
 subprocess.run(['cmake','--build',str(APP/'build/r08'),'--parallel','2'],check=True)

def build_navigation():
 out=APP/'build/r08_source';source=BASE/'ws/src/gnss_supervisor/src/gnss_supervisor.cpp';s=source.read_text();s='#include "'+str(PKG/'r08_backend/global_consistency.hpp')+'"\n'+s
 old='(t.translation()-correction_.translation()).norm()';new='r08::correctionDisplacement(t,correction_,odoms_.empty()?V::Zero():V(odoms_.back().second.translation()))';assert s.count(old)==1;s=s.replace(old,new)
 s=s.replace('V gps_=V::Zero();', 'V gps_=V::Zero(),gps_antenna_=V::Zero(),gps_lever_=V::Zero();T gps_odom_=T::Identity();')
 old='(next.translation()-gps_).head<2>().norm()';new='r08::antennaDisagreement(next,gps_odom_,gps_lever_,gps_antenna_)';assert s.count(old)==1;s=s.replace(old,new)
 s=s.replace('gps_=base-reference.linear()*o.translation();', 'gps_antenna_=antenna;gps_lever_=lever;gps_odom_=o;gps_=base-reference.linear()*o.translation();')
 (out/'global_manager.cpp').write_text(cooperative_main(s));(out/'supervisor.cpp').write_text(cooperative_main((APP/'build/r07_source/supervisor.cpp').read_text()))
 cmake=(out/'CMakeLists.txt').read_text()+'''
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
add_executable(r08_global_manager global_manager.cpp)
ament_target_dependencies(r08_global_manager rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2_ros)
target_link_libraries(r08_global_manager Eigen3::Eigen nlohmann_json::nlohmann_json)
add_executable(r08_supervisor supervisor.cpp)
target_include_directories(r08_supervisor PRIVATE "@POLICY@")
ament_target_dependencies(r08_supervisor rclcpp std_msgs std_srvs behaviortree_cpp)
target_link_libraries(r08_supervisor nlohmann_json::nlohmann_json)
'''.replace('@POLICY@',str(PKG/'r08_backend'));(out/'CMakeLists.txt').write_text(cmake)
 subprocess.run(['cmake','-S',str(out),'-B',str(APP/'build/r08')],check=True);subprocess.run(['cmake','--build',str(APP/'build/r08'),'--parallel','2'],check=True)
 subprocess.run(['cmake','-S',str(PKG/'r08_nav'),'-B',str(APP/'build/r08_nav'),'-DCMAKE_INSTALL_PREFIX='+str(APP/'build/r08_nav_install')],check=True);subprocess.run(['cmake','--build',str(APP/'build/r08_nav'),'--parallel','2'],check=True);subprocess.run(['cmake','--install',str(APP/'build/r08_nav')],check=True)
