"""Independent Dino terrain binary; preserve sealed R08 source and executable."""
from common import *
def put(p,s):
 if not p.exists() or p.read_text()!=s:p.write_text(s)
def build_terrain():
 out=APP/'build/r09_source';out.mkdir(parents=True,exist_ok=True)
 s=(APP/'build/r08_source/terrain.cpp').read_text();old='if(v.x()>self_box_[0]&&v.x()<self_box_[1]&&v.y()>self_box_[2]&&v.y()<self_box_[3]&&v.z()<val("self_z_max"))continue;'
 new='if(r09::selfReturn(v.x(),v.y(),v.z(),self_box_,val("self_uncertainty_margin_m"),val("self_z_max")))continue;'
 assert s.count(old)==1;s='#include "'+str(PKG/'r09_backend/self_filter.hpp')+'"\n'+s.replace(old,new);s=s.replace('(s14::riskAt(c.cost[0],c.risk_t[0],t,val("unknown_after_s"))==100 && (o.inverse()*pos).head<2>().norm()<2.)','(s14::riskAt(c.cost[1],c.risk_t[1],t,val("unknown_after_s"))==100 && (o.inverse()*pos).head<2>().norm()<4.)');put(out/'terrain.cpp',s)
 cm=(APP/'build/r08_source/CMakeLists.txt').read_text().split('add_executable(r08_policy_test')[0].replace('r08_terrain','r09_terrain');cm+='\nadd_executable(r09_self_filter_test '+str(PKG/'r09_backend/self_filter_test.cpp')+')\n';from tool_build_r09 import generate_tool
 cm+=generate_tool(out);put(out/'CMakeLists.txt',cm)
 subprocess.run(['cmake','-S',str(out),'-B',str(APP/'build/r09')],check=True)
 subprocess.run(['cmake','--build',str(APP/'build/r09'),'--parallel','2'],check=True)
 subprocess.run([str(APP/'build/r09/r09_self_filter_test')],check=True)
