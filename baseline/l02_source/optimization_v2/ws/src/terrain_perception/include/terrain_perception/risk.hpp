#pragma once
#include <algorithm>
#include <cmath>
namespace s09 {
struct Profile {double step, slope, roughness, clearance, tool_width; double slope_sigma=.7,height_sigma=.012,min_obstacle=.045;};
// Geometry candidates, not a claim that the real chassis can climb a step.
inline int risk(double slope,double step,double rough,double height,bool plane,const Profile&p){
 if(!std::isfinite(height))return -1;
 if(step+p.height_sigma>p.step || rough>p.roughness || slope+p.slope_sigma>p.slope)return 100;
 if(height>p.min_obstacle && (!plane || slope<1.5))return 100;
 if(height>p.clearance && !plane)return 100;
 return slope>1.5?std::min(80,int(20+10*slope)):0;
}
inline bool clear(int previous,int candidate,int &count,int required){
 if(previous==100&&candidate<100){++count;return count>=required;}count=0;return true;
}
}
