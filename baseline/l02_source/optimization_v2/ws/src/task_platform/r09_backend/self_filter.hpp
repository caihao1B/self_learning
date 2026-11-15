#pragma once
#include <array>
namespace r09 {
// Keep real low ground/steps in the uncertainty annulus. The physical body
// still removes its own returns; this never synthesizes free-space cells.
inline bool selfReturn(double x,double y,double z,const std::array<double,4>& expanded,
                       double margin,double ceiling) {
 const bool padded=x>expanded[0]&&x<expanded[1]&&y>expanded[2]&&y<expanded[3];
 const bool physical=x>expanded[0]+margin&&x<expanded[1]-margin&&
                     y>expanded[2]+margin&&y<expanded[3]-margin;
 return z<ceiling && (physical || (padded && z>.08));
}
}
