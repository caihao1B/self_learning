#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
namespace r08 {
struct SurfaceStatistics {double ground=0,top=0;int supported_modes=0;};
inline SurfaceStatistics surfaceStatistics(const std::vector<Eigen::Vector3d>& rays,
 double voxel_ground,double voxel_top,double floor,double sigma=.012,double strong=.045) {
 if(rays.empty())return {voxel_ground,voxel_top,0};
 std::vector<double> z;for(const auto&p:rays)z.push_back(p.z());std::sort(z.begin(),z.end());
 SurfaceStatistics r{z[size_t(.25*(z.size()-1))]-floor,z[size_t(.8*(z.size()-1))]-floor,0};
 // Strong geometric hazards keep the existing immediate upper-channel policy.
 if(voxel_top>=strong||voxel_top<=-strong)r.top=voxel_top;
 // A small, coherent raised surface must not disappear into many floor rays.
 // Require spatial support and a confidence-separated mode; random isolated
 // range-noise bins must not receive the same weight as hundreds of floor rays.
 std::map<std::pair<int,int>,std::vector<double>> high,low;
 for(const auto&p:rays){double h=p.z()-floor;auto key=std::make_pair(int(std::floor(p.x()/.04)),int(std::floor(p.y()/.04)));if(h>r.ground+2*sigma)high[key].push_back(h);if(h<r.top-2*sigma)low[key].push_back(h);}
 for(const auto&[key,a]:high)if(a.size()>=3){double mean=0;for(double v:a)mean+=v;mean/=a.size();if(mean-2*sigma/std::sqrt(a.size())>r.ground+2*sigma){r.top=std::max(r.top,mean);++r.supported_modes;}}
 for(const auto&[key,a]:low)if(a.size()>=3){double mean=0;for(double v:a)mean+=v;mean/=a.size();if(mean+2*sigma/std::sqrt(a.size())<r.top-2*sigma){r.ground=std::min(r.ground,mean);++r.supported_modes;}}
 return r;
}
}
