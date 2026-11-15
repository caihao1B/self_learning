// Offline known-prior connector. No runtime truth, ROS topic or control authority.
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/RandomNumbers.h>
#include <boost/geometry.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
namespace ob=ompl::base;namespace og=ompl::geometric;namespace bg=boost::geometry;
using J=nlohmann::json;using P=bg::model::d2::point_xy<double>;using Poly=bg::model::polygon<P>;using Multi=bg::model::multi_polygon<Poly>;
Poly polygon(const J& rings){Poly p;for(const auto& xy:rings.at(0))bg::append(p.outer(),P(xy[0],xy[1]));for(size_t i=1;i<rings.size();++i){p.inners().resize(i);for(const auto& xy:rings[i])bg::append(p.inners()[i-1],P(xy[0],xy[1]));}bg::correct(p);return p;}
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("request.json result.json required");std::ifstream in(argv[1]);J q;in>>q;
 ompl::RNG::setSeed(q.value("seed",41));double radius=q.at("radius");if(!std::isfinite(radius)||radius<2.998)throw std::runtime_error("turning radius contract");
 Multi allowed;J geo=q.at("allowed");if(geo.at("type")=="Polygon")allowed.push_back(polygon(geo.at("coordinates")));else if(geo.at("type")=="MultiPolygon")for(auto& p:geo.at("coordinates"))allowed.push_back(polygon(p));else throw std::runtime_error("polygon prior required");
 auto space=std::make_shared<ob::ReedsSheppStateSpace>(radius);ob::RealVectorBounds bounds(2);
 for(int i=0;i<2;++i){bounds.setLow(i,std::min(q["start"][i].get<double>(),q["goal"][i].get<double>())-12.);bounds.setHigh(i,std::max(q["start"][i].get<double>(),q["goal"][i].get<double>())+12.);}space->setBounds(bounds);
 auto si=std::make_shared<ob::SpaceInformation>(space);size_t checks=0;
 si->setStateValidityChecker([&](const ob::State* state){++checks;auto s=state->as<ob::SE2StateSpace::StateType>();Poly body;double c=cos(s->getYaw()),sn=sin(s->getYaw());for(auto& xy:q["footprint"]){double x=xy[0],y=xy[1];bg::append(body.outer(),P(s->getX()+28+c*x-sn*y,s->getY()+14+sn*x+c*y));}bg::correct(body);return bg::covered_by(body,allowed);});
 si->setStateValidityCheckingResolution(.04/space->getMaximumExtent());si->setup();ob::ScopedState<> a(space),b(space);
 auto set=[](ob::State* p,const J& v){auto s=p->as<ob::SE2StateSpace::StateType>();s->setX(v[0]);s->setY(v[1]);s->setYaw(v[2]);};set(a.get(),q["start"]);set(b.get(),q["goal"]);
 if(!si->isValid(a.get())||!si->isValid(b.get()))throw std::runtime_error("ENDPOINT_ENVELOPE_OUTSIDE_PRIOR");
 auto path=std::make_shared<og::PathGeometric>(si);std::string method;
 if(si->checkMotion(a.get(),b.get())){path->append(a.get());path->append(b.get());method="direct collision-checked Reeds-Shepp";}
 else{auto problem=std::make_shared<ob::ProblemDefinition>(si);problem->setStartAndGoalStates(a,b);auto planner=std::make_shared<og::RRTConnect>(si);planner->setRange(2.);planner->setProblemDefinition(problem);planner->setup();auto status=planner->solve(ob::timedPlannerTerminationCondition(q.value("seconds",3.)));if(status!=ob::PlannerStatus::EXACT_SOLUTION)throw std::runtime_error("NO_EXACT_BOUNDED_CONNECTOR");path=std::dynamic_pointer_cast<og::PathGeometric>(problem->getSolutionPath());og::PathSimplifier simplifier(si);simplifier.reduceVertices(*path,150,40,1.);simplifier.partialShortcutPath(*path,150,40,1.);method="bounded RRTConnect in Reeds-Shepp space + collision-checked shortcuts";}
 J points=J::array();auto sample=space->allocState();
 for(size_t i=1;i<path->getStateCount();++i){auto x=path->getState(i-1),y=path->getState(i);auto rs=space->reedsShepp(x,y);double total=rs.length()*radius,offset=0.;
  if(total<1e-9)continue;
  // Include every exact Reeds-Shepp primitive boundary, especially gear cusps.
  // Uniform arc samples across a cusp can create a false high-curvature chord.
  for(int part=0;part<5;++part){double length=std::abs(rs.length_[part])*radius;if(length<1e-9)continue;int count=std::max(1,(int)ceil(length/.04));
   for(int k=(points.empty()?0:1);k<=count;++k){space->interpolate(x,y,(offset+length*double(k)/count)/total,sample);if(!si->isValid(sample))throw std::runtime_error("DENSE_FINAL_COLLISION");auto s=sample->as<ob::SE2StateSpace::StateType>();points.push_back({s->getX(),s->getY(),s->getYaw()});}offset+=length;
  }
 }
 space->freeState(sample);std::ofstream out(argv[2]);out<<J({{"points",points},{"length_m",path->length()},{"method",method},{"validity_checks",checks},{"seed",q.value("seed",41)}}).dump();return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
