// Actual installed Fields2Cover and OR-Tools APIs. No trust in geometric success alone.
#include <fields2cover.h>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_parameters.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
using json=nlohmann::json;
void need(bool v,const char* s){if(!v)throw std::runtime_error(s);}
F2CCell cell(const json& g){
  need(g.at("type")=="Polygon","POLYGON_REQUIRED");F2CCell c;
  for(const auto& ring:g.at("coordinates")){F2CLinearRing r;for(const auto& p:ring){double x=p.at(0),y=p.at(1);need(std::isfinite(x)&&std::isfinite(y),"NONFINITE_GEOMETRY");r.addPoint(x,y);}c.addRing(r);}
  need(c.area()>1e-8,"EMPTY_CELL");return c;
}
template<class G> json geo(const G& g){char* s=g->exportToJson();json j=json::parse(s);CPLFree(s);return j;}
json swaths(const json& j){
  auto c=cell(j.at("work"));auto motion=cell(j.at("motion"));double width=j.at("spacing"),hl=j.at("headland");need(width>.05&&width<3&&hl>=0,"BAD_SWATH_PARAMETERS");
  f2c::hg::ConstHL h;auto inner=h.generateHeadlands(F2CCells(c),hl);auto inner_motion=h.generateHeadlands(F2CCells(motion),hl);
  f2c::sg::BruteForce generator;generator.setStepAngle(M_PI/12);f2c::obj::NSwath objective;
  double angle=j.contains("angle")?j.at("angle").get<double>():generator.computeBestAngle(objective,width,c);
  f2c::decomp::BoustrophedonDecomp decomp;decomp.setSplitAngle(angle);auto cells=decomp.decompose(F2CCells(c));
  // One global line phase over the entire work zone avoids seams at cell/cluster boundaries.
  auto lines=generator.generateSwaths(angle,width,c);f2c::rp::BoustrophedonOrder order;auto sorted=order.genSortedSwaths(lines);
  json rows=json::array();for(const auto& s:sorted){auto a=s.startPoint(),b=s.endPoint();rows.push_back({{"id",s.getId()},{"a",{a.getX(),a.getY()}},{"b",{b.getX(),b.getY()}},{"length",s.length()}});}
  return {{"angle",angle},{"swaths",rows},{"inner_work",geo(inner)},{"inner_motion",geo(inner_motion)},{"decomposition",geo(cells)},{"library","Fields2Cover"},{"whole_zone_headland",true}};
}
json turns(const json& j){
  double radius=j.at("radius"),ds=j.value("ds",.05);need(radius>=2.11&&ds>0&&ds<=.1,"TURN_LIMITS");
  F2CRobot robot(1.04,.9);robot.setMinTurningRadius(radius);robot.setMaxDiffCurv(.25);robot.setCruiseVel(.12);robot.setTurnVel(.12);
  f2c::pp::DubinsCurvesCC cc;cc.setDiscretization(ds);cc.setUsingCache(false);f2c::pp::DubinsCurves simple;simple.setDiscretization(ds);simple.setUsingCache(false);json out=json::array();
  need(j.at("queries").size()<=300,"QUERY_LIMIT");
  for(const auto& q:j.at("queries")){
    auto a=q.at("start"),b=q.at("goal");
    for(int mode=0;mode<2;mode++){
    f2c::pp::TurningBase& curve=mode==0?static_cast<f2c::pp::TurningBase&>(cc):static_cast<f2c::pp::TurningBase&>(simple);
    auto path=curve.createTurn(robot,F2CPoint(a[0],a[1]),a[2],F2CPoint(b[0],b[1]),b[2]);json poses=json::array();
    for(const auto& s:path.getStates())poses.push_back({s.point.getX(),s.point.getY(),s.angle});
    out.push_back({{"poses",poses},{"length",path.length()},{"continuous_curvature",curve.hasContinuousCurvature()},{"id",q.at("id")}});
  }
  }
  return {{"turns",out}};
}
json tsp(const json& j){
  using namespace operations_research;auto cost=j.at("cost").get<std::vector<std::vector<int64_t>>>();int n=cost.size();need(n>=2&&n<=500,"TSP_SIZE");
  for(const auto& row:cost){need(row.size()==static_cast<size_t>(n),"MATRIX_SHAPE");for(auto v:row)need(v>=0&&v<1000000000000LL,"MATRIX_VALUE");}
  RoutingIndexManager manager(n,1,RoutingIndexManager::NodeIndex(0));RoutingModel routing(manager);
  const int cb=routing.RegisterTransitCallback([&](int64_t i,int64_t k){return cost[manager.IndexToNode(i).value()][manager.IndexToNode(k).value()];});routing.SetArcCostEvaluatorOfAllVehicles(cb);
  auto options=DefaultRoutingSearchParameters();options.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);options.set_local_search_metaheuristic(LocalSearchMetaheuristic::GREEDY_DESCENT);
  options.mutable_time_limit()->set_seconds(2);options.set_solution_limit(1000);const Assignment* solution=routing.SolveWithParameters(options);need(solution!=nullptr,"TSP_NO_SOLUTION");
  json order=json::array();for(auto i=routing.Start(0);!routing.IsEnd(i);i=solution->Value(routing.NextVar(i)))order.push_back(manager.IndexToNode(i).value());
  return {{"order",order},{"cost",solution->ObjectiveValue()},{"optimality_claimed",false},{"solver","OR-Tools RoutingModel"}};
}
int main(int argc,char** argv){
 try{need(argc==3,"USAGE request.json result.json");std::ifstream f(argv[1]);need(f.good(),"NO_INPUT");json j;f>>j;std::string mode=j.at("mode");json result;
 if(mode=="swaths")result=swaths(j);else if(mode=="turns")result=turns(j);else if(mode=="tsp")result=tsp(j);else throw std::runtime_error("UNKNOWN_MODE");
 std::ofstream o(argv[2]);need(o.good(),"NO_OUTPUT");o<<result.dump()<<"\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}
}
