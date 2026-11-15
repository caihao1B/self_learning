// Offline, read-only access to the actual RTAB-Map DB and its registration engine.
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/RegistrationIcp.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using json=nlohmann::json;
json pose(const rtabmap::Transform &t){auto q=t.getQuaternionf();return {t.x(),t.y(),t.z(),q.x(),q.y(),q.z(),q.w()};}
int main(int argc,char **argv){
  if(argc!=3){std::cerr<<"graph_probe DB OUTPUT_JSON\n";return 2;}
  rtabmap::Rtabmap map;
  std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
  if(!db->openConnection(argv[1],false))return 3;
  auto params=db->getLastParameters();db->closeConnection(false);db.reset();
  params["Mem/IncrementalMemory"]="false";params["Mem/LocalizationReadOnly"]="true";params["Mem/InitWMWithAllNodes"]="true";
  map.init(params,argv[1]);
  std::map<int,rtabmap::Transform> raw,opt;std::multimap<int,rtabmap::Link> links;std::map<int,rtabmap::Signature> signatures;
  map.getGraph(raw,links,false,true,&signatures,true,true,false,false,false,false);
  std::multimap<int,rtabmap::Link> optimized_links;map.getGraph(opt,optimized_links,true,true);
  json out;out["nodes"]=json::array();out["links"]=json::array();out["reopened_read_only"]=true;out["stored_optimizer_parameters"]={{"strategy",params["Optimizer/Strategy"]},{"gravity_sigma",params["Optimizer/GravitySigma"]},{"odom_gravity",params["Mem/UseOdomGravity"]}};
  for(const auto &[id,t]:raw)if(id>0){auto &s=signatures.at(id);out["nodes"].push_back({{"id",id},{"stamp",s.getStamp()},{"raw",pose(t)},{"optimized",pose(opt.at(id))},{"compressed_scan_bytes",s.sensorData().laserScanCompressed().data().total()*s.sensorData().laserScanCompressed().data().elemSize()},{"compressed_image_bytes",s.sensorData().imageCompressed().total()*s.sensorData().imageCompressed().elemSize()}});}
  for(const auto &[id,l]:links){(void)id;json info=json::array();for(int r=0;r<6;++r)for(int c=0;c<6;++c)info.push_back(l.infMatrix().at<double>(r,c));out["links"].push_back({{"from",l.from()},{"to",l.to()},{"type",l.type()},{"transform",pose(l.transform())},{"information",info}});}
  // Same settings as production: never add fabricated constraints to a real DB.
  rtabmap::ParametersMap icp={{"Icp/Strategy","1"},{"Icp/VoxelSize","0.20"},{"Icp/PointToPlane","true"},{"Icp/PointToPlaneK","12"},{"Icp/PointToPlaneMinComplexity","0.02"},{"Icp/PointToPlaneLowComplexityStrategy","0"},{"Icp/MaxCorrespondenceDistance","0.7"},{"Icp/MaxTranslation","1.5"},{"Icp/MaxRotation","0.5"},{"Icp/CorrespondenceRatio","0.30"},{"Icp/OutlierRatio","0.75"},{"Icp/Iterations","30"}};
  rtabmap::RegistrationIcp registration(icp);
  auto test=[&](int from,int to,const rtabmap::Transform &guess){
    auto a=signatures.at(from).sensorData(),b=signatures.at(to).sensorData();a.uncompressData();b.uncompressData();rtabmap::RegistrationInfo info;
    auto tr=registration.computeTransformation(a,b,guess,&info);
    json j={{"from",from},{"to",to},{"accepted",!tr.isNull()},{"rejected_reason",info.rejectedMsg},{"inlier_ratio",info.icpInliersRatio},{"complexity",info.icpStructuralComplexity},{"rms_m",info.icpRMS},{"correspondences",info.icpCorrespondences}};
    if(!tr.isNull())j["transform"]=pose(tr);
    return j;
  };
  for(const auto &[id,l]:links)if(l.type()==rtabmap::Link::kGlobalClosure||l.type()==rtabmap::Link::kLocalSpaceClosure){
    (void)id;out["accepted_loop_recheck"]=test(l.from(),l.to(),l.transform());
    for(int key:{l.from(),l.to()}){
      auto data=signatures.at(key).sensorData();data.uncompressData();const auto &scan=data.laserScanRaw();json points=json::array();
      for(int i=0;i<int(scan.data().total());i+=2){const float *p=scan.data().ptr<float>()+i*scan.data().channels();
        Eigen::Vector3f xyz=scan.localTransform().toEigen3f()*Eigen::Vector3f(p[0],p[1],p[2]);points.push_back({xyz.x(),xyz.y(),xyz.z()});}
      out["loop_scans"][std::to_string(key)]=points;
    }
    break;
  }
  if(raw.size()>3){int first=raw.begin()->first,far=first;float distance=0;
    for(const auto &[id,t]:raw){float d=(raw.at(first).inverse()*t).getNorm();if(d>distance){distance=d;far=id;}}
    out["false_candidate"]=test(first,far,rtabmap::Transform::getIdentity());out["false_candidate"]["raw_separation_m"]=distance;out["false_candidate"]["fault"]="Non-coincident real scans proposed as same place with identity guess";
    // Global place candidates need visual estimation BEFORE ICP refinement.
    // Demonstrate why a ground-dominated ICP-only false match is insufficient.
    auto visual_params=icp;visual_params["Reg/Strategy"]="2";visual_params["Vis/MinInliers"]="20";visual_params["Vis/FeatureType"]="8";
    std::unique_ptr<rtabmap::Registration> pipeline(rtabmap::Registration::create(visual_params));
    auto a=signatures.at(first).sensorData(),b=signatures.at(far).sensorData();a.uncompressData();b.uncompressData();rtabmap::RegistrationInfo info;
    auto verified=pipeline->computeTransformation(a,b,rtabmap::Transform(),&info);
    out["false_candidate"]["visual_then_icp_accepted"]=!verified.isNull();out["false_candidate"]["pipeline_rejected_reason"]=info.rejectedMsg;out["false_candidate"]["visual_inliers"]=info.inliers;
  }
  map.close(false);std::ofstream(argv[2])<<out.dump(2)<<"\n";
  std::cout<<"Reopened "<<raw.size()<<" nodes, "<<links.size()<<" constraints\n";return raw.empty()?1:0;
}
