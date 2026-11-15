// Real METIS bridge. Geometry and vehicle feasibility belong to the Python verifier.
#include <metis.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <stdexcept>
#include <limits>
using json=nlohmann::json;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
std::vector<idx_t> integers(const json& j) {
  require(j.is_array(),"CSR_ARRAY_REQUIRED"); std::vector<idx_t> result;
  for (const auto& x:j) {
    require(x.is_number_integer(),"NONINTEGER_CSR"); auto v=x.get<int64_t>();
    require(v>=0 && v<=std::numeric_limits<idx_t>::max(),"IDX_RANGE");
    result.push_back(static_cast<idx_t>(v));
  }
  return result;
}
int main(int argc,char** argv) {
  try {
    if(argc==2 && std::string(argv[1])=="--version") {
      std::cout<<json{{"metis",std::to_string(METIS_VER_MAJOR)+"."+std::to_string(METIS_VER_MINOR)+"."+std::to_string(METIS_VER_SUBMINOR)},
        {"idx_bits",8*sizeof(idx_t)},{"real_bits",8*sizeof(real_t)}}.dump()<<"\n";return 0;
    }
    require(argc==3,"USAGE input.json output.json"); std::ifstream f(argv[1]);require(f.good(),"NO_INPUT");json j;f>>j;
    auto x=integers(j.at("xadj")), a=integers(j.at("adjncy")), v=integers(j.at("vwgt")), w=integers(j.at("adjwgt"));
    require(j.at("k").is_number_integer() && j.at("seed").is_number_integer() && j.at("ufactor").is_number_integer(),"INTEGER_OPTIONS_REQUIRED");
    auto numbers=integers(json::array({j.at("k"),j.at("seed"),j.at("ufactor")}));
    idx_t n=static_cast<idx_t>(v.size()),k=numbers[0],seed=numbers[1],ufactor=numbers[2];
    require(n>=k && k>=2 && n<=10000 && a.size()<=100000,"GRAPH_SIZE");
    require(seed>=0 && ufactor>=1 && ufactor<=500,"OPTIONS");
    require(x.size()==static_cast<size_t>(n+1) && x.front()==0 && x.back()==static_cast<idx_t>(a.size()) && w.size()==a.size(),"CSR_SIZE");
    std::vector<std::map<idx_t,idx_t>> edges(n);int64_t total=0;
    for(idx_t i=0;i<n;i++) {
      require(v[i]>0 && x[i]<=x[i+1] && x[i+1]<=static_cast<idx_t>(a.size()),"BAD_WEIGHT_OR_OFFSET");total+=v[i];
      for(idx_t e=x[i];e<x[i+1];e++) {
        require(a[e]<n && a[e]!=i && w[e]>0,"BAD_EDGE");
        require(edges[i].emplace(a[e],w[e]).second,"DUPLICATE_EDGE");
      }
    }
    require(total<std::numeric_limits<idx_t>::max()/4,"WEIGHT_OVERFLOW");
    int64_t edge_total=0;
    for(idx_t i=0;i<n;i++) for(auto [b,weight]:edges[i]) {
      require(edges[b].count(i) && edges[b].at(i)==weight,"ASYMMETRIC_GRAPH");edge_total+=weight;
    }
    require(edge_total<std::numeric_limits<idx_t>::max()/4,"EDGE_WEIGHT_OVERFLOW");
    std::vector<bool> seen(n,false);std::queue<idx_t> q;q.push(0);seen[0]=true;idx_t reached=0;
    while(!q.empty()){auto i=q.front();q.pop();reached++;for(auto [b,weight]:edges[i])if(!seen[b]){seen[b]=true;q.push(b);}}
    require(reached==n,"DISCONNECTED_INPUT");
    idx_t opts[METIS_NOPTIONS];METIS_SetDefaultOptions(opts);
    opts[METIS_OPTION_NUMBERING]=0;opts[METIS_OPTION_SEED]=seed;
    opts[METIS_OPTION_OBJTYPE]=METIS_OBJTYPE_CUT;opts[METIS_OPTION_CONTIG]=1;
    opts[METIS_OPTION_MINCONN]=1;opts[METIS_OPTION_UFACTOR]=ufactor;
    opts[METIS_OPTION_NCUTS]=1;opts[METIS_OPTION_NITER]=20;
    idx_t ncon=1,cut=0;std::vector<idx_t> labels(n);
    int status=METIS_PartGraphKway(&n,&ncon,x.data(),a.data(),v.data(),nullptr,w.data(),&k,nullptr,nullptr,opts,&cut,labels.data());
    require(status==METIS_OK,"METIS_FAILED");std::vector<int> counts(k,0);
    for(auto c:labels){require(c>=0 && c<k,"BAD_LABEL");counts[c]++;}
    for(auto c:counts)require(c>0,"EMPTY_PART");
    json out={{"status","OK"},{"labels",labels},{"edge_cut",cut},{"counts",counts},{"seed",seed},{"contig",true},{"ufactor",ufactor}};
    std::ofstream o(argv[2]);require(o.good(),"NO_OUTPUT");o<<out.dump()<<"\n";std::cout<<"METIS_OK vertices="<<n<<" cut="<<cut<<"\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}
}
