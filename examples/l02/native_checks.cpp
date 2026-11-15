// Direct calls to frozen production helpers. No ROS/Gazebo process is started.
#include "../../baseline/source/optimization_v2/ws/src/task_platform/r10_backend/precision_progress.hpp"
#include "../../baseline/source/optimization_v2/ws/src/task_platform/r10_backend/correction_filter.hpp"
#include "../../baseline/l02_source/optimization_v2/ws/src/task_platform/r09_backend/self_filter.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char* reason) {
    if (!condition) throw std::runtime_error(reason);
}

int main() {
    try {
        using J=nlohmann::json;
        J cfg={{"progress_delta_m",.1},{"precision_progress_policy",true}};
        J dock={{"motion_scope","VALIDATED_LOCAL_DOCKING"},{"expected_translation_m",.036}};
        require(std::abs(r10::progressDelta(cfg,dock)-.009)<1e-12,"precision scale");
        dock["motion_scope"]="NORMAL_NAVIGATION";
        require(r10::progressDelta(cfg,dock)==.1,"scope guard");
        dock["motion_scope"]="VALIDATED_LOCAL_DOCKING";
        dock["expected_translation_m"]=std::numeric_limits<double>::quiet_NaN();
        require(r10::progressDelta(cfg,dock)==.1,"finite guard");
        Eigen::Isometry3d previous=Eigen::Isometry3d::Identity();
        Eigen::Isometry3d observed=previous;
        observed.translation().x()=1.;
        const auto filtered=r10::smoothCorrection(previous,observed,.1,1.);
        require(std::abs(filtered.translation().x()-(1.-std::exp(-.1)))<1e-12,"filter alpha");
        require(r10::smoothCorrection(previous,observed,0.,1.).translation().norm()==0.,"nonpositive dt");
        bool rejected=false;
        try { (void)r10::smoothCorrection(previous,observed,.1,0.); }
        catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"invalid tau");
        const std::array<double,4> bounds={-1.,1.,-1.,1.};
        require(r09::selfReturn(0.,0.,.1,bounds,.2,1.),"physical body filtered");
        require(!r09::selfReturn(.9,0.,.02,bounds,.2,1.),"low annulus preserved");
        require(r09::selfReturn(.9,0.,.2,bounds,.2,1.),"high annulus filtered");
        std::cout<<"PASS 9 direct production-helper checks\n"
                 <<"filter_translation_x="<<filtered.translation().x()<<" m\n"
                 <<"Scope: finite/scope guards, correction interpolation and self-return classification.\n"
                 <<"No physical safety, ROS lifecycle or full mission claim.\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 2;
    }
}
