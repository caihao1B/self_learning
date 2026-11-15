// Teaching invocation of the exact frozen production header, not a second controller.
#include "../baseline/source/optimization_v2/ws/src/task_platform/r10_backend/precision_progress.hpp"
#include <iomanip>
#include <iostream>

int main() {
    nlohmann::json cfg = {{"progress_delta_m", 0.10}, {"precision_progress_policy", true}};
    nlohmann::json ordinary = {{"motion_scope", "NORMAL_NAVIGATION"}};
    nlohmann::json precision = {{"motion_scope", "VALIDATED_LOCAL_DOCKING"},
                                {"expected_translation_m", 0.036}};
    nlohmann::json invalid = {{"motion_scope", "VALIDATED_LOCAL_DOCKING"},
                              {"expected_translation_m", 0.20}};
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "ordinary=" << r10::progressDelta(cfg, ordinary) << " m\n";
    std::cout << "precision=" << r10::progressDelta(cfg, precision) << " m\n";
    std::cout << "out_of_range=" << r10::progressDelta(cfg, invalid) << " m\n";
    std::cout << "Scope: pure policy calculation; no BT timeout, ROS or physical motion run.\n";
}
