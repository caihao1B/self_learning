#pragma once
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
namespace r10 {
// Match progress scale to an explicitly validated, bounded precision motion.
// The 10 s timeout, emergency priority and original work admission stay intact.
inline double progressDelta(const nlohmann::json& cfg,const nlohmann::json& e) {
  const double coarse=cfg.at("progress_delta_m").get<double>();
  if (!cfg.contains("precision_progress_policy") ||
      e.value("motion_scope",std::string())!="VALIDATED_LOCAL_DOCKING" ||
      !e.contains("expected_translation_m") || !e["expected_translation_m"].is_number()) return coarse;
  const double distance=e["expected_translation_m"].get<double>();
  if (!std::isfinite(distance) || distance<=.008 || distance>.15) return coarse;
  return std::min(coarse,std::max(.005,.25*distance));
}
}
