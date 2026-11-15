#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace r10 {
// Filter only the map->odom correction. Local odometry retains its full rate.
// Caller must check RAW innovation, jumps, freshness and plan holds first.
inline Eigen::Isometry3d smoothCorrection(const Eigen::Isometry3d &previous,
                                         const Eigen::Isometry3d &observed,
                                         double dt, double tau) {
  if (!previous.matrix().allFinite() || !observed.matrix().allFinite() ||
      !std::isfinite(dt) || !std::isfinite(tau) || tau <= 0.)
    throw std::invalid_argument("invalid correction filter input");
  if (dt <= 0.) return previous;
  const double alpha = -std::expm1(-dt / tau);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = previous.translation() + alpha * (observed.translation() - previous.translation());
  result.linear() = Eigen::Quaterniond(previous.linear()).slerp(alpha, Eigen::Quaterniond(observed.linear())).normalized().toRotationMatrix();
  return result;
}
}
