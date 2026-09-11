#pragma once
#include <cstdint>
#include <cmath>
#include <string>
#include <Eigen/Geometry>

namespace pnp7_policy {
constexpr double kTrackingPositionLimit = .09;
constexpr double kTrackingRotationLimit = .60;
inline bool fresh(int64_t received, int64_t started, int64_t now) {
  return received > started && now >= received && now - received < 250000000;
}
inline bool close(const Eigen::Vector3d& target, const Eigen::Quaterniond& target_r,
                  const Eigen::Vector3d& measured, const Eigen::Quaterniond& measured_r) {
  return (target - measured).norm() <= kTrackingPositionLimit &&
         measured_r.angularDistance(target_r) <= kTrackingRotationLimit;
}
inline bool valid(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation,
                  double age, const std::string& frame) {
  return frame == "fr3_link0" && position.allFinite() && rotation.coeffs().allFinite() &&
         rotation.norm() >= .99 && rotation.norm() <= 1.01 &&
         std::isfinite(age) && age >= -.05 && age <= .10;
}
}  // namespace pnp7_policy
