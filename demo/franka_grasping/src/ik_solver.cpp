/**
 * @file ik_solver.cpp
 * @brief Inverse kinematics solver implementation
 */

#include "ik_solver.h"
#include "config.h"
#include "grassland/util/log.h"
#include <limits>
#include <algorithm>

using grassland::LogInfo;
using grassland::LogWarning;

namespace franka::ik {

IKSolver::IKSolver(Model& model, const std::vector<JointInfo>& joints)
    : model_(model), rng_(std::random_device{}()) {

  int n = model_.dof_count;
  position_lower_.resize(n);
  position_upper_.resize(n);
  position_range_.resize(n);

  for (int i = 0; i < n && i < static_cast<int>(joints.size()); ++i) {
    position_lower_(i) = joints[i].lower_bound;
    position_upper_(i) = joints[i].upper_bound;
    position_range_(i) = joints[i].upper_bound - joints[i].lower_bound;

    // Avoid division by zero
    if (position_range_(i) < 1e-6) {
      position_range_(i) = 1.0;
    }
  }
}

bool IKSolver::solveFullPose(
    const VectorX& Q_current,
    unsigned int tcp_body_id,
    const Vector3& target_pos,
    const Matrix3& target_rot,
    VectorX& Q_result,
    const IKSolverConfig& config) {

  std::vector<IKResult> valid_solutions;

  // First try with current configuration
  IKResult result = solveSingle(Q_current, tcp_body_id, target_pos, &target_rot, config);

  if (result.success && result.within_limits) {
    Q_result = result.Q;
    last_result_ = result;
    LogInfo("IK: success from current config, error={:.6f}, iter={}",
             result.error_norm, result.iterations);
    return true;
  }

  if (result.success) {
    result.distance_from_start = configurationDistance(Q_current, result.Q);
    valid_solutions.push_back(result);
  }

  // Try random samples
  for (int sample = 0; sample < config.num_samples; ++sample) {
    VectorX Q_init = generateRandomConfiguration();
    IKResult sample_result = solveSingle(Q_init, tcp_body_id, target_pos, &target_rot, config);

    if (sample_result.success) {
      sample_result.distance_from_start = configurationDistance(Q_current, sample_result.Q);

      if (sample_result.within_limits) {
        valid_solutions.push_back(sample_result);
      }
    }
  }

  // Select best solution (closest to current configuration)
  if (!valid_solutions.empty()) {
    IKResult* best_in_limits = nullptr;
    double min_dist = std::numeric_limits<double>::max();

    for (auto& sol : valid_solutions) {
      if (sol.within_limits && sol.distance_from_start < min_dist) {
        min_dist = sol.distance_from_start;
        best_in_limits = &sol;
      }
    }

    if (best_in_limits != nullptr) {
      Q_result = best_in_limits->Q;
      last_result_ = *best_in_limits;
      LogInfo("IK: success from sample {}, dist={:.3f}, error={:.6f}",
               best_in_limits->iterations, min_dist, best_in_limits->error_norm);
      return true;
    }
  }

  // No valid in-limits solution found
  last_result_.success = false;
  LogWarning("IK: failed to find valid solution after {} attempts", config.num_samples + 1);
  return false;
}

bool IKSolver::solvePosition(
    const VectorX& Q_current,
    unsigned int tcp_body_id,
    const Vector3& target_pos,
    VectorX& Q_result,
    const IKSolverConfig& config) {

  return solveFullPose(Q_current, tcp_body_id, target_pos, Matrix3::Identity(),
                       Q_result, config);
}

bool IKSolver::isWithinLimits(const VectorX& Q) const {
  for (int i = 0; i < Q.size(); ++i) {
    if (Q(i) < position_lower_(i) || Q(i) > position_upper_(i)) {
      return false;
    }
  }
  return true;
}

bool IKSolver::clampToLimits(VectorX& Q) const {
  bool clamped = false;
  for (int i = 0; i < Q.size(); ++i) {
    double old_val = Q(i);
    Q(i) = std::clamp(Q(i), position_lower_(i), position_upper_(i));
    if (std::abs(Q(i) - old_val) > 1e-10) {
      clamped = true;
    }
  }
  return clamped;
}

VectorX IKSolver::generateRandomConfiguration() const {
  VectorX Q(model_.dof_count);
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  for (int i = 0; i < model_.dof_count; ++i) {
    double lower = position_lower_(i);
    double range = position_range_(i);
    Q(i) = lower + dist(rng_) * range;
  }

  return Q;
}

double IKSolver::configurationDistance(const VectorX& Q1, const VectorX& Q2) const {
  // Weighted distance using joint ranges
  VectorX diff = (Q1 - Q2).cwiseQuotient(position_range_);
  return diff.norm();
}

IKResult IKSolver::solveSingle(
    const VectorX& Q_init,
    unsigned int tcp_body_id,
    const Vector3& target_pos,
    const Matrix3* target_rot,
    const IKSolverConfig& config) {

  IKResult result;
  result.Q.resize(model_.dof_count);

  // Setup IK constraint
  ::contradium::rbd::IKConstraint ik_constraint;
  ik_constraint.max_iter = config.max_iter;
  ik_constraint.step_tol = config.step_tol;
  ik_constraint.constraint_tol = config.constraint_tol;

  if (target_rot != nullptr) {
    ik_constraint.AddFullConstraint(tcp_body_id, Vector3(0, 0, 0), target_pos, *target_rot);
  } else {
    ik_constraint.AddPointConstraint(tcp_body_id, Vector3(0, 0, 0), target_pos);
  }

  // Solve IK
  result.success = ::contradium::rbd::InverseKinematics(model_, Q_init, ik_constraint, result.Q);
  result.error_norm = ik_constraint.error_norm;
  result.iterations = ik_constraint.num_iter;

  if (result.success) {
    result.within_limits = isWithinLimits(result.Q);
    result.distance_from_start = configurationDistance(Q_init, result.Q);
  }

  return result;
}

} // namespace franka::ik
