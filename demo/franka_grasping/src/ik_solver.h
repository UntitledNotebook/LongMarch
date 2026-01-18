/**
 * @file ik_solver.h
 * @brief Inverse kinematics solver with sampling strategy
 *
 * Provides a clean interface for solving IK with multiple random restarts
 * to find solutions within joint limits.
 */

#pragma once

#include "types.h"
#include "urdf_utils.h"
#include "contradium/rbd/rbd_kinematics.h"
#include <random>
#include <vector>

namespace franka::ik {

// Alias for convenience
using namespace contradium::rbd;

/**
 * @brief IK solver with sampling for finding valid joint configurations
 *
 * Uses random restarts to find IK solutions that are within joint limits
 * and close to the current configuration.
 */
class IKSolver {
public:
  /**
   * @brief Construct IK solver
   * @param model RBD model
   * @param joints Joint information (for limits and names)
   */
  IKSolver(Model& model, const std::vector<JointInfo>& joints);

  /**
   * @brief Solve full 6-DOF pose constraint with sampling
   *
   * Tries the current configuration first, then samples random configurations
   * within joint limits to find valid IK solutions.
   *
   * @param Q_current Current joint configuration
   * @param tcp_body_id Body ID of the TCP link
   * @param target_pos Desired position
   * @param target_rot Desired rotation (body-to-world)
   * @param Q_result Output configuration
   * @param config Solver configuration
   * @return true if a valid solution was found
   */
  bool solveFullPose(
      const VectorX& Q_current,
      unsigned int tcp_body_id,
      const Vector3& target_pos,
      const Matrix3& target_rot,
      VectorX& Q_result,
      const IKSolverConfig& config = IKSolverConfig());

  /**
   * @brief Solve position-only constraint with sampling
   *
   * @param Q_current Current joint configuration
   * @param tcp_body_id Body ID of the TCP link
   * @param target_pos Desired position
   * @param Q_result Output configuration
   * @param config Solver configuration
   * @return true if a valid solution was found
   */
  bool solvePosition(
      const VectorX& Q_current,
      unsigned int tcp_body_id,
      const Vector3& target_pos,
      VectorX& Q_result,
      const IKSolverConfig& config = IKSolverConfig());

  /**
   * @brief Check if configuration is within joint limits
   */
  bool isWithinLimits(const VectorX& Q) const;

  /**
   * @brief Clamp configuration to joint limits (in-place)
   * @return true if any value was clamped
   */
  bool clampToLimits(VectorX& Q) const;

  /**
   * @brief Get result from last solve
   */
  const IKResult& getLastResult() const { return last_result_; }

private:
  /**
   * @brief Single IK solve attempt from a given initial configuration
   */
  IKResult solveSingle(
      const VectorX& Q_init,
      unsigned int tcp_body_id,
      const Vector3& target_pos,
      const Matrix3* target_rot,
      const IKSolverConfig& config);

  /**
   * @brief Generate random configuration within joint limits
   */
  VectorX generateRandomConfiguration() const;

  /**
   * @brief Compute weighted distance between two configurations
   *
   * Uses joint limits to normalize each dimension.
   */
  double configurationDistance(const VectorX& Q1, const VectorX& Q2) const;

  Model& model_;
  VectorX position_lower_;
  VectorX position_upper_;
  VectorX position_range_;  // For normalized distance computation
  mutable std::mt19937 rng_;
  IKResult last_result_;
};

} // namespace franka::ik
