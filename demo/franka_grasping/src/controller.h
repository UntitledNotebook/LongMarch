/**
 * @file controller.h
 * @brief PD controller and physics simulation
 *
 * Implements a PD controller with gravity compensation and
 * forward dynamics simulation using the Articulated Body Algorithm.
 */

#pragma once

#include "types.h"
#include "config.h"
#include "contradium/rbd/rbd_dynamics.h"
#include <boost/numeric/odeint.hpp>

namespace franka::controller {

// Alias for contradium::rbd types
using namespace contradium::rbd;

/**
 * @brief PD controller with gravity compensation
 *
 * Computes control torques using:
 *   tau = Kp * (q_des - q) + Kd * (qdot_des - qdot) + tau_gravity
 *
 * Uses separate gains for base joints (1-4) and wrist joints (5-7)
 * to prevent chatter on low-inertia distal links.
 */
class PDController {
public:
  /**
   * @brief Construct PD controller with per-joint-group configs
   * @param arm_base_config Configuration for base/elbow joints (0-3)
   * @param arm_wrist_config Configuration for wrist joints (4-6)
   * @param gripper_config Configuration for gripper joints (7+)
   * @param num_arm_joints Number of arm joints (default 7)
   */
  PDController(
      const ControllerConfig& arm_base_config,
      const ControllerConfig& arm_wrist_config,
      const ControllerConfig& gripper_config,
      int num_arm_joints = 7);

  /**
   * @brief Compute control torques
   *
   * @param model RBD model (for gravity compensation)
   * @param Q_current Current joint positions
   * @param QDot_current Current joint velocities
   * @param Q_desired Desired joint positions
   * @param QDot_desired Desired joint velocities
   * @param Tau Output torques
   */
  void computeTorques(
      Model& model,
      const VectorX& Q_current,
      const VectorX& QDot_current,
      const VectorX& Q_desired,
      const VectorX& QDot_desired,
      VectorX& Tau);

  /**
   * @brief Set gains for base arm joints (0-3)
   */
  void setArmBaseGains(double kp, double kd) {
    arm_base_config_.kp = kp;
    arm_base_config_.kd = kd;
  }

  /**
   * @brief Set gains for wrist arm joints (4-6)
   */
  void setArmWristGains(double kp, double kd) {
    arm_wrist_config_.kp = kp;
    arm_wrist_config_.kd = kd;
  }

  /**
   * @brief Set gains for gripper joints
   */
  void setGripperGains(double kp, double kd) {
    gripper_config_.kp = kp;
    gripper_config_.kd = kd;
  }

  /**
   * @brief Reset velocity filter state
   */
  void resetFilter(int dof_count);

private:
  /// Get config for joint based on index
  const ControllerConfig& getConfigForJoint(int joint_index) const;

  ControllerConfig arm_base_config_;   ///< Joints 0-3 (shoulder/elbow)
  ControllerConfig arm_wrist_config_;  ///< Joints 4-6 (wrist)
  ControllerConfig gripper_config_;    ///< Joints 7+ (gripper)
  int num_arm_joints_;
  int num_base_joints_ = 4;  ///< Joints 0-3 are "base"

  /// Filtered velocity for derivative term
  VectorX qdot_filtered_;
  bool filter_initialized_ = false;
};

/**
 * @brief Robot physics simulator
 *
 * Wraps forward dynamics simulation with integration.
 */
class RobotSimulator {
public:
  /**
   * @brief Construct simulator
   * @param model RBD model
   * @param controller PD controller
   * @param dt Simulation timestep
   */
  RobotSimulator(Model& model, PDController& controller, double dt);

  /**
   * @brief Step simulation forward
   *
   * @param Q_desired Desired joint positions
   * @param QDot_desired Desired joint velocities
   * @param Q Current joint positions (updated in place)
   * @param QDot Current joint velocities (updated in place)
   * @return true if step was successful (no NaNs)
   */
  bool step(
      const VectorX& Q_desired,
      const VectorX& QDot_desired,
      VectorX& Q,
      VectorX& QDot);

  /**
   * @brief Step simulation with multiple substeps
   *
   * @param num_substeps Number of substeps to take
   * @return true if all steps were successful
   */
  bool stepSubsteps(
      const VectorX& Q_desired,
      const VectorX& QDot_desired,
      VectorX& Q,
      VectorX& QDot,
      int num_substeps);

  /**
   * @brief Get current torques (from last step)
   */
  const VectorX& getTorques() const { return Tau_; }

private:
  Model& model_;
  PDController& controller_;
  double dt_;
  VectorX Tau_;  // Computed torques

  using StateVector = std::vector<double>;

  /**
   * @brief System dynamics for ODE integrator
   */
  struct SystemDynamics {
    Model& model;
    const VectorX& Tau;

    void operator()(const StateVector& state, StateVector& dstate, double /*t*/);
  };
};

} // namespace franka::controller
