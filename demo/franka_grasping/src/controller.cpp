/**
 * @file controller.cpp
 * @brief PD controller and physics simulation implementation
 */

#include "controller.h"
#include "grassland/util/log.h"
#include <algorithm>
#include <cmath>

using grassland::LogWarning;
using grassland::LogError;

namespace franka::controller {

// ============================================================================
// PDController
// ============================================================================

PDController::PDController(
    const ControllerConfig& arm_base_config,
    const ControllerConfig& arm_wrist_config,
    const ControllerConfig& gripper_config,
    int num_arm_joints)
    : arm_base_config_(arm_base_config),
      arm_wrist_config_(arm_wrist_config),
      gripper_config_(gripper_config),
      num_arm_joints_(num_arm_joints) {
}

const ControllerConfig& PDController::getConfigForJoint(int joint_index) const {
  if (joint_index >= num_arm_joints_) {
    return gripper_config_;  // Gripper joints
  } else if (joint_index < num_base_joints_) {
    return arm_base_config_;  // Base/elbow joints 0-3
  } else {
    return arm_wrist_config_;  // Wrist joints 4-6
  }
}

void PDController::resetFilter(int dof_count) {
  qdot_filtered_.resize(dof_count);
  qdot_filtered_.setZero();
  filter_initialized_ = false;
}

void PDController::computeTorques(
    Model& model,
    const VectorX& Q_current,
    const VectorX& QDot_current,
    const VectorX& Q_desired,
    const VectorX& QDot_desired,
    VectorX& Tau) {

  int n = model.dof_count;

  // Initialize velocity filter if needed
  if (!filter_initialized_ || qdot_filtered_.size() != n) {
    qdot_filtered_ = QDot_current;
    filter_initialized_ = true;
  }

  // Apply low-pass filter to velocity for derivative term
  // qdot_filtered = alpha * qdot_current + (1 - alpha) * qdot_filtered_prev
  const double alpha = config::VELOCITY_FILTER_ALPHA;
  for (int i = 0; i < n; ++i) {
    qdot_filtered_(i) = alpha * QDot_current(i) + (1.0 - alpha) * qdot_filtered_(i);
  }

  // Compute gravity compensation using inverse dynamics
  // with zero velocity and zero acceleration
  VectorX QDot_zero = VectorX::Zero(n);
  VectorX QDDot_zero = VectorX::Zero(n);
  VectorX Tau_gravity(n);

  InverseDynamics(model, Q_current, QDot_zero, QDDot_zero, Tau_gravity);

  // Compute PD control + gravity compensation
  for (int i = 0; i < Tau.size(); ++i) {
    const ControllerConfig& cfg = getConfigForJoint(i);

    double q_error = Q_desired(i) - Q_current(i);
    // Use filtered velocity for derivative term to reduce chatter
    double qdot_error = QDot_desired(i) - qdot_filtered_(i);

    // PD control law
    double tau_pd = cfg.kp * q_error + cfg.kd * qdot_error;

    // Add gravity compensation and saturate
    double tau_raw = tau_pd + Tau_gravity(i);
    Tau(i) = std::clamp(tau_raw, -cfg.max_torque, cfg.max_torque);

    // Log large torques (only occasionally to avoid spam)
    if (std::abs(Tau(i)) > config::TORQUE_WARNING_THRESHOLD &&
        std::abs(Tau(i)) > 0.9 * cfg.max_torque) {
      LogWarning("Joint {}: torque = {:.2f} Nm (limit: {:.2f})", i, Tau(i), cfg.max_torque);
    }
  }
}

// ============================================================================
// RobotSimulator
// ============================================================================

RobotSimulator::RobotSimulator(Model& model, PDController& controller, double dt)
    : model_(model),
      controller_(controller),
      dt_(dt) {

  Tau_.resize(model_.dof_count);
  Tau_.setZero();
}

bool RobotSimulator::step(
    const VectorX& Q_desired,
    const VectorX& QDot_desired,
    VectorX& Q,
    VectorX& QDot) {

  // Check inputs for NaNs
  if (Q.hasNaN() || QDot.hasNaN()) {
    LogError("RobotSimulator: NaN in input state");
    return false;
  }

  // Compute control torques
  controller_.computeTorques(model_, Q, QDot, Q_desired, QDot_desired, Tau_);

  if (Tau_.hasNaN()) {
    LogError("RobotSimulator: NaN in computed torques");
    return false;
  }

  // Pack state for integrator: [Q, QDot]
  int n = model_.dof_count;
  SystemDynamics dynamics{model_, Tau_};

  std::vector<double> state(2 * n);
  for (int i = 0; i < n; ++i) {
    state[i] = Q(i);
    state[n + i] = QDot(i);
  }

  // Integrate using Runge-Kutta 4th order
  using namespace boost::numeric::odeint;
  runge_kutta4<std::vector<double>> stepper;
  stepper.do_step(dynamics, state, 0.0, dt_);

  // Unpack state with velocity clamping
  for (int i = 0; i < n; ++i) {
    Q(i) = state[i];
    QDot(i) = std::clamp(state[n + i], -config::MAX_VELOCITY, config::MAX_VELOCITY);
  }

  // Check outputs for NaNs
  if (Q.hasNaN() || QDot.hasNaN()) {
    LogError("RobotSimulator: NaN in output state after integration");
    return false;
  }

  return true;
}

bool RobotSimulator::stepSubsteps(
    const VectorX& Q_desired,
    const VectorX& QDot_desired,
    VectorX& Q,
    VectorX& QDot,
    int num_substeps) {

  for (int i = 0; i < num_substeps; ++i) {
    if (!step(Q_desired, QDot_desired, Q, QDot)) {
      LogError("RobotSimulator: Failed at substep {}", i);
      return false;
    }
  }
  return true;
}

// ============================================================================
// SystemDynamics
// ============================================================================

void RobotSimulator::SystemDynamics::operator()(
    const StateVector& state,
    StateVector& dstate,
    double /*t*/) {

  int n = model.dof_count;

  // Extract Q and QDot from state
  VectorX Q(n);
  VectorX QDot(n);
  for (int i = 0; i < n; ++i) {
    Q(i) = state[i];
    QDot(i) = state[n + i];
  }

  // Compute QDDot using Forward Dynamics
  VectorX QDDot(n);
  QDDot.setZero();
  ForwardDynamics(model, Q, QDot, Tau, QDDot);

  // Check for NaNs in QDDot
  if (QDDot.hasNaN()) {
    LogError("SystemDynamics: NaN in QDDot");
    LogError("  Q: [{:.3f}, {:.3f}, ...]", Q(0), Q(1));
    LogError("  Tau: [{:.2f}, {:.2f}, ...]", Tau(0), Tau(1));
    QDDot.setZero();
  }

  // Build derivative: dstate = [QDot, QDDot]
  dstate.resize(2 * n);
  for (int i = 0; i < n; ++i) {
    dstate[i] = QDot(i);           // dQ/dt = QDot
    dstate[n + i] = QDDot(i);      // dQDot/dt = QDDot
  }
}

} // namespace franka::controller
