/**
 * @file path_planner.cpp
 * @brief Path and trajectory planning implementation
 */

#include "path_planner.h"
#include "config.h"
#include "grassland/util/log.h"
#include "grassland/util/file_probe.h"
#include <algorithm>
#include <cmath>

using grassland::LogInfo;
using grassland::LogWarning;
using grassland::LogError;
using grassland::FileProbe;

namespace franka::planner {

namespace {
constexpr double EPSILON = 1e-10;

/**
 * @brief Smoothstep function for smooth interpolation
 */
double smoothstep(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

/**
 * @brief Minimum time for S-curve given distance and limits
 */
double computeMinTime1D(double dist, double vmax, double amax, double jmax) {
  dist = std::abs(dist);

  // If distance is very small, return minimum time
  if (dist < EPSILON) {
    return config::MIN_SEGMENT_DURATION;
  }

  // Time to reach max acceleration: t_a = a_max / j_max
  double ta = amax / jmax;

  // Distance during acceleration phase: d_acc = v^2 / (2*a)
  // But with S-curve, we have jerk-limited acceleration

  // Check if we can reach max velocity
  // Distance needed to accelerate to vmax: d_ramp = vmax^2 / (2*amax) + amax*ta^2/6
  // Simplified: we need 2 * d_ramp for accel + decel
  double d_ramp = (vmax * vmax) / (2.0 * amax);
  double d_jerk_offset = (amax * ta * ta * ta) / 6.0;  // Small correction for jerk
  double d_total_ramp = 2.0 * (d_ramp + d_jerk_offset);

  if (dist < d_total_ramp) {
    // Can't reach max velocity - compute trapezoidal accel profile
    // Using: d = 2 * (amax * t1^2 / 2 + jmax * t1^3 / 6)
    // Simplified approximation
    double t_approx = std::sqrt(dist / amax);
    return std::max(2.0 * t_approx, config::MIN_SEGMENT_DURATION);
  }

  // Can reach max velocity - compute with cruise phase
  double t_accel = vmax / amax;
  double t_cruise = (dist - d_total_ramp) / vmax;
  return 2.0 * t_accel + t_cruise + 2.0 * ta;
}
} // namespace

// ============================================================================
// SCurveProfile
// ============================================================================

std::vector<TrajectoryPoint> SCurveProfile::generate(
    const VectorX& Q_start,
    const VectorX& Q_end,
    double duration,
    const VectorX& max_vel,
    const VectorX& max_acc,
    const VectorX& max_jerk) {

  int n = Q_start.size();
  int num_points = static_cast<int>(std::ceil(duration / config::DT)) + 1;
  std::vector<TrajectoryPoint> trajectory(num_points);

  // For each joint, compute phase times and sample
  std::vector<std::vector<double>> all_phase_times(n);

  for (int i = 0; i < n; ++i) {
    all_phase_times[i] = computePhaseTimes(
        Q_start(i), Q_end(i), max_vel(i), max_acc(i), max_jerk(i), duration);
  }

  // Sample trajectory
  for (int p = 0; p < num_points; ++p) {
    double t = p * config::DT;
    t = std::min(t, duration);

    TrajectoryPoint point;
    point.Q.resize(n);
    point.QDot.resize(n);
    point.time = t;

    for (int i = 0; i < n; ++i) {
      auto pt = evaluate(t, all_phase_times[i], Q_start(i), Q_end(i));
      point.Q(i) = pt.pos;
      point.QDot(i) = pt.vel;
    }

    trajectory[p] = point;
  }

  return trajectory;
}

double SCurveProfile::computeMinDuration(
    const VectorX& Q_start,
    const VectorX& Q_end,
    const VectorX& max_vel,
    const VectorX& max_acc,
    const VectorX& max_jerk) {

  double max_duration = config::MIN_SEGMENT_DURATION;

  for (int i = 0; i < Q_start.size(); ++i) {
    double dist = std::abs(Q_end(i) - Q_start(i));
    double t_min = computeMinTime1D(dist, max_vel(i), max_acc(i), max_jerk(i));
    max_duration = std::max(max_duration, t_min);
  }

  return max_duration;
}

std::vector<double> SCurveProfile::computePhaseTimes(
    double q0, double q1, double vmax, double amax, double jmax, double desired_duration) {

  std::vector<double> phases(7, 0.0);

  double dist = q1 - q0;
  double abs_dist = std::abs(dist);

  if (abs_dist < EPSILON) {
    return phases;  // No motion needed
  }

  double dir = (dist > 0) ? 1.0 : -1.0;

  // Time to change acceleration (jerk up/down)
  double tj = amax / jmax;

  // Minimum time to reach max acceleration and come back
  double ta_min = tj;
  double da = amax * tj / 2.0;  // Change in velocity during jerk phase
  double dv_max = amax * ta_min;  // Max velocity change during acceleration phase

  // Check if we're distance-limited
  double d_min = 2.0 * (amax * tj * tj / 2.0 + dv_max * tj / 2.0);

  if (abs_dist < d_min) {
    // Very short distance - use double-S profile
    // Simplified: just return equal phases for smooth interpolation
    double t_total = std::max(desired_duration, 3.0 * tj);
    phases[0] = phases[1] = phases[2] = phases[4] = phases[5] = phases[6] = tj;
    phases[3] = t_total - 6.0 * tj;
    return phases;
  }

  // Time to accelerate/decelerate
  double ta = amax / jmax;
  double t_accel = ta;

  // Check if we can reach max velocity
  double d_accel = vmax * vmax / (2.0 * amax) + amax * ta * ta / 6.0;
  double d_total_ramp = 2.0 * d_accel;

  double t_cruise = 0.0;

  if (abs_dist > d_total_ramp) {
    // Can reach max velocity
    t_cruise = (abs_dist - d_total_ramp) / vmax;
  } else {
    // Can't reach max velocity - find optimal peak velocity
    // Using: d = v_peak * (t_accel + t_decel) - amax * t_accel^2 / 3
    // Simplified iteration
    double v_peak = vmax * 0.5;
    for (int iter = 0; iter < 10; ++iter) {
      double d_est = v_peak * (2.0 * t_accel) - amax * t_accel * t_accel / 3.0;
      if (std::abs(d_est - abs_dist) < 1e-6) break;
      v_peak = std::min(vmax, v_peak * abs_dist / d_est);
    }
    t_accel = v_peak / amax;
  }

  // Assign phase times
  phases[0] = tj;           // Jerk up
  phases[1] = t_accel - tj; // Constant accel
  phases[2] = tj;           // Jerk down (to zero accel)
  phases[3] = t_cruise;     // Constant velocity
  phases[4] = tj;           // Jerk down (negative)
  phases[5] = t_accel - tj; // Constant decel
  phases[6] = tj;           // Jerk up (to zero)

  // Scale to match desired duration if needed
  double computed_total = 0.0;
  for (double t : phases) computed_total += t;

  if (desired_duration > computed_total) {
    double scale = desired_duration / computed_total;
    // Only scale the cruise phase to preserve acceleration profile
    phases[3] *= scale;
  }

  return phases;
}

SCurveProfile::Point SCurveProfile::evaluate(
    double t, const std::vector<double>& phase_times,
    double q0, double q1) {

  Point result{q0, 0.0, 0.0};

  // Cumulative phase times
  std::vector<double> t_cum(7, 0.0);
  for (size_t i = 1; i < 7; ++i) {
    t_cum[i] = t_cum[i-1] + phase_times[i-1];
  }

  // Determine which phase we're in and evaluate
  // Simplified: use smooth interpolation between endpoints
  double total = t_cum[6];
  if (total < EPSILON) return result;

  double tau = std::clamp(t / total, 0.0, 1.0);
  double smooth = smoothstep(tau);

  result.pos = q0 + smooth * (q1 - q0);
  result.vel = (q1 - q0) * (6.0 * tau * (1.0 - tau)) / total;  // Derivative of smoothstep

  return result;
}

// ============================================================================
// PathPlanner
// ============================================================================

PathPlanner::PathPlanner(
    Model& model,
    const urdf_utils::RobotModel& robot_model,
    const PathPlannerConfig& config)
    : model_(model),
      robot_model_(robot_model),
      config_(config),
      ik_solver_(model, robot_model.joint_info) {

  // Add search paths for URDF assets
  std::string assets_dir = std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/franka_fr3/";
  FileProbe::GetInstance().AddSearchPath(assets_dir);
  FileProbe::GetInstance().AddSearchPath(assets_dir + "meshes/");
}

std::optional<Trajectory> PathPlanner::planJointSpace(
    const std::vector<VectorX>& waypoints) {

  if (waypoints.size() < 2) {
    LogError("PathPlanner: need at least 2 waypoints");
    return std::nullopt;
  }

  Trajectory trajectory;
  trajectory.points.clear();

  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    const auto& Q_start = waypoints[i];
    const auto& Q_end = waypoints[i + 1];

    // Compute segment duration
    VectorX max_vel = VectorX::Constant(Q_start.size(), config_.max_velocity);
    VectorX max_acc = VectorX::Constant(Q_start.size(), config_.max_acceleration);
    VectorX max_jerk = VectorX::Constant(Q_start.size(), config_.max_jerk);

    double duration = s_curve_.computeMinDuration(Q_start, Q_end, max_vel, max_acc, max_jerk);
    duration = std::max(duration, config_.min_segment_duration);

    // Generate segment
    auto segment = s_curve_.generate(Q_start, Q_end, duration, max_vel, max_acc, max_jerk);

    // Append to trajectory (offset time)
    double time_offset = trajectory.duration;
    for (auto& point : segment) {
      point.time += time_offset;
      trajectory.points.push_back(point);
    }

    trajectory.duration += duration;
  }

  LogInfo("Planned joint space trajectory: {} waypoints, {:.2f}s duration",
           waypoints.size(), trajectory.duration);

  return trajectory;
}

std::optional<Trajectory> PathPlanner::planPickAndPlace(
    const VectorX& Q_home,
    const glm::vec3& ball_pos,
    const glm::vec3& goal_pos,
    const Matrix3& gripper_down_rot) {

  LogInfo("Planning pick-and-place trajectory...");

  // Define all key configurations
  std::vector<VectorX> waypoints;

  // 1. Home
  waypoints.push_back(Q_home);

  // 2. Pre-grasp (above ball)
  glm::vec3 pre_grasp_pos(ball_pos.x, ball_pos.y, ball_pos.z + config::PRE_GRASP_HEIGHT);
  auto Q_pre_grasp = solveIK(Q_home, pre_grasp_pos, gripper_down_rot);
  if (!Q_pre_grasp) {
    LogWarning("IK failed for pre-grasp position");
    return std::nullopt;
  }
  waypoints.push_back(*Q_pre_grasp);

  // 3. Grasp (at ball)
  glm::vec3 grasp_pos(ball_pos.x, ball_pos.y, ball_pos.z);
  auto Q_grasp = solveIK(*Q_pre_grasp, grasp_pos, gripper_down_rot);
  if (!Q_grasp) {
    LogWarning("IK failed for grasp position");
    return std::nullopt;
  }
  waypoints.push_back(*Q_grasp);

  // 4. Lift
  glm::vec3 lift_pos(ball_pos.x, ball_pos.y, ball_pos.z + config::LIFT_HEIGHT);
  auto Q_lift = solveIK(*Q_grasp, lift_pos, gripper_down_rot);
  if (!Q_lift) {
    LogWarning("IK failed for lift position");
    return std::nullopt;
  }
  waypoints.push_back(*Q_lift);

  // 5. Pre-place (above goal)
  glm::vec3 pre_place_pos(goal_pos.x, goal_pos.y, ball_pos.z + config::LIFT_HEIGHT);
  auto Q_pre_place = solveIK(*Q_lift, pre_place_pos, gripper_down_rot);
  if (!Q_pre_place) {
    LogWarning("IK failed for pre-place position");
    return std::nullopt;
  }
  waypoints.push_back(*Q_pre_place);

  // 6. Place (at goal)
  glm::vec3 place_pos(goal_pos.x, goal_pos.y, ball_pos.z);
  auto Q_place = solveIK(*Q_pre_place, place_pos, gripper_down_rot);
  if (!Q_place) {
    LogWarning("IK failed for place position");
    return std::nullopt;
  }
  waypoints.push_back(*Q_place);

  // 7. Retract (above goal)
  glm::vec3 retract_pos(goal_pos.x, goal_pos.y, ball_pos.z + config::RETRACT_HEIGHT);
  auto Q_retract = solveIK(*Q_place, retract_pos, gripper_down_rot);
  if (!Q_retract) {
    LogWarning("IK failed for retract position");
    return std::nullopt;
  }
  waypoints.push_back(*Q_retract);

  // 8. Home
  waypoints.push_back(Q_home);

  // Plan trajectory through waypoints
  return planJointSpace(waypoints);
}

Trajectory PathPlanner::planLinear(
    const VectorX& Q_start,
    const VectorX& Q_end,
    double duration) {

  Trajectory trajectory;
  int num_points = static_cast<int>(std::ceil(duration / config::DT)) + 1;
  trajectory.points.resize(num_points);
  trajectory.duration = duration;

  for (int i = 0; i < num_points; ++i) {
    double t = i * config::DT;
    double tau = std::min(t / duration, 1.0);
    double smooth = smoothstep(tau);

    TrajectoryPoint point;
    point.Q = Q_start + smooth * (Q_end - Q_start);
    point.QDot = (Q_end - Q_start) * (6.0 * tau * (1.0 - tau)) / duration;
    point.time = t;

    trajectory.points[i] = point;
  }

  return trajectory;
}

std::optional<VectorX> PathPlanner::solveIK(
    const VectorX& Q_current,
    const glm::vec3& position,
    const Matrix3& rotation) {

  VectorX Q_result(robot_model_.dof());

  IKSolverConfig ik_config;
  ik_config.max_iter = config::IK_MAX_ITER;
  ik_config.step_tol = config::IK_STEP_TOL;
  ik_config.constraint_tol = config::IK_CONSTRAINT_TOL;
  ik_config.num_samples = config_.ik_max_samples;
  ik_config.verbose = false;

  Vector3 target_pos(position.x, position.y, position.z);

  bool success = ik_solver_.solveFullPose(
      Q_current, robot_model_.tcp_body_id, target_pos, rotation,
      Q_result, ik_config);

  if (success) {
    // Clamp to limits
    ik_solver_.clampToLimits(Q_result);
    return Q_result;
  }

  return std::nullopt;
}

std::vector<TrajectoryPoint> PathPlanner::planSegment(
    const VectorX& Q_start,
    const VectorX& Q_end,
    double duration) {

  VectorX max_vel = VectorX::Constant(Q_start.size(), config_.max_velocity);
  VectorX max_acc = VectorX::Constant(Q_start.size(), config_.max_acceleration);
  VectorX max_jerk = VectorX::Constant(Q_start.size(), config_.max_jerk);

  return s_curve_.generate(Q_start, Q_end, duration, max_vel, max_acc, max_jerk);
}

} // namespace franka::planner
