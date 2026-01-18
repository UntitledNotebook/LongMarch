/**
 * @file path_planner.h
 * @brief Path and trajectory planning with S-curve profiles
 *
 * Implements industrial-standard trajectory generation using jerk-limited
 * S-curve profiles for smooth robot motion.
 */

#pragma once

#include "types.h"
#include "urdf_utils.h"
#include "ik_solver.h"
#include <vector>
#include <optional>

namespace franka::planner {

// Alias for contradium::rbd types
using namespace contradium::rbd;

/**
 * @brief S-curve trajectory generator
 *
 * Generates smooth trajectories with limited jerk (rate of change of acceleration).
 * This produces more natural motion and reduces wear on robot joints.
 */
class SCurveProfile {
public:
  /**
   * @brief Generate S-curve trajectory between two configurations
   *
   * @param Q_start Start configuration
   * @param Q_end End configuration
   * @param duration Desired duration (will be adjusted if needed)
   * @param max_vel Maximum velocity per joint
   * @param max_acc Maximum acceleration per joint
   * @param max_jerk Maximum jerk per joint
   * @return Vector of trajectory points
   */
  std::vector<TrajectoryPoint> generate(
      const VectorX& Q_start,
      const VectorX& Q_end,
      double duration,
      const VectorX& max_vel,
      const VectorX& max_acc,
      const VectorX& max_jerk);

  /**
   * @brief Compute minimum duration given limits
   */
  double computeMinDuration(
      const VectorX& Q_start,
      const VectorX& Q_end,
      const VectorX& max_vel,
      const VectorX& max_acc,
      const VectorX& max_jerk);

private:
  /**
   * @brief Compute S-curve phase times for a single dimension
   *
   * Returns the 7 phase times: [t1, t2, t3, t4, t5, t6, t7]
   * where t1=t7 (jerk up), t2=t6 (accel constant), t3=t5 (jerk down), t4 (cruise)
   */
  std::vector<double> computePhaseTimes(
      double q0, double q1, double vmax, double amax, double jmax, double desired_duration);

  /**
   * @brief Evaluate S-curve position, velocity, acceleration at time t
   */
  struct Point {
    double pos, vel, acc;
  };
  Point evaluate(double t, const std::vector<double>& phase_times,
                 double q0, double q1);
};

/**
 * @brief Path planner for robot trajectories
 *
 * Plans smooth trajectories through waypoints using IK and S-curve profiles.
 */
class PathPlanner {
public:
  /**
   * @brief Construct path planner
   * @param model RBD model
   * @param robot_model Complete robot model with joint info
   * @param config Planning configuration
   */
  PathPlanner(
      Model& model,
      const urdf_utils::RobotModel& robot_model,
      const PathPlannerConfig& config = PathPlannerConfig());

  /**
   * @brief Plan trajectory through joint space waypoints
   *
   * @param waypoints Joint space waypoints
   * @return Trajectory if planning successful
   */
  std::optional<Trajectory> planJointSpace(
      const std::vector<VectorX>& waypoints);

  /**
   * @brief Plan pick-and-place trajectory
   *
   * Generates a complete pick-and-place trajectory:
   * - Home -> Pre-grasp -> Grasp -> Lift -> Pre-place -> Place -> Retract -> Home
   *
   * @param Q_home Home configuration
   * @param ball_pos Ball position (x, y, z)
   * @param goal_pos Goal position (x, y)
   * @param gripper_down_rot Orientation for grasping
   * @return Trajectory if planning successful
   */
  std::optional<Trajectory> planPickAndPlace(
      const VectorX& Q_home,
      const glm::vec3& ball_pos,
      const glm::vec3& goal_pos,
      const Matrix3& gripper_down_rot);

  /**
   * @brief Plan simple linear interpolation trajectory (fallback)
   */
  Trajectory planLinear(
      const VectorX& Q_start,
      const VectorX& Q_end,
      double duration);

  /**
   * @brief Get the IK solver (for external use)
   */
  ik::IKSolver& getIKSolver() { return ik_solver_; }

private:
  /**
   * @brief Solve IK for a Cartesian target with retries
   */
  std::optional<VectorX> solveIK(
      const VectorX& Q_current,
      const glm::vec3& position,
      const Matrix3& rotation);

  /**
   * @brief Plan trajectory segment between two configurations
   */
  std::vector<TrajectoryPoint> planSegment(
      const VectorX& Q_start,
      const VectorX& Q_end,
      double duration);

  Model& model_;
  const urdf_utils::RobotModel& robot_model_;
  PathPlannerConfig config_;
  ik::IKSolver ik_solver_;
  SCurveProfile s_curve_;
};

} // namespace franka::planner
