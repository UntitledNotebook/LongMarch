/**
 * @file types.h
 * @brief Common data structures for the Franka grasping demo
 *
 * Defines all shared types and enums used across modules.
 */

#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <optional>
#include "contradium/rbd/rbd_model.h"

namespace franka {

// Forward declaration for visual::CombinedMesh
namespace visual {
class CombinedMesh;
}

// Use contradium::rbd types directly
using RBDVector3 = contradium::rbd::Vector3;
using RBDVectorX = contradium::rbd::VectorX;
using RBDMatrix3 = contradium::rbd::Matrix3;
using RBDModel = contradium::rbd::Model;

// ============================================================================
// Task State Machine
// ============================================================================

/**
 * @brief Stages of the pick-and-place task
 */
enum class TaskStage {
  IDLE,                  // Not running
  MOVE_TO_PRE_GRASP,     // Move to position above ball
  DESCEND_TO_GRASP,      // Lower to ball
  CLOSE_GRIPPER,         // Close gripper on ball
  LIFT_BALL,             // Lift ball up
  MOVE_TO_PRE_PLACE,     // Move to position above goal
  LOWER_TO_PLACE,        // Lower to goal
  OPEN_GRIPPER,          // Open gripper to release
  RETRACT,               // Move up from goal
  RETURN_HOME,           // Return to home configuration
  COMPLETE               // Task finished
};

/**
 * @brief Convert task stage to human-readable string
 */
inline const char* taskStageToString(TaskStage stage) {
  switch (stage) {
    case TaskStage::IDLE: return "Idle";
    case TaskStage::MOVE_TO_PRE_GRASP: return "Moving to Pre-Grasp";
    case TaskStage::DESCEND_TO_GRASP: return "Descending to Grasp";
    case TaskStage::CLOSE_GRIPPER: return "Closing Gripper";
    case TaskStage::LIFT_BALL: return "Lifting Ball";
    case TaskStage::MOVE_TO_PRE_PLACE: return "Moving to Pre-Place";
    case TaskStage::LOWER_TO_PLACE: return "Lowering to Place";
    case TaskStage::OPEN_GRIPPER: return "Opening Gripper";
    case TaskStage::RETRACT: return "Retracting";
    case TaskStage::RETURN_HOME: return "Returning Home";
    case TaskStage::COMPLETE: return "Complete";
    default: return "Unknown";
  }
}

// ============================================================================
// Joint Information
// ============================================================================

/**
 * @brief Information about a single joint
 */
struct JointInfo {
  std::string name;           ///< Joint name
  float lower_bound{-3.14159f}; ///< Lower position limit (radians)
  float upper_bound{3.14159f};  ///< Upper position limit (radians)
  double max_velocity{2.0};     ///< Maximum velocity (rad/s)
  double max_torque{87.0};      ///< Maximum torque (Nm)
};

// ============================================================================
// Link Visual Information
// ============================================================================

/**
 * @brief Information for rendering a link's visual mesh
 */
struct LinkVisualInfo {
  std::string link_name;
  int link_index;               ///< RBD body index (-1 for root)
  std::string mesh_path;
  glm::mat4 visual_origin;      ///< Transform from link to visual frame
  visual::CombinedMesh* mesh;     ///< Loaded mesh (non-owning pointer)
};

// ============================================================================
// IK Solver Types
// ============================================================================

/**
 * @brief Result of an IK solve attempt
 */
struct IKResult {
  bool success = false;         ///< Whether IK converged
  bool within_limits = false;   ///< Whether solution is within joint limits
  double error_norm = 0.0;      ///< Final constraint error
  unsigned int iterations = 0;  ///< Number of iterations used
  double distance_from_start = 0.0; ///< Configuration distance from start
  contradium::rbd::VectorX Q;               ///< Solution configuration
};

/**
 * @brief Configuration for IK solver
 */
struct IKSolverConfig {
  unsigned int max_iter = 100;
  double step_tol = 1e-10;
  double constraint_tol = 1e-10;
  int num_samples = 50;
  bool verbose = false;
};

// ============================================================================
// Trajectory Types
// ============================================================================

/**
 * @brief A single point in a joint space trajectory
 */
struct TrajectoryPoint {
  contradium::rbd::VectorX Q;      ///< Joint positions
  contradium::rbd::VectorX QDot;   ///< Joint velocities
  double time;         ///< Time from trajectory start
};

/**
 * @brief A complete trajectory
 */
struct Trajectory {
  std::vector<TrajectoryPoint> points;
  double duration;     ///< Total trajectory duration

  /**
   * @brief Get trajectory state at a given time via linear interpolation
   */
  std::optional<TrajectoryPoint> sample(double t) const {
    if (points.empty()) return std::nullopt;
    t = std::clamp(t, 0.0, duration);

    // Find surrounding points
    size_t idx = 0;
    for (size_t i = 0; i < points.size() - 1; ++i) {
      if (t >= points[i].time && t <= points[i+1].time) {
        idx = i;
        break;
      }
    }

    if (idx >= points.size() - 1) {
      return points.back();
    }

    // Linear interpolation
    const auto& p0 = points[idx];
    const auto& p1 = points[idx + 1];
    double alpha = (t - p0.time) / (p1.time - p0.time);

    TrajectoryPoint result;
    result.Q = p0.Q + alpha * (p1.Q - p0.Q);
    result.QDot = p0.QDot + alpha * (p1.QDot - p0.QDot);
    result.time = t;
    return result;
  }
};

// ============================================================================
// Path Planning Waypoint
// ============================================================================

/**
 * @brief A waypoint for path planning
 */
struct Waypoint {
  enum class Type {
    CONFIGURATION,  ///< Joint space waypoint
    CARTESIAN       ///< Cartesian space waypoint (requires IK)
  };

  Type type = Type::CONFIGURATION;

  // For CONFIGURATION type
  contradium::rbd::VectorX Q;
  contradium::rbd::VectorX QDot;       ///< Desired velocity at waypoint

  // For CARTESIAN type
  glm::vec3 position;
  glm::mat3 orientation;   ///< Rotation matrix

  double min_duration = 0.5;  ///< Minimum time to reach waypoint
};

// ============================================================================
// Task State
// ============================================================================

/**
 * @brief Complete state of the pick-and-place task
 */
struct TaskState {
  TaskStage stage = TaskStage::IDLE;
  double stage_time = 0.0;        ///< Time elapsed in current stage
  double stage_duration = 0.0;    ///< Total duration of current stage

  bool ball_grasped = false;
  glm::vec3 ball_position{0.4f, 0.0f, 0.0175f};
  glm::vec3 goal_position{0.4f, 0.4f, 0.0f};

  Trajectory current_trajectory;  ///< Trajectory for current stage
};

// ============================================================================
// Controller Configuration
// ============================================================================

/**
 * @brief PD controller configuration
 */
struct ControllerConfig {
  double kp = 100.0;        ///< Proportional gain
  double kd = 5.0;          ///< Derivative gain
  double max_torque = 87.0; ///< Torque limit (Nm)

  ControllerConfig() = default;

  ControllerConfig(double p, double d, double max_t)
    : kp(p), kd(d), max_torque(max_t) {}
};

// ============================================================================
// Path Planning Configuration
// ============================================================================

/**
 * @brief Configuration for path planner
 */
struct PathPlannerConfig {
  double max_velocity = 2.0;     ///< Maximum joint velocity (rad/s)
  double max_acceleration = 5.0;  ///< Maximum joint acceleration (rad/s^2)
  double max_jerk = 20.0;         ///< Maximum joint jerk (rad/s^3)
  double min_segment_duration = 0.5; ///< Minimum segment time (seconds)
  int ik_max_samples = 50;        ///< IK samples per waypoint
};

} // namespace franka
