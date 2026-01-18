/**
 * @file config.h
 * @brief Configuration constants for the Franka grasping demo
 *
 * All tunable parameters are centralized here for easy adjustment.
 * Values are constexpr where possible for compile-time optimization.
 */

#pragma once

#include <glm/glm.hpp>

namespace franka::config {

// ============================================================================
// Physical Dimensions
// ============================================================================

/// Ball radius (meters) - 1/4 of max gripper opening
constexpr float BALL_RADIUS = 0.0175f;

/// Open gripper position per finger (meters)
constexpr float GRIPPER_OPEN = 0.035f;

/// Closed gripper position (meters) - equals ball radius
constexpr float GRIPPER_CLOSED = 0.0175f;

// ============================================================================
// Task Heights
// ============================================================================

/// Height above ball for pre-grasp approach (meters)
constexpr float PRE_GRASP_HEIGHT = 0.15f;

/// How high to lift the ball after grasping (meters)
constexpr float LIFT_HEIGHT = 0.3f;

/// Height above goal for retract after placing (meters)
constexpr float RETRACT_HEIGHT = 0.15f;

// ============================================================================
// Controller Gains (Per-Joint Groups)
// ============================================================================

// High inertia joints (1-4) - shoulder and elbow
constexpr double KP_ARM_BASE = 100.0;  ///< Proportional gain (Nm/rad)
constexpr double KD_ARM_BASE = 15.0;   ///< Derivative gain (Nms/rad)

// Low inertia joints (5-7) - wrist
// Lower gains prevent chatter on lightweight distal links
constexpr double KP_ARM_WRIST = 40.0;  ///< Proportional gain (Nm/rad)
constexpr double KD_ARM_WRIST = 2.0;   ///< Derivative gain (Nms/rad)

/// Proportional gain for gripper joints (Nm/rad)
constexpr double KP_GRIPPER = 50.0;

/// Derivative gain for gripper joints (Nms/rad)
/// 2*sqrt(KP_GRIPPER) for critical damping
constexpr double KD_GRIPPER = 14.0;

// ============================================================================
// Torque Limits (Franka Emika FR3 specifications - per joint group)
// ============================================================================

/// Maximum torque for base/elbow joints 1-4 (Nm)
constexpr double MAX_TORQUE_J1_J4 = 87.0;

/// Maximum torque for wrist joints 5-7 (Nm)
/// Joint 6 would physically break at 87 Nm - limit to actual spec
constexpr double MAX_TORQUE_J5_J7 = 12.0;

/// Maximum torque for gripper joints (Nm)
constexpr double MAX_TORQUE_GRIPPER = 20.0;

/// Helper to get torque limit by joint index (0-based)
inline constexpr double getTorqueLimit(const int joint_index) {
  return (joint_index < 4) ? MAX_TORQUE_J1_J4 : MAX_TORQUE_J5_J7;
}

// ============================================================================
// Simulation Parameters
// ============================================================================

/// Simulation timestep (seconds)
constexpr double DT = 0.001;

/// Number of physics substeps per render frame
constexpr int SUBSTEPS = 10;

/// Maximum joint velocity (rad/s) for clamping
constexpr double MAX_VELOCITY = 10.0;

/// Velocity low-pass filter alpha (0.0 to 1.0)
/// 1.0 = no filtering, lower values = more smoothing
/// Helps prevent derivative chatter on wrist joints
constexpr double VELOCITY_FILTER_ALPHA = 0.3;

// ============================================================================
// Stage Durations (seconds)
// ============================================================================

/// Time to move to pre-grasp position
constexpr double STAGE_MOVE_TO_PRE_GRASP = 3.0;

/// Time to descend to ball
constexpr double STAGE_DESCEND = 2.0;

/// Time to close gripper
constexpr double STAGE_CLOSE_GRIPPER = 1.0;

/// Time to lift ball
constexpr double STAGE_LIFT = 2.5;

/// Time to move to pre-place position
constexpr double STAGE_MOVE_TO_PLACE = 5.0;

/// Time to lower to goal
constexpr double STAGE_LOWER = 3.0;

/// Time to open gripper
constexpr double STAGE_OPEN_GRIPPER = 1.0;

/// Time to retract from goal
constexpr double STAGE_RETRACT = 2.0;

/// Time to return to home configuration
constexpr double STAGE_RETURN_HOME = 5.0;

// ============================================================================
// Path Planning Configuration
// ============================================================================

/// Maximum joint velocity for trajectory planning (rad/s)
constexpr double MAX_JOINT_VELOCITY = 2.0;

/// Maximum joint acceleration for trajectory planning (rad/s^2)
constexpr double MAX_JOINT_ACCELERATION = 5.0;

/// Maximum joint jerk for S-curve planning (rad/s^3)
constexpr double MAX_JOINT_JERK = 20.0;

/// Minimum trajectory segment duration (seconds)
constexpr double MIN_SEGMENT_DURATION = 0.5;

// ============================================================================
// IK Solver Configuration
// ============================================================================

/// Maximum iterations for IK solver
constexpr int IK_MAX_ITER = 100;

/// Step tolerance for IK convergence
constexpr double IK_STEP_TOL = 1e-10;

/// Constraint tolerance for IK convergence
constexpr double IK_CONSTRAINT_TOL = 1e-10;

/// Default number of random samples for IK
constexpr int IK_DEFAULT_SAMPLES = 50;

// ============================================================================
// Rendering Configuration
// ============================================================================

/// Main render window width
constexpr int RENDER_WIDTH = 1280;

/// Main render window height
constexpr int RENDER_HEIGHT = 720;

/// TCP view window width
constexpr int TCP_VIEW_WIDTH = 640;

/// TCP view window height
constexpr int TCP_VIEW_HEIGHT = 480;

/// Samples per pixel for path tracing
constexpr int SAMPLES_PER_DISPATCH = 32;

/// Film persistence for temporal accumulation
constexpr float FILM_PERSISTENCE = 0.98f;

/// Camera field of view (degrees)
constexpr float CAMERA_FOV = 40.0f;

/// TCP camera field of view (degrees)
constexpr float TCP_CAMERA_FOV = 60.0f;

// ============================================================================
// Camera Position (Main View)
// ============================================================================

constexpr glm::vec3 CAMERA_POSITION = glm::vec3{1.5f, -1.0f, 0.8f};
constexpr glm::vec3 CAMERA_TARGET = glm::vec3{0.3f, 0.3f, 0.3f};
constexpr glm::vec3 CAMERA_UP = glm::vec3{0.0f, 0.0f, 1.0f};

// ============================================================================
// Scene Configuration
// ============================================================================

/// Ground position (Z coordinate)
constexpr float GROUND_Z = -1000.0f;

/// Ground scale for large plane
constexpr float GROUND_SCALE = 1000.0f;

/// Sky sphere radius
constexpr float SKY_RADIUS = 60.0f;

/// Area light emission intensity
constexpr glm::vec3 AREA_LIGHT_EMISSION = glm::vec3{1000.0f};
constexpr glm::vec3 AREA_LIGHT_POSITION = glm::vec3{40.0f, -30.0f, 30.0f};
constexpr float AREA_LIGHT_SIZE = 1.0f;

/// Ambient light intensity
constexpr glm::vec3 AMBIENT_LIGHT = glm::vec3{0.5f, 0.5f, 0.5f};

// ============================================================================
// Visualization Configuration
// ============================================================================

/// Target frame axis length
constexpr float TARGET_FRAME_LENGTH = 0.1f;

/// Target frame axis thickness
constexpr float TARGET_FRAME_THICKNESS = 0.006f;

/// Current frame axis length
constexpr float CURRENT_FRAME_LENGTH = 0.08f;

/// Current frame axis thickness
constexpr float CURRENT_FRAME_THICKNESS = 0.004f;

/// Goal marker size (meters)
constexpr float GOAL_MARKER_SIZE = 0.05f;

/// Goal marker height above ground (meters)
constexpr float GOAL_MARKER_HEIGHT = 0.001f;

// ============================================================================
// Logging Configuration
// ============================================================================

/// Log interval for physics steps (number of steps)
constexpr int PHYSICS_LOG_INTERVAL = 100;

/// Log interval for joint torques (number of frames)
constexpr int TORQUE_LOG_INTERVAL = 10;

/// Torque threshold for warnings (Nm)
/// Warns when torque approaches limits
constexpr double TORQUE_WARNING_THRESHOLD = 10.0;

} // namespace franka::config
