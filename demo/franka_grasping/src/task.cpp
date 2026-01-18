/**
 * @file task.cpp
 * @brief Task state machine implementation
 */

#include "task.h"
#include "grassland/util/log.h"
#include <algorithm>

using grassland::LogInfo;
using grassland::LogWarning;

namespace franka::task {

namespace {
constexpr double SMOOTHSTEP_EPSILON = 1e-6;

/**
 * @brief Smoothstep interpolation
 */
double smoothstep(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}
} // namespace

// ============================================================================
// TaskManager
// ============================================================================

TaskManager::TaskManager(
    Model& model,
    planner::PathPlanner& planner,
    const urdf_utils::RobotModel& robot_model)
    : model_(model),
      planner_(planner),
      robot_model_(robot_model),
      gripper_down_rot_(urdf_utils::rpyToRotationMatrix(M_PI, 0.0, 0.0)),
      trajectory_time_(0.0),
      gripper_start_(config::GRIPPER_OPEN),
      gripper_target_(config::GRIPPER_OPEN),
      num_arm_joints_(7) {

  // Initialize home configuration to middle of joint limits
  Q_home_.resize(robot_model_.dof());
  for (int i = 0; i < robot_model_.dof(); ++i) {
    auto [lower, upper] = urdf_utils::getJointLimits(robot_model_, i);
    Q_home_(i) = (lower + upper) / 2.0;
  }

  // Set gripper to open
  if (robot_model_.dof() >= 9) {
    Q_home_(7) = config::GRIPPER_OPEN;
    Q_home_(8) = config::GRIPPER_OPEN;
  }
}

bool TaskManager::start(const glm::vec3& ball_pos, const glm::vec3& goal_pos, const VectorX& Q_current) {
  state_.stage = TaskStage::MOVE_TO_PRE_GRASP;
  state_.stage_time = 0.0;
  state_.ball_position = ball_pos;
  state_.goal_position = goal_pos;
  state_.ball_grasped = false;

  trajectory_time_ = 0.0;

  // Initialize stage trajectory from current configuration
  Q_stage_start_ = Q_current;
  gripper_start_ = config::GRIPPER_OPEN;
  gripper_target_ = config::GRIPPER_OPEN;

  // Plan trajectory to first stage target
  planCurrentStage(Q_current);

  LogInfo("Task: Starting pick-and-place");
  LogInfo("  Ball: ({}, {}, {})", ball_pos.x, ball_pos.y, ball_pos.z);
  LogInfo("  Goal: ({}, {})", goal_pos.x, goal_pos.y);

  return true;
}

void TaskManager::reset() {
  state_.stage = TaskStage::IDLE;
  state_.stage_time = 0.0;
  state_.ball_grasped = false;

  LogInfo("Task: Reset to idle");
}

bool TaskManager::update(
    double dt,
    const VectorX& Q,
    const VectorX& QDot,
    VectorX& Q_desired,
    VectorX& QDot_desired) {

  // Handle idle state
  if (state_.stage == TaskStage::IDLE || state_.stage == TaskStage::COMPLETE) {
    Q_desired = Q_home_;
    QDot_desired = VectorX::Zero(Q_home_.size());
    return false;
  }

  // Update stage time
  state_.stage_time += dt;

  // Get stage duration
  double duration = getStageDuration(state_.stage);
  double t = std::min(state_.stage_time / duration, 1.0);
  double t_smooth = smoothstep(t);

  // Interpolate arm joints
  for (int i = 0; i < num_arm_joints_ && i < Q_desired.size(); ++i) {
    Q_desired(i) = Q_stage_start_(i) + t_smooth * (Q_stage_end_(i) - Q_stage_start_(i));
  }

  // Interpolate gripper (smooth opening/closing)
  if (Q_desired.size() >= 9) {
    float gripper = gripper_start_ + t_smooth * (gripper_target_ - gripper_start_);
    Q_desired(7) = gripper;
    Q_desired(8) = gripper;
  }

  QDot_desired.setZero();

  // Check for stage completion
  if (state_.stage_time >= duration) {
    transitionToNextStage();
  }

  return state_.stage != TaskStage::COMPLETE;
}

double TaskManager::getStageProgress() const {
  double duration = getStageDuration(state_.stage);
  if (duration > 0) {
    return std::min(state_.stage_time / duration, 1.0);
  }
  return 0.0;
}

void TaskManager::setGripperOpen(bool open) {
  float target = open ? config::GRIPPER_OPEN : config::GRIPPER_CLOSED;
  gripper_start_ = gripper_target_;
  gripper_target_ = target;
}

double TaskManager::getStageDuration(TaskStage stage) const {
  switch (stage) {
    case TaskStage::MOVE_TO_PRE_GRASP: return config::STAGE_MOVE_TO_PRE_GRASP;
    case TaskStage::DESCEND_TO_GRASP: return config::STAGE_DESCEND;
    case TaskStage::CLOSE_GRIPPER: return config::STAGE_CLOSE_GRIPPER;
    case TaskStage::LIFT_BALL: return config::STAGE_LIFT;
    case TaskStage::MOVE_TO_PRE_PLACE: return config::STAGE_MOVE_TO_PLACE;
    case TaskStage::LOWER_TO_PLACE: return config::STAGE_LOWER;
    case TaskStage::OPEN_GRIPPER: return config::STAGE_OPEN_GRIPPER;
    case TaskStage::RETRACT: return config::STAGE_RETRACT;
    case TaskStage::RETURN_HOME: return config::STAGE_RETURN_HOME;
    default: return 1.0;
  }
}

void TaskManager::transitionToNextStage() {
  TaskStage prev_stage = state_.stage;
  state_.stage_time = 0.0;

  // Prepare for next stage
  Q_stage_start_ = Q_stage_end_;

  // Log transition
  LogInfo("Task Stage: {} -> {}", taskStageToString(prev_stage),
           taskStageToString(state_.stage));

  switch (prev_stage) {
    case TaskStage::MOVE_TO_PRE_GRASP:
      // Descend to ball
      state_.stage = TaskStage::DESCEND_TO_GRASP;
      planCurrentStage(Q_stage_start_);
      gripper_start_ = config::GRIPPER_OPEN;
      gripper_target_ = config::GRIPPER_OPEN;
      break;

    case TaskStage::DESCEND_TO_GRASP:
      // Close gripper
      state_.stage = TaskStage::CLOSE_GRIPPER;
      Q_stage_end_ = Q_stage_start_;  // Stay in place
      gripper_start_ = config::GRIPPER_OPEN;
      gripper_target_ = config::GRIPPER_CLOSED;
      break;

    case TaskStage::CLOSE_GRIPPER:
      // Lift ball
      state_.stage = TaskStage::LIFT_BALL;
      state_.ball_grasped = true;
      planCurrentStage(Q_stage_start_);
      gripper_start_ = config::GRIPPER_CLOSED;
      gripper_target_ = config::GRIPPER_CLOSED;
      break;

    case TaskStage::LIFT_BALL:
      // Move to pre-place
      state_.stage = TaskStage::MOVE_TO_PRE_PLACE;
      planCurrentStage(Q_stage_start_);
      gripper_start_ = config::GRIPPER_CLOSED;
      gripper_target_ = config::GRIPPER_CLOSED;
      break;

    case TaskStage::MOVE_TO_PRE_PLACE:
      // Lower to place
      state_.stage = TaskStage::LOWER_TO_PLACE;
      planCurrentStage(Q_stage_start_);
      gripper_start_ = config::GRIPPER_CLOSED;
      gripper_target_ = config::GRIPPER_CLOSED;
      break;

    case TaskStage::LOWER_TO_PLACE:
      // Open gripper
      state_.stage = TaskStage::OPEN_GRIPPER;
      Q_stage_end_ = Q_stage_start_;  // Stay in place
      gripper_start_ = config::GRIPPER_CLOSED;
      gripper_target_ = config::GRIPPER_OPEN;
      break;

    case TaskStage::OPEN_GRIPPER:
      // Retract
      state_.stage = TaskStage::RETRACT;
      state_.ball_grasped = false;
      planCurrentStage(Q_stage_start_);
      gripper_start_ = config::GRIPPER_OPEN;
      gripper_target_ = config::GRIPPER_OPEN;
      break;

    case TaskStage::RETRACT:
      // Return home
      state_.stage = TaskStage::RETURN_HOME;
      Q_stage_end_ = Q_home_;
      gripper_start_ = config::GRIPPER_OPEN;
      gripper_target_ = config::GRIPPER_OPEN;
      break;

    case TaskStage::RETURN_HOME:
      // Complete
      state_.stage = TaskStage::COMPLETE;
      LogInfo("Task: Pick-and-place complete!");
      break;

    default:
      break;
  }
}

bool TaskManager::planCurrentStage(const VectorX& Q_current) {
  glm::vec3 target_pos;
  float target_z = 0.0f;

  switch (state_.stage) {
    case TaskStage::MOVE_TO_PRE_GRASP:
      target_pos = glm::vec3(
          state_.ball_position.x,
          state_.ball_position.y,
          state_.ball_position.z + config::PRE_GRASP_HEIGHT);
      break;

    case TaskStage::DESCEND_TO_GRASP:
      target_pos = state_.ball_position;
      break;

    case TaskStage::LIFT_BALL:
      target_pos = glm::vec3(
          state_.ball_position.x,
          state_.ball_position.y,
          state_.ball_position.z + config::LIFT_HEIGHT);
      break;

    case TaskStage::MOVE_TO_PRE_PLACE:
      target_pos = glm::vec3(
          state_.goal_position.x,
          state_.goal_position.y,
          state_.ball_position.z + config::LIFT_HEIGHT);
      break;

    case TaskStage::LOWER_TO_PLACE:
      target_pos = glm::vec3(
          state_.goal_position.x,
          state_.goal_position.y,
          state_.ball_position.z);
      break;

    case TaskStage::RETRACT:
      target_pos = glm::vec3(
          state_.goal_position.x,
          state_.goal_position.y,
          state_.ball_position.z + config::RETRACT_HEIGHT);
      break;

    default:
      return false;
  }

  // Solve IK
  Vector3 target_rbd(target_pos.x, target_pos.y, target_pos.z);

  IKSolverConfig ik_config;
  ik_config.max_iter = config::IK_MAX_ITER;
  ik_config.num_samples = 20;

  auto& ik_solver = planner_.getIKSolver();
  bool success = ik_solver.solveFullPose(
      Q_current, robot_model_.tcp_body_id,
      target_rbd, gripper_down_rot_,
      Q_stage_end_, ik_config);

  if (!success) {
    LogWarning("IK failed for stage: {}, using current config",
               taskStageToString(state_.stage));
    Q_stage_end_ = Q_current;
  }

  // Keep gripper at current state
  if (Q_stage_end_.size() >= 9) {
    Q_stage_end_(7) = gripper_target_;
    Q_stage_end_(8) = gripper_target_;
  }

  return success;
}

} // namespace franka::task
