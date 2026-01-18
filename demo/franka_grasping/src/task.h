/**
 * @file task.h
 * @brief Task state machine for pick-and-place
 *
 * Manages the high-level task execution including stage transitions
 * and trajectory following.
 */

#pragma once

#include "types.h"
#include "urdf_utils.h"
#include "path_planner.h"
#include "config.h"
#include <glm/glm.hpp>

namespace franka::task {

// Alias for contradium::rbd types
using namespace contradium::rbd;

/**
 * @brief Task manager for pick-and-place operation
 *
 * Orchestrates the complete pick-and-place task using a state machine.
 * Handles stage transitions, trajectory following, and gripper control.
 */
class TaskManager {
public:
  /**
   * @brief Construct task manager
   * @param model RBD model
   * @param planner Path planner
   * @param robot_model Complete robot model
   */
  TaskManager(
      Model& model,
      planner::PathPlanner& planner,
      const urdf_utils::RobotModel& robot_model);

  /**
   * @brief Start the pick-and-place task
   * @param ball_pos Ball position (x, y, z)
   * @param goal_pos Goal position (x, y)
   * @param Q_current Current robot configuration (used to plan first stage)
   * @return true if task started successfully
   */
  bool start(const glm::vec3& ball_pos, const glm::vec3& goal_pos, const VectorX& Q_current);

  /**
   * @brief Reset task to idle state
   */
  void reset();

  /**
   * @brief Update task state
   *
   * Advances the task state machine and computes desired joint states.
   *
   * @param dt Time step (seconds)
   * @param Q Current joint positions (input)
   * @param QDot Current joint velocities (input)
   * @param Q_desired Output desired joint positions
   * @param QDot_desired Output desired joint velocities
   * @return true if task is still running
   */
  bool update(
      double dt,
      const VectorX& Q,
      const VectorX& QDot,
      VectorX& Q_desired,
      VectorX& QDot_desired);

  /**
   * @brief Get current task stage
   */
  TaskStage getCurrentStage() const { return state_.stage; }

  /**
   * @brief Get current stage progress (0.0 to 1.0)
   */
  double getStageProgress() const;

  /**
   * @brief Get stage name as string
   */
  const char* getStageName() const { return taskStageToString(state_.stage); }

  /**
   * @brief Get task state
   */
  const TaskState& getState() const { return state_; }

  /**
   * @brief Check if ball is grasped
   */
  bool isBallGrasped() const { return state_.ball_grasped; }

  /**
   * @brief Get stage end configuration (trajectory goal)
   */
  const VectorX& getStageEnd() const { return Q_stage_end_; }

  /**
   * @brief Set ball position (for GUI control)
   */
  void setBallPosition(const glm::vec3& pos) { state_.ball_position = pos; }

  /**
   * @brief Set goal position (for GUI control)
   */
  void setGoalPosition(const glm::vec3& pos) { state_.goal_position = pos; }

  /**
   * @brief Set gripper state manually
   */
  void setGripperOpen(bool open);

  /**
   * @brief Get home configuration
   */
  const VectorX& getHomeConfiguration() const { return Q_home_; }

  /**
   * @brief Set home configuration
   */
  void setHomeConfiguration(const VectorX& Q_home) { Q_home_ = Q_home; }

private:
  /**
   * @brief Plan trajectory for current stage
   */
  bool planCurrentStage(const VectorX& Q_current);

  /**
   * @brief Transition to next stage
   */
  void transitionToNextStage();

  /**
   * @brief Get stage duration
   */
  double getStageDuration(TaskStage stage) const;

  Model& model_;
  planner::PathPlanner& planner_;
  const urdf_utils::RobotModel& robot_model_;

  TaskState state_;
  VectorX Q_home_;
  Matrix3 gripper_down_rot_;

  // Trajectory following state
  double trajectory_time_;
  VectorX Q_stage_start_;
  VectorX Q_stage_end_;
  float gripper_start_;
  float gripper_target_;

  // Stage-specific configurations
  int num_arm_joints_;
};

} // namespace franka::task
