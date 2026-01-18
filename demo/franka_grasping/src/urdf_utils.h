/**
 * @file urdf_utils.h
 * @brief URDF model loading and processing utilities
 *
 * Provides functions for loading URDF models, extracting joint information,
 * and processing visual meshes for rendering.
 */

#pragma once

#include "types.h"
#include "long_march.h"
#include "contradium/rbd/urdf/urdf.h"
#include <glm/glm.hpp>
#include <optional>
#include <memory>
#include <vector>

namespace franka::urdf_utils {

/**
 * @brief Complete robot model including URDF, RBD, and metadata
 */
struct RobotModel {
  std::shared_ptr<::urdf::Model> urdf_model;
  contradium::rbd::Model rbd_model;
  std::vector<std::shared_ptr<::urdf::Joint>> movable_joints;
  std::vector<JointInfo> joint_info;
  std::vector<LinkVisualInfo> link_visuals;
  unsigned int tcp_body_id;

  /// Get number of degrees of freedom
  int dof() const { return static_cast<int>(rbd_model.dof_count); }
};

/**
 * @brief Load robot model from URDF file
 *
 * Uses FileProbe to locate the URDF file in registered asset directories.
 * Automatically sets up gravity and collects joint/visual information.
 *
 * @param urdf_filename URDF filename (e.g., "fr3_franka_hand.urdf")
 * @param tcp_link_name Name of the TCP link (default: "fr3_hand_tcp")
 * @return RobotModel if successful, std::nullopt on failure
 */
std::optional<RobotModel> loadRobotModel(
    const std::string& urdf_filename,
    const std::string& tcp_link_name = "fr3_hand_tcp");

/**
 * @brief Resolve package:// URI to actual file path
 *
 * Converts package:// URIs like "package://franka_description/meshes/..."
 * to actual file paths using FileProbe's registered search paths.
 *
 * @param uri The package:// URI to resolve
 * @return Resolved file path, or empty string if not found
 */
std::string resolvePackageUri(const std::string& uri);

/**
 * @brief Create glm::mat4 from xyz position and rpy orientation
 */
glm::mat4 xyzRpyToMat4(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy);

/**
 * @brief Create rotation matrix from roll, pitch, yaw (XYZ Euler angles)
 */
contradium::rbd::Matrix3 rpyToRotationMatrix(double roll, double pitch, double yaw);

/**
 * @brief Create glm::mat4 from position and rotation matrix
 */
glm::mat4 posRotToGLM(const contradium::rbd::Vector3& pos, const contradium::rbd::Matrix3& R);

/**
 * @brief Compute world transform for a link using RBD kinematics
 *
 * Handles both regular bodies and fixed bodies automatically.
 *
 * @param link_index RBD body index of the link
 * @param rbd_model Rigid body dynamics model
 * @param Q Current joint configuration
 * @return World transform of the link
 */
glm::mat4 computeLinkTransform(int link_index, contradium::rbd::Model& rbd_model, const contradium::rbd::VectorX& Q);

/**
 * @brief Update all visual transforms from joint angles
 *
 * Calls UpdateKinematicsCustom and updates each visual's transform.
 *
 * @param rbd_model RBD model
 * @param urdf_model URDF model (for link info)
 * @param visuals Vector of visual info to update
 * @param Q Current joint configuration
 */
void updateVisuals(
    contradium::rbd::Model& rbd_model,
    const std::shared_ptr<::urdf::Model>& urdf_model,
    std::vector<LinkVisualInfo>& visuals,
    const contradium::rbd::VectorX& Q);

// ============================================================================
// Joint Query Functions
// ============================================================================

/**
 * @brief Get joint name for a DOF index
 */
std::string getJointName(const RobotModel& model, int dof_index);

/**
 * @brief Get joint limits for a DOF index
 */
std::pair<float, float> getJointLimits(const RobotModel& model, int dof_index);

/**
 * @brief Check if configuration is within joint limits
 */
bool isWithinLimits(const RobotModel& model, const contradium::rbd::VectorX& Q);

/**
 * @brief Clamp configuration to joint limits (in-place)
 */
void clampToLimits(const RobotModel& model, contradium::rbd::VectorX& Q);

} // namespace franka::urdf_utils
