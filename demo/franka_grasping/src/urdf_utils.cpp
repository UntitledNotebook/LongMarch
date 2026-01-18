/**
 * @file urdf_utils.cpp
 * @brief URDF model loading and processing implementation
 */

#include "urdf_utils.h"
#include "grassland/util/log.h"
#include "grassland/util/file_probe.h"
#include <glm/gtc/matrix_transform.hpp>
#include <sstream>
#include <fstream>

using grassland::LogInfo;
using grassland::LogWarning;
using grassland::LogError;
using grassland::FindAssetFile;

namespace franka::urdf_utils {

namespace {

/**
 * @brief Read file contents to string
 */
std::string readFileToString(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    LogError("Failed to open file: {}", filepath);
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

/**
 * @brief Collect movable joints recursively in depth-first order
 */
void collectMovableJointsRecursive(
    const std::shared_ptr<::urdf::Link>& link,
    std::vector<std::shared_ptr<::urdf::Joint>>& movable_joints) {

  for (size_t i = 0; i < link->child_joints.size(); ++i) {
    const auto& joint = link->child_joints[i];
    const auto& child_link = link->child_links[i];

    if (joint->type == ::urdf::JointType::REVOLUTE ||
        joint->type == ::urdf::JointType::CONTINUOUS ||
        joint->type == ::urdf::JointType::PRISMATIC) {
      movable_joints.push_back(joint);
    }

    collectMovableJointsRecursive(child_link, movable_joints);
  }
}

/**
 * @brief Collect all movable joints from URDF
 */
std::vector<std::shared_ptr<::urdf::Joint>> collectMovableJoints(
    const std::shared_ptr<::urdf::Model>& urdf_model) {

  std::vector<std::shared_ptr<::urdf::Joint>> movable_joints;
  auto root = urdf_model->getRoot();
  if (root) {
    collectMovableJointsRecursive(root, movable_joints);
  }
  return movable_joints;
}

/**
 * @brief Collect visual information from URDF links
 */
std::vector<LinkVisualInfo> collectLinkVisuals(
    const std::shared_ptr<::urdf::Model>& urdf_model) {

  std::vector<LinkVisualInfo> visuals;

  for (const auto& [name, link] : urdf_model->linkMap) {
    for (const auto& visual : link->visuals) {
      if (visual->geometry.has_value()) {
        auto geom = visual->geometry.value();
        if (geom->type == ::urdf::GeometryType::MESH) {
          auto mesh_geom = std::dynamic_pointer_cast<::urdf::Mesh>(geom);
          if (mesh_geom) {
            LinkVisualInfo info;
            info.link_name = name;
            info.link_index = link->link_index;
            info.mesh_path = resolvePackageUri(mesh_geom->filename);
            info.visual_origin = xyzRpyToMat4(visual->origin.xyz, visual->origin.rpy);
            info.mesh = nullptr;  // Will be set later by visual module
            visuals.push_back(std::move(info));
          }
        }
      }
    }
  }

  return visuals;
}

} // namespace

// ============================================================================
// Public Functions
// ============================================================================

std::optional<RobotModel> loadRobotModel(
    const std::string& urdf_filename,
    const std::string& tcp_link_name) {

  RobotModel model;

  // Find and read URDF file
  std::string urdf_path = FindAssetFile(urdf_filename);
  if (urdf_path.empty()) {
    LogError("URDF file not found: {}", urdf_filename);
    return std::nullopt;
  }

  LogInfo("Loading URDF from: {}", urdf_path);

  std::string xml_string = readFileToString(urdf_path);
  if (xml_string.empty()) {
    LogError("Failed to read URDF file");
    return std::nullopt;
  }

  // Parse URDF
  model.urdf_model = ::urdf::Model::fromXmlStr(xml_string);
  if (!model.urdf_model) {
    LogError("Failed to parse URDF");
    return std::nullopt;
  }

  LogInfo("URDF loaded: {}", model.urdf_model->getName());
  LogInfo("Links: {}", model.urdf_model->linkMap.size());
  LogInfo("Joints: {}", model.urdf_model->jointMap.size());

  // Convert to RBD model
  model.rbd_model = ::urdf::toRBDModel(model.urdf_model);
  model.rbd_model.gravity = contradium::rbd::Vector3(0, 0, -9.81);

  LogInfo("RBD Model DOF: {}", model.rbd_model.dof_count);

  // Collect movable joints
  model.movable_joints = collectMovableJoints(model.urdf_model);
  LogInfo("Movable joints: {}", model.movable_joints.size());

  // Build joint info
  model.joint_info.reserve(model.rbd_model.dof_count);
  for (unsigned int i = 0; i < model.rbd_model.dof_count; ++i) {
    JointInfo info;
    if (i < model.movable_joints.size()) {
      const auto& joint = model.movable_joints[i];
      info.name = joint->name;
      if (joint->limits.has_value()) {
        auto limits = joint->limits.value();
        info.lower_bound = static_cast<float>(limits->lower);
        info.upper_bound = static_cast<float>(limits->upper);
      }
    } else {
      info.name = "joint_" + std::to_string(i);
    }
    model.joint_info.push_back(info);
  }

  // Print joint info
  for (unsigned int i = 0; i < model.rbd_model.dof_count; ++i) {
    const auto& info = model.joint_info[i];
    LogInfo("  DOF {}: {} [{}, {}]", i, info.name, info.lower_bound, info.upper_bound);
  }

  // Get TCP link
  auto tcp_link = model.urdf_model->getLink(tcp_link_name);
  if (!tcp_link) {
    LogError("Could not find TCP link: {}", tcp_link_name);
    return std::nullopt;
  }
  model.tcp_body_id = static_cast<unsigned int>(tcp_link->link_index);
  LogInfo("TCP link '{}' body_id: {}", tcp_link_name, model.tcp_body_id);

  // Collect visuals
  model.link_visuals = collectLinkVisuals(model.urdf_model);
  LogInfo("Found {} visual meshes", model.link_visuals.size());

  return model;
}

std::string resolvePackageUri(const std::string& uri) {
  const std::string package_prefix = "package://";

  if (uri.substr(0, package_prefix.size()) == package_prefix) {
    std::string remaining = uri.substr(package_prefix.size());

    // Try to extract package name
    size_t slash_pos = remaining.find('/');
    if (slash_pos != std::string::npos) {
      std::string package = remaining.substr(0, slash_pos);
      std::string path = remaining.substr(slash_pos + 1);

      // Skip "meshes/" prefix if present
      const std::string meshes_prefix = "meshes/";
      if (path.substr(0, meshes_prefix.size()) == meshes_prefix) {
        path = path.substr(meshes_prefix.size());
      }

      // Try to find with FileProbe
      std::string result = FindAssetFile(path);
      if (!result.empty()) {
        return result;
      }

      // Fallback: construct path manually
      return std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/franka_fr3/meshes/" + path;
    }
  }

  // Not a package URI, try direct lookup
  return FindAssetFile(uri);
}

glm::mat4 xyzRpyToMat4(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
  glm::mat4 t = glm::translate(glm::mat4(1.0f),
      glm::vec3(static_cast<float>(xyz.x()), static_cast<float>(xyz.y()), static_cast<float>(xyz.z())));

  glm::mat4 r = glm::rotate(glm::mat4(1.0f), static_cast<float>(rpy.z()), glm::vec3(0.0f, 0.0f, 1.0f)) *
                glm::rotate(glm::mat4(1.0f), static_cast<float>(rpy.y()), glm::vec3(0.0f, 1.0f, 0.0f)) *
                glm::rotate(glm::mat4(1.0f), static_cast<float>(rpy.x()), glm::vec3(1.0f, 0.0f, 0.0f));

  return t * r;
}

contradium::rbd::Matrix3 rpyToRotationMatrix(double roll, double pitch, double yaw) {
  double cy = cos(yaw), sy = sin(yaw);
  double cp = cos(pitch), sp = sin(pitch);
  double cr = cos(roll), sr = sin(roll);

  contradium::rbd::Matrix3 R;
  R(0,0) = cy*cp;  R(0,1) = cy*sp*sr - sy*cr;  R(0,2) = cy*sp*cr + sy*sr;
  R(1,0) = sy*cp;  R(1,1) = sy*sp*sr + cy*cr;  R(1,2) = sy*sp*cr - cy*sr;
  R(2,0) = -sp;    R(2,1) = cp*sr;             R(2,2) = cp*cr;

  return R;
}

glm::mat4 posRotToGLM(const contradium::rbd::Vector3& pos, const contradium::rbd::Matrix3& R) {
  glm::mat4 result(1.0f);
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      result[col][row] = static_cast<float>(R(row, col));
    }
  }
  result[3][0] = static_cast<float>(pos(0));
  result[3][1] = static_cast<float>(pos(1));
  result[3][2] = static_cast<float>(pos(2));
  return result;
}

glm::mat4 computeLinkTransform(int link_index, contradium::rbd::Model& rbd_model, const contradium::rbd::VectorX& Q) {
  if (link_index < 0) {
    return glm::mat4(1.0f);
  }

  unsigned int body_id = static_cast<unsigned int>(link_index);

  // CalcBodyToBaseRotation returns E (world-to-body in spatial convention)
  // E^T gives rotation from body frame to world frame
  contradium::rbd::Matrix3 E = contradium::rbd::CalcBodyToBaseRotation(rbd_model, Q, body_id, false);
  contradium::rbd::Vector3 r = contradium::rbd::CalcBodyToBaseCoordinates(rbd_model, Q, body_id, contradium::rbd::Vector3(0, 0, 0), false);

  contradium::rbd::Matrix3 R = E.transpose();

  glm::mat4 result(1.0f);
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      result[col][row] = static_cast<float>(R(row, col));
    }
  }
  result[3][0] = static_cast<float>(r(0));
  result[3][1] = static_cast<float>(r(1));
  result[3][2] = static_cast<float>(r(2));
  return result;
}

void updateVisuals(
    contradium::rbd::Model& rbd_model,
    const std::shared_ptr<::urdf::Model>& urdf_model,
    std::vector<LinkVisualInfo>& visuals,
    const contradium::rbd::VectorX& Q) {

  // Update kinematics
  contradium::rbd::UpdateKinematicsCustom(rbd_model, &Q, nullptr, nullptr);

  // Update each visual
  for (auto& visual : visuals) {
    glm::mat4 link_transform = computeLinkTransform(visual.link_index, rbd_model, Q);
    glm::mat4 final_transform = link_transform * visual.visual_origin;

    // The visual module will handle the actual mesh update
    // (visual.mesh is a non-owning pointer set by the visual module)
  }
}

// ============================================================================
// Joint Query Functions
// ============================================================================

std::string getJointName(const RobotModel& model, int dof_index) {
  if (dof_index >= 0 && dof_index < static_cast<int>(model.joint_info.size())) {
    return model.joint_info[dof_index].name;
  }
  return "joint_" + std::to_string(dof_index);
}

std::pair<float, float> getJointLimits(const RobotModel& model, int dof_index) {
  if (dof_index >= 0 && dof_index < static_cast<int>(model.joint_info.size())) {
    const auto& info = model.joint_info[dof_index];
    return {info.lower_bound, info.upper_bound};
  }
  return {-3.14159f, 3.14159f};
}

bool isWithinLimits(const RobotModel& model, const contradium::rbd::VectorX& Q) {
  for (int i = 0; i < Q.size() && i < static_cast<int>(model.joint_info.size()); ++i) {
    const auto& info = model.joint_info[i];
    if (Q(i) < info.lower_bound || Q(i) > info.upper_bound) {
      return false;
    }
  }
  return true;
}

void clampToLimits(const RobotModel& model, contradium::rbd::VectorX& Q) {
  for (int i = 0; i < Q.size() && i < static_cast<int>(model.joint_info.size()); ++i) {
    const auto& info = model.joint_info[i];
    double old_val = Q(i);
    Q(i) = std::clamp(Q(i), static_cast<double>(info.lower_bound), static_cast<double>(info.upper_bound));
  }
}

} // namespace franka::urdf_utils
