/**
 * @file main.cpp
 * @brief G1 Humanoid Robot URDF Loading and Forward Kinematics Exporter
 *
 * This program loads the Unitree G1 23-DOF humanoid robot from a URDF file,
 * computes forward kinematics, and exports the result to JSON for Blender rendering.
 *
 * Features:
 * - URDF model loading using urdf::Model::fromXmlStr
 * - Forward kinematics computation using RBD model
 * - JSON export of mesh paths, transforms, and materials
 * - Uses FileProbe for asset file discovery
 * - Uses nlohmann-json for JSON serialization
 */

#include <long_march.h>
#include <nlohmann/json.hpp>
#include <fstream>

#include "contradium/rbd/urdf/urdf.h"

using namespace long_march;
namespace rbd = contradium::rbd;
using json = nlohmann::json;

// ============================================================================
// Type Aliases
// ============================================================================

using Transform4x4 = std::array<std::array<double, 4>, 4>;

// ============================================================================
// Helper Structures
// ============================================================================

struct JointInfo {
  std::string name;
  float lower_bound{0.0f};
  float upper_bound{0.0f};
  float value{0.0f};
};

struct LinkData {
  int link_index;
  std::string mesh_path;
  std::array<double, 3> color;
  Transform4x4 transform;

  json toJson() const {
    json j;
    j["link_index"] = link_index;
    j["mesh_path"] = mesh_path;
    j["color"] = color;
    j["transform"] = transform;
    return j;
  }
};

// ============================================================================
// Utility Functions
// ============================================================================

std::string readFileToString(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    grassland::LogError("Failed to open file: {}", filepath);
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void exportRobotStateToJSON(const std::string& output_path,
                            const std::vector<LinkData>& links) {
  json j;
  j["links"] = json::array();

  for (const auto& link : links) {
    j["links"].push_back(link.toJson());
  }

  std::ofstream out(output_path);
  if (!out.is_open()) {
    grassland::LogError("Failed to open output file: {}", output_path);
    return;
  }

  out << std::setw(4) << j;
  out.close();

  grassland::LogInfo("Exported robot state to {}", output_path);
}

// ============================================================================
// Main Application
// ============================================================================

int main(int argc, char** argv) {
  std::string export_path = "robot_state.json";
  if (argc > 1) {
    export_path = argv[1];
  }

  // Register G1 robot assets directory in FileProbe
  // This allows us to use short filenames like "g1_23dof_mode_10.urdf"
  std::string g1_assets_dir = std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/unitree_g1/";
  grassland::FileProbe::GetInstance().AddSearchPath(g1_assets_dir);
  grassland::FileProbe::GetInstance().AddSearchPath(g1_assets_dir + "meshes/");

  // Load URDF file using short filename
  std::string urdf_path = grassland::FileProbe::GetInstance().FindFile("g1_23dof_mode_10.urdf");
  if (urdf_path.empty()) {
    grassland::LogError("Failed to find URDF file: g1_23dof_mode_10.urdf");
    return 1;
  }

  grassland::LogInfo("Loading URDF from: {}", urdf_path);

  std::string urdf_xml = readFileToString(urdf_path);
  if (urdf_xml.empty()) {
    grassland::LogError("Failed to read URDF file");
    return 1;
  }

  // Parse URDF
  auto urdf_model = urdf::Model::fromXmlStr(urdf_xml);
  if (!urdf_model) {
    grassland::LogError("Failed to parse URDF");
    return 1;
  }
  grassland::LogInfo("URDF has {} links", urdf_model->linkMap.size());

  // Convert to RBD model
  auto rbd_model = urdf::toRBDModel(urdf_model);
  grassland::LogInfo("RBD model has {} movable bodies and {} fixed bodies",
          rbd_model.mBodies.size(), rbd_model.mFixedBodies.size());

  // Collect joint information for movable joints
  std::vector<JointInfo> joint_infos;
  for (const auto& [joint_name, joint] : urdf_model->jointMap) {
    if (joint->type == urdf::JointType::REVOLUTE || joint->type == urdf::JointType::CONTINUOUS) {
      JointInfo info;
      info.name = joint_name;
      if (joint->limits) {
        info.lower_bound = (*joint->limits)->lower;
        info.upper_bound = (*joint->limits)->upper;
        info.value = 0.0f;  // Set to zero
      }
      joint_infos.push_back(info);
    }
  }

  // Collect mesh data without rendering
  // Store: mesh_path, color, visual_origin, link_index
  struct VisualData {
    std::string mesh_path;
    std::array<double, 3> color;
    urdf::Transform visual_origin;
    Transform4x4 transform;
  };
  std::map<int, VisualData> link_data_for_export;

  // Load mesh info and store
  for (const auto& [link_name, link] : urdf_model->linkMap) {
    for (const auto& visual : link->visuals) {
      if (visual->geometry && (*visual->geometry)->type == urdf::GeometryType::MESH) {
        const auto* mesh_geom = dynamic_cast<const urdf::Mesh*>(visual->geometry->get());
        if (mesh_geom) {
          // Mesh filename in URDF is relative (e.g., "meshes/pelvis.STL")
          // Extract just the filename part for FileProbe lookup
          std::string mesh_filename = mesh_geom->filename;

          // Find the last path separator to get just the filename
          size_t last_sep = mesh_filename.find_last_of("/\\");
          std::string short_name = (last_sep != std::string::npos)
              ? mesh_filename.substr(last_sep + 1)
              : mesh_filename;

          // Use FileProbe to find the mesh (searches registered paths)
          std::string mesh_path = grassland::FileProbe::GetInstance().FindFile(short_name);
          if (mesh_path.empty()) {
            grassland::LogWarning("Mesh file not found: {}, using full path fallback", short_name);
            // Fallback to full path construction
            mesh_path = std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/unitree_g1/" + mesh_filename;
          }

          // Get material color from URDF if available
          std::array<double, 3> color{{0.7, 0.7, 0.7}};  // Default white-ish color
          if (visual->material.has_value() && visual->material.value()) {
            const auto& mat = visual->material.value();
            color[0] = static_cast<double>(mat->color(0));
            color[1] = static_cast<double>(mat->color(1));
            color[2] = static_cast<double>(mat->color(2));
          }

          // Store mesh info including the visual origin transform
          VisualData data;
          data.mesh_path = mesh_path;
          data.color = color;
          data.visual_origin = visual->origin;
          link_data_for_export[link->link_index] = data;
          break;  // Only first visual mesh
        }
      }
    }
  }

  grassland::LogInfo("Found {} links with visual meshes", link_data_for_export.size());

  // Set up joint positions for forward kinematics
  Eigen::VectorXd Q = Eigen::VectorXd::Zero(rbd_model.dof_count);
  for (size_t i = 0; i < joint_infos.size() && i < rbd_model.dof_count; i++) {
    Q[i] = joint_infos[i].value;
  }

  // Compute forward kinematics
  rbd::UpdateKinematicsCustom(rbd_model, &Q, nullptr, nullptr);

  // Helper to convert RPY to rotation matrix (same as in urdf.cpp)
  auto rpyToRotationMatrix = [](const Eigen::Vector3d &rpy) -> rbd::Matrix3 {
    const double roll = rpy(0);
    const double pitch = rpy(1);
    const double yaw = rpy(2);

    const double su = std::sin(roll);
    const double cu = std::cos(roll);
    const double sv = std::sin(pitch);
    const double cv = std::cos(pitch);
    const double sw = std::sin(yaw);
    const double cw = std::cos(yaw);

    rbd::Matrix3 R;
    R(0, 0) = cw * cv;
    R(0, 1) = cw * sv * su - sw * cu;
    R(0, 2) = cw * sv * cu + sw * su;
    R(1, 0) = sw * cv;
    R(1, 1) = sw * sv * su + cw * cu;
    R(1, 2) = sw * sv * cu - cw * su;
    R(2, 0) = -sv;
    R(2, 1) = cv * su;
    R(2, 2) = cv * cu;
    return R;
  };

  // Update transformations based on FK results
  for (auto& [link_index, data] : link_data_for_export) {
    if (link_index < 0) continue;

    unsigned int body_id = static_cast<unsigned int>(link_index);

    // RBD functions handle both movable and fixed bodies
    // Get link frame in world coordinates
    rbd::Matrix3 E_link = rbd::CalcBodyToBaseRotation(rbd_model, Q, body_id, false);
    rbd::Vector3 r_link = rbd::CalcBodyToBaseCoordinates(rbd_model, Q, body_id, rbd::Vector3(0, 0, 0), false);

    // E is rotation in spatial convention (world-to-body), E^T gives body-to-world
    rbd::Matrix3 R_link = E_link.transpose();

    // Apply visual origin transform (link frame -> visual frame)
    rbd::Matrix3 R_visual_origin = rpyToRotationMatrix(data.visual_origin.rpy);
    rbd::Vector3 t_visual_origin = data.visual_origin.xyz;

    // Compose: world -> link -> visual
    // R_final = R_link * R_visual_origin
    // t_final = r_link + R_link * t_visual_origin
    rbd::Matrix3 R_final = R_link * R_visual_origin;
    rbd::Vector3 t_final = r_link + R_link * t_visual_origin;

    auto& transform = data.transform;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        transform[row][col] = R_final(row, col);
      }
    }
    transform[0][3] = t_final(0);
    transform[1][3] = t_final(1);
    transform[2][3] = t_final(2);
    // Bottom row stays [0, 0, 0, 1]
    transform[3][0] = 0.0;
    transform[3][1] = 0.0;
    transform[3][2] = 0.0;
    transform[3][3] = 1.0;
  }

  // Convert to LinkData for export
  std::vector<LinkData> export_data;
  for (const auto& [link_index, data] : link_data_for_export) {
    LinkData link_data;
    link_data.link_index = link_index;
    link_data.mesh_path = data.mesh_path;
    link_data.color = data.color;
    link_data.transform = data.transform;
    export_data.push_back(link_data);
  }

  exportRobotStateToJSON(export_path, export_data);

  return 0;
}
