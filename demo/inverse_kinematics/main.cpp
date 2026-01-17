/**
 * @file main.cpp
 * @brief Franka FR3 Inverse Kinematics Demo with URDF Loading
 *
 * This demo loads a robot model from URDF and demonstrates inverse kinematics.
 * It provides sliders for target position/orientation and solves IK in real-time.
 *
 * Features:
 * - URDF model loading using urdf::toRBDModel
 * - 6-DOF target pose control (xyz + rpy)
 * - Full IK constraint over fr3_hand_tcp link
 * - Visual target/current frame indicators (RGB axis sticks)
 * - Mesh rendering with proper placement using CalcBodyToBaseCoordinates
 */

#include <long_march.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <algorithm>
#include <fstream>
#include <random>
#include <sstream>

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "glm/gtc/matrix_transform.hpp"
#include "stb_image_write.h"

#include "contradium/rbd/urdf/urdf.h"

using namespace long_march;

namespace rbd = contradium::rbd;

/**
 * @brief Combined mesh structure for loading and rendering DAE/mesh files
 */
struct CombinedMesh {
  std::vector<std::unique_ptr<sparkium::GeometryMesh>> meshes;
  std::vector<std::unique_ptr<sparkium::MaterialPrincipled>> materials;
  std::vector<std::pair<std::unique_ptr<sparkium::EntityGeometryMaterial>, glm::mat4>> entities;

  void LoadEntities(sparkium::Core *core,
                    const aiScene *scene,
                    const aiNode *node,
                    const glm::mat4 transformation = {1.0f}) {
    glm::mat4 local_transform{
        node->mTransformation.a1, node->mTransformation.b1, node->mTransformation.c1, node->mTransformation.d1,
        node->mTransformation.a2, node->mTransformation.b2, node->mTransformation.c2, node->mTransformation.d2,
        node->mTransformation.a3, node->mTransformation.b3, node->mTransformation.c3, node->mTransformation.d3,
        node->mTransformation.a4, node->mTransformation.b4, node->mTransformation.c4, node->mTransformation.d4};
    if (scene->mRootNode == node) {
      local_transform = glm::mat4{1.0f};
    }
    local_transform = transformation * local_transform;
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
      LoadEntities(core, scene, node->mChildren[i], local_transform);
    }
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
      int mesh_index = node->mMeshes[i];
      auto &mesh = meshes[mesh_index];
      auto &material = materials[scene->mMeshes[mesh_index]->mMaterialIndex];
      auto entity =
          std::make_unique<sparkium::EntityGeometryMaterial>(core, mesh.get(), material.get(), local_transform);
      entities.emplace_back(std::move(entity), local_transform);
    }
  }

  void LoadModel(sparkium::Core *core, const std::string &path) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
      LogError("Failed to load model: " + path);
      return;
    }

    for (unsigned int i = 0; i < scene->mNumMaterials; i++) {
      auto material = scene->mMaterials[i];
      aiColor4D color;
      material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
      float reflectivity = 0.0f;
      material->Get(AI_MATKEY_REFLECTIVITY, reflectivity);
      auto mat = std::make_unique<sparkium::MaterialPrincipled>(core, glm::vec3{color.r, color.g, color.b});
      mat->specular = reflectivity;
      mat->roughness = reflectivity;
      materials.emplace_back(std::move(mat));
    }

    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
      auto mesh = scene->mMeshes[i];
      std::vector<glm::vec3> positions;
      std::vector<glm::vec3> normals;
      std::vector<glm::vec2> tex_coords;
      std::vector<uint32_t> indices;

      for (unsigned int v = 0; v < mesh->mNumVertices; v++) {
        positions.emplace_back(mesh->mVertices[v].x, mesh->mVertices[v].y, mesh->mVertices[v].z);
        if (mesh->HasNormals()) {
          normals.emplace_back(mesh->mNormals[v].x, mesh->mNormals[v].y, mesh->mNormals[v].z);
        }
        if (mesh->HasTextureCoords(0)) {
          tex_coords.emplace_back(mesh->mTextureCoords[0][v].x, mesh->mTextureCoords[0][v].y);
        }
      }

      for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
        auto face = mesh->mFaces[f];
        for (unsigned int j = 0; j < face.mNumIndices; j++) {
          indices.push_back(face.mIndices[j]);
        }
      }

      Mesh<> m(positions.size(), indices.size(), indices.data(),
               reinterpret_cast<grassland::Vector3<float> *>(positions.data()),
               mesh->HasNormals() ? reinterpret_cast<grassland::Vector3<float> *>(normals.data()) : nullptr,
               mesh->HasTextureCoords(0) ? reinterpret_cast<grassland::Vector2<float> *>(tex_coords.data()) : nullptr,
               nullptr);
      auto geom = std::make_unique<sparkium::GeometryMesh>(core, m);
      meshes.emplace_back(std::move(geom));
    }

    LoadEntities(core, scene, scene->mRootNode);
    importer.FreeScene();
  }

  void SetTransformation(const glm::mat4 &transform) {
    for (auto &[entity, local_transform] : entities) {
      entity->SetTransformation(transform * local_transform);
    }
  }

  void PutInScene(sparkium::Scene *scene) {
    for (auto &[entity, _] : entities) {
      scene->AddEntity(entity.get());
    }
  }

  void Clear() {
    entities.clear();
    meshes.clear();
    materials.clear();
  }
};

/**
 * @brief Area light helper class
 */
class AreaLight {
  const uint32_t indices[6] = {0, 2, 1, 0, 3, 2};
  const grassland::Vector3<float> vertices[4] = {
      {-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f}};

 public:
  AreaLight(sparkium::Core *core,
            const glm::vec3 &emission = {1.0f, 1.0f, 1.0f},
            float size = 1.0f,
            const glm::vec3 &position = {0.0f, 0.0f, 0.0f},
            const glm::vec3 &direction = {0.0f, 0.0f, 1.0f},
            const glm::vec3 &up = {0.0f, 1.0f, 0.0f})
      : light_(core, emission, false, false),
        emission(light_.emission),
        position(position),
        size(size),
        direction(direction),
        up(up) {
    mesh_ = std::make_unique<sparkium::GeometryMesh>(core, Mesh<>{4, 6, indices, vertices});
    entity_geometry_material_ = std::make_unique<sparkium::EntityGeometryMaterial>(core, mesh_.get(), &light_);
    Sync();
  }

  void Sync() {
    entity_geometry_material_->SetTransformation(glm::inverse(glm::lookAt(position, position + direction, up)) *
                                                 glm::scale(glm::mat4{1.0f}, glm::vec3{size}));
  }

  operator sparkium::Entity *() { return entity_geometry_material_.get(); }

  glm::vec3 &emission;
  float size{1.0f};
  glm::vec3 position{0.0f, 0.0f, 0.0f};
  glm::vec3 direction{0.0f, -1.0f, 0.0f};
  glm::vec3 up{0.0f, 1.0f, 0.0f};
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_geometry_material_;

 private:
  std::unique_ptr<sparkium::GeometryMesh> mesh_;
  sparkium::MaterialLight light_;
};

/**
 * @brief Joint information structure
 */
struct JointInfo {
  std::string name;
  float lower_bound{-3.14159f};
  float upper_bound{3.14159f};
  float value{0.0f};
};

// ============================================================================
// IK Solver with Sampling
// ============================================================================

/**
 * @brief Configuration for the IK solver
 */
struct IKSolverConfig {
  unsigned int max_iter = 100;
  double step_tol = 1e-10;
  double constraint_tol = 1e-10;
  int num_samples = 50;
  bool verbose = false;
};

/**
 * @brief Result of an IK solve attempt
 */
struct IKResult {
  bool success = false;
  bool within_limits = false;
  double error_norm = 0.0;
  unsigned int iterations = 0;
  double distance_from_start = 0.0;
  rbd::VectorX Q;
};

/**
 * @brief IK Solver with sampling strategy for finding solutions within joint limits
 */
class IKSolver {
public:
  IKSolver(rbd::Model& model, const std::vector<JointInfo>& joints)
      : model_(model), rng_(std::random_device{}()) {
    // Cache joint limits
    position_lower_.resize(joints.size());
    position_upper_.resize(joints.size());
    for (size_t i = 0; i < joints.size(); ++i) {
      position_lower_(i) = joints[i].lower_bound;
      position_upper_(i) = joints[i].upper_bound;
    }
  }

  /**
   * @brief Solve IK with full 6-DOF constraint using sampling
   */
  bool solveFullPoseWithSampling(
      const rbd::VectorX& Q_current,
      unsigned int tcp_body_id,
      const rbd::Vector3& target_pos,
      const rbd::Matrix3& target_rot,
      rbd::VectorX& Q_result,
      const IKSolverConfig& config = IKSolverConfig()) {
    
    std::vector<IKResult> valid_solutions;
    
    // First try with current configuration
    IKResult result = solveSingle(Q_current, tcp_body_id, target_pos, &target_rot, config);
    
    if (result.success && result.within_limits) {
      Q_result = result.Q;
      last_result_ = result;
      return true;
    }
    
    if (result.success) {
      result.distance_from_start = configurationDistance(Q_current, result.Q);
      valid_solutions.push_back(result);
    }
    
    // Try random samples
    for (int sample = 0; sample < config.num_samples; ++sample) {
      rbd::VectorX Q_init = generateRandomConfiguration();
      IKResult sample_result = solveSingle(Q_init, tcp_body_id, target_pos, &target_rot, config);
      
      if (sample_result.success) {
        sample_result.distance_from_start = configurationDistance(Q_current, sample_result.Q);
        
        if (sample_result.within_limits) {
          valid_solutions.push_back(sample_result);
        }
      }
    }
    
    // Select best solution (closest to current configuration and within limits)
    if (!valid_solutions.empty()) {
      IKResult* best_in_limits = nullptr;
      IKResult* best_overall = nullptr;
      double min_dist_in_limits = std::numeric_limits<double>::max();
      double min_dist_overall = std::numeric_limits<double>::max();
      
      for (auto& sol : valid_solutions) {
        if (sol.within_limits && sol.distance_from_start < min_dist_in_limits) {
          min_dist_in_limits = sol.distance_from_start;
          best_in_limits = &sol;
        }
        if (sol.distance_from_start < min_dist_overall) {
          min_dist_overall = sol.distance_from_start;
          best_overall = &sol;
        }
      }
      
      if (best_in_limits != nullptr) {
        Q_result = best_in_limits->Q;
        last_result_ = *best_in_limits;
        return true;
      }
      
      if (best_overall != nullptr) {
        Q_result = best_overall->Q;
        clampToLimits(Q_result);
        last_result_ = *best_overall;
        last_result_.within_limits = false;
        return false;
      }
    }
    
    // No solution found
    last_result_.success = false;
    return false;
  }

  /**
   * @brief Check if configuration is within joint limits
   */
  bool isWithinLimits(const rbd::VectorX& Q) const {
    for (int i = 0; i < Q.size() && i < position_lower_.size(); ++i) {
      if (Q(i) < position_lower_(i) || Q(i) > position_upper_(i)) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Clamp configuration to joint limits
   */
  bool clampToLimits(rbd::VectorX& Q) const {
    bool clamped = false;
    for (int i = 0; i < Q.size() && i < position_lower_.size(); ++i) {
      double old_val = Q(i);
      Q(i) = std::clamp(Q(i), position_lower_(i), position_upper_(i));
      if (std::abs(Q(i) - old_val) > 1e-10) {
        clamped = true;
      }
    }
    return clamped;
  }

  /**
   * @brief Generate random configuration within joint limits
   */
  rbd::VectorX generateRandomConfiguration() const {
    rbd::VectorX Q(model_.dof_count);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    
    for (unsigned int i = 0; i < model_.dof_count; ++i) {
      double lower = position_lower_(i);
      double upper = position_upper_(i);
      double range = upper - lower;
      Q(i) = lower + dist(rng_) * range;
    }
    
    return Q;
  }

  /**
   * @brief Compute distance between two configurations
   */
  double configurationDistance(const rbd::VectorX& Q1, const rbd::VectorX& Q2) const {
    return (Q1 - Q2).norm();
  }

  const IKResult& getLastResult() const { return last_result_; }

private:
  IKResult solveSingle(
      const rbd::VectorX& Q_init,
      unsigned int tcp_body_id,
      const rbd::Vector3& target_pos,
      const rbd::Matrix3* target_rot,
      const IKSolverConfig& config) {
    
    IKResult result;
    result.Q.resize(model_.dof_count);
    
    // Setup IK constraint
    rbd::IKConstraint ik_constraint;
    ik_constraint.max_iter = config.max_iter;
    ik_constraint.step_tol = config.step_tol;
    ik_constraint.constraint_tol = config.constraint_tol;
    
    if (target_rot != nullptr) {
      ik_constraint.AddFullConstraint(tcp_body_id, rbd::Vector3(0, 0, 0), target_pos, *target_rot);
    } else {
      ik_constraint.AddPointConstraint(tcp_body_id, rbd::Vector3(0, 0, 0), target_pos);
    }
    
    // Solve IK
    result.success = rbd::InverseKinematics(model_, Q_init, ik_constraint, result.Q);
    result.error_norm = ik_constraint.error_norm;
    result.iterations = ik_constraint.num_iter;
    
    if (result.success) {
      result.within_limits = isWithinLimits(result.Q);
      result.distance_from_start = configurationDistance(Q_init, result.Q);
    }
    
    return result;
  }

  rbd::Model& model_;
  rbd::VectorX position_lower_;
  rbd::VectorX position_upper_;
  mutable std::mt19937 rng_;
  IKResult last_result_;
};

/**
 * @brief Link visual information for rendering
 */
struct LinkVisualInfo {
  std::string link_name;
  int link_index;  // RBD body index (-1 for root)
  std::string mesh_path;
  glm::mat4 visual_origin;  // Transform from link frame to visual frame
  std::unique_ptr<CombinedMesh> mesh;
};

/**
 * @brief Convert URDF package:// URI to actual file path using FindAssetFile
 *
 * Extracts the mesh path from package://franka_description/meshes/... URIs
 * and uses FileProbe with pre-configured search paths to find the file.
 */
std::string resolvePackageUri(const std::string &uri) {
  const std::string package_prefix = "package://franka_description/";
  if (uri.substr(0, package_prefix.size()) == package_prefix) {
    // Remove "package://franka_description/" prefix
    // For URIs like "package://franka_description/meshes/robot_arms/fr3/visual/link0.dae"
    // extract "robot_arms/fr3/visual/link0.dae" (strip the "meshes/" part)
    std::string remaining = uri.substr(package_prefix.size());

    // Skip "meshes/" prefix if present
    const std::string meshes_prefix = "meshes/";
    if (remaining.substr(0, meshes_prefix.size()) == meshes_prefix) {
      remaining = remaining.substr(meshes_prefix.size());
    }

    // Use FileProbe to find the mesh (searches registered paths)
    std::string result = FindAssetFile(remaining);
    if (!result.empty()) {
      return result;
    }

    // Fallback to full path construction
    return std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/franka_fr3/meshes/" + remaining;
  }
  // If not a package URI, try to find it as an asset
  return FindAssetFile(uri);
}

/**
 * @brief Create glm::mat4 from xyz and rpy
 */
glm::mat4 xyzRpyToMat4(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy) {
  glm::mat4 t = glm::translate(glm::mat4(1.0f), glm::vec3(xyz.x(), xyz.y(), xyz.z()));
  glm::mat4 r = glm::rotate(glm::mat4(1.0f), static_cast<float>(rpy.z()), glm::vec3(0.0f, 0.0f, 1.0f)) *
                glm::rotate(glm::mat4(1.0f), static_cast<float>(rpy.y()), glm::vec3(0.0f, 1.0f, 0.0f)) *
                glm::rotate(glm::mat4(1.0f), static_cast<float>(rpy.x()), glm::vec3(1.0f, 0.0f, 0.0f));
  return t * r;
}

/**
 * @brief Convert SpatialTransform X_base to glm::mat4 homogeneous transformation
 */
glm::mat4 SpatialTransformToGLM(const rbd::SpatialTransform &X) {
  rbd::Matrix3 R = X.E.transpose();
  glm::mat4 result(1.0f);
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 3; row++) {
      result[col][row] = static_cast<float>(R(row, col));
    }
  }
  result[3][0] = static_cast<float>(X.r(0));
  result[3][1] = static_cast<float>(X.r(1));
  result[3][2] = static_cast<float>(X.r(2));
  return result;
}

/**
 * @brief Compute transform for a link using CalcBodyToBaseCoordinates and CalcBodyToBaseRotation
 * 
 * This handles both regular bodies and fixed bodies (those with link_index >= fixed_body_discriminator).
 * The Q vector must be passed in since it's needed for the kinematics functions.
 */
glm::mat4 computeLinkTransformFromRBD(int link_index, rbd::Model &rbd_model, const rbd::VectorX &Q) {
  // Handle invalid link_index (e.g., root link or unassigned)
  if (link_index < 0) {
    return glm::mat4(1.0f);
  }
  
  unsigned int body_id = static_cast<unsigned int>(link_index);
  
  // Use CalcBodyToBaseRotation and CalcBodyToBaseCoordinates which handle both
  // regular bodies and fixed bodies (body_id >= fixed_body_discriminator)
  rbd::Matrix3 E = rbd::CalcBodyToBaseRotation(rbd_model, Q, body_id, false);
  rbd::Vector3 r = rbd::CalcBodyToBaseCoordinates(rbd_model, Q, body_id, rbd::Vector3(0, 0, 0), false);
  
  // E is the rotation matrix in spatial convention (world to body)
  // E^T gives rotation from body frame to world frame
  rbd::Matrix3 R = E.transpose();
  
  glm::mat4 result(1.0f);
  // GLM uses column-major ordering
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 3; row++) {
      result[col][row] = static_cast<float>(R(row, col));
    }
  }
  result[3][0] = static_cast<float>(r(0));
  result[3][1] = static_cast<float>(r(1));
  result[3][2] = static_cast<float>(r(2));
  return result;
}

/**
 * @brief Read file contents to string
 */
std::string readFileToString(const std::string &filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    LogError("Failed to open file: " + filepath);
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

/**
 * @brief Collects movable joints (non-fixed) from URDF in depth-first order
 *
 * This traverses the kinematic tree and collects joints that contribute to DOFs
 * (revolute, continuous, prismatic joints). The order matches how the RBD model
 * assigns DOF indices.
 */
void collectMovableJointsRecursive(const std::shared_ptr<urdf::Link> &link,
                                    std::vector<std::shared_ptr<urdf::Joint>> &movable_joints) {
  for (size_t i = 0; i < link->child_joints.size(); ++i) {
    const auto &joint = link->child_joints[i];
    const auto &child_link = link->child_links[i];

    // Only non-fixed joints contribute to DOFs
    if (joint->type == urdf::JointType::REVOLUTE ||
        joint->type == urdf::JointType::CONTINUOUS ||
        joint->type == urdf::JointType::PRISMATIC) {
      movable_joints.push_back(joint);
    }

    // Continue traversing children
    collectMovableJointsRecursive(child_link, movable_joints);
  }
}

std::vector<std::shared_ptr<urdf::Joint>> collectMovableJoints(const std::shared_ptr<urdf::Model> &urdf_model) {
  std::vector<std::shared_ptr<urdf::Joint>> movable_joints;
  auto root = urdf_model->getRoot();
  if (root) {
    collectMovableJointsRecursive(root, movable_joints);
  }
  return movable_joints;
}

/**
 * @brief Find joint name for a given DOF index using movable joints list
 */
std::string findJointNameForDof(const std::vector<std::shared_ptr<urdf::Joint>> &movable_joints, int dof_index) {
  if (dof_index >= 0 && dof_index < static_cast<int>(movable_joints.size())) {
    return movable_joints[dof_index]->name;
  }
  return "Joint " + std::to_string(dof_index + 1);
}

/**
 * @brief Get joint limits from URDF for a given DOF index using movable joints list
 */
std::pair<float, float> getJointLimits(const std::vector<std::shared_ptr<urdf::Joint>> &movable_joints, int dof_index) {
  if (dof_index >= 0 && dof_index < static_cast<int>(movable_joints.size())) {
    const auto &joint = movable_joints[dof_index];
    if (joint->limits.has_value()) {
      auto limits = joint->limits.value();
      return {static_cast<float>(limits->lower), static_cast<float>(limits->upper)};
    }
  }
  return {-3.14159f, 3.14159f};  // Default limits
}

/**
 * @brief Collect all visual information from URDF links
 */
std::vector<LinkVisualInfo> collectLinkVisuals(const std::shared_ptr<urdf::Model> &urdf_model) {
  std::vector<LinkVisualInfo> visuals;

  for (const auto &[name, link] : urdf_model->linkMap) {
    for (const auto &visual : link->visuals) {
      if (visual->geometry.has_value()) {
        auto geom = visual->geometry.value();
        if (geom->type == urdf::GeometryType::MESH) {
          auto mesh_geom = std::dynamic_pointer_cast<urdf::Mesh>(geom);
          if (mesh_geom) {
            LinkVisualInfo info;
            info.link_name = name;
            info.link_index = link->link_index;
            info.mesh_path = resolvePackageUri(mesh_geom->filename);
            info.visual_origin = xyzRpyToMat4(visual->origin.xyz, visual->origin.rpy);
            info.mesh = std::make_unique<CombinedMesh>();
            visuals.push_back(std::move(info));
          }
        }
      }
    }
  }

  return visuals;
}

/**
 * @brief Update visual transforms from joint angles using forward kinematics
 * 
 * Uses CalcBodyToBaseCoordinates and CalcBodyToBaseRotation to compute transforms
 * for both regular bodies and fixed bodies (with link_index >= fixed_body_discriminator).
 */
void updateVisualsFromJoints(rbd::Model &rbd_model,
                             const std::shared_ptr<urdf::Model> &urdf_model,
                             std::vector<LinkVisualInfo> &visuals,
                             const rbd::VectorX &Q) {
  // Update kinematics first
  rbd::UpdateKinematicsCustom(rbd_model, &Q, nullptr, nullptr);

  // Update each visual's transform
  for (auto &visual : visuals) {
    // Compute the link's world transform using the RBD model
    // This handles both regular bodies and fixed bodies automatically
    glm::mat4 link_transform = computeLinkTransformFromRBD(visual.link_index, rbd_model, Q);

    // Apply visual origin offset
    glm::mat4 final_transform = link_transform * visual.visual_origin;
    if (visual.mesh) {
      visual.mesh->SetTransformation(final_transform);
    }
  }
}

/**
 * @brief Create a rotation matrix from roll, pitch, yaw (XYZ Euler angles)
 */
rbd::Matrix3 rpyToRotationMatrix(double roll, double pitch, double yaw) {
  double cy = cos(yaw), sy = sin(yaw);
  double cp = cos(pitch), sp = sin(pitch);
  double cr = cos(roll), sr = sin(roll);
  
  rbd::Matrix3 R;
  R(0,0) = cy*cp;  R(0,1) = cy*sp*sr - sy*cr;  R(0,2) = cy*sp*cr + sy*sr;
  R(1,0) = sy*cp;  R(1,1) = sy*sp*sr + cy*cr;  R(1,2) = sy*sp*cr - cy*sr;
  R(2,0) = -sp;    R(2,1) = cp*sr;             R(2,2) = cp*cr;
  
  return R;
}

/**
 * @brief Create glm::mat4 from position and rotation matrix
 */
glm::mat4 posRotToGLM(const rbd::Vector3 &pos, const rbd::Matrix3 &R) {
  glm::mat4 result(1.0f);
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 3; row++) {
      result[col][row] = static_cast<float>(R(row, col));
    }
  }
  result[3][0] = static_cast<float>(pos(0));
  result[3][1] = static_cast<float>(pos(1));
  result[3][2] = static_cast<float>(pos(2));
  return result;
}

/**
 * @brief Axis frame visualizer - 3 long-thin cubes representing XYZ axes
 */
class AxisFrame {
 public:
  AxisFrame(sparkium::Core *core, sparkium::GeometryMesh *cube_geom, float axis_length = 0.1f, float axis_thickness = 0.005f)
      : axis_length_(axis_length), axis_thickness_(axis_thickness) {
    // X axis - Red
    material_x_ = std::make_unique<sparkium::MaterialPrincipled>(core, glm::vec3{1.0f, 0.0f, 0.0f});
    material_x_->roughness = 0.5f;
    entity_x_ = std::make_unique<sparkium::EntityGeometryMaterial>(core, cube_geom, material_x_.get());
    
    // Y axis - Green
    material_y_ = std::make_unique<sparkium::MaterialPrincipled>(core, glm::vec3{0.0f, 1.0f, 0.0f});
    material_y_->roughness = 0.5f;
    entity_y_ = std::make_unique<sparkium::EntityGeometryMaterial>(core, cube_geom, material_y_.get());
    
    // Z axis - Blue
    material_z_ = std::make_unique<sparkium::MaterialPrincipled>(core, glm::vec3{0.0f, 0.0f, 1.0f});
    material_z_->roughness = 0.5f;
    entity_z_ = std::make_unique<sparkium::EntityGeometryMaterial>(core, cube_geom, material_z_.get());
    
    SetTransform(glm::mat4(1.0f));
  }
  
  void SetTransform(const glm::mat4 &frame_transform) {
    // X axis: scaled along X, offset to center on origin
    glm::mat4 x_local = glm::translate(glm::mat4(1.0f), glm::vec3(axis_length_ / 2.0f, 0.0f, 0.0f)) *
                        glm::scale(glm::mat4(1.0f), glm::vec3(axis_length_ / 2.0f, axis_thickness_, axis_thickness_));
    entity_x_->SetTransformation(frame_transform * x_local);
    
    // Y axis: scaled along Y, offset to center on origin
    glm::mat4 y_local = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, axis_length_ / 2.0f, 0.0f)) *
                        glm::scale(glm::mat4(1.0f), glm::vec3(axis_thickness_, axis_length_ / 2.0f, axis_thickness_));
    entity_y_->SetTransformation(frame_transform * y_local);
    
    // Z axis: scaled along Z, offset to center on origin
    glm::mat4 z_local = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, axis_length_ / 2.0f)) *
                        glm::scale(glm::mat4(1.0f), glm::vec3(axis_thickness_, axis_thickness_, axis_length_ / 2.0f));
    entity_z_->SetTransformation(frame_transform * z_local);
  }
  
  void AddToScene(sparkium::Scene *scene) {
    scene->AddEntity(entity_x_.get());
    scene->AddEntity(entity_y_.get());
    scene->AddEntity(entity_z_.get());
  }
  
 private:
  float axis_length_;
  float axis_thickness_;
  std::unique_ptr<sparkium::MaterialPrincipled> material_x_, material_y_, material_z_;
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_x_, entity_y_, entity_z_;
};

int main() {
  // Register FR3 robot assets directory in FileProbe
  std::string fr3_assets_dir = std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/franka_fr3/";
  FileProbe::GetInstance().AddSearchPath(fr3_assets_dir);
  FileProbe::GetInstance().AddSearchPath(fr3_assets_dir + "meshes/");

  // Load URDF
  std::string urdf_path = FindAssetFile("fr3_franka_hand.urdf");

  LogInfo("Loading URDF from: {}", urdf_path);

  std::string xml_string = readFileToString(urdf_path);
  if (xml_string.empty()) {
    LogError("Failed to read URDF file");
    return 1;
  }

  auto urdf_model = urdf::Model::fromXmlStr(xml_string);
  if (!urdf_model) {
    LogError("Failed to parse URDF");
    return 1;
  }

  LogInfo("URDF loaded successfully: {}", urdf_model->getName());
  LogInfo("Links: {}", urdf_model->linkMap.size());
  LogInfo("Joints: {}", urdf_model->jointMap.size());

  // Convert to RBD model
  rbd::Model rbd_model = urdf::toRBDModel(urdf_model);
  rbd_model.gravity = rbd::Vector3(0, 0, -9.81);

  LogInfo("RBD Model DOF count: {}", rbd_model.dof_count);
  LogInfo("RBD Model body count: {}", rbd_model.mBodies.size());

  // Collect movable joints for DOF mapping
  auto movable_joints = collectMovableJoints(urdf_model);
  LogInfo("Movable joints: {}", movable_joints.size());

  // Collect visual information
  std::vector<LinkVisualInfo> link_visuals = collectLinkVisuals(urdf_model);
  LogInfo("Found {} visual meshes to load", link_visuals.size());

  // Setup joint info with proper names and limits
  std::vector<JointInfo> joints(rbd_model.dof_count);
  for (unsigned int i = 0; i < rbd_model.dof_count; i++) {
    joints[i].name = findJointNameForDof(movable_joints, i);
    auto [lower, upper] = getJointLimits(movable_joints, i);
    joints[i].lower_bound = lower;
    joints[i].upper_bound = upper;
    joints[i].value = 0.0f;
    LogInfo("DOF {}: {} [{}, {}]", i, joints[i].name, lower, upper);
  }

  // Initialize graphics
  std::unique_ptr<graphics::Core> core_;
  graphics::CreateCore(graphics::BACKEND_API_DEFAULT, graphics::Core::Settings{}, &core_);
  core_->InitializeLogicalDeviceAutoSelect(false);
  sparkium::Core sparkium_core(core_.get());

  sparkium::Scene scene(&sparkium_core);
  scene.settings.samples_per_dispatch = 32;
  sparkium::Film film(&sparkium_core, 1280, 720);
  film.info.persistence = 0.98f;
  sparkium::Camera camera(
      &sparkium_core, glm::lookAt(glm::vec3{2.0f, -1.0f, 0.5f}, glm::vec3{0.0f, 0.0f, 0.5f}, glm::vec3{0.0, 0.0, 1.0}),
      glm::radians(30.0f), static_cast<float>(film.GetWidth()) / film.GetHeight());

  // Create basic geometries
  Mesh<> cube_mesh;
  cube_mesh.LoadObjFile(FindAssetFile("meshes/cube.obj"));
  sparkium::GeometryMesh geometry_sphere(&sparkium_core, Mesh<>::Sphere(30));
  sparkium::GeometryMesh geometry_cube(&sparkium_core, cube_mesh);

  // Ground
  sparkium::MaterialPrincipled material_ground(&sparkium_core, {0.1f, 0.2f, 0.4f});
  material_ground.roughness = 0.2f;
  material_ground.metallic = 0.0f;
  sparkium::EntityGeometryMaterial entity_ground(&sparkium_core, &geometry_cube, &material_ground,
                                                 glm::translate(glm::mat4{1.0f}, glm::vec3{0.0f, 0.0f, -1000.0f}) *
                                                     glm::scale(glm::mat4(1.0f), glm::vec3(1000.0f)));

  // Sky
  sparkium::MaterialLight material_sky(&sparkium_core, {0.8f, 0.8f, 0.8f}, true, false);
  sparkium::EntityGeometryMaterial entity_sky(
      &sparkium_core, &geometry_sphere, &material_sky,
      glm::translate(glm::mat4{1.0f}, glm::vec3{0.0f, 0.0f, 0.0f}) * glm::scale(glm::mat4(1.0f), glm::vec3(60.0f)));
  entity_sky.raster_light = false;
  scene.settings.raster.ambient_light = glm::vec3{0.5f, 0.5f, 0.5f};

  // Area light
  AreaLight area_light(&sparkium_core, glm::vec3{1.0f, 1.0f, 1.0f}, 1.0f, glm::vec3{40.0f, -30.0f, 30.0f},
                       glm::normalize(glm::vec3{-4.0f, 3.0f, -3.0f}), glm::vec3{0.0f, 0.0f, 1.0f});
  area_light.emission = glm::vec3{1000.0f};

  scene.AddEntity(&entity_ground);
  scene.AddEntity(&entity_sky);
  scene.AddEntity(area_light);

  // Load visual meshes
  LogInfo("Loading visual meshes...");
  for (auto &visual : link_visuals) {
    LogInfo("  Loading: {} for link: {}", visual.mesh_path, visual.link_name);
    visual.mesh->LoadModel(&sparkium_core, visual.mesh_path);
    visual.mesh->PutInScene(&scene);
  }
  LogInfo("Mesh loading complete.");

  // Get the fr3_hand_tcp link index for IK
  auto hand_tcp_link = urdf_model->getLink("fr3_hand_tcp");
  if (!hand_tcp_link) {
    LogError("Could not find fr3_hand_tcp link!");
    return 1;
  }
  unsigned int hand_tcp_body_id = static_cast<unsigned int>(hand_tcp_link->link_index);
  LogInfo("fr3_hand_tcp body_id: {}", hand_tcp_body_id);

  // Create axis frame visualizers
  AxisFrame target_frame(&sparkium_core, &geometry_cube, 0.15f, 0.008f);  // Target - larger, thicker
  AxisFrame current_frame(&sparkium_core, &geometry_cube, 0.1f, 0.004f);  // Current - smaller, thinner
  target_frame.AddToScene(&scene);
  current_frame.AddToScene(&scene);

  // Create window
  std::unique_ptr<graphics::Image> srgb_image;
  core_->CreateImage(film.GetWidth(), film.GetHeight(), graphics::IMAGE_FORMAT_R8G8B8A8_UNORM, &srgb_image);

  std::unique_ptr<graphics::Window> window;
  core_->CreateWindowObject(film.GetWidth(), film.GetHeight(), "Inverse Kinematics Demo", &window);
  FPSCounter fps_counter;

  // Initialize joint positions to zero
  rbd::VectorX Q(rbd_model.dof_count);
  Q.setZero();
  
  // Get initial end-effector pose for target initialization
  rbd::UpdateKinematicsCustom(rbd_model, &Q, nullptr, nullptr);
  rbd::Vector3 init_pos = rbd::CalcBodyToBaseCoordinates(rbd_model, Q, hand_tcp_body_id, rbd::Vector3(0, 0, 0), false);
  
  // Target pose controls (6 DOF: xyz + rpy)
  float target_x = static_cast<float>(init_pos(0));
  float target_y = static_cast<float>(init_pos(1));
  float target_z = static_cast<float>(init_pos(2));
  float target_roll = 0.0f;
  float target_pitch = 0.0f;
  float target_yaw = 0.0f;

  LogInfo("Initial end-effector position: [{}, {}, {}]", target_x, target_y, target_z);

  // IK settings
  bool auto_solve_ik = true;
  bool ik_success = false;
  bool ik_within_limits = false;
  int ik_iterations = 0;
  double ik_error = 0.0;
  int ik_num_samples = 20;
  bool use_sampling = false;

  // Create IK solver
  IKSolver ik_solver(rbd_model, joints);

  bool ray_tracing = false;
  window->InitImGui(nullptr, 20.0f);

  while (!window->ShouldClose()) {
    window->BeginImGuiFrame();

    if (ImGui::Begin("Inverse Kinematics Control", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Checkbox("Ray Tracing", &ray_tracing);
      if (ray_tracing && !core_->DeviceRayTracingSupport()) {
        ImGui::Text("Ray Tracing not supported on this device!");
      }

      ImGui::Separator();
      ImGui::Text("Robot: %s", urdf_model->getName().c_str());
      ImGui::Text("DOF: %d", rbd_model.dof_count);

      ImGui::Separator();
      ImGui::Text("Target Pose (6 DOF)");
      
      // Position sliders
      ImGui::Text("Position:");
      ImGui::SliderFloat("X", &target_x, -1.0f, 1.0f);
      ImGui::SliderFloat("Y", &target_y, -1.0f, 1.0f);
      ImGui::SliderFloat("Z", &target_z, 0.0f, 1.5f);
      
      // Orientation sliders (in degrees for display, convert to radians internally)
      ImGui::Text("Orientation (degrees):");
      float roll_deg = target_roll * 180.0f / 3.14159f;
      float pitch_deg = target_pitch * 180.0f / 3.14159f;
      float yaw_deg = target_yaw * 180.0f / 3.14159f;
      
      if (ImGui::SliderFloat("Roll", &roll_deg, -180.0f, 180.0f)) {
        target_roll = roll_deg * 3.14159f / 180.0f;
      }
      if (ImGui::SliderFloat("Pitch", &pitch_deg, -180.0f, 180.0f)) {
        target_pitch = pitch_deg * 3.14159f / 180.0f;
      }
      if (ImGui::SliderFloat("Yaw", &yaw_deg, -180.0f, 180.0f)) {
        target_yaw = yaw_deg * 3.14159f / 180.0f;
      }

      ImGui::Separator();
      ImGui::Checkbox("Auto Solve IK", &auto_solve_ik);
      ImGui::Checkbox("Use Sampling", &use_sampling);
      if (use_sampling) {
        ImGui::SliderInt("Num Samples", &ik_num_samples, 1, 100);
      }
      
      if (ImGui::Button("Solve IK")) {
        auto_solve_ik = false;  // Manual solve
      }
      
      ImGui::Separator();
      ImGui::Text("IK Status:");
      ImGui::Text("Success: %s", ik_success ? "Yes" : "No");
      ImGui::Text("Within Limits: %s", ik_within_limits ? "Yes" : "No");
      ImGui::Text("Iterations: %d", ik_iterations);
      ImGui::Text("Error: %.6f", ik_error);
      
      ImGui::Separator();
      if (ImGui::Button("Reset Target to Current")) {
        rbd::Vector3 current_pos = rbd::CalcBodyToBaseCoordinates(rbd_model, Q, hand_tcp_body_id, rbd::Vector3(0, 0, 0), false);
        target_x = static_cast<float>(current_pos(0));
        target_y = static_cast<float>(current_pos(1));
        target_z = static_cast<float>(current_pos(2));
        // Keep current orientation
        rbd::Matrix3 current_rot = rbd::CalcBodyToBaseRotation(rbd_model, Q, hand_tcp_body_id, false);
        // Extract RPY from rotation matrix (approximate)
        target_pitch = -asin(current_rot(2, 0));
        target_roll = atan2(current_rot(2, 1), current_rot(2, 2));
        target_yaw = atan2(current_rot(1, 0), current_rot(0, 0));
      }
      
      if (ImGui::Button("Reset Joints to Zero")) {
        Q.setZero();
      }
    }
    ImGui::End();
    window->EndImGuiFrame();

    // Solve IK if enabled
    if (auto_solve_ik || ImGui::IsItemClicked()) {
      // Build target pose
      rbd::Vector3 target_pos(target_x, target_y, target_z);
      rbd::Matrix3 target_rot = rpyToRotationMatrix(target_roll, target_pitch, target_yaw);
      
      rbd::VectorX Q_result(rbd_model.dof_count);
      
      if (use_sampling) {
        // Use sampling-based IK solver for better joint limit handling
        IKSolverConfig config;
        config.max_iter = 100;
        config.step_tol = 1e-10;
        config.constraint_tol = 1e-10;
        config.num_samples = ik_num_samples;
        
        ik_success = ik_solver.solveFullPoseWithSampling(Q, hand_tcp_body_id, target_pos, target_rot, Q_result, config);
        
        const auto& result = ik_solver.getLastResult();
        ik_within_limits = result.within_limits;
        ik_iterations = static_cast<int>(result.iterations);
        ik_error = result.error_norm;
      } else {
        // Use simple IK solver without sampling
        rbd::IKConstraint ik_constraint;
        ik_constraint.max_iter = 100;
        ik_constraint.step_tol = 1e-10;
        ik_constraint.constraint_tol = 1e-10;
        
        ik_constraint.AddFullConstraint(hand_tcp_body_id, rbd::Vector3(0, 0, 0), target_pos, target_rot);
        
        ik_success = rbd::InverseKinematics(rbd_model, Q, ik_constraint, Q_result);
        ik_within_limits = ik_solver.isWithinLimits(Q_result);
        ik_iterations = static_cast<int>(ik_constraint.num_iter);
        ik_error = ik_constraint.error_norm;
      }
      
      // Only use first 7 DOF (arm joints), keep gripper joints unchanged
      for (int i = 0; i < 7 && i < static_cast<int>(rbd_model.dof_count); ++i) {
        Q(i) = Q_result(i);
      }
    }

    // Update visual transforms
    updateVisualsFromJoints(rbd_model, urdf_model, link_visuals, Q);

    // Update target frame visualization
    // rpyToRotationMatrix returns body-to-world rotation, but for visualization
    // we need to transpose to match the world-to-body convention used by GLM
    rbd::Vector3 target_pos(target_x, target_y, target_z);
    rbd::Matrix3 target_rot = rpyToRotationMatrix(target_roll, target_pitch, target_yaw);
    glm::mat4 target_transform = posRotToGLM(target_pos, target_rot.transpose());
    target_frame.SetTransform(target_transform);
    
    // Update current frame visualization (actual end-effector pose)
    // CalcBodyToBaseRotation returns E (world-to-body), transpose to get body-to-world
    rbd::UpdateKinematicsCustom(rbd_model, &Q, nullptr, nullptr);
    rbd::Vector3 current_pos = rbd::CalcBodyToBaseCoordinates(rbd_model, Q, hand_tcp_body_id, rbd::Vector3(0, 0, 0), false);
    rbd::Matrix3 current_rot = rbd::CalcBodyToBaseRotation(rbd_model, Q, hand_tcp_body_id, false);
    glm::mat4 current_transform = posRotToGLM(current_pos, current_rot.transpose());
    current_frame.SetTransform(current_transform);

    // Render
    sparkium_core.Render(&scene, &camera, &film,
                         ray_tracing ? sparkium::RENDER_PIPELINE_AUTO : sparkium::RENDER_PIPELINE_RASTERIZATION);
    film.Develop(srgb_image.get());

    std::unique_ptr<graphics::CommandContext> cmd_context;
    core_->CreateCommandContext(&cmd_context);
    cmd_context->CmdPresent(window.get(), srgb_image.get());
    core_->SubmitCommandContext(cmd_context.get());

    glfwPollEvents();

    float fps = fps_counter.TickFPS();
    char fps_buf[32];
    sprintf(fps_buf, "%.2f", fps);
    float rps = film.GetWidth() * film.GetHeight() * fps * scene.settings.samples_per_dispatch;
    char rps_buf[32];
    sprintf(rps_buf, "%.2f", rps * 1e-6f);
    
    std::string ik_status = ik_success ? (ik_within_limits ? " [IK OK]" : " [IK OK, Out of Limits]") : " [IK FAIL]";
    window->SetTitle(std::string("Inverse Kinematics - ") + fps_buf + " fps - " + rps_buf + " Mrays/s" + ik_status);
  }

  // Cleanup
  for (auto &visual : link_visuals) {
    if (visual.mesh) {
      visual.mesh->Clear();
    }
  }

  // Save final frame
  film.Develop(srgb_image.get());
  std::vector<uint8_t> image_data(film.GetWidth() * film.GetHeight() * 4);
  srgb_image->DownloadData(image_data.data());
  stbi_write_bmp("inverse_kinematics_output.bmp", film.GetWidth(), film.GetHeight(), 4, image_data.data());

  return 0;
}
