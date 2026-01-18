/**
 * @file visual.h
 * @brief Rendering utilities for the grasping demo
 *
 * Provides mesh loading, area lights, axis frames, and robot visualization.
 */

#pragma once

#include "types.h"
#include "urdf_utils.h"
#include "long_march.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include <vector>

// Forward declarations for Assimp types
struct aiScene;
struct aiNode;

namespace franka::visual {

/**
 * @brief Combined mesh structure for loading and rendering DAE/mesh files
 *
 * Handles complex mesh files with multiple submeshes and materials.
 */
class CombinedMesh {
public:
  CombinedMesh() = default;
  ~CombinedMesh() = default;

  /**
   * @brief Load model from file
   * @param core Sparkium core
   * @param path Path to mesh file
   * @return true if successful
   */
  bool loadModel(sparkium::Core* core, const std::string& path);

  /**
   * @brief Set transformation for all entities
   */
  void setTransformation(const glm::mat4& transform);

  /**
   * @brief Add all entities to scene
   */
  void putInScene(sparkium::Scene* scene);

  /**
   * @brief Clear all resources
   */
  void clear();

private:
  void loadEntities(
      sparkium::Core* core,
      const aiScene* scene,
      const aiNode* node,
      const glm::mat4 transformation = glm::mat4{1.0f});

  std::vector<std::unique_ptr<sparkium::GeometryMesh>> meshes_;
  std::vector<std::unique_ptr<sparkium::MaterialPrincipled>> materials_;
  std::vector<std::pair<std::unique_ptr<sparkium::EntityGeometryMaterial>, glm::mat4>> entities_;
};

/**
 * @brief Area light helper class
 *
 * Creates a rectangular area light that can be positioned in the scene.
 */
class AreaLight {
public:
  AreaLight(
      sparkium::Core* core,
      const glm::vec3& emission = glm::vec3{1.0f, 1.0f, 1.0f},
      float size = 1.0f,
      const glm::vec3& position = glm::vec3{0.0f, 0.0f, 0.0f},
      const glm::vec3& direction = glm::vec3{0.0f, -1.0f, 0.0f},
      const glm::vec3& up = glm::vec3{0.0f, 0.0f, 1.0f});

  /**
   * @brief Update transformation based on current position/direction
   */
  void sync();

  /**
   * @brief Get entity pointer for adding to scene
   */
  operator sparkium::Entity*() { return entity_geometry_material_.get(); }

  glm::vec3 emission;
  float size;
  glm::vec3 position;
  glm::vec3 direction;
  glm::vec3 up;

private:
  std::unique_ptr<sparkium::GeometryMesh> mesh_;
  std::unique_ptr<sparkium::MaterialLight> light_;
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_geometry_material_;
};

/**
 * @brief Axis frame visualizer
 *
 * Renders three colored cylinders representing X (red), Y (green), Z (blue) axes.
 */
class AxisFrame {
public:
  AxisFrame(
      sparkium::Core* core,
      sparkium::GeometryMesh* cube_geom,
      float axis_length = 0.1f,
      float axis_thickness = 0.005f);

  /**
   * @brief Set transform for all axes
   */
  void setTransform(const glm::mat4& frame_transform);

  /**
   * @brief Add all axes to scene
   */
  void addToScene(sparkium::Scene* scene);

private:
  float axis_length_;
  float axis_thickness_;

  std::unique_ptr<sparkium::MaterialPrincipled> material_x_;
  std::unique_ptr<sparkium::MaterialPrincipled> material_y_;
  std::unique_ptr<sparkium::MaterialPrincipled> material_z_;

  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_x_;
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_y_;
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_z_;
};

/**
 * @brief Robot visualizer
 *
 * Manages loading and updating of robot link meshes.
 */
class RobotVisualizer {
public:
  RobotVisualizer(
      sparkium::Core* core,
      sparkium::Scene* scene,
      sparkium::GeometryMesh* cube_geom);

  /**
   * @brief Load all visual meshes
   * @param visuals Visual information from URDF
   * @return true if all meshes loaded successfully
   */
  bool loadVisuals(std::vector<LinkVisualInfo>& visuals);

  /**
   * @brief Update visual transforms from joint angles
   */
  void update(
      contradium::rbd::Model& rbd_model,
      const std::shared_ptr<::urdf::Model>& urdf_model,
      const contradium::rbd::VectorX& Q);

  /**
   * @brief Get axis frames for TCP visualization
   */
  AxisFrame& getTargetFrame() { return target_frame_; }
  AxisFrame& getCurrentFrame() { return current_frame_; }

  /**
   * @brief Destructor - cleans up allocated meshes
   */
  ~RobotVisualizer();

private:
  sparkium::Core* core_;
  sparkium::Scene* scene_;
  std::vector<LinkVisualInfo> visuals_;
  AxisFrame target_frame_;
  AxisFrame current_frame_;
};

/**
 * @brief Scene setup helper
 *
 * Creates and owns standard scene elements (ground, sky, lights).
 * Must remain alive while scene is in use.
 */
class SceneSetup {
public:
  SceneSetup() = default;
  ~SceneSetup() = default;

  /**
   * @brief Create standard scene elements and add to scene
   */
  void setup(
      sparkium::Core* core,
      sparkium::Scene* scene,
      sparkium::GeometryMesh* sphere_geom,
      sparkium::GeometryMesh* cube_geom);

  /**
   * @brief Create ball entity with material (caller owns the EntityWithMaterial)
   */
  struct EntityWithMaterial {
    std::unique_ptr<sparkium::MaterialPrincipled> material;
    std::unique_ptr<sparkium::EntityGeometryMaterial> entity;

    sparkium::EntityGeometryMaterial* get() { return entity.get(); }
    void SetTransformation(const glm::mat4& t) { entity->SetTransformation(t); }
  };

  static EntityWithMaterial createBall(
      sparkium::Core* core,
      sparkium::GeometryMesh* sphere_geom,
      const glm::vec3& position = glm::vec3{0.4f, 0.0f, 0.0175f},
      float radius = 0.0175f,
      const glm::vec3& color = glm::vec3{0.9f, 0.2f, 0.2f});

  /**
   * @brief Create goal marker entity with material (caller owns the EntityWithMaterial)
   */
  static EntityWithMaterial createGoalMarker(
      sparkium::Core* core,
      sparkium::GeometryMesh* cube_geom,
      const glm::vec3& position = glm::vec3{0.4f, 0.4f, 0.001f},
      float size = 0.05f,
      const glm::vec3& color = glm::vec3{0.2f, 0.9f, 0.2f});

private:
  // Ground
  std::unique_ptr<sparkium::MaterialPrincipled> material_ground_;
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_ground_;

  // Sky
  std::unique_ptr<sparkium::MaterialLight> material_sky_;
  std::unique_ptr<sparkium::EntityGeometryMaterial> entity_sky_;

  // Area light
  std::unique_ptr<AreaLight> area_light_;
};

} // namespace franka::visual
