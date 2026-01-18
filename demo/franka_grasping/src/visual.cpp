/**
 * @file visual.cpp
 * @brief Rendering utilities implementation
 */

#include "visual.h"
#include "config.h"
#include "grassland/util/log.h"
#include "grassland/math/math_mesh.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

using grassland::LogInfo;
using grassland::LogWarning;
using grassland::LogError;
using grassland::Mesh;

namespace franka::visual {

// ============================================================================
// CombinedMesh
// ============================================================================

bool CombinedMesh::loadModel(sparkium::Core* core, const std::string& path) {
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    LogError("Failed to load model: {}", path);
    return false;
  }

  // Load materials
  for (unsigned int i = 0; i < scene->mNumMaterials; ++i) {
    auto material = scene->mMaterials[i];
    aiColor4D color;
    material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
    float reflectivity = 0.0f;
    material->Get(AI_MATKEY_REFLECTIVITY, reflectivity);

    auto mat = std::make_unique<sparkium::MaterialPrincipled>(
        core, glm::vec3{color.r, color.g, color.b});
    mat->specular = reflectivity;
    mat->roughness = reflectivity;
    materials_.emplace_back(std::move(mat));
  }

  // Load meshes
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    auto mesh = scene->mMeshes[i];
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> tex_coords;
    std::vector<uint32_t> indices;

    for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
      positions.emplace_back(
          mesh->mVertices[v].x,
          mesh->mVertices[v].y,
          mesh->mVertices[v].z);

      if (mesh->HasNormals()) {
        normals.emplace_back(
            mesh->mNormals[v].x,
            mesh->mNormals[v].y,
            mesh->mNormals[v].z);
      }

      if (mesh->HasTextureCoords(0)) {
        tex_coords.emplace_back(
            mesh->mTextureCoords[0][v].x,
            mesh->mTextureCoords[0][v].y);
      }
    }

    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
      auto face = mesh->mFaces[f];
      for (unsigned int j = 0; j < face.mNumIndices; ++j) {
        indices.push_back(face.mIndices[j]);
      }
    }

    Mesh<> m(
        positions.size(), indices.size(), indices.data(),
        reinterpret_cast<grassland::Vector3<float>*>(positions.data()),
        mesh->HasNormals() ? reinterpret_cast<grassland::Vector3<float>*>(normals.data()) : nullptr,
        mesh->HasTextureCoords(0) ? reinterpret_cast<grassland::Vector2<float>*>(tex_coords.data()) : nullptr,
        nullptr);

    auto geom = std::make_unique<sparkium::GeometryMesh>(core, m);
    meshes_.emplace_back(std::move(geom));
  }

  // Load entities
  loadEntities(core, scene, scene->mRootNode);

  importer.FreeScene();
  LogInfo("  Loaded mesh: {} submeshes, {} materials", meshes_.size(), materials_.size());
  return true;
}

void CombinedMesh::loadEntities(
    sparkium::Core* core,
    const aiScene* scene,
    const aiNode* node,
    const glm::mat4 transformation) {

  glm::mat4 local_transform{
      node->mTransformation.a1, node->mTransformation.b1, node->mTransformation.c1, node->mTransformation.d1,
      node->mTransformation.a2, node->mTransformation.b2, node->mTransformation.c2, node->mTransformation.d2,
      node->mTransformation.a3, node->mTransformation.b3, node->mTransformation.c3, node->mTransformation.d3,
      node->mTransformation.a4, node->mTransformation.b4, node->mTransformation.c4, node->mTransformation.d4};

  if (scene->mRootNode == node) {
    local_transform = glm::mat4{1.0f};
  }
  local_transform = transformation * local_transform;

  for (unsigned int i = 0; i < node->mNumChildren; ++i) {
    loadEntities(core, scene, node->mChildren[i], local_transform);
  }

  for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
    int mesh_index = node->mMeshes[i];
    auto& mesh = meshes_[mesh_index];
    auto& material = materials_[scene->mMeshes[mesh_index]->mMaterialIndex];
    auto entity = std::make_unique<sparkium::EntityGeometryMaterial>(
        core, mesh.get(), material.get(), local_transform);
    entities_.emplace_back(std::move(entity), local_transform);
  }
}

void CombinedMesh::setTransformation(const glm::mat4& transform) {
  for (auto& [entity, local_transform] : entities_) {
    entity->SetTransformation(transform * local_transform);
  }
}

void CombinedMesh::putInScene(sparkium::Scene* scene) {
  for (auto& [entity, _] : entities_) {
    scene->AddEntity(entity.get());
  }
}

void CombinedMesh::clear() {
  entities_.clear();
  meshes_.clear();
  materials_.clear();
}

// ============================================================================
// AreaLight
// ============================================================================

AreaLight::AreaLight(
    sparkium::Core* core,
    const glm::vec3& emission,
    float size,
    const glm::vec3& position,
    const glm::vec3& direction,
    const glm::vec3& up)
    : emission(emission),
      size(size),
      position(position),
      direction(direction),
      up(up) {

  const uint32_t indices[6] = {0, 2, 1, 0, 3, 2};
  const grassland::Vector3<float> vertices[4] = {
      {-1.0f, -1.0f, 0.0f},
      {1.0f, -1.0f, 0.0f},
      {1.0f, 1.0f, 0.0f},
      {-1.0f, 1.0f, 0.0f}};

  light_ = std::make_unique<sparkium::MaterialLight>(core, emission, false, false);
  mesh_ = std::make_unique<sparkium::GeometryMesh>(core, Mesh<>{4, 6, indices, vertices});
  entity_geometry_material_ = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, mesh_.get(), light_.get());

  sync();
}

void AreaLight::sync() {
  entity_geometry_material_->SetTransformation(
      glm::inverse(glm::lookAt(position, position + direction, up)) *
      glm::scale(glm::mat4{1.0f}, glm::vec3{size}));
}

// ============================================================================
// AxisFrame
// ============================================================================

AxisFrame::AxisFrame(
    sparkium::Core* core,
    sparkium::GeometryMesh* cube_geom,
    float axis_length,
    float axis_thickness)
    : axis_length_(axis_length),
      axis_thickness_(axis_thickness) {

  // X axis - Red
  material_x_ = std::make_unique<sparkium::MaterialPrincipled>(
      core, glm::vec3{1.0f, 0.0f, 0.0f});
  material_x_->roughness = 0.5f;
  entity_x_ = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, cube_geom, material_x_.get());

  // Y axis - Green
  material_y_ = std::make_unique<sparkium::MaterialPrincipled>(
      core, glm::vec3{0.0f, 1.0f, 0.0f});
  material_y_->roughness = 0.5f;
  entity_y_ = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, cube_geom, material_y_.get());

  // Z axis - Blue
  material_z_ = std::make_unique<sparkium::MaterialPrincipled>(
      core, glm::vec3{0.0f, 0.0f, 1.0f});
  material_z_->roughness = 0.5f;
  entity_z_ = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, cube_geom, material_z_.get());

  setTransform(glm::mat4(1.0f));
}

void AxisFrame::setTransform(const glm::mat4& frame_transform) {
  glm::mat4 x_local = glm::translate(glm::mat4(1.0f), glm::vec3(axis_length_ / 2.0f, 0.0f, 0.0f)) *
                      glm::scale(glm::mat4(1.0f), glm::vec3(axis_length_ / 2.0f, axis_thickness_, axis_thickness_));
  entity_x_->SetTransformation(frame_transform * x_local);

  glm::mat4 y_local = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, axis_length_ / 2.0f, 0.0f)) *
                      glm::scale(glm::mat4(1.0f), glm::vec3(axis_thickness_, axis_length_ / 2.0f, axis_thickness_));
  entity_y_->SetTransformation(frame_transform * y_local);

  glm::mat4 z_local = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, axis_length_ / 2.0f)) *
                      glm::scale(glm::mat4(1.0f), glm::vec3(axis_thickness_, axis_thickness_, axis_length_ / 2.0f));
  entity_z_->SetTransformation(frame_transform * z_local);
}

void AxisFrame::addToScene(sparkium::Scene* scene) {
  scene->AddEntity(entity_x_.get());
  scene->AddEntity(entity_y_.get());
  scene->AddEntity(entity_z_.get());
}

// ============================================================================
// RobotVisualizer
// ============================================================================

RobotVisualizer::RobotVisualizer(
    sparkium::Core* core,
    sparkium::Scene* scene,
    sparkium::GeometryMesh* cube_geom)
    : core_(core),
      scene_(scene),
      target_frame_(core, cube_geom, config::TARGET_FRAME_LENGTH, config::TARGET_FRAME_THICKNESS),
      current_frame_(core, cube_geom, config::CURRENT_FRAME_LENGTH, config::CURRENT_FRAME_THICKNESS) {

  target_frame_.addToScene(scene);
  current_frame_.addToScene(scene);
}

bool RobotVisualizer::loadVisuals(std::vector<LinkVisualInfo>& visuals) {
  visuals_ = std::move(visuals);

  for (auto& visual : visuals_) {
    visual.mesh = new CombinedMesh();  // Non-owning pointer pattern

    if (!visual.mesh->loadModel(core_, visual.mesh_path)) {
      LogWarning("Failed to load mesh: {}", visual.mesh_path);
      continue;
    }

    visual.mesh->putInScene(scene_);
  }

  LogInfo("RobotVisualizer: loaded {} meshes", visuals_.size());
  return true;
}

void RobotVisualizer::update(
    contradium::rbd::Model& rbd_model,
    const std::shared_ptr<::urdf::Model>& urdf_model,
    const contradium::rbd::VectorX& Q) {

  // Update link visuals
  urdf_utils::updateVisuals(rbd_model, urdf_model, visuals_, Q);

  // Update each mesh's transform
  for (auto& visual : visuals_) {
    if (visual.mesh) {
      glm::mat4 link_transform = urdf_utils::computeLinkTransform(
          visual.link_index, rbd_model, Q);
      glm::mat4 final_transform = link_transform * visual.visual_origin;
      visual.mesh->setTransformation(final_transform);
    }
  }
}

RobotVisualizer::~RobotVisualizer() {
  for (auto& visual : visuals_) {
    if (visual.mesh) {
      delete visual.mesh;
      visual.mesh = nullptr;
    }
  }
}

// ============================================================================
// SceneSetup
// ============================================================================

void SceneSetup::setup(
    sparkium::Core* core,
    sparkium::Scene* scene,
    sparkium::GeometryMesh* sphere_geom,
    sparkium::GeometryMesh* cube_geom) {

  // Ground
  material_ground_ = std::make_unique<sparkium::MaterialPrincipled>(core, glm::vec3{0.2f, 0.3f, 0.2f});
  material_ground_->roughness = 0.8f;
  material_ground_->metallic = 0.0f;
  entity_ground_ = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, cube_geom, material_ground_.get(),
      glm::translate(glm::mat4{1.0f}, glm::vec3{0.0f, 0.0f, config::GROUND_Z}) *
          glm::scale(glm::mat4(1.0f), glm::vec3(config::GROUND_SCALE)));

  // Sky
  material_sky_ = std::make_unique<sparkium::MaterialLight>(
      core, glm::vec3{0.6f, 0.7f, 0.8f}, true, false);
  entity_sky_ = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, sphere_geom, material_sky_.get(),
      glm::translate(glm::mat4{1.0f}, glm::vec3{0.0f, 0.0f, 0.0f}) *
          glm::scale(glm::mat4(1.0f), glm::vec3(config::SKY_RADIUS)));
  entity_sky_->raster_light = false;

  scene->settings.raster.ambient_light = config::AMBIENT_LIGHT;
  scene->settings.samples_per_dispatch = config::SAMPLES_PER_DISPATCH;

  // Area light
  area_light_ = std::make_unique<AreaLight>(
      core, config::AREA_LIGHT_EMISSION, config::AREA_LIGHT_SIZE,
      config::AREA_LIGHT_POSITION,
      glm::normalize(glm::vec3{-4.0f, 3.0f, -3.0f}),
      glm::vec3{0.0f, 0.0f, 1.0f});

  scene->AddEntity(entity_ground_.get());
  scene->AddEntity(entity_sky_.get());
  scene->AddEntity(*area_light_);
}

SceneSetup::EntityWithMaterial SceneSetup::createBall(
    sparkium::Core* core,
    sparkium::GeometryMesh* sphere_geom,
    const glm::vec3& position,
    float radius,
    const glm::vec3& color) {

  EntityWithMaterial result;
  result.material = std::make_unique<sparkium::MaterialPrincipled>(core, color);
  result.material->roughness = 0.3f;
  result.material->metallic = 0.0f;

  result.entity = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, sphere_geom, result.material.get(),
      glm::translate(glm::mat4{1.0f}, position) *
          glm::scale(glm::mat4(1.0f), glm::vec3(radius)));

  return result;
}

SceneSetup::EntityWithMaterial SceneSetup::createGoalMarker(
    sparkium::Core* core,
    sparkium::GeometryMesh* cube_geom,
    const glm::vec3& position,
    float size,
    const glm::vec3& color) {

  EntityWithMaterial result;
  result.material = std::make_unique<sparkium::MaterialPrincipled>(core, color);
  result.material->roughness = 0.8f;
  result.material->metallic = 0.0f;

  result.entity = std::make_unique<sparkium::EntityGeometryMaterial>(
      core, cube_geom, result.material.get(),
      glm::translate(glm::mat4{1.0f}, position) *
          glm::scale(glm::mat4(1.0f), glm::vec3(size, size, config::GOAL_MARKER_HEIGHT)));

  return result;
}

} // namespace franka::visual
