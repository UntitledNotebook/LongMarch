#include <long_march.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <boost/numeric/odeint.hpp>
#include "glm/gtc/matrix_transform.hpp"
#include "stb_image_write.h"

using namespace long_march;
namespace rbd = contradium::rbd;
namespace odeint = boost::numeric::odeint;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------
constexpr double kAB = 2.0;              // horizontal edge of slope (X direction)
constexpr double kBC = 0.5;              // vertical edge of slope (Y direction)
constexpr double kDepth = 0.4;           // depth of slopes (Z direction)
constexpr double kCubeSize = 0.15;       // size of the cube on top
constexpr double kSlopeMass = 1.0;       // mass of each slope
constexpr double kCubeMass = 0.5;        // mass of the cube
constexpr double kGravityDefault = 0.5; // m/s^2 (points to -Y)
constexpr double kSimDt = 0.001;         // physics tick (1000 Hz for stability)

// Derived geometric parameters
const double kAC = std::sqrt(kAB * kAB + kBC * kBC);  // inclined surface length
const double kTheta = std::atan2(kBC, kAB);           // angle of inclination
const double kCosTheta = kAB / kAC;
const double kSinTheta = kBC / kAC;

using state_type = std::array<double, 6>;  // [q0, q1, q2, qdot0, qdot1, qdot2]

struct SlopeState {
  rbd::VectorX q{rbd::VectorX::Zero(3)};      // [x_bottom, s_upper, x_cube]
  rbd::VectorX qdot{rbd::VectorX::Zero(3)};   // velocities
  double time{0.0};
};

struct SlopeSim {
  rbd::Model model;
  SlopeState state;
  bool use_crba{false};
  bool paused{true};

  rbd::Vector3 GravityVector() const { return rbd::Vector3(0.0, -kGravityDefault, 0.0); }

  void Reset() {
    state.q.setZero();
    state.qdot.setZero();
    state.time = 0.0;
  }

  void Step(double dt) {
    if (paused) {
      return;
    }

    const auto compute_qdd = [&](const rbd::VectorX &q, const rbd::VectorX &qdot) {
      rbd::VectorX qdd = rbd::VectorX::Zero(model.dof_count);
      rbd::VectorX tau = rbd::VectorX::Zero(model.dof_count);  // no actuation
      rbd::VectorX zero_qdd = rbd::VectorX::Zero(model.dof_count);

      rbd::UpdateKinematics(model, q, qdot, zero_qdd);
      model.gravity = GravityVector();

      if (use_crba) {
        rbd::ForwardDynamicsLagragian(model, q, qdot, tau, qdd, nullptr);
      } else {
        rbd::ForwardDynamics(model, q, qdot, tau, qdd, nullptr);
      }

      return qdd;
    };

    auto system = [&](const state_type &x, state_type &dxdt, double /*t*/) {
      rbd::VectorX q(3), qdot(3);
      q(0) = x[0];
      q(1) = x[1];
      q(2) = x[2];
      qdot(0) = x[3];
      qdot(1) = x[4];
      qdot(2) = x[5];

      const rbd::VectorX qdd = compute_qdd(q, qdot);

      dxdt[0] = qdot(0);
      dxdt[1] = qdot(1);
      dxdt[2] = qdot(2);
      dxdt[3] = qdd(0);
      dxdt[4] = qdd(1);
      dxdt[5] = qdd(2);
    };

    state_type x = {state.q(0), state.q(1), state.q(2),
                    state.qdot(0), state.qdot(1), state.qdot(2)};
    static odeint::runge_kutta4<state_type> stepper;
    stepper.do_step(system, x, state.time, dt);

    state.q(0) = x[0];
    state.q(1) = x[1];
    state.q(2) = x[2];
    state.qdot(0) = x[3];
    state.qdot(1) = x[4];
    state.qdot(2) = x[5];
    state.time += dt;

    rbd::VectorX zero_qdd = rbd::VectorX::Zero(model.dof_count);
    rbd::UpdateKinematics(model, state.q, state.qdot, zero_qdd);
  }
};

double ComputeTotalEnergy(SlopeSim &sim) {
  rbd::VectorX zero_qdd = rbd::VectorX::Zero(sim.model.dof_count);
  rbd::UpdateKinematics(sim.model, sim.state.q, sim.state.qdot, zero_qdd);

  double kinetic = 0.0;
  double potential = 0.0;
  const rbd::Vector3 g = sim.GravityVector();

  for (size_t i = 1; i < sim.model.mBodies.size(); ++i) {
    const auto &body = sim.model.mBodies[i];
    const rbd::SpatialVector &v = sim.model.v[i];
    rbd::SpatialVector Iv = sim.model.I[i] * v;
    kinetic += 0.5 * v.dot(Iv);

    const rbd::Vector3 origin_world = sim.model.X_base[i].r;
    const rbd::Vector3 com_world = sim.model.X_base[i].E.transpose() * body.mCenterOfMass + origin_world;
    potential -= body.mMass * g.dot(com_world);
  }

  return kinetic + potential;
}

// Get cube world position for verification (should only move in Y)
rbd::Vector3 GetCubeWorldPosition(SlopeSim &sim) {
  rbd::VectorX zero_qdd = rbd::VectorX::Zero(sim.model.dof_count);
  rbd::UpdateKinematics(sim.model, sim.state.q, sim.state.qdot, zero_qdd);
  
  // Body 3 is the cube, get its origin in world frame
  const rbd::Vector3 origin_world = sim.model.X_base[3].r;
  const rbd::Vector3 com_world = sim.model.X_base[3].E.transpose() * 
                                  sim.model.mBodies[3].mCenterOfMass + origin_world;
  return com_world;
}

// -----------------------------------------------------------------------------
// Rendering helpers
// -----------------------------------------------------------------------------
glm::mat4 SpatialToGlm(const rbd::SpatialTransform &X) {
  glm::mat4 M(1.0f);
  const rbd::Matrix3 R = X.E.transpose();  // body -> world
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      M[c][r] = static_cast<float>(R(r, c));
    }
  }
  M[3][0] = static_cast<float>(X.r(0));
  M[3][1] = static_cast<float>(X.r(1));
  M[3][2] = static_cast<float>(X.r(2));
  return M;
}

// Create bottom slope mesh ABC-A'B'C' (right triangle cross-section)
// Triangle ABC where B is the right angle:
//   A = (0, 0), B = (ab, 0), C = (ab, bc)
// Extruded along Z from -depth/2 to +depth/2
Mesh<> CreateBottomSlopeMesh(double ab, double bc, double depth) {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;

  float d2 = static_cast<float>(depth / 2);
  float fab = static_cast<float>(ab);
  float fbc = static_cast<float>(bc);

  // Vertices: A=(0,0), B=(ab,0), C=(ab,bc)
  glm::vec3 A_front(0, 0, d2);
  glm::vec3 B_front(fab, 0, d2);
  glm::vec3 C_front(fab, fbc, d2);
  glm::vec3 A_back(0, 0, -d2);
  glm::vec3 B_back(fab, 0, -d2);
  glm::vec3 C_back(fab, fbc, -d2);

  // Front triangle (ABC) - normal +Z, CCW winding from front
  positions.push_back(A_front); positions.push_back(B_front); positions.push_back(C_front);
  normals.push_back({0, 0, 1}); normals.push_back({0, 0, 1}); normals.push_back({0, 0, 1});
  indices.push_back(0); indices.push_back(1); indices.push_back(2);

  // Back triangle (A'B'C') - normal -Z, CCW winding from back = CW from front
  positions.push_back(A_back); positions.push_back(B_back); positions.push_back(C_back);
  normals.push_back({0, 0, -1}); normals.push_back({0, 0, -1}); normals.push_back({0, 0, -1});
  indices.push_back(3); indices.push_back(5); indices.push_back(4);

  // Bottom face (ABB'A') - normal -Y
  positions.push_back(A_front); positions.push_back(B_front); positions.push_back(B_back); positions.push_back(A_back);
  normals.push_back({0, -1, 0}); normals.push_back({0, -1, 0}); normals.push_back({0, -1, 0}); normals.push_back({0, -1, 0});
  indices.push_back(6); indices.push_back(9); indices.push_back(8);
  indices.push_back(6); indices.push_back(8); indices.push_back(7);

  // Right face (BCC'B') - normal +X (vertical face at x=ab)
  // Vertices in CCW order when viewed from +X: B_front, B_back, C_back, C_front
  positions.push_back(B_front); positions.push_back(B_back); positions.push_back(C_back); positions.push_back(C_front);
  normals.push_back({1, 0, 0}); normals.push_back({1, 0, 0}); normals.push_back({1, 0, 0}); normals.push_back({1, 0, 0});
  indices.push_back(10); indices.push_back(11); indices.push_back(12);
  indices.push_back(10); indices.push_back(12); indices.push_back(13);

  // Hypotenuse face (ACC'A') - normal pointing outward from slope (-bc, ab, 0) normalized
  // Vertices in CCW order when viewed from outside: A_front, C_front, C_back, A_back
  float hyp_len = std::sqrt(fab * fab + fbc * fbc);
  glm::vec3 hyp_normal(-fbc / hyp_len, fab / hyp_len, 0);
  positions.push_back(A_front); positions.push_back(C_front); positions.push_back(C_back); positions.push_back(A_back);
  normals.push_back(hyp_normal); normals.push_back(hyp_normal); normals.push_back(hyp_normal); normals.push_back(hyp_normal);
  indices.push_back(14); indices.push_back(15); indices.push_back(16);
  indices.push_back(14); indices.push_back(16); indices.push_back(17);

  Mesh<> mesh(positions.size(), indices.size(), indices.data(),
              reinterpret_cast<Vector3<float>*>(positions.data()),
              reinterpret_cast<Vector3<float>*>(normals.data()),
              nullptr, nullptr);
  return mesh;
}

// Create upper slope mesh DEF-D'E'F' (right triangle cross-section)
// Triangle DEF where E is the right angle:
//   D = (ab, bc), E = (0, bc), F = (0, 0)
// At initial state: D coincides with C, E is at (0,BC), F coincides with A
// Extruded along Z from -depth/2 to +depth/2
Mesh<> CreateUpperSlopeMesh(double ab, double bc, double depth) {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  std::vector<uint32_t> indices;

  float d2 = static_cast<float>(depth / 2);
  float fab = static_cast<float>(ab);
  float fbc = static_cast<float>(bc);

  // Vertices: D=(ab,bc), E=(0,bc), F=(0,0)
  glm::vec3 D_front(fab, fbc, d2);
  glm::vec3 E_front(0, fbc, d2);
  glm::vec3 F_front(0, 0, d2);
  glm::vec3 D_back(fab, fbc, -d2);
  glm::vec3 E_back(0, fbc, -d2);
  glm::vec3 F_back(0, 0, -d2);

  // Front triangle (DEF) - normal +Z, CCW winding
  positions.push_back(D_front); positions.push_back(E_front); positions.push_back(F_front);
  normals.push_back({0, 0, 1}); normals.push_back({0, 0, 1}); normals.push_back({0, 0, 1});
  indices.push_back(0); indices.push_back(1); indices.push_back(2);

  // Back triangle (D'E'F') - normal -Z
  positions.push_back(D_back); positions.push_back(E_back); positions.push_back(F_back);
  normals.push_back({0, 0, -1}); normals.push_back({0, 0, -1}); normals.push_back({0, 0, -1});
  indices.push_back(3); indices.push_back(5); indices.push_back(4);

  // Top face (DEE'D') - normal +Y (horizontal surface at y=bc)
  positions.push_back(D_front); positions.push_back(E_front); positions.push_back(E_back); positions.push_back(D_back);
  normals.push_back({0, 1, 0}); normals.push_back({0, 1, 0}); normals.push_back({0, 1, 0}); normals.push_back({0, 1, 0});
  indices.push_back(6); indices.push_back(9); indices.push_back(8);
  indices.push_back(6); indices.push_back(8); indices.push_back(7);

  // Left face (EFF'E') - normal -X (vertical face at x=0)
  positions.push_back(E_front); positions.push_back(F_front); positions.push_back(F_back); positions.push_back(E_back);
  normals.push_back({-1, 0, 0}); normals.push_back({-1, 0, 0}); normals.push_back({-1, 0, 0}); normals.push_back({-1, 0, 0});
  indices.push_back(10); indices.push_back(11); indices.push_back(12);
  indices.push_back(10); indices.push_back(12); indices.push_back(13);

  // Hypotenuse face (FDD'F') - normal pointing outward from slope (same direction as bottom slope)
  float hyp_len = std::sqrt(fab * fab + fbc * fbc);
  glm::vec3 hyp_normal(fbc / hyp_len, -fab / hyp_len, 0);  // opposite direction to bottom slope
  positions.push_back(F_front); positions.push_back(D_front); positions.push_back(D_back); positions.push_back(F_back);
  normals.push_back(hyp_normal); normals.push_back(hyp_normal); normals.push_back(hyp_normal); normals.push_back(hyp_normal);
  indices.push_back(14); indices.push_back(15); indices.push_back(16);
  indices.push_back(14); indices.push_back(16); indices.push_back(17);

  Mesh<> mesh(positions.size(), indices.size(), indices.data(),
              reinterpret_cast<Vector3<float>*>(positions.data()),
              reinterpret_cast<Vector3<float>*>(normals.data()),
              nullptr, nullptr);
  return mesh;
}

// Transform for the bottom slope (body 1)
// Body frame origin is at point A, with AB along +X and BC along +Y
glm::mat4 BottomSlopeTransform(const rbd::SpatialTransform &body_tf) {
  return SpatialToGlm(body_tf);
}

// Transform for the upper slope (body 2)
// Upper slope DEF: D=(AB,BC), E=(0,BC), F=(0,0)
// The mesh is created with F at origin, so just apply the body transform
glm::mat4 UpperSlopeTransform(const rbd::SpatialTransform &body_tf) {
  return SpatialToGlm(body_tf);
}

// Transform for the cube (body 3)
glm::mat4 CubeTransform(const rbd::SpatialTransform &body_tf, double cube_size) {
  glm::mat4 T_body = SpatialToGlm(body_tf);
  // Cube mesh is unit cube [-1,1]^3, scale to cube_size and offset so bottom is at origin
  glm::mat4 offset = glm::translate(glm::mat4(1.0f), glm::vec3(0, static_cast<float>(cube_size / 2), 0));
  glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(cube_size / 2)));
  return T_body * offset * scale;
}

// -----------------------------------------------------------------------------
// Model construction
// -----------------------------------------------------------------------------
// Compute inertia tensor for a triangular prism about its COM
// Simplified approximation: treat as box with equivalent dimensions
rbd::Matrix3 TriangularPrismInertia(double mass, double ab, double bc, double depth) {
  // For a triangular prism, approximate inertia
  // Using parallel axis theorem from a bounding box approximation
  rbd::Matrix3 I = rbd::Matrix3::Zero();
  double Ixx = mass * (bc * bc + depth * depth) / 18.0;
  double Iyy = mass * (ab * ab + depth * depth) / 18.0;
  double Izz = mass * (ab * ab + bc * bc) / 18.0;
  I(0, 0) = Ixx;
  I(1, 1) = Iyy;
  I(2, 2) = Izz;
  return I;
}

// COM of bottom slope ABC: A=(0,0), B=(ab,0), C=(ab,bc)
// Centroid = ((0 + ab + ab)/3, (0 + 0 + bc)/3) = (2*ab/3, bc/3)
rbd::Vector3 BottomSlopeCOM(double ab, double bc) {
  return rbd::Vector3(2.0 * ab / 3.0, bc / 3.0, 0.0);
}

// COM of upper slope DEF: D=(ab,bc), E=(0,bc), F=(0,0)
// Centroid = ((ab + 0 + 0)/3, (bc + bc + 0)/3) = (ab/3, 2*bc/3)
rbd::Vector3 UpperSlopeCOM(double ab, double bc) {
  return rbd::Vector3(ab / 3.0, 2.0 * bc / 3.0, 0.0);
}

rbd::Model BuildModel() {
  rbd::Model model;
  model.gravity = rbd::Vector3(0.0, -kGravityDefault, 0.0);

  // ----- Body 1: Bottom slope (ABC-A'B'C') -----
  // Slides along X-axis, connected to world
  // Body frame origin at point A (which is at world origin initially)
  rbd::Matrix3 inertia_slope = TriangularPrismInertia(kSlopeMass, kAB, kBC, kDepth);
  rbd::Vector3 com_bottom = BottomSlopeCOM(kAB, kBC);
  rbd::Body bottom_slope(kSlopeMass, com_bottom, inertia_slope);

  rbd::Joint prismatic_x(rbd::JointTypePrismatic, rbd::Vector3(1.0, 0.0, 0.0));
  rbd::SpatialTransform joint_frame_root(rbd::Matrix3::Identity(), rbd::Vector3::Zero());
  model.AddBody(0, joint_frame_root, prismatic_x, bottom_slope);

  // ----- Body 2: Upper slope (DEF-D'E'F') -----
  // D=(AB,BC), E=(0,BC), F=(0,0) - F coincides with A at initial state
  // The upper slope slides along AC direction (the hypotenuse)
  // AC direction: from A(0,0) to C(AB,BC), normalized = (cos(theta), sin(theta))
  
  // Joint frame: at point A of body 1 (origin), upper slope will slide along AC
  // When q2=0, upper slope's F is at A, D is at C
  rbd::SpatialTransform joint_frame_upper(rbd::Matrix3::Identity(), rbd::Vector3::Zero());
  
  // Upper slope body with its own COM
  rbd::Vector3 com_upper = UpperSlopeCOM(kAB, kBC);
  rbd::Body upper_slope(kSlopeMass, com_upper, inertia_slope);
  
  // Joint along AC direction (sliding from A towards C, i.e., up the slope)
  // Positive q means the upper slope moves towards C (up and right)
  rbd::Joint prismatic_ac(rbd::JointTypePrismatic, rbd::Vector3(kCosTheta, kSinTheta, 0.0));
  model.AddBody(1, joint_frame_upper, prismatic_ac, upper_slope);

  // ----- Body 3: Cube -----
  // Sits on the horizontal surface EDD'E' of the upper slope
  // In the upper slope's local frame (rendered flipped), the top surface is at y=BC
  // The horizontal surface spans from x=0 to x=AB at height BC
  // But since upper slope is rendered with 180 deg rotation, we need to account for that
  // In physics frame of upper slope (same as bottom), the "top" after flip is still at y=BC
  
  // Cube inertia (solid cube about COM)
  double cube_inertia_val = (1.0 / 6.0) * kCubeMass * kCubeSize * kCubeSize;
  rbd::Matrix3 inertia_cube = rbd::Matrix3::Zero();
  inertia_cube(0, 0) = cube_inertia_val;
  inertia_cube(1, 1) = cube_inertia_val;
  inertia_cube(2, 2) = cube_inertia_val;
  
  // Cube COM at center, body frame origin at bottom center of cube
  rbd::Vector3 com_cube(0.0, kCubeSize / 2.0, 0.0);
  rbd::Body cube(kCubeMass, com_cube, inertia_cube);
  
  // The cube sits on surface EDD'E'. In the combined system:
  // E = (0, BC), D = (AB, BC) - this is the top horizontal edge at height BC
  // Place cube at center of this edge, at height BC in upper slope's frame
  // Since upper slope frame starts at A=(0,0), offset by (AB/2, BC) puts cube on top
  rbd::SpatialTransform joint_frame_cube(rbd::Matrix3::Identity(), rbd::Vector3(kAB / 2.0, kBC, 0.0));
  
  rbd::Joint prismatic_x_cube(rbd::JointTypePrismatic, rbd::Vector3(1.0, 0.0, 0.0));
  model.AddBody(2, joint_frame_cube, prismatic_x_cube, cube);

  return model;
}

// -----------------------------------------------------------------------------
// Entry point
// -----------------------------------------------------------------------------
int main() {
  // Graphics setup
  std::shared_ptr<graphics::Core> core;
  graphics::CreateCore(graphics::BACKEND_API_DEFAULT, graphics::Core::Settings{}, &core);
  core->InitializeLogicalDeviceAutoSelect(false);

  sparkium::Core spark_core(core.get());
  sparkium::Scene scene(&spark_core);
  scene.settings.raster.ambient_light = glm::vec3{0.4f, 0.4f, 0.4f};
  sparkium::Film film(&spark_core, 1280, 720);
  film.info.persistence = 0.0f;
  sparkium::Camera camera(&spark_core,
                          glm::lookAt(glm::vec3{1.5f, 1.5f, 3.0f}, glm::vec3{0.5f, 0.3f, 0.0f},
                                      glm::vec3{0.0f, 1.0f, 0.0f}),
                          glm::radians(45.0f), static_cast<float>(film.GetWidth()) / film.GetHeight());

  std::unique_ptr<graphics::Image> srgb_image;
  core->CreateImage(film.GetWidth(), film.GetHeight(), graphics::IMAGE_FORMAT_R8G8B8A8_UNORM, &srgb_image);

  std::unique_ptr<graphics::Window> window;
  core->CreateWindowObject(film.GetWidth(), film.GetHeight(), "Slope Prismatic Joints", false, true, &window);
  window->InitImGui(nullptr, 20.0f);

  // Scene assets - create meshes
  Mesh<> bottom_slope_mesh = CreateBottomSlopeMesh(kAB, kBC, kDepth);
  Mesh<> upper_slope_mesh = CreateUpperSlopeMesh(kAB, kBC, kDepth);
  Mesh<> cube_mesh;
  cube_mesh.LoadObjFile(FindAssetFile("meshes/cube.obj"));

  // Materials
  sparkium::MaterialPrincipled bottom_slope_material(&spark_core, glm::vec3{0.2f, 0.6f, 0.9f});
  bottom_slope_material.roughness = 0.4f;
  sparkium::MaterialPrincipled upper_slope_material(&spark_core, glm::vec3{0.9f, 0.5f, 0.2f});
  upper_slope_material.roughness = 0.4f;
  sparkium::MaterialPrincipled cube_material(&spark_core, glm::vec3{0.2f, 0.9f, 0.3f});
  cube_material.roughness = 0.3f;
  sparkium::MaterialPrincipled ground_material(&spark_core, glm::vec3{0.5f, 0.5f, 0.5f});
  ground_material.roughness = 0.8f;

  // Geometry
  sparkium::GeometryMesh bottom_slope_geom(&spark_core, bottom_slope_mesh);
  sparkium::GeometryMesh upper_slope_geom(&spark_core, upper_slope_mesh);
  sparkium::GeometryMesh cube_geom(&spark_core, cube_mesh);

  // Entities
  sparkium::EntityGeometryMaterial bottom_slope_entity(&spark_core, &bottom_slope_geom, &bottom_slope_material);
  sparkium::EntityGeometryMaterial upper_slope_entity(&spark_core, &upper_slope_geom, &upper_slope_material);
  sparkium::EntityGeometryMaterial cube_entity(&spark_core, &cube_geom, &cube_material);

  // Ground plane (using cube mesh scaled flat)
  sparkium::EntityGeometryMaterial ground_entity(
      &spark_core, &cube_geom, &ground_material,
      glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.05f, 0.0f)), 
                 glm::vec3(3.0f, 0.05f, 2.0f)));

  // Sky
  sparkium::MaterialLight sky(&spark_core, glm::vec3{0.7f, 0.7f, 0.8f}, true, false);
  sparkium::GeometryMesh sky_mesh(&spark_core, Mesh<>::Sphere(12));
  sparkium::EntityGeometryMaterial sky_entity(
      &spark_core, &sky_mesh, &sky,
      glm::scale(glm::mat4{1.0f}, glm::vec3{50.0f}));
  sky_entity.raster_light = false;

  scene.AddEntity(&sky_entity);
  scene.AddEntity(&ground_entity);
  scene.AddEntity(&bottom_slope_entity);
  scene.AddEntity(&upper_slope_entity);
  scene.AddEntity(&cube_entity);

  // Simulation setup
  SlopeSim sim;
  sim.model = BuildModel();
  sim.Reset();
  rbd::VectorX zero_qdd = rbd::VectorX::Zero(sim.model.dof_count);
  rbd::UpdateKinematics(sim.model, sim.state.q, sim.state.qdot, zero_qdd);

  // Track initial cube X position
  rbd::Vector3 initial_cube_pos = GetCubeWorldPosition(sim);
  double initial_cube_x = initial_cube_pos(0);

  auto refresh_mesh_transforms = [&]() {
    rbd::VectorX zero_qdd_local = rbd::VectorX::Zero(sim.model.dof_count);
    rbd::UpdateKinematics(sim.model, sim.state.q, sim.state.qdot, zero_qdd_local);
    bottom_slope_entity.SetTransformation(BottomSlopeTransform(sim.model.X_base[1]));
    upper_slope_entity.SetTransformation(UpperSlopeTransform(sim.model.X_base[2]));
    cube_entity.SetTransformation(CubeTransform(sim.model.X_base[3], kCubeSize));
  };

  refresh_mesh_transforms();

  // Timing
  double last_time = glfwGetTime();
  double accumulator = 0.0;
  FPSCounter fps_counter;

  while (!window->ShouldClose()) {
    const double now = glfwGetTime();
    accumulator += now - last_time;
    last_time = now;

    while (accumulator >= kSimDt) {
      sim.Step(kSimDt);
      accumulator -= kSimDt;
    }

    refresh_mesh_transforms();

    // Get cube position for verification
    rbd::Vector3 cube_pos = GetCubeWorldPosition(sim);
    double cube_x_drift = cube_pos(0) - initial_cube_x;

    window->BeginImGuiFrame();
    ImGui::SetNextWindowPos({0, 0}, ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.35f);
    if (ImGui::Begin("Slope Simulation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Checkbox("Pause", &sim.paused);
      ImGui::Checkbox("Use CRBA", &sim.use_crba);
      if (ImGui::Button("Reset")) {
        sim.Reset();
        initial_cube_pos = GetCubeWorldPosition(sim);
        initial_cube_x = initial_cube_pos(0);
        refresh_mesh_transforms();
      }
      ImGui::Separator();
      ImGui::Text("Parameters:");
      ImGui::Text("  AB = %.2f m, BC = %.2f m", kAB, kBC);
      ImGui::Text("  Slope mass = %.2f kg", kSlopeMass);
      ImGui::Text("  Cube mass = %.2f kg", kCubeMass);
      ImGui::Text("  Gravity = %.2f m/s^2", kGravityDefault);
      ImGui::Separator();
      ImGui::Text("State:");
      ImGui::Text("  time: %.3f s", sim.state.time);
      ImGui::Text("  q [m]: (%.4f, %.4f, %.4f)", sim.state.q(0), sim.state.q(1), sim.state.q(2));
      ImGui::Text("  qdot [m/s]: (%.4f, %.4f, %.4f)", sim.state.qdot(0), sim.state.qdot(1), sim.state.qdot(2));
      ImGui::Separator();
      ImGui::Text("Energy:");
      ImGui::Text("  Total: %.6e J", ComputeTotalEnergy(sim));
      ImGui::Separator();
      ImGui::Text("Verification (cube should only move in Y):");
      ImGui::Text("  Cube world pos: (%.4f, %.4f, %.4f)", cube_pos(0), cube_pos(1), cube_pos(2));
      ImGui::Text("  Cube X drift: %.6f m", cube_x_drift);
    }
    ImGui::End();
    window->EndImGuiFrame();

    spark_core.Render(&scene, &camera, &film, sparkium::RENDER_PIPELINE_RASTERIZATION);
    film.Develop(srgb_image.get());

    std::unique_ptr<graphics::CommandContext> ctx;
    core->CreateCommandContext(&ctx);
    ctx->CmdPresent(window.get(), srgb_image.get());
    core->SubmitCommandContext(ctx.get());

    glfwPollEvents();

    const float fps = fps_counter.TickFPS();
    window->SetTitle(std::string("Slope Prismatic Joints - ") + std::to_string(fps) + " FPS");
  }

  film.Develop(srgb_image.get());
  std::vector<uint8_t> image_data(film.GetWidth() * film.GetHeight() * 4);
  srgb_image->DownloadData(image_data.data());
  stbi_write_bmp("output.bmp", film.GetWidth(), film.GetHeight(), 4, image_data.data());

  window->TerminateImGui();
  core->WaitGPU();
  return 0;
}
