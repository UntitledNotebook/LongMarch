/**
 * @file main.cpp
 * @brief Franka FR3 Grasping Demo - Refactored
 *
 * A clean pick-and-place demo using modular components:
 * - URDF loading and processing
 * - Inverse kinematics with sampling
 * - Path planning with S-curve trajectories
 * - PD control with gravity compensation
 * - Forward dynamics simulation
 * - Visual rendering with mesh loading
 *
 * Usage:
 *   - Use GUI controls to set ball/goal positions
 *   - Click "Start Task" to begin pick-and-place
 *   - Toggle "Use Physics" for dynamics vs kinematic mode
 */

#include <long_march.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "config.h"
#include "types.h"
#include "urdf_utils.h"
#include "ik_solver.h"
#include "path_planner.h"
#include "controller.h"
#include "visual.h"
#include "task.h"

#include "stb_image_write.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace long_march;
using namespace franka;

// ============================================================================
// Main Application
// ============================================================================

int main() {
  LogInfo("=== Franka FR3 Grasping Demo ===");

  // =========================================================================
  // 1. Load Robot Model
  // =========================================================================

  // Register asset search paths
  std::string assets_dir = std::string(LONGMARCH_ASSETS_DIR) + "/urdfs/franka_fr3/";
  FileProbe::GetInstance().AddSearchPath(assets_dir);
  FileProbe::GetInstance().AddSearchPath(assets_dir + "meshes/");

  auto robot_model = urdf_utils::loadRobotModel("fr3_franka_hand.urdf");
  if (!robot_model) {
    LogError("Failed to load robot model");
    return 1;
  }

  LogInfo("Robot loaded successfully: {} DOF", robot_model->dof());

  // =========================================================================
  // 2. Initialize Graphics
  // =========================================================================

  std::unique_ptr<graphics::Core> core_;
  graphics::CreateCore(graphics::BACKEND_API_DEFAULT, graphics::Core::Settings{}, &core_);
  core_->InitializeLogicalDeviceAutoSelect(false);
  sparkium::Core sparkium_core(core_.get());

  // Create scene
  sparkium::Scene scene(&sparkium_core);
  scene.settings.samples_per_dispatch = config::SAMPLES_PER_DISPATCH;
  scene.settings.raster.ambient_light = config::AMBIENT_LIGHT;

  // Create cameras
  sparkium::Camera camera(
      &sparkium_core,
      glm::lookAt(config::CAMERA_POSITION, config::CAMERA_TARGET, config::CAMERA_UP),
      glm::radians(config::CAMERA_FOV),
      static_cast<float>(config::RENDER_WIDTH) / config::RENDER_HEIGHT);

  // TCP view camera (avoid naming variables with tcp suffix due to X11 macro)
  sparkium::Camera camera_tcp_view(
      &sparkium_core,
      glm::mat4(1.0f),
      glm::radians(config::TCP_CAMERA_FOV),
      static_cast<float>(config::TCP_VIEW_WIDTH) / config::TCP_VIEW_HEIGHT);

  // Create films
  sparkium::Film film(&sparkium_core, config::RENDER_WIDTH, config::RENDER_HEIGHT);
  film.info.persistence = config::FILM_PERSISTENCE;

  sparkium::Film film_tcp_view(&sparkium_core, config::TCP_VIEW_WIDTH, config::TCP_VIEW_HEIGHT);
  film_tcp_view.info.persistence = config::FILM_PERSISTENCE;

  // Create images
  std::unique_ptr<graphics::Image> srgb_image;
  core_->CreateImage(config::RENDER_WIDTH, config::RENDER_HEIGHT,
                     graphics::IMAGE_FORMAT_R8G8B8A8_UNORM, &srgb_image);

  std::unique_ptr<graphics::Image> srgb_tcp_image;
  core_->CreateImage(config::TCP_VIEW_WIDTH, config::TCP_VIEW_HEIGHT,
                     graphics::IMAGE_FORMAT_R8G8B8A8_UNORM, &srgb_tcp_image);

  // Create windows
  std::unique_ptr<graphics::Window> window;
  core_->CreateWindowObject(config::RENDER_WIDTH, config::RENDER_HEIGHT,
                           "Franka Grasping Demo", &window);

  std::unique_ptr<graphics::Window> tcp_view_window;
  core_->CreateWindowObject(config::TCP_VIEW_WIDTH, config::TCP_VIEW_HEIGHT,
                           "TCP View", &tcp_view_window);

  FPSCounter fps_counter;

  // =========================================================================
  // 3. Setup Scene
  // =========================================================================

  // Create geometries
  Mesh<> cube_mesh;
  cube_mesh.LoadObjFile(FindAssetFile("meshes/cube.obj"));
  sparkium::GeometryMesh geometry_sphere(&sparkium_core, Mesh<>::Sphere(30));
  sparkium::GeometryMesh geometry_cube(&sparkium_core, cube_mesh);

  // Setup standard scene elements (must stay alive for duration of main loop)
  visual::SceneSetup scene_setup;
  scene_setup.setup(&sparkium_core, &scene, &geometry_sphere, &geometry_cube);

  // Create ball
  auto ball_entity = visual::SceneSetup::createBall(
      &sparkium_core, &geometry_sphere,
      glm::vec3{0.4f, 0.0f, config::BALL_RADIUS},
      config::BALL_RADIUS,
      glm::vec3{0.9f, 0.2f, 0.2f});
  scene.AddEntity(ball_entity.get());

  // Create goal marker
  auto goal_entity = visual::SceneSetup::createGoalMarker(
      &sparkium_core, &geometry_cube,
      glm::vec3{0.4f, 0.4f, config::GOAL_MARKER_HEIGHT},
      config::GOAL_MARKER_SIZE,
      glm::vec3{0.2f, 0.9f, 0.2f});
  scene.AddEntity(goal_entity.get());

  // =========================================================================
  // 4. Create Components
  // =========================================================================

  // Visualizer
  visual::RobotVisualizer robot_visualizer(&sparkium_core, &scene, &geometry_cube);
  if (!robot_visualizer.loadVisuals(robot_model->link_visuals)) {
    LogWarning("Some robot meshes failed to load");
  }

  // Controller - separate configs for base joints (high inertia) and wrist joints (low inertia)
  ControllerConfig arm_base_cfg(config::KP_ARM_BASE, config::KD_ARM_BASE, config::MAX_TORQUE_J1_J4);
  ControllerConfig arm_wrist_cfg(config::KP_ARM_WRIST, config::KD_ARM_WRIST, config::MAX_TORQUE_J5_J7);
  ControllerConfig gripper_cfg(config::KP_GRIPPER, config::KD_GRIPPER, config::MAX_TORQUE_GRIPPER);
  controller::PDController pd_controller(arm_base_cfg, arm_wrist_cfg, gripper_cfg, 7);

  // Simulator
  controller::RobotSimulator simulator(robot_model->rbd_model, pd_controller, config::DT);

  // Path planner
  PathPlannerConfig planner_config;
  planner_config.max_velocity = config::MAX_JOINT_VELOCITY;
  planner_config.max_acceleration = config::MAX_JOINT_ACCELERATION;
  planner_config.max_jerk = config::MAX_JOINT_JERK;
  planner_config.ik_max_samples = config::IK_DEFAULT_SAMPLES;

  planner::PathPlanner path_planner(robot_model->rbd_model, *robot_model, planner_config);

  // Task manager
  task::TaskManager task_manager(robot_model->rbd_model, path_planner, *robot_model);

  // =========================================================================
  // 5. Initialize Robot State
  // =========================================================================

  contradium::rbd::VectorX Q = task_manager.getHomeConfiguration();
  contradium::rbd::VectorX QDot = contradium::rbd::VectorX::Zero(robot_model->dof());
  contradium::rbd::VectorX Q_desired = Q;
  contradium::rbd::VectorX QDot_desired = contradium::rbd::VectorX::Zero(robot_model->dof());

  // =========================================================================
  // 6. GUI State
  // =========================================================================

  bool ray_tracing = false;
  bool use_physics = true;
  bool auto_run = false;
  bool simulation_running = false;

  float ball_x = 0.4f;
  float ball_y = 0.0f;
  float goal_x = 0.4f;
  float goal_y = 0.4f;

  window->InitImGui(nullptr, 20.0f);

  // Log axis frame verification
  LogInfo("Axis frames initialized - Target frame and Current frame added to scene");

  // =========================================================================
  // 7. Main Loop
  // =========================================================================

  int frame_count = 0;

  while (!window->ShouldClose() && !tcp_view_window->ShouldClose()) {
    window->BeginImGuiFrame();

    // =======================================================================
    // GUI
    // =======================================================================

    if (ImGui::Begin("Grasping Control", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::Checkbox("Ray Tracing", &ray_tracing);
      if (ray_tracing && !core_->DeviceRayTracingSupport()) {
        ImGui::Text("Ray Tracing not supported!");
      }

      ImGui::Checkbox("Use Physics (ABA)", &use_physics);

      ImGui::Separator();
      ImGui::Text("Task Stage: %s", task_manager.getStageName());
      if (task_manager.getCurrentStage() != TaskStage::IDLE &&
          task_manager.getCurrentStage() != TaskStage::COMPLETE) {
        ImGui::ProgressBar(static_cast<float>(task_manager.getStageProgress()));
      }

      ImGui::Separator();
      ImGui::Text("Ball Position:");
      bool ball_changed = ImGui::SliderFloat("Ball X", &ball_x, -0.5f, 0.8f);
      ball_changed |= ImGui::SliderFloat("Ball Y", &ball_y, -0.5f, 0.5f);

      ImGui::Text("Goal Position:");
      bool goal_changed = ImGui::SliderFloat("Goal X", &goal_x, -0.5f, 0.8f);
      goal_changed |= ImGui::SliderFloat("Goal Y", &goal_y, -0.5f, 0.5f);

      // Update positions
      if (ball_changed && !task_manager.isBallGrasped()) {
        glm::vec3 ball_pos(ball_x, ball_y, config::BALL_RADIUS);
        task_manager.setBallPosition(ball_pos);
        ball_entity.SetTransformation(
            glm::translate(glm::mat4{1.0f}, ball_pos) *
            glm::scale(glm::mat4(1.0f), glm::vec3(config::BALL_RADIUS)));
      }

      if (goal_changed) {
        glm::vec3 goal_pos(goal_x, goal_y, config::GOAL_MARKER_HEIGHT);
        task_manager.setGoalPosition(goal_pos);
        goal_entity.SetTransformation(
            glm::translate(glm::mat4{1.0f}, goal_pos) *
            glm::scale(glm::mat4(1.0f), glm::vec3(config::GOAL_MARKER_SIZE,
                                                  config::GOAL_MARKER_SIZE,
                                                  config::GOAL_MARKER_HEIGHT)));
      }

      ImGui::Separator();
      ImGui::Checkbox("Auto Run", &auto_run);

      if (ImGui::Button("Start Task") && task_manager.getCurrentStage() == TaskStage::IDLE) {
        glm::vec3 ball_pos(ball_x, ball_y, config::BALL_RADIUS);
        glm::vec3 goal_pos(goal_x, goal_y, 0.0f);
        task_manager.start(ball_pos, goal_pos, Q);
        simulation_running = true;
      }

      if (ImGui::Button("Reset") || (task_manager.getCurrentStage() == TaskStage::COMPLETE && auto_run)) {
        task_manager.reset();
        simulation_running = false;
        Q = task_manager.getHomeConfiguration();
        QDot.setZero();
      }

      ImGui::Separator();
      ImGui::Text("Status:");
      ImGui::Text("  Running: %s", simulation_running ? "Yes" : "No");
      ImGui::Text("  Ball Grasped: %s", task_manager.isBallGrasped() ? "Yes" : "No");
    }
    ImGui::End();
    window->EndImGuiFrame();

    // =======================================================================
    // Task Update
    // =======================================================================

    if (simulation_running) {
      task_manager.update(config::DT * config::SUBSTEPS, Q, QDot, Q_desired, QDot_desired);
    } else {
      Q_desired = task_manager.getHomeConfiguration();
      QDot_desired.setZero();
    }

    // =======================================================================
    // Physics Simulation
    // =======================================================================

    if (simulation_running && use_physics) {
      if (!simulator.stepSubsteps(Q_desired, QDot_desired, Q, QDot, config::SUBSTEPS)) {
        LogError("Physics simulation failed - stopping");
        simulation_running = false;
        task_manager.reset();
      }

      // Clamp to limits
      urdf_utils::clampToLimits(*robot_model, Q);

      // Log joint torques
      const auto& tau = simulator.getTorques();
      LogInfo("Joint Torques: [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]",
              tau(0), tau(1), tau(2), tau(3), tau(4), tau(5), tau(6), tau(7), tau(8));
    } else {
      // Kinematic mode
      Q = Q_desired;
      QDot.setZero();
    }

    // =======================================================================
    // Visual Update
    // =======================================================================

    // Update robot visuals
    robot_visualizer.update(robot_model->rbd_model, robot_model->urdf_model, Q);

    // Update ball position (follows TCP if grasped)
    if (task_manager.isBallGrasped()) {
      contradium::rbd::UpdateKinematicsCustom(robot_model->rbd_model, &Q, nullptr, nullptr);
      contradium::rbd::Vector3 tcp_pos = contradium::rbd::CalcBodyToBaseCoordinates(
          robot_model->rbd_model, Q, robot_model->tcp_body_id,
          contradium::rbd::Vector3(0, 0, 0), false);
      glm::vec3 ball_pos(tcp_pos(0), tcp_pos(1), tcp_pos(2));
      ball_entity.SetTransformation(
          glm::translate(glm::mat4{1.0f}, ball_pos) *
          glm::scale(glm::mat4(1.0f), glm::vec3(config::BALL_RADIUS)));
    }

    // Update TCP camera
    contradium::rbd::UpdateKinematicsCustom(robot_model->rbd_model, &Q, nullptr, nullptr);
    contradium::rbd::Vector3 tcp_pos = contradium::rbd::CalcBodyToBaseCoordinates(
        robot_model->rbd_model, Q, robot_model->tcp_body_id,
        contradium::rbd::Vector3(0, 0, 0), false);
    contradium::rbd::Matrix3 tcp_rot = contradium::rbd::CalcBodyToBaseRotation(
        robot_model->rbd_model, Q, robot_model->tcp_body_id, false);

    glm::vec3 tcp_pos_glm(tcp_pos(0), tcp_pos(1), tcp_pos(2));
    glm::mat3 tcp_rot_glm;
    for (int col = 0; col < 3; ++col) {
      for (int row = 0; row < 3; ++row) {
        tcp_rot_glm[col][row] = static_cast<float>(tcp_rot.transpose()(row, col));
      }
    }
    glm::vec3 tcp_forward = tcp_rot_glm[2];
    glm::vec3 tcp_up = tcp_rot_glm[1];
    camera_tcp_view.view = glm::lookAt(tcp_pos_glm, tcp_pos_glm + tcp_forward, tcp_up);

    // Update axis frame visualizers
    glm::mat4 current_transform = urdf_utils::posRotToGLM(tcp_pos, tcp_rot.transpose());
    robot_visualizer.getCurrentFrame().setTransform(current_transform);

    // Update target frame based on task state
    if (task_manager.getCurrentStage() != TaskStage::IDLE &&
        task_manager.getCurrentStage() != TaskStage::COMPLETE) {
      // Show target from stage end configuration (trajectory goal)
      const auto& Q_stage_end = task_manager.getStageEnd();
      contradium::rbd::UpdateKinematicsCustom(robot_model->rbd_model, &Q_stage_end, nullptr, nullptr);
      contradium::rbd::Vector3 target_pos = contradium::rbd::CalcBodyToBaseCoordinates(
          robot_model->rbd_model, Q_stage_end, robot_model->tcp_body_id,
          contradium::rbd::Vector3(0, 0, 0), false);
      contradium::rbd::Matrix3 target_rot = contradium::rbd::CalcBodyToBaseRotation(
          robot_model->rbd_model, Q_stage_end, robot_model->tcp_body_id, false);
      glm::mat4 target_transform = urdf_utils::posRotToGLM(target_pos, target_rot.transpose());
      robot_visualizer.getTargetFrame().setTransform(target_transform);
    } else {
      // Show pre-grasp position as target
      contradium::rbd::Vector3 target_pos(ball_x, ball_y, config::BALL_RADIUS + config::PRE_GRASP_HEIGHT);
      contradium::rbd::Matrix3 target_rot = urdf_utils::rpyToRotationMatrix(M_PI, 0.0, 0.0);
      glm::mat4 target_transform = urdf_utils::posRotToGLM(target_pos, target_rot.transpose());
      robot_visualizer.getTargetFrame().setTransform(target_transform);
    }

    // =======================================================================
    // Render
    // =======================================================================

    auto pipeline = ray_tracing ? sparkium::RENDER_PIPELINE_AUTO
                                : sparkium::RENDER_PIPELINE_RASTERIZATION;

    // Main view
    sparkium_core.Render(&scene, &camera, &film, pipeline);
    film.Develop(srgb_image.get());

    // TCP view
    sparkium_core.Render(&scene, &camera_tcp_view, &film_tcp_view, pipeline);
    film_tcp_view.Develop(srgb_tcp_image.get());

    // Present
    std::unique_ptr<graphics::CommandContext> cmd_context;
    core_->CreateCommandContext(&cmd_context);
    cmd_context->CmdPresent(window.get(), srgb_image.get());
    core_->SubmitCommandContext(cmd_context.get());

    std::unique_ptr<graphics::CommandContext> cmd_context_tcp_view;
    core_->CreateCommandContext(&cmd_context_tcp_view);
    cmd_context_tcp_view->CmdPresent(tcp_view_window.get(), srgb_tcp_image.get());
    core_->SubmitCommandContext(cmd_context_tcp_view.get());

    glfwPollEvents();

    // Update window title
    float fps = fps_counter.TickFPS();
    char title[256];
    sprintf(title, "Franka Grasping - %.1f fps - %s",
            fps, task_manager.getStageName());
    window->SetTitle(std::string(title));

    frame_count++;
  }

  // =========================================================================
  // 8. Cleanup
  // =========================================================================

  LogInfo("Shutting down...");

  // Save final frame
  film.Develop(srgb_image.get());
  std::vector<uint8_t> image_data(config::RENDER_WIDTH * config::RENDER_HEIGHT * 4);
  srgb_image->DownloadData(image_data.data());
  stbi_write_bmp("franka_grasping_output.bmp", config::RENDER_WIDTH,
                 config::RENDER_HEIGHT, 4, image_data.data());

  return 0;
}
