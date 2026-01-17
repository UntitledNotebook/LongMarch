/**
 * @file main.cpp
 * @brief Forward Dynamics Algorithm Benchmark with Multiprocess Testing
 *
 * This demo benchmarks the running time of forward dynamics algorithms
 * (ABA - Articulated Body Algorithm and CRBA - Composite Rigid Body Algorithm)
 * on different kinematic tree structures:
 *
 * 1. Chain: A serial chain of bodies (depth = n, branching = 1)
 * 2. Full Binary Tree: Complete binary tree (depth = log2(n), branching = 2)
 * 3. Star (Depth-1 Tree): All bodies attached to root (depth = 1, branching = n)
 *
 * Features:
 * - Random model initialization (random joint frames, inertias, masses)
 * - Large body sizes up to 2000 bodies
 * - Multiprocess testing to prevent blocking on slow tests
 * - Timeout handling for very slow CRBA computations
 *
 * Results are saved to CSV for analysis and plotting.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <long_march.h>

namespace rbd = contradium::rbd;

using rbd::Body;
using rbd::CompositeRigidBodyAlgorithm;
using rbd::ForwardDynamics;
using rbd::ForwardDynamicsLagragian;
using rbd::Joint;
using rbd::JointTypeRevolute;
using rbd::Matrix3;
using rbd::MatrixX;
using rbd::Model;
using rbd::SpatialRigidBodyInertia;
using rbd::SpatialTransform;
using rbd::SpatialVector;
using rbd::Vector3;
using rbd::VectorX;

// Benchmark parameters
constexpr int NUM_ITERATIONS = 5;       // Number of iterations per benchmark
constexpr int WARMUP_ITERATIONS = 1;     // Warmup iterations before timing
constexpr int TIMEOUT_SECONDS = 600;       // Timeout for each benchmark in seconds
constexpr double CRBA_SKIP_THRESHOLD_BODIES = 3000;  // Skip CRBA for very large models
std::string output_dir = "output";        // Output directory

// Global random number generator
thread_local std::mt19937 g_rng;

/**
 * @brief Structure to hold benchmark results
 */
struct BenchmarkResult {
  std::string tree_type;
  int num_bodies;
  double aba_time_us;    // Articulated Body Algorithm time in microseconds
  double crba_time_us;   // Composite Rigid Body Algorithm time in microseconds (-1 if skipped)
  int tree_depth;
  int max_branching;
};

/**
 * @brief Generate a random positive definite inertia matrix
 */
Matrix3 RandomInertia(std::mt19937& rng) {
  std::uniform_real_distribution<double> dist(0.1, 2.0);

  // Generate random principal moments of inertia
  double Ixx = dist(rng);
  double Iyy = dist(rng);
  double Izz = dist(rng);

  // Ensure triangle inequality for physical validity
  // |Ii - Ij| <= Ik <= Ii + Ij for all permutations
  double max_I = std::max({Ixx, Iyy, Izz});
  double sum_I = Ixx + Iyy + Izz;
  if (max_I > sum_I - max_I) {
    // Scale to ensure validity
    double scale = (sum_I - max_I) / max_I * 0.9;
    if (Ixx == max_I)
      Ixx *= scale;
    else if (Iyy == max_I)
      Iyy *= scale;
    else
      Izz *= scale;
  }

  Matrix3 inertia = Matrix3::Zero();
  inertia(0, 0) = Ixx;
  inertia(1, 1) = Iyy;
  inertia(2, 2) = Izz;

  return inertia;
}

/**
 * @brief Generate a random rotation matrix
 */
Matrix3 RandomRotation(std::mt19937& rng) {
  std::uniform_real_distribution<double> dist(-M_PI, M_PI);

  double roll = dist(rng);
  double pitch = dist(rng);
  double yaw = dist(rng);

  // Rotation matrices for each axis
  Matrix3 Rx, Ry, Rz;

  Rx << 1, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, std::sin(roll),
      std::cos(roll);

  Ry << std::cos(pitch), 0, std::sin(pitch), 0, 1, 0, -std::sin(pitch), 0,
      std::cos(pitch);

  Rz << std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0,
      1;

  return Rz * Ry * Rx;
}

/**
 * @brief Generate a random joint axis (unit vector)
 */
Vector3 RandomJointAxis(std::mt19937& rng) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Vector3 axis;
  do {
    axis = Vector3(dist(rng), dist(rng), dist(rng));
  } while (axis.norm() < 0.1);

  return axis.normalized();
}

/**
 * @brief Create a random body with random mass, CoM, and inertia
 */
Body CreateRandomBody(std::mt19937& rng) {
  std::uniform_real_distribution<double> mass_dist(0.5, 5.0);
  std::uniform_real_distribution<double> com_dist(-0.5, 0.5);

  double mass = mass_dist(rng);
  Vector3 com(com_dist(rng), com_dist(rng), com_dist(rng));
  Matrix3 inertia = RandomInertia(rng);

  return Body(mass, com, inertia);
}

/**
 * @brief Create a random spatial transform for joint frame
 */
SpatialTransform RandomJointFrame(std::mt19937& rng,
                                  double translation_scale = 1.0) {
  std::uniform_real_distribution<double> trans_dist(-translation_scale,
                                                    translation_scale);

  Matrix3 rotation = RandomRotation(rng);
  Vector3 translation(trans_dist(rng), trans_dist(rng), trans_dist(rng));

  // Ensure non-zero translation for proper kinematics
  if (translation.norm() < 0.1) {
    translation = Vector3(translation_scale, 0, 0);
  }

  return SpatialTransform(rotation, translation);
}

/**
 * @brief Create a random chain kinematic tree (serial manipulator)
 */
Model CreateRandomChainModel(int num_bodies, std::mt19937& rng) {
  Model model;
  model.gravity = Vector3(0, 0, -9.81);

  // Add first body attached to root
  Body first_body = CreateRandomBody(rng);
  Joint first_joint(JointTypeRevolute, RandomJointAxis(rng));
  SpatialTransform first_frame(Matrix3::Identity(), Vector3::Zero());
  model.AddBody(0, first_frame, first_joint, first_body);

  // Add remaining bodies in chain
  for (int i = 1; i < num_bodies; ++i) {
    Body body = CreateRandomBody(rng);
    Joint joint(JointTypeRevolute, RandomJointAxis(rng));
    SpatialTransform frame = RandomJointFrame(rng);
    model.AddBody(i, frame, joint, body);
  }

  return model;
}

/**
 * @brief Create a random full binary tree kinematic tree
 *
 * Structure: Each internal node has exactly 2 children
 * Body IDs are assigned level-by-level (BFS order)
 *
 * Example for depth 2:
 *          0 (root)
 *         / \
 *        1   2
 *       /\   /\
 *      3  4 5  6
 */
Model CreateRandomBinaryTreeModel(int depth, std::mt19937& rng) {
  Model model;
  model.gravity = Vector3(0, 0, -9.81);

  // Add root body
  Body root_body = CreateRandomBody(rng);
  Joint root_joint(JointTypeRevolute, RandomJointAxis(rng));
  SpatialTransform first_frame(Matrix3::Identity(), Vector3::Zero());
  model.AddBody(0, first_frame, root_joint, root_body);

  if (depth == 0)
    return model;

  // Build tree level by level
  for (int level = 1; level <= depth; ++level) {
    int nodes_at_level = 1 << level;
    int first_node_prev_level = (1 << (level - 1)) - 1;

    for (int i = 0; i < nodes_at_level; ++i) {
      int parent = first_node_prev_level + (i / 2);
      unsigned int parent_body_id = parent + 1;

      Body body = CreateRandomBody(rng);
      Joint joint(JointTypeRevolute, RandomJointAxis(rng));
      SpatialTransform frame = RandomJointFrame(rng);

      model.AddBody(parent_body_id, frame, joint, body);
    }
  }

  return model;
}

/**
 * @brief Create a random star (depth-1) kinematic tree
 *
 * Structure: All bodies directly connected to root
 *        0 (root)
 *      / | | | \
 *     1  2 3 4  ...  n
 */
Model CreateRandomStarModel(int num_bodies, std::mt19937& rng) {
  Model model;
  model.gravity = Vector3(0, 0, -9.81);

  // Distribute bodies radially around the root with random perturbations
  for (int i = 0; i < num_bodies; ++i) {
    double angle = 2.0 * M_PI * i / num_bodies;
    std::uniform_real_distribution<double> perturb(-0.2, 0.2);

    Vector3 base_offset(std::cos(angle), std::sin(angle), 0.0);
    Vector3 offset = base_offset + Vector3(perturb(rng), perturb(rng), perturb(rng));

    Matrix3 rotation = RandomRotation(rng);
    SpatialTransform frame(rotation, offset);

    Body body = CreateRandomBody(rng);
    Joint joint(JointTypeRevolute, RandomJointAxis(rng));

    model.AddBody(0, frame, joint, body);
  }

  return model;
}

/**
 * @brief Run benchmark for a given model (ABA only)
 */
double RunABABenchmark(Model& model) {
  int dof = model.dof_count;

  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  VectorX Q(dof), QDot(dof), Tau(dof), QDDot(dof);
  for (int i = 0; i < dof; ++i) {
    Q(i) = dist(g_rng);
    QDot(i) = dist(g_rng);
    Tau(i) = dist(g_rng);
  }

  // Warmup
  for (int i = 0; i < WARMUP_ITERATIONS; ++i) {
    ForwardDynamics(model, Q, QDot, Tau, QDDot);
  }

  // Time ABA
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    ForwardDynamics(model, Q, QDot, Tau, QDDot);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  return duration.count() / (1000.0 * NUM_ITERATIONS);
}

/**
 * @brief Run benchmark for a given model (CRBA only)
 */
double RunCRBABenchmark(Model& model) {
  int dof = model.dof_count;

  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  VectorX Q(dof), QDot(dof), Tau(dof), QDDot(dof);
  for (int i = 0; i < dof; ++i) {
    Q(i) = dist(g_rng);
    QDot(i) = dist(g_rng);
    Tau(i) = dist(g_rng);
  }

  // Warmup
  for (int i = 0; i < WARMUP_ITERATIONS; ++i) {
    ForwardDynamicsLagragian(model, Q, QDot, Tau, QDDot);
  }

  // Time CRBA
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    ForwardDynamicsLagragian(model, Q, QDot, Tau, QDDot);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
  return duration.count() / (1000.0 * NUM_ITERATIONS);
}

/**
 * @brief Structure for passing benchmark task via pipe
 */
struct BenchmarkTask {
  char tree_type[32];
  int num_bodies;
  int tree_depth;
  int max_branching;
  unsigned int seed;
  bool run_crba;
};

/**
 * @brief Structure for benchmark result via pipe
 */
struct BenchmarkResultData {
  double aba_time_us;
  double crba_time_us;
  bool success;
};

/**
 * @brief Run a single benchmark in a child process with timeout
 */
bool RunBenchmarkWithTimeout(const BenchmarkTask& task,
                             BenchmarkResultData& result_data) {
  int pipe_fd[2];
  if (pipe(pipe_fd) == -1) {
    perror("pipe");
    return false;
  }

  pid_t pid = fork();

  if (pid == -1) {
    perror("fork");
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    return false;
  }

  if (pid == 0) {
    // Child process
    close(pipe_fd[0]);  // Close read end

    g_rng.seed(task.seed);

    Model model;
    std::string tree_type(task.tree_type);

    if (tree_type == "chain") {
      model = CreateRandomChainModel(task.num_bodies, g_rng);
    } else if (tree_type == "binary_tree") {
      model = CreateRandomBinaryTreeModel(task.tree_depth - 1, g_rng);
    } else if (tree_type == "star") {
      model = CreateRandomStarModel(task.num_bodies, g_rng);
    }

    BenchmarkResultData res;
    res.aba_time_us = RunABABenchmark(model);

    if (task.run_crba) {
      res.crba_time_us = RunCRBABenchmark(model);
    } else {
      res.crba_time_us = -1.0;
    }
    res.success = true;

    write(pipe_fd[1], &res, sizeof(res));
    close(pipe_fd[1]);
    _exit(0);
  } else {
    // Parent process
    close(pipe_fd[1]);  // Close write end

    // Set up timeout
    int status;
    int elapsed = 0;
    bool child_done = false;

    while (elapsed < TIMEOUT_SECONDS) {
      pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == pid) {
        child_done = true;
        break;
      } else if (result == -1) {
        perror("waitpid");
        break;
      }
      sleep(1);
      elapsed++;
    }

    if (!child_done) {
      // Timeout - kill child
      kill(pid, SIGKILL);
      waitpid(pid, &status, 0);
      close(pipe_fd[0]);
      result_data.success = false;
      return false;
    }

    // Read result from pipe
    ssize_t bytes_read = read(pipe_fd[0], &result_data, sizeof(result_data));
    close(pipe_fd[0]);

    if (bytes_read != sizeof(result_data)) {
      result_data.success = false;
      return false;
    }

    return result_data.success;
  }
}

/**
 * @brief Write benchmark results to CSV file
 */
void WriteResultsCSV(const std::string& filename,
                     const std::vector<BenchmarkResult>& results) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    grassland::LogError("Could not open file: {}", filename);
    return;
  }

  // Header
  file << "tree_type,num_bodies,aba_time_us,crba_time_us,tree_depth,max_"
          "branching\n";

  // Data
  file << std::fixed << std::setprecision(6);
  for (const auto& result : results) {
    file << result.tree_type << "," << result.num_bodies << ","
         << result.aba_time_us << "," << result.crba_time_us << ","
         << result.tree_depth << "," << result.max_branching << "\n";
  }

  file.close();
  grassland::LogInfo("Wrote {} results to {}", results.size(), filename);
}

/**
 * @brief Append a single result to CSV file
 */
void AppendResultCSV(const std::string& filename,
                     const BenchmarkResult& result,
                     bool write_header = false) {
  std::ofstream file(filename, std::ios::app);
  if (!file.is_open()) {
    grassland::LogError("Could not open file: {}", filename);
    return;
  }

  if (write_header) {
    file << "tree_type,num_bodies,aba_time_us,crba_time_us,tree_depth,max_"
            "branching\n";
  }

  file << std::fixed << std::setprecision(6);
  file << result.tree_type << "," << result.num_bodies << ","
       << result.aba_time_us << "," << result.crba_time_us << ","
       << result.tree_depth << "," << result.max_branching << "\n";

  file.close();
}

/**
 * @brief Print progress
 */
void PrintBenchmarkInfo(const std::string& tree_type,
                        int num_bodies,
                        int current,
                        int total) {
  grassland::LogInfo("[{}/{}] {} with {} bodies...", current, total, tree_type,
                     num_bodies);
}

int main(int argc, char* argv[]) {
  grassland::LogInfo("=== Forward Dynamics Algorithm Benchmark (Multiprocess) ===");
  grassland::LogInfo("Iterations per benchmark: {}", NUM_ITERATIONS);
  grassland::LogInfo("Warmup iterations: {}", WARMUP_ITERATIONS);
  grassland::LogInfo("Timeout per benchmark: {} seconds", TIMEOUT_SECONDS);
  grassland::LogInfo("CRBA skip threshold: {} bodies", CRBA_SKIP_THRESHOLD_BODIES);
  grassland::LogInfo("");

  // Parse command line arguments
  if (argc > 1) {
    output_dir = argv[1];
  }

  // Create output directory
  std::filesystem::create_directories(output_dir);

  std::string csv_path = output_dir + "/benchmark_results.csv";

  // Remove old results file if exists
  std::filesystem::remove(csv_path);

  std::vector<BenchmarkResult> results;

  // Define body counts to test - larger range
  std::vector<int> chain_sizes = {4,   8,   16,  32,   64,   128,
                                  256, 512, 768, 1024, 1536, 2048};
  std::vector<int> binary_tree_depths = {
      2, 3, 4, 5, 6, 7, 8, 9, 10};  // Gives 7, 15, 31, 63, 127, 255, 511, 1023, 2047 bodies
  std::vector<int> star_sizes = {4,   8,   16,  32,   64,   128,
                                 256, 512, 768, 1024, 1536, 2048};

  // Calculate total benchmarks
  int total_benchmarks =
      chain_sizes.size() + binary_tree_depths.size() + star_sizes.size();
  int current_benchmark = 0;

  // Random seed for reproducibility
  std::random_device rd;
  unsigned int base_seed = rd();

  bool first_write = true;

  // Benchmark Chain Models
  grassland::LogInfo("--- Benchmarking Chain Models ---");
  for (size_t idx = 0; idx < chain_sizes.size(); ++idx) {
    int size = chain_sizes[idx];
    PrintBenchmarkInfo("Chain", size, ++current_benchmark, total_benchmarks);

    BenchmarkTask task;
    std::strncpy(task.tree_type, "chain", sizeof(task.tree_type));
    task.num_bodies = size;
    task.tree_depth = size;
    task.max_branching = 1;
    task.seed = base_seed + idx;
    task.run_crba = (size <= CRBA_SKIP_THRESHOLD_BODIES);

    BenchmarkResultData result_data;
    bool success = RunBenchmarkWithTimeout(task, result_data);

    BenchmarkResult result;
    result.tree_type = "chain";
    result.num_bodies = size;
    result.tree_depth = size;
    result.max_branching = 1;

    if (success) {
      result.aba_time_us = result_data.aba_time_us;
      result.crba_time_us = result_data.crba_time_us;
      if (result.crba_time_us >= 0) {
        grassland::LogInfo("ABA: {:.2f} us, CRBA: {:.2f} us", result.aba_time_us,
                           result.crba_time_us);
      } else {
        grassland::LogInfo("ABA: {:.2f} us, CRBA: skipped", result.aba_time_us);
      }
    } else {
      result.aba_time_us = -1;
      result.crba_time_us = -1;
      grassland::LogWarning("TIMEOUT");
    }

    results.push_back(result);
    AppendResultCSV(csv_path, result, first_write);
    first_write = false;
  }

  // Benchmark Binary Tree Models
  grassland::LogInfo("--- Benchmarking Binary Tree Models ---");
  for (size_t idx = 0; idx < binary_tree_depths.size(); ++idx) {
    int depth = binary_tree_depths[idx];
    int num_bodies = (1 << (depth + 1)) - 1;  // 2^(depth+1) - 1
    PrintBenchmarkInfo("Binary Tree", num_bodies, ++current_benchmark,
                       total_benchmarks);

    BenchmarkTask task;
    std::strncpy(task.tree_type, "binary_tree", sizeof(task.tree_type));
    task.num_bodies = num_bodies;
    task.tree_depth = depth + 1;
    task.max_branching = 2;
    task.seed = base_seed + chain_sizes.size() + idx;
    task.run_crba = (num_bodies <= CRBA_SKIP_THRESHOLD_BODIES);

    BenchmarkResultData result_data;
    bool success = RunBenchmarkWithTimeout(task, result_data);

    BenchmarkResult result;
    result.tree_type = "binary_tree";
    result.num_bodies = num_bodies;
    result.tree_depth = depth + 1;
    result.max_branching = 2;

    if (success) {
      result.aba_time_us = result_data.aba_time_us;
      result.crba_time_us = result_data.crba_time_us;
      if (result.crba_time_us >= 0) {
        grassland::LogInfo("ABA: {:.2f} us, CRBA: {:.2f} us", result.aba_time_us,
                           result.crba_time_us);
      } else {
        grassland::LogInfo("ABA: {:.2f} us, CRBA: skipped", result.aba_time_us);
      }
    } else {
      result.aba_time_us = -1;
      result.crba_time_us = -1;
      grassland::LogWarning("TIMEOUT");
    }

    results.push_back(result);
    AppendResultCSV(csv_path, result, false);
  }

  // Benchmark Star Models
  grassland::LogInfo("--- Benchmarking Star Models ---");
  for (size_t idx = 0; idx < star_sizes.size(); ++idx) {
    int size = star_sizes[idx];
    PrintBenchmarkInfo("Star", size, ++current_benchmark, total_benchmarks);

    BenchmarkTask task;
    std::strncpy(task.tree_type, "star", sizeof(task.tree_type));
    task.num_bodies = size;
    task.tree_depth = 1;
    task.max_branching = size;
    task.seed = base_seed + chain_sizes.size() + binary_tree_depths.size() + idx;
    task.run_crba = (size <= CRBA_SKIP_THRESHOLD_BODIES);

    BenchmarkResultData result_data;
    bool success = RunBenchmarkWithTimeout(task, result_data);

    BenchmarkResult result;
    result.tree_type = "star";
    result.num_bodies = size;
    result.tree_depth = 1;
    result.max_branching = size;

    if (success) {
      result.aba_time_us = result_data.aba_time_us;
      result.crba_time_us = result_data.crba_time_us;
      if (result.crba_time_us >= 0) {
        grassland::LogInfo("ABA: {:.2f} us, CRBA: {:.2f} us", result.aba_time_us,
                           result.crba_time_us);
      } else {
        grassland::LogInfo("ABA: {:.2f} us, CRBA: skipped", result.aba_time_us);
      }
    } else {
      result.aba_time_us = -1;
      result.crba_time_us = -1;
      grassland::LogWarning("TIMEOUT");
    }

    results.push_back(result);
    AppendResultCSV(csv_path, result, false);
  }

  // Print summary
  grassland::LogInfo("=== Summary ===");
  grassland::LogInfo("{:<15} {:<12} {:<18} {:<18} {:<10} {:<12}", "Tree Type",
                     "Bodies", "ABA (us)", "CRBA (us)", "Depth", "Branching");
  grassland::LogInfo("{}", std::string(85, '-'));

  for (const auto& r : results) {
    std::string aba_str = (r.aba_time_us >= 0)
                              ? fmt::format("{:.2f}", r.aba_time_us)
                              : "TIMEOUT";
    std::string crba_str =
        (r.crba_time_us >= 0)
            ? fmt::format("{:.2f}", r.crba_time_us)
            : (r.aba_time_us >= 0 ? "skipped" : "TIMEOUT");
    grassland::LogInfo("{:<15} {:<12} {:<18} {:<18} {:<10} {:<12}", r.tree_type,
                       r.num_bodies, aba_str, crba_str, r.tree_depth,
                       r.max_branching);
  }

  grassland::LogInfo("Results saved to: {}", csv_path);
  grassland::LogInfo("Run plot_benchmark.py to visualize the results.");

  return 0;
}
