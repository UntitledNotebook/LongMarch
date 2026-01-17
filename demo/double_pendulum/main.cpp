/**
 * @file main.cpp
 * @brief Double Pendulum Dynamics Verification Demo
 *
 * Simulates a double pendulum using rigid body dynamics and
 * compares with reference Lagrangian dynamics.
 *
 * Coordinate system: X outward, Y right, Z up
 * - Two uniform sticks, each with mass 1kg and length 1m
 * - Both rotate around the X axis
 * - Initially horizontal (along Y axis)
 * - Gravity: 9.81 m/s^2 towards negative Z
 */

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

#include <boost/numeric/odeint.hpp>

#include <long_march.h>
#include "grassland/util/log.h"

namespace rbd = contradium::rbd;
namespace odeint = boost::numeric::odeint;

using rbd::Body;
using rbd::CompositeRigidBodyAlgorithm;
using rbd::ForwardDynamics;
using rbd::ForwardDynamicsLagragian;
using rbd::InverseDynamics;
using rbd::Joint;
using rbd::JointTypeRevolute;
using rbd::Matrix3;
using rbd::MatrixX;
using rbd::Model;
using rbd::SpatialTransform;
using rbd::UpdateKinematics;
using rbd::Vector3;
using rbd::VectorX;

// Constants
constexpr double kGravity = 9.81;
constexpr double kMass = 1.0;
constexpr double kLength = 1.0;
constexpr double kSimulationTime = 10.0;
constexpr double kDt = 0.0001;
constexpr double kOutputDt = 0.01;

using State = std::vector<double>;

struct Record {
  double time;
  double q1, q2;
  double qdot1, qdot2;
  double qddot1_aba, qddot2_aba;
  double qddot1_crba, qddot2_crba;
  double qddot1_ref, qddot2_ref;
  double qddot1_diff, qddot2_diff;
  double kinetic_energy;
  double potential_energy;
  double total_energy;
};

struct ReferenceRecord {
  double time;
  double q1, q2;
  double qdot1, qdot2;
  double qddot1, qddot2;
  double kinetic_energy;
  double potential_energy;
  double total_energy;
};

Model* g_model = nullptr;

Model CreateModel() {
  Model model;
  model.gravity = Vector3(0, 0, -kGravity);

  Matrix3 inertia = Matrix3::Zero();
  const double I = (1.0 / 12.0) * kMass * kLength * kLength;
  inertia(0, 0) = I;
  inertia(2, 2) = I;

  Body rod_body(kMass, Vector3(0, kLength / 2.0, 0), inertia);
  Joint revolute(JointTypeRevolute, Vector3(1, 0, 0));

  model.AddBody(0, SpatialTransform(Matrix3::Identity(), Vector3::Zero()),
                revolute, rod_body);
  model.AddBody(1, SpatialTransform(Matrix3::Identity(), Vector3(0, kLength, 0)),
                revolute, rod_body);

  return model;
}

std::pair<double, double> CalculateEnergies(Model& model, const VectorX& q,
                                            const VectorX& qdot) {
  VectorX qddot = VectorX::Zero(model.dof_count);
  UpdateKinematics(model, q, qdot, qddot);

  double kinetic = 0.0;
  double potential = 0.0;

  for (unsigned int i = 1; i < model.mBodies.size(); ++i) {
    const Body& body = model.mBodies[i];
    const auto& v = model.v[i];
    kinetic += 0.5 * v.dot(model.I[i] * v);

    Vector3 com_world =
        model.X_base[i].E.transpose() * body.mCenterOfMass + model.X_base[i].r;
    potential += body.mMass * kGravity * com_world(2);
  }

  return {kinetic, potential};
}

void ComputeReferenceAccelerations(double q1, double q2, double q1dot, double q2dot,
                                   double& q1ddot, double& q2ddot) {
  const double M11 = 5.0/3.0 + std::cos(q2);
  const double M12 = 1.0/3.0 + 0.5*std::cos(q2);
  const double M22 = 1.0/3.0;

  const double sin_q2 = std::sin(q2);
  const double cos_q1 = std::cos(q1);
  const double cos_q1q2 = std::cos(q1 + q2);

  const double b1 = 0.5*sin_q2*(2*q1dot*q2dot + q2dot*q2dot)
                    - (kGravity/2.0)*(3*cos_q1 + cos_q1q2);
  const double b2 = -0.5*sin_q2*q1dot*q1dot - (kGravity/2.0)*cos_q1q2;

  const double det = M11*M22 - M12*M12;
  q1ddot = (M22*b1 - M12*b2) / det;
  q2ddot = (-M12*b1 + M11*b2) / det;
}

std::pair<double, double> ComputeReferenceLagrangianEnergy(
    double q1, double q2, double q1dot, double q2dot) {
  double KE = 0.5*(5.0/3.0 + std::cos(q2))*q1dot*q1dot
            + (1.0/6.0)*q2dot*q2dot
            + (1.0/3.0 + 0.5*std::cos(q2))*q1dot*q2dot;
  double PE = (kGravity/2.0)*(3*std::sin(q1) + std::sin(q1 + q2));
  return {KE, PE};
}

void RBDODE(const State& x, State& dxdt, double) {
  VectorX q(2), qdot(2), qddot(2), tau(2);
  q << x[0], x[1];
  qdot << x[2], x[3];
  tau.setZero();

  ForwardDynamics(*g_model, q, qdot, tau, qddot);

  dxdt[0] = qdot(0);
  dxdt[1] = qdot(1);
  dxdt[2] = qddot(0);
  dxdt[3] = qddot(1);
}

void ReferenceLagrangianODE(const State& x, State& dxdt, double) {
  double q1 = x[0], q2 = x[1];
  double q1dot = x[2], q2dot = x[3];

  double M11 = 5.0/3.0 + std::cos(q2);
  double M12 = 1.0/3.0 + 0.5*std::cos(q2);
  double M22 = 1.0/3.0;

  double sin_q2 = std::sin(q2);
  double cos_q1 = std::cos(q1);
  double cos_q1q2 = std::cos(q1 + q2);

  double b1 = 0.5*sin_q2*(2*q1dot*q2dot + q2dot*q2dot)
              - (kGravity/2.0)*(3*cos_q1 + cos_q1q2);
  double b2 = -0.5*sin_q2*q1dot*q1dot - (kGravity/2.0)*cos_q1q2;

  double det = M11*M22 - M12*M12;
  double q1ddot = (M22*b1 - M12*b2) / det;
  double q2ddot = (-M12*b1 + M11*b2) / det;

  dxdt[0] = q1dot;
  dxdt[1] = q2dot;
  dxdt[2] = q1ddot;
  dxdt[3] = q2ddot;
}

class Observer {
 public:
  std::vector<Record>& records;
  Model& model;
  double next_time;
  const double interval;

  Observer(std::vector<Record>& recs, Model& m, double dt)
      : records(recs), model(m), next_time(0.0), interval(dt) {}

  void operator()(const State& x, double t) {
    if (t < next_time - 1e-9) return;

    VectorX q(2), qdot(2), qddot_aba(2), qddot_crba(2), tau(2);
    q << x[0], x[1];
    qdot << x[2], x[3];
    tau.setZero();

    ForwardDynamics(model, q, qdot, tau, qddot_aba);
    ForwardDynamicsLagragian(model, q, qdot, tau, qddot_crba);

    double qddot1_ref, qddot2_ref;
    ComputeReferenceAccelerations(x[0], x[1], x[2], x[3], qddot1_ref, qddot2_ref);

    auto [ke, pe] = CalculateEnergies(model, q, qdot);

    records.push_back({t, x[0], x[1], x[2], x[3],
                       qddot_aba(0), qddot_aba(1),
                       qddot_crba(0), qddot_crba(1),
                       qddot1_ref, qddot2_ref,
                       qddot_aba(0) - qddot_crba(0), qddot_aba(1) - qddot_crba(1),
                       ke, pe, ke + pe});
    next_time += interval;
  }
};

void WriteCSV(const std::string& path, const std::vector<Record>& records) {
  std::ofstream file(path);
  if (!file.is_open()) {
    grassland::LogError("Failed to open {}", path);
    return;
  }

  file << "time,q1,q2,qdot1,qdot2,"
          "qddot1_aba,qddot2_aba,qddot1_crba,qddot2_crba,"
          "qddot1_ref,qddot2_ref,"
          "qddot1_diff,qddot2_diff,"
          "kinetic_energy,potential_energy,total_energy\n";

  file << std::scientific << std::setprecision(12);
  for (const auto& r : records) {
    file << r.time << "," << r.q1 << "," << r.q2 << ","
         << r.qdot1 << "," << r.qdot2 << ","
         << r.qddot1_aba << "," << r.qddot2_aba << ","
         << r.qddot1_crba << "," << r.qddot2_crba << ","
         << r.qddot1_ref << "," << r.qddot2_ref << ","
         << r.qddot1_diff << "," << r.qddot2_diff << ","
         << r.kinetic_energy << "," << r.potential_energy << ","
         << r.total_energy << "\n";
  }

  grassland::LogInfo("Wrote {} records to {}", records.size(), path);
}

void WriteReferenceCSV(const std::string& path,
                       const std::vector<ReferenceRecord>& records) {
  std::ofstream file(path);
  if (!file.is_open()) {
    grassland::LogError("Failed to open {}", path);
    return;
  }

  file << "time,q1,q2,qdot1,qdot2,qddot1,qddot2,"
          "kinetic_energy,potential_energy,total_energy\n";

  file << std::scientific << std::setprecision(12);
  for (const auto& r : records) {
    file << r.time << "," << r.q1 << "," << r.q2 << ","
         << r.qdot1 << "," << r.qdot2 << ","
         << r.qddot1 << "," << r.qddot2 << ","
         << r.kinetic_energy << "," << r.potential_energy << ","
         << r.total_energy << "\n";
  }

  grassland::LogInfo("Wrote {} reference records to {}", records.size(), path);
}

int main(int argc, char* argv[]) {
  grassland::LogInfo("=== Double Pendulum Simulation ===");

  std::string output_dir = (argc > 1) ? argv[1] : "output";
  std::filesystem::create_directories(output_dir);

  Model model = CreateModel();
  g_model = &model;

  grassland::LogInfo("DOF: {}, Simulation time: {:.1f}s, dt: {:.4f}s",
                     model.dof_count, kSimulationTime, kDt);

  // === RBD Simulation ===
  State x{0.0, 0.0, 0.0, 0.0};

  VectorX q0(2), qdot0(2);
  q0 << x[0], x[1];
  qdot0 << x[2], x[3];
  auto [ke0, pe0] = CalculateEnergies(model, q0, qdot0);
  double initial_energy = ke0 + pe0;

  grassland::LogInfo("Initial energy: KE={:.6f}J, PE={:.6f}J, Total={:.6f}J",
                     ke0, pe0, initial_energy);

  std::vector<Record> records;
  records.reserve(static_cast<size_t>(kSimulationTime / kOutputDt) + 1);
  Observer observer(records, model, kOutputDt);

  grassland::LogInfo("Running RBD simulation...");
  size_t steps = odeint::integrate_const(
      odeint::runge_kutta4<State>(), RBDODE, x, 0.0, kSimulationTime, kDt,
      observer);
  grassland::LogInfo("Completed {} steps", steps);

  VectorX qf(2), qdotf(2);
  qf << x[0], x[1];
  qdotf << x[2], x[3];
  auto [kef, pef] = CalculateEnergies(model, qf, qdotf);
  double final_energy = kef + pef;

  std::string csv_path = std::filesystem::path(output_dir) / "simulation_data.csv";
  WriteCSV(csv_path, records);

  // === Reference Lagrangian Simulation ===
  grassland::LogInfo("Running reference Lagrangian simulation...");

  State x_ref{0.0, 0.0, 0.0, 0.0};

  auto [ke0_ref, pe0_ref] = ComputeReferenceLagrangianEnergy(
      x_ref[0], x_ref[1], x_ref[2], x_ref[3]);
  double initial_ref_energy = ke0_ref + pe0_ref;

  std::vector<ReferenceRecord> ref_records;
  ref_records.reserve(static_cast<size_t>(kSimulationTime / kOutputDt) + 1);

  double next_ref_time = 0.0;
  auto ref_observer = [&](const State& state, double t) {
    if (t >= next_ref_time - 1e-9) {
      double qddot1, qddot2;
      ComputeReferenceAccelerations(state[0], state[1], state[2], state[3],
                                     qddot1, qddot2);
      auto [ke, pe] = ComputeReferenceLagrangianEnergy(
          state[0], state[1], state[2], state[3]);

      ref_records.push_back({t, state[0], state[1], state[2], state[3],
                             qddot1, qddot2, ke, pe, ke + pe});
      next_ref_time += kOutputDt;
    }
  };

  size_t ref_steps = odeint::integrate_const(
      odeint::runge_kutta4<State>(),
      ReferenceLagrangianODE, x_ref, 0.0, kSimulationTime, kDt * 0.1,
      ref_observer);
  grassland::LogInfo("Completed {} reference steps", ref_steps);

  auto [kef_ref, pef_ref] = ComputeReferenceLagrangianEnergy(
      x_ref[0], x_ref[1], x_ref[2], x_ref[3]);
  double final_ref_energy = kef_ref + pef_ref;

  std::string ref_csv_path = std::filesystem::path(output_dir) / "reference_data.csv";
  WriteReferenceCSV(ref_csv_path, ref_records);

  // === Error Summary ===
  double max_q1_error = 0.0, max_q2_error = 0.0;
  double max_qdot1_error = 0.0, max_qdot2_error = 0.0;
  double max_qddot1_error = 0.0, max_qddot2_error = 0.0;

  size_t n = std::min(records.size(), ref_records.size());
  for (size_t i = 0; i < n; ++i) {
    max_q1_error = std::max(max_q1_error,
                            std::abs(records[i].q1 - ref_records[i].q1));
    max_q2_error = std::max(max_q2_error,
                            std::abs(records[i].q2 - ref_records[i].q2));
    max_qdot1_error = std::max(max_qdot1_error,
                               std::abs(records[i].qdot1 - ref_records[i].qdot1));
    max_qdot2_error = std::max(max_qdot2_error,
                               std::abs(records[i].qdot2 - ref_records[i].qdot2));
    max_qddot1_error = std::max(max_qddot1_error,
                                std::abs(records[i].qddot1_aba - ref_records[i].qddot1));
    max_qddot2_error = std::max(max_qddot2_error,
                                std::abs(records[i].qddot2_aba - ref_records[i].qddot2));
  }

  grassland::LogInfo("Max |q1_rbd - q1_ref|:     {:.3e} rad", max_q1_error);
  grassland::LogInfo("Max |q2_rbd - q2_ref|:     {:.3e} rad", max_q2_error);
  grassland::LogInfo("Max |qdot1_rbd - qdot1_ref|: {:.3e} rad/s", max_qdot1_error);
  grassland::LogInfo("Max |qdot2_rbd - qdot2_ref|: {:.3e} rad/s", max_qdot2_error);
  grassland::LogInfo("Max |qddot1_aba - qddot1_ref|: {:.3e} rad/s²", max_qddot1_error);
  grassland::LogInfo("Max |qddot2_aba - qddot2_ref|: {:.3e} rad/s²", max_qddot2_error);

  double max_energy_error = 0.0;
  double max_ref_energy_error = 0.0;
  for (const auto& r : records) {
    max_energy_error = std::max(max_energy_error,
                                std::abs(r.total_energy - initial_energy));
  }
  for (const auto& r : ref_records) {
    max_ref_energy_error = std::max(max_ref_energy_error,
                                    std::abs(r.total_energy - initial_ref_energy));
  }

  grassland::LogInfo("RBD max energy error:     {:.3e} J", max_energy_error);
  grassland::LogInfo("Ref max energy error:     {:.3e} J", max_ref_energy_error);

  grassland::LogInfo("Output: {}", csv_path);
  grassland::LogInfo("Output: {}", ref_csv_path);

  return 0;
}
