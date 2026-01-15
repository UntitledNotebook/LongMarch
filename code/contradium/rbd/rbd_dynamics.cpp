#include "contradium/rbd/rbd_dynamics.h"

#include <cassert>

#include "contradium/rbd/rbd_model.h"

namespace contradium::rbd {
LM_DEVICE_FUNC void InverseDynamics(Model &model,
                                    const VectorX &Q,
                                    const VectorX &QDot,
                                    const VectorX &QDDot,
                                    VectorX &Tau,
                                    const std::vector<SpatialVector> *f_ext) {
  model.v[0] = SpatialVector::Zero();
  model.a[0] = SpatialVector(0, 0, 0, -model.gravity(0), -model.gravity(1), -model.gravity(2));

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    jcalc(model, i, Q, QDot);
    const unsigned int parent = model.lambda[i];
    model.v[i] = model.X_lambda[i].apply(model.v[parent]) + model.v_J[i];
    model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
    model.a[i] = model.X_lambda[i].apply(model.a[parent]) + model.c[i] + model.S[i] * QDDot[i - 1];
    model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i], model.I[i] * model.v[i]);
  }

  if (f_ext) {
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      const unsigned int parent = model.lambda[i];
      model.X_base[i] = model.X_lambda[i] * model.X_base[parent];
      model.f[i] -= model.X_base[i].applyAdjoint((*f_ext)[i]);
    }
  }

  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    Tau[i - 1] = model.S[i].dot(model.f[i]);
    if (model.lambda[i] != 0) {
      model.f[model.lambda[i]] += model.X_lambda[i].applyTranspose(model.f[i]);
    }
  }
}

LM_DEVICE_FUNC void CompositeRigidBodyAlgorithm(Model &model, const VectorX &Q, MatrixX &H,
                                                const bool update_kinematics) {
  assert(H.rows() == model.dof_count && H.cols() == model.dof_count);
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    if (update_kinematics) {
      jcalc_X_lambda_S(model, i, Q);
    }
    model.Ic[i] = model.I[i];
  }
  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if (model.lambda[i] != 0) {
      model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.Ic[i]);
    }
    SpatialVector F_ = model.Ic[i] * model.S[i];
    H(i - 1, i - 1) = model.S[i].dot(F_);
    unsigned int j = i;
    while (model.lambda[j] != 0) {
      F_ = model.X_lambda[j].applyTranspose(F_);
      j = model.lambda[j];
      H(i - 1, j - 1) = F_.dot(model.S[j]);
      H(j - 1, i - 1) = H(i - 1, j - 1);
    }
  }
}

LM_DEVICE_FUNC void ForwardDynamicsLagragian(Model &model,
                                             const VectorX &Q,
                                             const VectorX &QDot,
                                             const VectorX &Tau,
                                             VectorX &QDDot,
                                             const std::vector<SpatialVector> *f_ext) {
  assert(QDDot.size() == model.dof_count);
  const auto H = new MatrixX(MatrixX::Zero(model.dof_count, model.dof_count));
  const auto C = new VectorX(model.dof_count);
  QDDot.setZero();
  InverseDynamics(model, Q, QDot, QDDot, *C, f_ext);
  CompositeRigidBodyAlgorithm(model, Q, *H, false);
  QDDot = H->colPivHouseholderQr().solve(*C * -1. + Tau);
  delete H;
  delete C;
}

LM_DEVICE_FUNC void ForwardDynamics(Model &model,
                                    const VectorX &Q,
                                    const VectorX &QDot,
                                    const VectorX &Tau,
                                    VectorX &QDDot,
                                    const std::vector<SpatialVector> *f_ext) {
  model.v[0] = SpatialVector::Zero();
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    const unsigned int parent = model.lambda[i];
    jcalc(model, i, Q, QDot);
    if (parent != 0) {
      model.X_base[i] = model.X_lambda[i] * model.X_base[parent];
    } else {
      model.X_base[i] = model.X_lambda[i];
    }
    model.v[i] = model.X_lambda[i].apply(model.v[parent]) + model.v_J[i];
    model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
    model.I[i].setSpatialMatrix(model.IA[i]);
    model.pA[i] = crossf(model.v[i], model.I[i] * model.v[i]);
    if (f_ext && (*f_ext)[i] != SpatialVector::Zero()) {
      model.pA[i] -= model.X_base[i].applyAdjoint((*f_ext)[i]);
    }
  }
  for (int i = model.mBodies.size() - 1; i > 0; i--) {
    model.U[i] = model.IA[i] * model.S[i];
    model.d[i] = model.S[i].dot(model.U[i]);
    model.u[i] = Tau[i - 1] - model.S[i].dot(model.pA[i]);
    if (const unsigned int parent = model.lambda[i]; parent != 0) {
      SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
      SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];
      model.IA[parent].noalias() += model.X_lambda[i].ToMatrixTranspose() * Ia * model.X_lambda[i].ToMatrix();
      model.pA[parent].noalias() += model.X_lambda[i].applyTranspose(pa);
    }
  }
  model.a[0] = SpatialVector(0, 0, 0, -model.gravity(0), -model.gravity(1), -model.gravity(2));
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    model.a[i] = model.X_lambda[i].apply(model.a[model.lambda[i]]) + model.c[i];
    QDDot(i - 1) = 1. / model.d[i] * (model.u[i] - model.U[i].dot(model.a[i]));
    model.a[i] += model.S[i] * QDDot(i - 1);
  }
}
}// namespace contradium::rbd