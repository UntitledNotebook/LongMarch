#include "contradium/rbd/rbd_kinematics.h"

#include <cassert>

#include "contradium/rbd/rbd_model.h"

namespace contradium::rbd {
LM_DEVICE_FUNC void UpdateKinematics(Model &model, const VectorX &Q, const VectorX &QDot, const VectorX &QDDot) {
  assert(Q.size() == model.dof_count && QDot.size() == model.dof_count && QDDot.size() == model.dof_count);
  model.a[0] = SpatialVector::Zero();
  model.v[0] = SpatialVector::Zero();
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    jcalc(model, i, Q, QDot);
    const unsigned int parent = model.lambda[i];
    if (parent != 0) {
      model.X_base[i] = model.X_lambda[i] * model.X_base[parent];
      model.v[i] = model.X_lambda[i].apply(model.v[parent]) + model.v_J[i];
    } else {
      model.X_base[i] = model.X_lambda[i];
      model.v[i] = model.v_J[i];
    }
    model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
    model.a[i] = model.X_lambda[i].apply(model.a[parent]) + model.S[i] * QDDot[i - 1] + model.c[i];
  }
}

LM_DEVICE_FUNC void UpdateKinematicsCustom(Model &model, const VectorX *Q, const VectorX *QDot, const VectorX *QDDot) {
  model.a[0] = SpatialVector::Zero();
  model.v[0] = SpatialVector::Zero();
  if (Q) {
    assert(Q->size() == model.dof_count);
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      const unsigned int parent = model.lambda[i];
      VectorX QDot_zero(VectorX::Zero(model.dof_count));
      jcalc(model, i, *Q, QDot_zero);
      if (parent != 0) {
        model.X_base[i] = model.X_lambda[i] * model.X_base[parent];
      } else {
        model.X_base[i] = model.X_lambda[i];
      }
    }
  }
  if (QDot) {
    assert(QDot->size() == model.dof_count && Q);
    model.v[0] = SpatialVector::Zero();
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      const unsigned int parent = model.lambda[i];
      jcalc(model, i, *Q, *QDot);
      if (parent != 0) {
        model.v[i] = model.X_lambda[i].apply(model.v[parent]) + model.v_J[i];
        model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
      } else {
        model.v[i] = model.v_J[i];
        model.c[i] = model.c_J[i] + crossm(model.v[i], model.v_J[i]);
      }
    }
  }
  if (QDDot) {
    assert(QDDot->size() == model.dof_count && Q && QDot);
    model.a[0] = SpatialVector::Zero();
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      if (unsigned int parent = model.lambda[i]; parent != 0) {
        model.a[i] = model.X_lambda[i].apply(model.a[parent]) + model.c[i] + model.S[i] * (*QDDot)[i - 1];
      } else {
        model.a[i] = model.c[i] + model.S[i] * (*QDDot)[i - 1];
      }
    }
  }
}

LM_DEVICE_FUNC Vector3 CalcBodyToBaseCoordinates(Model &model,
                                                 const VectorX &Q,
                                                 const unsigned int body_id,
                                                 const Vector3 &position,
                                                 bool update_kinematics) {
  if (update_kinematics) {
    UpdateKinematicsCustom(model, &Q, nullptr, nullptr);
  }
  if (body_id >= model.fixed_body_discriminator) {
    const unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    const unsigned int parent_id = model.mFixedBodies[fbody_id].mMovableParent;
    return model.X_base[parent_id].r + model.X_base[parent_id].E.transpose() *
                                           (model.mFixedBodies[fbody_id].mParentTransform.r +
                                            model.mFixedBodies[fbody_id].mParentTransform.E.transpose() * position);
  }

  return model.X_base[body_id].r + model.X_base[body_id].E.transpose() * position;
}

LM_DEVICE_FUNC Matrix3 CalcBodyToBaseRotation(Model &model,
                                              const VectorX &Q,
                                              unsigned int body_id,
                                              bool update_kinematics) {
  if (update_kinematics) {
    UpdateKinematicsCustom(model, &Q, nullptr, nullptr);
  }
  if (body_id >= model.fixed_body_discriminator) {
    const unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    const unsigned int parent_id = model.mFixedBodies[fbody_id].mMovableParent;
    // Spatial transform multiplication: (A * B).E = A.E * B.E
    return (model.mFixedBodies[fbody_id].mParentTransform.E *
            model.X_base[parent_id].E);
  }
  return model.X_base[body_id].E;
}

LM_DEVICE_FUNC void CalcPointJacobian(Model &model,
                                      const VectorX &Q,
                                      const unsigned int body_id,
                                      const Vector3 &point,
                                      MatrixX &G,
                                      const bool update_kinematics) {
  assert(G.rows() == 3 && G.cols() == model.dof_count);
  if (update_kinematics) {
    UpdateKinematicsCustom(model, &Q, nullptr, nullptr);
  }
  const SpatialTransform trans(Matrix3::Identity(), CalcBodyToBaseCoordinates(model, Q, body_id, point, false));
  unsigned int reference_body_id = body_id;
  if (body_id > model.fixed_body_discriminator) {
    const unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
  }
  for (unsigned int j = reference_body_id; j != 0; j = model.lambda[j]) {
    G.col(j - 1) = trans.apply(model.X_base[j].Inverse().apply(model.S[j])).tail<3>();
  }
}

LM_DEVICE_FUNC void CalcPointJacobian6D(Model &model,
                                        const VectorX &Q,
                                        unsigned int body_id,
                                        const Vector3 &point,
                                        MatrixX &G,
                                        bool update_kinematics) {
  assert(G.rows() == 6 && G.cols() == model.dof_count);
  if (update_kinematics) {
    UpdateKinematicsCustom(model, &Q, nullptr, nullptr);
  }
  const SpatialTransform trans(Matrix3::Identity(), CalcBodyToBaseCoordinates(model, Q, body_id, point, false));
  unsigned int reference_body_id = body_id;
  if (body_id > model.fixed_body_discriminator) {
    const unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
  }
  for (unsigned int j = reference_body_id; j != 0; j = model.lambda[j]) {
    G.col(j - 1) = trans.apply(model.X_base[j].Inverse().apply(model.S[j]));
  }
}

bool InverseKinematics(Model &model,
                       const VectorX &Qinit,
                       const std::vector<unsigned int> &body_id,
                       const std::vector<Vector3> &body_point,
                       const std::vector<Vector3> &target_pos,
                       VectorX &Qres,
                       const unsigned int max_iter,
                       const double step_tol,
                       const double gamma_max) {
  assert(Qinit.size() == model.dof_count);
  assert(body_id.size() == body_point.size());
  assert(body_id.size() == target_pos.size());
  MatrixX J(MatrixX::Zero(3 * body_id.size(), model.dof_count));
  VectorX e(VectorX::Zero(3 * body_id.size()));
  VectorX delta(VectorX::Zero(model.dof_count));
  Qres = Qinit;
  for (unsigned int iter = 0; iter < max_iter; iter++) {
    UpdateKinematicsCustom(model, &Qres, nullptr, nullptr);
    delta.setZero();
    for (unsigned int k = 0; k < body_id.size(); k++) {
      MatrixX G(MatrixX::Zero(3, model.dof_count));
      CalcPointJacobian(model, Qres, body_id[k], body_point[k], G, false);
      Vector3 point_pos = model.X_base[body_id[k]].E.transpose() * body_point[k] + model.X_base[body_id[k]].r;
      J.middleRows<3>(3 * k) = G;
      e.segment<3>(3 * k) = target_pos[k] - point_pos;
    }
    if (e.norm() < step_tol) {
      return true;
    }
    Eigen::JacobiSVD<MatrixX> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorX sigma = svd.singularValues();
    MatrixX U = svd.matrixU();
    MatrixX V = svd.matrixV();
    const unsigned int rank = svd.rank();
    for (unsigned int i = 0; i < rank; i++) {
      double N = 0, M = 0;
      for (unsigned int l = 0; l < body_id.size(); l++) {
        N += U.col(i).segment<3>(3 * l).norm();
        M += 1.0 / sigma(i) * J.middleRows<3>(3 * l).colwise().norm().dot(V.col(i).cwiseAbs());
      }
      const double gamma = std::min(N / M, 1.) * gamma_max;
      delta += _ClampMaxAbs(V.col(i) * (U.col(i).dot(e) / sigma(i)), gamma);
    }
    delta = _ClampMaxAbs(delta, gamma_max);
    Qres = Qres + delta;
  }
  return false;
}
IKConstraint::IKConstraint() {
  num_constraints = 0;
  num_iter = 0;
  max_iter = 50;
  step_tol = 1e-12;
  constraint_tol = 1e-12;
  error_norm = std::numeric_limits<double>::max();
  delta_q_norm = std::numeric_limits<double>::max();
}
unsigned int IKConstraint::AddPointConstraint(unsigned int body_id,
                                              const Vector3 &body_point,
                                              const Vector3 &target_pos) {
  constraint_type.push_back(ConstraintTypePosition);
  body_ids.push_back(body_id);
  body_points.push_back(body_point);
  target_positions.push_back(target_pos);
  target_orientations.emplace_back(Matrix3::Identity());
  constraint_row_index.push_back(num_constraints);
  num_constraints += 3;
  return constraint_type.size() - 1;
}
unsigned int IKConstraint::AddOrientationConstraint(unsigned int body_id, const Matrix3 &target_orientation) {
  constraint_type.push_back(ConstraintTypeOrientation);
  body_ids.push_back(body_id);
  body_points.emplace_back(Vector3::Zero());
  target_positions.emplace_back(Vector3::Zero());
  target_orientations.push_back(target_orientation);
  constraint_row_index.push_back(num_constraints);
  num_constraints += 3;
  return constraint_type.size() - 1;
}
unsigned int IKConstraint::AddFullConstraint(unsigned int body_id,
                                             const Vector3 &body_point,
                                             const Vector3 &target_pos,
                                             const Matrix3 &target_orientation) {
  constraint_type.push_back(ConstraintTypeFull);
  body_ids.push_back(body_id);
  body_points.push_back(body_point);
  target_positions.push_back(target_pos);
  target_orientations.push_back(target_orientation);
  constraint_row_index.push_back(num_constraints);
  num_constraints += 6;
  return constraint_type.size() - 1;
}
unsigned int IKConstraint::ClearConstraints() {
  num_constraints = 0;
  constraint_type.clear();
  body_ids.clear();
  body_points.clear();
  target_positions.clear();
  target_orientations.clear();
  constraint_row_index.clear();
  return 0;
}

bool InverseKinematics(Model &model, const VectorX &Qinit, IKConstraint &CS, VectorX &Qres) {
  assert(Qinit.size() == model.dof_count);
  assert(Qres.size() == model.dof_count);
  CS.J.resize(CS.num_constraints, model.dof_count);
  CS.e.resize(CS.num_constraints);
  VectorX delta(model.dof_count);
  Qres = Qinit;
  for (CS.num_iter = 0; CS.num_iter < CS.max_iter; CS.num_iter++) {
    UpdateKinematicsCustom(model, &Qres, nullptr, nullptr);

    for (unsigned int k = 0; k < CS.body_ids.size(); k++) {
      CS.G.setZero(6, model.dof_count);
      CalcPointJacobian6D(model, Qres, CS.body_ids[k], CS.body_points[k], CS.G, false);
      Vector3 point_base = CalcBodyToBaseCoordinates(model, Qres, CS.body_ids[k], CS.body_points[k], true);
      Matrix3 R = CalcBodyToBaseRotation(model, Qres, CS.body_ids[k], false);
      Vector3 e_angle = R.transpose() * rotationLog(R * CS.target_orientations[k].transpose());

      if (CS.constraint_type[k] == IKConstraint::ConstraintTypeFull) {
        CS.J.middleRows<6>(CS.constraint_row_index[k]) = CS.G;
        CS.e.segment<3>(CS.constraint_row_index[k]) = e_angle;
        CS.e.segment<3>(CS.constraint_row_index[k] + 3) = CS.target_positions[k] - point_base;
      } else if (CS.constraint_type[k] == IKConstraint::ConstraintTypeOrientation) {
        CS.J.middleRows<3>(CS.constraint_row_index[k]) = CS.G.topRows(3);
        CS.e.segment<3>(CS.constraint_row_index[k]) = e_angle;
      } else if (CS.constraint_type[k] == IKConstraint::ConstraintTypePosition) {
        CS.J.middleRows<3>(CS.constraint_row_index[k]) = CS.G.bottomRows(3);
        CS.e.segment<3>(CS.constraint_row_index[k]) - CS.target_positions[k] - point_base;
      } else {
        assert(false);
      }
    }

    CS.error_norm = CS.e.norm();
    if (CS.error_norm < CS.step_tol) {
      return true;
    }
    Eigen::JacobiSVD<MatrixX> svd(CS.J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorX sigma = svd.singularValues();
    MatrixX U = svd.matrixU();
    MatrixX V = svd.matrixV();
    delta.setZero();
    const unsigned int rank = svd.rank();
    for (unsigned int i = 0; i < rank; i++) {
      double N = 0, M = 0;
      for (unsigned int k = 0; k < CS.body_ids.size(); k++) {
        if (CS.constraint_type[k] == IKConstraint::ConstraintTypeOrientation ||
            CS.constraint_type[k] == IKConstraint::ConstraintTypePosition) {
          N += U.col(i).segment<3>(CS.constraint_row_index[k]).norm();
          M +=
              1.0 / sigma(i) * CS.J.middleRows<3>(CS.constraint_row_index[k]).colwise().norm().dot(V.col(i).cwiseAbs());
        } else {
          N += U.col(i).segment<6>(CS.constraint_row_index[k]).norm();
          M +=
              1.0 / sigma(i) * CS.J.middleRows<6>(CS.constraint_row_index[k]).colwise().norm().dot(V.col(i).cwiseAbs());
        }
      }
      const double gamma = std::min(N / M, 1.) * CS.gamma_max;
      delta += _ClampMaxAbs(V.col(i) * (U.col(i).dot(CS.e) / sigma(i)), gamma);
    }
    delta = _ClampMaxAbs(delta, CS.gamma_max);
    Qres = Qres + delta;
    CS.delta_q_norm = delta.norm();
    if (CS.delta_q_norm < CS.step_tol) {
      return false;
    }
  }
  return false;
}

}  // namespace contradium::rbd