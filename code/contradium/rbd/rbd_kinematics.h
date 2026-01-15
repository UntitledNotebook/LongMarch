#ifndef LONGMARCH_RBD_KINEMATICS_H
#define LONGMARCH_RBD_KINEMATICS_H

#include "contradium/rbd/rbd_util.h"

namespace contradium::rbd {

struct Model;

LM_DEVICE_FUNC void UpdateKinematics(Model &model, const VectorX &Q, const VectorX &QDot, const VectorX &QDDot);
LM_DEVICE_FUNC void UpdateKinematicsCustom(Model &model, const VectorX *Q, const VectorX *QDot, const VectorX *QDDot);

// Only set the corresponding cols, need to initialize to 0 before calling
LM_DEVICE_FUNC void CalcPointJacobian(Model &model,
                                      const VectorX &Q,
                                      unsigned int body_id,
                                      const Vector3 &point,
                                      MatrixX &G,
                                      bool update_kinematics = true);

LM_DEVICE_FUNC void CalcPointJacobian6D(Model &model,
                                        const VectorX &Q,
                                        unsigned int body_id,
                                        const Vector3 &point,
                                        MatrixX &G,
                                        bool update_kinematics = true);

LM_DEVICE_FUNC Vector3 CalcBodyToBaseCoordinates(Model &model,
                                  const VectorX &Q,
                                  unsigned int body_id,
                                  const Vector3 &position,
                                  bool update_kinematics = true);

LM_DEVICE_FUNC Matrix3 CalcBodyToBaseRotation(Model &model, const VectorX &Q, unsigned int body_id, bool update_kinematics = true);

LM_DEVICE_FUNC bool InverseKinematics(Model &model,
                                      const VectorX &Qinit,
                                      const std::vector<unsigned int> &body_id,
                                      const std::vector<Vector3> &body_point,
                                      const std::vector<Vector3> &target_pos,
                                      VectorX &Qres,
                                      unsigned int max_iter = 42,
                                      double step_tol = 1e-12,
                                      double gamma_max = PI<double>() / 4);

struct IKConstraint {
  enum ConstraintType { ConstraintTypePosition = 0, ConstraintTypeOrientation, ConstraintTypeFull };

  MatrixX J;
  MatrixX G;
  VectorX e;

  unsigned int num_constraints;
  unsigned int num_iter;
  unsigned int max_iter;
  double step_tol;
  double constraint_tol;
  double error_norm;
  double delta_q_norm;
  const double gamma_max = PI<double>() / 4;

  std::vector<ConstraintType> constraint_type;
  std::vector<unsigned int> body_ids;
  std::vector<Vector3> body_points;
  std::vector<Vector3> target_positions;
  std::vector<Matrix3> target_orientations;
  std::vector<unsigned int> constraint_row_index;

  IKConstraint();
  unsigned int AddPointConstraint(unsigned int body_id, const Vector3 &body_point, const Vector3 &target_pos);
  unsigned int AddOrientationConstraint(unsigned int body_id, const Matrix3 &target_orientation);
  unsigned int AddFullConstraint(unsigned int body_id,
                                 const Vector3 &body_point,
                                 const Vector3 &target_pos,
                                 const Matrix3 &target_orientation);
  unsigned int ClearConstraints();
};

LM_DEVICE_FUNC bool InverseKinematics(Model &model, const VectorX &Qinit, IKConstraint &CS, VectorX &Qres);

}  // namespace contradium::rbd

#endif  // LONGMARCH_RBD_KINEMATICS_H
