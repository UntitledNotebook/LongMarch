#ifndef LONGMARCH_RBD_DYNAMICS_H
#define LONGMARCH_RBD_DYNAMICS_H

#include "contradium/rbd/rbd_util.h"

namespace contradium::rbd {
struct Model;

LM_DEVICE_FUNC void InverseDynamics(Model &model,
                                    const VectorX &Q,
                                    const VectorX &QDot,
                                    const VectorX &QDDot,
                                    VectorX &Tau,
                                    const std::vector<SpatialVector> *f_ext = nullptr);

LM_DEVICE_FUNC void CompositeRigidBodyAlgorithm(Model &model,
                                                const VectorX &Q,
                                                MatrixX &H,
                                                bool update_kinematics = true);

LM_DEVICE_FUNC void ForwardDynamicsLagragian(Model &model,
                                             const VectorX &Q,
                                             const VectorX &QDot,
                                             const VectorX &Tau,
                                             VectorX &QDDot,
                                             const std::vector<SpatialVector> *f_ext = nullptr);

LM_DEVICE_FUNC void ForwardDynamics(Model &model,
                                    const VectorX &Q,
                                    const VectorX &QDot,
                                    const VectorX &Tau,
                                    VectorX &QDDot,
                                    const std::vector<SpatialVector> *f_ext = nullptr);

}  // namespace contradium::rbd

#endif  // LONGMARCH_RBD_DYNAMICS_H
