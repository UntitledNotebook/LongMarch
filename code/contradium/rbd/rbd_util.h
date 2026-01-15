#pragma once
#include "contradium/core/core.h"


namespace contradium::rbd {

// Type aliases for convenience
using Vector3 = Vector3<double>;
using Matrix3 = Matrix3<double>;
using SpatialVector = SpatialVector<double>;
using SpatialMatrix = SpatialMatrix<double>;
using SpatialTransform = SpatialTransform<double>;
using SpatialRigidBodyInertia = SpatialRigidBodyInertia<double>;
using VectorX = VectorX<double>;
using MatrixX = MatrixX<double>;

VectorX _ClampMaxAbs(const VectorX& v, double gamma);


/**
 * Computes the rotation vector (log map) from a 3x3 rotation matrix.
 * @param R A 3x3 rotation matrix (must be orthogonal with det=1)
 * @return Vector3d representing the rotation (axis * angle)
 */
Vector3 rotationLog(const Matrix3& R);

} // namespace contradium::rbd

