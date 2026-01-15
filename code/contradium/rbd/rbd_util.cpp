#include "rbd_util.h"

namespace contradium::rbd {

VectorX _ClampMaxAbs(const VectorX& v, const double gamma) {
  if (const double norm = v.lpNorm<Eigen::Infinity>(); norm > gamma) {
    return v * (gamma / norm);
  }
  return v;
}

Vector3 rotationLog(const Matrix3& R) {
    Eigen::AngleAxisd angleAxis(R);
    return angleAxis.axis() * angleAxis.angle();
}

} // namespace contradium::rbd