#pragma once

#include "grassland/math/math_util.h"

namespace grassland {

// =============================================================================
// Spatial Algebra Type Aliases
// =============================================================================
/// 6D spatial vector (angular components in top 3, linear in bottom 3)
/// Follows Featherstone's convention: [ω; v] for motion, [n; f] for force
template <typename Scalar>
using SpatialVector = Matrix<Scalar, 6, 1>;

/// 6x6 spatial matrix
template <typename Scalar>
using SpatialMatrix = Matrix<Scalar, 6, 6>;

// Convenience type aliases for common scalar types
using SpatialVectord = SpatialVector<double>;
using SpatialVectorf = SpatialVector<float>;
using SpatialMatrixd = SpatialMatrix<double>;
using SpatialMatrixf = SpatialMatrix<float>;

// =============================================================================
// Skew-Symmetric Matrix (3x3)
// =============================================================================

template <typename Scalar>
LM_DEVICE_FUNC Matrix3<Scalar> SkewMatrix(const Vector3<Scalar> &v) {
  Matrix3<Scalar> m;
  m << Scalar(0), -v(2), v(1),
       v(2), Scalar(0), -v(0),
       -v(1), v(0), Scalar(0);
  return m;
}


  // Ixx, Iyx, Iyy, Izx, Izy, Izz are inertial at origin
template <typename Scalar>
struct SpatialRigidBodyInertia {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Scalar m;
  Vector3<Scalar> h;
  Scalar Ixx, Iyx, Iyy, Izx, Izy, Izz;

  LM_DEVICE_FUNC SpatialRigidBodyInertia()
      : m(Scalar(0)),
        h(Vector3<Scalar>::Zero()),
        Ixx(Scalar(0)), Iyx(Scalar(0)), Iyy(Scalar(0)),
        Izx(Scalar(0)), Izy(Scalar(0)), Izz(Scalar(0)) {}

  LM_DEVICE_FUNC SpatialRigidBodyInertia(
      Scalar mass,
      const Vector3<Scalar> &first_moment,
      Scalar ixx, Scalar iyx, Scalar iyy,
      Scalar izx, Scalar izy, Scalar izz)
      : m(mass), h(first_moment),
        Ixx(ixx), Iyx(iyx), Iyy(iyy),
        Izx(izx), Izy(izy), Izz(izz) {}

  LM_DEVICE_FUNC SpatialRigidBodyInertia(
      Scalar mass,
      const Vector3<Scalar> &first_moment,
      const Matrix3<Scalar> &inertia)
      : m(mass), h(first_moment),
        Ixx(inertia(0, 0)),
        Iyx(inertia(1, 0)), Iyy(inertia(1, 1)),
        Izx(inertia(2, 0)), Izy(inertia(2, 1)), Izz(inertia(2, 2)) {}

  LM_DEVICE_FUNC static SpatialRigidBodyInertia CreateFromMassComInertiaC(
      Scalar mass,
      const Vector3<Scalar> &com,
      const Matrix3<Scalar> &inertia_c) {
    /// [I_C - m cx cx, m cx,
    ///  -m cx        , m I_3]
    SpatialRigidBodyInertia result;
    result.m = mass;
    result.h = mass * com;

    Matrix3<Scalar> I = inertia_c + SkewMatrix(com) * SkewMatrix(com).transpose() * mass;
    result.Ixx = I(0, 0);
    result.Iyx = I(1, 0); result.Iyy = I(1, 1);
    result.Izx = I(2, 0); result.Izy = I(2, 1); result.Izz = I(2, 2);

    return result;
  }

  LM_DEVICE_FUNC SpatialVector<Scalar> operator*(
      const SpatialVector<Scalar> &v) const {
    /// fv(Iw + hx w, mv - hx w)
    Vector3<Scalar> omega(v(0), v(1), v(2));
    Vector3<Scalar> vel(v(3), v(4), v(5));

    Vector3<Scalar> res_upper(
        Ixx * v(0) + Iyx * v(1) + Izx * v(2),
        Iyx * v(0) + Iyy * v(1) + Izy * v(2),
        Izx * v(0) + Izy * v(1) + Izz * v(2));
    res_upper += h.cross(vel);

    // f = m * v - h × ω
    Vector3<Scalar> res_lower = m * vel - h.cross(omega);

    SpatialVector<Scalar> result;
    result << res_upper(0), res_upper(1), res_upper(2),
              res_lower(0), res_lower(1), res_lower(2);
    return result;
  }

  /// Add two spatial inertias
  LM_DEVICE_FUNC SpatialRigidBodyInertia operator+(
      const SpatialRigidBodyInertia &other) const {
    return SpatialRigidBodyInertia(
        m + other.m,
        h + other.h,
        Ixx + other.Ixx,
        Iyx + other.Iyx, Iyy + other.Iyy,
        Izx + other.Izx, Izy + other.Izy, Izz + other.Izz);
  }

  /// Add another spatial inertia to this one (in-place)
  LM_DEVICE_FUNC SpatialRigidBodyInertia& operator+=(
      const SpatialRigidBodyInertia &other) {
    m += other.m;
    h += other.h;
    Ixx += other.Ixx;
    Iyx += other.Iyx; Iyy += other.Iyy;
    Izx += other.Izx; Izy += other.Izy; Izz += other.Izz;
    return *this;
  }

  LM_DEVICE_FUNC SpatialMatrix<Scalar> ToMatrix() const {
    /// [Ixx, hx;
    ///  -hx, m * I_3]
    SpatialMatrix<Scalar> result;

    result(0, 0) = Ixx; result(0, 1) = Iyx; result(0, 2) = Izx;
    result(1, 0) = Iyx; result(1, 1) = Iyy; result(1, 2) = Izy;
    result(2, 0) = Izx; result(2, 1) = Izy; result(2, 2) = Izz;

    result.template block<3, 3>(0, 3) = SkewMatrix(h);
    result.template block<3, 3>(3, 0) = - SkewMatrix(h);
    result.template block<3, 3>(3, 3) = Matrix3<Scalar>::Identity() * m;

    return result;
  }

  LM_DEVICE_FUNC void setSpatialMatrix(SpatialMatrix<Scalar> &mat) const {
    mat(0, 0) = Ixx; mat(0, 1) = Iyx; mat(0, 2) = Izx;
    mat(1, 0) = Iyx; mat(1, 1) = Iyy; mat(1, 2) = Izy;
    mat(2, 0) = Izx; mat(2, 1) = Izy; mat(2, 2) = Izz;

    mat(3, 0) = Scalar(0); mat(3, 1) =      h(2); mat(3,2) =     -h(1);
    mat(4, 0) =     -h(2); mat(4, 1) = Scalar(0); mat(4,2) =      h(0);
    mat(5, 0) =      h(1); mat(5, 1) =     -h(0); mat(5,2) = Scalar(0);

    mat(0, 3) = Scalar(0); mat(0, 4) =     -h(2); mat(0,5) =      h(1);
    mat(1, 3) =      h(2); mat(1, 4) = Scalar(0); mat(1,5) =     -h(0);
    mat(2, 3) =     -h(1); mat(2, 4) =      h(0); mat(2,5) = Scalar(0);

    mat(3,3) =         m; mat(3,4) = Scalar(0); mat(3,5) = Scalar(0);
    mat(4,3) = Scalar(0); mat(4,4) =         m; mat(4,5) = Scalar(0);
    mat(5,3) = Scalar(0); mat(5,4) = Scalar(0); mat(5,5) =         m;
  }

  LM_DEVICE_FUNC void createFromMatrix(const SpatialMatrix<Scalar> &Ic) {
    m = Ic(3, 3);
    h(0) = -Ic(1, 5); h(1) = Ic(0, 5); h(2) = -Ic(0, 4);
    Ixx = Ic(0, 0);
    Iyx = Ic(1, 0); Iyy = Ic(1, 1);
    Izx = Ic(2, 0); Izy = Ic(2, 1); Izz = Ic(2, 2);
  }

  };

// Convenience type aliases
using SpatialRigidBodyInertiad = SpatialRigidBodyInertia<double>;
using SpatialRigidBodyInertiaf = SpatialRigidBodyInertia<float>;


/// The transform X from frame A to frame B is defined as:
///   E = rotation matrix (B_R_A: rotation from A to B)
///   r = position of origin of B in A coordinates
///
template <typename Scalar>
struct SpatialTransform {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Matrix3<Scalar> E;
  Vector3<Scalar> r;

  LM_DEVICE_FUNC SpatialTransform()
      : E(Matrix3<Scalar>::Identity()),
        r(Vector3<Scalar>::Zero()) {}

  LM_DEVICE_FUNC SpatialTransform(const Matrix3<Scalar> &rotation,
                                  const Vector3<Scalar> &translation)
      : E(rotation), r(translation) {}

  LM_DEVICE_FUNC explicit SpatialTransform(
      const Matrix3<Scalar> &rotation)
      : E(rotation), r(Vector3<Scalar>::Zero()) {}

  /// X v
  LM_DEVICE_FUNC SpatialVector<Scalar> apply(
      const SpatialVector<Scalar> &v) const {
    /// mv(E * w, E * (v - rx w))
    Vector3<Scalar> omega = v.template head<3>();
    Vector3<Scalar> vel = v.template tail<3>();

    SpatialVector<Scalar> result;
    result.template head<3>() = E * omega;
    result.template tail<3>() = E * (vel - r.cross(omega));
    return result;
  }

  /// X^* I X^{-1}
  LM_DEVICE_FUNC SpatialRigidBodyInertia<Scalar> apply(
    const SpatialRigidBodyInertia<Scalar> &rbi) const {
    /// rbi(m, E * (h - m r),
    ///     lt(E * (I + rx hx + (h - mr)x rx) * E^T))
    Matrix<Scalar, 3, 1> _h_mr = rbi.h - rbi.m * r;
    Matrix<Scalar, 3, 3> I_orig;
    I_orig << rbi.Ixx, rbi.Iyx, rbi.Izx,
        rbi.Iyx, rbi.Iyy, rbi.Izy,
        rbi.Izx, rbi.Izy, rbi.Izz;
    return SpatialRigidBodyInertia<Scalar>(
      rbi.m,
      E * _h_mr,
      E *
      (
        I_orig
        + SkewMatrix(r) * SkewMatrix(rbi.h)
        + SkewMatrix(_h_mr) * SkewMatrix(r)
      )
      * E.transpose()
    );
  }

  /// X^T f
  LM_DEVICE_FUNC SpatialVector<Scalar> applyTranspose(
      const SpatialVector<Scalar> &f) const {
    /// fv(E^T * n + rx E^T * f, E^T * f)
    Vector3<Scalar> n = f.template head<3>();
    Vector3<Scalar> force = f.template tail<3>();

    Vector3<Scalar> Et_force = E.transpose() * force;
    SpatialVector<Scalar> result;
    result.template head<3>() = E.transpose() * n + r.cross(Et_force);
    result.template tail<3>() = Et_force;
    return result;
  }

  /// X^T I X
  LM_DEVICE_FUNC SpatialRigidBodyInertia<Scalar> applyTranspose(
    const SpatialRigidBodyInertia<Scalar> &rbi) const {
    /// rbi(m, E^T * h + mr,
    ///     lt(E^T * I * E - rx (E^T * h)x - (E^T * h + mr)x rx))
    Matrix3<Scalar> I_orig;
    I_orig << rbi.Ixx, rbi.Iyx, rbi.Izx,
        rbi.Iyx, rbi.Iyy, rbi.Izy,
        rbi.Izx, rbi.Izy, rbi.Izz;
    Vector3<Scalar> ETh_mr = E.transpose() * rbi.h + rbi.m * r;
    return SpatialRigidBodyInertia<Scalar>(
      rbi.m,
      ETh_mr,
      E.transpose() * I_orig * E
      - SkewMatrix<Scalar>(r) * SkewMatrix<Scalar>(E.transpose() * rbi.h)
      - SkewMatrix<Scalar>(ETh_mr) * SkewMatrix<Scalar>(r)
    );
  }

  /// X^* f
  LM_DEVICE_FUNC SpatialVector<Scalar> applyAdjoint(const SpatialVector<Scalar> &f) {
    /// fv(E * (n - rx f), E * f)
    Vector3<Scalar> n_ = f.template head<3>();
    Vector3<Scalar> f_ = f.template tail<3>();
    SpatialVector<Scalar> result;
    result.template head<3>() = E * (n_ - r.cross(f_));
    result.template tail<3>() = E * f_;
    return result;
  }

  /// X^{-1}
  LM_DEVICE_FUNC SpatialTransform Inverse() const {
    /// plx(E^T, -E * r)
    return SpatialTransform(E.transpose(), -E * r);
  }

  /// X_1 * X_2
  LM_DEVICE_FUNC SpatialTransform operator*(
      const SpatialTransform &other) const {
    /// plx(E_1 * E_2, r_2 + E_2^T * r_1)
    return SpatialTransform(E * other.E, other.r + other.E.transpose() * r);
  }

  /// TODO: expand
  LM_DEVICE_FUNC SpatialMatrix<Scalar> ToMatrix() const {
    /// [E      , 0;
    ///  -E * rx, E]
    SpatialMatrix<Scalar> X;
    Matrix3<Scalar> Erx = E * SkewMatrix(r);

    X.template block<3, 3>(0, 0) = E;
    X.template block<3, 3>(0, 3).setZero();
    X.template block<3, 3>(3, 0) = -Erx;
    X.template block<3, 3>(3, 3) = E;
    return X;
  }

  /// TODO: expand
  LM_DEVICE_FUNC SpatialMatrix<Scalar> ToMatrixTranspose() const {
    SpatialMatrix<Scalar> Xt;
    Matrix3<Scalar> Erx = E * SkewMatrix(r);

    Xt.template block<3, 3>(0, 0) = E.transpose();
    Xt.template block<3, 3>(0, 3) = -Erx.transpose();
    Xt.template block<3, 3>(3, 0).setZero();
    Xt.template block<3, 3>(3, 3) = E.transpose();
    return Xt;
  }
};


// Convenience type aliases
using SpatialTransformd = SpatialTransform<double>;
using SpatialTransformf = SpatialTransform<float>;

template<typename Scalar>
 std::ostream &operator<<(std::ostream &output, const SpatialRigidBodyInertia<Scalar> &rbi) {
  output << "rbi.m = " << rbi.m << std::endl;
  output << "rbi.h = " << rbi.h.transpose();
  output << "rbi.Ixx = " << rbi.Ixx << std::endl;
  output << "rbi.Iyx = " << rbi.Iyx << " rbi.Iyy = " << rbi.Iyy << std::endl;
  output << "rbi.Izx = " << rbi.Izx << " rbi.Izy = " << rbi.Izy << " rbi.Izz = " << rbi.Izz << std::endl;
  return output;
}

template<typename Scalar>
 std::ostream &operator<<(std::ostream &output, const SpatialTransform<Scalar> &X) {
  output << "X.E = " << std::endl << X.E << std::endl;
  output << "X.r = " << X.r.transpose();
  return output;
}

// =============================================================================
// Spatial Cross Products
// =============================================================================

/// crossm(v) = [ω×,  0 ]
///             [vO×, ω×]
template <typename Scalar>
LM_DEVICE_FUNC SpatialMatrix<Scalar> crossm(
  const SpatialVector<Scalar> &v) {
  SpatialMatrix<Scalar> result;
  result <<
      0, -v(2), v(1), 0, 0, 0,
      v(2), 0, -v(0), 0, 0, 0,
      -v(1), v(0), 0, 0, 0, 0,
      0, -v(5), v(4), 0, -v(2), v(1),
      v(5), 0, -v(3), v(2), 0, -v(0),
      -v(4), v(3), 0, -v(1), v(0), 0;

  return result;
}

template <typename Scalar>
LM_DEVICE_FUNC SpatialVector<Scalar> crossm(
    const SpatialVector<Scalar> &v1,
    const SpatialVector<Scalar> &v2) {
  SpatialVector<Scalar> result;
  result <<
      -v1(2) * v2(1) + v1(1) * v2(2),
      v1(2) * v2(0) - v1(0) * v2(2),
      -v1(1) * v2(0) + v1(0) * v2(1),
      -v1(5) * v2(1) + v1(4) * v2(2) - v1(2) * v2(4) + v1(1) * v2(5),
      v1(5) * v2(0) - v1(3) * v2(2) + v1(2) * v2(3) - v1(0) * v2(5),
      -v1(4) * v2(0) + v1(3) * v2(1) - v1(1) * v2(3) + v1(0) * v2(4);
  return result;
}

/// crossf(v) = [ω×,  v0×]
///             [0,   ω× ]
template <typename Scalar>
LM_DEVICE_FUNC SpatialMatrix<Scalar> crossf(
  const SpatialVector<Scalar> &v) {
  SpatialMatrix<Scalar> result;
  result <<
      0, -v(2), v(1), 0, -v(5), v(4),
      v(2), 0, -v(0), v(5), 0, -v(3),
      -v(1), v(0), 0, -v(4), v(3), 0,
      0, 0, 0, 0, -v(2), v(1),
      0, 0, 0, v(2), 0, -v(0),
      0, 0, 0, -v(1), v(0), 0;
  return result;
}

template <typename Scalar>
LM_DEVICE_FUNC SpatialVector<Scalar> crossf(
    const SpatialVector<Scalar> &v1,
    const SpatialVector<Scalar> &v2) {
  SpatialVector<Scalar> result;
  result << -v1(2) * v2(1) + v1(1) * v2(2) - v1(5) * v2(4) + v1(4) * v2(5),
            v1(2) * v2(0) - v1(0) * v2(2) + v1(5) * v2(3) - v1(3) * v2(5),
            -v1(1) * v2(0) + v1(0) * v2(1) - v1(4) * v2(3) + v1(3) * v2(4),
            -v1(2) * v2(4) + v1(1) * v2(5),
            v1(2) * v2(3) - v1(0) * v2(5),
            -v1(1) * v2(3) + v1(0) * v2(4);
  return result;
}

// =============================================================================
// Spatial Transform Factory Functions
// =============================================================================

template <typename Scalar>
LM_DEVICE_FUNC SpatialTransform<Scalar> Xrot(
    Scalar angle,
    const Vector3<Scalar> &axis) {
  // Rodrigues' rotation formula - matching RBDL convention
  // RBDL uses column-major construction, so we need to transpose our row-major formula
  Scalar c = cos(angle);
  Scalar s = sin(angle);
  Scalar t = Scalar(1) - c;

  Vector3<Scalar> n = axis.normalized();
  Scalar x = n(0), y = n(1), z = n(2);

  // RBDL's convention (transposed from standard Rodrigues)
  Matrix3<Scalar> E;
  E << t * x * x + c, t * x * y + s * z, t * x * z - s * y,
       t * x * y - s * z, t * y * y + c, t * y * z + s * x,
       t * x * z + s * y, t * y * z - s * x, t * z * z + c;

  return SpatialTransform<Scalar>(E, Vector3<Scalar>::Zero());
}

template <typename Scalar>
LM_DEVICE_FUNC SpatialTransform<Scalar> Xrotx(Scalar angle) {
  Scalar c = cos(angle);
  Scalar s = sin(angle);

  Matrix3<Scalar> E;
  E << Scalar(1), Scalar(0), Scalar(0),
       Scalar(0), c, s,
       Scalar(0), -s, c;

  return SpatialTransform<Scalar>(E, Vector3<Scalar>::Zero());
}

template <typename Scalar>
LM_DEVICE_FUNC SpatialTransform<Scalar> Xroty(Scalar angle) {
  Scalar c = cos(angle);
  Scalar s = sin(angle);

  Matrix3<Scalar> E;
  E << c, Scalar(0), -s, Scalar(0), Scalar(1), Scalar(0), s, Scalar(0), c;

  return SpatialTransform<Scalar>(E, Vector3<Scalar>::Zero());
}

template <typename Scalar>
LM_DEVICE_FUNC SpatialTransform<Scalar> Xrotz(Scalar angle) {
  Scalar c = cos(angle);
  Scalar s = sin(angle);

  Matrix3<Scalar> E;
  E << c, s, Scalar(0),
       -s, c, Scalar(0),
       Scalar(0), Scalar(0), Scalar(1);

  return SpatialTransform<Scalar>(E, Vector3<Scalar>::Zero());
}

template <typename Scalar>
LM_DEVICE_FUNC SpatialTransform<Scalar> Xtrans(
    const Vector3<Scalar> &r) {
  return SpatialTransform<Scalar>(Matrix3<Scalar>::Identity(), r);
}


}  // namespace grassland
