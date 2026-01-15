#pragma once

#include "rbd_util.h"

namespace contradium::rbd {

/// Inertia at COM
struct Body {
  Body() : mMass(0.), mCenterOfMass(0., 0., 0.), mInertia(Matrix3::Zero()) {
  }

  Body(const Body &body) : mMass(body.mMass), mCenterOfMass(body.mCenterOfMass), mInertia(body.mInertia) {
  }

  Body &operator=(const Body &body) {
    if (this != &body) {
      mMass = body.mMass;
      mInertia = body.mInertia;
      mCenterOfMass = body.mCenterOfMass;
    }
    return *this;
  }

  Body(const double mass, Vector3 com, const Vector3 &gyration_radii) : mMass(mass), mCenterOfMass(std::move(com)) {
    mInertia = Matrix3::Zero();
    mInertia(0, 0) = gyration_radii[0];
    mInertia(1, 1) = gyration_radii[1];
    mInertia(2, 2) = gyration_radii[2];
  }

  Body(const double mass, Vector3 com, Matrix3 inertia_C)
      : mMass(mass), mCenterOfMass(std::move(com)), mInertia(std::move(inertia_C)) {
  }

  /// transform: from this to other
  void join(const SpatialTransform &transform, const Body &otherBody) {
    if (otherBody.mMass == 0. && otherBody.mInertia == Matrix3::Zero()) {
      return;
    }
    const Matrix3 otherInertia_COM = transform.E.transpose() * otherBody.mInertia * transform.E;
    const Vector3 otherCOM = transform.E.transpose() * otherBody.mCenterOfMass + transform.r;
    const double newMass = mMass + otherBody.mMass;
    if (newMass == 0.) {
      LogError("Zero mass");
      assert(false);
    }
    const Vector3 newCOM = (1 / newMass) * (mMass * mCenterOfMass + otherBody.mMass * otherCOM);
    const Vector3 relative = otherCOM - mCenterOfMass;
    const Matrix3 newInertia = mInertia + otherInertia_COM -
                               (1 / newMass) * mMass * otherBody.mMass * SkewMatrix(relative) * SkewMatrix(relative);
    *this = Body(newMass, newCOM, newInertia);
  }

  ~Body() = default;

  double mMass;
  Vector3 mCenterOfMass;
  /// Inertia matrix at the center of mass
  Matrix3 mInertia;
};

struct FixedBody {
  double mMass{};
  Vector3 mCenterOfMass;
  Matrix3 mInertia;
  unsigned int mMovableParent{};
  SpatialTransform mParentTransform;

  static FixedBody CreateFromBody(const Body &body) {
    FixedBody fbody;

    fbody.mMass = body.mMass;
    fbody.mCenterOfMass = body.mCenterOfMass;
    fbody.mInertia = body.mInertia;

    return fbody;
  }

  Body ToBody() {
    return Body(mMass, mCenterOfMass, mInertia);
  }
};

}  // namespace contradium::rbd
