#pragma once

#include "contradium/rbd/rbd_util.h"

namespace contradium::rbd {

/// Only 1-DOF supported
enum JointType {
  JointTypeUndefined = 0,
  JointTypeRevolute,
  JointTypePrismatic,
  JointTypeFixed,
};


/** \brief Describes a joint relative to the predecessor body.
 *
 * This class contains all information required for one single joint. This
 * contains the joint type and the axis of the joint.
 *
 * Simplified version supporting only 1-DoF joints (revolute, prismatic, fixed).
 */
struct Joint {
  JointType mJointType;
  SpatialVector mJointAxis;

  Joint() : mJointType(JointTypeUndefined), mJointAxis(SpatialVector::Zero()) {
  }

  explicit Joint(const JointType joint_type) : mJointType(joint_type), mJointAxis(SpatialVector::Zero()) {
  }

  /** \brief Constructs a joint from the given cartesian parameters.
   *
   * \param joint_type whether the joint is revolute or prismatic
   * \param joint_axis the axis of rotation or translation
   */
  Joint(const JointType joint_type, const Vector3 &joint_axis) : mJointType(joint_type) {
    assert(joint_type == JointTypeRevolute || joint_type == JointTypePrismatic);
    Vector3 axis_normalized = joint_axis.normalized();

    if (joint_type == JointTypeRevolute) {
      mJointAxis << axis_normalized[0], axis_normalized[1], axis_normalized[2], 0., 0., 0.;
    } else {
      mJointAxis << 0., 0., 0., axis_normalized[0], axis_normalized[1], axis_normalized[2];
    }
  }
};

struct Model;

void jcalc_X_lambda_S(Model &model, unsigned int joint_id, const VectorX &q);

/** \brief Joint calculation function
 *   - X_lambda[i]: Transform from parent body to current body
 *   - S[i]: Motion subspace (joint axis)
 *   - v_J[i]: Joint velocity = S * qdot
 *   - c_J[i]: Joint bias acceleration (zero for 1-DoF joints)
 */
void jcalc(Model &model, unsigned int joint_id, const VectorX &q, const VectorX &qdot);

}  // namespace contradium::rbd
