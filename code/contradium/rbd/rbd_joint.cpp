#include "contradium/rbd/rbd_joint.h"

#include <cassert>

#include "contradium/rbd/rbd_model.h"

namespace contradium::rbd {

void jcalc_X_lambda_S(Model &model, unsigned int joint_id, const VectorX &q) {
  assert(joint_id > 0 && q.size() == model.dof_count);
  const Joint &joint = model.mJoints[joint_id];
  if (joint.mJointType == JointTypePrismatic) {
    model.S[joint_id] = joint.mJointAxis;
    model.X_lambda[joint_id] =
        Xtrans(Vector3(joint.mJointAxis[3] * q[joint_id - 1], joint.mJointAxis[4] * q[joint_id - 1],
                       joint.mJointAxis[5] * q[joint_id - 1])) *
        model.X_T[joint_id];
  } else if (joint.mJointType == JointTypeRevolute) {
    model.S[joint_id] = joint.mJointAxis;
    model.X_lambda[joint_id] =
        Xrot(q[joint_id - 1], Vector3(joint.mJointAxis[0], joint.mJointAxis[1], joint.mJointAxis[2])) *
        model.X_T[joint_id];
  } else {
    LogError(
        "jcalc_X_lambda_S: Joint ID={} is not supported. Only "
        "Revolute and Prismatic joints are currently implemented.",
        joint_id);
  }
}

void jcalc(Model &model, unsigned int joint_id, const VectorX &q, const VectorX &qdot) {
  assert(joint_id > 0);
  const Joint &joint = model.mJoints[joint_id];
  if (joint.mJointType == JointTypePrismatic) {
    model.c_J[joint_id] = SpatialVector::Zero();
    model.S[joint_id] = joint.mJointAxis;
    model.X_lambda[joint_id] =
        Xtrans(Vector3(joint.mJointAxis[3] * q[joint_id - 1], joint.mJointAxis[4] * q[joint_id - 1],
                       joint.mJointAxis[5] * q[joint_id - 1])) *
        model.X_T[joint_id];
    model.v_J[joint_id] = model.S[joint_id] * qdot[joint_id - 1];
    return;
  } else if (joint.mJointType == JointTypeRevolute) {
    model.c_J[joint_id] = SpatialVector::Zero();
    model.S[joint_id] = joint.mJointAxis;
    model.X_lambda[joint_id] =
        Xrot(q[joint_id - 1], Vector3(joint.mJointAxis[0], joint.mJointAxis[1], joint.mJointAxis[2])) *
        model.X_T[joint_id];
    model.v_J[joint_id] = model.S[joint_id] * qdot[joint_id - 1];
    return;
  }

  LogError(
      "jcalc_X_lambda_S: Joint ID={} is not supported. Only "
      "Revolute and Prismatic joints are currently implemented.",
      joint_id);
}

}  // namespace contradium::rbd

