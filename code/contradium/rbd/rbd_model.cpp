#include "contradium/rbd/rbd_model.h"

namespace contradium::rbd {

Model::Model() {
  const SpatialVector zero_spatial = SpatialVector::Zero();
  const SpatialRigidBodyInertia zero_inertia(0., Vector3(0, 0, 0), Matrix3::Zero());
  const SpatialTransform identity_transform;
  const Joint baseJoint;
  const Body baseBody;
  lambda.push_back(0);
  dof_count = 0;
  previously_added_body_id = 0;
  gravity = Vector3(0, 0, 0);
  v.push_back(zero_spatial);
  a.push_back(zero_spatial);
  mJoints.push_back(baseJoint);
  S.push_back(zero_spatial);
  v_J.push_back(zero_spatial);
  c_J.push_back(zero_spatial);
  X_T.push_back(identity_transform);
  c.push_back(zero_spatial);
  I.push_back(zero_inertia);
  IA.emplace_back(SpatialMatrix::Identity());
  pA.push_back(zero_spatial);
  U.push_back(zero_spatial);
  d = VectorX::Zero(1);
  u = VectorX::Zero(1);
  f.push_back(zero_spatial);
  Ic.push_back(zero_inertia);
  X_lambda.push_back(identity_transform);
  X_base.push_back(identity_transform);
  mBodies.push_back(baseBody);
  // Use max/4 to leave room for fixed body indices without overflow
  fixed_body_discriminator = std::numeric_limits<unsigned int>::max() / 4;
}

unsigned int AddBodyFixedJoint(Model &model,
                               unsigned int parent_id,
                               const SpatialTransform &joint_frame,
                               const Joint &joint,
                               const Body &body) {
  FixedBody fbody = FixedBody::CreateFromBody(body);
  fbody.mMovableParent = parent_id;
  fbody.mParentTransform = joint_frame;
  if (parent_id >= model.fixed_body_discriminator) {
    const FixedBody fixed_parent = model.mFixedBodies[parent_id - model.fixed_body_discriminator];
    fbody.mMovableParent = fixed_parent.mMovableParent;
    fbody.mParentTransform = joint_frame * fixed_parent.mParentTransform;
  }
  Body parent_body = model.mBodies[fbody.mMovableParent];
  parent_body.join(fbody.mParentTransform, body);
  model.mBodies[fbody.mMovableParent] = parent_body;
  model.I[fbody.mMovableParent] = SpatialRigidBodyInertia::CreateFromMassComInertiaC(
      parent_body.mMass, parent_body.mCenterOfMass, parent_body.mInertia);
  model.mFixedBodies.push_back(fbody);

  return model.mFixedBodies.size() + model.fixed_body_discriminator - 1;
}

unsigned int Model::AddBody(unsigned int parent_id,
                            const SpatialTransform &joint_frame,
                            const Joint &joint,
                            const Body &body) {
  assert (joint.mJointType != JointTypeUndefined);
  if (joint.mJointType == JointTypeFixed) {
    previously_added_body_id = AddBodyFixedJoint(*this, parent_id, joint_frame, joint, body);
    return previously_added_body_id;
  }
  assert(joint.mJointType == JointTypePrismatic || joint.mJointType == JointTypeRevolute);
  unsigned int movable_parent_id = parent_id;
  SpatialTransform movable_parent_transform;
  if (parent_id >= fixed_body_discriminator) {
    const unsigned int fbody_id = parent_id - fixed_body_discriminator;
    movable_parent_id = mFixedBodies[fbody_id].mMovableParent;
    movable_parent_transform = mFixedBodies[fbody_id].mParentTransform;
  }

  const SpatialVector zero_spatial = SpatialVector::Zero();
  const SpatialTransform identity_transform;
  lambda.push_back(movable_parent_id);
  dof_count++;
  previously_added_body_id = dof_count;
  v.push_back(zero_spatial);
  a.push_back(zero_spatial);
  mJoints.push_back(joint);
  mBodies.push_back(body);
  S.push_back(joint.mJointAxis);
  v_J.push_back(zero_spatial);
  c_J.push_back(zero_spatial);
  X_T.push_back(joint_frame * movable_parent_transform);
  c.push_back(zero_spatial);
  const SpatialRigidBodyInertia rbi =
      SpatialRigidBodyInertia::CreateFromMassComInertiaC(body.mMass, body.mCenterOfMass, body.mInertia);
  I.push_back(rbi);
  IA.emplace_back(SpatialMatrix::Zero());
  pA.push_back(zero_spatial);
  U.push_back(zero_spatial);
  d = VectorX::Zero(mBodies.size());
  u = VectorX::Zero(mBodies.size());
  f.push_back(zero_spatial);
  Ic.push_back(rbi);
  X_lambda.push_back(identity_transform);
  X_base.push_back(identity_transform);
  return previously_added_body_id;
}

unsigned int Model::AppendBody(const SpatialTransform &joint_frame, const Joint &joint, const Body &body) {
  return AddBody(previously_added_body_id, joint_frame, joint, body);
}

}  // namespace contradium::rbd