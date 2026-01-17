//
// Created by f3f3xo on 12/25/25.
//

#include "urdf.h"

namespace urdf {

namespace {

contradium::rbd::Matrix3 rpyToRotationMatrix(const Eigen::Vector3d &rpy) {
  const double roll = rpy(0);
  const double pitch = rpy(1);
  const double yaw = rpy(2);

  const double su = std::sin(roll);
  const double cu = std::cos(roll);
  const double sv = std::sin(pitch);
  const double cv = std::cos(pitch);
  const double sw = std::sin(yaw);
  const double cw = std::cos(yaw);

  contradium::rbd::Matrix3 R;
  R(0, 0) = cw * cv;
  R(0, 1) = cw * sv * su - sw * cu;
  R(0, 2) = cw * sv * cu + sw * su;

  R(1, 0) = sw * cv;
  R(1, 1) = sw * sv * su + cw * cu;
  R(1, 2) = sw * sv * cu - cw * su;

  R(2, 0) = -sv;
  R(2, 1) = cv * su;
  R(2, 2) = cv * cu;

  return R;
}

contradium::rbd::Body toRBDBody(const std::shared_ptr<Link> &urdfLink) {
  if (!urdfLink->inertial.has_value()) {
    return {};
  }

  const auto &inertial = urdfLink->inertial.value();

  // Build the inertia matrix from URDF inertia components
  contradium::rbd::Matrix3 inertia;
  inertia << inertial.ixx, inertial.ixy, inertial.ixz, inertial.ixy, inertial.iyy, inertial.iyz, inertial.ixz,
      inertial.iyz, inertial.izz;

  // Transform inertia to link frame if origin has rotation
  const contradium::rbd::Matrix3 R = rpyToRotationMatrix(inertial.origin.rpy);
  const contradium::rbd::Matrix3 inertia_link = R * inertia * R.transpose();

  // Center of mass in link frame
  const contradium::rbd::Vector3 com = inertial.origin.xyz;

  return {inertial.mass, com, inertia_link};
}

contradium::rbd::Joint toRBDJoint(const std::shared_ptr<Joint> &urdfJoint) {
  if (urdfJoint->type == JointType::FIXED) {
    return contradium::rbd::Joint(contradium::rbd::JointTypeFixed);
  } else if (urdfJoint->type == JointType::REVOLUTE || urdfJoint->type == JointType::CONTINUOUS) {
    return {contradium::rbd::JointTypeRevolute, urdfJoint->axis};
  } else if (urdfJoint->type == JointType::PRISMATIC) {
    return {contradium::rbd::JointTypePrismatic, urdfJoint->axis};
  }
  assert(false);  // Not implemented
}

// Helper to create SpatialTransform from URDF Transform (xyz, rpy)
contradium::rbd::SpatialTransform toSpatialTransform(const Transform &transform) {
  const contradium::rbd::Matrix3 R = rpyToRotationMatrix(transform.rpy);
  return {R.transpose(), transform.xyz};
}

// Recursive helper to build the model tree
// For fixed joints, the RBD model merges the body with the parent, so AddBody returns
// the parent's body ID. We track this by setting link_index = -1 for fixed joint links.
void buildModelRecursive(contradium::rbd::Model &model,
                         const std::shared_ptr<urdf::Link> &link,
                         const unsigned int parent_id,
                         const std::shared_ptr<urdf::Model> &urdf_model) {
  for (size_t i = 0; i < link->child_joints.size(); ++i) {
    const auto &child_joint = link->child_joints[i];
    const auto &child_link = link->child_links[i];
    const contradium::rbd::SpatialTransform joint_frame = toSpatialTransform(child_joint->parent_to_joint_transform);
    const contradium::rbd::Joint rbd_joint = toRBDJoint(child_joint);
    const contradium::rbd::Body rbd_body = toRBDBody(child_link);

    // Add body to model and get the new body id
    const unsigned int child_id = model.AddBody(parent_id, joint_frame, rbd_joint, rbd_body);

    // Store the link index - this works for both fixed and non-fixed joints
    // For fixed joints, AddBody returns fixed_body_discriminator + fixed_body_index
    // which CalcBodyToBaseCoordinates/CalcBodyToBaseRotation can handle correctly
    child_link->link_index = static_cast<int>(child_id);

    // For fixed joints, continue with child_id (which is the fixed body ID)
    // This allows children of fixed bodies to reference the correct parent
    buildModelRecursive(model, child_link, child_id, urdf_model);
  }
}

}  // anonymous namespace

contradium::rbd::Model toRBDModel(const std::shared_ptr<urdf::Model> &urdf_model) {
  contradium::rbd::Model model;

  // Get root link
  const auto root_link = urdf_model->getRoot();
  assert(root_link);

  root_link->link_index = 0;

  buildModelRecursive(model, root_link, 0, urdf_model);

  return model;
}

}  // namespace urdf
