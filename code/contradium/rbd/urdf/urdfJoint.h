//
// Created by f3f3xo on 12/23/25.
//

#ifndef LONGMARCH_URDFJOINT_H
#define LONGMARCH_URDFJOINT_H

#include "urdfutil.h"

namespace urdf {

struct JointDynamics {
  double damping;
  double friction;

  void clear() {
    damping = 0;
    friction = 0;
  }

  JointDynamics() : damping(0.), friction(0.) {
  }

  static std::shared_ptr<JointDynamics> fromXml(tinyxml2::XMLElement *xml);
};

struct JointLimits {
  double lower;
  double upper;
  double effort;
  double velocity;

  void clear() {
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
  }

  JointLimits() : lower(0.), upper(0.), effort(0.), velocity(0.) {
  }

  static std::shared_ptr<JointLimits> fromXml(tinyxml2::XMLElement *xml);
};

struct JointSafety {
  double upper_limit;
  double lower_limit;
  double k_position;
  double k_velocity;

  void clear() {
    upper_limit = 0;
    lower_limit = 0;
    k_position = 0;
    k_velocity = 0;
  }

  JointSafety() : upper_limit(0.), lower_limit(0.), k_position(0.), k_velocity(0.) {};

  static std::shared_ptr<JointSafety> fromXml(tinyxml2::XMLElement *xml);
};

struct JointCalibration {
  std::optional<double> rising;
  std::optional<double> falling;

  void clear() {
    rising.reset();
    falling.reset();
  }

  JointCalibration() {
    clear();
  }
  static std::shared_ptr<JointCalibration> fromXml(tinyxml2::XMLElement *xml);
};

struct JointMimic {
  std::string joint_name;
  double offset;
  double multiplier;

  void clear() {
    joint_name = "";
    offset = 0.;
    multiplier = 0.;
  }

  JointMimic() : joint_name(""), offset(0.), multiplier(0.) {
  }
  static std::shared_ptr<JointMimic> fromXml(tinyxml2::XMLElement *xml);
};

enum JointType {
  UNKNOWN,
  REVOLUTE,  // rotation axis
  CONTINUOUS,
  PRISMATIC,  // translation axis
  FLOATING,
  PLANAR,  // plane normal axis
  FIXED
};

struct Joint {
  std::string name;
  JointType type;
  Eigen::Vector3d axis;
  std::string child_link_name;
  std::string parent_link_name;
  Transform parent_to_joint_transform;

  std::optional<std::shared_ptr<JointDynamics>> dynamics;
  std::optional<std::shared_ptr<JointLimits>> limits;
  std::optional<std::shared_ptr<JointSafety>> safety;
  std::optional<std::shared_ptr<JointCalibration>> calibration;
  std::optional<std::shared_ptr<JointMimic>> mimic;

  void clear() {
    this->axis.setZero();
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->parent_to_joint_transform.clear();

    this->dynamics.reset();
    this->limits.reset();
    this->safety.reset();
    this->calibration.reset();
    this->type = JointType::UNKNOWN;
  }

  Joint() : type(JointType::UNKNOWN) {
    clear();
  }

  static std::shared_ptr<Joint> fromXml(tinyxml2::XMLElement *xml);
};

}  // namespace urdf

#endif  // LONGMARCH_URDFJOINT_H
