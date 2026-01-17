//
// Created by f3f3xo on 12/23/25.
//

#include "urdfJoint.h"

namespace urdf {

std::shared_ptr<JointDynamics> JointDynamics::fromXml(tinyxml2::XMLElement *xml) {
  auto dynamics = std::make_shared<JointDynamics>();

  if (xml->Attribute("damping")) {
    std::string damping_str = xml->Attribute("damping");
    boost::algorithm::trim(damping_str);
    dynamics->damping = boost::lexical_cast<double>(damping_str);
  }

  if (xml->Attribute("friction")) {
    std::string friction_str = xml->Attribute("friction");
    boost::algorithm::trim(friction_str);
    dynamics->friction = boost::lexical_cast<double>(friction_str);
  }

  return dynamics;
}

std::shared_ptr<JointLimits> JointLimits::fromXml(tinyxml2::XMLElement *xml) {
  auto limits = std::make_shared<JointLimits>();

  if (xml->Attribute("lower")) {
    std::string lower_str = xml->Attribute("lower");
    boost::algorithm::trim(lower_str);
    limits->lower = boost::lexical_cast<double>(lower_str);
  }

  if (xml->Attribute("upper")) {
    std::string upper_str = xml->Attribute("upper");
    boost::algorithm::trim(upper_str);
    limits->upper = boost::lexical_cast<double>(upper_str);
  }

  assert(xml->Attribute("effort") != nullptr);
  std::string effort_str = xml->Attribute("effort");
  boost::algorithm::trim(effort_str);
  limits->effort = boost::lexical_cast<double>(effort_str);

  assert(xml->Attribute("velocity") != nullptr);
  std::string velocity_str = xml->Attribute("velocity");
  boost::algorithm::trim(velocity_str);
  limits->velocity = boost::lexical_cast<double>(velocity_str);

  return limits;
}

std::shared_ptr<JointSafety> JointSafety::fromXml(tinyxml2::XMLElement *xml) {
  auto safety = std::make_shared<JointSafety>();

  if (xml->Attribute("soft_lower_limit")) {
    std::string soft_lower_str = xml->Attribute("soft_lower_limit");
    boost::algorithm::trim(soft_lower_str);
    safety->lower_limit = boost::lexical_cast<double>(soft_lower_str);
  }

  if (xml->Attribute("soft_upper_limit")) {
    std::string soft_upper_str = xml->Attribute("soft_upper_limit");
    boost::algorithm::trim(soft_upper_str);
    safety->upper_limit = boost::lexical_cast<double>(soft_upper_str);
  }

  if (xml->Attribute("k_position")) {
    std::string k_position_str = xml->Attribute("k_position");
    boost::algorithm::trim(k_position_str);
    safety->k_position = boost::lexical_cast<double>(k_position_str);
  }

  assert(xml->Attribute("k_velocity") != nullptr);
  std::string k_velocity_str = xml->Attribute("k_velocity");
  boost::algorithm::trim(k_velocity_str);
  safety->k_velocity = boost::lexical_cast<double>(k_velocity_str);

  return safety;
}

std::shared_ptr<JointCalibration> JointCalibration::fromXml(tinyxml2::XMLElement *xml) {
  auto calibration = std::make_shared<JointCalibration>();

  if (xml->Attribute("rising")) {
    std::string rising_str = xml->Attribute("rising");
    boost::algorithm::trim(rising_str);
    calibration->rising = boost::lexical_cast<double>(rising_str);
  }

  if (xml->Attribute("falling")) {
    std::string falling_str = xml->Attribute("falling");
    boost::algorithm::trim(falling_str);
    calibration->falling = boost::lexical_cast<double>(falling_str);
  }

  return calibration;
}

std::shared_ptr<JointMimic> JointMimic::fromXml(tinyxml2::XMLElement *xml) {
  auto mimic = std::make_shared<JointMimic>();

  assert(xml->Attribute("joint") != nullptr);
  mimic->joint_name = xml->Attribute("joint");

  if (xml->Attribute("multiplier")) {
    std::string multiplier_str = xml->Attribute("multiplier");
    boost::algorithm::trim(multiplier_str);
    mimic->multiplier = boost::lexical_cast<double>(multiplier_str);
  } else {
    mimic->multiplier = 1.0;
  }

  if (xml->Attribute("offset")) {
    std::string offset_str = xml->Attribute("offset");
    boost::algorithm::trim(offset_str);
    mimic->offset = boost::lexical_cast<double>(offset_str);
  } else {
    mimic->offset = 0.0;
  }

  return mimic;
}

std::shared_ptr<Joint> Joint::fromXml(tinyxml2::XMLElement *xml) {
  auto joint = std::make_shared<Joint>();

  // Parse name attribute (required)
  assert(xml->Attribute("name") != nullptr);
  joint->name = xml->Attribute("name");

  // Parse type attribute (required)
  assert(xml->Attribute("type") != nullptr);
  std::string type_str = xml->Attribute("type");
  boost::algorithm::trim(type_str);

  if (type_str == "revolute") {
    joint->type = JointType::REVOLUTE;
  } else if (type_str == "continuous") {
    joint->type = JointType::CONTINUOUS;
  } else if (type_str == "prismatic") {
    joint->type = JointType::PRISMATIC;
  } else if (type_str == "fixed") {
    joint->type = JointType::FIXED;
  } else if (type_str == "floating") {
    joint->type = JointType::FLOATING;
  } else if (type_str == "planar") {
    joint->type = JointType::PLANAR;
  } else {
    assert(false);
  }

  // Parse origin element (optional)
  tinyxml2::XMLElement *origin = xml->FirstChildElement("origin");
  if (origin) {
    joint->parent_to_joint_transform = Transform::fromXml(origin);
  }

  // Parse parent element (required)
  tinyxml2::XMLElement *parent = xml->FirstChildElement("parent");
  assert(parent != nullptr);
  assert(parent->Attribute("link") != nullptr);
  joint->parent_link_name = parent->Attribute("link");

  // Parse child element (required)
  tinyxml2::XMLElement *child = xml->FirstChildElement("child");
  assert(child != nullptr);
  assert(child->Attribute("link") != nullptr);
  joint->child_link_name = child->Attribute("link");

  // Parse axis element (optional, defaults to (1,0,0))
  tinyxml2::XMLElement *axis = xml->FirstChildElement("axis");
  if (axis && axis->Attribute("xyz")) {
    joint->axis = fromVec3dString(axis->Attribute("xyz"));
  } else {
    joint->axis = Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  // Parse calibration element (optional)
  tinyxml2::XMLElement *calibration = xml->FirstChildElement("calibration");
  if (calibration) {
    joint->calibration = JointCalibration::fromXml(calibration);
  }

  // Parse dynamics element (optional)
  tinyxml2::XMLElement *dynamics = xml->FirstChildElement("dynamics");
  if (dynamics) {
    joint->dynamics = JointDynamics::fromXml(dynamics);
  }

  // Parse limit element (required for revolute and prismatic)
  tinyxml2::XMLElement *limit = xml->FirstChildElement("limit");
  if (limit) {
    joint->limits = JointLimits::fromXml(limit);
  }

  // Parse safety_controller element (optional)
  tinyxml2::XMLElement *safety = xml->FirstChildElement("safety_controller");
  if (safety) {
    joint->safety = JointSafety::fromXml(safety);
  }

  // Parse mimic element (optional)
  tinyxml2::XMLElement *mimic = xml->FirstChildElement("mimic");
  if (mimic) {
    joint->mimic = JointMimic::fromXml(mimic);
  }

  return joint;
}

}  // namespace urdf
