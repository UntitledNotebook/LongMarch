//
// Created by f3f3xo on 12/23/25.
//

#ifndef LONGMARCH_URDFLINK_H
#define LONGMARCH_URDFLINK_H

#include "urdfJoint.h"
#include "urdfutil.h"
#include "urdfGeometry.h"

namespace urdf {
struct Material {
  std::string name;
  std::string texture_filename;
  Eigen::Vector4f color;

  void clear() {
    name.clear();
    texture_filename.clear();
    color.setZero();
  }

  Material() {
    clear();
  }

  static std::shared_ptr<Material> fromXml(tinyxml2::XMLElement *xml, bool);
};

struct Inertial {
  Transform origin;
  double mass;
  double ixx, ixy, ixz, iyy, iyz, izz;

  void clear() {
    origin.clear();
    mass = 0.;
    ixx = 0.;
    ixy = 0.;
    ixz = 0.;
    iyy = 0.;
    iyz = 0.;
    izz = 0;
  }

  Inertial() : mass(0.), ixx(0.), ixy(0.), ixz(0.), iyy(0.), iyz(0.), izz(0.) {
  }

  static Inertial fromXml(tinyxml2::XMLElement *xml);
};

struct Visual {
  std::string name;
  std::string material_name;
  Transform origin;

  std::optional<std::shared_ptr<Geometry>> geometry;
  std::optional<std::shared_ptr<Material>> material;

  void clear() {
    origin.clear();
    name.clear();
    material_name.clear();

    material.reset();
    geometry.reset();
  }

  Visual() {
    this->clear();
  }

  static std::shared_ptr<Visual> fromXml(tinyxml2::XMLElement *xml);
};

struct Collision {
  std::string name;
  Transform origin;
  std::optional<std::shared_ptr<Geometry>> geometry;

  void clear() {
    name.clear();
    origin.clear();

    geometry.reset();
  }

  Collision() {
    this->clear();
  }

  static std::shared_ptr<Collision> fromXml(tinyxml2::XMLElement *xml);
};

const char *getParentLinkName(tinyxml2::XMLElement *xml);

struct Link {
  std::string name;

  std::optional<Inertial> inertial;

  std::vector<std::shared_ptr<Collision>> collisions;
  std::vector<std::shared_ptr<Visual>> visuals;

  std::shared_ptr<Joint> parent_joint;
  std::shared_ptr<Link> parent_link;

  std::vector<std::shared_ptr<Joint>> child_joints;
  std::vector<std::shared_ptr<Link>> child_links;

  int link_index;

  [[nodiscard]] std::shared_ptr<Link> getParent() const {
    return parent_link;
  }

  void setParentLink(const std::shared_ptr<Link> &parent) {
    parent_link = parent;
  }

  void setParentJoint(const std::shared_ptr<Joint> &parent) {
    parent_joint = parent;
  }

  void clear() {
    name.clear();
    link_index = -1;

    child_joints.clear();
    child_links.clear();
    collisions.clear();
    visuals.clear();

    inertial.reset();

    parent_joint = nullptr;
    parent_link = nullptr;
  }

  Link() {
    this->clear();
  }

  static std::shared_ptr<Link> fromXml(tinyxml2::XMLElement *xml);
};

}  // namespace urdf

#endif  // LONGMARCH_URDFLINK_H
