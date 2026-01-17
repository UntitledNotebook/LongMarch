//
// Created by f3f3xo on 12/23/25.
//

#ifndef LONGMARCH_URDFGEOMETRY_H
#define LONGMARCH_URDFGEOMETRY_H

#include "urdfutil.h"

namespace urdf {
enum GeometryType { SPHERE, BOX, CYLINDER, CAPSULE, MESH };

class Geometry {
 public:
  GeometryType type;
  virtual ~Geometry() = default;

  explicit Geometry(const GeometryType type) : type(type) {
  }

  static std::shared_ptr<Geometry> fromXml(tinyxml2::XMLElement *xml);
};

class Sphere : public Geometry {
 public:
  double radius;

  void clear() {
    radius = 0;
  }

  Sphere() : Geometry(GeometryType::SPHERE), radius(0.) {
  }

  static std::shared_ptr<Sphere> fromXml(const tinyxml2::XMLElement *xml);
};

class Box : public Geometry {
 public:
  Eigen::Vector3d dim;

  void clear() {
    this->dim.setOnes();
  }

  Box() : Geometry(GeometryType::BOX) {
  }

  static std::shared_ptr<Box> fromXml(tinyxml2::XMLElement *xml);
};

class Cylinder : public Geometry {
 public:
  double length;
  double radius;

  void clear() {
    length = 0;
    radius = 0;
  }

  Cylinder() : Geometry(GeometryType::CYLINDER), length(0.), radius(0.) {
  }

  static std::shared_ptr<Cylinder> fromXml(tinyxml2::XMLElement *xml);
};

class Capsule : public Geometry {
 public:
  double length;
  double radius;

  void clear() {
    length = 0;
    radius = 0;
  }

  Capsule() : Geometry(GeometryType::CAPSULE), length(0.), radius(0.) {
  }

  static std::shared_ptr<Capsule> fromXml(tinyxml2::XMLElement *xml);
};

class Mesh : public Geometry {
 public:
  std::string filename;
  Eigen::Vector3d scale;

  void clear() {
    filename.clear();
    scale.setOnes();
  }

  Mesh() : Geometry(GeometryType::MESH), scale(Eigen::Vector3d(1., 1., 1.)) {
  }

  static std::shared_ptr<Mesh> fromXml(tinyxml2::XMLElement *xml);
};
}  // namespace urdf

#endif  // LONGMARCH_URDFGEOMETRY_H
