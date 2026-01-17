//
// Created by f3f3xo on 12/23/25.
//

#ifndef LONGMARCH_URDFUTIL_H
#define LONGMARCH_URDFUTIL_H

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <optional>

#include "Eigen/Eigen"
#include "boost/algorithm/string.hpp"
#include "boost/lexical_cast.hpp"
#include "tinyxml2.h"

namespace urdf {

Eigen::Vector3d fromVec3dString(const std::string &str);

Eigen::Vector4f fromVec4fString(const std::string &str);

struct Transform {
  Eigen::Vector3d xyz;
  Eigen::Vector3d rpy;

  void clear() {
    this->xyz.setZero();
    this->rpy.setZero();
  }

  static Transform fromXml(tinyxml2::XMLElement* xml);
};

}  // namespace urdf

#endif  // LONGMARCH_URDFUTIL_H
