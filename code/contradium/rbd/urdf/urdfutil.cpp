//
// Created by f3f3xo on 12/23/25.
//

#include "urdfutil.h"

namespace urdf {

Eigen::Vector3d fromVec3dString(const std::string &str) {
  Eigen::Vector3d vec;
  std::string trimmed = str;
  boost::algorithm::trim(trimmed);
  std::vector<std::string> pieces;
  boost::algorithm::split(pieces, trimmed, boost::algorithm::is_space(),
                          boost::algorithm::token_compress_on);
  assert(pieces.size() == 3);

  vec.x() = boost::lexical_cast<double>(pieces[0]);
  vec.y() = boost::lexical_cast<double>(pieces[1]);
  vec.z() = boost::lexical_cast<double>(pieces[2]);

  return vec;
}

Eigen::Vector4f fromVec4fString(const std::string &str) {
  Eigen::Vector4f vec;
  std::string trimmed = str;
  boost::algorithm::trim(trimmed);
  std::vector<std::string> pieces;
  boost::algorithm::split(pieces, trimmed, boost::algorithm::is_space(),
                          boost::algorithm::token_compress_on);
  assert(pieces.size() == 4);

  vec.x() = boost::lexical_cast<float>(pieces[0]);
  vec.y() = boost::lexical_cast<float>(pieces[1]);
  vec.z() = boost::lexical_cast<float>(pieces[2]);
  vec.w() = boost::lexical_cast<float>(pieces[3]);

  return vec;
}

Transform Transform::fromXml(tinyxml2::XMLElement* xml) {
  Transform transform;
  transform.clear();

  if (xml) {
    if (xml->Attribute("xyz")) {
      transform.xyz = fromVec3dString(xml->Attribute("xyz"));
    }

    if (xml->Attribute("rpy")) {
      transform.rpy = fromVec3dString(xml->Attribute("rpy"));
    }
  }

  return transform;
}

}  // namespace urdf
