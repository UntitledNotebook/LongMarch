//
// Created by f3f3xo on 12/23/25.
//

#ifndef LONGMARCH_URDFMODEL_H
#define LONGMARCH_URDFMODEL_H

#include "urdfGeometry.h"
#include "urdfJoint.h"
#include "urdfLink.h"
#include "urdfutil.h"

namespace urdf {
struct Model {
  std::string robotName;
  std::shared_ptr<Link> rootLink;

  std::map<std::string, std::shared_ptr<Link>> linkMap;
  std::map<std::string, std::shared_ptr<Joint>> jointMap;
  std::map<std::string, std::shared_ptr<Material>> materialMap;

  [[nodiscard]] const std::string &getName() const {
    return robotName;
  }
  [[nodiscard]] std::shared_ptr<Link> getRoot() const {
    return rootLink;
  }

  std::shared_ptr<Link> getLink(const std::string &name);
  std::shared_ptr<Joint> getJoint(const std::string &name);
  std::shared_ptr<Material> getMaterial(const std::string &name);

  void getLinks(std::vector<std::shared_ptr<Link>> &linklist) const;

  void clear() {
    robotName.clear();
    linkMap.clear();
    jointMap.clear();
    materialMap.clear();
    rootLink = nullptr;
  };

  void initLinkTree(std::map<std::string, std::string> &parent_link_tree);
  void findRoot(const std::map<std::string, std::string> &parent_link_tree);

  Model() {
    clear();
  }

  static std::shared_ptr<Model> fromXmlStr(const std::string &xml_string);
};
}  // namespace urdf

#endif  // LONGMARCH_URDFMODEL_H
