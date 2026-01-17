//
// Created by f3f3xo on 12/23/25.
//

#include "urdfModel.h"

namespace urdf {

std::shared_ptr<Link> Model::getLink(const std::string &name) {
  if (const auto it = linkMap.find(name); it != linkMap.end()) {
    return it->second;
  }
  return nullptr;
}

std::shared_ptr<Joint> Model::getJoint(const std::string &name) {
  if (const auto it = jointMap.find(name); it != jointMap.end()) {
    return it->second;
  }
  return nullptr;
}

std::shared_ptr<Material> Model::getMaterial(const std::string &name) {
  if (const auto it = materialMap.find(name); it != materialMap.end()) {
    return it->second;
  }
  return nullptr;
}

void Model::getLinks(std::vector<std::shared_ptr<Link>> &linklist) const {
  linklist.clear();
  for (const auto &[name, link] : linkMap) {
    linklist.push_back(link);
  }
}

void Model::initLinkTree(std::map<std::string, std::string> &parent_link_tree) {
  // Build the parent-child relationships
  for (auto &[joint_name, joint] : jointMap) {
    // Get parent and child links
    auto parent_link = getLink(joint->parent_link_name);
    auto child_link = getLink(joint->child_link_name);

    if (!parent_link || !child_link) {
      continue;  // Skip invalid joints
    }

    // Set up the tree relationships
    parent_link->child_joints.push_back(joint);
    parent_link->child_links.push_back(child_link);

    child_link->setParentJoint(joint);
    child_link->setParentLink(parent_link);

    // Record in parent_link_tree for root finding
    parent_link_tree[child_link->name] = parent_link->name;
  }
}

void Model::findRoot(const std::map<std::string, std::string> &parent_link_tree) {
  // Find the root link (a link that has no parent)
  for (const auto &[link_name, link] : linkMap) {
    if (parent_link_tree.find(link_name) == parent_link_tree.end()) {
      // This link has no parent, so it's a potential root
      if (!rootLink) {
        rootLink = link;
      }
    }
  }
}

std::shared_ptr<Model> Model::fromXmlStr(const std::string &xml_string) {
  tinyxml2::XMLDocument doc;

  // Parse the XML string
  tinyxml2::XMLError error = doc.Parse(xml_string.c_str());
  assert(error == tinyxml2::XML_SUCCESS);

  // Get the root robot element
  tinyxml2::XMLElement *robot_xml = doc.FirstChildElement("robot");
  assert(robot_xml != nullptr);

  auto model = std::make_shared<Model>();

  // Parse robot name attribute
  if (robot_xml->Attribute("name")) {
    model->robotName = robot_xml->Attribute("name");
  }

  // Parse all material elements
  for (tinyxml2::XMLElement *material_xml = robot_xml->FirstChildElement("material"); material_xml != nullptr;
       material_xml = material_xml->NextSiblingElement("material")) {
    if (auto material = Material::fromXml(material_xml, true); material && !material->name.empty()) {
      model->materialMap[material->name] = material;
    }
  }

  // Parse all link elements
  for (tinyxml2::XMLElement *link_xml = robot_xml->FirstChildElement("link"); link_xml != nullptr;
       link_xml = link_xml->NextSiblingElement("link")) {
    if (auto link = Link::fromXml(link_xml); link && !link->name.empty()) {
      model->linkMap[link->name] = link;

      // Handle materials in visuals
      for (auto &visual : link->visuals) {
        if (!visual->material_name.empty()) {
          if (auto mat = model->getMaterial(visual->material_name)) {
            visual->material = mat;
          }
        }
      }
    }
  }

  // Parse all joint elements
  for (tinyxml2::XMLElement *joint_xml = robot_xml->FirstChildElement("joint"); joint_xml != nullptr;
       joint_xml = joint_xml->NextSiblingElement("joint")) {
    if (auto joint = Joint::fromXml(joint_xml); joint && !joint->name.empty()) {
      model->jointMap[joint->name] = joint;
    }
  }

  // Build the link tree structure
  std::map<std::string, std::string> parent_link_tree;
  model->initLinkTree(parent_link_tree);

  // Find the root link
  model->findRoot(parent_link_tree);

  return model;
}

}  // namespace urdf
