//
// Created by f3f3xo on 12/23/25.
//

#include "urdfLink.h"

namespace urdf {

std::shared_ptr<Material> Material::fromXml(tinyxml2::XMLElement *xml, bool) {
  auto material = std::make_shared<Material>();

  if (xml->Attribute("name")) {
    material->name = xml->Attribute("name");
  }

  // Parse color element
  tinyxml2::XMLElement *color = xml->FirstChildElement("color");
  if (color && color->Attribute("rgba")) {
    material->color = fromVec4fString(color->Attribute("rgba"));
  }

  // Parse texture element
  tinyxml2::XMLElement *texture = xml->FirstChildElement("texture");
  if (texture && texture->Attribute("filename")) {
    material->texture_filename = texture->Attribute("filename");
  }

  return material;
}

Inertial Inertial::fromXml(tinyxml2::XMLElement *xml) {
  Inertial inertial;

  // Parse origin element
  tinyxml2::XMLElement *origin = xml->FirstChildElement("origin");
  if (origin) {
    inertial.origin = Transform::fromXml(origin);
  }

  // Parse mass element
  tinyxml2::XMLElement *mass = xml->FirstChildElement("mass");
  if (mass && mass->Attribute("value")) {
    std::string mass_str = mass->Attribute("value");
    boost::algorithm::trim(mass_str);
    inertial.mass = boost::lexical_cast<double>(mass_str);
  }

  // Parse inertia element
  tinyxml2::XMLElement *inertia = xml->FirstChildElement("inertia");
  if (inertia) {
    if (inertia->Attribute("ixx")) {
      std::string ixx_str = inertia->Attribute("ixx");
      boost::algorithm::trim(ixx_str);
      inertial.ixx = boost::lexical_cast<double>(ixx_str);
    }
    if (inertia->Attribute("ixy")) {
      std::string ixy_str = inertia->Attribute("ixy");
      boost::algorithm::trim(ixy_str);
      inertial.ixy = boost::lexical_cast<double>(ixy_str);
    }
    if (inertia->Attribute("ixz")) {
      std::string ixz_str = inertia->Attribute("ixz");
      boost::algorithm::trim(ixz_str);
      inertial.ixz = boost::lexical_cast<double>(ixz_str);
    }
    if (inertia->Attribute("iyy")) {
      std::string iyy_str = inertia->Attribute("iyy");
      boost::algorithm::trim(iyy_str);
      inertial.iyy = boost::lexical_cast<double>(iyy_str);
    }
    if (inertia->Attribute("iyz")) {
      std::string iyz_str = inertia->Attribute("iyz");
      boost::algorithm::trim(iyz_str);
      inertial.iyz = boost::lexical_cast<double>(iyz_str);
    }
    if (inertia->Attribute("izz")) {
      std::string izz_str = inertia->Attribute("izz");
      boost::algorithm::trim(izz_str);
      inertial.izz = boost::lexical_cast<double>(izz_str);
    }
  }

  return inertial;
}

std::shared_ptr<Visual> Visual::fromXml(tinyxml2::XMLElement *xml) {
  auto visual = std::make_shared<Visual>();

  // Parse name attribute
  if (xml->Attribute("name")) {
    visual->name = xml->Attribute("name");
  }

  // Parse origin element
  tinyxml2::XMLElement *origin = xml->FirstChildElement("origin");
  if (origin) {
    visual->origin = Transform::fromXml(origin);
  }

  // Parse geometry element
  tinyxml2::XMLElement *geometry = xml->FirstChildElement("geometry");
  if (geometry) {
    visual->geometry = Geometry::fromXml(geometry);
  }

  // Parse material element
  tinyxml2::XMLElement *material = xml->FirstChildElement("material");
  if (material) {
    visual->material = Material::fromXml(material, false);
    if (visual->material.has_value() && !visual->material.value()->name.empty()) {
      visual->material_name = visual->material.value()->name;
    }
  }

  return visual;
}

std::shared_ptr<Collision> Collision::fromXml(tinyxml2::XMLElement *xml) {
  auto collision = std::make_shared<Collision>();

  // Parse name attribute
  if (xml->Attribute("name")) {
    collision->name = xml->Attribute("name");
  }

  // Parse origin element
  tinyxml2::XMLElement *origin = xml->FirstChildElement("origin");
  if (origin) {
    collision->origin = Transform::fromXml(origin);
  }

  // Parse geometry element
  tinyxml2::XMLElement *geometry = xml->FirstChildElement("geometry");
  if (geometry) {
    collision->geometry = Geometry::fromXml(geometry);
  }

  return collision;
}

const char *getParentLinkName(tinyxml2::XMLElement *xml) {
  tinyxml2::XMLElement *parent = xml->FirstChildElement("parent");
  if (parent && parent->Attribute("link")) {
    return parent->Attribute("link");
  }
  return nullptr;
}

std::shared_ptr<Link> Link::fromXml(tinyxml2::XMLElement *xml) {
  auto link = std::make_shared<Link>();

  // Parse name attribute (required)
  assert(xml->Attribute("name") != nullptr);
  link->name = xml->Attribute("name");

  // Parse inertial element (optional)
  tinyxml2::XMLElement *inertial = xml->FirstChildElement("inertial");
  if (inertial) {
    link->inertial = Inertial::fromXml(inertial);
  }

  // Parse visual elements (multiple allowed)
  for (tinyxml2::XMLElement *visual = xml->FirstChildElement("visual");
       visual != nullptr;
       visual = visual->NextSiblingElement("visual")) {
    link->visuals.push_back(Visual::fromXml(visual));
  }

  // Parse collision elements (multiple allowed)
  for (tinyxml2::XMLElement *collision = xml->FirstChildElement("collision");
       collision != nullptr;
       collision = collision->NextSiblingElement("collision")) {
    link->collisions.push_back(Collision::fromXml(collision));
  }

  return link;
}

}  // namespace urdf
