//
// Created by f3f3xo on 12/23/25.
//

#include "urdfGeometry.h"

namespace urdf {

std::shared_ptr<Sphere> Sphere::fromXml(const tinyxml2::XMLElement *xml) {
  auto sphere = std::make_shared<Sphere>();
  assert(xml->Attribute("radius") != nullptr);
  std::string r = xml->Attribute("radius");
  boost::algorithm::trim(r);
  sphere->radius = boost::lexical_cast<double>(r);
  return sphere;
}

std::shared_ptr<Box> Box::fromXml(tinyxml2::XMLElement *xml) {
  auto box = std::make_shared<Box>();
  assert(xml->Attribute("size") != nullptr);
  box->dim = fromVec3dString(xml->Attribute("size"));
  return box;
}

std::shared_ptr<Cylinder> Cylinder::fromXml(tinyxml2::XMLElement *xml) {
  auto cylinder = std::make_shared<Cylinder>();
  assert(xml->Attribute("radius") != nullptr);
  assert(xml->Attribute("length") != nullptr);
  std::string r = xml->Attribute("radius");
  boost::algorithm::trim(r);
  cylinder->radius = boost::lexical_cast<double>(r);

  std::string l = xml->Attribute("length");
  boost::algorithm::trim(l);
  cylinder->length = boost::lexical_cast<double>(l);

  return cylinder;
}

std::shared_ptr<Capsule> Capsule::fromXml(tinyxml2::XMLElement *xml) {
  auto capsule = std::make_shared<Capsule>();
  assert(xml->Attribute("radius") != nullptr);
  assert(xml->Attribute("length") != nullptr);
  std::string r = xml->Attribute("radius");
  boost::algorithm::trim(r);
  capsule->radius = boost::lexical_cast<double>(r);

  std::string l = xml->Attribute("length");
  boost::algorithm::trim(l);
  capsule->length = boost::lexical_cast<double>(l);

  return capsule;
}

std::shared_ptr<Mesh> Mesh::fromXml(tinyxml2::XMLElement *xml) {
  auto mesh = std::make_shared<Mesh>();
  assert(xml->Attribute("filename") != nullptr);
  mesh->filename = xml->Attribute("filename");

  if (xml->Attribute("scale") != nullptr) {
    mesh->scale = fromVec3dString(xml->Attribute("scale"));
  }

  return mesh;
}

std::shared_ptr<Geometry> Geometry::fromXml(tinyxml2::XMLElement *xml) {
  assert(xml != nullptr);

  std::shared_ptr<Geometry> geom;

  // Get the geometry type
  tinyxml2::XMLElement *shape = xml->FirstChildElement();
  assert(shape != nullptr);

  std::string type_name = shape->Name();
  if (type_name == "sphere") {
    geom = Sphere::fromXml(shape);
  } else if (type_name == "box") {
    geom = Box::fromXml(shape);
  } else if (type_name == "cylinder") {
    geom = Cylinder::fromXml(shape);
  } else if (type_name == "capsule") {
    geom = Capsule::fromXml(shape);
  } else if (type_name == "mesh") {
    geom = Mesh::fromXml(shape);
  } else {
    assert(false);
  }

  return geom;
}

}  // namespace urdf
