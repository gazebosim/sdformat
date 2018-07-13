/*
 * Copyright 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "sdf/Geometry.hh"
#include "sdf/Box.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Sphere.hh"

using namespace sdf;

// Private data class
class sdf::GeometryPrivate
{
  // \brief The geometry type.
  public: GeometryType type = GeometryType::EMPTY;

  /// \brief Pointer to a box.
  public: std::unique_ptr<Box> box;

  /// \brief Pointer to a cylinder.
  public: std::unique_ptr<Cylinder> cylinder;

  /// \brief Pointer to a plane.
  public: std::unique_ptr<Plane> plane;

  /// \brief Pointer to a sphere.
  public: std::unique_ptr<Sphere> sphere;

  /// \brief Pointer to a mesh.
  public: std::unique_ptr<Mesh> mesh;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Geometry::Geometry()
  : dataPtr(new GeometryPrivate)
{
}

/////////////////////////////////////////////////
Geometry::~Geometry()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Geometry::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Geometry, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <geometry>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "geometry")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Geometry, but the provided SDF element is not a "
        "<geometry>."});
    return errors;
  }

  if (_sdf->HasElement("box"))
  {
    this->dataPtr->type = GeometryType::BOX;
    this->dataPtr->box.reset(new Box());
    Errors err = this->dataPtr->box->Load(_sdf->GetElement("box"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("cylinder"))
  {
    this->dataPtr->type = GeometryType::CYLINDER;
    this->dataPtr->cylinder.reset(new Cylinder());
    Errors err = this->dataPtr->cylinder->Load(_sdf->GetElement("cylinder"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("plane"))
  {
    this->dataPtr->type = GeometryType::PLANE;
    this->dataPtr->plane.reset(new Plane());
    Errors err = this->dataPtr->plane->Load(_sdf->GetElement("plane"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("sphere"))
  {
    this->dataPtr->type = GeometryType::SPHERE;
    this->dataPtr->sphere.reset(new Sphere());
    Errors err = this->dataPtr->sphere->Load(_sdf->GetElement("sphere"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("mesh"))
  {
    this->dataPtr->type = GeometryType::MESH;
    this->dataPtr->mesh.reset(new Mesh());
    Errors err = this->dataPtr->mesh->Load(_sdf->GetElement("mesh"));
    errors.insert(errors.end(), err.begin(), err.end());
  }

  return errors;
}

/////////////////////////////////////////////////
GeometryType Geometry::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Geometry::SetType(const GeometryType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
const Box *Geometry::BoxShape() const
{
  return this->dataPtr->box.get();
}

/////////////////////////////////////////////////
const Sphere *Geometry::SphereShape() const
{
  return this->dataPtr->sphere.get();
}

/////////////////////////////////////////////////
const Cylinder *Geometry::CylinderShape() const
{
  return this->dataPtr->cylinder.get();
}

/////////////////////////////////////////////////
const Plane *Geometry::PlaneShape() const
{
  return this->dataPtr->plane.get();
}

/////////////////////////////////////////////////
const Mesh *Geometry::MeshShape() const
{
  return this->dataPtr->mesh.get();
}

/////////////////////////////////////////////////
sdf::ElementPtr Geometry::Element() const
{
  return this->dataPtr->sdf;
}
