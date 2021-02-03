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

#include <optional>

#include "sdf/Geometry.hh"
#include "sdf/Box.hh"
#include "sdf/Capsule.hh"
#include "sdf/Cylinder.hh"
#include "sdf/Ellipsoid.hh"
#include "sdf/Heightmap.hh"
#include "sdf/Mesh.hh"
#include "sdf/Plane.hh"
#include "sdf/Sphere.hh"

#include "Utils.hh"

using namespace sdf;

// Private data class
class sdf::Geometry::Implementation
{
  // \brief The geometry type.
  public: GeometryType type = GeometryType::EMPTY;

  /// \brief Optional box.
  public: std::optional<Box> box;

  /// \brief Optional capsule.
  public: std::optional<Capsule> capsule;

  /// \brief Optional cylinder.
  public: std::optional<Cylinder> cylinder;

  /// \brief Optional ellipsoid
  public: std::optional<Ellipsoid> ellipsoid;

  /// \brief Optional plane.
  public: std::optional<Plane> plane;

  /// \brief Optional sphere.
  public: std::optional<Sphere> sphere;

  /// \brief Optional mesh.
  public: std::optional<Mesh> mesh;

  /// \brief Optional heightmap.
  public: std::optional<Heightmap> heightmap;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Geometry::Geometry()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
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
    this->dataPtr->box.emplace();
    Errors err = this->dataPtr->box->Load(_sdf->GetElement("box"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("capsule"))
  {
    this->dataPtr->type = GeometryType::CAPSULE;
    this->dataPtr->capsule.emplace();
    Errors err = this->dataPtr->capsule->Load(_sdf->GetElement("capsule"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("cylinder"))
  {
    this->dataPtr->type = GeometryType::CYLINDER;
    this->dataPtr->cylinder.emplace();
    Errors err = this->dataPtr->cylinder->Load(_sdf->GetElement("cylinder"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("ellipsoid"))
  {
    this->dataPtr->type = GeometryType::ELLIPSOID;
    this->dataPtr->ellipsoid.emplace();
    Errors err = this->dataPtr->ellipsoid->Load(_sdf->GetElement("ellipsoid"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("plane"))
  {
    this->dataPtr->type = GeometryType::PLANE;
    this->dataPtr->plane.emplace();
    Errors err = this->dataPtr->plane->Load(_sdf->GetElement("plane"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("sphere"))
  {
    this->dataPtr->type = GeometryType::SPHERE;
    this->dataPtr->sphere.emplace();
    Errors err = this->dataPtr->sphere->Load(_sdf->GetElement("sphere"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("mesh"))
  {
    this->dataPtr->type = GeometryType::MESH;
    this->dataPtr->mesh.emplace();
    Errors err = this->dataPtr->mesh->Load(_sdf->GetElement("mesh"));
    errors.insert(errors.end(), err.begin(), err.end());
  }
  else if (_sdf->HasElement("heightmap"))
  {
    this->dataPtr->type = GeometryType::HEIGHTMAP;
    this->dataPtr->heightmap.emplace();
    Errors err = this->dataPtr->heightmap->Load(_sdf->GetElement("heightmap"));
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
  return optionalToPointer(this->dataPtr->box);
}

/////////////////////////////////////////////////
void Geometry::SetBoxShape(const Box &_box)
{
  this->dataPtr->box = _box;
}

/////////////////////////////////////////////////
const Sphere *Geometry::SphereShape() const
{
  return optionalToPointer(this->dataPtr->sphere);
}

/////////////////////////////////////////////////
void Geometry::SetSphereShape(const Sphere &_sphere)
{
  this->dataPtr->sphere = _sphere;
}

/////////////////////////////////////////////////
const Capsule *Geometry::CapsuleShape() const
{
  return optionalToPointer(this->dataPtr->capsule);
}

/////////////////////////////////////////////////
void Geometry::SetCapsuleShape(const Capsule &_capsule)
{
  this->dataPtr->capsule = _capsule;
}

/////////////////////////////////////////////////
const Cylinder *Geometry::CylinderShape() const
{
  return optionalToPointer(this->dataPtr->cylinder);
}

/////////////////////////////////////////////////
void Geometry::SetCylinderShape(const Cylinder &_cylinder)
{
  this->dataPtr->cylinder = _cylinder;
}

/////////////////////////////////////////////////
const Ellipsoid *Geometry::EllipsoidShape() const
{
  return optionalToPointer(this->dataPtr->ellipsoid);
}

/////////////////////////////////////////////////
void Geometry::SetEllipsoidShape(const Ellipsoid &_ellipsoid)
{
  this->dataPtr->ellipsoid = _ellipsoid;
}

/////////////////////////////////////////////////
const Plane *Geometry::PlaneShape() const
{
  return optionalToPointer(this->dataPtr->plane);
}

/////////////////////////////////////////////////
void Geometry::SetPlaneShape(const Plane &_plane)
{
  this->dataPtr->plane = _plane;
}

/////////////////////////////////////////////////
const Mesh *Geometry::MeshShape() const
{
  return optionalToPointer(this->dataPtr->mesh);
}

/////////////////////////////////////////////////
void Geometry::SetMeshShape(const Mesh &_mesh)
{
  this->dataPtr->mesh = _mesh;
}

/////////////////////////////////////////////////
const Heightmap *Geometry::HeightmapShape() const
{
  return optionalToPointer(this->dataPtr->heightmap);
}

/////////////////////////////////////////////////
void Geometry::SetHeightmapShape(const Heightmap &_heightmap)
{
  this->dataPtr->heightmap = _heightmap;
}

/////////////////////////////////////////////////
sdf::ElementPtr Geometry::Element() const
{
  return this->dataPtr->sdf;
}
