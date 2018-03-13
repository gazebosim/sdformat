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
#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::VisualPrivate
{
  /// \brief Name of the visual.
  public: std::string name = "";

  /// \brief Pointer to a box geometry.
  public: std::unique_ptr<Box> box = nullptr;

  /// \brief Pointer to a cylinder geometry.
  public: std::unique_ptr<Cylinder> cylinder = nullptr;

  /// \brief Pointer to a sphere geometry.
  public: std::unique_ptr<Sphere> sphere = nullptr;

  /// \brief Pointer to a plane geometry.
  public: std::unique_ptr<Plane> plane = nullptr;
};

/////////////////////////////////////////////////
Visual::Visual()
  : dataPtr(new VisualPrivate)
{
}

/////////////////////////////////////////////////
Visual::Visual(Visual &&_visual)
{
  this->dataPtr = _visual.dataPtr;
  _visual.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Visual::~Visual()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Visual::Load(ElementPtr _sdf)
{
  Errors errors;

  // Check that the provided SDF element is a <visual>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "visual")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Visual, but the provided SDF element is not a "
        "<visual>."});
    return errors;
  }

  // Read the visuals's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A visual name is required, but the name is not set."});
  }

  // Load the geometry
  if (_sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geom = _sdf->GetElement("geometry");
    if (geom->HasElement("box"))
    {
      this->dataPtr->box.reset(new Box());
      Errors err = this->dataPtr->box->Load(geom);
      errors.insert(errors.end(), err.begin(), err.end());
    }
    if (geom->HasElement("cylinder"))
    {
      this->dataPtr->cylinder.reset(new Cylinder());
      Errors err = this->dataPtr->cylinder->Load(geom);
      errors.insert(errors.end(), err.begin(), err.end());
    }
    if (geom->HasElement("plane"))
    {
      this->dataPtr->plane.reset(new Plane());
      Errors err = this->dataPtr->plane->Load(geom);
      errors.insert(errors.end(), err.begin(), err.end());
    }
    if (geom->HasElement("sphere"))
    {
      this->dataPtr->sphere.reset(new Sphere());
      Errors err = this->dataPtr->sphere->Load(geom);
      errors.insert(errors.end(), err.begin(), err.end());
    }
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Visual::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Visual::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const Box *Visual::BoxGeom() const
{
  return this->dataPtr->box.get();
}

/////////////////////////////////////////////////
const Sphere *Visual::SphereGeom() const
{
  return this->dataPtr->sphere.get();
}

/////////////////////////////////////////////////
const Cylinder *Visual::CylinderGeom() const
{
  return this->dataPtr->cylinder.get();
}

/////////////////////////////////////////////////
const Plane *Visual::PlaneGeom() const
{
  return this->dataPtr->plane.get();
}
