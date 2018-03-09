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

using namespace sdf;

// Private data class
class sdf::GeometryPrivate
{
  public: ignition::math::Vector3d size;
};

/////////////////////////////////////////////////
Geometry::Geometry()
  : dataPtr(new GeometryPrivate)
{
}

/////////////////////////////////////////////////
Geometry::Geometry(Geometry &&_geometry)
{
  this->dataPtr = _collision.dataPtr;
  _collision.dataPtr = nullptr;
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
    sdf::ElementPtr boxElem = _sdf->GetElement("box");
    std::pair<ignition::math::Vector3d, bool> sizePair =
      boxElem->Get<ignition::math::Vector3d>("size",
          ignition::math::Vector3d::Zero);

    if (!sizePair.second)
      // HERE
  }

}
