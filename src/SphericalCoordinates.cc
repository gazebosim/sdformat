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
#include <string>

#include "sdf/Types.hh"
#include "sdf/SphericalCoordinates.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::SphericalCoordinatesPrivate
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates()
  : dataPtr(new SphericalCoordinatesPrivate)
{
}

/////////////////////////////////////////////////
SphericalCoordinates::~SphericalCoordinates()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
SphericalCoordinates::SphericalCoordinates(SphericalCoordinates &&_coordinates)
{
  this->dataPtr = _coordinates.dataPtr;
  _coordinates.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors SphericalCoordinates::Load(sdf::ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <spherical_coordinates>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "spherical_coordinates")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a SphericalCoordinates, but the provided SDF"
        " element is not a <spherical_coordinates>."});
    return errors;
  }

  std::pair<double, bool> doublePair = _sdf->Get<double>("latitude_deg",
      this->dataPtr->latitude);
  if (!doublePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <latitude_deg>, child of "
        "<spherical_coordinates>."});
  }
  this->dataPtr->latitude = doublePair.first;

  doublePair = _sdf->Get<double>("longitude_deg", this->dataPtr->longitude);
  if (!doublePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <longitude_deg>, child of "
        "<spherical_coordinates>."});
  }
  this->dataPtr->longitude = doublePair.first;

  doublePair = _sdf->Get<double>("heading_deg",
      this->dataPtr->heading.Degree());
  if (!doublePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <heading_deg>, child of "
        "<spherical_coordinates>."});
  }
  this->dataPtr->heading.Degree(doublePair.first);

  doublePair = _sdf->Get<double>("elevation", this->dataPtr->elevation);
  if (!doublePair.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Missing required element <elevation>, child of "
        "<spherical_coordinates>."});
  }
  this->dataPtr->elevation = doublePair.first;

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr SphericalCoordinates::Element() const
{
  return this->dataPtr->sdf;
}
