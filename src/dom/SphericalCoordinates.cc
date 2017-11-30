/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <iostream>
#include "sdf/dom/SphericalCoordinates.hh"

using namespace sdf;

class sdf::SphericalCoordinatesPrivate
{
  /// \brief The surface model.
  public: std::string surfaceModel = "EARTH_WGS84";

  /// \brief The world frame orientation.
  public: std::string worldFrameOrientation = "ENU";

  /// \brief Latitude at origin of the reference frame
  public: double latitude = 0.0;

  /// \brief Longitude at origin of the reference frame
  public: double longitude = 0.0;

  /// \brief Elevation at origin of the reference frame
  public: double elevation = 0.0;

  /// \brief Heading offset of the reference frame,
  public: double heading = 0.0;
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
bool SphericalCoordinates::Load(ElementPtr _sdf)
{
  if (_sdf->GetName() != "spherical_coordinates")
  {
    std::cerr << "Provided element pointer is not a <spherical_coordinates> "
      << "element.\n";
    return false;
  }

  bool result = true;

  // Get the surface model
  std::pair<std::string, bool> surfaceModelPair =
    _sdf->Get<std::string>("surface_model", this->dataPtr->surfaceModel);
  if (!surfaceModelPair.second)
  {
    std::cerr << "Missing <surface_model> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->dataPtr->surfaceModel << std::endl;
    result = false;
  }
  this->dataPtr->surfaceModel = surfaceModelPair.first;

  // Get the world frame orientation
  std::pair<std::string, bool> worldOrientPair =
    _sdf->Get<std::string>("world_frame_orientation",
        this->dataPtr->worldFrameOrientation);
  if (!worldOrientPair.second)
  {
    std::cerr << "Missing <world_frame_orientation> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->dataPtr->worldFrameOrientation << std::endl;
    result = false;
  }
  this->dataPtr->worldFrameOrientation = worldOrientPair.first;

  // Get the latitude
  std::pair<double, bool> latPair = _sdf->Get<double>("latitude_deg",
        this->dataPtr->latitude);
  if (!latPair.second)
  {
    std::cerr << "Missing <latitude> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->dataPtr->latitude << std::endl;
    result = false;
  }
  this->dataPtr->latitude = latPair.first;

  // Get the longitude
  std::pair<double, bool> lonPair = _sdf->Get<double>("longitude_deg",
        this->dataPtr->longitude);
  if (!lonPair.second)
  {
    std::cerr << "Missing <longitude> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->dataPtr->longitude << std::endl;
    result = false;
  }
  this->dataPtr->longitude = lonPair.first;

  // Get the elevation
  std::pair<double, bool> elevationPair = _sdf->Get<double>("elevation",
        this->dataPtr->elevation);
  if (!elevationPair.second)
  {
    std::cerr << "Missing <elevation> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->dataPtr->elevation << std::endl;
    result = false;
  }
  this->dataPtr->elevation = elevationPair.first;

  // Get the heading
  std::pair<double, bool> headingPair = _sdf->Get<double>("heading",
        this->dataPtr->heading);
  if (!headingPair.second)
  {
    std::cerr << "Missing <heading> element, "
      << "child of <spherical_coordinates>. Using default value of "
      << this->dataPtr->heading << std::endl;
    result = false;
  }
  this->dataPtr->heading = headingPair.first;

  return result;
}

/////////////////////////////////////////////////
std::string SphericalCoordinates::SurfaceModel()
{
  return this->dataPtr->surfaceModel;
}

/////////////////////////////////////////////////
std::string SphericalCoordinates::WorldFrameOrientation()
{
  return this->dataPtr->worldFrameOrientation;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Latitude()
{
  return this->dataPtr->latitude;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Longitude()
{
  return this->dataPtr->longitude;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Elevation()
{
  return this->dataPtr->elevation;
}

/////////////////////////////////////////////////
double SphericalCoordinates::Heading()
{
  return this->dataPtr->heading;
}
