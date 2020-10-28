/*
 * Copyright 2020 Open Source Robotics Foundation
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
#include "sdf/Sky.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Sky private data.
class sdf::SkyPrivate
{
  /// \brief Time of day
  public: double time = 10.0;

  /// \brief Sunrise time
  public: double sunrise = 6.0;

  /// \brief Sunset time
  public: double sunset = 20.0;

  /// \brief Cloud speed
  public: double cloudSpeed = 0.6;

  /// \brief Cloud direction.
  public: ignition::math::Angle cloudDirection;

  /// \brief Cloud humidity
  public: double cloudHumidity = 0.5;

  /// \brief Cloud mean size
  public: double cloudMeanSize = 0.5;

  /// \brief Cloud ambient color
  public: ignition::math::Color cloudAmbient =
      ignition::math::Color(0.8f, 0.8f, 0.8f);

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Sky::Sky()
  : dataPtr(new SkyPrivate)
{
}

/////////////////////////////////////////////////
Sky::~Sky()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Sky::Sky(const Sky &_sky)
  : dataPtr(new SkyPrivate(*_sky.dataPtr))
{
}

/////////////////////////////////////////////////
Sky::Sky(Sky &&_sky) noexcept
  : dataPtr(std::exchange(_sky.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Sky &Sky::operator=(const Sky &_sky)
{
  return *this = Sky(_sky);
}

/////////////////////////////////////////////////
Sky &Sky::operator=(Sky &&_sky)
{
  std::swap(this->dataPtr, _sky.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
double Sky::Time() const
{
  return this->dataPtr->time;
}

/////////////////////////////////////////////////
void Sky::SetTime(double _time)
{
  this->dataPtr->time = _time;
}

/////////////////////////////////////////////////
double Sky::Sunrise() const
{
  return this->dataPtr->sunrise;
}

/////////////////////////////////////////////////
void Sky::SetSunrise(double _sunrise)
{
  this->dataPtr->sunrise = _sunrise;
}

/////////////////////////////////////////////////
double Sky::Sunset() const
{
  return this->dataPtr->sunset;
}

/////////////////////////////////////////////////
void Sky::SetSunset(double _sunset)
{
  this->dataPtr->sunset = _sunset;
}

/////////////////////////////////////////////////
double Sky::CloudSpeed() const
{
  return this->dataPtr->cloudSpeed;
}

/////////////////////////////////////////////////
void Sky::SetCloudSpeed(double _speed)
{
  this->dataPtr->cloudSpeed = _speed;
}

/////////////////////////////////////////////////
ignition::math::Angle Sky::CloudDirection() const
{
  return this->dataPtr->cloudDirection;
}

/////////////////////////////////////////////////
void Sky::SetCloudDirection(const ignition::math::Angle &_angle)
{
  this->dataPtr->cloudDirection = _angle;
}

/////////////////////////////////////////////////
double Sky::CloudHumidity() const
{
  return this->dataPtr->cloudHumidity;
}

/////////////////////////////////////////////////
void Sky::SetCloudHumidity(double _humidity)
{
  this->dataPtr->cloudHumidity = _humidity;
}

/////////////////////////////////////////////////
double Sky::CloudMeanSize() const
{
  return this->dataPtr->cloudMeanSize;
}

/////////////////////////////////////////////////
void Sky::SetCloudMeanSize(double _size)
{
  this->dataPtr->cloudMeanSize = _size;
}

/////////////////////////////////////////////////
ignition::math::Color Sky::CloudAmbient() const
{
  return this->dataPtr->cloudAmbient;
}

/////////////////////////////////////////////////
void Sky::SetCloudAmbient(const ignition::math::Color &_ambient)
{
  this->dataPtr->cloudAmbient = _ambient;
}

/////////////////////////////////////////////////
Errors Sky::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <sky> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "sky")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Sky, but the provided SDF element is not a "
        "<sky>."});
    return errors;
  }

  this->dataPtr->time = _sdf->Get<double>("time", this->dataPtr->time).first;
  this->dataPtr->sunrise =
      _sdf->Get<double>("sunrise", this->dataPtr->sunrise).first;
  this->dataPtr->sunset =
      _sdf->Get<double>("sunset", this->dataPtr->sunset).first;

  if ( _sdf->HasElement("clouds"))
  {
    sdf::ElementPtr cloudElem = _sdf->GetElement("clouds");
    this->dataPtr->cloudSpeed =
        cloudElem->Get<double>("speed", this->dataPtr->cloudSpeed).first;
    this->dataPtr->cloudDirection =
        cloudElem->Get<ignition::math::Angle>("direction",
        this->dataPtr->cloudDirection).first;
    this->dataPtr->cloudHumidity =
        cloudElem->Get<double>("humidity", this->dataPtr->cloudHumidity).first;
    this->dataPtr->cloudMeanSize =
        cloudElem->Get<double>("mean_size", this->dataPtr->cloudMeanSize).first;
    this->dataPtr->cloudAmbient =
        cloudElem->Get<ignition::math::Color>("ambient",
        this->dataPtr->cloudAmbient).first;
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Sky::Element() const
{
  return this->dataPtr->sdf;
}
