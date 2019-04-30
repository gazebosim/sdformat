/*
 * Copyright 2019 Open Source Robotics Foundation
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
#include <array>
#include <string>
#include "sdf/Imu.hh"

using namespace sdf;

/// \brief Private imu data.
class sdf::ImuPrivate
{
  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
Imu::Imu()
  : dataPtr(new ImuPrivate)
{
}

//////////////////////////////////////////////////
Imu::Imu(const Imu &_imu)
  : dataPtr(new ImuPrivate(*_imu.dataPtr))
{
}

//////////////////////////////////////////////////
Imu::Imu(Imu &&_imu) noexcept
{
  this->dataPtr = _imu.dataPtr;
  _imu.dataPtr = nullptr;
}

//////////////////////////////////////////////////
Imu::~Imu()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Imu &Imu::operator=(const Imu &_imu)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new ImuPrivate;
  }
  *this->dataPtr = *_imu.dataPtr;
  return *this;
}

//////////////////////////////////////////////////
Imu &Imu::operator=(Imu &&_imu) noexcept
{
  this->dataPtr = _imu.dataPtr;
  _imu.dataPtr = nullptr;
  return *this;
}

//////////////////////////////////////////////////
Errors Imu::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <imu> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "imu")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load an IMU, but the provided SDF element is "
        "not an <imu>."});
    return errors;
  }

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr Imu::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool Imu::operator!=(const Imu &_imu) const
{
  return !(*this == _imu);
}

//////////////////////////////////////////////////
bool Imu::operator==(const Imu &_imu) const
{

  return true;
}

//////////////////////////////////////////////////
const Noise &Imu::LinearAccelerationNoiseX() const
{
  return this->dataPtr->linearAccelNoiseX;
}

//////////////////////////////////////////////////
void Imu::SetLinearAccelerationNoiseX(const Noise &_noise)
{
  this->dataPtr->linearAccelNoiseX = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::LinearAccelerationNoiseY() const
{
  return this->dataPtr->linearAccelNoiseY;
}

//////////////////////////////////////////////////
void Imu::SetLinearAccelerationNoiseY(const Noise &_noise)
{
  this->dataPtr->linearAccelNoiseY = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::LinearAccelerationNoiseZ() const
{
  return this->dataPtr->linearAccelNoiseZ;
}

//////////////////////////////////////////////////
void Imu::SetLinearAccelerationNoiseZ(const Noise &_noise)
{
  this->dataPtr->linearAccelNoiseZ = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::AngularVelocityNoiseX() const
{
  return this->dataPtr->angularVelNoiseX;
}

//////////////////////////////////////////////////
void Imu::SetAngularVelocityNoiseX(const Noise &_noise)
{
  this->dataPtr->angularVelNoiseX = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::AngularVelocityNoiseY() const
{
  return this->dataPtr->angularVelNoiseY;
}

//////////////////////////////////////////////////
void Imu::SetAngularVelocityNoiseY(const Noise &_noise)
{
  this->dataPtr->angularVelNoiseY = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::AngularVelocityNoiseZ() const
{
  return this->dataPtr->angularVelNoiseZ;
}

//////////////////////////////////////////////////
void Imu::SetAngularVelocityNoiseZ(const Noise &_noise)
{
  this->dataPtr->angularVelNoiseZ = _noise;
}

//////////////////////////////////////////////////
ignition::math::Vector3d &Imu::GravityDirX() const
{
  return this->dataPtr->gravityDirX;
}

//////////////////////////////////////////////////
void Imu::SetGravityDirX(const ignition::math::Vector3d &_grav) const
{
  this->dataPtr->gravityDirX = _grav;
}

//////////////////////////////////////////////////
const std::string &Imu::OrientationLocalization() const
{
  return this->dataPtr->localization;
}

//////////////////////////////////////////////////
void Imu::SetOrientationLocalization(const std::string &_localization)
{
  this->dataPtr->localization = _localization;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d &Imu::OrientationCustomRpy() const
{
  return this->dataPtr->customRpy;
}

//////////////////////////////////////////////////
void Imu::SetOrientationCustomRpy(
    const ignition::math::Vector3d &_rpy) const
{
  this->dataPtr->customRpy = _rpy;
}

//////////////////////////////////////////////////
const std::string &Imu::OrientationParentFrame() const
{
  return this->dataPtr->parentFrame;
}

//////////////////////////////////////////////////
void Imu::SetOrientationParentFrame(const std::string &_frame) const
{
  this->dataPtr->parentFrame = _frame;
}
