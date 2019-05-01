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
  /// \brief Noise values related to the body-frame linear acceleration on the
  /// X-axis.
  public: Noise linearAccelNoiseX;

  /// \brief Noise values related to the body-frame linear acceleration on the
  /// Y-axis.
  public: Noise linearAccelNoiseY;

  /// \brief Noise values related to the body-frame linear acceleration on the
  /// Z-axis.
  public: Noise linearAccelNoiseZ;

  /// \brief Noise values related to the body-frame angular velocity on the
  /// X-axis.
  public: Noise angularVelNoiseX;

  /// \brief Noise values related to the body-frame angular velocity on the
  /// Y-axis.
  public: Noise angularVelNoiseY;

  /// \brief Noise values related to the body-frame angular velocity on the
  /// Z-axis.
  public: Noise angularVelNoiseZ;

  /// \brief The gravity dir
  public: ignition::math::Vector3d gravityDirX{ignition::math::Vector3d::UnitX};

  /// \brief Name of the parent frame for the gravityDirX vector.
  public: std::string gravityDirXParentFrame;

  /// \brief Localization string
  public: std::string localization = "CUSTOM";

  /// \brief Custom RPY
  public: ignition::math::Vector3d customRpy;

  /// \brief Name of the parent frame for the customRpy vector.
  public: std::string customRpyParentFrame;

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

  // Load the linear acceleration noise values.
  if (_sdf->HasElement("linear_acceleration"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("linear_acceleration");
    if (elem->HasElement("x"))
    {
      if (elem->GetElement("x")->HasElement("noise"))
      {
        this->dataPtr->linearAccelNoiseX.Load(
            elem->GetElement("x")->GetElement("noise"));
      }
    }

    if (elem->HasElement("y"))
    {
      if (elem->GetElement("y")->HasElement("noise"))
      {
        this->dataPtr->linearAccelNoiseY.Load(
            elem->GetElement("y")->GetElement("noise"));
      }
    }

    if (elem->HasElement("z"))
    {
      if (elem->GetElement("z")->HasElement("noise"))
      {
        this->dataPtr->linearAccelNoiseZ.Load(
            elem->GetElement("z")->GetElement("noise"));
      }
    }
  }

  // Load the angular velocity noise values.
  if (_sdf->HasElement("angular_velocity"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("angular_velocity");
    if (elem->HasElement("x"))
    {
      if (elem->GetElement("x")->HasElement("noise"))
      {
        this->dataPtr->angularVelNoiseX.Load(
            elem->GetElement("x")->GetElement("noise"));
      }
    }

    if (elem->HasElement("y"))
    {
      if (elem->GetElement("y")->HasElement("noise"))
      {
        this->dataPtr->angularVelNoiseY.Load(
            elem->GetElement("y")->GetElement("noise"));
      }
    }

    if (elem->HasElement("z"))
    {
      if (elem->GetElement("z")->HasElement("noise"))
      {
        this->dataPtr->angularVelNoiseZ.Load(
            elem->GetElement("z")->GetElement("noise"));
      }
    }
  }

  if (_sdf->HasElement("orientation_reference_frame"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("orientation_reference_frame");
    this->dataPtr->localization = elem->Get<std::string>("localization",
        this->dataPtr->localization).first;

    if (elem->HasElement("grav_dir_x"))
    {
      this->dataPtr->gravityDirX = elem->Get<ignition::math::Vector3d>(
          "grav_dir_x", this->dataPtr->gravityDirX).first;
      this->dataPtr->gravityDirXParentFrame =
        elem->GetElement("grav_dir_x")->Get<std::string>("parent_frame",
            this->dataPtr->gravityDirXParentFrame).first;
    }

    if (elem->HasElement("custom_rpy"))
    {
      this->dataPtr->customRpy = elem->Get<ignition::math::Vector3d>(
          "custom_rpy", this->dataPtr->customRpy).first;
      this->dataPtr->customRpyParentFrame =
        elem->GetElement("custom_rpy")->Get<std::string>("parent_frame",
            this->dataPtr->customRpyParentFrame).first;
    }
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
  return this->dataPtr->linearAccelNoiseX == _imu.dataPtr->linearAccelNoiseX &&
         this->dataPtr->linearAccelNoiseY == _imu.dataPtr->linearAccelNoiseY &&
         this->dataPtr->linearAccelNoiseZ == _imu.dataPtr->linearAccelNoiseZ &&
         this->dataPtr->angularVelNoiseX == _imu.dataPtr->angularVelNoiseX &&
         this->dataPtr->angularVelNoiseY == _imu.dataPtr->angularVelNoiseY &&
         this->dataPtr->angularVelNoiseZ == _imu.dataPtr->angularVelNoiseZ &&
         this->dataPtr->localization == _imu.dataPtr->localization &&

         this->dataPtr->gravityDirX == _imu.dataPtr->gravityDirX &&
         this->dataPtr->gravityDirXParentFrame ==
         _imu.dataPtr->gravityDirXParentFrame &&

         this->dataPtr->customRpy == _imu.dataPtr->customRpy &&
         this->dataPtr->customRpyParentFrame ==
         _imu.dataPtr->customRpyParentFrame;
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
const std::string &Imu::GravityDirXParentFrame() const
{
  return this->dataPtr->gravityDirXParentFrame;
}

//////////////////////////////////////////////////
void Imu::SetGravityDirXParentFrame(const std::string &_frame) const
{
  this->dataPtr->gravityDirXParentFrame = _frame;
}

//////////////////////////////////////////////////
void Imu::SetGravityDirX(const ignition::math::Vector3d &_grav) const
{
  this->dataPtr->gravityDirX = _grav;
}

//////////////////////////////////////////////////
const std::string &Imu::Localization() const
{
  return this->dataPtr->localization;
}

//////////////////////////////////////////////////
void Imu::SetLocalization(const std::string &_localization)
{
  this->dataPtr->localization = _localization;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d &Imu::CustomRpy() const
{
  return this->dataPtr->customRpy;
}

//////////////////////////////////////////////////
void Imu::SetCustomRpy(
    const ignition::math::Vector3d &_rpy) const
{
  this->dataPtr->customRpy = _rpy;
}

//////////////////////////////////////////////////
const std::string &Imu::CustomRpyParentFrame() const
{
  return this->dataPtr->customRpyParentFrame;
}

//////////////////////////////////////////////////
void Imu::SetCustomRpyParentFrame(const std::string &_frame) const
{
  this->dataPtr->customRpyParentFrame = _frame;
}
