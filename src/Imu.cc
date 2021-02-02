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
class sdf::Imu::Implementation
{
  /// \brief Noise values related to the body-frame linear acceleration on the
  /// X-axis.
  public: Noise linearAccelXNoise;

  /// \brief Noise values related to the body-frame linear acceleration on the
  /// Y-axis.
  public: Noise linearAccelYNoise;

  /// \brief Noise values related to the body-frame linear acceleration on the
  /// Z-axis.
  public: Noise linearAccelZNoise;

  /// \brief Noise values related to the body-frame angular velocity on the
  /// X-axis.
  public: Noise angularVelXNoise;

  /// \brief Noise values related to the body-frame angular velocity on the
  /// Y-axis.
  public: Noise angularVelYNoise;

  /// \brief Noise values related to the body-frame angular velocity on the
  /// Z-axis.
  public: Noise angularVelZNoise;

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
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
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
        this->dataPtr->linearAccelXNoise.Load(
            elem->GetElement("x")->GetElement("noise"));
      }
    }

    if (elem->HasElement("y"))
    {
      if (elem->GetElement("y")->HasElement("noise"))
      {
        this->dataPtr->linearAccelYNoise.Load(
            elem->GetElement("y")->GetElement("noise"));
      }
    }

    if (elem->HasElement("z"))
    {
      if (elem->GetElement("z")->HasElement("noise"))
      {
        this->dataPtr->linearAccelZNoise.Load(
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
        this->dataPtr->angularVelXNoise.Load(
            elem->GetElement("x")->GetElement("noise"));
      }
    }

    if (elem->HasElement("y"))
    {
      if (elem->GetElement("y")->HasElement("noise"))
      {
        this->dataPtr->angularVelYNoise.Load(
            elem->GetElement("y")->GetElement("noise"));
      }
    }

    if (elem->HasElement("z"))
    {
      if (elem->GetElement("z")->HasElement("noise"))
      {
        this->dataPtr->angularVelZNoise.Load(
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
  return this->dataPtr->linearAccelXNoise == _imu.dataPtr->linearAccelXNoise &&
         this->dataPtr->linearAccelYNoise == _imu.dataPtr->linearAccelYNoise &&
         this->dataPtr->linearAccelZNoise == _imu.dataPtr->linearAccelZNoise &&
         this->dataPtr->angularVelXNoise == _imu.dataPtr->angularVelXNoise &&
         this->dataPtr->angularVelYNoise == _imu.dataPtr->angularVelYNoise &&
         this->dataPtr->angularVelZNoise == _imu.dataPtr->angularVelZNoise &&
         this->dataPtr->localization == _imu.dataPtr->localization &&

         this->dataPtr->gravityDirX == _imu.dataPtr->gravityDirX &&
         this->dataPtr->gravityDirXParentFrame ==
         _imu.dataPtr->gravityDirXParentFrame &&

         this->dataPtr->customRpy == _imu.dataPtr->customRpy &&
         this->dataPtr->customRpyParentFrame ==
         _imu.dataPtr->customRpyParentFrame;
}

//////////////////////////////////////////////////
const Noise &Imu::LinearAccelerationXNoise() const
{
  return this->dataPtr->linearAccelXNoise;
}

//////////////////////////////////////////////////
void Imu::SetLinearAccelerationXNoise(const Noise &_noise)
{
  this->dataPtr->linearAccelXNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::LinearAccelerationYNoise() const
{
  return this->dataPtr->linearAccelYNoise;
}

//////////////////////////////////////////////////
void Imu::SetLinearAccelerationYNoise(const Noise &_noise)
{
  this->dataPtr->linearAccelYNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::LinearAccelerationZNoise() const
{
  return this->dataPtr->linearAccelZNoise;
}

//////////////////////////////////////////////////
void Imu::SetLinearAccelerationZNoise(const Noise &_noise)
{
  this->dataPtr->linearAccelZNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::AngularVelocityXNoise() const
{
  return this->dataPtr->angularVelXNoise;
}

//////////////////////////////////////////////////
void Imu::SetAngularVelocityXNoise(const Noise &_noise)
{
  this->dataPtr->angularVelXNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::AngularVelocityYNoise() const
{
  return this->dataPtr->angularVelYNoise;
}

//////////////////////////////////////////////////
void Imu::SetAngularVelocityYNoise(const Noise &_noise)
{
  this->dataPtr->angularVelYNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &Imu::AngularVelocityZNoise() const
{
  return this->dataPtr->angularVelZNoise;
}

//////////////////////////////////////////////////
void Imu::SetAngularVelocityZNoise(const Noise &_noise)
{
  this->dataPtr->angularVelZNoise = _noise;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d &Imu::GravityDirX() const
{
  return this->dataPtr->gravityDirX;
}

//////////////////////////////////////////////////
const std::string &Imu::GravityDirXParentFrame() const
{
  return this->dataPtr->gravityDirXParentFrame;
}

//////////////////////////////////////////////////
void Imu::SetGravityDirXParentFrame(const std::string &_frame)
{
  this->dataPtr->gravityDirXParentFrame = _frame;
}

//////////////////////////////////////////////////
void Imu::SetGravityDirX(const ignition::math::Vector3d &_grav)
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
void Imu::SetCustomRpy(const ignition::math::Vector3d &_rpy)
{
  this->dataPtr->customRpy = _rpy;
}

//////////////////////////////////////////////////
const std::string &Imu::CustomRpyParentFrame() const
{
  return this->dataPtr->customRpyParentFrame;
}

//////////////////////////////////////////////////
void Imu::SetCustomRpyParentFrame(const std::string &_frame)
{
  this->dataPtr->customRpyParentFrame = _frame;
}
