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
#include <string>
#include "sdf/ForceTorque.hh"

using namespace sdf;

/// \brief Private force torque data.
class sdf::ForceTorque::Implementation
{
  /// \brief Noise values related to the body-frame force on the
  /// X-axis.
  public: Noise forceXNoise;

  /// \brief Noise values related to the body-frame force on the
  /// Y-axis.
  public: Noise forceYNoise;

  /// \brief Noise values related to the body-frame force on the
  /// Z-axis.
  public: Noise forceZNoise;

  /// \brief Noise values related to the body-frame torque on the
  /// X-axis.
  public: Noise torqueXNoise;

  /// \brief Noise values related to the body-frame torque on the
  /// Y-axis.
  public: Noise torqueYNoise;

  /// \brief Noise values related to the body-frame torque on the
  /// Z-axis.
  public: Noise torqueZNoise;

  /// \brief Name of the reference frame for the wrench values.
  public: ForceTorqueFrame frame = ForceTorqueFrame::CHILD;

  /// \brief The direction of the measured wrench values.
  public: ForceTorqueMeasureDirection measure_direction =
      ForceTorqueMeasureDirection::CHILD_TO_PARENT;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
ForceTorque::ForceTorque()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors ForceTorque::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <force_torque> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "force_torque")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a force torque sensor, but the provided SDF "
        "element is not a <force_torque>."});
    return errors;
  }

  if (_sdf->HasElement("frame"))
  {
    std::string frame = _sdf->Get<std::string>("frame", "child").first;

    if (frame == "parent")
    {
      this->dataPtr->frame = ForceTorqueFrame::PARENT;
    }
    else if (frame == "child")
    {
      this->dataPtr->frame = ForceTorqueFrame::CHILD;
    }
    else if (frame == "sensor")
    {
      this->dataPtr->frame = ForceTorqueFrame::SENSOR;
    }
    else
    {
      this->dataPtr->frame = ForceTorqueFrame::INVALID;
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "ForceTorque element 'frame' is invalid with a value of [" + frame +
          "]. Refer to the SDF documentation for the list of valid frames"});
    }
  }

  if (_sdf->HasElement("measure_direction"))
  {
    std::string direction =
        _sdf->Get<std::string>("measure_direction", "child_to_parent").first;

    if (direction == "parent_to_child")
    {
      this->dataPtr->measure_direction =
        ForceTorqueMeasureDirection::PARENT_TO_CHILD;
    }
    else if (direction == "child_to_parent")
    {
      this->dataPtr->measure_direction =
        ForceTorqueMeasureDirection::CHILD_TO_PARENT;
    }
    else
    {
      this->dataPtr->measure_direction = ForceTorqueMeasureDirection::INVALID;
      errors.push_back({ErrorCode::ELEMENT_INVALID,
          "ForceTorque element 'measure_direction' is invalid with a value "
          "of [" + direction + "]. Refer to the SDF documentation for the "
          "list of valid frames"});
    }
  }

  auto loadAxisNoise = [&errors](sdf::ElementPtr _parent,
                          const std::string _groupLabel,
                          const std::string _axisLabel,
                          sdf::Noise& _noise)
  {
    if (_parent->HasElement(_groupLabel) &&
        _parent->GetElement(_groupLabel)->HasElement(_axisLabel))
    {
        auto axis = _parent->GetElement(_groupLabel)->GetElement(_axisLabel);
        sdf::Errors noiseErrors = _noise.Load(axis->GetElement("noise"));
        errors.insert(errors.end(), noiseErrors.begin(), noiseErrors.end());
        return true;
    }
    return false;
  };

  loadAxisNoise(_sdf, "force", "x", this->dataPtr->forceXNoise);
  loadAxisNoise(_sdf, "force", "y", this->dataPtr->forceYNoise);
  loadAxisNoise(_sdf, "force", "z", this->dataPtr->forceZNoise);
  loadAxisNoise(_sdf, "torque", "x", this->dataPtr->torqueXNoise);
  loadAxisNoise(_sdf, "torque", "y", this->dataPtr->torqueYNoise);
  loadAxisNoise(_sdf, "torque", "z", this->dataPtr->torqueZNoise);

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr ForceTorque::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool ForceTorque::operator!=(const ForceTorque &_ft) const
{
  return !(*this == _ft);
}

//////////////////////////////////////////////////
bool ForceTorque::operator==(const ForceTorque &_ft) const
{
  return this->dataPtr->frame == _ft.dataPtr->frame &&
         this->dataPtr->measure_direction == _ft.dataPtr->measure_direction &&
         this->dataPtr->forceXNoise == _ft.dataPtr->forceXNoise &&
         this->dataPtr->forceYNoise == _ft.dataPtr->forceYNoise &&
         this->dataPtr->forceZNoise == _ft.dataPtr->forceZNoise &&
         this->dataPtr->torqueXNoise == _ft.dataPtr->torqueXNoise &&
         this->dataPtr->torqueYNoise == _ft.dataPtr->torqueYNoise &&
         this->dataPtr->torqueZNoise == _ft.dataPtr->torqueZNoise;
}

//////////////////////////////////////////////////
const Noise &ForceTorque::ForceXNoise() const
{
  return this->dataPtr->forceXNoise;
}

//////////////////////////////////////////////////
void ForceTorque::SetForceXNoise(const Noise &_noise)
{
  this->dataPtr->forceXNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &ForceTorque::ForceYNoise() const
{
  return this->dataPtr->forceYNoise;
}

//////////////////////////////////////////////////
void ForceTorque::SetForceYNoise(const Noise &_noise)
{
  this->dataPtr->forceYNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &ForceTorque::ForceZNoise() const
{
  return this->dataPtr->forceZNoise;
}

//////////////////////////////////////////////////
void ForceTorque::SetForceZNoise(const Noise &_noise)
{
  this->dataPtr->forceZNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &ForceTorque::TorqueXNoise() const
{
  return this->dataPtr->torqueXNoise;
}

//////////////////////////////////////////////////
void ForceTorque::SetTorqueXNoise(const Noise &_noise)
{
  this->dataPtr->torqueXNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &ForceTorque::TorqueYNoise() const
{
  return this->dataPtr->torqueYNoise;
}

//////////////////////////////////////////////////
void ForceTorque::SetTorqueYNoise(const Noise &_noise)
{
  this->dataPtr->torqueYNoise = _noise;
}

//////////////////////////////////////////////////
const Noise &ForceTorque::TorqueZNoise() const
{
  return this->dataPtr->torqueZNoise;
}

//////////////////////////////////////////////////
void ForceTorque::SetTorqueZNoise(const Noise &_noise)
{
  this->dataPtr->torqueZNoise = _noise;
}

//////////////////////////////////////////////////
ForceTorqueFrame ForceTorque::Frame() const
{
  return this->dataPtr->frame;
}

//////////////////////////////////////////////////
void ForceTorque::SetFrame(ForceTorqueFrame _frame)
{
  this->dataPtr->frame = _frame;
}

//////////////////////////////////////////////////
ForceTorqueMeasureDirection ForceTorque::MeasureDirection() const
{
  return this->dataPtr->measure_direction;
}

//////////////////////////////////////////////////
void ForceTorque::SetMeasureDirection(
    ForceTorqueMeasureDirection _direction)
{
  this->dataPtr->measure_direction = _direction;
}
