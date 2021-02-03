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
         this->dataPtr->measure_direction == _ft.dataPtr->measure_direction;
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
