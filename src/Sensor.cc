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
#include <vector>
#include <ignition/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::SensorPrivate
{
  // \brief The sensor type.
  public: SensorType type = SensorType::NONE;

  /// \brief Name of the sensor.
  public: std::string name = "";

  /// \brief Pose of the sensor
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseFrame = "";

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Sensor::Sensor()
  : dataPtr(new SensorPrivate)
{
}

/////////////////////////////////////////////////
Sensor::Sensor(Sensor &&_sensor)
{
  this->dataPtr = _sensor.dataPtr;
  _sensor.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Sensor::~Sensor()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Sensor::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Sensor, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <sensor>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "sensor")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Sensor, but the provided SDF element is not a "
        "<sensor>."});
    return errors;
  }

  // Read the sensors's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A sensor name is required, but the name is not set."});
    return errors;
  }

  std::string type = _sdf->Get<std::string>("type");
  if (type == "altimeter")
  {
    this->dataPtr->type = SensorType::ALTIMETER;
  }
  else if (type == "camera")
  {
    this->dataPtr->type = SensorType::CAMERA;
  }
  else if (type == "contact")
  {
    this->dataPtr->type = SensorType::CONTACT;
  }
  else if (type == "depth")
  {
    this->dataPtr->type = SensorType::DEPTH;
  }
  else if (type == "force_torque")
  {
    this->dataPtr->type = SensorType::FORCE_TORQUE;
  }
  else if (type == "gps")
  {
    this->dataPtr->type = SensorType::GPS;
  }
  else if (type == "gpu_ray")
  {
    this->dataPtr->type = SensorType::GPU_RAY;
  }
  else if (type == "imu")
  {
    this->dataPtr->type = SensorType::IMU;
  }
  else if (type == "logical_camera")
  {
    this->dataPtr->type = SensorType::LOGICAL_CAMERA;
  }
  else if (type == "magnetometer")
  {
    this->dataPtr->type = SensorType::MAGNETOMETER;
  }
  else if (type == "multicamera")
  {
    this->dataPtr->type = SensorType::MULTICAMERA;
  }
  else if (type == "ray")
  {
    this->dataPtr->type = SensorType::RAY;
  }
  else if (type == "rfid")
  {
    this->dataPtr->type = SensorType::RFID;
  }
  else if (type == "rfidtag")
  {
    this->dataPtr->type = SensorType::RFIDTAG;
  }
  else if (type == "sonar")
  {
    this->dataPtr->type = SensorType::SONAR;
  }
  else if (type == "wireless_receiver")
  {
    this->dataPtr->type = SensorType::WIRELESS_RECEIVER;
  }
  else if (type == "wireless_transmitter")
  {
    this->dataPtr->type = SensorType::WIRELESS_TRANSMITTER;
  }
  else
  {
    errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Attempting to load a Sensor, but the provided sensor type is missing "
        "or invalid."});
    return errors;
  }

  // Load the pose. Ignore the return value since the sensor pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseFrame);

  return errors;
}

/////////////////////////////////////////////////
std::string Sensor::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Sensor::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Sensor::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Sensor::PoseFrame() const
{
  return this->dataPtr->poseFrame;
}

/////////////////////////////////////////////////
void Sensor::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Sensor::SetPoseFrame(const std::string &_frame)
{
  this->dataPtr->poseFrame = _frame;
}

/////////////////////////////////////////////////
sdf::ElementPtr Sensor::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
SensorType Sensor::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Sensor::SetType(const SensorType _type)
{
  this->dataPtr->type = _type;
}
