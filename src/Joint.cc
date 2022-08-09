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
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <utility>
#include <ignition/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::JointPrivate
{
  public: JointPrivate()
  {
    // Initialize here because windows does not support list initialization
    // at member initialization (ie ... axis = {{nullptr, nullpter}};).
    this->axis[0] = nullptr;
    this->axis[1] = nullptr;
  }

  /// \brief Copy constructor
  /// \param[in] _jointPrivate JointPrivate to copy.
  public: explicit JointPrivate(const JointPrivate &_jointPrivate);

  // Delete copy assignment so it is not accidentally used
  public: JointPrivate &operator=(const JointPrivate &_) = delete;

  /// \brief Name of the joint.
  public: std::string name = "";

  /// \brief Name of the parent link.
  public: std::string parentLinkName = "";

  /// \brief Name of the child link.
  public: std::string childLinkName = "";

  /// \brief the joint type.
  public: JointType type = JointType::INVALID;

  /// \brief Pose of the joint
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief Thread pitch for screw joints.
  public: double threadPitch = 1.0;

  /// \brief Joint axis
  public: std::array<std::unique_ptr<JointAxis>, 2> axis;

  /// \brief The SDF element pointer used during load.
  public: ElementPtr sdf;

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: std::weak_ptr<const PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief The sensors specified in this joint.
  public: std::vector<Sensor> sensors;
};

/////////////////////////////////////////////////
JointPrivate::JointPrivate(const JointPrivate &_jointPrivate)
    : name(_jointPrivate.name),
      parentLinkName(_jointPrivate.parentLinkName),
      childLinkName(_jointPrivate.childLinkName),
      type(_jointPrivate.type),
      pose(_jointPrivate.pose),
      poseRelativeTo(_jointPrivate.poseRelativeTo),
      threadPitch(_jointPrivate.threadPitch),
      sdf(_jointPrivate.sdf),
      poseRelativeToGraph(_jointPrivate.poseRelativeToGraph)
{
  for (std::size_t i = 0; i < _jointPrivate.axis.size(); ++i)
  {
    if (_jointPrivate.axis[i])
    {
      this->axis[i] = std::make_unique<JointAxis>(*_jointPrivate.axis[i]);
    }
  }
}

/////////////////////////////////////////////////
Joint::Joint()
  : dataPtr(new JointPrivate)
{
}

/////////////////////////////////////////////////
Joint::~Joint()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Joint::Joint(const Joint &_joint)
  : dataPtr(new JointPrivate(*_joint.dataPtr))
{
}

/////////////////////////////////////////////////
Joint::Joint(Joint &&_joint) noexcept
  : dataPtr(std::exchange(_joint.dataPtr, nullptr))
{
}

/////////////////////////////////////////////////
Joint &Joint::operator=(const Joint &_joint)
{
  return *this = Joint(_joint);
}

/////////////////////////////////////////////////
Joint &Joint::operator=(Joint &&_joint)
{
  std::swap(this->dataPtr, _joint.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Errors Joint::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <joint>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "joint")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Joint, but the provided SDF element is not a "
        "<joint>."});
    return errors;
  }

  // Read the joints's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A joint name is required, but the name is not set."});
  }

  // Check that the joint's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied joint name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  // Read the parent link name
  std::pair<std::string, bool> parentPair =
    _sdf->Get<std::string>("parent", "");
  if (parentPair.second)
  {
    this->dataPtr->parentLinkName = parentPair.first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The parent element is missing."});
  }

  // Read the child link name
  std::pair<std::string, bool> childPair = _sdf->Get<std::string>("child", "");
  if (childPair.second)
  {
    this->dataPtr->childLinkName = childPair.first;
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The child element is missing."});
  }

  if (this->dataPtr->childLinkName == "world")
  {
    errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Joint with name[" + this->dataPtr->name +
        "] specified invalid child link [world]."});
  }

  if (this->dataPtr->childLinkName == this->dataPtr->parentLinkName)
  {
    errors.push_back({ErrorCode::JOINT_PARENT_SAME_AS_CHILD,
        "Joint with name[" + this->dataPtr->name +
        "] must specify different link names for "
        "parent and child, while [" + this->dataPtr->childLinkName +
        "] was specified for both."});
  }

  if (_sdf->HasElement("axis"))
  {
    this->dataPtr->axis[0].reset(new JointAxis());
    Errors axisErrors = this->dataPtr->axis[0]->Load(_sdf->GetElement("axis"));
    errors.insert(errors.end(), axisErrors.begin(), axisErrors.end());
  }

  if (_sdf->HasElement("axis2"))
  {
    this->dataPtr->axis[1].reset(new JointAxis());
    Errors axisErrors = this->dataPtr->axis[1]->Load(_sdf->GetElement("axis2"));
    errors.insert(errors.end(), axisErrors.begin(), axisErrors.end());
  }

  this->dataPtr->threadPitch = _sdf->Get<double>("thread_pitch", 1.0).first;

  // Read the type
  std::pair<std::string, bool> typePair = _sdf->Get<std::string>("type", "");
  if (typePair.second)
  {
    typePair.first = lowercase(typePair.first);
    if (typePair.first == "ball")
      this->dataPtr->type = JointType::BALL;
    else if (typePair.first == "continuous")
      this->dataPtr->type = JointType::CONTINUOUS;
    else if (typePair.first == "fixed")
      this->dataPtr->type = JointType::FIXED;
    else if (typePair.first == "gearbox")
      this->dataPtr->type = JointType::GEARBOX;
    else if (typePair.first == "prismatic")
      this->dataPtr->type = JointType::PRISMATIC;
    else if (typePair.first == "revolute")
      this->dataPtr->type = JointType::REVOLUTE;
    else if (typePair.first == "revolute2")
      this->dataPtr->type = JointType::REVOLUTE2;
    else if (typePair.first == "screw")
      this->dataPtr->type = JointType::SCREW;
    else if (typePair.first == "universal")
      this->dataPtr->type = JointType::UNIVERSAL;
    else
    {
      this->dataPtr->type = JointType::INVALID;
      errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
          "Joint type of " + typePair.first +
          " is invalid. Refer to the SDF documentation for a list of "
          "valid joint types"});
    }
  }
  else
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "A joint type is required, but is not set."});
  }

  // Load all the sensors.
  Errors sensorLoadErrors = loadUniqueRepeated<Sensor>(_sdf, "sensor",
      this->dataPtr->sensors);
  errors.insert(errors.end(), sensorLoadErrors.begin(), sensorLoadErrors.end());
  return errors;
}

/////////////////////////////////////////////////
const std::string &Joint::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Joint::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
JointType Joint::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Joint::SetType(const JointType _jointType)
{
  this->dataPtr->type = _jointType;
}

/////////////////////////////////////////////////
const std::string &Joint::ParentLinkName() const
{
  return this->dataPtr->parentLinkName;
}

/////////////////////////////////////////////////
void Joint::SetParentLinkName(const std::string &_name)
{
  this->dataPtr->parentLinkName = _name;
}

/////////////////////////////////////////////////
const std::string &Joint::ChildLinkName() const
{
  return this->dataPtr->childLinkName;
}

/////////////////////////////////////////////////
void Joint::SetChildLinkName(const std::string &_name)
{
  this->dataPtr->childLinkName = _name;
}

/////////////////////////////////////////////////
const JointAxis *Joint::Axis(const unsigned int _index) const
{
  return this->dataPtr->axis[std::min(_index, 1u)].get();
}

/////////////////////////////////////////////////
void Joint::SetAxis(const unsigned int _index, const JointAxis &_axis)
{
  this->dataPtr->axis[std::min(_index, 1u)] =
      std::make_unique<JointAxis>(_axis);
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Joint::Pose() const
{
  return this->RawPose();
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Joint::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Joint::PoseFrame() const
{
  return this->PoseRelativeTo();
}

/////////////////////////////////////////////////
const std::string &Joint::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Joint::SetPose(const ignition::math::Pose3d &_pose)
{
  this->SetRawPose(_pose);
}

/////////////////////////////////////////////////
void Joint::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Joint::SetPoseFrame(const std::string &_frame)
{
  this->SetPoseRelativeTo(_frame);
}

/////////////////////////////////////////////////
void Joint::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Joint::SetPoseRelativeToGraph(
    std::weak_ptr<const PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;

  for (auto& axis : this->dataPtr->axis)
  {
    if (axis)
    {
      axis->SetXmlParentName(this->dataPtr->name);
      axis->SetPoseRelativeToGraph(this->dataPtr->poseRelativeToGraph);
    }
  }
}

/////////////////////////////////////////////////
SemanticPose Joint::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->ChildLinkName(),
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
double Joint::ThreadPitch() const
{
  return this->dataPtr->threadPitch;
}

/////////////////////////////////////////////////
void Joint::SetThreadPitch(double _threadPitch)
{
  this->dataPtr->threadPitch = _threadPitch;
}

/////////////////////////////////////////////////
ElementPtr Joint::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
uint64_t Joint::SensorCount() const
{
  return this->dataPtr->sensors.size();
}

/////////////////////////////////////////////////
const Sensor *Joint::SensorByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->sensors.size())
    return &this->dataPtr->sensors[_index];
  return nullptr;
}

/////////////////////////////////////////////////
bool Joint::SensorNameExists(const std::string &_name) const
{
  return nullptr != this->SensorByName(_name);
}

/////////////////////////////////////////////////
const Sensor *Joint::SensorByName(const std::string &_name) const
{
  for (auto const &s : this->dataPtr->sensors)
  {
    if (s.Name() == _name)
    {
      return &s;
    }
  }
  return nullptr;
}
