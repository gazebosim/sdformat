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
#include <gz/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/parser.hh"
#include "sdf/Sensor.hh"
#include "sdf/Types.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Joint::Implementation
{
  /// \brief Name of the joint.
  public: std::string name = "";

  /// \brief Name of the parent frame.
  public: std::string parentName = "";

  /// \brief Name of the child frame.
  public: std::string childName = "";

  /// \brief the joint type.
  public: JointType type = JointType::INVALID;

  /// \brief Pose of the joint
  public: gz::math::Pose3d pose = gz::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief Thread pitch for screw joints in meters / revolution with a
  /// positive value for right-handed threads.
  public: double threadPitch = 1.0;

  /// \brief Joint axis
  public: std::array<std::optional<JointAxis>, 2> axis;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief Scoped Frame Attached-To graph at the parent model scope
  public: sdf::ScopedGraph<sdf::FrameAttachedToGraph> frameAttachedToGraph;

  /// \brief Scoped Pose Relative-To graph at the parent model scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief The sensors specified in this joint.
  public: std::vector<Sensor> sensors;
};

/////////////////////////////////////////////////
Joint::Joint()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
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
    this->dataPtr->parentName = parentPair.first;
    if (!isValidFrameReference(this->dataPtr->parentName))
    {
      errors.push_back({ErrorCode::RESERVED_NAME,
          "The supplied joint parent name [" + this->dataPtr->parentName +
              "] is not valid."});
    }
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
    this->dataPtr->childName = childPair.first;
    if (!isValidFrameReference(this->dataPtr->childName))
    {
      errors.push_back({ErrorCode::RESERVED_NAME,
          "The supplied joint child name [" + this->dataPtr->childName +
              "] is not valid."});
    }
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "The child element is missing."});
  }

  if (this->dataPtr->childName == "world")
  {
    errors.push_back({ErrorCode::JOINT_CHILD_LINK_INVALID,
        "Joint with name[" + this->dataPtr->name +
        "] specified invalid child link [world]."});
  }

  if (this->dataPtr->childName == this->dataPtr->parentName)
  {
    errors.push_back({ErrorCode::JOINT_PARENT_SAME_AS_CHILD,
        "Joint with name[" + this->dataPtr->name +
        "] must specify different frame names for "
        "parent and child, while [" + this->dataPtr->childName +
        "] was specified for both."});
  }

  // Map of follower axis name to dataPtr->axis[] array index
  // The keys of this map are also the only valid leader axis names
  const std::map<std::string, std::size_t>
      followerAxisNames = {{"axis", 0}, {"axis2", 1}};
  for (const auto &[followerAxis, i] : followerAxisNames)
  {
    if (_sdf->HasElement(followerAxis))
    {
      auto &axis = this->dataPtr->axis[i].emplace();
      Errors axisErrors = axis.Load(_sdf->GetElement(followerAxis));
      errors.insert(errors.end(), axisErrors.begin(), axisErrors.end());

      if (axis.Mimic())
      {
        const auto leaderAxis = axis.Mimic()->Axis();
        if (axis.Mimic()->Joint() == this->Name() &&
            leaderAxis == followerAxis)
        {
          errors.push_back({ErrorCode::JOINT_AXIS_MIMIC_INVALID,
            "Axis with name [" + followerAxis + "] in " +
            "joint with name [" + this->dataPtr->name +
            "] cannot mimic itself."});
        }
        if (followerAxisNames.find(leaderAxis) == followerAxisNames.end())
        {
          errors.push_back({ErrorCode::JOINT_AXIS_MIMIC_INVALID,
            "Axis with name [" + followerAxis + "] in " +
            "joint with name [" + this->dataPtr->name +
            "] specified an invalid leader axis name [" + leaderAxis + "]."});
        }
      }
    }
  }

  if (_sdf->HasElement("screw_thread_pitch"))
  {
    // Prefer the screw_thread_pitch parameter if available.
    this->dataPtr->threadPitch = _sdf->Get<double>("screw_thread_pitch");
  }
  else if (_sdf->HasElement("thread_pitch"))
  {
    // If thread_pitch is available, convert to meters / revolution
    // and fix sign.
    this->dataPtr->threadPitch = -2*GZ_PI / _sdf->Get<double>("thread_pitch");
  }
  // Otherwise the default value of threadPitch will be used

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
const std::string &Joint::ParentName() const
{
  return this->dataPtr->parentName;
}

/////////////////////////////////////////////////
void Joint::SetParentName(const std::string &_name)
{
  this->dataPtr->parentName = _name;
}

/////////////////////////////////////////////////
const std::string &Joint::ChildName() const
{
  return this->dataPtr->childName;
}

/////////////////////////////////////////////////
void Joint::SetChildName(const std::string &_name)
{
  this->dataPtr->childName = _name;
}

/////////////////////////////////////////////////
const std::string &Joint::ParentLinkName() const
{
  return this->ParentName();
}

/////////////////////////////////////////////////
void Joint::SetParentLinkName(const std::string &_name)
{
  this->SetParentName(_name);
}

/////////////////////////////////////////////////
const std::string &Joint::ChildLinkName() const
{
  return this->ChildName();
}

/////////////////////////////////////////////////
void Joint::SetChildLinkName(const std::string &_name)
{
  this->SetChildName(_name);
}

/////////////////////////////////////////////////
const JointAxis *Joint::Axis(const unsigned int _index) const
{
  return optionalToPointer(this->dataPtr->axis[std::min(_index, 1u)]);
}

/////////////////////////////////////////////////
void Joint::SetAxis(const unsigned int _index, const JointAxis &_axis)
{
  this->dataPtr->axis[std::min(_index, 1u)] = _axis;
}

/////////////////////////////////////////////////
const gz::math::Pose3d &Joint::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Joint::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Joint::SetRawPose(const gz::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Joint::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Joint::SetFrameAttachedToGraph(
    sdf::ScopedGraph<FrameAttachedToGraph> _graph)
{
  this->dataPtr->frameAttachedToGraph = _graph;
}

/////////////////////////////////////////////////
void Joint::SetPoseRelativeToGraph(
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
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
  for (auto &sensor : this->dataPtr->sensors)
  {
    sensor.SetXmlParentName(this->dataPtr->name);
    sensor.SetPoseRelativeToGraph(_graph);
  }
}

/////////////////////////////////////////////////
Errors Joint::ResolveChildLink(std::string &_link) const
{
  Errors errors;

  auto graph = this->dataPtr->frameAttachedToGraph;
  if (!graph)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Frame has invalid pointer to FrameAttachedToGraph."});
    return errors;
  }

  std::string link;
  errors = resolveFrameAttachedToBody(link, graph, this->ChildName());
  if (errors.empty())
  {
    _link = link;
  }
  return errors;
}

/////////////////////////////////////////////////
Errors Joint::ResolveParentLink(std::string &_link) const
{
  Errors errors;

  // special case for world, return without resolving since it's not in a
  // model's FrameAttachedToGraph
  if ("world" == this->ParentName())
  {
    _link = "world";
    return errors;
  }

  auto graph = this->dataPtr->frameAttachedToGraph;
  if (!graph)
  {
    errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Frame has invalid pointer to FrameAttachedToGraph."});
    return errors;
  }

  std::string link;
  errors = resolveFrameAttachedToBody(link, graph, this->ParentName());
  if (errors.empty())
  {
    _link = link;
  }
  return errors;
}

/////////////////////////////////////////////////
sdf::SemanticPose Joint::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->name,
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->ChildName(),
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
double Joint::ScrewThreadPitch() const
{
  return this->dataPtr->threadPitch;
}

/////////////////////////////////////////////////
void Joint::SetScrewThreadPitch(double _threadPitch)
{
  this->dataPtr->threadPitch = _threadPitch;
}

/////////////////////////////////////////////////
double Joint::ThreadPitch() const
{
  return -2*GZ_PI / this->dataPtr->threadPitch;
}

/////////////////////////////////////////////////
void Joint::SetThreadPitch(double _threadPitch)
{
  this->dataPtr->threadPitch = -2*GZ_PI / _threadPitch;
}

/////////////////////////////////////////////////
sdf::ElementPtr Joint::Element() const
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
Sensor *Joint::SensorByIndex(const uint64_t _index)
{
  return const_cast<Sensor*>(
      static_cast<const Joint*>(this)->SensorByIndex(_index));
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

/////////////////////////////////////////////////
Sensor *Joint::SensorByName(const std::string &_name)
{
  return const_cast<Sensor*>(
      static_cast<const Joint*>(this)->SensorByName(_name));
}

/////////////////////////////////////////////////
sdf::ElementPtr Joint::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("joint.sdf", elem);

  elem->GetAttribute("name")->Set<std::string>(this->Name());
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<gz::math::Pose3d>(this->RawPose());

  std::string jointType = "invalid";
  switch (this->Type())
  {
    case JointType::BALL:
      jointType = "ball";
      break;
    case JointType::CONTINUOUS:
      jointType = "continuous";
      break;
    case JointType::FIXED:
      jointType = "fixed";
      break;
    case JointType::PRISMATIC:
      jointType = "prismatic";
      break;
    case JointType::GEARBOX:
      jointType = "gearbox";
      break;
    case JointType::REVOLUTE:
      jointType = "revolute";
      break;
    case JointType::REVOLUTE2:
      jointType = "revolute2";
      break;
    case JointType::SCREW:
      jointType = "screw";
      break;
    case JointType::UNIVERSAL:
      jointType = "universal";
      break;
    default:
      break;
  }

  elem->GetAttribute("type")->Set<std::string>(jointType);
  elem->GetElement("parent")->Set<std::string>(this->ParentName());
  elem->GetElement("child")->Set<std::string>(this->ChildName());
  for (unsigned int i = 0u; i < 2u; ++i)
  {
    const JointAxis *axis =  this->Axis(i);
    if (!axis)
      break;

    std::string axisElemName = "axis";
    if (i > 0u)
      axisElemName += std::to_string(i+1);
    sdf::ElementPtr axisElem = elem->GetElement(axisElemName);
    axisElem->Copy(axis->ToElement(i));
  }

  for (uint64_t i = 0u; i < this->SensorCount(); ++i)
  {
    const Sensor *sensor = this->SensorByIndex(i);
    if (!sensor)
      continue;
    sdf::ElementPtr sensorElem = elem->GetElement("sensor");
    sensorElem->Copy(sensor->ToElement());
  }

  if (this->Type() == JointType::SCREW)
    elem->GetElement("thread_pitch")->Set<double>(this->ThreadPitch());

  // gearbox_ratio, gearbox_reference_box, and physcs elements are not yet
  // supported.
  return elem;
}

//////////////////////////////////////////////////
bool Joint::AddSensor(const Sensor &_sensor)
{
  if (this->SensorNameExists(_sensor.Name()))
    return false;
  this->dataPtr->sensors.push_back(_sensor);
  return true;
}

//////////////////////////////////////////////////
void Joint::ClearSensors()
{
  this->dataPtr->sensors.clear();
}
