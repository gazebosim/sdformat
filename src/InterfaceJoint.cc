/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "sdf/InterfaceJoint.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class InterfaceJoint::Implementation
{
  /// \brief Name of this interface joint.
  public: std::string name;

  /// \brief Name of this joint's child frame.
  public: std::string childName;

  /// \brief Pose of this joint relative to the child frame.
  public: ignition::math::Pose3d pose;
};

InterfaceJoint::InterfaceJoint(const std::string &_name,
    const std::string &_childName, const ignition::math::Pose3d &_pose)
    : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  this->dataPtr->name = _name;
  this->dataPtr->childName = _childName;
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
const std::string &InterfaceJoint::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
const std::string &InterfaceJoint::ChildName() const
{
  return this->dataPtr->childName;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &InterfaceJoint::PoseInChildFrame() const
{
  return this->dataPtr->pose;
}
}
}

