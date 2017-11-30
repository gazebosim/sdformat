/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include "sdf/dom/Entity.hh"

using namespace sdf;

/// \brief Private data for the Entity
class sdf::EntityPrivate
{
  /// \brief Name of the entity.
  public: std::string name = "";

  /// \brief Pose of the light relative to the entity.
  public: ignition::math::Pose3d pose;

  /// \brief Frame of reference for the entity.
  public: std::string frame;
};

/////////////////////////////////////////////////
Entity::Entity()
  : dataPtr(new EntityPrivate)
{
}

/////////////////////////////////////////////////
Entity::~Entity()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
std::string Entity::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Entity::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
std::string Entity::Frame() const
{
  return this->dataPtr->frame;
}

/////////////////////////////////////////////////
void Entity::SetFrame(const std::string &_frame)
{
  this->dataPtr->frame = _frame;
}

/////////////////////////////////////////////////
ignition::math::Pose3d Entity::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Entity::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
bool Entity::LoadName(sdf::ElementPtr _sdf)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");
  this->dataPtr->name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
bool Entity::LoadPose(sdf::ElementPtr _sdf)
{
  // Read the frame. An empty frame implies the parent frame.
  std::pair<std::string, bool> framePair = _sdf->Get<std::string>("frame", "");

  std::pair<ignition::math::Pose3d, bool> posePair =
    _sdf->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero);

  this->dataPtr->pose = posePair.first;
  this->dataPtr->frame = framePair.first;

  return true;
}
