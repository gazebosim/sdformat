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
#include "sdf/Collision.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::CollisionPrivate
{
  /// \brief Name of the collision.
  public: std::string name = "";

  /// \brief The pose of the collision, as specified in SDF.
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief The frame of the pose
  public: std::string poseFrame = "";

  public: std::shared_ptr<FrameGraph> frameGraph = nullptr;
};

/////////////////////////////////////////////////
Collision::Collision()
  : dataPtr(new CollisionPrivate)
{
}

/////////////////////////////////////////////////
Collision::Collision(Collision &&_collision)
{
  this->dataPtr = _collision.dataPtr;
  _collision.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Collision::~Collision()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Collision::Load(ElementPtr _sdf, std::shared_ptr<FrameGraph> _frameGraph)
{
  Errors errors;

  // Check that the provided SDF element is a <collision>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "collision")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Collision, but the provided SDF element is not a "
        "<collision>."});
    return errors;
  }

  // Read the collisions's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A collision name is required, but the name is not set."});
  }

  // Load the pose. Ignore the return value because the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseFrame);

  if (_frameGraph)
  {
    _frameGraph->AddVertex(this->dataPtr->name,
        ignition::math::Matrix4d(this->dataPtr->pose));
    this->dataPtr->frameGraph = _frameGraph;
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Collision::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Collision::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}
