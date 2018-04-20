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
#include <ignition/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Types.hh"
#include "sdf/Visual.hh"
#include "sdf/Geometry.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::VisualPrivate
{
  /// \brief Name of the visual.
  public: std::string name = "";

  /// \brief Pose of the collision object
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseFrame = "";

  /// \brief The visual's a geometry.
  public: Geometry geom;

  public: std::shared_ptr<FrameGraph> frameGraph = nullptr;
};

/////////////////////////////////////////////////
Visual::Visual()
  : dataPtr(new VisualPrivate)
{
}

/////////////////////////////////////////////////
Visual::Visual(Visual &&_visual)
{
  this->dataPtr = _visual.dataPtr;
  _visual.dataPtr = nullptr;
}

/////////////////////////////////////////////////
Visual::~Visual()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Errors Visual::Load(ElementPtr _sdf, std::shared_ptr<FrameGraph> _frameGraph)
{
  Errors errors;

  // Check that the provided SDF element is a <visual>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "visual")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Visual, but the provided SDF element is not a "
        "<visual>."});
    return errors;
  }

  // Read the visuals's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A visual name is required, but the name is not set."});
  }

  // Load the pose. Ignore the return value since the pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseFrame);

  if (_frameGraph)
  {
    _frameGraph->AddVertex(this->dataPtr->name,
        ignition::math::Matrix4d(this->dataPtr->pose));
    this->dataPtr->frameGraph = _frameGraph;
  }

  // Load the geometry
  Errors geomErr = this->dataPtr->geom.Load(_sdf->GetElement("geometry"));
  errors.insert(errors.end(), geomErr.begin(), geomErr.end());

  return errors;
}

/////////////////////////////////////////////////
std::string Visual::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Visual::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Visual::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Visual::PoseFrame() const
{
  return this->dataPtr->poseFrame;
}

/////////////////////////////////////////////////
void Visual::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Visual::SetPoseFrame(const std::string &_frame)
{
  this->dataPtr->poseFrame = _frame;
}

/////////////////////////////////////////////////
const Geometry *Visual::Geom() const
{
  return &this->dataPtr->geom;
}
