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
#include <string>
#include <utility>
#include "Utils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

/////////////////////////////////////////////////
bool isReservedName(const std::string &_name)
{
  const std::size_t size = _name.size();
  return _name == "world" ||
      (size >= 4 &&
       _name.compare(0, 2, "__") == 0 &&
       _name.compare(size-2, 2, "__") == 0);
}

/////////////////////////////////////////////////
bool loadName(sdf::ElementPtr _sdf, std::string &_name)
{
  // Read the name
  std::pair<std::string, bool> namePair = _sdf->Get<std::string>("name", "");

  _name = namePair.first;
  return namePair.second;
}

/////////////////////////////////////////////////
bool loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
              std::string &_frame)
{
  sdf::ElementPtr sdf = _sdf;
  if (_sdf->GetName() != "pose")
  {
    if (_sdf->HasElement("pose"))
      sdf = _sdf->GetElement("pose");
    else
      return false;
  }

  // Read the frame. An empty frame implies the parent frame.
  std::pair<std::string, bool> framePair =
      sdf->Get<std::string>("relative_to", "");

  // Read the pose value.
  std::pair<ignition::math::Pose3d, bool> posePair =
    sdf->Get<ignition::math::Pose3d>("", ignition::math::Pose3d::Zero);

  // Set output, but only if the return value is true.
  if (posePair.second)
  {
    _pose = posePair.first;
    _frame = framePair.first;
  }

  // The frame attribute is optional, so only return true or false based
  // on the pose element value.
  return posePair.second;
}

/////////////////////////////////////////////////
double infiniteIfNegative(const double _value)
{
  if (_value < 0.0)
    return std::numeric_limits<double>::infinity();

  return _value;
}
}
}
