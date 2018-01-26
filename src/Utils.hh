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
#ifndef SDFORMAT_UTILS_HH
#define SDFORMAT_UTILS_HH

#include <string>
#include "sdf/system_util.hh"
#include "sdf/Element.hh"

namespace sdf
{
  /// \brief Read the "name" attribute from an element.
  /// \param[in] _sdf SDF element pointer which contains the name.
  /// \param[out] _name String to hold the name value.
  /// \return True when the "name" attribute exists.
  SDFORMAT_VISIBLE
  bool loadName(sdf::ElementPtr _sdf, std::string &_name);

  /// \brief Read a pose element from and SDF pointer, and return (via
  /// function parameters) the pose value and coordinate frame.
  /// \param[in] _sdf Pointer to an SDF element that is a pose element.
  /// \param[out] _pose Value of the pose element. The default value is
  /// ignition::math::Pose3d::Zero.
  /// \param[out] _frame Value of the frame attribute. The default value is
  /// and empty string.
  /// \return True if the pose element contained an ignition::math::Pose3d
  /// value.
  SDFORMAT_VISIBLE
  bool loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
                std::string &_frame);
}
#endif
