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

#ifndef SDF_INTERFACE_JOINT_HH_
#define SDF_INTERFACE_JOINT_HH_

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
/// \brief Interface element representing a Joint
class SDFORMAT_VISIBLE InterfaceJoint
{
  /// \brief Constructor
  /// \param[in] name The *local* name.
  /// \param[in] _childName Name of the child link or frame.
  /// \param[in] _pose The pose of the joint relative to the child frame.
  public: InterfaceJoint(const std::string &_name,
      const std::string &_childName, const ignition::math::Pose3d &_pose);

  /// \brief Get the name of the joint.
  /// \return Local name of the joint.
  public: const std::string &Name() const;

  /// \brief Get the name of the joint's child.
  /// \return The name of the joint's child link or frame.
  public: const std::string &ChildName() const;

  /// \brief Get the pose of this joint in the child frame.
  /// \return The pose of this joint in the child frame.
  public: const ignition::math::Pose3d &PoseInChildFrame() const;

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
