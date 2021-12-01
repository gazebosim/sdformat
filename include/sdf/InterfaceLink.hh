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

#ifndef SDF_INTERFACE_LINK_HH_
#define SDF_INTERFACE_LINK_HH_

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
/// \brief Interface element representing a Link
class SDFORMAT_VISIBLE InterfaceLink
{
  /// \brief Constructor
  /// \param[in] name The *local* name.
  /// \param[in] _pose The pose of the link relative to model frame.
  public: InterfaceLink(
              const std::string &_name, const ignition::math::Pose3d &_pose);

  /// \brief Get the name of the link.
  /// \return Local name of the link.
  public: const std::string &Name() const;

  /// \brief Get the pose of this link in the parent model frame.
  /// \return The pose of this link in the parent model frame.
  public: const ignition::math::Pose3d &PoseInModelFrame() const;

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
