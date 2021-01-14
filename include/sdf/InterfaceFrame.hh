
/*
 * Copyright 2020 Open Source Robotics Foundation
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

#ifndef SDF_INTERFACE_FRAME_HH_
#define SDF_INTERFACE_FRAME_HH_

#include <string>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class SDFORMAT_VISIBLE InterfaceFrame
{
  /// Constructor
  /// \param[in] _name The *local* name.
  /// \param[in] _attachedTo Name of the attached-to frame. Must be "__model__"
  /// if attached to model. TODO (addisu) Can it be ""
  /// \param[in] _pose The pose of the frame relative to model frame.
  public: InterfaceFrame(const std::string &_name,
      const std::string &_attachedTo, const ignition::math::Pose3d &_pose);

  /// Get the name of the frame.
  /// \return Local name of the frame.
  public: const std::string &Name() const;

  /// \brief Get the name of the coordinate frame to which this frame is
  /// attached.
  /// \return The name of the attached-to frame.
  public: const std::string &AttachedTo() const;

  /// \brief Get the pose of this frame relative to the attached-to frame.
  /// \return The pose of this frame in the attached-to frame.
  public: const ignition::math::Pose3d &PoseInAttachedToFrame() const;

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
