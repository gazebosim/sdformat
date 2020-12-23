
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

#ifndef SDF_INTERFACE_MODEL_HH_
#define SDF_INTERFACE_MODEL_HH_

#include <string>
#include <memory>

#include <ignition/math/Pose3.hh>

#include "sdf/InterfaceFrame.hh"
#include "sdf/InterfaceLink.hh"

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{

// Forward declare private data class.
class InterfaceModelPrivate;

class InterfaceModel;

using InterfaceModelPtr = std::shared_ptr<InterfaceModel>;
using InterfaceModelConstPtr = std::shared_ptr<const InterfaceModel>;

class InterfaceModel
{
  /// \brief Constructor
  /// \param[in] name The *local* name (no nesting, e.g. "::").
  ///   If this name contains "::", an error will be raised.
  /// \param[in] _canonicalLinkNameThe canonical link's name. (It must be
  ///   registered).
  /// \param[in] model_frame_pose_in_canonical_link_frame Model frame pose
  ///   relative to canonical link's frame. Defaults to identity.
  /// \param[in] model_frame_pose_in_parent_model_frame Model frame pose
  ///   relative to the including model's frame. Defaults to identity.
  ///   \note This will not be used if //include/pose is specified.
  public: InterfaceModel(const std::string &_name,
              const std::string &_canonicalLinkName,
              const ignition::math::Pose3d &_poseInCanonicalLinkFrame = {},
              const ignition::math::Pose3d &_poseInParentModelFrame = {});
  /// \brief Get the name of the model.
  /// \return Local name of the model.
  public: std::string Name() const;

  /// \brief Get the pose of this model in the parent model frame.
  /// \return Pose of this model in the parent model frame.
  public: ignition::math::Pose3d PoseInModelFrame() const;

  public: std::string GetCanonicalLinkName() const;
  public: ignition::math::Pose3d GetModelFramePoseInCanonicalLinkFrame() const;
  public: ignition::math::Pose3d GetModelFramePoseInParentModelFrame() const;
  /// Provided so that hierarchy can still be leveraged from SDFormat.
  public: void AddNestedModel(sdf::InterfaceModelPtr nested_model);
  /// Gets registered nested models.
  public: std::vector<sdf::InterfaceModelConstPtr> GetNestedModels() const;
  /// Provided so that the including SDFormat model can still interface with
  /// the declared frames.
  public: void AddFrame(sdf::InterfaceFrame frame);
  /// Gets registered frames.
  public: std::vector<sdf::InterfaceFrame> GetFrames() const;
  /// Provided so that the including SDFormat model can still interface with
  /// the declared links.
  public: void AddLink(sdf::InterfaceLink link);
  /// Gets registered links.
  public: std::vector<sdf::InterfaceLink> GetLinks() const;

  /// \brief Private data pointer.
  private: InterfaceModelPrivate *dataPtr;
};
}
}

#endif
