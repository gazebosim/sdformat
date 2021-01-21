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

#ifndef SDF_INTERFACE_MODEL_HH_
#define SDF_INTERFACE_MODEL_HH_

#include <string>
#include <memory>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/utils/ImplPtr.hh>

#include "sdf/InterfaceFrame.hh"
#include "sdf/InterfaceJoint.hh"
#include "sdf/InterfaceLink.hh"

#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
class InterfaceModel;

using InterfaceModelPtr = std::shared_ptr<InterfaceModel>;
using InterfaceModelConstPtr = std::shared_ptr<const InterfaceModel>;

class SDFORMAT_VISIBLE InterfaceModel
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
              const ignition::math::Pose3d &_poseInParentFrame = {});

  /// \brief Get the name of the model.
  /// \return Local name of the model.
  public: const std::string &Name() const;

  /// \brief Get the pose of this model in the parent model frame.
  /// \return Pose of this model in the parent model frame.
  // public: const ignition::math::Pose3d &PoseInModelFrame() const;
  // TODO: (addisu) The CanonicalLinkName function mirrores
  // Model::CanonicalLinkName, which is only a getter function. If the
  // canonical_link attribute is not set, this will return an empty string. We
  // need another function that resolves the canonical link.
  public: const std::string &CanonicalLinkName() const;

  public: const ignition::math::Pose3d &
          ModelFramePoseInCanonicalLinkFrame() const;

  public: const ignition::math::Pose3d &
          ModelFramePoseInParentFrame() const;

  // TODO: Why is this nested_model a shared_ptr?
  /// Provided so that hierarchy can still be leveraged from SDFormat.
  public: void AddNestedModel(sdf::InterfaceModelPtr _nestedModel);

  /// Gets registered nested models.
  public: const std::vector<sdf::InterfaceModelConstPtr> &NestedModels() const;

  /// Provided so that the including SDFormat model can still interface with
  /// the declared frames.
  public: void AddFrame(sdf::InterfaceFrame _frame);

  /// Gets registered frames.
  public: const std::vector<sdf::InterfaceFrame> &Frames() const;

  /// Provided so that the including SDFormat model can still interface with
  /// the declared joints.
  public: void AddJoint(sdf::InterfaceJoint _joint);

  /// Gets registered joints.
  public: const std::vector<sdf::InterfaceJoint> &Joints() const;

  /// Provided so that the including SDFormat model can still interface with
  /// the declared links.
  public: void AddLink(sdf::InterfaceLink _link);

  /// Gets registered links.
  public: const std::vector<sdf::InterfaceLink> &Links() const;

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
