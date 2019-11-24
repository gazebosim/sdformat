/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_SEMANTIC_POSE_HH_
#define SDF_SEMANTIC_POSE_HH_

#include <string>
#include <ignition/math/Pose3.hh>

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class SemanticPosePrivate;

  /// \brief SemanticPose is a data structure that can be used by different
  /// DOM objects to resolve poses on a PoseRelativeToGraph. This object holds
  /// a Pose3 object, the name of the frame relative to which it is defined,
  /// a pointer to a PoseRelativeToGraph, and a default frame to resolve to.
  /// The name of the default frame to resolve to must not be empty.
  /// This class only has a private constructor, and is may be accessed from
  /// its friend DOM classes.
  class SDFORMAT_VISIBLE SemanticPose
  {
    /// \brief Get the raw Pose3 transform.
    /// \return The raw Pose3 transform.
    public: const ignition::math::Pose3d &RawPose() const;

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent model.
    /// \return The name of the pose relative-to frame.
    public: const std::string &RelativeTo() const;

    /// \brief Resolve pose of this object with respect to another named frame.
    /// If there are any errors resolving the pose, the output will not be
    /// modified.
    /// \param[in] _resolveTo The pose will be resolved with respect to this
    /// frame.
    /// \param[out] _pose The resolved pose.
    /// \return Errors in resolving pose.
    public: Errors Resolve(const std::string &_resolveTo,
                           ignition::math::Pose3d &_pose) const;

    /// \brief Resolve pose of this object with respect to this object's
    /// default resolve-to frame.
    /// If there are any errors resolving the pose, the output will not be
    /// modified.
    /// \param[out] _pose The resolved pose.
    /// \return Errors in resolving pose.
    public: Errors Resolve(ignition::math::Pose3d &_pose) const;

    /// \brief Private constructor.
    private: SemanticPose(
        const ignition::math::Pose3d &_pose,
        const std::string &_relativeTo,
        const std::string &_defaultResolveTo,
        std::weak_ptr<const sdf::PoseRelativeToGraph> _graph);

    friend class Collision;
    friend class Frame;
    friend class Joint;
    // friend SemanticPose Light::SemanticPose();
    friend class Link;
    // friend SemanticPose Model::SemanticPose();
    // friend SemanticPose Sensor::SemanticPose();
    friend class Visual;

    /// \brief Private data pointer.
    private: SemanticPosePrivate *dataPtr;
  };
  }
}
#endif
