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
#ifndef SDF_COLLISION_HH_
#define SDF_COLLISION_HH_

#include <memory>
#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/FrameSemantics.hh"
#include "sdf/Link.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declaration.
  class CollisionPrivate;
  class Geometry;

  /// \brief A collision element descibes the collision properties associated
  /// with a link. This can be different from the visual properties of a link.
  /// For example, simple collision models are often used to reduce
  /// computation time.
  class SDFORMAT_VISIBLE Collision
  {
    /// \brief Default constructor
    public: Collision();

    /// \brief Move constructor
    /// \param[in] _collision Collision to move.
    public: Collision(Collision &&_collision);

    /// \brief Destructor
    public: ~Collision();

    /// \brief Load the collision based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the collision.
    /// The name of the collision must be unique within the scope of a Link.
    /// \return Name of the collision.
    public: std::string Name() const;

    /// \brief Set the name of the collision.
    /// The name of the collision must be unique within the scope of a Link.
    /// \param[in] _name Name of the collision.
    public: void SetName(const std::string &_name) const;

    /// \brief Get a pointer to the collisions's geometry.
    /// \return The collision's geometry.
    public: const Geometry *Geom() const;

    /// \brief Get the pose of the collision object. This is the pose of the
    /// collision as specified in SDF
    /// (<collision><pose> ... </pose></collision>).
    /// \return The pose of the collision object.
    public: const ignition::math::Pose3d &Pose() const;

    /// \brief Set the pose of the collision object.
    /// \sa const ignition::math::Pose3d &Pose() const
    /// \param[in] _pose The pose of the collision object.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \return The name of the pose relative-to frame.
    public: const std::string &PoseRelativeTo() const;

    /// \brief Set the name of the coordinate frame relative to which this
    /// object's pose is expressed. An empty value indicates that the frame is
    /// relative to the parent link.
    /// \param[in] _frame The name of the pose relative-to frame.
    public: void SetPoseRelativeTo(const std::string &_frame);

    /// \brief Resolve pose of this object relative to another named frame.
    /// \param[in] _relativeTo Name of frame relative to which the pose of
    /// this object should be resolved.
    /// \param[out] _pose Resolved pose.
    /// \return Errors.
    public: Errors ResolvePose(
        const std::string &_relativeTo,
        ignition::math::Pose3d &_pose) const;

    /// \brief Resolve pose of this object relative to the implicit frame
    /// of its xml parent object, which is always a link frame.
    /// \param[out] _pose Resolved pose.
    /// \return Errors.
    public: Errors ResolvePose(ignition::math::Pose3d &_pose) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Give the name of the xml parent of this object, to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _xmlParentName Name of xml parent object.
    private: void SetXmlParentName(const std::string &_xmlParentName);

    /// \brief Give a weak pointer to the PoseRelativeToGraph to be used
    /// for resolving poses. This is private and is intended to be called by
    /// Link::SetPoseRelativeToGraph.
    /// \param[in] _graph Weak pointer to PoseRelativeToGraph.
    private: void SetPoseRelativeToGraph(
        std::weak_ptr<const PoseRelativeToGraph> _graph);

    /// \brief Allow Link::SetPoseRelativeToGraph to call SetXmlParentName
    /// and SetPoseRelativeToGraph, but Link::SetPoseRelativeToGraph is
    /// a private function, so we need to befriend the entire class.
    friend Link;

    /// \brief Private data pointer.
    private: CollisionPrivate *dataPtr = nullptr;
  };
  }
}
#endif
