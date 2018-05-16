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
#ifndef SDF_LINK_HH_
#define SDF_LINK_HH_

#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class Collision;
  class Visual;
  class LinkPrivate;

  class SDFORMAT_VISIBLE Link
  {
    /// \brief Default constructor
    public: Link();

    /// \brief Move constructor
    /// \param[in] _link Link to move.
    public: Link(Link &&_link);

    /// \brief Destructor
    public: ~Link();

    /// \brief Load the link based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the link.
    /// The name of a link must be unique within the scope of a Model.
    /// \return Name of the link.
    public: std::string Name() const;

    /// \brief Set the name of the link.
    /// The name of a link must be unique within the scope of a Model.
    /// \param[in] _name Name of the link.
    public: void SetName(const std::string &_name) const;

    /// \brief Get the number of visuals.
    /// \return Number of visuals contained in this Link object.
    public: uint64_t VisualCount() const;

    /// \brief Get a visual based on an index.
    /// \param[in] _index Index of the visual. The index should be in the
    /// range [0..VisualCount()).
    /// \return Pointer to the visual. Nullptr if the index does not exist.
    /// \sa uint64_t VisualCount() const
    public: const Visual *VisualByIndex(const uint64_t _index) const;

    /// \brief Get whether a visual name exists.
    /// \param[in] _name Name of the visual to check.
    /// \return True if there exists a visual with the given name.
    public: bool VisualNameExists(const std::string &_name) const;

    /// \brief Get a visual based on a name.
    /// \param[in] _name Name of the visual.
    /// \return Pointer to the visual. Nullptr if the name does not exist.
    public: const Visual *VisualByName(const std::string &_name) const;

    /// \brief Get the number of collisions.
    /// \return Number of collisions contained in this Link object.
    public: uint64_t CollisionCount() const;

    /// \brief Get a collision based on an index.
    /// \param[in] _index Index of the collision. The index should be in the
    /// range [0..CollisionCount()).
    /// \return Pointer to the collision. Nullptr if the index does not exist.
    /// \sa uint64_t CollisionCount() const
    public: const Collision *CollisionByIndex(const uint64_t _index) const;

    /// \brief Get whether a collision name exists.
    /// \param[in] _name Name of the collision to check.
    /// \return True if there exists a collision with the given name.
    public: bool CollisionNameExists(const std::string &_name) const;

    /// \brief Get a collision based on a name.
    /// \param[in] _name Name of the collision.
    /// \return Pointer to the collision. Nullptr if the name does not exist.
    public: const Collision *CollisionByName(const std::string &_name) const;

    /// \brief Get the inertial value for this link. The inertial object
    /// consists of the link's mass, a 3x3 rotational inertia matrix, and
    /// a pose for the inertial reference frame. The units for mass is
    /// kilograms with a default value of 1kg. The 3x3 rotational inertia
    /// matrix is symmetric and only 6 above-diagonal elements of this matrix
    /// are specified the Interial's ignition::math::MassMatrix3 property.
    ///
    /// The origin of the inertial reference frame needs to be at the center
    /// of mass expressed in this link's frame.
    /// The axes of the inertial reference frame do not need to
    /// be aligned with the principal axes of the inertia.
    /// \return The link's inertial value.
    /// \sa void SetInertial(const ignition::math::Inertiald &_inertial)
    public: const ignition::math::Inertiald &Inertial() const;

    /// \brief Set the inertial value for this link.
    /// \param[in] _inertial The link's inertial value.
    /// \return True if the inertial is valid, false otherwise.
    /// \sa const ignition::math::Inertiald &Inertial() const
    public: bool SetInertial(const ignition::math::Inertiald &_inertial);

    /// \brief Get the pose of the link. This is the pose of the link
    /// as specified in SDF (<link> <pose> ... </pose></link>).
    /// \return The pose of the link.
    public: const ignition::math::Pose3d &Pose() const;

    /// \brief Set the pose of the link.
    /// \sa const ignition::math::Pose3d &Pose() const
    /// \param[in] _pose The new link pose.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame in which this link's
    /// pose is expressed. A empty value indicates that the frame is the
    /// parent model.
    /// \return The name of the pose frame.
    public: const std::string &PoseFrame() const;

    /// \brief Set the name of the coordinate frame in which this link's
    /// pose is expressed. A empty value indicates that the frame is the
    /// parent model.
    /// \param[in] _frame The name of the pose frame.
    public: void SetPoseFrame(const std::string &_frame);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: LinkPrivate *dataPtr = nullptr;
  };
}
#endif
