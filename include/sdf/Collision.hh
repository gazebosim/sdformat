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

#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declaration.
  class CollisionPrivate;
  class Geometry;

  /// \brief A collision element descibes the collison properties associated
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
    /// collison as specified in SDF
    /// (<collision><pose> ... </pose></collision>).
    /// \return The pose of the collision object.
    public: const ignition::math::Pose3d &Pose() const;

    /// \brief Set the pose of the collision object.
    /// \sa const ignition::math::Pose3d &Pose() const
    /// \param[in] _pose The pose of the collision object.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame in which this collision
    /// object's pose is expressed. A empty value indicates that the frame is
    /// the parent link.
    /// \return The name of the pose frame.
    public: const std::string &PoseFrame() const;

    /// \brief Set the name of the coordinate frame in which this collision
    /// object's pose is expressed. A empty value indicates that the frame is
    /// the parent link.
    /// \param[in] _frame The name of the pose frame.
    public: void SetPoseFrame(const std::string &_frame);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: CollisionPrivate *dataPtr = nullptr;
  };
}
#endif
