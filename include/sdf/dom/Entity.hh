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
#ifndef SDF_DOM_ENTITY_HH_
#define SDF_DOM_ENTITY_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class EntityPrivate;

  class SDFORMAT_VISIBLE Entity
  {
    /// \brief Constructor
    public: Entity();

    /// \brief Destructor
    public: virtual ~Entity();

    /// \brief Load the link based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf An SDF Element pointer to a <link> element.
    /// \return True when no errors were encountered.
    public: virtual bool Load(sdf::ElementPtr _sdf) = 0;

    /// \brief Get the name of the entity.
    /// \return Name of the entity.
    public: std::string Name() const;

    /// \brief Set the name of the entity.
    /// \param[in] _name Name of the entity.
    public: void SetName(const std::string &_name);

    /// \brief Get the name of the frame. The Pose of the entity is relative
    /// to this frame of reference.
    /// \return Name of the frame of reference.
    /// \sa ignition::math::Pose3d Pose() const
    public: std::string Frame() const;

    /// \brief Set the name of the frame the Pose of the entity is relative
    /// to.
    /// \param[in] _frame Name of the frame of reference.
    public: void SetFrame(const std::string &_frame);

    /// \brief Get the pose of the entity. This pose is relative to the
    /// frame.
    /// \return The pose of the entity relative to the entity's frame.
    /// \sa std::string Frame() const
    public: ignition::math::Pose3d Pose() const;

    /// \brief Set the pose of the entity.
    /// \param[in] _pose Pose of the entity.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Helper function that loads an entity's name from SDF.
    /// \param[in] _sdf Pointer to the SDF element that contains the name.
    /// \return True if no errors were encountered.
    protected: bool LoadName(sdf::ElementPtr _sdf);

    /// \brief Helper function that loads an entity's pose from SDF.
    /// \param[in] _sdf Pointer to the SDF element that contains the pose.
    /// \return True if no errors were encountered. A pose is optional for
    /// entities, so this function should return true.
    protected: bool LoadPose(sdf::ElementPtr _sdf);

    /// \brief Private data pointer
    private: EntityPrivate *dataPtr;
  };
}
#endif
