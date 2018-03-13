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
#ifndef SDF_VISUAL_HH_
#define SDF_VISUAL_HH_

#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class VisualPrivate;

  class SDFORMAT_VISIBLE Visual
  {
    /// \brief Default constructor
    public: Visual();

    /// \brief Move constructor
    /// \param[in] _visual Visual to move.
    public: Visual(Visual &&_visual);

    /// \brief Destructor
    public: ~Visual();

    /// \brief Load the visual based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the visual.
    /// The name of the visual must be unique within the scope of a Link.
    /// \return Name of the visual.
    public: std::string Name() const;

    /// \brief Set the name of the visual.
    /// The name of the visual must be unique within the scope of a Link.
    /// \param[in] _name Name of the visual.
    public: void SetName(const std::string &_name) const;

    /// \brief Get the pose of the visual object. This is the pose of the
    /// visual as specified in SDF
    /// (<visual><pose> ... </pose></visual>).
    /// \return The pose of the visual object.
    public: const ignition::math::Pose3d &Pose() const;

    /// \brief Set the pose of the visual object.
    /// \sa const ignition::math::Pose3d &Pose() const
    /// \param[in] _pose The pose of the visual object.
    public: void SetPose(const ignition::math::Pose3d &_pose);

    /// \brief Get the name of the coordinate frame in which this visual
    /// object's pose is expressed. A empty value indicates that the frame is
    /// the parent link.
    /// \return The name of the pose frame.
    public: const std::string &PoseFrame() const;

    /// \brief Set the name of the coordinate frame in which this visual
    /// object's pose is expressed. A empty value indicates that the frame is
    /// the parent link.
    /// \return The name of the pose frame.
    public: void SetPoseFrame(const std::string &_pose);

    /// \brief Private data pointer.
    private: VisualPrivate *dataPtr = nullptr;
  };
}
#endif
