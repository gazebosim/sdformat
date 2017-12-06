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
#ifndef SDF_DOM_JOINT_HH_
#define SDF_DOM_JOINT_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"
#include "sdf/dom/Entity.hh"

namespace sdf
{
  // Forward declare private data class.
  class JointPrivate;

  /// \enum JointType
  /// \brief This enum contains the set of supported joint types. A joint
  /// type defines the kinematic behavior of the joint.
  enum class JointType
  {
    /// \brief A ball and socket joint.
    BALL,

    /// \brief A joint with zero degrees of freedom that rigidly connects
    /// two links.
    FIXED,

    /// \brief A geared revolute joint.
    GEARBOX,

    /// \brief A joint that slides along an axis with a limited
    /// range specified by upper and lower limits.
    PRISMATIC,

    /// \brief A hinge joint that rotates on a single axis with either
    /// a fixed or continuous range of motion.
    REVOLUTE,

    /// \brief Same as two revolute joints connected in series.
    REVOLUTE2,

    /// \brief A single degree of freedom joint with coupled sliding and
    /// rotational motion
    SCREW,

    /// \brief Like a ball joint, but constrains one degree of freedom
    UNIVERSAL,

    /// \brief Unknown or unsupported joint type
    UNKNOWN
  };

  /// \brief A joint connects two links with kinematic and dynamic
  /// properties. This class contains information for a single joint.
  class SDFORMAT_VISIBLE Joint : public Entity
  {
    /// \brief Constructor
    public: Joint();

    /// \brief Destructor
    public: ~Joint();

    /// \brief Load the joint based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf An SDF Element pointer to a <joint> element.
    /// \return True when no errors were encountered.
    public: bool Load(sdf::ElementPtr _sdf);

    /// \brief Print debug information to standard out.
    /// \param[in] _prefix String to prefix all output.
    public: void Print(const std::string &_prefix = "") const;

    /// \brief Get the type of the joint.
    /// \return Type of the joint.
    public: JointType Type() const;

    /// \brief Get a string version of the joint type.
    /// \return Name of the joint type.
    public: std::string Typename() const;

    /// \brief Set the type of the joint.
    /// \param[in] _type Type of the joint.
    public: void SetType(const JointType _type);

    /// \brief Private data pointer
    public: JointPrivate *dataPtr;
  };
}
#endif
