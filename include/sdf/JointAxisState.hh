/*
 * Copyright 2024 Open Source Robotics Foundation
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
#ifndef SDF_JOINTAXISSTATE_HH_
#define SDF_JOINTAXISSTATE_HH_

#include <memory>
#include <string>
#include <utility>
#include <gz/math/Vector3.hh>
#include <gz/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/Exception.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief Parameters related to the state variables of an axis of rotation
  /// for rotational joints, and the axis of translation for prismatic joints.
  class SDFORMAT_VISIBLE JointAxisState
  {
    /// \brief Default constructor
    public: JointAxisState();

    /// \brief Load the joint axis state based on a element pointer. This is
    /// *not* the usual entry point. Typical usage of the SDF DOM is through
    /// the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the joint axis position in units of radians [rad] for a
    /// rotational axis and units of meters [m] for a translational axis.
    /// \return The joint axis position.
    public: double Position();

    /// \brief Get the joint axis velocity in units of radians per second
    /// [rad/s] for a rotational axis and units of meters per second for a
    /// translational axis.
    /// \return The joint axis velocity.
    public: double Velocity();

    /// \brief Get the joint axis acceleration in units of radians per second
    /// per second [rad/s^2] for a rotational axis and units of meters per
    /// second per second [m/s^2] for a translational axis.
    /// \return The joint axis acceleration.
    public: double Acceleration();

    /// \brief Get the joint axis effort in units of Newton-meters [Nm] for a
    /// rotational axis and units of Newtons [N] for a translational axis.
    /// \return The joint axis effort.
    public: double Effort();

    public: void SetPosition(double _position);
    public: void SetVelocity(double _velocity);
    public: void SetAcceleration(double _acceleration);
    public: void SetEffort(double _effort);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// joint axis.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[in] _index Index of this joint axis
    /// \return SDF element pointer with updated joint values.
    public: sdf::ElementPtr ToElement(unsigned int _index = 0u) const;

    /// \brief Create and return an SDF element filled with data from this
    /// joint axis.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _index Index of this joint axis
    /// \return SDF element pointer with updated joint values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors,
                                      unsigned int _index = 0u) const;

    /// \brief Private data pointer
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
