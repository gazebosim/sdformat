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
#ifndef SDF_CAPSULE_HH_
#define SDF_CAPSULE_HH_

#include <ignition/math/Capsule.hh>
#include <ignition/utils/ImplPtr.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \brief Capsule represents a capsule shape, and is usually accessed
  /// through a Geometry.
  class SDFORMAT_VISIBLE Capsule
  {
    /// \brief Constructor
    public: Capsule();

    /// \brief Load the capsule geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the capsule's radius in meters.
    /// \return The radius of the capsule in meters.
    public: double Radius() const;

    /// \brief Set the capsule's radius in meters.
    /// \param[in] _radius The radius of the capsule in meters.
    public: void SetRadius(const double _radius);

    /// \brief Get the capsule's length in meters.
    /// \return The length of the capsule in meters.
    public: double Length() const;

    /// \brief Set the capsule's length in meters.
    /// \param[in] _length The length of the capsule in meters.
    public: void SetLength(const double _length);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the Ignition Math representation of this Capsule.
    /// \return A const reference to an ignition::math::Sphered object.
    public: const ignition::math::Capsuled &Shape() const;

    /// \brief Get a mutable Ignition Math representation of this Capsule.
    /// \return A reference to an ignition::math::Capsuled object.
    public: ignition::math::Capsuled &Shape();

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
