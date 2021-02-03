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
#ifndef SDF_ELLIPSOID_HH_
#define SDF_ELLIPSOID_HH_

#include <ignition/math/Ellipsoid.hh>
#include <ignition/utils/ImplPtr.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \brief Ellipsoid represents a ellipsoid shape, and is usually accessed
  /// through a Geometry.
  class SDFORMAT_VISIBLE Ellipsoid
  {
    /// \brief Constructor
    public: Ellipsoid();

    /// \brief Load the ellipsoid geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the ellipsoid's radii in meters.
    /// \return The radius of the ellipsoid in meters.
    public: ignition::math::Vector3d Radii() const;

    /// \brief Set the ellipsoid's x, y, and z radii in meters.
    /// \param[in] _radius Vector of radii (x, y, z) of the ellipsoid in meters.
    public: void SetRadii(const ignition::math::Vector3d &_radii);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the Ignition Math representation of this Ellipsoid.
    /// \return A const reference to an ignition::math::Ellipsoidd object.
    public: const ignition::math::Ellipsoidd &Shape() const;

    /// \brief Get a mutable Ignition Math representation of this Ellipsoid.
    /// \return A reference to an ignition::math::Ellipsoidd object.
    public: ignition::math::Ellipsoidd &Shape();

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
