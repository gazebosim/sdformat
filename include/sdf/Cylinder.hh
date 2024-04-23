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
#ifndef SDF_CYLINDER_HH_
#define SDF_CYLINDER_HH_

#include <optional>

#include <gz/math/Cylinder.hh>
#include <gz/math/Inertial.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \brief Cylinder represents a cylinder shape, and is usually accessed
  /// through a Geometry.
  class SDFORMAT_VISIBLE Cylinder
  {
    /// \brief Constructor
    public: Cylinder();

    /// \brief Load the cylinder geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the cylinder's radius in meters.
    /// \return The radius of the cylinder in meters.
    public: [[nodiscard]] double Radius() const;

    /// \brief Set the cylinder's radius in meters.
    /// \param[in] _radius The radius of the cylinder in meters.
    public: void SetRadius(const double _radius);

    /// \brief Get the cylinder's length in meters.
    /// \return The length of the cylinder in meters.
    public: [[nodiscard]] double Length() const;

    /// \brief Set the cylinder's length in meters.
    /// \param[in] _length The length of the cylinder in meters.
    public: void SetLength(const double _length);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: [[nodiscard]] sdf::ElementPtr Element() const;

    /// \brief Get the Gazebo Math representation of this Cylinder.
    /// \return A const reference to a gz::math::Sphered object.
    public: [[nodiscard]] const gz::math::Cylinderd &Shape() const;

    /// \brief Get a mutable Gazebo Math representation of this Cylinder.
    /// \return A reference to a gz::math::Cylinderd object.
    public: gz::math::Cylinderd &Shape();

    /// \brief Calculate and return the Inertial values for the Cylinder. In
    /// order to calculate the inertial properties, the function mutates the
    /// object by updating its material properties.
    /// \param[in] _density Density of the cylinder in kg/m^3
    /// \return A std::optional with gz::math::Inertiald object or std::nullopt
    public: std::optional<gz::math::Inertiald>
            CalculateInertial(double _density);

    /// \brief Create and return an SDF element filled with data from this
    /// cylinder.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated cylinder values.
    public: [[nodiscard]] sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// cylinder.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated cylinder values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
