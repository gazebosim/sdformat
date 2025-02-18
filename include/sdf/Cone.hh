/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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
#ifndef SDF_CONE_HH_
#define SDF_CONE_HH_

#include <optional>

#include <gz/math/Cone.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/config.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \brief Cone represents a cone shape, and is usually accessed
  /// through a Geometry.
  class SDFORMAT_VISIBLE Cone
  {
    /// \brief Constructor
    public: Cone();

    /// \brief Load the cone geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the cone's radius in meters.
    /// \return The radius of the cone in meters.
    public: double Radius() const;

    /// \brief Set the cone's radius in meters.
    /// \param[in] _radius The radius of the cone in meters.
    public: void SetRadius(double _radius);

    /// \brief Get the cone's length in meters.
    /// \return The length of the cone in meters.
    public: double Length() const;

    /// \brief Set the cone's length in meters.
    /// \param[in] _length The length of the cone in meters.
    public: void SetLength(double _length);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the Gazebo Math representation of this cone.
    /// \return A const reference to a gz::math::Sphered object.
    public: const gz::math::Coned &Shape() const;

    /// \brief Get a mutable Gazebo Math representation of this cone.
    /// \return A reference to a gz::math::Coned object.
    public: gz::math::Coned &Shape();

    /// \brief Calculate and return the Inertial values for the cone. In
    /// order to calculate the inertial properties, the function mutates the
    /// object by updating its material properties.
    /// \param[in] _density Density of the cone in kg/m^3
    /// \return A std::optional with gz::math::Inertiald object or std::nullopt
    public: std::optional<gz::math::Inertiald>
            CalculateInertial(double _density);

    /// \brief Get the Axis-aligned box for this Cone.
    /// \return A gz::math::AxisAlignedBox object.
    public: gz::math::AxisAlignedBox AxisAlignedBox() const;

    /// \brief Create and return an SDF element filled with data from this
    /// cone.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated cone values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// cone.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated cone values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
