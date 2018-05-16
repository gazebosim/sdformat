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
#ifndef SDF_PLANE_HH_
#define SDF_PLANE_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>

namespace sdf
{
  // Forward declare private data class.
  class PlanePrivate;

  /// \brief Plane represents a plane shape, and is usually accessed through a
  /// Geometry.
  class SDFORMAT_VISIBLE Plane
  {
    /// \brief Constructor
    public: Plane();

    /// \brief Destructor
    public: virtual ~Plane();

    /// \brief Load the plane geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the plane normal vector. When a Plane is used as a geometry
    /// for a Visual or Collision object, then the normal is specified in the
    /// Visual or Collision frame, respectively.
    /// \return The plane normal vector.
    public: ignition::math::Vector3d Normal() const;

    /// \brief Set the plane normal vector. The _normal vector will be
    /// normalized. See ignition::math::Vector3d Normal() for more information
    /// about the normal vector, such as the frame in which it is specified.
    /// \param[in] _normal The plane normal vector.
    public: void SetNormal(const ignition::math::Vector3d &_normal);

    /// \brief Get the plane size in meters.
    /// \return The plane size in meters.
    public: ignition::math::Vector2d Size() const;

    /// \brief Set the plane size in meters.
    /// \param[in] _size The plane size in meters.
    public: void SetSize(const ignition::math::Vector2d &_size);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: PlanePrivate *dataPtr;
  };
}
#endif
