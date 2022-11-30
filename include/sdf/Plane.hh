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

#include <gz/math/Plane.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Vector2.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declare private data class.
  class PlanePrivate;

  /// \brief Plane represents a plane shape, and is usually accessed through a
  /// Geometry.
  class SDFORMAT_VISIBLE Plane
  {
    /// \brief Constructor
    public: Plane();

    /// \brief Copy constructor
    /// \param[in] _plane Plane to copy.
    public: Plane(const Plane &_plane);

    /// \brief Move constructor
    /// \param[in] _plane Plane to move.
    public: Plane(Plane &&_plane) noexcept;

    /// \brief Destructor
    public: virtual ~Plane();

    /// \brief Move assignment operator.
    /// \param[in] _plane Plane to move.
    /// \return Reference to this.
    public: Plane &operator=(Plane &&_plane);

    /// \brief Assignment operator.
    /// \param[in] _plane The plane to set values from.
    /// \return *this
    public: Plane &operator=(const Plane &_plane);

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
    public: gz::math::Vector3d Normal() const;

    /// \brief Set the plane normal vector. The normal vector will be
    /// normalized. See gz::math::Vector3d Normal() for more information
    /// about the normal vector, such as the frame in which it is specified.
    /// \param[in] _normal The plane normal vector.
    public: void SetNormal(const gz::math::Vector3d &_normal);

    /// \brief Get the plane size in meters.
    /// \return The plane size in meters.
    public: gz::math::Vector2d Size() const;

    /// \brief Set the plane size in meters.
    /// \param[in] _size The plane size in meters.
    public: void SetSize(const gz::math::Vector2d &_size);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the Gazebo Math representation of this Plane.
    /// \return A const reference to a gz::math::Planed object.
    public: const gz::math::Planed &Shape() const;

    /// \brief Get a mutable Gazebo Math representation of this Plane.
    /// \return A reference to a gz::math::Planed object.
    public: gz::math::Planed &Shape();

    /// \brief Private data pointer.
    private: PlanePrivate *dataPtr;
  };
  }
}
#endif
