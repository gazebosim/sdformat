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
#ifndef SDF_BOX_HH_
#define SDF_BOX_HH_

#include <gz/math/Box.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  // Forward declare private data class.
  class BoxPrivate;

  /// \brief Box represents a box shape, and is usually accessed through a
  /// Geometry.
  class SDFORMAT_VISIBLE Box
  {
    /// \brief Constructor
    public: Box();

    /// \brief Copy constructor
    /// \param[in] _box Box to copy.
    public: Box(const Box &_box);

    /// \brief Move constructor
    /// \param[in] _box Box to move.
    public: Box(Box &&_box) noexcept;

    /// \brief Move assignment operator.
    /// \param[in] _box Box to move.
    /// \return Reference to this.
    public: Box &operator=(Box &&_box);

    /// \brief Destructor
    public: virtual ~Box();

    /// \brief Assignment operator.
    /// \param[in] _box The box to set values from.
    /// \return *this
    public: Box &operator=(const Box &_box);

    /// \brief Load the box geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the box size in meters.
    /// \return Size of the box in meters.
    public: gz::math::Vector3d Size() const;

    /// \brief Set the box size in meters.
    /// \param[in] _size Size of the box in meters.
    public: void SetSize(const gz::math::Vector3d &_size);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the Gazebo Math representation of this Box.
    /// \return A const reference to a gz::math::Boxd object.
    public: const gz::math::Boxd &Shape() const;

    /// \brief Get a mutable Gazebo Math representation of this Box.
    /// \return A reference to a gz::math::Boxd object.
    public: gz::math::Boxd &Shape();

    /// \brief Private data pointer.
    private: BoxPrivate *dataPtr;
  };
  }
}

#endif
