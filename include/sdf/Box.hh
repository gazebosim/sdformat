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

#include <ignition/math/Vector3.hh>
#include <sdf/Error.hh>
#include <sdf/Element.hh>

namespace sdf
{
  // Forward declare private data class.
  class BoxPrivate;

  /// \brief Box represents a box shape, and is usually accessed through a
  /// Geometry.
  class SDFORMAT_VISIBLE Box
  {
    /// \brief Constructor
    public: Box();

    /// \brief Destructor
    public: virtual ~Box();

    /// \brief Load the box geometry based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the box size in meters.
    /// \return Size of the box in meters.
    public: ignition::math::Vector3d Size() const;

    /// \brief Set the box size in meters.
    /// \param[in] _size Size of the box in meters.
    public: void SetSize(const ignition::math::Vector3d &_size);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: BoxPrivate *dataPtr;
  };
}

#endif
