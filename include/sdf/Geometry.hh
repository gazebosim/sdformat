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
#ifndef SDF_GEOMETRY_HH_
#define SDF_GEOMETRY_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>

namespace sdf
{
  // Forward declare private data class.
  class GeometryPrivate;

  /// \enum GeometryType
  /// \brief The set of geometry types. INVALID indicates an invalid or
  /// geometry.
  enum class GeometryType
  {
    /// \brief Invalid geometry.
    INVALID = 0,

    /// \brief A box geometry.
    BOX = 1,

    /// \brief A cylinder geometry.
    CYLINDER = 2,

    /// \brief A plane geometry.
    PLANE = 3,

    /// \brief A sphere geometry.
    SPHERE = 4,
  };

  class SDFORMAT_VISIBLE Geometry
  {
    /// \brief Default constructor
    public: Geometry();

    /// \brief Destructor
    public: virtual ~Geometry();

    /// \brief Load the geometry based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: virtual Errors Load(ElementPtr _sdf) = 0;

    /// \brief Get the type of geometry.
    /// \return The geometry type.
    public: GeometryType Type() const;

    /// \brief Set the type of geometry.
    /// \param[in] _type The geometry type.
    public: void SetType(const GeometryType _type);

    /// \brief Private data pointer.
    private: GeometryPrivate *dataPtr;
  };
}
#endif
