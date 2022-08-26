/*
 * Copyright 2022 Open Source Robotics Foundation
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
#ifndef SDF_POLYLINE_HH_
#define SDF_POLYLINE_HH_

#include <vector>

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
  class PolylinePrivate;

  /// \brief Polyline represents a 2D path. Multiple polylines can be combined
  class SDFORMAT_VISIBLE Polyline
  {
    /// \brief Constructor
    public: Polyline();

    /// \brief Copy constructor
    /// \param[in] _polyline Polyline to copy.
    public: Polyline(const Polyline &_polyline);

    /// \brief Move constructor
    /// \param[in] _polyline Polyline to move.
    public: Polyline(Polyline &&_polyline) noexcept;

    /// \brief Destructor
    public: virtual ~Polyline();

    /// \brief Assignment operator.
    /// \param[in] _polyline The polyline to set values from.
    /// \return *this
    public: Polyline &operator=(const Polyline &_polyline);

    /// \brief Move assignment operator.
    /// \param[in] _polyline Polyline to move.
    /// \return Reference to this.
    public: Polyline &operator=(Polyline &&_polyline);

    /// \brief Load the polyline geometry based on an element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the polyline's height in meters.
    /// \return The height of the polyline in meters.
    public: double Height() const;

    /// \brief Set the polyline's height in meters.
    /// \param[in] _height The height of the polyline in meters.
    public: void SetHeight(const double _height);

    /// \brief Get the number of points.
    /// \return Number of points.
    public: uint64_t PointCount() const;

    /// \brief Get a point by its index.
    /// \return Constant pointer to the point.
    public: const gz::math::Vector2d *PointByIndex(uint64_t _index) const;

    /// \brief Get a point by its index.
    /// \return Mutable pointer to the point.
    public: gz::math::Vector2d *PointByIndex(uint64_t _index);

    /// \brief Add a point to the polyline.
    /// \param[in] _point 2D point to add.
    /// \return True for success.
    public: bool AddPoint(const gz::math::Vector2d &_point);

    /// \brief Remove all points from the polyline.
    public: void ClearPoints();

    /// \brief Get the polyline's points. Each point has 2D coordinates in
    /// meters.
    /// \return The polyline's points.
    public: const std::vector<gz::math::Vector2d> &Points() const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: PolylinePrivate *dataPtr;
  };
  }
}
#endif
