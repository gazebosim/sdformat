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
#ifndef SDF_SPHERICAL_COORDINATES_HH_
#define SDF_SPHERICAL_COORDINATES_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declarations.
  class SphericalCoordinatesPrivate;

  /// \enum SurfaceModel
  /// \brief Name of planetary surface model, used to determine the surface
  /// altitude at a given latitude and longitude. The only supported model
  ///  is an ellipsoid model of the earth based on the WGS-84 standard.
  enum class SurfaceModel
  {
    /// \brief An invalid surface model.
    INVALID = 0,

    /// \brief An ellipsoid model of the earth based on the WGS-84 standard.
    EARTH_WGS84 = 1
  };

  /// \enum WorldFrameOrientation
  /// \brief Identifies how the world frame is aligned in a Geographical
  /// sense.  The final world frame orientation is obtained by rotating
  /// a frame aligned with following notation by the Heading (Note
  /// that Heading corresponds to positive yaw rotation in the NED frame,
  /// so it's inverse specifies positive Z-rotation in ENU or NWU).
  enum class WorldFrameOrientation
  {
    /// \brief An invalid world frame orientation.
    INVALID = 0,

    /// \brief East-North-Up
    ENU = 1,

    /// \brief North-East-Down
    NED = 2,

    /// \brief North-WEST-UP
    NWU = 3
  };

  class SDFORMAT_VISIBLE SphericalCoordinates
  {
    /// \brief Default constructor
    public: SphericalCoordinates();

    /// \brief Move constructor
    /// \param[in] _spherical coordinates SphericalCoordinates to move.
    public: SphericalCoordinates(SphericalCoordinates &&_coordinates);

    /// \brief Destructor
    public: ~SphericalCoordinates();

    /// \brief Load spherical coordinates based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the planetary surface model, used to determine
    /// the surface altitude at a given latitude and longitude.
    /// \return The planetary surface model.
    public: SurfaceModel SurfaceModel() const;

    /// \brief Set the planetary surface model, used to determine
    /// the surface altitude at a given latitude and longitude.
    /// \param[in] _model The planetary surface model.
    public: void SetSurfaceModel(const SurfaceModel _model);

    /// \brief Get the geodetic latitude at the origin reference frame,
    /// specified in units of degrees.
    /// \return Latitude in degrees
    public: double Latitude() const;

    /// \brief Set the geodetic latitude at the origin reference frame,
    /// specified in units of degrees.
    /// \param[in] _lat Latitude in degrees
    public: void SetLatitude(const double _lat) const;

    /// \brief Get the longitude at origin of reference frame,
    /// specified in units of degrees.
    /// \return Longitude in degrees.
    public: double Longitude() const;

    /// \brief Set the longitude at origin of reference frame,
    /// specified in units of degrees.
    /// \param[in] _lon Longitude in degrees.
    public: void SetLongitude(const double _lon) const;

    /// \brief Get the elevation of the origin frame, specified in meters.
    /// \return Elevation at the origin.
    public: double Elevation() const;

    /// \brief Set the elevation of the origin frame, specified in meters.
    /// \param[in] _elevation Elevation at the origin.
    public: void SetElevation(const double _elevation) const;

    /// \brief Heading offset of the origin reference frame,
    /// measured as angle between the world frame and the
    /// WorldFrameorientation type (ENU/NED/NWU). Rotations about the
    /// downward-vector (e.g. North to East) are positive. The direction of
    /// rotation is chosen to be consistent with compass heading convention
    /// (e.g. 0 degrees points North and 90 degrees points East,
    /// positive rotation indicates counterclockwise rotation when viewed from
    /// top-down direction).
    /// \return Heading offset of the origin reference frame.
    public: ignition::math::Angle Heading() const;

    /// \brief Set the heading offset.
    /// \sa ignition::math::Angle Heading()
    /// \param[in] _heading Heading offset.
    public: void SetHeading(const ignition::math::Angle _heading) const;

    /// \brief Private data pointer.
    private: SphericalCoordinatesPrivate *dataPtr = nullptr;
  };
}
#endif
