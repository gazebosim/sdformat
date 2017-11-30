#ifndef SDF_DOM_SPHERICALCOORDINATES_HH_
#define SDF_DOM_SPHERICALCOORDINATES_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class SphericalCoordinatesPrivate;

  /// \brief Spherical coordinates encapsulates information to specify
  /// a location and heading on Earth. The information provided can be used
  /// by a GPS sensor, or magnetic field sensor.
  class SDFORMAT_VISIBLE SphericalCoordinates
  {
    /// \brief Default constructor
    public: SphericalCoordinates();

    /// \brief Destructor
    public: ~SphericalCoordinates();

    /// \brief Load the spherical coordinates based on a element pointer.
    /// This is *not* the usual entry point. Typical usage of the SDF DOM is
    /// through the Root object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return True if the sdf pointer was parsed without errors.
    public: bool Load(ElementPtr _sdf);

    /// \brief Get the name of planetary surface model, used to determine the
    /// surface altitude at a given latitude and longitude. The default is an
    /// ellipsoid model of the earth based on the WGS-84 standard. It can be
    /// used in a GPS sensor.
    /// The default value is EARTH_WGS84.
    /// \return The surface model.
    public: std::string SurfaceModel();

    /// \brief This identifies how the world frame is aligned in the
    /// geographical sense. The final world frame orientation is obtained by
    /// rotating a frame aligned with following notation by the Heading()
    /// (Note that Heading() corresponds to positive yaw
    /// rotation in the NED frame, so it's inverse specifies positive
    /// Z-rotation in ENU or NWU).
    ///
    /// Options are:
    ///   - ENU (East-North-Up)
    ///   - NED (North-East-Down)
    ///   - NWU (North-West-Up)
    ///
    /// For example, world frame specified by setting WorldOrientation="ENU"
    /// and Heading=-90° is effectively equivalent to NWU with heading of 0°.
    /// \return The world frame orientation.
    public: std::string WorldFrameOrientation();

    /// \brief Latitude at origin of the reference frame, specified
    /// in units of degrees.
    /// \return Latitude in degrees.
    public: double Latitude();

    /// \brief Longitude at origin of the reference frame, specified in units
    /// of degrees.
    /// \return Longitude in degrees.
    public: double Longitude();

    /// \brief Elevation at origin of the reference frame, specified in meters.
    /// \return Elevation in meters.
    public: double Elevation();

    /// \brief Heading offset of the reference frame, measured as
    /// angle between the world frame and the WorldFrameOrientation
    /// (ENU/NED/NWU). Rotations about the downward-vector (e.g. North to East)
    /// are positive. The direction of rotation is chosen to be consistent
    /// with compass heading convention (e.g. 0 degrees points North and 90
    /// degrees points East, positive rotation indicates
    /// counterclockwise rotation when viewed from top-down direction).
    /// The angle is specified in degrees.
    /// \return The heading in degrees.
    public: double Heading();

    /// \brief Private data pointer.
    private: SphericalCoordinatesPrivate *dataPtr;
  };
}
#endif
