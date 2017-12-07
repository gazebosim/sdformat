#ifndef SDF_DOM_SPHERICALCOORDINATES_HH_
#define SDF_DOM_SPHERICALCOORDINATES_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class SphericalCoordinatesPrivate;

  /// \enum SphericalCoordinatesWorldFrameOrientation
  /// \brief This enum contains different world frame orientations.
  enum class SphericalCoordinatesWorldFrameOrientation
  {
    /// \brief East-North-Up
    ENU,

    /// \brief North-East-Down
    NED,

    /// \brief North-West-Up
    NWU,

    /// \brief An unknown or undefined world frame orientation
    UNKNOWN
  };

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
    /// \sa void SurfaceModel(const std::string &_surfaceMode)
    public: std::string SurfaceModel() const;

    /// \brief Set the surface model.
    /// \param[in] _surfaceModel Name of the surface model.
    /// \sa std::string SurfaceModel() const.
    public: void SetSurfaceModel(const std::string &_surfaceModel);

    /// \brief This identifies how the world frame is aligned in the
    /// geographical sense. The final world frame orientation is obtained by
    /// rotating a frame aligned with following notation by the Heading()
    /// (Note that Heading() corresponds to positive yaw
    /// rotation in the NED frame, so it's inverse specifies positive
    /// Z-rotation in ENU or NWU).
    ///
    /// For example, world frame specified by setting WorldOrientation=ENU
    /// and Heading=-90° is effectively equivalent to NWU with heading of 0°.
    /// \return The world frame orientation.
    /// \sa SphericalCoordinatesWorldFrameOrientation
    public: SphericalCoordinatesWorldFrameOrientation
            WorldFrameOrientation() const;

    /// \brief Get a string version of WorldFrameOrientation().
    /// \return A string that represents the world frame orientation.
    public: std::string WorldFrameOrientationName() const;

    /// \brief Set the world frame orientation.
    /// \param[in] _orientation The world frame orientation.
    /// \sa SphericalCoordinatesWorldFrameOrientation
    ///     WorldFrameOrientation() const
    public: void SetWorldFrameOrientation(
                const SphericalCoordinatesWorldFrameOrientation _orientation);

    /// \brief Latitude at origin of the reference frame, specified
    /// in units of degrees.
    /// \return Latitude in degrees.
    public: double Latitude() const;

    /// \brief Set the latitude at origin of the reference frame, specified
    /// in units of degrees.
    /// \param[in] _latitude Latitude in degrees.
    /// \sa double Latitude()
    public: void SetLatitude(const double _latitude);

    /// \brief Longitude at origin of the reference frame, specified in units
    /// of degrees.
    /// \return Longitude in degrees.
    public: double Longitude() const;

    /// \brief Set the longitude at origin of the reference frame, specified
    /// in units of degrees.
    /// \param[in] _longitude Longitude in degrees.
    public: void SetLongitude(const double _longitude);

    /// \brief Elevation at origin of the reference frame, specified in meters.
    /// \return Elevation in meters.
    public: double Elevation() const;

    /// \brief Elevation at origin of the reference frame, specified in meters.
    /// \return Elevation in meters.
    public: void SetElevation(const double _elevation);

    /// \brief Heading offset of the reference frame, measured as
    /// angle between the world frame and the WorldFrameOrientation
    /// (ENU/NED/NWU). Rotations about the downward-vector (e.g. North to East)
    /// are positive. The direction of rotation is chosen to be consistent
    /// with compass heading convention (e.g. 0 degrees points North and 90
    /// degrees points East, positive rotation indicates
    /// counterclockwise rotation when viewed from top-down direction).
    /// The angle is specified in degrees.
    /// \return The heading in degrees.
    public: double Heading() const;

    /// Set the heading offset of the reference frame. See double Heading()
    /// for more information.
    /// \param[in] _heading New heading.
    /// \sa double Heading() const
    public: void SetHeading(const double _heading);

    /// \brief Private data pointer.
    private: SphericalCoordinatesPrivate *dataPtr;
  };
}
#endif
