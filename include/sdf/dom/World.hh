#ifndef SDF_DOM_WORLD_HH_
#define SDF_DOM_WORLD_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class WorldPrivate;

  class SDFORMAT_VISIBLE World
  {
    /// \brief Default constructor
    public: World();

    /// \brief Destructor
    public: ~World();

    /// \brief Load the world based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return True when no errors were encountered.
    public: bool Load(ElementPtr _sdf);

    /// \brief Print debug information to standard out.
    /// \param[in] _prefix String to prefix all output.
    public: void Print(const std::string &_prefix = "") const;

    /// \brief Get the name of the world.
    /// \return Name of the world.
    public: std::string Name() const;

    /// \brief Get the audio device name. The audio device can be used to
    /// playback audio files. A value of "default" or an empty string
    /// indicates that the system  default audio device should be used.
    /// \return Audio device name.
    public: std::string AudioDevice() const;

    /// \brief Get the wind linear velocity.
    /// \return Linear velocity of wind.
    public: ignition::math::Vector3d WindLinearVelocity() const;

    /// \brief Get the acceleration due to gravity. The default value is
    /// Earth's standard gravity at sea level, which equals
    /// [0, 0, -9.80665] m/s^2.
    /// \return Gravity vector.
    public: ignition::math::Vector3d Gravity() const;

    /// \brief Get the magnetic vector in Tesla, expressed in
    /// a coordinate frame defined by the SphericalCoordinate.
    /// \return Magnetic field vector.
    public: ignition::math::Vector3d MagneticField() const;

    /// \brief Private data pointer.
    private: WorldPrivate *dataPtr;
  };
}
#endif
