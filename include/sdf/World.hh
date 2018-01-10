/*
 * Copyright 2017 Open Source Robotics Foundation
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
#ifndef SDF_WORLD_HH_
#define SDF_WORLD_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class WorldPrivate;

  class SDFORMAT_VISIBLE World
  {
    /// \brief Default constructor
    public: World();

    /// \brief Move constructor
    /// \param[in] _world World to move.
    public: World(World &&_world);

    /// \brief Destructor
    public: ~World();

    /// \brief Load the world based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the world.
    /// \return Name of the world.
    public: std::string Name() const;

    /// \brief Set the name of the world.
    /// \param[in] _name Name of the world.
    public: void SetName(const std::string &_name) const;

    /// \brief Get the audio device name. The audio device can be used to
    /// playback audio files. A value of "default" or an empty string
    /// indicates that the system's default audio device should be used.
    /// \return Audio device name.
    public: std::string AudioDevice() const;

    /// \brief Set the audio device name. See std::string AudioDevice() const
    /// for more information.
    /// \param[in] _device The new audio device name.
    /// \sa std::string AudioDevice() const
    public: void SetAudioDevice(const std::string &_device);

    /// \brief Get the wind linear velocity in the global/world coordinate
    /// frame. Units are meters per second \f$(\frac{m}{s})\f$
    /// \return Linear velocity of wind in the global/world coordinate frame.
    /// \sa void SetWindLinearVelocity(const ignition::math::Vector3d &_wind)
    public: ignition::math::Vector3d WindLinearVelocity() const;

    /// \brief Set the wind linear velocity in the global/world coordinate
    /// frame. Units are meters per second \f$(\frac{m}{s})\f$
    /// \param[in] _win The new linear velocity of wind.
    /// \sa ignition::math::Vector3d WindLinearVelocity() const
    public: void SetWindLinearVelocity(const ignition::math::Vector3d &_wind);

    /// \brief Get the acceleration due to gravity. The default value is
    /// Earth's standard gravity at sea level, which equals
    /// [0, 0, -9.80665] \f$(\frac{m}{s^2})\f$
    /// \return Gravity vector in meters per second squared
    /// \f$(\frac{m}{s^2})\f$
    public: ignition::math::Vector3d Gravity() const;

    /// \brief Set the acceleration due to gravity. Units are meters per
    /// second squared \f$(\frac{m}{s^2})\f$
    /// \param[in] _gravity The new gravity vector.
    public: void SetGravity(const ignition::math::Vector3d &_gravity);

    /// \brief Get the magnetic vector in Tesla, expressed in
    /// a coordinate frame defined by the SphericalCoordinates property.
    /// A spherical coordindate can be specified in SDF using the
    /// <spherical_coordinates> element.
    /// \return Magnetic field vector.
    /// \sa SphericalCoordinates
    public: ignition::math::Vector3d MagneticField() const;

    /// \brief Set the magnetic vector in Tesla, expressed in
    /// a coordinate frame defined by the SphericalCoordinate.
    /// A spherical coordindate can be specified in SDF using the
    /// <spherical_coordinates> element.
    /// \param[in] _mag The new magnetic field vector.
    /// \sa SphericalCoordinates
    public: void SetMagneticField(const ignition::math::Vector3d &_mag);

    /// \brief Private data pointer.
    private: WorldPrivate *dataPtr = nullptr;
  };
}
#endif
