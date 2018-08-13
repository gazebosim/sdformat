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
#include <ignition/math/Vector3.hh>

#include "sdf/Atmosphere.hh"
#include "sdf/Element.hh"
#include "sdf/Gui.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class Light;
  class Model;
  class Physics;
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
    /// \param[in] _wind The new linear velocity of wind.
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
    /// A spherical coordinate can be specified in SDF using the
    /// <spherical_coordinates> element.
    /// \return Magnetic field vector.
    /// \sa SphericalCoordinates
    public: ignition::math::Vector3d MagneticField() const;

    /// \brief Set the magnetic vector in Tesla, expressed in
    /// a coordinate frame defined by the SphericalCoordinate.
    /// A spherical coordinate can be specified in SDF using the
    /// <spherical_coordinates> element.
    /// \param[in] _mag The new magnetic field vector.
    /// \sa SphericalCoordinates
    public: void SetMagneticField(const ignition::math::Vector3d &_mag);

    /// \brief Get the number of models.
    /// \return Number of models contained in this World object.
    public: uint64_t ModelCount() const;

    /// \brief Get a model based on an index.
    /// \param[in] _index Index of the model. The index should be in the
    /// range [0..ModelCount()).
    /// \return Pointer to the model. Nullptr if the index does not exist.
    /// \sa uint64_t ModelCount() const
    public: const Model *ModelByIndex(const uint64_t _index) const;

    /// \brief Get whether a model name exists.
    /// \param[in] _name Name of the model to check.
    /// \return True if there exists a model with the given name.
    public: bool ModelNameExists(const std::string &_name) const;

    /// \brief Get the number of lights.
    /// \return Number of lights contained in this World object.
    public: uint64_t LightCount() const;

    /// \brief Get a light based on an index.
    /// \param[in] _index Index of the light. The index should be in the
    /// range [0..LightCount()).
    /// \return Pointer to the light. Nullptr if the index does not exist.
    /// \sa uint64_t LightCount() const
    public: const Light *LightByIndex(const uint64_t _index) const;

    /// \brief Get whether a light name exists.
    /// \param[in] _name Name of the light to check.
    /// \return True if there exists a light with the given name.
    public: bool LightNameExists(const std::string &_name) const;

    /// \brief Get a pointer to the atmosphere model associated with this
    /// world. A nullptr indicates that an atmosphere model has not been set.
    /// \return Pointer to this world's atmosphere model. Nullptr inidicates
    /// that there is no atmosphere model.
    public: const sdf::Atmosphere *Atmosphere() const;

    /// \brief Set the atmosphere model associated with this world.
    /// \param[in] _atmosphere The new atmosphere model for this world.
    public: void SetAtmosphere(const sdf::Atmosphere &_atmosphere) const;

    /// \brief Get a pointer to the Gui associated with this
    /// world. A nullptr indicates that a Gui element has not been specified.
    /// \return Pointer to this world's Gui parameters. Nullptr inidicates
    /// that there are no Gui parameters.
    public: sdf::Gui *Gui() const;

    /// \brief Set the Gui parameters associated with this world.
    /// \param[in] _gui The new Gui parameter for this world
    public: void SetGui(const sdf::Gui &_gui);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the number of physics profiles.
    /// \return Number of physics profiles contained in this World object.
    public: uint64_t PhysicsCount() const;

    /// \brief Get a physics profile based on an index.
    /// \param[in] _index Index of the physics profile.
    /// The index should be in the range [0..PhysicsCount()).
    /// \return Pointer to the physics profile. Nullptr if the index does not
    /// exist.
    ///// \sa uint64_t PhysicsCount() const
    public: const Physics *PhysicsByIndex(const uint64_t _index) const;

    /// \brief Get the default physics profile.
    /// \return Pointer to the default physics profile.
    public: const Physics *PhysicsDefault() const;

    /// \brief Get whether a physics profile name exists.
    /// \param[in] _name Name of the physics profile to check.
    /// \return True if there exists a physics profile with the given name.
    public: bool PhysicsNameExists(const std::string &_name) const;

    /// \brief Private data pointer.
    private: WorldPrivate *dataPtr = nullptr;
  };
}
#endif
