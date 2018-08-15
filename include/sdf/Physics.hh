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
#ifndef SDF_PHYSICS_HH_
#define SDF_PHYSICS_HH_

#include <string>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class PhysicsPrivate;

  /// \brief The physics element specifies the type and properties of a
  /// dynamics engine.
  class SDFORMAT_VISIBLE Physics
  {
    /// \brief Default constructor
    public: Physics();

    /// \brief Move constructor
    /// \param[in] _physics Physics to move.
    public: Physics(Physics &&_physics) noexcept;

    /// \brief Destructor
    public: ~Physics();

    /// \brief Load the physics based on an element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of this set of physics parameters.
    /// \return Name of the physics profile.
    public: std::string Name() const;

    /// \brief Set the name of this set of physics parameters.
    /// \param[in] _name Name of the physics profile.
    public: void SetName(const std::string &_name) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get whether this physics profile is marked as default.
    /// If true, this physics profile is set as the default physics profile
    /// for the World. If multiple default physics elements exist, the first
    /// physics profile marked as default is chosen. If no default physics
    /// element exists, the first physics element is chosen.
    /// \return True if this profile is the default.
    public: bool IsDefault() const;

    /// \brief Set whether this physics profile is the default.
    /// \param[in] _default True to make this profile default.
    public: void SetDefault(const bool _default) const;

    /// \brief Get the physics profile dynamics engine type.
    /// Current options are ode, bullet, simbody and dart. Defaults to ode if
    /// left unspecified.
    /// \return The type of dynamics engine.
    public: std::string EngineType() const;

    /// \brief Set the physics profile dynamics engine type.
    /// \param[in] _type The type of dynamics engine.
    public: void SetEngineType(const std::string &_type);

    /// \brief Get the max step size in seconds.
    /// The Maximum time step size at which every system in simulation can
    /// interact with the states of the world.
    /// \return The max step size in seconds.
    public: double MaxStepSize() const;

    /// \brief Set the max step size in seconds.
    /// \param[in] _step The max step size in seconds.
    public: void SetMaxStepSize(const double _step);

    /// \brief Get the target real time factor.
    /// Target simulation speedup factor, defined by ratio of simulation time
    /// to real-time.
    /// \return The target real time factor.
    public: double RealTimeFactor() const;

    /// \brief Set the target realtime factor.
    /// \param[in] _factor The target real time factor.
    public: void SetRealTimeFactor(const double _factor);

    /// \brief Private data pointer.
    private: PhysicsPrivate *dataPtr = nullptr;
  };
}
#endif
