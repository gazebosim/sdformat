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
#ifndef SDF_ATMOSPHERE_HH_
#define SDF_ATMOSPHERE_HH_

#include <ignition/math/Temperature.hh>
#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  /// \enum AtmosphereType
  /// \brief The set of atmosphere model types.
  enum class AtmosphereType
  {
    /// \brief Adiabatic atmosphere model.
    ADIABATIC = 0,
  };

  // Forward declarations.
  class AtmospherePrivate;

  /// \brief The Atmosphere class contains information about
  /// an atmospheric model and related parameters such as temperature
  /// and pressure at sea level. An Atmosphere instance is optionally part of
  /// a World.
  class SDFORMAT_VISIBLE Atmosphere
  {
    /// \brief Default constructor
    public: Atmosphere();

    /// \brief Copy constructor
    /// \param[in] _atmosphere Atmosphere to copy.
    public: Atmosphere(const Atmosphere &_atmosphere);

    /// \brief Move constructor
    /// \param[in] _atmosphere Atmosphere to move.
    public: Atmosphere(Atmosphere &&_atmosphere);

    /// \brief Destructor
    public: ~Atmosphere();

    /// \brief Load the atmosphere based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the type of the atmospheric model.
    /// \return The type of atmosphere engine, which defaults to adiabatic.
    public: AtmosphereType Type() const;

    /// \brief Set the type of the atmospheric model.
    /// \param[in] _type The type of atmosphere engine.
    public: void SetType(const AtmosphereType _type);

    /// \brief Get the temperature at sea level.
    /// \return The temperature at sea level.
    public: ignition::math::Temperature Temperature() const;

    /// \brief Set the temperature at sea level.
    /// \param[in] _temp The temperature at sea level.
    public: void SetTemperature(const ignition::math::Temperature &_temp);

    /// \brief Get the temperature gradient with respect to increasing
    /// altitude in units of K/m.
    /// \return The temperature gradient in K/m.
    public: double TemperatureGradient() const;

    /// \brief Set the temperature gradient with respect to increasing
    /// altitude in units of K/m.
    /// \param[in] _gradient The temperature gradient in K/m.
    public: void SetTemperatureGradient(const double _gradient);

    /// \brief Get the pressure at sea level in pascals.
    /// \return The pressure at sea level in pascals.
    public: double Pressure() const;

    /// \brief Set the pressure at sea level in pascals.
    /// \param[in] _pressure The pressure at sea level in pascals.
    public: void SetPressure(const double _pressure);

    /// \brief Equality operator that returns true if this atmosphere
    /// instance equals the given atmosphere instance.
    /// \param[in] _atmosphere Atmosphere instance to compare.
    /// \return True if this instance equals the given atmosphere.
    public: bool operator==(const Atmosphere &_atmosphere);

    /// \brief Private data pointer.
    private: AtmospherePrivate *dataPtr = nullptr;
  };
}
#endif
