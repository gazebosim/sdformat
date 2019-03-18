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
#ifndef SDF_BATTERY_HH_
#define SDF_BATTERY_HH_

#include <string>
#include "sdf/Element.hh"
#include "sdf/Types.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {

  // Forward declarations.
  class BatteryPrivate;

  /// \brief Information about an SDF battery.
  class SDFORMAT_VISIBLE Battery
  {
    /// \brief Default constructor
    public: Battery();

    /// \brief Copy constructor
    /// \param[in] _battery Battery to copy.
    public: Battery(const Battery &_battery);

    /// \brief Move constructor
    /// \param[in] _battery Battery to move.
    public: Battery(Battery &&_battery);

    /// \brief Destructor
    public: ~Battery();

    /// \brief Assignment operator.
    /// \param[in] _battery The battery to set values from.
    /// \return *this
    public: Battery &operator=(const Battery &_battery);

    /// \brief Load the battery based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the battery.
    /// The name of the battery should be unique within the scope of a World.
    /// \return Name of the battery.
    public: std::string Name() const;

    /// \brief Set the name of the battery.
    /// The name of the battery should be unique within the scope of a World.
    /// \param[in] _name Name of the battery.
    public: void SetName(const std::string &_name);

    /// \brief Get the voltage of the battery.
    /// \return Voltage of the battery.
    public: double Voltage() const;

    /// \brief Set the voltage of the battery.
    /// \param[in] _voltage Voltage of the battery.
    public: void SetVoltage(double _voltage);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: BatteryPrivate *dataPtr = nullptr;
  };
  }
}
#endif
