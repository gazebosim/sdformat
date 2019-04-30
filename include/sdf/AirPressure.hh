/*
 * Copyright 2019 Open Source Robotics Foundation
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
#ifndef SDF_AIRPRESSURE_HH_
#define SDF_AIRPRESSURE_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class AirPressurePrivate;

  /// \brief AirPressure contains information about a general
  /// purpose fluid pressure sensor.
  /// This sensor can be attached to a link.
  class SDFORMAT_VISIBLE AirPressure
  {
    /// \brief Default constructor
    public: AirPressure();

    /// \brief Copy constructor
    /// \param[in] _airPressure AirPressure to copy.
    public: AirPressure(const AirPressure &_sensor);

    /// \brief Move constructor
    /// \param[in] _airPressure AirPressure to move.
    public: AirPressure(AirPressure &&_sensor);

    /// \brief Destructor
    public: ~AirPressure();

    /// \brief Assignment operator.
    /// \param[in] _airPressure The airPressure to set values
    /// from.
    /// \return *this
    public: AirPressure &operator=(const AirPressure &_sensor);

    /// \brief Move assignment operator.
    /// \param[in] _airPressure The airPressure to set values
    /// from.
    /// \return *this
    public: AirPressure &operator=(AirPressure &&_sensor);

    /// \brief Load the airPressure based on an element pointer.
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

    /// \brief Get the reference altitude of the sensor in meters. This value
    /// can be used by a sensor implementation to augment the altitude of the
    /// sensor. For example, if you are using simulation instead of creating a
    /// 1000 m mountain model on which to place your sensor, you could instead
    /// set this value to 1000 and place your model on a ground plane with a Z
    /// height of zero.
    /// \return Reference altitude in meters.
    public: double ReferenceAltitude() const;

    /// \brief Set the reference altitude of the sensor in meters.
    /// \sa ReferenceAltitude()
    /// \param[in] _ref Reference altitude in meters.
    public: void SetReferenceAltitude(double _ref);

    /// \brief Get the noise values.
    /// \return Noise values for pressure data.
    public: const Noise &PressureNoise() const;

    /// \brief Set the noise values related to the pressure data.
    /// \param[in] _noise Noise values for the pressure data.
    public: void SetPressureNoise(const Noise &_noise);

    /// \brief Return true if both AirPressure objects contain the
    /// same values.
    /// \param[_in] _mag AirPressure value to compare.
    /// \returen True if 'this' == _mag.
    public: bool operator==(const AirPressure &_air) const;

    /// \brief Return true this AirPressure object does not contain
    /// the same values as the passed in parameter.
    /// \param[_in] _mag AirPressure value to compare.
    /// \returen True if 'this' != _mag.
    public: bool operator!=(const AirPressure &_air) const;

    /// \brief Private data pointer.
    private: AirPressurePrivate *dataPtr;
  };
  }
}
#endif
