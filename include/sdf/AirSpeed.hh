/*
 * Copyright 2023 Open Source Robotics Foundation
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
#ifndef SDF_AIRSPEED_HH_
#define SDF_AIRSPEED_HH_

#include <gz/utils/ImplPtr.hh>

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/config.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  /// \brief AirSpeed contains information about a general
  /// purpose air speed sensor.
  /// This sensor can be attached to a link.
  class SDFORMAT_VISIBLE AirSpeed
  {
    /// \brief Default constructor
    public: AirSpeed();

    /// \brief Load the air speed based on an element pointer.
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

    /// \brief Get the noise values.
    /// \return Noise values for differential pressure data.
    public: const Noise &PressureNoise() const;

    /// \brief Set the noise values related to the differential pressure data.
    /// \param[in] _noise Noise values for the pressure data.
    public: void SetPressureNoise(const Noise &_noise);

    /// \brief Return true if both AirSpeed objects contain the
    /// same values.
    /// \param[_in] _air AirSpeed value to compare.
    /// \return True if 'this' == _air.
    public: bool operator==(const AirSpeed &_air) const;

    /// \brief Return true this AirSpeed object does not contain
    /// the same values as the passed in parameter.
    /// \param[_in] _air AirSpeed value to compare.
    /// \return True if 'this' != _air.
    public: bool operator!=(const AirSpeed &_air) const;

    /// \brief Create and return an SDF element filled with data from this
    /// air pressure sensor.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated sensor values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
