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
#ifndef SDF_MAGNETOMETER_HH_
#define SDF_MAGNETOMETER_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class MagnetometerPrivate;

  /// \brief Magnetometer contains information about a magnetometer sensor.
  /// This sensor can be attached to a link.
  class SDFORMAT_VISIBLE Magnetometer
  {
    /// \brief Default constructor
    public: Magnetometer();

    /// \brief Copy constructor
    /// \param[in] _magnetometer Magnetometer to copy.
    public: Magnetometer(const Magnetometer &_magnetometer);

    /// \brief Move constructor
    /// \param[in] _magnetometer Magnetometer to move.
    public: Magnetometer(Magnetometer &&_magnetometer) noexcept;

    /// \brief Destructor
    public: ~Magnetometer();

    /// \brief Assignment operator.
    /// \param[in] _magnetometer The magnetometer to set values from.
    /// \return *this
    public: Magnetometer &operator=(const Magnetometer &_magnetometer);

    /// \brief Move assignment operator.
    /// \param[in] _magnetometer The magnetometer to set values from.
    /// \return *this
    public: Magnetometer &operator=(Magnetometer &&_magnetometer);

    /// \brief Load the magnetometer based on an element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Get the noise values related to the body-frame x axis.
    /// \return Noise values for the x axis.
    public: const Noise &XNoise() const;

    /// \brief Set the noise values related to the body-frame x axis.
    /// \param[in] _noise Noise values for the x axis.
    public: void SetXNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame y axis.
    /// \return Noise values for the y axis.
    public: const Noise &YNoise() const;

    /// \brief Set the noise values related to the body-frame y axis.
    /// \param[in] _noise Noise values for the y axis.
    public: void SetYNoise(const Noise &_noise);

    /// \brief Get the noise values related to the body-frame z axis.
    /// \return Noise values for the z axis.
    public: const Noise &ZNoise() const;

    /// \brief Set the noise values related to the body-frame z axis.
    /// \param[in] _noise Noise values for the z axis.
    public: void SetZNoise(const Noise &_noise);

    /// \brief Return true if both Magnetometer objects contain the same values.
    /// \param[_in] _mag Magnetometer value to compare.
    /// \returen True if 'this' == _mag.
    public: bool operator==(const Magnetometer &_mag) const;

    /// \brief Return true this Magnetometer object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _mag Magnetometer value to compare.
    /// \returen True if 'this' != _mag.
    public: bool operator!=(const Magnetometer &_mag) const;

    /// \brief Private data pointer.
    private: MagnetometerPrivate *dataPtr;
  };
  }
}
#endif
