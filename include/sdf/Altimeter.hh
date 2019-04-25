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
#ifndef SDF_ALTIMETER_HH_
#define SDF_ALTIMETER_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class AltimeterPrivate;

  /// \brief Altimeter contains information about an altimeter sensor.
  /// This sensor can be attached to a link.
  class SDFORMAT_VISIBLE Altimeter
  {
    /// \brief Default constructor
    public: Altimeter();

    /// \brief Copy constructor
    /// \param[in] _altimeter Altimeter to copy.
    public: Altimeter(const Altimeter &_altimeter);

    /// \brief Move constructor
    /// \param[in] _altimeter Altimeter to move.
    public: Altimeter(Altimeter &&_altimeter) noexcept;

    /// \brief Destructor
    public: ~Altimeter();

    /// \brief Assignment operator.
    /// \param[in] _altimeter The altimeter to set values from.
    /// \return *this
    public: Altimeter &operator=(const Altimeter &_altimeter);

    /// \brief Move assignment operator.
    /// \param[in] _altimeter The altimeter to set values from.
    /// \return *this
    public: Altimeter &operator=(Altimeter &&_altimeter) noexcept;

    /// \brief Load the altimeter based on an element pointer. This is *not*
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

    /// \brief Get the noise values related to the vertical position.
    /// \return Noise values for the vertical position.
    public: const Noise &VerticalPositionNoise() const;

    /// \brief Set the noise values related to the vertical position.
    /// \param[in] _noise Noise values for the vertical position.
    public: void SetVerticalPositionNoise(const Noise &_noise);

    /// \brief Get the noise values related to the vertical velocity.
    /// \return Noise values for the vertical velocity.
    public: const Noise &VerticalVelocityNoise() const;

    /// \brief Set the noise values related to the vertical velocity.
    /// \param[in] _noise Noise values for the vertical velocity.
    public: void SetVerticalVelocityNoise(const Noise &_noise);

    /// \brief Return true if both Altimeter objects contain the same values.
    /// \param[_in] _alt Altimeter value to compare.
    /// \returen True if 'this' == _alt.
    public: bool operator==(const Altimeter &_alt) const;

    /// \brief Return true this Altimeter object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _alt Altimeter value to compare.
    /// \returen True if 'this' != _alt.
    public: bool operator!=(const Altimeter &_alt) const;

    /// \brief Private data pointer.
    private: AltimeterPrivate *dataPtr;
  };
  }
}
#endif
