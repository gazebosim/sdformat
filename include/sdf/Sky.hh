/*
 * Copyright 2020 Open Source Robotics Foundation
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

#ifndef SDF_SKY_HH_
#define SDF_SKY_HH_

#include <gz/math/Color.hh>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class SkyPrivate;

  class SDFORMAT_VISIBLE Sky
  {
    /// \brief Default constructor
    public: Sky();

    /// \brief Copy constructor
    /// \param[in] _sky Sky element to copy.
    public: Sky(const Sky &_sky);

    /// \brief Move constructor
    /// \param[in] _sky Sky to move.
    public: Sky(Sky &&_sky) noexcept;

    /// \brief Destructor
    public: ~Sky();

    /// \brief Assignment operator.
    /// \param[in] _sky The sky to set values from.
    /// \return *this
    public: Sky &operator=(const Sky &_sky);

    /// \brief Move assignment operator.
    /// \param[in] _workflow The sky to move from.
    /// \return *this
    public: Sky &operator=(Sky &&_sky);

    /// \brief Get time of day [0..24]
    /// \return Time of day
    public: double Time() const;

    /// \brief Set time of day
    /// \param[in] _time Time of day [0..24]
    public: void SetTime(double _time);

    /// \brief Get sunrise time
    /// \return sunrise time [0..24]
    public: double Sunrise() const;

    /// \brief Set Sunrise time
    /// \param[in] _time Sunrise time [0..24]
    public: void SetSunrise(double _time);

    /// \brief Get sunset time
    /// \return sunset time [0..24]
    public: double Sunset() const;

    /// \brief Set Sunset time
    /// \param[in] _time Sunset time [0..24]
    public: void SetSunset(double _time);

    /// \brief Get cloud speed
    /// \return cloud speed in meters per second
    public: double CloudSpeed() const;

    /// \brief Set cloud speed
    /// \param[in] _speed cloud speed in meters per second.
    public: void SetCloudSpeed(double _speed);

    /// \brief Get cloud direction angle (angle around up axis)
    /// \return cloud direction angle in world frame
    public: gz::math::Angle CloudDirection() const;

    /// \brief Set cloud direction angle (angle around up axis)
    /// \param[in] _angle Cloud direction angle in world frame.
    public: void SetCloudDirection(const gz::math::Angle &_angle);

    /// \brief Get cloud humidity
    /// \return cloud humidity [0..1]
    public: double CloudHumidity() const;

    /// \brief Set cloud humidity
    /// \param[in] _humidity cloud humidity [0..1]
    public: void SetCloudHumidity(double _humidity);

    /// \brief Get cloud mean size
    /// \return cloud mean size [0..1]
    public: double CloudMeanSize() const;

    /// \brief Set cloud mean siz
    /// \param[in] _size cloud mean size [0..1]
    public: void SetCloudMeanSize(double _size);

    /// \brief Get cloud ambient color
    /// \return cloud ambient color
    public: gz::math::Color CloudAmbient() const;

    /// \brief Set cloud ambient color
    /// \param[in] _ambient cloud ambient color
    public: void SetCloudAmbient(const gz::math::Color &_ambient);

    /// \brief Load the sky based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
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

    /// \brief Private data pointer.
    private: SkyPrivate *dataPtr = nullptr;
  };
  }
}
#endif
