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

#include <string>

#include <gz/math/Color.hh>
#include <gz/utils/ImplPtr.hh>

#include "sdf/Element.hh"
#include "sdf/Types.hh"
#include "sdf/config.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  // Forward declarations.
  class ParserConfig;

  class SDFORMAT_VISIBLE Sky
  {
    /// \brief Default constructor
    public: Sky();

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

    /// \brief Set cloud mean size
    /// \param[in] _size cloud mean size [0..1]
    public: void SetCloudMeanSize(double _size);

    /// \brief Get cloud ambient color
    /// \return cloud ambient color
    public: gz::math::Color CloudAmbient() const;

    /// \brief Set cloud ambient color
    /// \param[in] _ambient cloud ambient color
    public: void SetCloudAmbient(const gz::math::Color &_ambient);

    /// \brief Get the skybox texture URI.
    /// \return The URI of the skybox texture.
    public: const std::string &CubemapUri() const;

    /// \brief Set the skybox texture URI.
    /// \param[in] _uri The URI of the skybox texture.
    public: void SetCubemapUri(const std::string &_uri);

    /// \brief Load the sky based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Load the sky based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf, const ParserConfig &_config);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// sky.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated sky values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Create and return an SDF element filled with data from this
    /// sky.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated sky values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
