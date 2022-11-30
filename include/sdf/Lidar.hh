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
#ifndef SDF_LIDAR_HH_
#define SDF_LIDAR_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

#include <gz/math/Angle.hh>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class LidarPrivate;

  /// \brief Lidar contains information about a Lidar sensor.
  /// This sensor can be attached to a link. The Lidar sensor can be defined
  /// SDF XML using either the "ray" or "lidar" types. The "lidar" type is
  /// preffered as "ray" is considered legacy.
  ///
  /// # Example SDF XML using lidar type:
  ///
  /// ~~~{.xml}
  /// <sensor name="lidar_sensor" type="lidar">
  ///     <pose>1 2 3 0 0 0</pose>
  ///     <lidar>
  ///         <scan>
  ///             <horizontal>
  ///                 <samples>320</samples>
  ///                 <resolution>0.9</resolution>
  ///                 <min_angle>1.75</min_angle>
  ///                 <max_angle>2.94</max_angle>
  ///             </horizontal>
  ///             <vertical>
  ///                 <samples>240</samples>
  ///                 <resolution>0.8</resolution>
  ///                 <min_angle>2.75</min_angle>
  ///                 <max_angle>3.94</max_angle>
  ///             </vertical>
  ///         </scan>
  ///         <range>
  ///             <min>1.23</min>
  ///             <max>4.56</max>
  ///             <resolution>7.89</resolution>
  ///         </range>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///     </lidar>
  /// </sensor>
  /// ~~~
  ///
  /// # Example SDF XML using ray type:
  ///
  /// ~~~{.xml}
  /// <sensor name="ray_sensor" type="lidar">
  ///     <pose>1 2 3 0 0 0</pose>
  ///     <ray>
  ///         <scan>
  ///             <horizontal>
  ///                 <samples>320</samples>
  ///                 <resolution>0.9</resolution>
  ///                 <min_angle>1.75</min_angle>
  ///                 <max_angle>2.94</max_angle>
  ///             </horizontal>
  ///             <vertical>
  ///                 <samples>240</samples>
  ///                 <resolution>0.8</resolution>
  ///                 <min_angle>2.75</min_angle>
  ///                 <max_angle>3.94</max_angle>
  ///             </vertical>
  ///         </scan>
  ///         <range>
  ///             <min>1.23</min>
  ///             <max>4.56</max>
  ///             <resolution>7.89</resolution>
  ///         </range>
  ///         <noise type="gaussian">
  ///             <mean>0.98</mean>
  ///             <stddev>0.76</stddev>
  ///         </noise>
  ///     </ray>
  /// </sensor>
  /// ~~~
  class SDFORMAT_VISIBLE Lidar
  {
    /// \brief Default constructor
    public: Lidar();

    /// \brief Copy constructor
    /// \param[in] _lidar Lidar to copy.
    public: Lidar(const Lidar &_lidar);

    /// \brief Move constructor
    /// \param[in] _lidar Lidar to move.
    public: Lidar(Lidar &&_lidar) noexcept;

    /// \brief Destructor
    public: ~Lidar();

    /// \brief Assignment operator
    /// \param[in] _lidar The lidar to set values from.
    /// \return *this
    public: Lidar &operator=(const Lidar &_lidar);

    /// \brief Move assignment operator
    /// \param[in] _lidar The lidar to set values from.
    /// \return *this
    public: Lidar &operator=(Lidar &&_lidar) noexcept;

    /// \brief Load the lidar based on an element pointer. This is *not*
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

    /// \brief Get the number of lidar rays horizontally to generate per laser
    /// sweep.
    /// \return Number of lidar rays horizontally per laser sweep.
    public: unsigned int HorizontalScanSamples() const;

    /// \brief Set the number of lidar rays horizontally to generate per laser
    /// sweep.
    /// \param[in] Number of lidar rays horizontally per laser sweep.
    public: void SetHorizontalScanSamples(unsigned int _samples);

    /// \brief Get the resolution for horizontal scan.
    /// \return Resolution for horizontal scan.
    public: double HorizontalScanResolution() const;

    /// \brief Set the resolution for horizontal scan.
    /// \param[in] Resolution for horizontal scan.
    public: void SetHorizontalScanResolution(double _res);

    /// \brief Get the minimum angle for horizontal scan.
    /// \return Minimum angle for horizontal scan.
    public: gz::math::Angle HorizontalScanMinAngle() const;

    /// \brief Set the minimum angle for horizontal scan.
    /// \param[in] Minimum angle for horizontal scan.
    public: void SetHorizontalScanMinAngle(const gz::math::Angle &_min);

    /// \brief Get the maximum angle for horizontal scan.
    /// \return Maximum angle for horizontal scan.
    public: gz::math::Angle HorizontalScanMaxAngle() const;

    /// \brief Set the maximum angle for horizontal scan.
    /// \param[in] Maximum angle for horizontal scan.
    public: void SetHorizontalScanMaxAngle(const gz::math::Angle &_max);

    /// \brief Get the number of lidar rays vertically to generate per laser
    /// sweep.
    /// \return Number of lidar rays vertically per laser sweep.
    public: unsigned int VerticalScanSamples() const;

    /// \brief Set the number of lidar rays vertically to generate per laser
    /// sweep.
    /// \param[in] Number of lidar rays vertically per laser sweep.
    public: void SetVerticalScanSamples(unsigned int _samples);

    /// \brief Get the resolution for vertical scan.
    /// \return Resolution for vertical scan.
    public: double VerticalScanResolution() const;

    /// \brief Set the resolution for vertical scan.
    /// \param[in] Resolution for vertical scan.
    public: void SetVerticalScanResolution(double _res);

    /// \brief Get the minimum angle for vertical scan.
    /// \return Minimum angle for vertical scan.
    public: gz::math::Angle VerticalScanMinAngle() const;

    /// \brief Set the minimum angle for vertical scan.
    /// \param[in] Minimum angle for vertical scan.
    public: void SetVerticalScanMinAngle(const gz::math::Angle &_min);

    /// \brief Get the maximum angle for vertical scan.
    /// \return Maximum angle for vertical scan.
    public: gz::math::Angle VerticalScanMaxAngle() const;

    /// \brief Set the maximum angle for vertical scan.
    /// \param[in] Maximum angle for vertical scan.
    public: void SetVerticalScanMaxAngle(const gz::math::Angle &_max);

    /// \brief Get minimum distance for each lidar ray.
    /// \return Minimum distance for each lidar ray.
    public: double RangeMin() const;

    /// \brief Set minimum distance for each lidar ray.
    /// \param[in] Minimum distance for each lidar ray.
    public: void SetRangeMin(double _min);

    /// \brief Get maximum distance for each lidar ray.
    /// \return Maximum distance for each lidar ray.
    public: double RangeMax() const;

    /// \brief Set maximum distance for each lidar ray.
    /// \param[in] Maximum distance for each lidar ray.
    public: void SetRangeMax(double _max);

    /// \brief Get linear resolution of each lidar ray.
    /// \return Linear resolution for each lidar ray.
    public: double RangeResolution() const;

    /// \brief Set linear resolution of each lidar ray.
    /// \param[in] Linear resolution for each lidar ray.
    public: void SetRangeResolution(double _range);

    /// \brief Get the noise values for the lidar sensor.
    /// \return Noise values for the lidar sensor.
    public: const Noise &LidarNoise() const;

    /// \biref Set the noise values for the lidar sensor.
    /// \param[in] _noise Noise values for the lidar sensor.
    public: void SetLidarNoise(const Noise &_noise);

    /// \brief Return true if both Lidar objects contain the same values.
    /// \param[_in] _lidar Lidar value to compare.
    /// \return True if 'this' == _lidar.
    public: bool operator==(const Lidar &_lidar) const;

    /// \brief Return true this Lidar object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _lidar Lidar value to compare.
    /// \return True if 'this' != _lidar.
    public: bool operator!=(const Lidar &_lidar) const;

    /// \brief Private data pointer.
    private: LidarPrivate *dataPtr;
  };
  }
}
#endif
