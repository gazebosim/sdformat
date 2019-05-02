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
#ifndef SDF_RAY_HH_
#define SDF_RAY_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/Noise.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class RayPrivate;

  /// \brief Ray contains information about a ray sensor.
  /// This sensor can be attached to a link
  class SDFORMAT_VISIBLE Ray
  {
    /// \brief Default constructor
    public: Ray();

    /// \brief Copy constructor
    /// \param[in] _ray Ray to copy.
    public: Ray(const Ray &_ray);

    /// \brief Move constructor
    /// \param[in] _ray Ray to move.
    public: Ray(Ray &&_ray) noexcept;

    /// \brief Destructor
    public: ~Ray();

    /// \brief Assignment operator
    /// \param[in] _ray The ray to set values from.
    /// \return *this
    public: Ray &operator=(const Ray &_ray);

    /// \brief Move assignment operator
    /// \param[in] _ray The ray to set values from.
    /// \return *this
    public: Ray &operator=(Ray &&_ray) noexcept;

    /// \brief Load the ray based on an element pointer. This is *not*
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

    /// \brief Get the number of rays horizontally to generate per laser sweep.
    /// \return Number of rays horizontally per laser sweep.
    public: unsigned int HorizontalScanSamples() const;

    /// \brief Set the number of rays horizontally to generate per laser sweep.
    /// \param[in] Number of rays horizontally per laser sweep.
    public: void SetHorizontalScanSamples(unsigned int _samples);

    /// \brief Get the resolution for horizontal scan.
    /// \return Resolution for horizontal scan.
    public: double HorizontalScanResolution() const;

    /// \brief Set the resolution for horizontal scan.
    /// \param[in] Resolution for horizontal scan.
    public: void SetHorizontalScanResolution(double _res);

    /// \brief Get the minimum angle for horizontal scan.
    /// \return Minimum angle for horizontal scan.
    public: double HorizontalScanMinAngle() const;

    /// \brief Set the minimum angle for horizontal scan.
    /// \param[in] Minimum angle for horizontal scan.
    public: void SetHorizontalScanMinAngle(double _min);

    /// \brief Get the maximum angle for horizontal scan.
    /// \return Maximum angle for horizontal scan.
    public: double HorizontalScanMaxAngle() const;

    /// \brief Set the maximum angle for horizontal scan.
    /// \param[in] Maximum angle for horizontal scan.
    public: void SetHorizontalScanMaxAngle(double _max);

    /// \brief Get the number of rays horizontally to generate per laser sweep.
    /// \return Number of rays horizontally per laser sweep.
    public: unsigned int VerticalScanSamples() const;

    /// \brief Set the number of rays vertically to generate per laser sweep.
    /// \param[in] Number of rays vertically per laser sweep.
    public: void SetVerticalScanSamples(unsigned int _samples);

    /// \brief Get the resolution for vertical scan.
    /// \return Resolution for vertical scan.
    public: double VerticalScanResolution() const;

    /// \brief Set the resolution for vertical scan.
    /// \param[in] Resolution for vertical scan.
    public: void SetVerticalScanResolution(double _res);

    /// \brief Get the minimum angle for vertical scan.
    /// \return Minimum angle for vertical scan.
    public: double VerticalScanMinAngle() const;

    /// \brief Set the minimum angle for vertical scan.
    /// \param[in] Minimum angle for vertical scan.
    public: void SetVerticalScanMinAngle(double _min);

    /// \brief Get the maximum angle for vertical scan.
    /// \return Maximum angle for vertical scan.
    public: double VerticalScanMaxAngle() const;

    /// \brief Set the maximum angle for vertical scan.
    /// \param[in] Maximum angle for vertical scan.
    public: void SetVerticalScanMaxAngle(double _max);

    /// \brief Get minimum distance for each ray.
    /// \return Minimum distance for each ray.
    public: double MinRange() const;

    /// \brief Set minimum distance for each ray.
    /// \param[in] Minimum distance for each ray.
    public: void SetMinRange(double _min);

    /// \brief Get maximum distance for each ray.
    /// \return Maximum distance for each ray.
    public: double MaxRange() const;

    /// \brief Set maximum distance for each ray.
    /// \param[in] Maximum distance for each ray.
    public: void SetMaxRange(double _max);

    /// \brief Get linear resolution of each ray.
    /// \return Linear resolution for each ray.
    public: double RangeResolution() const;

    /// \brief Set linear resolution of each ray.
    /// \param[in] Linear resolution for each ray.
    public: void SetRangeResolution(double _range);

    /// \brief Get the noise values for the ray sensor.
    /// \return Noise values for the ray sensor.
    public: const Noise &RayNoise() const;

    /// \biref Set the noise values for the ray sensor.
    /// \param[in] _noise Noise values for the ray sensor.
    public: void SetRayNoise(const Noise &_noise);

    /// \brief Return true if both Ray objects contain the same values.
    /// \param[_in] _ray Ray value to compare.
    /// \return True if 'this' == _ray.
    public: bool operator==(const Ray &_ray) const;

    /// \brief Return true this Ray object does not contain the same
    /// values as the passed in parameter.
    /// \param[_in] _ray Ray value to compare.
    /// \return True if 'this' != _ray.
    public: bool operator!=(const Ray &_ray) const;

    /// \brief Private data pointer.
    private: RayPrivate *dataPtr;
  };
  }
}
#endif
