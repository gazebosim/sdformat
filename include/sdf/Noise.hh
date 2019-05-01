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
#ifndef SDF_NOISE_HH_
#define SDF_NOISE_HH_

#include <sdf/Error.hh>
#include <sdf/Element.hh>
#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  // Forward declare private data class.
  class NoisePrivate;

  /// \enum NoiseType
  /// \brief The set of noise types.
  enum class NoiseType
  {
    /// \brief No noise model.
    NONE = 0,

    /// \brief Draw noise values independently for each measurement from a
    /// Gaussian distribution
    GAUSSIAN = 1,

    /// \brief Gaussian noise plus quantization of outputs (ie. rounding).
    GAUSSIAN_QUANTIZED = 2,
  };

  /// \brief The Noise class contains information about a noise
  /// model, such as a Gaussian distribution. A Noise DOM object is
  /// typically available from a Sensor.
  class SDFORMAT_VISIBLE Noise
  {
    /// \brief Default constructor
    public: Noise();

    /// \brief Copy constructor
    /// \param[in] _noise Noise to copy.
    public: Noise(const Noise &_noise);

    /// \brief Move constructor
    /// \param[in] _noise Noise to move.
    public: Noise(Noise &&_noise) noexcept;

    /// \brief Destructor
    public: ~Noise();

    /// \brief Assignment operator.
    /// \param[in] _noise The noise to set values from.
    /// \return *this
    public: Noise &operator=(const Noise &_noise);

    /// \brief Move assignment operator.
    /// \param[in] _noise The noise to set values from.
    /// \return *this
    public: Noise &operator=(Noise &&_noise);

    /// \brief Return true if both Noise objects contain the same values.
    /// \param[_in] _noise Noise value to compare.
    /// \return True if 'this' == _noise.
    public: bool operator==(const Noise &_noise) const;

    /// \brief Return true the Noise objects do not contain the same values.
    /// \param[_in] _noise Noise value to compare.
    /// \returen True if 'this' != _noise.
    public: bool operator!=(const Noise &_noise) const;

    /// \brief Load the noise based on a element pointer. This is *not*
    /// the usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the type of noise.
    /// \return The noise type.
    public: NoiseType Type() const;

    /// \brief Set the type of noise.
    /// \param[in] _type The noise type.
    public: void SetType(NoiseType _type);

    /// \brief Get the mean of the Gaussian distribution
    /// from which noise values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \return The mean of the Guassian distribution.
    public: double Mean() const;

    /// \brief Set the mean of the Gaussian distribution
    /// from which noise values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \param[in] _mean The mean of the Guassian distribution.
    public: void SetMean(double _mean);

    /// \brief Get the standard deviation of the Gaussian distribution
    /// from which noise values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \return The standard deviation of the Guassian distribution.
    public: double StdDev() const;

    /// \brief Set the standard deviation of the Gaussian distribution
    /// from which noise values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \param[in] _stddev The standard deviation of the Guassian distribution.
    public: void SetStdDev(double _stddev);

    /// \brief Get the mean of the Gaussian distribution
    /// from which bias values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \return The mean of the bias Guassian distribution.
    public: double BiasMean() const;

    /// \brief Set the mean of the Gaussian distribution
    /// from which bias values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \param[in] _bias The mean of the bias Guassian distribution.
    public: void SetBiasMean(double _bias);

    /// \brief Get the standard deviation of the Gaussian distribution
    /// from which bias values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \return The standard deviation of the bias Guassian distribution.
    public: double BiasStdDev() const;

    /// \brief Set the standard deviation of the Gaussian distribution
    /// from which bias values are drawn. This is applicable to "gaussian*"
    /// noise types.
    /// \param[in] _bias The standard deviation of the bias Guassian
    /// distribution.
    public: void SetBiasStdDev(double _bias);

    /// \brief For type "gaussian_quantized", get the precision of output
    /// signals. A value of zero implies infinite precision / no quantization.
    /// \return Precision of output signals.
    public: double Precision() const;

    /// \brief For type "gaussian_quantized", set the precision of output
    /// signals. A value of zero implies infinite precision / no quantization.
    /// \param[in] _precision Precision of output signals.
    public: void SetPrecision(double _precision);

    /// \brief For type "gaussian*", get the standard deviation of the noise
    /// used to drive a process to model slow variations in a sensor bias.
    /// \return The dynamic bias standard deviation.
    public: double DynamicBiasStdDev() const;

    /// \brief For type "gaussian*", set the standard deviation of the noise
    /// used to drive a process to model slow variations in a sensor bias.
    /// \param[in] _stddev The dynamic bias standard deviation.
    public: void SetDynamicBiasStdDev(double _stddev);

    /// \brief For type "gaussian*", get the correlation time of the noise
    /// used to drive a process to model slow variations in a sensor bias.
    /// \return The dynamic bias correlation time.
    public: double DynamicBiasCorrelationTime() const;

    /// \brief For type "gaussian*", set the correlation time in seconds of
    /// the noise used to drive a process to model slow variations in a sensor
    /// bias.A typical value, when used, would be on the order of
    /// 3600 seconds (1 hour).
    /// \param[in] _time The dynamic bias correlation time.
    public: void SetDynamicBiasCorrelationTime(double _time);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Private data pointer.
    private: NoisePrivate *dataPtr;
  };
  }
}
#endif
