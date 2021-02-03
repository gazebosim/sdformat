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

#include <algorithm>
#include "sdf/Noise.hh"
#include "sdf/Types.hh"

using namespace sdf;

/// \brief Private noise data.
class sdf::Noise::Implementation
{
  /// \brief The noise type.
  public: NoiseType type = NoiseType::NONE;

  /// \brief Gaussian mean.
  public: double mean = 0.0;

  /// \brief Gaussian standard deviation.
  public: double stdDev = 0.0;

  /// \brief Gaussian mean for the bias.
  public: double biasMean = 0.0;

  /// \brief Gaussian standard deviation for the bias.
  public: double biasStdDev = 0.0;

  /// \brief Precision value for gaussian quantized.
  public: double precision = 0.0;

  /// \brief For type "gaussian*", the standard deviation of the noise used to
  /// drive a process to model slow variations in a sensor bias.
  public: double dynamicBiasStdDev = 0.0;

  /// \brief For type "gaussian*", the correlation time of the noise used to
  /// drive a process to model slow variations in a sensor bias.
  public: double dynamicBiasCorrelationTime = 0.0;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
Noise::Noise()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
Errors Noise::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <noise> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "noise")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Noise, but the provided SDF element is not a "
        "<noise>."});
    return errors;
  }

  std::pair<std::string, bool> type = _sdf->Get<std::string>("type", "none");
  if (!type.second)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Noise is missing the type attribute. Defaulting to 'none'."});
  }

  std::string typeLower = lowercase(type.first);

  if (typeLower == "none")
    this->dataPtr->type = NoiseType::NONE;
  else if (typeLower == "gaussian")
    this->dataPtr->type = NoiseType::GAUSSIAN;
  else if (typeLower == "gaussian_quantized")
    this->dataPtr->type =NoiseType::GAUSSIAN_QUANTIZED;
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Noise 'type' attribute is invalid with a value of [" +
        type.first + "]. Using a value of 'none'."});

    this->dataPtr->type = NoiseType::NONE;
  }

  this->dataPtr->mean = _sdf->Get<double>("mean",
      this->dataPtr->mean).first;

  this->dataPtr->stdDev = _sdf->Get<double>("stddev",
      this->dataPtr->stdDev).first;

  this->dataPtr->biasMean = _sdf->Get<double>("bias_mean",
      this->dataPtr->biasMean).first;

  this->dataPtr->biasStdDev = _sdf->Get<double>("bias_stddev",
      this->dataPtr->biasStdDev).first;

  this->dataPtr->precision = _sdf->Get<double>("precision",
      this->dataPtr->precision).first;

  this->dataPtr->dynamicBiasStdDev = _sdf->Get<double>("dynamic_bias_stddev",
      this->dataPtr->dynamicBiasStdDev).first;

  this->dataPtr->dynamicBiasCorrelationTime = _sdf->Get<double>(
      "dynamic_bias_correlation_time",
      this->dataPtr->dynamicBiasCorrelationTime).first;

  return errors;
}

//////////////////////////////////////////////////
NoiseType Noise::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Noise::SetType(NoiseType _type)
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
double Noise::Mean() const
{
  return this->dataPtr->mean;
}

//////////////////////////////////////////////////
void Noise::SetMean(double _mean)
{
  this->dataPtr->mean = _mean;
}

//////////////////////////////////////////////////
double Noise::StdDev() const
{
  return this->dataPtr->stdDev;
}

//////////////////////////////////////////////////
void Noise::SetStdDev(double _stddev)
{
  this->dataPtr->stdDev = _stddev;
}

//////////////////////////////////////////////////
double Noise::BiasMean() const
{
  return this->dataPtr->biasMean;
}

//////////////////////////////////////////////////
void Noise::SetBiasMean(double _bias)
{
  this->dataPtr->biasMean = _bias;
}

//////////////////////////////////////////////////
double Noise::BiasStdDev() const
{
  return this->dataPtr->biasStdDev;
}

//////////////////////////////////////////////////
void Noise::SetBiasStdDev(double _bias)
{
  this->dataPtr->biasStdDev = _bias;
}

//////////////////////////////////////////////////
double Noise::Precision() const
{
  return this->dataPtr->precision;
}

//////////////////////////////////////////////////
void Noise::SetPrecision(double _precision)
{
  this->dataPtr->precision = _precision;
}

//////////////////////////////////////////////////
double Noise::DynamicBiasStdDev() const
{
  return this->dataPtr->dynamicBiasStdDev;
}

//////////////////////////////////////////////////
void Noise::SetDynamicBiasStdDev(double _stddev)
{
  this->dataPtr->dynamicBiasStdDev = _stddev;
}

//////////////////////////////////////////////////
double Noise::DynamicBiasCorrelationTime() const
{
  return this->dataPtr->dynamicBiasCorrelationTime;
}

//////////////////////////////////////////////////
void Noise::SetDynamicBiasCorrelationTime(double _time)
{
  this->dataPtr->dynamicBiasCorrelationTime = _time;
}

//////////////////////////////////////////////////
sdf::ElementPtr Noise::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
bool Noise::operator!=(const Noise &_noise) const
{
  return !(*this == _noise);
}

//////////////////////////////////////////////////
bool Noise::operator==(const Noise &_noise) const
{
  return this->dataPtr->type == _noise.Type() &&
    ignition::math::equal(this->dataPtr->mean, _noise.Mean()) &&
    ignition::math::equal(this->dataPtr->stdDev, _noise.StdDev()) &&
    ignition::math::equal(this->dataPtr->biasMean, _noise.BiasMean()) &&
    ignition::math::equal(this->dataPtr->biasStdDev, _noise.BiasStdDev()) &&
    ignition::math::equal(this->dataPtr->precision, _noise.Precision()) &&
    ignition::math::equal(this->dataPtr->dynamicBiasStdDev,
                          _noise.DynamicBiasStdDev()) &&
    ignition::math::equal(this->dataPtr->dynamicBiasCorrelationTime,
                          _noise.DynamicBiasCorrelationTime());
}
