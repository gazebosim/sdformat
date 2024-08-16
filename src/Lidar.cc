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
#include "sdf/Lidar.hh"
#include "sdf/parser.hh"

using namespace sdf;
using namespace gz;

/// \brief Private lidar data.
class sdf::Lidar::Implementation
{
  /// \brief Number of rays horizontally per laser sweep
  public: unsigned int horizontalScanSamples{640};

  /// \brief Resolution for horizontal scan
  public: double horizontalScanResolution{1.0};

  /// \brief Minimum angle for horizontal scan
  public: math::Angle horizontalScanMinAngle{0.0};

  /// \brief Maximum angle for horizontal scan
  public: math::Angle horizontalScanMaxAngle{0.0};

  /// \brief Number of rays vertically per laser sweep
  public: unsigned int verticalScanSamples{1};

  /// \brief Resolution for vertical scan
  public: double verticalScanResolution{1.0};

  /// \brief Minimum angle for vertical scan
  public: math::Angle verticalScanMinAngle{0.0};

  /// \brief Maximum angle for vertical scan
  public: math::Angle verticalScanMaxAngle{0.0};

  /// \brief Minimum distance for each ray
  public: double minRange{0.0};

  /// \brief Maximum distance for each ray
  public: double maxRange{0.0};

  /// \brief Linear resolution for each ray
  public: double rangeResolution{0.0};

  /// \brief Noise values for the lidar sensor
  public: Noise lidarNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf{nullptr};

  /// \brief Visibility mask of a lidar. Defaults to 0xFFFFFFFF
  public: uint32_t visibilityMask{UINT32_MAX};
};

//////////////////////////////////////////////////
Lidar::Lidar()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

//////////////////////////////////////////////////
/// \brief Load the lidar based on an element pointer. This is *not*
/// the usual entry point. Typical usage of the SDF DOM is through the Root
/// object.
/// \param[in] _sdf The SDF Element pointer
/// \return Errors, which is a vector of Error objects. Each Error includes
/// an error code and message. An empty vector indicates no error.
Errors Lidar::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Lidar, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <lidar> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "ray" && _sdf->GetName() != "lidar" &&
      _sdf->GetName() != "gpu_ray" && _sdf->GetName() != "gpu_lidar")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Lidar, but the provided SDF element is "
        "not a <lidar>."});
    return errors;
  }

  // Load lidar sensor properties
  if (_sdf->HasElement("scan"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("scan");
    if (elem->HasElement("horizontal"))
    {
      sdf::ElementPtr subElem = elem->GetElement("horizontal");
      if (subElem->HasElement("samples"))
        this->dataPtr->horizontalScanSamples = subElem->Get<unsigned int>(
          "samples");
      if (subElem->HasElement("resolution"))
        this->dataPtr->horizontalScanResolution = subElem->Get<double>(
          "resolution");
      if (subElem->HasElement("min_angle"))
        this->dataPtr->horizontalScanMinAngle = math::Angle(
          subElem->Get<double>("min_angle"));
      if (subElem->HasElement("max_angle"))
        this->dataPtr->horizontalScanMaxAngle = math::Angle(
          subElem->Get<double>("max_angle"));
    }
    else
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
        "A lidar scan horizontal element is required, but it is not set."});
      return errors;
    }

    if (elem->HasElement("vertical"))
    {
      sdf::ElementPtr subElem = elem->GetElement("vertical");
      if (subElem->HasElement("samples"))
        this->dataPtr->verticalScanSamples = subElem->Get<unsigned int>(
          "samples");
      if (subElem->HasElement("resolution"))
        this->dataPtr->verticalScanResolution = subElem->Get<double>(
          "resolution");
      if (subElem->HasElement("min_angle"))
        this->dataPtr->verticalScanMinAngle = math::Angle(subElem->Get<double>(
          "min_angle"));
      if (subElem->HasElement("max_angle"))
        this->dataPtr->verticalScanMaxAngle = math::Angle(subElem->Get<double>(
          "max_angle"));
    }
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
      "A lidar scan element is required, but the scan is not set."});
    return errors;
  }

  if (_sdf->HasElement("range"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("range");
    if (elem->HasElement("min"))
      this->dataPtr->minRange = elem->Get<double>("min");
    if (elem->HasElement("max"))
      this->dataPtr->maxRange = elem->Get<double>("max");
    if (elem->HasElement("resolution"))
      this->dataPtr->rangeResolution = elem->Get<double>("resolution");
  }
  else
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
      "A lidar range element is required, but the range is not set."});
    return errors;
  }

  if (_sdf->HasElement("noise"))
    this->dataPtr->lidarNoise.Load(_sdf->GetElement("noise"));

  if (_sdf->HasElement("visibility_mask"))
  {
    this->dataPtr->visibilityMask = _sdf->Get<uint32_t>("visibility_mask",
        this->dataPtr->visibilityMask).first;
  }

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr Lidar::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
unsigned int Lidar::HorizontalScanSamples() const
{
  return this->dataPtr->horizontalScanSamples;
}

//////////////////////////////////////////////////
void Lidar::SetHorizontalScanSamples(unsigned int _samples)
{
  this->dataPtr->horizontalScanSamples = _samples;
}

//////////////////////////////////////////////////
double Lidar::HorizontalScanResolution() const
{
  return this->dataPtr->horizontalScanResolution;
}

//////////////////////////////////////////////////
void Lidar::SetHorizontalScanResolution(double _res)
{
  this->dataPtr->horizontalScanResolution = _res;
}

//////////////////////////////////////////////////
math::Angle Lidar::HorizontalScanMinAngle() const
{
  return this->dataPtr->horizontalScanMinAngle;
}

//////////////////////////////////////////////////
void Lidar::SetHorizontalScanMinAngle(const math::Angle &_min)
{
  this->dataPtr->horizontalScanMinAngle = _min;
}

//////////////////////////////////////////////////
math::Angle Lidar::HorizontalScanMaxAngle() const
{
  return this->dataPtr->horizontalScanMaxAngle;
}

//////////////////////////////////////////////////
void Lidar::SetHorizontalScanMaxAngle(const math::Angle &_max)
{
  this->dataPtr->horizontalScanMaxAngle = _max;
}

//////////////////////////////////////////////////
unsigned int Lidar::VerticalScanSamples() const
{
  return this->dataPtr->verticalScanSamples;
}

//////////////////////////////////////////////////
void Lidar::SetVerticalScanSamples(unsigned int _samples)
{
  this->dataPtr->verticalScanSamples = _samples;
}

//////////////////////////////////////////////////
double Lidar::VerticalScanResolution() const
{
  return this->dataPtr->verticalScanResolution;
}

//////////////////////////////////////////////////
void Lidar::SetVerticalScanResolution(double _res)
{
  this->dataPtr->verticalScanResolution = _res;
}

//////////////////////////////////////////////////
math::Angle Lidar::VerticalScanMinAngle() const
{
  return this->dataPtr->verticalScanMinAngle;
}

//////////////////////////////////////////////////
void Lidar::SetVerticalScanMinAngle(const math::Angle &_min)
{
  this->dataPtr->verticalScanMinAngle = _min;
}

//////////////////////////////////////////////////
math::Angle Lidar::VerticalScanMaxAngle() const
{
  return this->dataPtr->verticalScanMaxAngle;
}

//////////////////////////////////////////////////
void Lidar::SetVerticalScanMaxAngle(const math::Angle &_max)
{
  this->dataPtr->verticalScanMaxAngle = _max;
}

//////////////////////////////////////////////////
double Lidar::RangeMin() const
{
  return this->dataPtr->minRange;
}

//////////////////////////////////////////////////
void Lidar::SetRangeMin(double _min)
{
  this->dataPtr->minRange = _min;
}

//////////////////////////////////////////////////
double Lidar::RangeMax() const
{
  return this->dataPtr->maxRange;
}

//////////////////////////////////////////////////
void Lidar::SetRangeMax(double _max)
{
  this->dataPtr->maxRange = _max;
}

//////////////////////////////////////////////////
double Lidar::RangeResolution() const
{
  return this->dataPtr->rangeResolution;
}

//////////////////////////////////////////////////
void Lidar::SetRangeResolution(double _range)
{
  this->dataPtr->rangeResolution = _range;
}

//////////////////////////////////////////////////
const Noise &Lidar::LidarNoise() const
{
  return this->dataPtr->lidarNoise;
}

//////////////////////////////////////////////////
void Lidar::SetLidarNoise(const Noise &_noise)
{
  this->dataPtr->lidarNoise = _noise;
}

/////////////////////////////////////////////////
uint32_t Lidar::VisibilityMask() const
{
  return this->dataPtr->visibilityMask;
}

/////////////////////////////////////////////////
void Lidar::SetVisibilityMask(uint32_t _mask)
{
  this->dataPtr->visibilityMask = _mask;
}

//////////////////////////////////////////////////
bool Lidar::operator==(const Lidar &_lidar) const
{
  if (this->dataPtr->horizontalScanSamples != _lidar.HorizontalScanSamples())
    return false;
  if (std::abs(this->dataPtr->horizontalScanResolution -
    _lidar.HorizontalScanResolution()) > 1e-6)
    return false;
  if (this->dataPtr->horizontalScanMinAngle != _lidar.HorizontalScanMinAngle())
    return false;
  if (this->dataPtr->horizontalScanMaxAngle != _lidar.HorizontalScanMaxAngle())
    return false;
  if (this->dataPtr->verticalScanSamples != _lidar.VerticalScanSamples())
    return false;
  if (std::abs(this->dataPtr->verticalScanResolution -
    _lidar.VerticalScanResolution()) > 1e-6)
    return false;
  if (this->dataPtr->verticalScanMinAngle != _lidar.VerticalScanMinAngle())
    return false;
  if (this->dataPtr->verticalScanMaxAngle != _lidar.VerticalScanMaxAngle())
    return false;
  if (std::abs(this->dataPtr->minRange - _lidar.RangeMin()) > 1e-6)
    return false;
  if (std::abs(this->dataPtr->maxRange - _lidar.RangeMax()) > 1e-6)
    return false;
  if (std::abs(this->dataPtr->rangeResolution -
        _lidar.RangeResolution()) > 1e-6)
  {
    return false;
  }
  if (this->dataPtr->lidarNoise != _lidar.LidarNoise())
    return false;
  if (this->dataPtr->visibilityMask != _lidar.VisibilityMask())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool Lidar::operator!=(const Lidar &_lidar) const
{
  return !(*this == _lidar);
}

/////////////////////////////////////////////////
sdf::ElementPtr Lidar::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile(std::string(this->SchemaFile()), elem);

  sdf::ElementPtr scanElem = elem->GetElement("scan");
  sdf::ElementPtr horElem = scanElem->GetElement("horizontal");
  horElem->GetElement("samples")->Set<double>(this->HorizontalScanSamples());
  horElem->GetElement("resolution")->Set<double>(
      this->HorizontalScanResolution());
  horElem->GetElement("min_angle")->Set<double>(
      this->HorizontalScanMinAngle().Radian());
  horElem->GetElement("max_angle")->Set<double>(
      this->HorizontalScanMaxAngle().Radian());

  sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
  vertElem->GetElement("samples")->Set<double>(this->VerticalScanSamples());
  vertElem->GetElement("resolution")->Set<double>(
      this->VerticalScanResolution());
  vertElem->GetElement("min_angle")->Set<double>(
      this->VerticalScanMinAngle().Radian());
  vertElem->GetElement("max_angle")->Set<double>(
      this->VerticalScanMaxAngle().Radian());

  sdf::ElementPtr rangeElem = elem->GetElement("range");
  rangeElem->GetElement("min")->Set<double>(this->RangeMin());
  rangeElem->GetElement("max")->Set<double>(this->RangeMax());
  rangeElem->GetElement("resolution")->Set<double>(
      this->RangeResolution());

  sdf::ElementPtr noiseElem = elem->GetElement("noise");
  std::string noiseType;
  switch (this->dataPtr->lidarNoise.Type())
  {
    case sdf::NoiseType::NONE:
      noiseType = "none";
      break;
    case sdf::NoiseType::GAUSSIAN:
      noiseType = "gaussian";
      break;
    case sdf::NoiseType::GAUSSIAN_QUANTIZED:
      noiseType = "gaussian_quantized";
      break;
    default:
      noiseType = "none";
  }

  // lidar does not use noise.sdf description
  noiseElem->GetElement("type")->Set<std::string>(noiseType);
  noiseElem->GetElement("mean")->Set<double>(this->dataPtr->lidarNoise.Mean());
  noiseElem->GetElement("stddev")->Set<double>(
      this->dataPtr->lidarNoise.StdDev());

  elem->GetElement("visibility_mask")->Set<uint32_t>(
      this->VisibilityMask());

  return elem;
}

/////////////////////////////////////////////////
inline std::string_view Lidar::SchemaFile() 
{
    static char kSchemaFile[] = "lidar.sdf";
    return kSchemaFile;
}

