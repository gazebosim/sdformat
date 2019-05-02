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
#include "sdf/Ray.hh"

using namespace sdf;
using namespace ignition;

/// \brief Private ray data.
class sdf::RayPrivate
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

  /// \brief Noise values for the ray sensor
  public: Noise rayNoise;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

//////////////////////////////////////////////////
Ray::Ray()
  : dataPtr(new RayPrivate)
{
}

//////////////////////////////////////////////////
Ray::Ray(const Ray &_ray)
  : dataPtr(new RayPrivate(*_ray.dataPtr))
{
}

//////////////////////////////////////////////////
Ray::Ray(Ray &&_ray) noexcept
{
  this->dataPtr = _ray.dataPtr;
  _ray.dataPtr = nullptr;
}

//////////////////////////////////////////////////
Ray::~Ray()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Ray &Ray::operator=(const Ray &_ray)
{
  if (!this->dataPtr)
  {
    this->dataPtr = new RayPrivate;
  }
  *this->dataPtr = *_ray.dataPtr;
  return *this;
}

//////////////////////////////////////////////////
Ray &Ray::operator=(Ray &&_ray) noexcept
{
  this->dataPtr = _ray.dataPtr;
  _ray.dataPtr = nullptr;
  return * this;
}

//////////////////////////////////////////////////
/// \brief Load the ray based on an element pointer. This is *not*
/// the usual entry point. Typical usage of the SDF DOM is through the Root
/// object.
/// \param[in] _sdf The SDF Element pointer
/// \return Errors, which is a vector of Error objects. Each Error includes
/// an error code and message. An empty vector indicates no error.
Errors Ray::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a Ray, but the provided SDF "
        "element is null."});
    return errors;
  }

  // Check that the provided SDF element is a <ray> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "ray")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Ray, but the provided SDF element is "
        "not a <ray>."});
    return errors;
  }

  // Load ray sensor properties
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
        "A ray scan horizontal element is required, but it is not set."});
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
      "A ray scan element is required, but the scan is not set."});
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
      "A ray range element is required, but the range is not set."});
    return errors;
  }

  if (_sdf->HasElement("noise"))
    this->dataPtr->rayNoise.Load(_sdf->GetElement("noise"));

  return errors;
}

//////////////////////////////////////////////////
sdf::ElementPtr Ray::Element() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
unsigned int Ray::HorizontalScanSamples() const
{
  return this->dataPtr->horizontalScanSamples;
}

//////////////////////////////////////////////////
void Ray::SetHorizontalScanSamples(unsigned int _samples)
{
  this->dataPtr->horizontalScanSamples = _samples;
}

//////////////////////////////////////////////////
double Ray::HorizontalScanResolution() const
{
  return this->dataPtr->horizontalScanResolution;
}

//////////////////////////////////////////////////
void Ray::SetHorizontalScanResolution(double _res)
{
  this->dataPtr->horizontalScanResolution = _res;
}

//////////////////////////////////////////////////
math::Angle Ray::HorizontalScanMinAngle() const
{
  return this->dataPtr->horizontalScanMinAngle;
}

//////////////////////////////////////////////////
void Ray::SetHorizontalScanMinAngle(math::Angle _min)
{
  this->dataPtr->horizontalScanMinAngle = _min;
}

//////////////////////////////////////////////////
math::Angle Ray::HorizontalScanMaxAngle() const
{
  return this->dataPtr->horizontalScanMaxAngle;
}

//////////////////////////////////////////////////
void Ray::SetHorizontalScanMaxAngle(math::Angle _max)
{
  this->dataPtr->horizontalScanMaxAngle = _max;
}

//////////////////////////////////////////////////
unsigned int Ray::VerticalScanSamples() const
{
  return this->dataPtr->verticalScanSamples;
}

//////////////////////////////////////////////////
void Ray::SetVerticalScanSamples(unsigned int _samples)
{
  this->dataPtr->verticalScanSamples = _samples;
}

//////////////////////////////////////////////////
double Ray::VerticalScanResolution() const
{
  return this->dataPtr->verticalScanResolution;
}

//////////////////////////////////////////////////
void Ray::SetVerticalScanResolution(double _res)
{
  this->dataPtr->verticalScanResolution = _res;
}

//////////////////////////////////////////////////
math::Angle Ray::VerticalScanMinAngle() const
{
  return this->dataPtr->verticalScanMinAngle;
}

//////////////////////////////////////////////////
void Ray::SetVerticalScanMinAngle(math::Angle _min)
{
  this->dataPtr->verticalScanMinAngle = _min;
}

//////////////////////////////////////////////////
math::Angle Ray::VerticalScanMaxAngle() const
{
  return this->dataPtr->verticalScanMaxAngle;
}

//////////////////////////////////////////////////
void Ray::SetVerticalScanMaxAngle(math::Angle _max)
{
  this->dataPtr->verticalScanMaxAngle = _max;
}

//////////////////////////////////////////////////
double Ray::MinRange() const
{
  return this->dataPtr->minRange;
}

//////////////////////////////////////////////////
void Ray::SetMinRange(double _min)
{
  this->dataPtr->minRange = _min;
}

//////////////////////////////////////////////////
double Ray::MaxRange() const
{
  return this->dataPtr->maxRange;
}

//////////////////////////////////////////////////
void Ray::SetMaxRange(double _max)
{
  this->dataPtr->maxRange = _max;
}

//////////////////////////////////////////////////
double Ray::RangeResolution() const
{
  return this->dataPtr->rangeResolution;
}

//////////////////////////////////////////////////
void Ray::SetRangeResolution(double _range)
{
  this->dataPtr->rangeResolution = _range;
}

//////////////////////////////////////////////////
const Noise &Ray::RayNoise() const
{
  return this->dataPtr->rayNoise;
}

//////////////////////////////////////////////////
void Ray::SetRayNoise(const Noise &_noise)
{
  this->dataPtr->rayNoise = _noise;
}

//////////////////////////////////////////////////
bool Ray::operator==(const Ray &_ray) const
{
  if (this->dataPtr->horizontalScanSamples != _ray.HorizontalScanSamples())
    return false;
  if (std::abs(this->dataPtr->horizontalScanResolution -
    _ray.HorizontalScanResolution()) > 1e-6)
    return false;
  if (this->dataPtr->horizontalScanMinAngle != _ray.HorizontalScanMinAngle())
    return false;
  if (this->dataPtr->horizontalScanMaxAngle != _ray.HorizontalScanMaxAngle())
    return false;
  if (this->dataPtr->verticalScanSamples != _ray.VerticalScanSamples())
    return false;
  if (std::abs(this->dataPtr->verticalScanResolution -
    _ray.VerticalScanResolution()) > 1e-6)
    return false;
  if (this->dataPtr->verticalScanMinAngle != _ray.VerticalScanMinAngle())
    return false;
  if (this->dataPtr->verticalScanMaxAngle != _ray.VerticalScanMaxAngle())
    return false;
  if (std::abs(this->dataPtr->minRange - _ray.MinRange()) > 1e-6)
    return false;
  if (std::abs(this->dataPtr->maxRange - _ray.MaxRange()) > 1e-6)
    return false;
  if (std::abs(this->dataPtr->rangeResolution - _ray.RangeResolution()) > 1e-6)
    return false;
  if (this->dataPtr->rayNoise != _ray.RayNoise())
    return false;

  return true;
}

//////////////////////////////////////////////////
bool Ray::operator!=(const Ray &_ray) const
{
  return !(*this == _ray);
}
