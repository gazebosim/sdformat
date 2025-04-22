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
#include <filesystem>
#include <unordered_set>

#include "sdf/parser.hh"
#include "sdf/Sky.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Sky private data.
class sdf::Sky::Implementation
{
  /// \brief Time of day
  public: double time = 10.0;

  /// \brief Sunrise time
  public: double sunrise = 6.0;

  /// \brief Sunset time
  public: double sunset = 20.0;

  /// \brief Cloud speed
  public: double cloudSpeed = 0.6;

  /// \brief Cloud direction.
  public: gz::math::Angle cloudDirection;

  /// \brief Cloud humidity
  public: double cloudHumidity = 0.5;

  /// \brief Cloud mean size
  public: double cloudMeanSize = 0.5;

  /// \brief Cloud ambient color
  public: gz::math::Color cloudAmbient =
      gz::math::Color(0.8f, 0.8f, 0.8f);

  /// \brief Skybox texture URI
  public: std::string cubemapUri = "";

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;
};

/////////////////////////////////////////////////
Sky::Sky()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
double Sky::Time() const
{
  return this->dataPtr->time;
}

/////////////////////////////////////////////////
void Sky::SetTime(double _time)
{
  this->dataPtr->time = _time;
}

/////////////////////////////////////////////////
double Sky::Sunrise() const
{
  return this->dataPtr->sunrise;
}

/////////////////////////////////////////////////
void Sky::SetSunrise(double _sunrise)
{
  this->dataPtr->sunrise = _sunrise;
}

/////////////////////////////////////////////////
double Sky::Sunset() const
{
  return this->dataPtr->sunset;
}

/////////////////////////////////////////////////
void Sky::SetSunset(double _sunset)
{
  this->dataPtr->sunset = _sunset;
}

/////////////////////////////////////////////////
double Sky::CloudSpeed() const
{
  return this->dataPtr->cloudSpeed;
}

/////////////////////////////////////////////////
void Sky::SetCloudSpeed(double _speed)
{
  this->dataPtr->cloudSpeed = _speed;
}

/////////////////////////////////////////////////
gz::math::Angle Sky::CloudDirection() const
{
  return this->dataPtr->cloudDirection;
}

/////////////////////////////////////////////////
void Sky::SetCloudDirection(const gz::math::Angle &_angle)
{
  this->dataPtr->cloudDirection = _angle;
}

/////////////////////////////////////////////////
double Sky::CloudHumidity() const
{
  return this->dataPtr->cloudHumidity;
}

/////////////////////////////////////////////////
void Sky::SetCloudHumidity(double _humidity)
{
  this->dataPtr->cloudHumidity = _humidity;
}

/////////////////////////////////////////////////
double Sky::CloudMeanSize() const
{
  return this->dataPtr->cloudMeanSize;
}

/////////////////////////////////////////////////
void Sky::SetCloudMeanSize(double _size)
{
  this->dataPtr->cloudMeanSize = _size;
}

/////////////////////////////////////////////////
gz::math::Color Sky::CloudAmbient() const
{
  return this->dataPtr->cloudAmbient;
}

/////////////////////////////////////////////////
void Sky::SetCloudAmbient(const gz::math::Color &_ambient)
{
  this->dataPtr->cloudAmbient = _ambient;
}

//////////////////////////////////////////////////
const std::string &Sky::CubemapUri() const
{
  return this->dataPtr->cubemapUri;
}

//////////////////////////////////////////////////
void Sky::SetCubemapUri(const std::string &_uri)
{
  this->dataPtr->cubemapUri = _uri;
}

/////////////////////////////////////////////////
Errors Sky::Load(ElementPtr _sdf)
{
  return this->Load(_sdf, ParserConfig::GlobalConfig());
}

/////////////////////////////////////////////////
Errors Sky::Load(ElementPtr _sdf, const ParserConfig &_config)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <sky> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "sky")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Sky, but the provided SDF element is not a "
        "<sky>."});
    return errors;
  }

  this->dataPtr->time = _sdf->Get<double>(
      errors, "time", this->dataPtr->time).first;
  this->dataPtr->sunrise =
      _sdf->Get<double>(errors, "sunrise", this->dataPtr->sunrise).first;
  this->dataPtr->sunset =
      _sdf->Get<double>(errors, "sunset", this->dataPtr->sunset).first;

  if (_sdf->HasElement("cubemap_uri"))
  {
    std::unordered_set<std::string> paths;
    if (!_sdf->FilePath().empty())
    {
      paths.insert(std::filesystem::path(
          _sdf->FilePath()).parent_path().string());
    }
    this->dataPtr->cubemapUri = resolveURI(
      _sdf->Get<std::string>(errors, "cubemap_uri", "").first,
      _config, errors, paths);
  }

  if ( _sdf->HasElement("clouds"))
  {
    sdf::ElementPtr cloudElem = _sdf->GetElement("clouds", errors);
    this->dataPtr->cloudSpeed = cloudElem->Get<double>(
        errors, "speed", this->dataPtr->cloudSpeed).first;
    this->dataPtr->cloudDirection =
        cloudElem->Get<gz::math::Angle>(errors, "direction",
        this->dataPtr->cloudDirection).first;
    this->dataPtr->cloudHumidity = cloudElem->Get<double>(
        errors, "humidity", this->dataPtr->cloudHumidity).first;
    this->dataPtr->cloudMeanSize = cloudElem->Get<double>(
        errors, "mean_size", this->dataPtr->cloudMeanSize).first;
    this->dataPtr->cloudAmbient = cloudElem->Get<gz::math::Color>(
        errors, "ambient", this->dataPtr->cloudAmbient).first;
  }

  return errors;
}

/////////////////////////////////////////////////
sdf::ElementPtr Sky::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
sdf::ElementPtr Sky::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Sky::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr sceneElem(new sdf::Element);
  sdf::initFile("scene.sdf", sceneElem);
  sdf::ElementPtr elem = sceneElem->GetElement("sky", _errors);

  elem->GetElement("time", _errors)->Set(_errors, this->Time());
  elem->GetElement("sunrise", _errors)->Set(_errors, this->Sunrise());
  elem->GetElement("sunset", _errors)->Set(_errors, this->Sunset());
  elem->GetElement("cubemap_uri", _errors)->Set(_errors, this->CubemapUri());

  sdf::ElementPtr cloudElem = elem->GetElement("clouds", _errors);
  cloudElem->GetElement("speed", _errors)->Set(_errors, this->CloudSpeed());
  cloudElem->GetElement("direction", _errors)->Set(
      _errors, this->CloudDirection().Radian());
  cloudElem->GetElement("humidity", _errors)->Set(
      _errors, this->CloudHumidity());
  cloudElem->GetElement("mean_size", _errors)->Set(
      _errors, this->CloudMeanSize());
  cloudElem->GetElement("ambient", _errors)->Set(
      _errors, this->CloudAmbient());

  return elem;
}
