/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <iostream>
#include "Utils.hh"
#include "sdf/dom/Light.hh"

using namespace sdf;

namespace sdf
{
  /// \brief Names that correspond to the LightType enum.
  static std::string LightTypenames[] =
  {
    "directional",
    "point",
    "spot",
    "unknown"
  };

  /// \brief Private data.
  class LightPrivate
  {
    /// \brief Type of light.
    public: LightType type = LightType::UNKNOWN;

    /// \brief True if the light should cast shadows.
    public: bool castShadows = false;

    /// \brief Diffuse light color.
    public: ignition::math::Color diffuse;

    /// \brief Specular light color.
    public: ignition::math::Color specular;
  };
}

/////////////////////////////////////////////////
Light::Light()
  : Entity(), dataPtr(new LightPrivate)
{
}

/////////////////////////////////////////////////
Light::~Light()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
bool Light::Load(sdf::ElementPtr _sdf)
{
  bool result = true;

  if (_sdf->GetName() != "light")
  {
    std::cerr << "Attempting to load a Light, but the provided "
      << "SDF element is not a <light>\n";

    // This is an error that cannot be recovered, so return false.
    return false;
  }

  // Get the name
  if (!this->LoadName(_sdf))
  {
    std::cerr << "A light name is required, but is not set.\n";
    result = false;
  }

  // Get the pose
  if (!this->LoadPose(_sdf))
  {
    std::cerr << "Unable to load the light[" << this->Name() << "]'s pose.\n";
    result = false;
  }

  // Read the type
  std::pair<std::string, bool> typePair = _sdf->Get<std::string>("type", "");
  this->dataPtr->type = LightType::UNKNOWN;
  if (!typePair.second)
  {
    std::cerr << "Light[" << this->Name() << "] has no type.\n";
    result = false;
  }
  else
  {
    if (typePair.first == "directional")
      this->dataPtr->type = LightType::DIRECTIONAL;
    else if (typePair.first == "point")
      this->dataPtr->type = LightType::POINT;
    else if (typePair.first == "spot")
      this->dataPtr->type = LightType::SPOT;
    else
    {
      std::cerr << "Unknown or unsupported light type["
                << typePair.first <<  "]\n";
      result = false;
    }
  }

  return result;
}

/////////////////////////////////////////////////
std::string Light::Typename() const
{
  return sdf::LightTypenames[static_cast<int>(this->dataPtr->type)];
}

/////////////////////////////////////////////////
LightType Light::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void Light::SetType(const LightType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
bool Light::CastShadows() const
{
  return this->dataPtr->castShadows;
}

/////////////////////////////////////////////////
void Light::SetCastShadows(const bool _shadows)
{
  this->dataPtr->castShadows = _shadows;
}

/////////////////////////////////////////////////
ignition::math::Color Light::Diffuse() const
{
  return this->dataPtr->diffuse;
}

/////////////////////////////////////////////////
void Light::SetDiffuse(const ignition::math::Color &_color)
{
  this->dataPtr->diffuse = _color;
}

/////////////////////////////////////////////////
ignition::math::Color Light::Specular() const
{
  return this->dataPtr->specular;
}

/////////////////////////////////////////////////
void Light::SetSpecular(const ignition::math::Color &_color)
{
  this->dataPtr->specular = _color;
}

/////////////////////////////////////////////////
void Light::Print(const std::string &_prefix) const
{
  std::cout << _prefix << "# Light: " << this->Name() << "\n"
            << _prefix << "  * Type: " << this->Typename() << "\n"
            << _prefix << "  * Pose:  " << this->Pose() << "\n"
            << _prefix << "  * Frame:  " << this->Frame() << "\n"
            << _prefix << "  * Cast shadows: " << this->CastShadows() << "\n"
            << _prefix << "  * Diffuse: " << this->Diffuse() << "\n"
            << _prefix << "  * Specular: " << this->Specular() << "\n";
}
