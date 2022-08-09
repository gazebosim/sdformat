/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <string>
#include <ignition/math/Pose3.hh>
#include "sdf/Error.hh"
#include "sdf/Light.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Light private data.
class sdf::LightPrivate
{
  /// \brief Name of the light.
  public: std::string name = "";

  /// \brief Pose of the light
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Frame of the pose.
  public: std::string poseRelativeTo = "";

  /// \brief The light type.
  public: LightType type = LightType::POINT;

  /// \brief The SDF element pointer used during load.
  public: ElementPtr sdf;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Weak pointer to model's Pose Relative-To Graph.
  public: std::weak_ptr<const PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief True if the light should cast shadows.
  public: bool castShadows = false;

  /// \brief The attenation range.
  public: double attenuationRange = 10.0;

  /// \brief The linear attenuation factor.
  public: double linearAttenuation = 1.0;

  /// \brief The constant attenuation factor.
  public: double constantAttenuation = 1.0;

  /// \brief The quadratic attenuation factor.
  public: double quadraticAttenuation = 0.0;

  /// \brief Direction of the light source.
  public: ignition::math::Vector3d direction {0, 0, -1};

  /// \brief Light diffuse color.
  public: ignition::math::Color diffuse;

  /// \brief Light specular color.
  public: ignition::math::Color specular;

  /// \brief Spot light inner angle.
  public: ignition::math::Angle spotInnerAngle {0.0};

  /// \brief Spot light outer angle.
  public: ignition::math::Angle spotOuterAngle {0.0};

  /// \brief Spot light falloff.
  public: double spotFalloff = 0.0;
};

/////////////////////////////////////////////////
Light::Light()
  : dataPtr(new LightPrivate)
{
}

/////////////////////////////////////////////////
Light::~Light()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
Light::Light(const Light &_light)
  : dataPtr(new LightPrivate)
{
  this->CopyFrom(_light);
}

/////////////////////////////////////////////////
Light::Light(Light &&_light) noexcept
  : dataPtr(std::exchange(_light.dataPtr, nullptr))
{
}

//////////////////////////////////////////////////
Light &Light::operator=(const Light &_light)
{
  return *this = Light(_light);
}

//////////////////////////////////////////////////
Light &Light::operator=(Light &&_light)
{
  std::swap(this->dataPtr, _light.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
void Light::CopyFrom(const Light &_light)
{
  this->dataPtr->name = _light.dataPtr->name;
  this->dataPtr->pose = _light.dataPtr->pose;
  this->dataPtr->poseRelativeTo = _light.dataPtr->poseRelativeTo;
  this->dataPtr->type = _light.dataPtr->type;
  this->dataPtr->sdf = _light.dataPtr->sdf;
  this->dataPtr->castShadows = _light.dataPtr->castShadows;
  this->dataPtr->attenuationRange = _light.dataPtr->attenuationRange;
  this->dataPtr->linearAttenuation = _light.dataPtr->linearAttenuation;
  this->dataPtr->constantAttenuation = _light.dataPtr->constantAttenuation;
  this->dataPtr->quadraticAttenuation = _light.dataPtr->quadraticAttenuation;
  this->dataPtr->direction = _light.dataPtr->direction;
  this->dataPtr->diffuse = _light.dataPtr->diffuse;
  this->dataPtr->specular = _light.dataPtr->specular;
  this->dataPtr->spotInnerAngle = _light.dataPtr->spotInnerAngle;
  this->dataPtr->spotOuterAngle = _light.dataPtr->spotOuterAngle;
  this->dataPtr->spotFalloff = _light.dataPtr->spotFalloff;
}

/////////////////////////////////////////////////
Errors Light::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <light>
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "light")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Light, but the provided SDF element is not a "
        "<light>."});
    return errors;
  }

  std::string typeString = _sdf->Get<std::string>("type",
      std::string("point")).first;
  if (typeString == "point")
    this->dataPtr->type = LightType::POINT;
  else if (typeString == "spot")
    this->dataPtr->type = LightType::SPOT;
  else if (typeString == "directional")
    this->dataPtr->type = LightType::DIRECTIONAL;
  else
  {
    this->dataPtr->type = LightType::INVALID;
    errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Invalid light type with a value of [" + typeString + "]."});
  }

  // Read the lights's name
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A light name is required, but the name is not set."});
  }

  // Check that the light's name is valid
  if (isReservedName(this->dataPtr->name))
  {
    errors.push_back({ErrorCode::RESERVED_NAME,
                     "The supplied light name [" + this->dataPtr->name +
                     "] is reserved."});
  }

  // Load the pose. Ignore the return value since the light pose is optional.
  loadPose(_sdf, this->dataPtr->pose, this->dataPtr->poseRelativeTo);

  this->dataPtr->castShadows = _sdf->Get<bool>("cast_shadows",
      this->dataPtr->castShadows).first;

  this->dataPtr->diffuse = _sdf->Get<ignition::math::Color>("diffuse",
      this->dataPtr->diffuse).first;

  this->dataPtr->specular = _sdf->Get<ignition::math::Color>("specular",
      this->dataPtr->specular).first;

  ElementPtr attenuationElem = _sdf->GetElement("attenuation");
  if (attenuationElem)
  {
    std::pair<double, bool> doubleValue = attenuationElem->Get<double>(
        "range", this->dataPtr->attenuationRange);
    if (!doubleValue.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "An <attenuation> requires a <range>."});
    }
    this->SetAttenuationRange(doubleValue.first);

    this->SetLinearAttenuationFactor(attenuationElem->Get<double>("linear",
          this->dataPtr->linearAttenuation).first);

    this->SetConstantAttenuationFactor(attenuationElem->Get<double>("constant",
          this->dataPtr->constantAttenuation).first);

    this->SetQuadraticAttenuationFactor(attenuationElem->Get<double>(
        "quadratic", this->dataPtr->quadraticAttenuation).first);
  }

  // Read the direction
  if (this->dataPtr->type == LightType::SPOT ||
      this->dataPtr->type == LightType::DIRECTIONAL)
  {
    std::pair<ignition::math::Vector3d, bool> dirPair =
      _sdf->Get<>("direction", this->dataPtr->direction);

    if (!dirPair.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A <direction> is required for a " + typeString + "light."});
    }

    this->dataPtr->direction = dirPair.first;
  }

  ElementPtr spotElem = _sdf->GetElement("spot");
  if (this->dataPtr->type == LightType::SPOT && spotElem)
  {
    // Check for and set inner_angle
    std::pair<double, bool> doubleValue = spotElem->Get<double>(
        "inner_angle", this->dataPtr->spotInnerAngle.Radian());
    if (!doubleValue.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A spot light requires an <inner_angle>."});
    }
    this->SetSpotInnerAngle(doubleValue.first);

    // Check for and set outer_angle
    doubleValue = spotElem->Get<double>(
        "outer_angle", this->dataPtr->spotOuterAngle.Radian());
    if (!doubleValue.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A spot light requires an <outer_angle>."});
    }
    this->SetSpotOuterAngle(doubleValue.first);

    // Check for and set falloff
    doubleValue = spotElem->Get<double>("falloff", this->dataPtr->spotFalloff);
    if (!doubleValue.second)
    {
      errors.push_back({ErrorCode::ELEMENT_MISSING,
          "A spot light requires a <falloff>."});
    }
    this->SetSpotFalloff(doubleValue.first);
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Light::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Light::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Light::Pose() const
{
  return this->RawPose();
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Light::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Light::PoseFrame() const
{
  return this->PoseRelativeTo();
}

/////////////////////////////////////////////////
const std::string &Light::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Light::SetPose(const ignition::math::Pose3d &_pose)
{
  this->SetRawPose(_pose);
}

/////////////////////////////////////////////////
void Light::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
void Light::SetPoseFrame(const std::string &_frame)
{
  this->SetPoseRelativeTo(_frame);
}

/////////////////////////////////////////////////
void Light::SetPoseRelativeTo(const std::string &_frame)
{
  this->dataPtr->poseRelativeTo = _frame;
}

/////////////////////////////////////////////////
void Light::SetXmlParentName(const std::string &_xmlParentName)
{
  this->dataPtr->xmlParentName = _xmlParentName;
}

/////////////////////////////////////////////////
void Light::SetPoseRelativeToGraph(
    std::weak_ptr<const PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
SemanticPose Light::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
ElementPtr Light::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
bool Light::CastShadows() const
{
  return this->dataPtr->castShadows;
}

/////////////////////////////////////////////////
void Light::SetCastShadows(const bool _cast)
{
  this->dataPtr->castShadows = _cast;
}

/////////////////////////////////////////////////
ignition::math::Color Light::Diffuse() const
{
  return this->dataPtr->diffuse;
}

/////////////////////////////////////////////////
void Light::SetDiffuse(const ignition::math::Color &_color) const
{
  this->dataPtr->diffuse = _color;
}

/////////////////////////////////////////////////
ignition::math::Color Light::Specular() const
{
  return this->dataPtr->specular;
}

/////////////////////////////////////////////////
void Light::SetSpecular(const ignition::math::Color &_color) const
{
  this->dataPtr->specular = _color;
}

/////////////////////////////////////////////////
double Light::AttenuationRange() const
{
  return this->dataPtr->attenuationRange;
}

/////////////////////////////////////////////////
void Light::SetAttenuationRange(const double _range)
{
  this->dataPtr->attenuationRange = std::max(0.0, _range);
}

/////////////////////////////////////////////////
double Light::LinearAttenuationFactor() const
{
  return this->dataPtr->linearAttenuation;
}

/////////////////////////////////////////////////
void Light::SetLinearAttenuationFactor(const double _factor)
{
  this->dataPtr->linearAttenuation = ignition::math::clamp(_factor, 0.0, 1.0);
}

/////////////////////////////////////////////////
double Light::ConstantAttenuationFactor() const
{
  return this->dataPtr->constantAttenuation;
}

/////////////////////////////////////////////////
void Light::SetConstantAttenuationFactor(const double _factor)
{
  this->dataPtr->constantAttenuation =
    ignition::math::clamp(_factor, 0.0, 1.0);
}

/////////////////////////////////////////////////
double Light::QuadraticAttenuationFactor() const
{
  return this->dataPtr->quadraticAttenuation;
}

/////////////////////////////////////////////////
void Light::SetQuadraticAttenuationFactor(const double _factor)
{
  this->dataPtr->quadraticAttenuation = std::max(0.0, _factor);
}

/////////////////////////////////////////////////
ignition::math::Vector3d Light::Direction() const
{
  return this->dataPtr->direction;
}

/////////////////////////////////////////////////
void Light::SetDirection(const ignition::math::Vector3d  &_dir)
{
  this->dataPtr->direction = _dir;
}

/////////////////////////////////////////////////
ignition::math::Angle Light::SpotInnerAngle() const
{
  return this->dataPtr->spotInnerAngle;
}

/////////////////////////////////////////////////
void Light::SetSpotInnerAngle(const ignition::math::Angle &_angle)
{
  this->dataPtr->spotInnerAngle.Radian(std::max(0.0, _angle.Radian()));
}

/////////////////////////////////////////////////
ignition::math::Angle Light::SpotOuterAngle() const
{
  return this->dataPtr->spotOuterAngle;
}

/////////////////////////////////////////////////
void Light::SetSpotOuterAngle(const ignition::math::Angle &_angle)
{
  this->dataPtr->spotOuterAngle.Radian(std::max(0.0, _angle.Radian()));
}

/////////////////////////////////////////////////
double Light::SpotFalloff() const
{
  return this->dataPtr->spotFalloff;
}

/////////////////////////////////////////////////
void Light::SetSpotFalloff(const double _falloff)
{
  this->dataPtr->spotFalloff = std::max(0.0, _falloff);
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
