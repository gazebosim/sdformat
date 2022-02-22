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
#include "sdf/parser.hh"
#include "FrameSemantics.hh"
#include "ScopedGraph.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Light private data.
class sdf::Light::Implementation
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
  public: sdf::ElementPtr sdf;

  /// \brief Name of xml parent object.
  public: std::string xmlParentName;

  /// \brief Scoped Pose Relative-To graph at the parent model or world scope.
  public: sdf::ScopedGraph<sdf::PoseRelativeToGraph> poseRelativeToGraph;

  /// \brief True if the light should cast shadows.
  public: bool castShadows = false;

  /// \brief Light intensity
  public: double intensity = 1.0;

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

  /// \brief Is light on ?
  public: bool isLightOn = true;
};

/////////////////////////////////////////////////
Light::Light()
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
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

  this->dataPtr->isLightOn = _sdf->Get<bool>("light_on",
      this->dataPtr->isLightOn).first;

  this->dataPtr->castShadows = _sdf->Get<bool>("cast_shadows",
      this->dataPtr->castShadows).first;

  this->dataPtr->intensity = _sdf->Get<double>("intensity",
      this->dataPtr->intensity).first;

  this->dataPtr->diffuse = _sdf->Get<ignition::math::Color>("diffuse",
      this->dataPtr->diffuse).first;

  this->dataPtr->specular = _sdf->Get<ignition::math::Color>("specular",
      this->dataPtr->specular).first;

  sdf::ElementPtr attenuationElem = _sdf->GetElement("attenuation");
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

  sdf::ElementPtr spotElem = _sdf->GetElement("spot");
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
void Light::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const ignition::math::Pose3d &Light::RawPose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
const std::string &Light::PoseRelativeTo() const
{
  return this->dataPtr->poseRelativeTo;
}

/////////////////////////////////////////////////
void Light::SetRawPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
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
    sdf::ScopedGraph<PoseRelativeToGraph> _graph)
{
  this->dataPtr->poseRelativeToGraph = _graph;
}

/////////////////////////////////////////////////
sdf::SemanticPose Light::SemanticPose() const
{
  return sdf::SemanticPose(
      this->dataPtr->pose,
      this->dataPtr->poseRelativeTo,
      this->dataPtr->xmlParentName,
      this->dataPtr->poseRelativeToGraph);
}

/////////////////////////////////////////////////
sdf::ElementPtr Light::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
double Light::Intensity() const
{
  return this->dataPtr->intensity;
}

/////////////////////////////////////////////////
void Light::SetIntensity(const double _intensity)
{
  this->dataPtr->intensity = _intensity;
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
bool Light::LightOn() const
{
  return this->dataPtr->isLightOn;
}

/////////////////////////////////////////////////
void Light::SetLightOn(const bool _isLightOn)
{
  this->dataPtr->isLightOn = _isLightOn;
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

/////////////////////////////////////////////////
sdf::ElementPtr Light::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("light.sdf", elem);

  std::string lightTypeStr = "point";
  switch (this->Type())
  {
    case LightType::POINT:
      lightTypeStr = "point";
      break;
    case LightType::DIRECTIONAL:
      lightTypeStr = "directional";
      break;
    case LightType::SPOT:
      lightTypeStr = "spot";
      break;
    default:
      break;
  }
  elem->GetAttribute("type")->Set<std::string>(lightTypeStr);
  elem->GetAttribute("name")->Set<std::string>(this->Name());
  sdf::ElementPtr poseElem = elem->GetElement("pose");
  if (!this->dataPtr->poseRelativeTo.empty())
  {
    poseElem->GetAttribute("relative_to")->Set<std::string>(
        this->dataPtr->poseRelativeTo);
  }
  poseElem->Set<ignition::math::Pose3d>(this->RawPose());

  elem->GetElement("cast_shadows")->Set<bool>(this->CastShadows());
  elem->GetElement("intensity")->Set<double>(this->Intensity());
  elem->GetElement("direction")->Set<ignition::math::Vector3d>(
      this->Direction());
  elem->GetElement("diffuse")->Set<ignition::math::Color>(this->Diffuse());
  elem->GetElement("specular")->Set<ignition::math::Color>(this->Specular());
  sdf::ElementPtr attenuationElem = elem->GetElement("attenuation");
  attenuationElem->GetElement("linear")->Set<double>(
      this->LinearAttenuationFactor());
  attenuationElem->GetElement("constant")->Set<double>(
      this->ConstantAttenuationFactor());
  attenuationElem->GetElement("quadratic")->Set<double>(
      this->QuadraticAttenuationFactor());
  attenuationElem->GetElement("range")->Set<double>(
      this->AttenuationRange());

  sdf::ElementPtr spotElem = elem->GetElement("spot");
  spotElem->GetElement("inner_angle")->Set<double>(
      this->SpotInnerAngle().Radian());
  spotElem->GetElement("outer_angle")->Set<double>(
      this->SpotOuterAngle().Radian());
  spotElem->GetElement("falloff")->Set<double>(this->SpotFalloff());
  return elem;
}
