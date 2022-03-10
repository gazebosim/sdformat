/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "USD2SDF.hh"

#include "usd_model/WorldInterface.hh"

#include "USDWorld.hh"

#include <ignition/common/Filesystem.hh>

#include "sdf/Light.hh"

namespace sdf {
inline namespace SDF_VERSION_NAMESPACE {
namespace usd {
/////////////////////////////////////////////////
USD2SDF::USD2SDF()
{
}

/////////////////////////////////////////////////
USD2SDF::~USD2SDF()
{
}

/////////////////////////////////////////////////
UsdErrors USD2SDF::Read(const std::string &_filename,
  tinyxml2::XMLDocument* _sdfXmlOut)
{
  UsdErrors errors;

  std::shared_ptr<WorldInterface> worldInterface =
    std::make_shared<WorldInterface>();

  auto errorsParseUSD =
    parseUSDWorld(_filename, worldInterface);
  if (!errorsParseUSD.empty())
  {
    errors.emplace_back(UsdError(
      UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
      "Error parsing the usd file"));
    return errors;
  }

  tinyxml2::XMLElement *sdf = nullptr;
  tinyxml2::XMLElement *world = nullptr;

  sdf = _sdfXmlOut->NewElement("sdf");
  sdf->SetAttribute("version", "1.7");

  world = _sdfXmlOut->NewElement("world");
  std::string worldName = worldInterface->worldName;
  if (worldName.empty())
  {
    worldName = "world_name";
  }
  world->SetAttribute("name", (worldName + "_world").c_str());

  AddLights(worldInterface->lights, world);

  AddKeyValue(world, "gravity", Vector32Str(
    worldInterface->gravity * worldInterface->magnitude));

  std::vector<std::shared_ptr<sdf::usd::ModelInterface>> robotModels =
    worldInterface->models;

  for(auto & robotModel: robotModels)
  {
    if (!robotModel)
    {
      errors.emplace_back(UsdError(
        UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
        "Unable to call parse USD model"));
      return errors;
    }

    // create model element
    tinyxml2::XMLElement *robot = _sdfXmlOut->NewElement("model");

    // Set model name to sdf robot name
    robot->SetAttribute("name", robotModel->Name().c_str());

    world->LinkEndChild(robot);
  }

  sdf->LinkEndChild(world);
  _sdfXmlOut->LinkEndChild(sdf);
  return errors;
}

////////////////////////////////////////////////////////////////////////////////
void USD2SDF::AddLights(
  const std::map<std::string, std::shared_ptr<sdf::Light>> &_lights,
  tinyxml2::XMLElement *attach)
{
  for (auto & light : _lights)
  {
    std::shared_ptr<sdf::Light> sdfLight = light.second;
    tinyxml2::XMLElement *lightXML = attach->GetDocument()->NewElement("light");
    lightXML->SetAttribute("name", ignition::common::basename(sdfLight->Name()).c_str());

    std::string lightType;
    switch (sdfLight->Type()) {
      case sdf::LightType::DIRECTIONAL:
        lightType = "directional";
        break;
      case sdf::LightType::POINT:
        lightType = "point";
        break;
      case sdf::LightType::SPOT:
        lightType = "spot";
        break;
      default:
        lightType = "invalid";
    }
    lightXML->SetAttribute("type", lightType.c_str());

    double pose_value[6];
    pose_value[0] = sdfLight->RawPose().Pos().X();
    pose_value[1] = sdfLight->RawPose().Pos().Y();
    pose_value[2] = sdfLight->RawPose().Pos().Z();
    pose_value[3] = sdfLight->RawPose().Rot().Euler()[0];
    pose_value[4] = sdfLight->RawPose().Rot().Euler()[1];
    pose_value[5] = sdfLight->RawPose().Rot().Euler()[2];
    AddKeyValue(lightXML, "pose", Values2str(6, pose_value));

    double color_diffuse[4];
    color_diffuse[0] = sdfLight->Diffuse().R();
    color_diffuse[1] = sdfLight->Diffuse().G();
    color_diffuse[2] = sdfLight->Diffuse().B();
    color_diffuse[3] = sdfLight->Diffuse().A();
    AddKeyValue(lightXML, "diffuse", Values2str(4, color_diffuse));

    double color_specular[4];
    color_specular[0] = sdfLight->Specular().R();
    color_specular[1] = sdfLight->Specular().G();
    color_specular[2] = sdfLight->Specular().B();
    color_specular[3] = sdfLight->Specular().A();
    AddKeyValue(lightXML, "specular", Values2str(4, color_specular));

    double intensity = sdfLight->Intensity();
    AddKeyValue(lightXML, "intensity", Values2str(1, &intensity));

    tinyxml2::XMLElement *attenuationXML =
      lightXML->GetDocument()->NewElement("attenuation");

    double range = sdfLight->AttenuationRange();
    double constant = sdfLight->ConstantAttenuationFactor();
    double linear = sdfLight->LinearAttenuationFactor();
    double quadratic = sdfLight->QuadraticAttenuationFactor();
    AddKeyValue(attenuationXML, "range", Values2str(1, &range));
    AddKeyValue(attenuationXML, "constant", Values2str(1, &constant));
    AddKeyValue(attenuationXML, "linear", Values2str(1, &linear));
    AddKeyValue(attenuationXML, "quadratic", Values2str(1, &quadratic));

    double direction[3];
    direction[0] = sdfLight->Direction().X();
    direction[1] = sdfLight->Direction().Y();
    direction[2] = sdfLight->Direction().Z();
    AddKeyValue(lightXML, "direction", Values2str(3, direction));

    if (sdfLight->Type() == sdf::LightType::SPOT)
    {
      tinyxml2::XMLElement *spotXML =
        lightXML->GetDocument()->NewElement("spot");

      double innerAngle = sdfLight->SpotInnerAngle().Radian();
      double outerAngle = sdfLight->SpotOuterAngle().Radian();
      double falloff = sdfLight->SpotFalloff();
      AddKeyValue(spotXML, "inner_angle", Values2str(1, &innerAngle));
      AddKeyValue(spotXML, "outer_angle", Values2str(1, &outerAngle));
      AddKeyValue(spotXML, "falloff", Values2str(1, &falloff));
      lightXML->LinkEndChild(spotXML);
    }

    lightXML->LinkEndChild(attenuationXML);
    attach->LinkEndChild(lightXML);
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string USD2SDF::GetKeyValueAsString(tinyxml2::XMLElement* _elem)
{
  std::string valueStr;
  if (_elem->Attribute("value"))
  {
    valueStr = _elem->Attribute("value");
  }
  else if (_elem->FirstChild())
  {
    // Check that this node is a XMLText
    if (_elem->FirstChild()->ToText())
    {
      valueStr = _elem->FirstChild()->Value();
    }
    else
    {
      sdfwarn << "Attribute value string not set\n";
    }
  }
  return trim(valueStr);
}

////////////////////////////////////////////////////////////////////////////////
void USD2SDF::AddKeyValue(
  tinyxml2::XMLElement *_elem,
  const std::string &_key,
  const std::string &_value)
{
  tinyxml2::XMLElement *childElem = _elem->FirstChildElement(_key.c_str());
  if (childElem)
  {
    std::string oldValue = GetKeyValueAsString(childElem);
    if (oldValue != _value)
    {
      sdferr << "multiple inconsistent <" << _key
              << "> exists due to fixed joint reduction"
              << " overwriting previous value [" << oldValue
              << "] with [" << _value << "].\n";
    }
    else
    {
       sdferr << "multiple consistent <" << _key
              << "> exists with [" << _value
              << "] due to fixed joint reduction.\n";
    }
    _elem->DeleteChild(childElem);  // remove old _elem
  }

  auto *doc = _elem->GetDocument();
  tinyxml2::XMLElement *ekey = doc->NewElement(_key.c_str());
  tinyxml2::XMLText *textEkey = doc->NewText(_value.c_str());
  ekey->LinkEndChild(textEkey);
  _elem->LinkEndChild(ekey);
}

////////////////////////////////////////////////////////////////////////////////
std::string USD2SDF::Values2str(unsigned int _count, const double *_values)
{
  std::stringstream ss;
  ss.precision(16);
  for (unsigned int i = 0 ; i < _count ; ++i)
  {
    if (i > 0)
    {
      ss << " ";
    }
    if (std::fpclassify(_values[i]) == FP_ZERO)
      ss << 0;
    else
      ss << _values[i];
  }
  return ss.str();
}

/////////////////////////////////////////////////
std::string USD2SDF::Vector32Str(const ignition::math::Vector3d _vector)
{
  std::stringstream ss;
  ss << _vector.X();
  ss << " ";
  ss << _vector.Y();
  ss << " ";
  ss << _vector.Z();
  return ss.str();
}
}
}
}
