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

#include "USD.hh"

namespace sdf {
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
  std::string worldName = worldInterface->_worldName;
  if (worldName.empty())
  {
    worldName = "world_name";
  }
  world->SetAttribute("name", (worldName + "_world").c_str());

  AddKeyValue(world, "gravity", Vector32Str(
    worldInterface->gravity * worldInterface->magnitude));

  sdf->LinkEndChild(world);
  _sdfXmlOut->LinkEndChild(sdf);
  return errors;
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
