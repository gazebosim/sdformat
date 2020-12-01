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
#include <cstring>
#include <vector>

#include "sdf/ParamPassing.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {

//////////////////////////////////////////////////
void updateParams(const tinyxml2::XMLElement *_childXmlParams,
                  SDFPtr _includeSDF, Errors &_errors)
{
  // loop through <experimental:params> children
  const tinyxml2::XMLElement *childElemXml = nullptr;
  for (childElemXml = _childXmlParams->FirstChildElement();
       childElemXml;
       childElemXml = childElemXml->NextSiblingElement())
  {

    // element identifier
    const char *childElemId = childElemXml->Attribute("name");
    if (!childElemId)
    {
      tinyxml2::XMLPrinter printer;
      childElemXml->Accept(&printer);

      _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "Missing element identifier. Skipping element modification:\n"
        + std::string(printer.CStr())
      });
      continue;
    }

    std::string actionStr;
    if (childElemXml->Attribute("action"))
      actionStr = std::string(childElemXml->Attribute("action"));

    // get element pointer to specified element using element identifier
    ElementPtr  elem;
    if (actionStr == "add")
    {
      // TODO(jenn) implement this, skipping for now
      continue;
    }
    else
    {
      elem = getElementById(_includeSDF, childElemId, childElemXml->Name());
    }

    if (elem == nullptr)
    {
      tinyxml2::XMLPrinter printer;
      childElemXml->Accept(&printer);

      _errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Could not find element <" + std::string(childElemXml->Name())
        + " name='" + childElemXml->Attribute("name") + "'>. " +
        "Skipping element modification:\n" + printer.CStr()
      });
      continue;
    }

    // TODO(jenn) element modifications
  }
}

//////////////////////////////////////////////////
ElementPtr getElementById(const SDFPtr _sdf, const char *_elemId,
                          const char *_elemName)
{
  // make a copy of _elemId so std::strtok doesn't change original
  auto elemIdName = std::make_unique<char[]>(std::strlen(_elemId) + 1);
  std::strcpy(elemIdName.get(), _elemId);
  char *idToken = std::strtok(elemIdName.get(), "::");

  ParamPtr modelNameAttr
    = _sdf->Root()->GetFirstElement()->GetAttribute("name");
  std::string modelName = modelNameAttr->GetAsString();
  // TODO(jenn) error check if modelName.empty()?
  modelName += "::";

  // child element of includeSDF
  ElementPtr childElem = _sdf->Root()->GetFirstElement()->GetFirstElement();

  // look for element using element identifier tokens
  bool foundElement = false;
  while (idToken)
  {
    std::string childName;
    // iterate through included model
    while (childElem)
    {
      if (childElem->HasAttribute("name"))
      {
        childName = childElem->GetAttribute("name")->GetAsString();

        if (childElem->GetName() == _elemName &&
            childName == modelName + idToken)
        {
          foundElement = true;
          break;
        }

        if (childName == modelName + idToken)
        {
          childElem = childElem->GetFirstElement();
          modelName = "";
          break;
        }
      }

      childElem = childElem->GetNextElement();
    }

    idToken = std::strtok(NULL, "::");
  }

  if (!foundElement) childElem = nullptr;

  return childElem;
}

}
}
