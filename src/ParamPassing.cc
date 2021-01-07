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

#include "ParamPassing.hh"

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
        "Element identifier requires a name, but the name is not set. "
        "Skipping element modification:\n"
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
      elem = getElementById(_includeSDF, childElemXml->Name(),
                            childElemId, true);
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
ElementPtr getElementById(const SDFPtr _sdf,
                          const std::string &_elemName,
                          const std::string &_elemId,
                          const bool &_prefixModelName)
{
  std::string modelName
    = _sdf->Root()->GetFirstElement()->GetAttribute("name")->GetAsString();

  // TODO(jenn) add requirement model name in element identifier?
  // child element of includeSDF
  ElementPtr childElem = _sdf->Root()->GetFirstElement()->GetFirstElement();

  // start and stop idx of elemId for finding longest substring
  int64_t startIdx = 0, stopIdx = 0, longestIdx = 0;
  ElementPtr matchingChild = nullptr;

  // iterate through included model
  while (childElem)
  {
    std::string childName;
    if (childElem->HasAttribute("name"))
    {
      childName = childElem->GetAttribute("name")->GetAsString();

      if (startIdx == 0 && _prefixModelName)
      {
        stopIdx = findPrefixLastIndex(modelName + "::" + _elemId,
                                      startIdx,
                                      childName);
        if (stopIdx != -1)
        {
          // removing added model name
          stopIdx = stopIdx - (modelName.size()-1);
        }
      }
      else
      {
        stopIdx = findPrefixLastIndex(_elemId, startIdx, childName);

        if (!_prefixModelName)
          stopIdx += 3;  // past "::"
      }

      if (stopIdx > longestIdx)
      {
        matchingChild = childElem;

        // found matching element break out of included model iteration
        if (childElem->GetName() == _elemName
              && _elemId.substr(startIdx) == childName)
          break;

        longestIdx = stopIdx;
      }
    }

    childElem = childElem->GetNextElement();

    // when no more children & found potential match,
    // set next iteration to first element of match
    if (!childElem && matchingChild)
    {
      childElem = matchingChild->GetFirstElement();
      matchingChild = nullptr;
      startIdx = longestIdx;
    }
  }

  // found no element match
  if (!matchingChild)
    childElem = nullptr;

  return childElem;
}

//////////////////////////////////////////////////
int64_t findPrefixLastIndex(const std::string &_elemId,
                               const int64_t &_startIdx,
                               const std::string &_ref)
{
  if (_elemId.substr(_startIdx, _ref.size()) == _ref)
    return _startIdx + (_ref.size()-1);

  return -1;
}
}
}
