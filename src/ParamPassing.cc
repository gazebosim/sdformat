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
void updateParams(tinyxml2::XMLElement *_childXmlParams,
                  SDFPtr _includeSDF, Errors &_errors)
{
  // loop through <experimental:params> children
  tinyxml2::XMLElement *childElemXml = nullptr;
  for (childElemXml = _childXmlParams->FirstChildElement();
       childElemXml;
       childElemXml = childElemXml->NextSiblingElement())
  {
    // element identifier
    const char *childElemId = childElemXml->Attribute("element_id");
    if (!childElemId)
    {
      tinyxml2::XMLPrinter printer;
      childElemXml->Accept(&printer);

      _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "Element identifier requires an element_id attribute, but the "
        "element_id is not set. Skipping element modification:\n"
        + std::string(printer.CStr())
      });
      continue;
    }

    std::string elemIdAttr(childElemId);

    // checks for name after last set of double colons
    size_t found = elemIdAttr.rfind("::");
    if (found != std::string::npos
        && (elemIdAttr.substr(found+2)).empty())
    {
      tinyxml2::XMLPrinter printer;
      childElemXml->Accept(&printer);

      _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Missing name after double colons in element identifier. "
        "Skipping element modification:\n"
        + std::string(printer.CStr())
      });
      continue;
    }

    // *** Retrieve specified element using element identifier ***

    std::string actionStr;
    if (childElemXml->Attribute("action"))
      actionStr = std::string(childElemXml->Attribute("action"));

    // get element pointer to specified element using element identifier
    ElementPtr elem = getElementById(_includeSDF, childElemXml->Name(),
                          childElemId);

    if (actionStr == "add")
    {
      if (elem != nullptr)
      {
        // TODO(jenn) make test for this
        tinyxml2::XMLPrinter printer;
        childElemXml->Accept(&printer);

        _errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Could not add element <" + std::string(childElemXml->Name())
          + " element_id='" + childElemXml->Attribute("element_id")
          + "'> because element already exists in included model. "
          + "Skipping element addition:\n"
          + printer.CStr()
        });
        continue;
      }

      if (found != std::string::npos)
      {
        // +2 past double colons
        elemIdAttr = elemIdAttr.substr(found+2);
      }

      if (!elemIdAttr.compare(childElemId))
      {
        // if equal add new element as direct child of included model
        elem = _includeSDF->Root()->GetFirstElement();  // model element
      }
      else
      {
        // get parent element of childElemId.substr(found+2)
        elem = getElementById(_includeSDF, "",
                              std::string(childElemId).substr(0, found),
                              true);

        // TODO(jenn) add error case for DUPLICATE_ELEMENT
      }
    }

    if (elem == nullptr)
    {
      tinyxml2::XMLPrinter printer;
      childElemXml->Accept(&printer);

      _errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Could not find element <" + std::string(childElemXml->Name())
        + " element_id='" + childElemXml->Attribute("element_id") + "'>. " +
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
                          const bool _isParentElement)
{
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

      // for add action, found parent element
      if (_isParentElement && _elemId == childName)
        return childElem;

      stopIdx = findPrefixLastIndex(_elemId, startIdx, childName);

      if (stopIdx > longestIdx)
      {
        matchingChild = childElem;

        // found matching element break out of included model iteration
        if (!_isParentElement && childElem->GetName() == _elemName
              && _elemId.substr(startIdx) == childName)
          break;
        else if (_isParentElement && _elemId.substr(startIdx) == childName)
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

      size_t found = _elemId.substr(startIdx).find("::");
      if (found != std::string::npos && found == 1u)
      {
        // past double colons
        startIdx += 3;
      }
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
