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

#include "sdf/Filesystem.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"

#include "ParamPassing.hh"
#include "parser_private.hh"
#include "XmlUtils.hh"

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
      _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "Element identifier requires an element_id attribute, but the "
        "element_id is not set. Skipping element modification:\n"
        + ElementToString(childElemXml)
      });
      continue;
    }

    std::string elemIdAttr(childElemId);

    // checks for name after last set of double colons
    size_t found = elemIdAttr.rfind("::");
    if (found != std::string::npos
        && (elemIdAttr.substr(found+2)).empty())
    {
      _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Missing name after double colons in element identifier. "
        "Skipping element modification:\n"
        + ElementToString(childElemXml)
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
        _errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Could not add element <" + std::string(childElemXml->Name())
          + " element_id='" + childElemXml->Attribute("element_id")
          + "'> because element already exists in included model. "
          + "Skipping element addition:\n"
          + ElementToString(childElemXml)
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
        elem = _includeSDF->Root()->GetFirstElement();
      }
      else
      {
        // get parent element of childElemId.substr(found+2)
        elem = getElementById(_includeSDF, "",
                              std::string(childElemId).substr(0, found),
                              true);
      }
    }

    if (elem == nullptr)
    {
      _errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Could not find element <" + std::string(childElemXml->Name())
        + " element_id='" + childElemXml->Attribute("element_id") + "'>. " +
        "Skipping element modification:\n" + ElementToString(childElemXml)
      });
      continue;
    }

    // *** Element modifications ***

    if (actionStr.empty())
    {
      // action attribute not in childElemXml so must be in all direct children
      // of childElemXml
      handleIndividualChildActions(childElemXml, elem, _errors);
    }
    else if (actionStr == "add")
    {
      add(childElemXml, elem, _errors, elemIdAttr);
    }

    // TODO(jenn) element modifications: modify, remove, replace
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

      size_t found = _elemId.find("::", startIdx);
      if (found != std::string::npos && (found - startIdx) == 1ul)
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

//////////////////////////////////////////////////
void handleIndividualChildActions(tinyxml2::XMLElement *_childrenXml,
                                  ElementPtr _elem, Errors &_errors)
{
  // get element description
  ElementPtr elemDesc = std::make_shared<Element>();
  std::string filename = std::string(_childrenXml->Name()) + ".sdf";

  if (!initFile(filename, elemDesc))
  {
    // TODO(jenn) not sure if we should load the element anyway
    // (e.g., user created their own element), maybe future implementation
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
      "Element [" + std::string(_childrenXml->Name()) + "] is not a defined "
      "SDF element. Skipping element modification: "
      + ElementToString(_childrenXml)
    });
    return;
  }

  // loop through children and handle corresponding actions
  for (tinyxml2::XMLElement *xmlChild = _childrenXml->FirstChildElement();
       xmlChild;
       xmlChild = xmlChild->NextSiblingElement())
  {
    if (xmlChild->Attribute("action") == nullptr)
    {
      _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "Missing an action attribute. Skipping child element modification "
        "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
        + std::string(_childrenXml->Attribute("element_id")) + "'>: "
        + ElementToString(xmlChild)
      });
      continue;
    }


    bool useParentDesc = false;
    std::string elemName = xmlChild->Name();

    // the modification will be skipped if not an existing sdf element
    // TODO(jenn) not sure if we should load element anyway,
    // might need to be a future implementation
    if (!elemDesc->HasElementDescription(elemName))
    {
      // TODO(anyone) when forward porting to sdf11,
      // need to add `|| elemName == "model"` since nested models are allowed
      if (elemName == "link")
      {
        useParentDesc = true;
      }
      else
      {
        _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Element [" + elemName + "] is not a defined SDF element. Skipping "
          "child element modification with parent <"
          + std::string(_childrenXml->Name()) + " element_id='"
          + std::string(_childrenXml->Attribute("element_id")) + "'>: "
          + ElementToString(xmlChild)
        });
        continue;
      }
    }

    std::string actionStr = xmlChild->Attribute("action");

    // get child element description
    ElementPtr elemChild;
    if (useParentDesc)
      elemChild = elemDesc;
    else
      elemChild = elemDesc->GetElementDescription(elemName);

    if (!readXml(xmlChild, elemChild, _errors))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Unable to convert XML to SDF. Skipping child element modification "
        "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
        + std::string(_childrenXml->Attribute("element_id")) + "'>: "
        + ElementToString(xmlChild)
      });
      continue;
    }

    if (actionStr == "add")
    {
      _elem->InsertElement(elemChild);
    }

    // TODO(jenn) finish (direct children only): modify, remove, replace
  }
}


//////////////////////////////////////////////////
void add(tinyxml2::XMLElement *_childXml, ElementPtr _elem,
         Errors &_errors, const std::string &_elemNameAttr)
{
  ElementPtr newElem = std::make_shared<Element>();
  std::string filename = std::string(_childXml->Name()) + ".sdf";

  if (!initFile(filename, newElem))
  {
    // TODO(jenn) not sure if we should load the element anyway
    // (e.g., user created their own element), maybe future implementation
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
      "Element [" + std::string(_childXml->Name()) + "] is not a defined "
      "SDF element. Skipping element modification: "
      + ElementToString(_childXml)
    });
    return;
  }

  newElem->GetAttribute("name")->Set(_elemNameAttr);

  if (readXml(_childXml, newElem, _errors))
  {
    _elem->InsertElement(newElem);
  }
  else
  {
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
      "Unable to convert XML to SDF. Skipping element modification: "
      + ElementToString(_childXml)
    });
  }
}
}
}
