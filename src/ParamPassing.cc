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
    {
      actionStr = std::string(childElemXml->Attribute("action"));

      if (!isValidAction(actionStr))
      {
        _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
          "Action [" + actionStr + "] is not a valid action. Skipping "
          "element modification:\n" + ElementToString(childElemXml)
        });
        continue;
      }
    }

    // get element pointer to specified element using element identifier
    ElementPtr elem;

    if (actionStr == "add")
    {
      const char *attr = childElemXml->Attribute("name");

      // checks name attribute exists
      if (!attr)
      {
        _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
          "Element to be added is missing a 'name' attribute. "
          "Skipping element addition:\n"
          + ElementToString(childElemXml)
        });
        continue;
      }

      // checks name attribute nonempty
      if (!strlen(attr))
      {
        _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
          "The 'name' attribute can not be empty. Skipping element addition:\n"
          + ElementToString(childElemXml)
        });
        continue;
      }

      std::string attrName = attr;

      // check that elem doesn't already exist
      elem  = getElementById(_includeSDF, childElemXml->Name(),
                            elemIdAttr + "::" + attrName);
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

      if (elemIdAttr.empty())
      {
        // add new element as direct child of included model
        elem = _includeSDF->Root()->GetFirstElement();
      }
      else
      {
        // get parent element of new element
        elem = getElementById(_includeSDF, "",
                              elemIdAttr,
                              true);
      }
    }
    else
    {
      elem  = getElementById(_includeSDF, childElemXml->Name(), elemIdAttr);
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
      add(childElemXml, elem, _errors);
    }
    else if (actionStr == "modify")
    {
    }
    else if (actionStr == "remove")
    {
      remove(childElemXml, elem, _errors);
    }
    else if (actionStr == "replace")
    {
    }

    // TODO(jenn) element modifications: modify, replace
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
bool isValidAction(const std::string &_action)
{
  return (_action == "add" || _action == "modify"
          || _action == "remove" || _action == "replace");
}

//////////////////////////////////////////////////
ElementPtr getElementByName(const ElementPtr _elem,
                            const tinyxml2::XMLElement *_xml)
{
  std::string elemName = _xml->Name();

  if (!_elem->HasElement(elemName))
    return nullptr;

  ElementPtr elem = _elem->GetElement(elemName);
  if (_xml->Attribute("name"))
  {
    while (elem != nullptr)
    {
      if (elem->HasAttribute("name") &&
          elem->Get<std::string>("name")
            == std::string(_xml->Attribute("name")))
      {
        return elem;
      }

      elem = elem->GetNextElement(elemName);
    }

    // if reached here then element was not found
    elem = nullptr;
  }
  else if (elem->HasAttribute("name"))
  {
    sdfwarn << "The original element [" << elemName << "] contains the "
            << "attribute 'name' but none was provided in the element modifier."
            << " The assumed element to be modified is: <" << elemName
            << " name='" << elem->Get<std::string>("name") << "'>\n";
  }

  return elem;
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
        + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
        + ElementToString(xmlChild)
      });
      continue;
    }

    std::string actionStr = xmlChild->Attribute("action");
    if (!isValidAction(actionStr))
    {
      _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
        "Action [" + actionStr + "] is not a valid action. Skipping "
        "child element modification with parent <"
        + std::string(_childrenXml->Name()) + " element_id='"
        + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
        + ElementToString(xmlChild)
      });

      continue;
    }

    if (actionStr == "remove")
    {
      ElementPtr e = getElementByName(_elem, xmlChild);
      if (e == nullptr)
      {
        _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Could not find element. Skipping child element removal "
          "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
          + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
          + ElementToString(xmlChild)
        });
      }
      else
      {
        remove(xmlChild, e, _errors);
      }

      continue;
    }

    // get child element description (used for actions: add, modify(?), replace)
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
          "Element [" + elemName + "] is not a defined SDF element or is an "
          "invalid child specification. Skipping "
          "child element modification with parent <"
          + std::string(_childrenXml->Name()) + " element_id='"
          + std::string(_childrenXml->Attribute("element_id")) + "'>: "
          + ElementToString(xmlChild)
        });
        continue;
      }
    }

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
    else if (actionStr == "modify")
    {
    }
    else if (actionStr == "replace")
    {
    }

    // TODO(jenn) finish (direct children only): modify, replace
  }
}


//////////////////////////////////////////////////
void add(tinyxml2::XMLElement *_childXml, ElementPtr _elem, Errors &_errors)
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

//////////////////////////////////////////////////
void remove(const tinyxml2::XMLElement *_xml, ElementPtr _elem, Errors &_errors)
{
  if (_xml->NoChildren())
  {
    _elem->RemoveFromParent();
  }
  else
  {
    // iterate through children of _xml
    ElementPtr elemChild = nullptr;
    const tinyxml2::XMLElement *xmlChild = nullptr;
    for (xmlChild = _xml->FirstChildElement();
         xmlChild;
         xmlChild = xmlChild->NextSiblingElement())
    {
      elemChild = getElementByName(_elem, xmlChild);
      if (elemChild == nullptr)
      {
        const tinyxml2::XMLElement *xmlParent = _xml->Parent()->ToElement();

        _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Could not find element. Skipping child element removal from <"
          + std::string(xmlParent->Name()) + " element_id='"
          + std::string(xmlParent->Attribute("element_id")) + "'> with parent <"
          + std::string(_xml->Name()) + ">:\n"
          + ElementToString(xmlChild)
        });

        continue;
      }

      elemChild->RemoveFromParent();
    }
  }
}
}
}
