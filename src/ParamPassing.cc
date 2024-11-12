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
#include <memory>
#include <vector>

#include "sdf/Filesystem.hh"
#include "sdf/parser.hh"
#include "sdf/config.hh"

#include "ParamPassing.hh"
#include "parser_private.hh"
#include "Utils.hh"
#include "XmlUtils.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace ParamPassing {

//////////////////////////////////////////////////
void updateParams(const ParserConfig &_config,
                  const std::string &_source,
                  tinyxml2::XMLElement *_childXmlParams,
                  ElementPtr _includeSDF,
                  Errors &_errors)
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
        "element_id is not set. Skipping element alteration:\n"
        + ElementToString(_errors, childElemXml)
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
        "Skipping element alteration:\n"
        + ElementToString(_errors, childElemXml)
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
          "element alteration:\n" + ElementToString(_errors, childElemXml)
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
          + ElementToString(_errors, childElemXml)
        });
        continue;
      }

      // checks name attribute nonempty
      if (!strlen(attr))
      {
        _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
          "The 'name' attribute can not be empty. Skipping element addition:\n"
          + ElementToString(_errors, childElemXml)
        });
        continue;
      }

      std::string attrName = attr;

      // check that elem doesn't already exist (except for //plugin)
      elem  = getElementById(_includeSDF, childElemXml->Name(),
                            elemIdAttr + "::" + attrName);
      if (elem != nullptr && elem->GetName() != "plugin")
      {
        _errors.push_back({ErrorCode::DUPLICATE_NAME,
          "Could not add element <" + std::string(childElemXml->Name())
          + " element_id='" + childElemXml->Attribute("element_id")
          + "'> because element already exists in included model. "
          + "Skipping element addition:\n"
          + ElementToString(_errors, childElemXml)
        });
        continue;
      }

      if (elemIdAttr.empty())
      {
        // add new element as direct child of included model
        elem = _includeSDF->GetFirstElement();
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
        "Skipping element modification:\n" +
        ElementToString(_errors, childElemXml)
      });
      continue;
    }

    // *** Element modifications ***

    if (actionStr.empty())
    {
      // action attribute not in childElemXml so must be in all direct children
      // of childElemXml
      handleIndividualChildActions(_config, _source,
                                   childElemXml, elem, _errors);
    }
    else if (actionStr == "add")
    {
      add(_config, _source, childElemXml, elem, _errors);
    }
    else if (actionStr == "modify")
    {
      modify(childElemXml, _config, elem, _errors);
    }
    else if (actionStr == "remove")
    {
      remove(childElemXml, _config, elem, _errors);
    }
    else if (actionStr == "replace")
    {
      ElementPtr newElem =
        initElementDescription(childElemXml, _config, _errors);
      if (!newElem)
        continue;

      if (!xmlToSdf(_config, _source, childElemXml, newElem, _errors))
      {
        _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Unable to convert XML to SDF. Skipping element replacement:\n"
          + ElementToString(_errors, childElemXml)
        });
        continue;
      }

      replace(newElem, elem);
    }
  }
}

//////////////////////////////////////////////////
ElementPtr getElementById(const ElementPtr _sdf,
                          const std::string &_elemName,
                          const std::string &_elemId,
                          const bool _isParentElement)
{
  // child element of includeSDF
  ElementPtr childElem = _sdf->GetFirstElement()->GetFirstElement();

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
                            const tinyxml2::XMLElement *_xml,
                            const sdf::ParserConfig _config,
                            sdf::Errors &_errors,
                            const bool _isModifyAction)
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

    // if action is modify then 'name' attribute is intended to be added
    if (_isModifyAction)
      elem = _elem->GetElement(elemName);
  }
  else if (elem->HasAttribute("name")
            && elem->GetAttribute("name")->GetRequired())
  {
    std::stringstream ss;
    ss << "The original element [" << elemName << "] contains the "
       << "attribute 'name' but none was provided in the element modifier."
       << " The assumed element to be modified is: <" << elemName
       << " name='" << elem->Get<std::string>("name") << "'>";
    Error err(ErrorCode::WARNING, ss.str());
    enforceConfigurablePolicyCondition(
        _config.WarningsPolicy(), err, _errors);
  }

  return elem;
}

//////////////////////////////////////////////////
ElementPtr initElementDescription(const tinyxml2::XMLElement *_xml,
                                  const ParserConfig &_config,
                                  Errors &_errors)
{
  ElementPtr elemDesc = std::make_shared<Element>();
  std::string filename = std::string(_xml->Name()) + ".sdf";

  if (!initFile(filename, _config, elemDesc))
  {
    // TODO(jenn) not sure if we should load the element anyway
    // (e.g., user created their own element), maybe future implementation
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
      "Element [" + std::string(_xml->Name()) + "] is not a defined "
      "SDF element. Skipping element alteration\n: "
      + ElementToString(_errors, _xml)
    });
    return nullptr;
  }
  return elemDesc;
}

//////////////////////////////////////////////////
void handleIndividualChildActions(const ParserConfig &_config,
                                  const std::string &_source,
                                  tinyxml2::XMLElement *_childrenXml,
                                  ElementPtr _elem,
                                  Errors &_errors)
{
  ElementPtr elemDesc = initElementDescription(_childrenXml, _config, _errors);
  if (!elemDesc)
    return;

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
        + ElementToString(_errors, xmlChild)
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
        + ElementToString(_errors, xmlChild)
      });
      continue;
    }

    if (actionStr == "remove")
    {
      ElementPtr e = getElementByName(_elem, xmlChild, _config, _errors);
      if (e == nullptr)
      {
        _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Could not find element. Skipping child element removal "
          "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
          + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
          + ElementToString(_errors, xmlChild)
        });
      }
      else
      {
        remove(xmlChild, _config, e, _errors);
      }

      continue;
    }
    else if (actionStr == "modify")
    {
      ElementPtr e = getElementByName(_elem, xmlChild, _config, _errors, true);
      if (e == nullptr)
      {
        _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Could not find element. Skipping child element modification "
          "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
          + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
          + ElementToString(_errors, xmlChild)
        });
      }
      else
      {
        modify(xmlChild, _config, e, _errors);
      }

      continue;
    }

    // get child element description (used for actions: add and replace)
    std::string elemName = xmlChild->Name();

    // the modification will be skipped if not an existing sdf element
    // TODO(jenn) not sure if we should load element anyway,
    // might need to be a future implementation
    if (!elemDesc->HasElementDescription(elemName))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Element [" + elemName + "] is not a defined SDF element or is an "
        "invalid child specification. Skipping "
        "child element modification with parent <"
        + std::string(_childrenXml->Name()) + " element_id='"
        + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
        + ElementToString(_errors, xmlChild)
      });
      continue;
    }

    ElementPtr elemChild = elemDesc->GetElementDescription(elemName);

    if (!xmlToSdf(_config, _source, xmlChild, elemChild, _errors))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Unable to convert XML to SDF. Skipping child element alteration "
        "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
        + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
        + ElementToString(_errors, xmlChild)
      });
      continue;
    }

    if (actionStr == "add")
    {
      _elem->InsertElement(elemChild->Clone(), true);
    }
    else if (actionStr == "replace")
    {
      ElementPtr e = getElementByName(_elem, xmlChild, _config, _errors);
      if (e == nullptr)
      {
        _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Could not find element. Skipping child element replacement "
          "with parent <" + std::string(_childrenXml->Name()) + " element_id='"
          + std::string(_childrenXml->Attribute("element_id")) + "'>:\n"
          + ElementToString(_errors, xmlChild)
        });
        continue;
      }

      // check name requirement of original and replacement element
      if (e->HasAttribute("name") && !xmlChild->Attribute("name"))
      {
        _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
          "Replacement element is missing a 'name' attribute. "
          "Skipping element replacement <" + std::string(_childrenXml->Name())
          + " element_id='" + std::string(_childrenXml->Attribute("element_id"))
          + "'>:\n" + ElementToString(_errors, xmlChild)
        });
        continue;
      }

      replace(elemChild, e);
    }
  }
}


//////////////////////////////////////////////////
void add(const ParserConfig &_config, const std::string &_source,
         tinyxml2::XMLElement *_childXml, ElementPtr _elem, Errors &_errors)
{
  ElementPtr newElem = initElementDescription(_childXml, _config, _errors);
  if (!newElem)
    return;

  if (xmlToSdf(_config, _source, _childXml, newElem, _errors))
  {
    _elem->InsertElement(newElem, true);
  }
  else
  {
    _errors.push_back({ErrorCode::ELEMENT_INVALID,
      "Unable to convert XML to SDF. Skipping element addition:\n"
      + ElementToString(_errors, _childXml)
    });
  }
}

//////////////////////////////////////////////////
void modifyAttributes(tinyxml2::XMLElement *_xml,
                      ElementPtr _elem, Errors &_errors)
{
  for (const tinyxml2::XMLAttribute *attr = _xml->FirstAttribute();
      attr; attr = attr->Next())
  {
    std::string attrName = attr->Name();

    if (attrName == "element_id" || attrName == "action")
      continue;

    if (!_elem->HasAttribute(attrName))
    {
      // since custom attribute is not in spec, HasAttribute will always fail
      // so need to add the attribute whereas other attributes that are part of
      // the spec HasAttribute will pass even if not set.
      // TODO(jenn) add attributes for namespaced elements
      if (attrName.find(":") != std::string::npos)
      {
        _elem->AddAttribute(attrName, "string", "", 1, "");
      }
      // invalid
      else
      {
        _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
          "Attribute [" + attrName + "] is invalid. "
          "Skipping attribute modification in:\n" +
          ElementToString(_errors, _xml)
        });
        continue;
      }
    }

    _elem->GetAttribute(attrName)->SetFromString(attr->Value());
  }
}

//////////////////////////////////////////////////
void modifyChildren(tinyxml2::XMLElement *_xml,
                    const sdf::ParserConfig &_config, ElementPtr _elem,
                    Errors &_errors)
{
  for (tinyxml2::XMLElement *xmlChild = _xml->FirstChildElement();
       xmlChild;
       xmlChild = xmlChild->NextSiblingElement())
  {
    std::string elemName = xmlChild->Name();

    if (!_elem->HasElement(elemName))
    {
      _errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Could not find element [" + elemName + "]. "
        "Skipping modification for:\n" + ElementToString(_errors, _xml)
      });
      continue;
    }

    ElementPtr elemChild = getElementByName(_elem, xmlChild,
                                            _config, _errors, true);
    ParamPtr paramChild = elemChild->GetValue();

    if (xmlChild->GetText())
    {
      // set xml value to sdf value
      if (paramChild && !paramChild->SetFromString(xmlChild->GetText()))
      {
        _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Value [" + std::string(xmlChild->GetText()) + "] for element ["
          + elemName + "] is invalid. Skipping modification for:\n"
          + ElementToString(_errors, _xml)
        });
        continue;
      }

      modifyAttributes(xmlChild, elemChild, _errors);
    }
    else if (xmlChild->NoChildren())
    {
      // no value was given
      if (paramChild)
      {
        // if sdf has no child elements, use default value
        paramChild->SetFromString(paramChild->GetDefaultAsString());
        modifyAttributes(xmlChild, elemChild, _errors);
      }
      else
      {
        // sdf has child elements but no children were specified in xml
        std::stringstream ss;
        ss << "No modifications for element "
           << ElementToString(_errors, xmlChild)
           << " provided, skipping modification for:\n"
           << ElementToString(_errors, _xml);
        Error err(ErrorCode::WARNING, ss.str());
        enforceConfigurablePolicyCondition(
            _config.WarningsPolicy(), err, _errors);
      }
    }
    else
    {
      modify(xmlChild, _config, elemChild, _errors);
    }
  }
}

//////////////////////////////////////////////////
void modify(tinyxml2::XMLElement *_xml,  const sdf::ParserConfig &_config,
            ElementPtr _elem, Errors &_errors)
{
  modifyAttributes(_xml, _elem, _errors);

  if (_xml->GetText())
  {
    // modify value of element
    ParamPtr param = _elem->GetValue();
    if (param && !param->SetFromString(_xml->GetText()))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
        "Value [" + std::string(_xml->GetText()) + "] for element [" +
        std::string(_xml->Name()) + "] is invalid. Skipping modification for:\n"
        + ElementToString(_errors, _xml)
      });
    }
  }
  else
  {
    // modify children elements
    modifyChildren(_xml, _config, _elem, _errors);
  }
}

//////////////////////////////////////////////////
void remove(const tinyxml2::XMLElement *_xml, const sdf::ParserConfig &_config,
            ElementPtr _elem, Errors &_errors)
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
      elemChild = getElementByName(_elem, xmlChild, _config, _errors);
      if (elemChild == nullptr)
      {
        const tinyxml2::XMLElement *xmlParent = _xml->Parent()->ToElement();

        _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "Could not find element. Skipping child element removal from <"
          + std::string(xmlParent->Name()) + " element_id='"
          + std::string(xmlParent->Attribute("element_id")) + "'> with parent <"
          + std::string(_xml->Name()) + ">:\n"
          + ElementToString(_errors, xmlChild)
        });
        continue;
      }

      elemChild->RemoveFromParent();
    }
  }
}

//////////////////////////////////////////////////
void replace(const ElementPtr _newElem, ElementPtr _origElem)
{
  if (!_newElem || !_origElem)
    return;

  _origElem->ClearElements();
  _origElem->RemoveAllAttributes();
  _origElem->Copy(_newElem);
}

//////////////////////////////////////////////////
bool xmlToSdf(const ParserConfig &_config, const std::string &_source,
              tinyxml2::XMLElement *_xml, ElementPtr _sdf, Errors &_errors)
{
  _xml->DeleteAttribute("element_id");
  _xml->DeleteAttribute("action");

  return readXml(_xml, _sdf, _config, _source, _errors);
}
}
}
}
