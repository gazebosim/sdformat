/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <algorithm>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sdf/Assert.hh"
#include "sdf/Console.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Types.hh"

#include "Converter.hh"
#include "EmbeddedSdf.hh"
#include "XmlUtils.hh"

using namespace sdf;

namespace {
bool EndsWith(const std::string& _a, const std::string& _b)
{
  return (_a.size() >= _b.size()) &&
      (_a.compare(_a.size() - _b.size(), _b.size(), _b) == 0);
}

/////////////////////////////////////////////////
// returns true if the element is not one of the listed for Unflatten conversion
bool IsNotFlattenedElement(const std::string &_elemName)
{
  return (_elemName != "frame" && _elemName != "joint" && _elemName != "link"
          && _elemName != "model" && _elemName != "gripper");
}

/////////////////////////////////////////////////
// used to update //pose/@relative_to in FindNewModelElements()
void UpdatePose(tinyxml2::XMLElement *_elem,
                const size_t &_childNameIdx,
                const std::string &_modelName)
{
  tinyxml2::XMLElement *pose = _elem->FirstChildElement("pose");
  if (pose && pose->Attribute("relative_to"))
  {
    std::string poseRelTo = pose->Attribute("relative_to");

    SDF_ASSERT(poseRelTo.compare(0, _modelName.size(), _modelName) == 0,
      "Error: Pose attribute 'relative_to' does not start with " + _modelName);

    poseRelTo = poseRelTo.substr(_childNameIdx);
    pose->SetAttribute("relative_to", poseRelTo.c_str());
  }

  if (_elem->FirstChildElement("camera"))
    UpdatePose(_elem->FirstChildElement("camera"), _childNameIdx, _modelName);
}
}

/////////////////////////////////////////////////
bool Converter::Convert(tinyxml2::XMLDocument *_doc,
                        const std::string &_toVersion,
                        bool _quiet)
{
  SDF_ASSERT(_doc != nullptr, "SDF XML doc is NULL");

  tinyxml2::XMLElement *elem = _doc->FirstChildElement("sdf");

  // Check that the <sdf> element exists
  if (!elem)
  {
    sdferr << "<sdf> element does not exist.\n";
    return false;
  }

  if (!elem || !elem->Attribute("version"))
  {
    sdferr << "  Unable to determine original SDF version\n";
    return false;
  }

  std::string origVersion = elem->Attribute("version");

  if (origVersion == _toVersion)
  {
    return true;
  }

  if (!_quiet)
  {
    sdfdbg << "Version[" << origVersion << "] to Version[" << _toVersion
           << "]\n"
           << "  Please use the gz sdf tool to update your SDF files.\n"
           << "    $ gz sdf -c [sdf_file]\n";
  }

  elem->SetAttribute("version", _toVersion.c_str());

  // The conversion recipes within the embedded files database are named, e.g.,
  // "1.8/1_7.convert" to upgrade from 1.7 to 1.8.
  const std::map<std::string, std::string> &embedded = GetEmbeddedSdf();

  // Apply the conversions one at a time until we reach the desired _toVersion.
  std::string curVersion = origVersion;
  while (curVersion != _toVersion)
  {
    // Find the (at most one) file named, e.g., ".../1_7.convert".
    std::string snakeVersion = curVersion;
    std::replace(snakeVersion.begin(), snakeVersion.end(), '.', '_');
    const std::string suffix = "/" + snakeVersion + ".convert";
    const char* convertXml = nullptr;
    for (const auto& [pathname, data] : embedded)
    {
      if (EndsWith(pathname, suffix))
      {
        curVersion = pathname.substr(0, pathname.size() - suffix.size());
        convertXml = data.c_str();
        break;
      }
    }
    if (convertXml == nullptr)
    {
      break;
    }

    // Parse and apply the conversion XML.
    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.Parse(convertXml);
    if (xmlDoc.Error())
    {
      sdferr << "Error parsing XML from string: "
             << xmlDoc.ErrorStr() << '\n';
      return false;
    }
    ConvertImpl(elem, xmlDoc.FirstChildElement("convert"));
  }

  // Check that we actually converted to the desired final version.
  if (curVersion != _toVersion)
  {
    sdferr << "Unable to convert from SDF version " << origVersion
           << " to " << _toVersion << "\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void Converter::Convert(tinyxml2::XMLDocument *_doc,
                        tinyxml2::XMLDocument *_convertDoc)
{
  SDF_ASSERT(_doc != NULL, "SDF XML doc is NULL");
  SDF_ASSERT(_convertDoc != NULL, "Convert XML doc is NULL");

  ConvertImpl(_doc->FirstChildElement(), _convertDoc->FirstChildElement());
}

/////////////////////////////////////////////////
void Converter::ConvertDescendantsImpl(tinyxml2::XMLElement *_e,
                                       tinyxml2::XMLElement *_c)
{
  if (!_c->Attribute("descendant_name"))
  {
    return;
  }

  if (strcmp(_e->Name(), "plugin") == 0)
  {
    return;
  }

  if (strchr(_e->Name(), ':') != nullptr)
  {
    return;
  }

  tinyxml2::XMLElement *e = _e->FirstChildElement();
  while (e)
  {
    if (strcmp(e->Name(), _c->Attribute("descendant_name")) == 0)
    {
      ConvertImpl(e, _c);
    }
    ConvertDescendantsImpl(e, _c);
    e = e->NextSiblingElement();
  }
}

/////////////////////////////////////////////////
void Converter::ConvertImpl(tinyxml2::XMLElement *_elem,
                            tinyxml2::XMLElement *_convert)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_convert != NULL, "Convert element is NULL");

  CheckDeprecation(_elem, _convert);

  for (auto *convertElem = _convert->FirstChildElement("convert");
       convertElem; convertElem = convertElem->NextSiblingElement("convert"))
  {
    if (convertElem->Attribute("name"))
    {
      tinyxml2::XMLElement *elem = _elem->FirstChildElement(
          convertElem->Attribute("name"));
      while (elem)
      {
        ConvertImpl(elem, convertElem);
        elem = elem->NextSiblingElement(convertElem->Attribute("name"));
      }
    }
    if (convertElem->Attribute("descendant_name"))
    {
      ConvertDescendantsImpl(_elem, convertElem);
    }
  }

  for (tinyxml2::XMLElement *childElem = _convert->FirstChildElement();
       childElem; childElem = childElem->NextSiblingElement())
  {
    const auto name = std::string(childElem->Name());

    if (name == "rename")
    {
      Rename(_elem, childElem);
    }
    else if (name == "copy")
    {
      Move(_elem, childElem, true);
    }
    else if (name == "map")
    {
      Map(_elem, childElem);
    }
    else if (name == "move")
    {
      Move(_elem, childElem, false);
    }
    else if (name == "add")
    {
      Add(_elem, childElem);
    }
    else if (name == "remove")
    {
      Remove(_elem, childElem);
    }
    else if (name == "unflatten")
    {
      Unflatten(_elem);
    }
    else if (name != "convert")
    {
      sdferr << "Unknown convert element[" << name << "]\n";
    }
  }
}

/////////////////////////////////////////////////
void Converter::Unflatten(tinyxml2::XMLElement *_elem)
{
  SDF_ASSERT(_elem != nullptr, "SDF element is nullptr");

  tinyxml2::XMLDocument *doc = _elem->GetDocument();

  tinyxml2::XMLElement *nextElem = nullptr, *firstUnflatModel = nullptr;
  for (tinyxml2::XMLElement *elem = _elem->FirstChildElement();
      elem;
      elem = nextElem)
  {
    // break loop if reached the first unflattened model
    if (firstUnflatModel == elem) break;

    nextElem = elem->NextSiblingElement();
    std::string elemName = elem->Name();

    // skip element if not one of the following or if missing name attribute
    if (IsNotFlattenedElement(elemName) || !elem->Attribute("name"))
      continue;

    std::string attrName = elem->Attribute("name");

    size_t found = attrName.find("::");
    if (found == std::string::npos)
    {
      // recursive unflatten
      if (elemName == "model")
      {
        Unflatten(elem);
        break;
      }

      continue;
    }

    std::string newModelName = attrName.substr(0, found);
    tinyxml2::XMLElement *newModel = doc->NewElement("model");
    newModel->SetAttribute("name", newModelName.c_str());

    if (FindNewModelElements(_elem, newModel, found + 2))
    {
      Unflatten(newModel);
      _elem->InsertEndChild(newModel);

      // since newModel is inserted at the end, point back to the top element
      nextElem = _elem->FirstChildElement();

      if (!firstUnflatModel)
        firstUnflatModel = newModel;
    }
  }
}

/////////////////////////////////////////////////
bool Converter::FindNewModelElements(tinyxml2::XMLElement *_elem,
                                    tinyxml2::XMLElement *_newModel,
                                    const size_t &_childNameIdx)
{
  bool unflattenedNewModel = false;
  std::string newModelName = _newModel->Attribute("name");
  size_t newModelNameSize = newModelName.size();

  // loop through looking for new model elements
  tinyxml2::XMLElement *elem = _elem->FirstChildElement(), *nextElem = nullptr;
  while (elem)
  {
    nextElem = elem->NextSiblingElement();

    std::string elemName = elem->Name();
    std::string elemAttrName;

    if (elem->Attribute("name"))
      elemAttrName = elem->Attribute("name");

    if (elemAttrName.empty() ||
        elemAttrName.compare(0, newModelNameSize, newModelName) != 0 ||
        IsNotFlattenedElement(elemName))
    {
      // since //gripper/@name may not be flattened but the children are
      // & elemAttrName.compare will evaluate to true, don't skip this element
      if (elemName != "gripper")
      {
        elem = nextElem;
        continue;
      }
    }

    // Child attribute name w/ newModelName prefix stripped except for
    // possibly //gripper, which may or may not have a prefix
    std::string childAttrName;
    if (elemAttrName.compare(0, newModelNameSize, newModelName) == 0)
    {
      childAttrName = elemAttrName.substr(_childNameIdx);
      elem->SetAttribute("name", childAttrName.c_str());
    }

    // strip new model prefix from //pose/@relative_to
    tinyxml2::XMLElement *poseElem = elem->FirstChildElement("pose");
    if (poseElem != nullptr && poseElem->Attribute("relative_to"))
    {
      std::string poseRelTo = poseElem->Attribute("relative_to");

      if (poseRelTo.compare(0, newModelNameSize, newModelName) == 0)
      {
        poseRelTo = poseRelTo.substr(_childNameIdx);
        poseElem->SetAttribute("relative_to", poseRelTo.c_str());
      }
    }

    if (elemName == "frame")
    {
      std::string attachedTo;

      if (elem->Attribute("attached_to"))
      {
        attachedTo = elem->Attribute("attached_to");

        SDF_ASSERT(attachedTo.compare(0, newModelNameSize, newModelName) == 0,
          "Error: Frame attribute 'attached_to' does not start with " +
          newModelName);

        // strip new model prefix from attached_to
        attachedTo = attachedTo.substr(_childNameIdx);
        elem->SetAttribute("attached_to", attachedTo.c_str());

        // remove frame if childAttrName == __model__
        if (childAttrName == "__model__")
        {
          _newModel->SetAttribute("canonical_link", attachedTo.c_str());
          _newModel->InsertFirstChild(poseElem);

          _elem->DeleteChild(elem);
          elem = poseElem;
        }
      }
    }  // frame

    else if (elemName == "link")
    {
      // find & strip new model prefix of all //link/<element>/pose/@relative_to
      for (tinyxml2::XMLElement *e = elem->FirstChildElement();
          e;
          e = e->NextSiblingElement())
      {
        UpdatePose(e, _childNameIdx, newModelName);
      }
    }  // link

    else if (elemName == "joint")
    {
      std::string eText;

      // strip new model prefix from //joint/parent
      tinyxml2::XMLElement *e = elem->FirstChildElement("parent");
      if (e != nullptr && e->GetText() != nullptr)
      {
        eText = e->GetText();

        SDF_ASSERT(eText.compare(0, newModelNameSize, newModelName) == 0,
        "Error: Joint's <parent> value does not start with " + newModelName);

        e->SetText(eText.substr(_childNameIdx).c_str());
      }

      // strip new model prefix from //joint/child
      e = elem->FirstChildElement("child");
      if (e != nullptr && e->GetText() != nullptr)
      {
        eText = e->GetText();

        SDF_ASSERT(eText.compare(0, newModelNameSize, newModelName) == 0,
        "Error: Joint's <child> value does not start with " + newModelName);

        e->SetText(eText.substr(_childNameIdx).c_str());
      }

      // strip new model prefix from //xyz/@expressed_in
      std::string axisStr = "axis";
      while (true)
      {
        tinyxml2::XMLElement *axisElem =
            elem->FirstChildElement(axisStr.c_str());
        if (axisElem != nullptr)
        {
          if (axisElem->FirstChildElement("xyz")->Attribute("expressed_in"))
          {
            std::string expressIn =
                axisElem->FirstChildElement("xyz")->Attribute("expressed_in");

            SDF_ASSERT(
              expressIn.compare(0, newModelNameSize, newModelName) == 0,
              "Error: <xyz>'s attribute 'expressed_in' does not start with " +
              newModelName);

            expressIn = expressIn.substr(_childNameIdx);

            axisElem->FirstChildElement("xyz")
                    ->SetAttribute("expressed_in", expressIn.c_str());
          }
        }

        if (axisStr == "axis2") break;

        axisStr += "2";
      }

      // strip new model prefix from all //joint/sensor/pose/@relative_to
      for (e = elem->FirstChildElement("sensor");
          e;
          e = e->NextSiblingElement("sensor"))
      {
        UpdatePose(e, _childNameIdx, newModelName);
      }
    }  // joint

    else if (elemName == "gripper")
    {
      bool hasPrefix = true;

      // strip prefix from all /model/gripper/gripper_link
      tinyxml2::XMLElement *e = elem->FirstChildElement("gripper_link");
      std::string eText;
      while (e)
      {
        if (e->GetText() != nullptr)
        {
          eText = e->GetText();

          if (eText.compare(0, newModelNameSize, newModelName) != 0)
          {
            hasPrefix = false;
            break;
          }

          e->SetText(eText.substr(_childNameIdx).c_str());
        }

        e = e->NextSiblingElement("gripper_link");
      }

      // if //model/gripper/gripper_link does not have new model prefix
      // don't add to new model
      if (!hasPrefix)
      {
        elem = nextElem;
        continue;
      }

      // strip prefix from //model/gripper/palm_link
      e = elem->FirstChildElement("palm_link");
      if (e != nullptr && e->GetText() != nullptr)
      {
        eText = e->GetText();

        SDF_ASSERT(eText.compare(0, newModelNameSize, newModelName) == 0,
        "Error: Gripper's <palm_link> value does not start with "
        + newModelName);

        e->SetText(eText.substr(_childNameIdx).c_str());
      }
    }  // gripper

    unflattenedNewModel = true;
    _newModel->InsertEndChild(elem);

    elem = nextElem;
  }

  return unflattenedNewModel;
}

/////////////////////////////////////////////////
void Converter::Rename(tinyxml2::XMLElement *_elem,
                       tinyxml2::XMLElement *_renameElem)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_renameElem != NULL, "Rename element is NULL");

  auto *fromConvertElem = _renameElem->FirstChildElement("from");
  auto *toConvertElem = _renameElem->FirstChildElement("to");

  const char *fromElemName = fromConvertElem->Attribute("element");
  const char *fromAttrName = fromConvertElem->Attribute("attribute");

  const char *toElemName = toConvertElem->Attribute("element");
  const char *toAttrName = toConvertElem->Attribute("attribute");

  const char *value = GetValue(fromElemName, fromAttrName, _elem);
  if (!value)
  {
    return;
  }

  if (!toElemName)
  {
    sdferr << "No 'to' element name specified\n";
    return;
  }

  auto *doc = _elem->GetDocument();
  tinyxml2::XMLElement *replaceTo = doc->NewElement(toElemName);
  if (toAttrName)
  {
    replaceTo->SetAttribute(toAttrName, value);
  }
  else
  {
    tinyxml2::XMLText *text = doc->NewText(value);
    replaceTo->LinkEndChild(text);
  }

  if (fromElemName)
  {
    tinyxml2::XMLElement *replaceFrom = _elem->FirstChildElement(fromElemName);
    _elem->InsertAfterChild(replaceFrom, replaceTo);
    _elem->DeleteChild(replaceFrom);
  }
  else if (fromAttrName)
  {
    _elem->DeleteAttribute(fromAttrName);
    _elem->LinkEndChild(replaceTo);
  }
}

/////////////////////////////////////////////////
void Converter::Add(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_addElem)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_addElem != NULL, "Add element is NULL");

  const char *attributeName = _addElem->Attribute("attribute");
  const char *elementName = _addElem->Attribute("element");
  const char *value = _addElem->Attribute("value");

  if (!((attributeName == nullptr) ^ (elementName == nullptr)))
  {
    sdferr << "Exactly one 'element' or 'attribute'"
           << " must be specified in <add>\n";
    return;
  }

  if (attributeName)
  {
    if (value)
    {
      _elem->SetAttribute(attributeName, value);
    }
    else
    {
      sdferr << "No 'value' specified in <add>\n";
      return;
    }
  }
  else
  {
    auto *doc = _elem->GetDocument();
    tinyxml2::XMLElement *addElem = doc->NewElement(elementName);
    if (value)
    {
      tinyxml2::XMLText *addText = doc->NewText(value);
      addElem->LinkEndChild(addText);
    }
    _elem->LinkEndChild(addElem);
  }
}

/////////////////////////////////////////////////
void Converter::Remove(tinyxml2::XMLElement *_elem,
                       tinyxml2::XMLElement *_removeElem)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_removeElem != NULL, "Move element is NULL");

  const char *attributeName = _removeElem->Attribute("attribute");
  const char *elementName = _removeElem->Attribute("element");

  if (!((attributeName == nullptr) ^ (elementName == nullptr)))
  {
    sdferr << "Exactly one 'element' or 'attribute'"
           << " must be specified in <remove>\n";
    return;
  }

  if (attributeName)
  {
    _elem->DeleteAttribute(attributeName);
  }
  else
  {
    tinyxml2::XMLElement *childElem = _elem->FirstChildElement(elementName);

    while (childElem)
    {
      _elem->DeleteChild(childElem);
      childElem = _elem->FirstChildElement(elementName);
    }
  }
}

/////////////////////////////////////////////////
void Converter::Map(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_mapElem)
{
  SDF_ASSERT(_elem != nullptr, "SDF element is nullptr");
  SDF_ASSERT(_mapElem != nullptr, "Map element is nullptr");

  tinyxml2::XMLElement *fromConvertElem = _mapElem->FirstChildElement("from");
  tinyxml2::XMLElement *toConvertElem = _mapElem->FirstChildElement("to");

  if (!fromConvertElem)
  {
    sdferr << "<map> element requires a <from> child element.\n";
    return;
  }
  if (!toConvertElem)
  {
    sdferr << "<map> element requires a <to> child element.\n";
    return;
  }

  const char *fromNameStr = fromConvertElem->Attribute("name");
  const char *toNameStr = toConvertElem->Attribute("name");

  if (!fromNameStr || fromNameStr[0] == '\0')
  {
    sdferr << "Map: <from> element requires a non-empty name attribute.\n";
    return;
  }
  if (!toNameStr || toNameStr[0] == '\0')
  {
    sdferr << "Map: <to> element requires a non-empty name attribute.\n";
    return;
  }

  // create map of input and output values
  std::map<std::string, std::string> valueMap;
  auto *fromValueElem = fromConvertElem->FirstChildElement("value");
  auto *toValueElem = toConvertElem->FirstChildElement("value");
  if (!fromValueElem)
  {
    sdferr << "Map: <from> element requires at least one <value> element.\n";
    return;
  }
  if (!toValueElem)
  {
    sdferr << "Map: <to> element requires at least one <value> element.\n";
    return;
  }
  if (!fromValueElem->GetText())
  {
    sdferr << "Map: from value must not be empty.\n";
    return;
  }
  if (!toValueElem->GetText())
  {
    sdferr << "Map: to value must not be empty.\n";
    return;
  }
  valueMap[fromValueElem->GetText()] = toValueElem->GetText();
  while (fromValueElem->NextSiblingElement("value"))
  {
    fromValueElem = fromValueElem->NextSiblingElement("value");
    if (toValueElem->NextSiblingElement("value"))
    {
      toValueElem = toValueElem->NextSiblingElement("value");
    }
    if (!fromValueElem->GetText())
    {
      sdferr << "Map: from value must not be empty.\n";
      return;
    }
    if (!toValueElem->GetText())
    {
      sdferr << "Map: to value must not be empty.\n";
      return;
    }
    valueMap[fromValueElem->GetText()] = toValueElem->GetText();
  }

  // tokenize 'from' and 'to' name attributes
  std::string fromStr = fromNameStr;
  std::string toStr = toNameStr;

  std::vector<std::string> fromTokens = split(fromStr, "/");
  std::vector<std::string> toTokens = split(toStr, "/");

  // split() always returns at least one element, even with the
  // empty string.  Thus we don't check if the fromTokens or toTokens are empty.

  // get value of the 'from' element/attribute
  tinyxml2::XMLElement *fromElem = _elem;
  for (unsigned int i = 0; i < fromTokens.size()-1; ++i)
  {
    fromElem = fromElem->FirstChildElement(fromTokens[i].c_str());
    if (!fromElem)
    {
      // Return when the tokens don't match. Don't output an error message
      // because it spams the console.
      return;
    }
  }

  const char *fromLeaf = fromTokens.back().c_str();
  if (fromLeaf[0] == '\0' ||
      (fromLeaf[0] == '@' && fromLeaf[1] == '\0'))
  {
    sdferr << "Map: <from> has invalid name attribute\n";
    return;
  }
  const char *fromValue = nullptr;
  if (fromLeaf[0] == '@')
  {
    // from an attribute
    fromValue = GetValue(nullptr, fromLeaf+1, fromElem);
  }
  else
  {
    // from an element
    fromValue = GetValue(fromLeaf, nullptr, fromElem);
  }

  if (!fromValue || valueMap.end() == valueMap.find(std::string(fromValue)))
  {
    // No match, no message to avoid spam.
    return;
  }
  const char *toValue = valueMap[std::string(fromValue)].c_str();
  // sdfdbg << "Map from [" << fromValue << "] to [" << toValue << "]\n";

  // check if destination elements before leaf exist and create if necessary
  unsigned int newDirIndex = 0;
  tinyxml2::XMLElement *toElem = _elem;
  tinyxml2::XMLElement *childElem = NULL;
  for (unsigned int i = 0; i < toTokens.size()-1; ++i)
  {
    childElem = toElem->FirstChildElement(toTokens[i].c_str());
    if (!childElem)
    {
      newDirIndex = i;
      break;
    }
    toElem = childElem;
  }

  // get the destination leaf name
  const char *toLeaf = toTokens.back().c_str();
  if (toLeaf[0] == '\0' ||
      (toLeaf[0] == '@' && toLeaf[1] == '\0'))
  {
    sdferr << "Map: <to> has invalid name attribute\n";
    return;
  }
  bool toAttribute = toLeaf[0] == '@';

  auto *doc = _elem->GetDocument();

  // found elements in 'to' string that are not present, so create new
  // elements if they aren't empty
  if (!childElem)
  {
    int offset = toAttribute ? 1 : 0;
    while (newDirIndex < (toTokens.size()-offset))
    {
      if (toTokens[newDirIndex].empty())
      {
        sdferr << "Map: <to> has invalid name attribute\n";
        return;
      }

      auto *newElem = doc->NewElement(toTokens[newDirIndex].c_str());
      toElem->LinkEndChild(newElem);
      toElem = newElem;
      newDirIndex++;
    }
  }

  if (toAttribute)
  {
    toElem->SetAttribute(toLeaf+1, toValue);
  }
  else
  {
    tinyxml2::XMLText *text = doc->NewText(toValue);
    toElem->LinkEndChild(text);
  }
}

/////////////////////////////////////////////////
void Converter::Move(tinyxml2::XMLElement *_elem,
                     tinyxml2::XMLElement *_moveElem,
                     const bool _copy)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_moveElem != NULL, "Move element is NULL");

  tinyxml2::XMLElement *fromConvertElem = _moveElem->FirstChildElement("from");
  tinyxml2::XMLElement *toConvertElem = _moveElem->FirstChildElement("to");

  const char *fromElemStr = fromConvertElem->Attribute("element");
  const char *fromAttrStr = fromConvertElem->Attribute("attribute");

  const char *toElemStr = toConvertElem->Attribute("element");
  const char *toAttrStr = toConvertElem->Attribute("attribute");

  // tokenize 'from' and 'to' strs
  std::string fromStr = "";
  if (fromElemStr)
  {
    fromStr = fromElemStr;
  }
  else if (fromAttrStr)
  {
    fromStr = fromAttrStr;
  }
  std::string toStr = "";
  if (toElemStr)
  {
    toStr = toElemStr;
  }
  else if (toAttrStr)
  {
    toStr = toAttrStr;
  }

  std::vector<std::string> fromTokens = split(fromStr, "::");
  std::vector<std::string> toTokens = split(toStr, "::");

  // split() always returns at least one element, even with the
  // empty string.  Thus we don't check if the fromTokens or toTokens are empty.

  // get value of the 'from' element/attribute
  tinyxml2::XMLElement *fromElem = _elem;
  for (unsigned int i = 0; i < fromTokens.size()-1; ++i)
  {
    fromElem = fromElem->FirstChildElement(fromTokens[i].c_str());
    if (!fromElem)
    {
      // Return when the tokens don't match. Don't output an error message
      // because it spams the console.
      return;
    }
  }

  const char *fromName = fromTokens.back().c_str();
  const char *value = nullptr;

  unsigned int newDirIndex = 0;
  // get the new element/attribute name
  const char *toName = toTokens.back().c_str();
  tinyxml2::XMLElement *toElem = _elem;
  tinyxml2::XMLElement *childElem = nullptr;
  for (unsigned int i = 0; i < toTokens.size()-1; ++i)
  {
    childElem = toElem->FirstChildElement(toTokens[i].c_str());
    if (!childElem)
    {
      newDirIndex = i;
      break;
    }
    toElem = childElem;
  }

  // found elements in 'to' string that are not present, so create new
  // elements
  if (!childElem)
  {
    int offset = toElemStr != nullptr && toAttrStr != nullptr ? 0 : 1;
    while (newDirIndex < (toTokens.size()-offset))
    {
      auto *doc = toElem->GetDocument();
      auto *newElem = doc->NewElement(toTokens[newDirIndex].c_str());
      toElem->LinkEndChild(newElem);
      toElem = newElem;
      newDirIndex++;
    }
  }

  // Get value, or return if no element/attribute found as they don't have to
  // be specified in the sdf.
  if (fromElemStr)
  {
    tinyxml2::XMLElement *moveFrom = fromElem->FirstChildElement(fromName);

    // No matching element, so return.
    if (!moveFrom)
    {
      return;
    }

    if (toElemStr && !toAttrStr)
    {
      tinyxml2::XMLNode *cloned = DeepClone(moveFrom->GetDocument(), moveFrom);
      tinyxml2::XMLElement *moveTo = static_cast<tinyxml2::XMLElement*>(cloned);

      moveTo->SetValue(toName);
      toElem->LinkEndChild(moveTo);
    }
    else
    {
      value = GetValue(fromName, nullptr, fromElem);
      if (!value)
      {
        return;
      }
      std::string valueStr = value;

      toElem->SetAttribute(toAttrStr, valueStr.c_str());
    }

    if (!_copy)
    {
      fromElem->DeleteChild(moveFrom);
    }
  }
  else if (fromAttrStr)
  {
    value = GetValue(nullptr, fromName, fromElem);

    if (!value)
    {
      return;
    }

    std::string valueStr = value;

    if (toElemStr)
    {
      auto *doc = toElem->GetDocument();
      tinyxml2::XMLElement *moveTo = doc->NewElement(toName);
      tinyxml2::XMLText *text = doc->NewText(valueStr.c_str());
      moveTo->LinkEndChild(text);
      toElem->LinkEndChild(moveTo);
    }
    else if (toAttrStr)
    {
      toElem->SetAttribute(toName, valueStr.c_str());
    }

    if (!_copy && fromAttrStr)
    {
      fromElem->DeleteAttribute(fromName);
    }
  }
}

/////////////////////////////////////////////////
const char *Converter::GetValue(const char *_valueElem, const char *_valueAttr,
                                tinyxml2::XMLElement *_elem)
{
  if (_valueElem)
  {
    // Check to see if the element that is being converted has the value
    if (!_elem->FirstChildElement(_valueElem))
    {
      return NULL;
    }

    if (_valueAttr)
    {
      return _elem->FirstChildElement(_valueElem)->Attribute(_valueAttr);
    }
    else
    {
      return _elem->FirstChildElement(_valueElem)->GetText();
    }
  }
  else if (_valueAttr)
  {
    return _elem->Attribute(_valueAttr);
  }

  return NULL;
}

/////////////////////////////////////////////////
void Converter::CheckDeprecation(tinyxml2::XMLElement *_elem,
                                 tinyxml2::XMLElement *_convert)
{
  // Process deprecated elements
  for (auto *deprecatedElem = _convert->FirstChildElement("deprecated");
       deprecatedElem;
       deprecatedElem = deprecatedElem->NextSiblingElement("deprecated"))
  {
    std::string value = deprecatedElem->GetText();
    std::vector<std::string> valueSplit = split(value, "/");

    bool found = false;
    tinyxml2::XMLElement *e = _elem;
    std::ostringstream stream;

    std::string prefix = "";
    for (unsigned int i = 0; i < valueSplit.size() && !found; ++i)
    {
      if (e->FirstChildElement(valueSplit[i].c_str()))
      {
        if (stream.str().size() != 0)
        {
          stream << ">\n";
          prefix += "  ";
        }

        stream << prefix << "<" << valueSplit[i];
        e = e->FirstChildElement(valueSplit[i].c_str());
      }
      else if (e->Attribute(valueSplit[i].c_str()))
      {
        stream << " " << valueSplit[i] << "='"
               << e->Attribute(valueSplit[i].c_str()) << "'";
        found = true;
      }
    }

    sdfwarn << "Deprecated SDF Values in original file:\n"
            << stream.str() << "\n\n";
  }
}
