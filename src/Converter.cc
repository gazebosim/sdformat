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

using namespace sdf;

namespace {
bool EndsWith(const std::string& _a, const std::string& _b)
{
  return (_a.size() >= _b.size()) &&
      (_a.compare(_a.size() - _b.size(), _b.size(), _b) == 0);
}
}

/////////////////////////////////////////////////
bool Converter::Convert(TiXmlDocument *_doc, const std::string &_toVersion,
                        bool _quiet)
{
  SDF_ASSERT(_doc != nullptr, "SDF XML doc is NULL");

  TiXmlElement *elem = _doc->FirstChildElement("sdf");

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

  elem->SetAttribute("version", _toVersion);

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
    TiXmlDocument xmlDoc;
    xmlDoc.Parse(convertXml);
    if (xmlDoc.Error())
    {
      sdferr << "Error parsing XML from string: "
             << xmlDoc.ErrorDesc() << '\n';
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
void Converter::Convert(TiXmlDocument *_doc, TiXmlDocument *_convertDoc)
{
  SDF_ASSERT(_doc != NULL, "SDF XML doc is NULL");
  SDF_ASSERT(_convertDoc != NULL, "Convert XML doc is NULL");

  ConvertImpl(_doc->FirstChildElement(), _convertDoc->FirstChildElement());
}

/////////////////////////////////////////////////
void Converter::ConvertDescendantsImpl(TiXmlElement *_e, TiXmlElement *_c)
{
  if (!_c->Attribute("descendant_name"))
  {
    return;
  }

  if (_e->ValueStr() == "plugin")
  {
    return;
  }

  if (_e->ValueStr().find(":") != std::string::npos)
  {
    return;
  }

  std::string name = _c->Attribute("descendant_name");
  TiXmlElement *e = _e->FirstChildElement();
  while (e)
  {
    if (name == e->ValueStr())
    {
      ConvertImpl(e, _c);
    }
    ConvertDescendantsImpl(e, _c);
    e = e->NextSiblingElement();
  }
}

/////////////////////////////////////////////////
void Converter::ConvertImpl(TiXmlElement *_elem, TiXmlElement *_convert)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_convert != NULL, "Convert element is NULL");

  CheckDeprecation(_elem, _convert);

  for (TiXmlElement *convertElem = _convert->FirstChildElement("convert");
       convertElem; convertElem = convertElem->NextSiblingElement("convert"))
  {
    if (convertElem->Attribute("name"))
    {
      TiXmlElement *elem = _elem->FirstChildElement(
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

  for (TiXmlElement *childElem = _convert->FirstChildElement();
       childElem; childElem = childElem->NextSiblingElement())
  {
    if (childElem->ValueStr() == "rename")
    {
      Rename(_elem, childElem);
    }
    else if (childElem->ValueStr() == "copy")
    {
      Move(_elem, childElem, true);
    }
    else if (childElem->ValueStr() == "map")
    {
      Map(_elem, childElem);
    }
    else if (childElem->ValueStr() == "move")
    {
      Move(_elem, childElem, false);
    }
    else if (childElem->ValueStr() == "add")
    {
      Add(_elem, childElem);
    }
    else if (childElem->ValueStr() == "remove")
    {
      Remove(_elem, childElem);
    }
    else if (childElem->ValueStr() != "convert")
    {
      sdferr << "Unknown convert element[" << childElem->ValueStr() << "]\n";
    }
  }
}

/////////////////////////////////////////////////
void Converter::Rename(TiXmlElement *_elem, TiXmlElement *_renameElem)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_renameElem != NULL, "Rename element is NULL");

  TiXmlElement *fromConvertElem = _renameElem->FirstChildElement("from");
  TiXmlElement *toConvertElem = _renameElem->FirstChildElement("to");

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

  TiXmlElement *replaceTo = new TiXmlElement(toElemName);
  if (toAttrName)
  {
    replaceTo->SetAttribute(toAttrName, value);
  }
  else
  {
    TiXmlText *text = new TiXmlText(value);
    // The tinyxml function LinkEndChild takes the pointer and takes ownership
    // of the memory, so it is responsible for freeing it later.
    replaceTo->LinkEndChild(text);
  }

  if (fromElemName)
  {
    TiXmlElement *replaceFrom = _elem->FirstChildElement(fromElemName);
    if (_elem->ReplaceChild(replaceFrom, *replaceTo) == nullptr)
    {
      sdferr << "Failed to rename element\n";
      // fall through so we can reclaim memory
    }

    // In this case, the tinyxml function ReplaceChild does a deep copy of the
    // node that is passed in, so we want to free it here.
    delete replaceTo;
  }
  else if (fromAttrName)
  {
    _elem->RemoveAttribute(fromAttrName);
    // In this case, the tinyxml function LinkEndChild just takes the pointer
    // and takes ownership of the memory, so it is responsible for freeing it
    // later.
    _elem->LinkEndChild(replaceTo);
  }
}

/////////////////////////////////////////////////
void Converter::Add(TiXmlElement *_elem, TiXmlElement *_addElem)
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
    TiXmlElement *addElem = new TiXmlElement(elementName);
    if (value)
    {
      TiXmlText *addText = new TiXmlText(value);
      addElem->LinkEndChild(addText);
    }
    _elem->LinkEndChild(addElem);
  }
}

/////////////////////////////////////////////////
void Converter::Remove(TiXmlElement *_elem, TiXmlElement *_removeElem)
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
    _elem->RemoveAttribute(attributeName);
  }
  else
  {
    TiXmlElement *childElem = _elem->FirstChildElement(elementName);
    while (childElem)
    {
      _elem->RemoveChild(childElem);
      childElem = _elem->FirstChildElement(elementName);
    }
  }
}

/////////////////////////////////////////////////
void Converter::Map(TiXmlElement *_elem, TiXmlElement *_mapElem)
{
  SDF_ASSERT(_elem != nullptr, "SDF element is nullptr");
  SDF_ASSERT(_mapElem != nullptr, "Map element is nullptr");

  TiXmlElement *fromConvertElem = _mapElem->FirstChildElement("from");
  TiXmlElement *toConvertElem = _mapElem->FirstChildElement("to");

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
  TiXmlElement *fromValueElem = fromConvertElem->FirstChildElement("value");
  TiXmlElement *toValueElem = toConvertElem->FirstChildElement("value");
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
  TiXmlElement *fromElem = _elem;
  for (unsigned int i = 0; i < fromTokens.size()-1; ++i)
  {
    fromElem = fromElem->FirstChildElement(fromTokens[i]);
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
  TiXmlElement *toElem = _elem;
  TiXmlElement *childElem = NULL;
  for (unsigned int i = 0; i < toTokens.size()-1; ++i)
  {
    childElem = toElem->FirstChildElement(toTokens[i]);
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

      TiXmlElement *newElem = new TiXmlElement(toTokens[newDirIndex]);
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
    TiXmlText *text = new TiXmlText(toValue);
    toElem->LinkEndChild(text);
  }
}

/////////////////////////////////////////////////
void Converter::Move(TiXmlElement *_elem, TiXmlElement *_moveElem,
                     const bool _copy)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_moveElem != NULL, "Move element is NULL");

  TiXmlElement *fromConvertElem = _moveElem->FirstChildElement("from");
  TiXmlElement *toConvertElem = _moveElem->FirstChildElement("to");

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
  TiXmlElement *fromElem = _elem;
  for (unsigned int i = 0; i < fromTokens.size()-1; ++i)
  {
    fromElem = fromElem->FirstChildElement(fromTokens[i]);
    if (!fromElem)
    {
      // Return when the tokens don't match. Don't output an error message
      // because it spams the console.
      return;
    }
  }

  const char *fromName = fromTokens.back().c_str();
  const char *value = NULL;

  unsigned int newDirIndex = 0;
  // get the new element/attribute name
  const char *toName = toTokens.back().c_str();
  TiXmlElement *toElem = _elem;
  TiXmlElement *childElem = NULL;
  for (unsigned int i = 0; i < toTokens.size()-1; ++i)
  {
    childElem = toElem->FirstChildElement(toTokens[i]);
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
    int offset = toElemStr != NULL && toAttrStr != NULL ? 0 : 1;
    while (newDirIndex < (toTokens.size()-offset))
    {
      TiXmlElement *newElem = new TiXmlElement(toTokens[newDirIndex]);
      toElem->LinkEndChild(newElem);
      toElem = newElem;
      newDirIndex++;
    }
  }

  // Get value, or return if no element/attribute found as they don't have to
  // be specified in the sdf.
  if (fromElemStr)
  {
    TiXmlElement *moveFrom = fromElem->FirstChildElement(fromName);

    // No matching element, so return.
    if (!moveFrom)
    {
      return;
    }

    if (toElemStr && !toAttrStr)
    {
      TiXmlElement *moveTo = static_cast<TiXmlElement*>(moveFrom->Clone());
      moveTo->SetValue(toName);
      toElem->LinkEndChild(moveTo);
    }
    else
    {
      value = GetValue(fromName, NULL, fromElem);
      if (!value)
      {
        return;
      }
      std::string valueStr = value;

      toElem->SetAttribute(toAttrStr, valueStr);
    }

    if (!_copy)
    {
      fromElem->RemoveChild(moveFrom);
    }
  }
  else if (fromAttrStr)
  {
    value = GetValue(NULL, fromName, fromElem);

    if (!value)
    {
      return;
    }

    std::string valueStr = value;

    if (toElemStr)
    {
      TiXmlElement *moveTo = new TiXmlElement(toName);
      TiXmlText *text = new TiXmlText(valueStr);
      moveTo->LinkEndChild(text);
      toElem->LinkEndChild(moveTo);
    }
    else if (toAttrStr)
    {
      toElem->SetAttribute(toName, valueStr);
    }

    if (!_copy && fromAttrStr)
    {
      fromElem->RemoveAttribute(fromName);
    }
  }
}

/////////////////////////////////////////////////
const char *Converter::GetValue(const char *_valueElem, const char *_valueAttr,
                                TiXmlElement *_elem)
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
void Converter::CheckDeprecation(TiXmlElement *_elem, TiXmlElement *_convert)
{
  // Process deprecated elements
  for (TiXmlElement *deprecatedElem = _convert->FirstChildElement("deprecated");
       deprecatedElem;
       deprecatedElem = deprecatedElem->NextSiblingElement("deprecated"))
  {
    std::string value = deprecatedElem->GetText();
    std::vector<std::string> valueSplit = split(value, "/");

    bool found = false;
    TiXmlElement *e = _elem;
    std::ostringstream stream;

    std::string prefix = "";
    for (unsigned int i = 0; i < valueSplit.size() && !found; ++i)
    {
      if (e->FirstChildElement(valueSplit[i]))
      {
        if (stream.str().size() != 0)
        {
          stream << ">\n";
          prefix += "  ";
        }

        stream << prefix << "<" << valueSplit[i];
        e = e->FirstChildElement(valueSplit[i]);
      }
      else if (e->Attribute(valueSplit[i]))
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
