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
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "sdf/Assert.hh"
#include "sdf/Console.hh"
#include "sdf/Converter.hh"
#include "sdf/Filesystem.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Types.hh"
#include "sdf/Types.hh"
#include "sdf/xml_util.hh"

using namespace sdf;

/////////////////////////////////////////////////
static bool case_insensitive_cmp(const char &_a, const char &_b)
{
  return tolower(_a) < tolower(_b);
}

/////////////////////////////////////////////////
bool Converter::Convert(tinyxml2::XMLDocument *_doc, const std::string &_toVersion,
                        bool _quiet)
{
  SDF_ASSERT(_doc != nullptr, "SDF XML doc is NULL");

  tinyxml2::XMLElement *elem = _doc->FirstChildElement("gazebo");

  // Replace <gazebo> with <sdf>
  if (elem && std::stod(_toVersion) >= 1.3)
  {
    elem->SetValue("sdf");
  }
  else if (!elem)
  {
    elem = _doc->FirstChildElement("sdf");
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

  std::string currFilenameVersion = origVersion;
  std::replace(currFilenameVersion.begin(), currFilenameVersion.end(),
               '.', '_');

  std::string filename = sdf::findFile(currFilenameVersion + ".convert");

  // Use convert file in the current sdf version folder for conversion. If file
  // does not exist, then find intermediate convert files and iteratively
  // convert the sdf elem. Ideally, users should use gzsdf convert so that the
  // latest sdf versioned file is written and no subsequent conversions are
  // necessary.
  tinyxml2::XMLDocument xmlDoc;
  if (filename.empty() || xmlDoc.LoadFile(filename.c_str()))
  {
    std::string sdfPath = sdf::findFile("sdformat/");
    if (sdfPath.empty()) 
    {
      sdferr << "Unable to find sdfPath[" << "sdformat/" << "]\n";
      return false;
    }

    // find all sdf version dirs in resource path
    sdf::filesystem::DirIter endIter;
    // Using a set here seems like overkill, since we mostly use it as a simple
    // iterator.  However, we have to make sure that anything we insert is
    // lexicographical order, and a set fits the bill.
    std::set<std::pair<std::string, std::string>> sdfDirs;
    if (sdf::filesystem::is_directory(sdfPath))
    {
      for (sdf::filesystem::DirIter dirIter(sdfPath);
           dirIter != endIter ; ++dirIter)
      {
        if (sdf::filesystem::is_directory(*dirIter))
        {
          std::string fname = sdf::filesystem::basename(*dirIter);
          if (std::lexicographical_compare(origVersion.begin(),
                                           origVersion.end(),
                                           fname.begin(),
                                           fname.end(),
                                           case_insensitive_cmp))
          {
            sdfDirs.insert(std::make_pair(*dirIter, fname));
          }
        }
      }
    }

    // loop through sdf dirs and do the intermediate conversions
    for (std::set<std::pair<std::string, std::string> >::iterator it =
           sdfDirs.begin(); it != sdfDirs.end(); ++it)
    {
      std::string convertFile = sdf::filesystem::append((*it).first,
          currFilenameVersion + ".convert");
      if (sdf::filesystem::exists(convertFile))
      {
        if (xmlDoc.LoadFile(convertFile.c_str()))
        {
          sdferr << "Unable to load file[" << convertFile << "]: " << xmlDoc.ErrorName() << "\n";
          return false;
        }
        ConvertImpl(elem, xmlDoc.FirstChildElement("convert"));
        if ((*it).second == _toVersion)
        {
          return true;
        }

        currFilenameVersion = (*it).second;
        std::replace(currFilenameVersion.begin(), currFilenameVersion.end(),
                     '.', '_');
      }
      else
      {
        continue;
      }
    }
    sdferr << "Unable to convert from SDF version " << origVersion
           << " to " << _toVersion << "\n";
    return false;
  }

  ConvertImpl(elem, xmlDoc.FirstChildElement("convert"));

  return true;
}

/////////////////////////////////////////////////
void Converter::Convert(tinyxml2::XMLDocument *_doc, tinyxml2::XMLDocument *_convertDoc)
{
  SDF_ASSERT(_doc != NULL, "SDF XML doc is NULL");
  SDF_ASSERT(_convertDoc != NULL, "Convert XML doc is NULL");

  ConvertImpl(_doc->FirstChildElement(), _convertDoc->FirstChildElement());
}

/////////////////////////////////////////////////
void Converter::ConvertImpl(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_convert)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_convert != NULL, "Convert element is NULL");

  CheckDeprecation(_elem, _convert);

  for (tinyxml2::XMLElement *convertElem = _convert->FirstChildElement("convert");
       convertElem; convertElem = convertElem->NextSiblingElement("convert"))
  {
    tinyxml2::XMLElement *elem = _elem->FirstChildElement(
        convertElem->Attribute("name"));
    while (elem)
    {
      ConvertImpl(elem, convertElem);
      elem = elem->NextSiblingElement(convertElem->Attribute("name"));
    }
  }

  for (tinyxml2::XMLElement *childElem = _convert->FirstChildElement();
       childElem; childElem = childElem->NextSiblingElement())
  {
    if (strcmp(childElem->Name(), "rename") == 0)
    {
      Rename(_elem, childElem);
    }
    else if (strcmp(childElem->Name(), "copy") == 0)
    {
      Move(_elem, childElem, true);
    }
    else if (strcmp(childElem->Name(), "move") == 0)
    {
      Move(_elem, childElem, false);
    }
    else if (strcmp(childElem->Name(), "add") == 0)
    {
      Add(_elem, childElem);
    }
    else if (strcmp(childElem->Name(), "remove") == 0)
    {
      Remove(_elem, childElem);
    }
    else if (strcmp(childElem->Name(), "convert") != 0)
    {
      sdferr << "Unknown convert element[" << childElem->Name() << "]\n";
    }
  }
}

/////////////////////////////////////////////////
void Converter::Rename(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_renameElem)
{
  SDF_ASSERT(_elem != NULL, "SDF element is NULL");
  SDF_ASSERT(_renameElem != NULL, "Rename element is NULL");

  tinyxml2::XMLElement *fromConvertElem = _renameElem->FirstChildElement("from");
  tinyxml2::XMLElement *toConvertElem = _renameElem->FirstChildElement("to");

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

  auto* doc = _elem->GetDocument();
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
    auto* doc = _elem->GetDocument();
    tinyxml2::XMLElement* addElem = doc->NewElement(elementName);
    if (value)
    {
      tinyxml2::XMLText *addText = doc->NewText(value);
      addElem->LinkEndChild(addText);
    }
    _elem->LinkEndChild(addElem);
  }
}

/////////////////////////////////////////////////
void Converter::Remove(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_removeElem)
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
    _elem->DeleteChild(_elem->FirstChildElement(elementName));
  }
}

/////////////////////////////////////////////////
void Converter::Move(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_moveElem,
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

  const char *fromName = fromTokens[fromTokens.size()-1].c_str();
  const char *value = nullptr;

  unsigned int newDirIndex = 0;
  // get the new element/attribute name
  const char *toName = toTokens[toTokens.size()-1].c_str();
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

  // found elements in 'to' string that is not present, so create new
  // elements
  if (!childElem)
  {
    int offset = toElemStr != nullptr && toAttrStr != nullptr ? 0 : 1;
    while (newDirIndex < (toTokens.size()-offset))
    {
      auto* doc = toElem->GetDocument();
      tinyxml2::XMLElement *newElem = doc->NewElement(toTokens[newDirIndex].c_str());
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
      tinyxml2::XMLElement *moveTo = static_cast<tinyxml2::XMLElement *>(DeepClone(moveFrom->GetDocument(), moveFrom));
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
      auto* doc = toElem->GetDocument();
      tinyxml2::XMLElement* moveTo = doc->NewElement(toName);
      tinyxml2::XMLText* text = doc->NewText(valueStr.c_str());
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
      return nullptr;
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

  return nullptr;
}

/////////////////////////////////////////////////
void Converter::CheckDeprecation(tinyxml2::XMLElement *_elem, tinyxml2::XMLElement *_convert)
{
  // Process deprecated elements
  for (tinyxml2::XMLElement *deprecatedElem = _convert->FirstChildElement("deprecated");
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
