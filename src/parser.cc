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

#include <iostream>
#include <cstdlib>
#include <map>
#include <string>

#include <ignition/math/SemanticVersion.hh>

#include "sdf/Console.hh"
#include "sdf/Filesystem.hh"
#include "sdf/Frame.hh"
#include "sdf/Joint.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Param.hh"
#include "sdf/Root.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/World.hh"
#include "sdf/parser.hh"
#include "sdf/sdf_config.h"

#include "Converter.hh"
#include "FrameSemantics.hh"
#include "parser_private.hh"
#include "parser_urdf.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
//////////////////////////////////////////////////
/// \brief Internal helper for readFile, which populates the SDF values
/// from a file
///
/// This populates the given sdf pointer from a file. If the file is a URDF
/// file it is converted to SDF first. Conversion to the latest
/// SDF version is controlled by a function parameter.
/// \param[in] _filename Name of the SDF file
/// \param[in] _sdf Pointer to an SDF object.
/// \param[in] _convert Convert to the latest version if true.
/// \param[out] _errors Parsing errors will be appended to this variable.
/// \return True if successful.
bool readFileInternal(
    const std::string &_filename,
    SDFPtr _sdf,
    const bool _convert,
    Errors &_errors);

/// \brief Internal helper for readString, which populates the SDF values
/// from a string
///
/// This populates the sdf pointer from a string. If the string is from a URDF
/// file it is converted to SDF first. Conversion to the latest
/// SDF version is controlled by a function parameter.
/// \param[in] _xmlString XML string to be parsed.
/// \param[in] _sdf Pointer to an SDF object.
/// \param[in] _convert Convert to the latest version if true.
/// \param[out] _errors Parsing errors will be appended to this variable.
/// \return True if successful.
bool readStringInternal(
    const std::string &_xmlString,
    SDFPtr _sdf,
    const bool _convert,
    Errors &_errors);

//////////////////////////////////////////////////
/// \brief Internal helper for creating XMLDocuments
///
/// This creates an XMLDocument with whitespace collapse
/// on, which is not default behavior in tinyxml2.
/// This function is to consolidate locations it is used.
///
/// There is a performance impact associated with collapsing whitespace.
///
/// For more information on the behavior and performance implications,
/// consult the TinyXML2 documentation: https://leethomason.github.io/tinyxml2/
inline auto makeSdfDoc()
{
  return tinyxml2::XMLDocument(true, tinyxml2::COLLAPSE_WHITESPACE);
}

//////////////////////////////////////////////////
template <typename TPtr>
static inline bool _initFile(const std::string &_filename, TPtr _sdf)
{
  auto xmlDoc = makeSdfDoc();
  if (tinyxml2::XML_SUCCESS != xmlDoc.LoadFile(_filename.c_str()))
  {
    sdferr << "Unable to load file["
           << _filename << "]: " << xmlDoc.ErrorStr() << "\n";
    return false;
  }

  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool init(SDFPtr _sdf)
{
  std::string xmldata = SDF::EmbeddedSpec("root.sdf", false);
  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(xmldata.c_str());
  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool initFile(const std::string &_filename, SDFPtr _sdf)
{
  std::string xmldata = SDF::EmbeddedSpec(_filename, true);
  if (!xmldata.empty())
  {
    auto xmlDoc = makeSdfDoc();
    xmlDoc.Parse(xmldata.c_str());
    return initDoc(&xmlDoc, _sdf);
  }
  return _initFile(sdf::findFile(_filename), _sdf);
}

//////////////////////////////////////////////////
bool initFile(const std::string &_filename, ElementPtr _sdf)
{
  std::string xmldata = SDF::EmbeddedSpec(_filename, true);
  if (!xmldata.empty())
  {
    auto xmlDoc = makeSdfDoc();
    xmlDoc.Parse(xmldata.c_str());
    return initDoc(&xmlDoc, _sdf);
  }
  return _initFile(sdf::findFile(_filename), _sdf);
}

//////////////////////////////////////////////////
bool initString(const std::string &_xmlString, SDFPtr _sdf)
{
  auto xmlDoc = makeSdfDoc();
  if (xmlDoc.Parse(_xmlString.c_str()))
  {
    sdferr << "Failed to parse string as XML: " << xmlDoc.ErrorStr() << '\n';
    return false;
  }

  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
inline tinyxml2::XMLElement *_initDocGetElement(tinyxml2::XMLDocument *_xmlDoc)
{
  if (!_xmlDoc)
  {
    sdferr << "Could not parse the xml\n";
    return nullptr;
  }

  tinyxml2::XMLElement *element = _xmlDoc->FirstChildElement("element");
  if (!element)
  {
    sdferr << "Could not find the 'element' element in the xml file\n";
    return nullptr;
  }

  return element;
}

//////////////////////////////////////////////////
bool initDoc(tinyxml2::XMLDocument *_xmlDoc, SDFPtr _sdf)
{
  auto element = _initDocGetElement(_xmlDoc);
  if (!element)
  {
    return false;
  }

  return initXml(element, _sdf->Root());
}

//////////////////////////////////////////////////
bool initDoc(tinyxml2::XMLDocument *_xmlDoc, ElementPtr _sdf)
{
  auto element = _initDocGetElement(_xmlDoc);
  if (!element)
  {
    return false;
  }

  return initXml(element, _sdf);
}

//////////////////////////////////////////////////
bool initXml(tinyxml2::XMLElement *_xml, ElementPtr _sdf)
{
  const char *refString = _xml->Attribute("ref");
  if (refString)
  {
    _sdf->SetReferenceSDF(std::string(refString));
  }

  const char *nameString = _xml->Attribute("name");
  if (!nameString)
  {
    sdferr << "Element is missing the name attribute\n";
    return false;
  }
  _sdf->SetName(std::string(nameString));

  const char *requiredString = _xml->Attribute("required");
  if (!requiredString)
  {
    sdferr << "Element is missing the required attributed\n";
    return false;
  }
  _sdf->SetRequired(requiredString);

  const char *elemTypeString = _xml->Attribute("type");
  if (elemTypeString)
  {
    bool required = std::string(requiredString) == "1" ? true : false;
    const char *elemDefaultValue = _xml->Attribute("default");
    std::string description;
    tinyxml2::XMLElement *descChild = _xml->FirstChildElement("description");
    if (descChild && descChild->GetText())
    {
      description = descChild->GetText();
    }

    std::string minValue;
    const char *elemMinValue = _xml->Attribute("min");
    if (nullptr != elemMinValue)
    {
      minValue = elemMinValue;
    }

    std::string maxValue;
    const char *elemMaxValue = _xml->Attribute("max");
    if (nullptr != elemMaxValue)
    {
      maxValue = elemMaxValue;
    }

    _sdf->AddValue(elemTypeString, elemDefaultValue, required, minValue,
                   maxValue, description);
  }

  // Get all attributes
  for (tinyxml2::XMLElement *child = _xml->FirstChildElement("attribute");
       child; child = child->NextSiblingElement("attribute"))
  {
    auto *descriptionChild = child->FirstChildElement("description");
    const char *name = child->Attribute("name");
    const char *type = child->Attribute("type");
    const char *defaultValue = child->Attribute("default");

    requiredString = child->Attribute("required");

    if (!name)
    {
      sdferr << "Attribute is missing a name\n";
      return false;
    }
    if (!type)
    {
      sdferr << "Attribute is missing a type\n";
      return false;
    }
    if (!defaultValue)
    {
      sdferr << "Attribute[" << name << "] is missing a default\n";
      return false;
    }
    if (!requiredString)
    {
      sdferr << "Attribute is missing a required string\n";
      return false;
    }
    std::string requiredStr = sdf::trim(requiredString);
    bool required = requiredStr == "1" ? true : false;
    std::string description;

    if (descriptionChild && descriptionChild->GetText())
    {
      description = descriptionChild->GetText();
    }

    _sdf->AddAttribute(name, type, defaultValue, required, description);
  }

  // Read the element description
  tinyxml2::XMLElement *descChild = _xml->FirstChildElement("description");
  if (descChild && descChild->GetText())
  {
    _sdf->SetDescription(descChild->GetText());
  }

  // Get all child elements
  for (tinyxml2::XMLElement *child = _xml->FirstChildElement("element");
       child; child = child->NextSiblingElement("element"))
  {
    const char *copyDataString = child->Attribute("copy_data");
    if (copyDataString &&
        (std::string(copyDataString) == "true" ||
         std::string(copyDataString) == "1"))
    {
      _sdf->SetCopyChildren(true);
    }
    else
    {
      ElementPtr element(new Element);
      initXml(child, element);
      _sdf->AddElementDescription(element);
    }
  }

  // Get all include elements
  for (tinyxml2::XMLElement *child = _xml->FirstChildElement("include");
       child; child = child->NextSiblingElement("include"))
  {
    std::string filename = child->Attribute("filename");

    ElementPtr element(new Element);

    initFile(filename, element);

    // override description for include elements
    tinyxml2::XMLElement *description = child->FirstChildElement("description");
    if (description)
    {
      element->SetDescription(description->GetText());
    }

    _sdf->AddElementDescription(element);
  }

  return true;
}

//////////////////////////////////////////////////
SDFPtr readFile(const std::string &_filename, Errors &_errors)
{
  // Create and initialize the data structure that will hold the parsed SDF data
  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);

  // Read an SDF file, and store the result in sdfParsed.
  if (!sdf::readFile(_filename, sdfParsed, _errors))
  {
    return SDFPtr();
  }

  return sdfParsed;
}

//////////////////////////////////////////////////
SDFPtr readFile(const std::string &_filename)
{
  Errors errors;
  SDFPtr result = readFile(_filename, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr _sdf)
{
  Errors errors;
  bool result = readFile(_filename, _sdf, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr _sdf, Errors &_errors)
{
  return readFileInternal(_filename, _sdf, true, _errors);
}

//////////////////////////////////////////////////
bool readFileWithoutConversion(
    const std::string &_filename, SDFPtr _sdf, Errors &_errors)
{
  return readFileInternal(_filename, _sdf, false, _errors);
}

//////////////////////////////////////////////////
bool readFileInternal(const std::string &_filename, SDFPtr _sdf,
      const bool _convert, Errors &_errors)
{
  auto xmlDoc = makeSdfDoc();
  std::string filename = sdf::findFile(_filename, true, true);

  if (filename.empty())
  {
    sdferr << "Error finding file [" << _filename << "].\n";
    return false;
  }

  if (filesystem::is_directory(filename))
  {
    filename = getModelFilePath(filename);
  }

  if (!filesystem::exists(filename))
  {
    sdferr << "File [" << filename << "] doesn't exist.\n";
    return false;
  }

  auto error_code = xmlDoc.LoadFile(filename.c_str());
  if (error_code)
  {
    sdferr << "Error parsing XML in file [" << filename << "]: "
           << xmlDoc.ErrorStr() << '\n';
    return false;
  }

  // Suppress deprecation for sdf::URDF2SDF
  if (readDoc(&xmlDoc, _sdf, filename, _convert, _errors))
  {
    return true;
  }
  else if (URDF2SDF::IsURDF(filename))
  {
    URDF2SDF u2g;
    auto doc = makeSdfDoc();
    u2g.InitModelFile(filename, &doc);
    if (sdf::readDoc(&doc, _sdf, "urdf file", _convert, _errors))
    {
      sdfdbg << "parse from urdf file [" << _filename << "].\n";
      return true;
    }
    else
    {
      sdferr << "parse as old deprecated model file failed.\n";
      return false;
    }
  }

  return false;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, SDFPtr _sdf)
{
  Errors errors;
  bool result = readString(_xmlString, _sdf, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, SDFPtr _sdf, Errors &_errors)
{
  return readStringInternal(_xmlString, _sdf, true, _errors);
}

//////////////////////////////////////////////////
bool readStringWithoutConversion(
    const std::string &_filename, SDFPtr _sdf, Errors &_errors)
{
  return readStringInternal(_filename, _sdf, false, _errors);
}

//////////////////////////////////////////////////
bool readStringInternal(const std::string &_xmlString, SDFPtr _sdf,
    const bool _convert, Errors &_errors)
{
  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(_xmlString.c_str());
  if (xmlDoc.Error())
  {
    sdferr << "Error parsing XML from string: " << xmlDoc.ErrorStr() << '\n';
    return false;
  }
  if (readDoc(&xmlDoc, _sdf, "data-string", _convert, _errors))
  {
    return true;
  }
  else
  {
    URDF2SDF u2g;
    auto doc = makeSdfDoc();
    u2g.InitModelString(_xmlString, &doc);

    if (sdf::readDoc(&doc, _sdf, "urdf string", _convert, _errors))
    {
      sdfdbg << "Parsing from urdf.\n";
      return true;
    }
    else
    {
      sdferr << "parse as old deprecated model file failed.\n";
      return false;
    }
  }

  return false;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, ElementPtr _sdf)
{
  Errors errors;
  bool result = readString(_xmlString, _sdf, errors);

  // Output errors
  for (auto const &e : errors)
    std::cerr << e << std::endl;

  return result;
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, ElementPtr _sdf, Errors &_errors)
{
  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(_xmlString.c_str());
  if (xmlDoc.Error())
  {
    sdferr << "Error parsing XML from string: " << xmlDoc.ErrorStr() << '\n';
    return false;
  }
  if (readDoc(&xmlDoc, _sdf, "data-string", true, _errors))
  {
    return true;
  }
  else
  {
    sdferr << "parse as sdf version " << SDF::Version() << " failed, "
           << "should try to parse as old deprecated format\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool readDoc(tinyxml2::XMLDocument *_xmlDoc, SDFPtr _sdf,
    const std::string &_source, bool _convert, Errors &_errors)
{
  if (!_xmlDoc)
  {
    sdfwarn << "Could not parse the xml from source[" << _source << "]\n";
    return false;
  }

  // check sdf version
  tinyxml2::XMLElement *sdfNode = _xmlDoc->FirstChildElement("sdf");
  if (!sdfNode)
  {
    return false;
  }

  if (nullptr == _sdf || nullptr == _sdf->Root())
  {
    sdferr << "SDF pointer or its Root is null.\n";
    return false;
  }

  if (_source != "data-string")
  {
    _sdf->SetFilePath(_source);
  }

  if (sdfNode && sdfNode->Attribute("version"))
  {
    if (_sdf->OriginalVersion().empty())
    {
      _sdf->SetOriginalVersion(sdfNode->Attribute("version"));
    }

    if (_sdf->Root()->OriginalVersion().empty())
    {
      _sdf->Root()->SetOriginalVersion(sdfNode->Attribute("version"));
    }

    if (_convert
        && strcmp(sdfNode->Attribute("version"), SDF::Version().c_str()) != 0)
    {
      sdfdbg << "Converting a deprecated source[" << _source << "].\n";
      Converter::Convert(_xmlDoc, SDF::Version());
    }

    // parse new sdf xml
    auto *elemXml = _xmlDoc->FirstChildElement(_sdf->Root()->GetName().c_str());
    if (!readXml(elemXml, _sdf->Root(), _errors))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Error reading element <" + _sdf->Root()->GetName() + ">"});
      return false;
    }
  }
  else
  {
    if (!sdfNode)
    {
      sdfdbg << "No <sdf> element in file[" << _source << "]\n";
    }
    else if (!sdfNode->Attribute("version"))
    {
      sdfdbg << "SDF <sdf> element has no version in file["
             << _source << "]\n";
    }
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool readDoc(tinyxml2::XMLDocument *_xmlDoc, ElementPtr _sdf,
             const std::string &_source, bool _convert, Errors &_errors)
{
  if (!_xmlDoc)
  {
    sdfwarn << "Could not parse the xml\n";
    return false;
  }

  // check sdf version
  tinyxml2::XMLElement *sdfNode = _xmlDoc->FirstChildElement("sdf");
  if (!sdfNode)
  {
    return false;
  }

  if (_source != "data-string")
  {
    _sdf->SetFilePath(_source);
  }

  if (sdfNode && sdfNode->Attribute("version"))
  {
    if (_sdf->OriginalVersion().empty())
    {
      _sdf->SetOriginalVersion(sdfNode->Attribute("version"));
    }

    if (_convert
        && strcmp(sdfNode->Attribute("version"), SDF::Version().c_str()) != 0)
    {
      sdfwarn << "Converting a deprecated SDF source[" << _source << "].\n";
      Converter::Convert(_xmlDoc, SDF::Version());
    }

    tinyxml2::XMLElement *elemXml = sdfNode;
    if (sdfNode->Value() != _sdf->GetName() &&
        sdfNode->FirstChildElement(_sdf->GetName().c_str()))
    {
      elemXml = sdfNode->FirstChildElement(_sdf->GetName().c_str());
    }

    // parse new sdf xml
    if (!readXml(elemXml, _sdf, _errors))
    {
      _errors.push_back({ErrorCode::ELEMENT_INVALID,
          "Unable to parse sdf element["+ _sdf->GetName() + "]"});
      return false;
    }
  }
  else
  {
    if (!sdfNode)
    {
      sdfdbg << "SDF has no <sdf> element\n";
    }
    else if (!sdfNode->Attribute("version"))
    {
      sdfdbg << "<sdf> element has no version\n";
    }
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
std::string getBestSupportedModelVersion(tinyxml2::XMLElement *_modelXML,
                                         std::string &_modelFileName)
{
  tinyxml2::XMLElement *sdfXML = _modelXML->FirstChildElement("sdf");
  tinyxml2::XMLElement *nameSearch = _modelXML->FirstChildElement("name");

  // If a match is not found, use the latest version of the element
  // that is not older than the SDF parser.
  ignition::math::SemanticVersion sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";

  tinyxml2::XMLElement *sdfSearch = sdfXML;
  while (sdfSearch)
  {
    if (sdfSearch->Attribute("version"))
    {
      auto version = std::string(sdfSearch->Attribute("version"));
      ignition::math::SemanticVersion modelVersion(version);
      ignition::math::SemanticVersion bestVersion(bestVersionStr);
      if (modelVersion > bestVersion)
      {
        // this model is better than the previous one
        if (modelVersion <= sdfParserVersion)
        {
          // the parser can read it
          sdfXML  = sdfSearch;
          bestVersionStr = version;
        }
        else
        {
          sdfwarn << "Ignoring version " << version
                  << " for model " << nameSearch->GetText()
                  << " because is newer than this sdf parser"
                  << " (version " << SDF_VERSION << ")\n";
        }
      }
    }
    sdfSearch = sdfSearch->NextSiblingElement("sdf");
  }

  if (!sdfXML || !sdfXML->GetText())
  {
    sdferr << "Failure to detect an sdf tag in the model config file"
           << " for model: " << nameSearch->GetText() << "\n";

    _modelFileName = "";
    return "";
  }

  if (!sdfXML->Attribute("version"))
  {
    sdfwarn << "Can not find the XML attribute 'version'"
            << " in sdf XML tag for model: " << nameSearch->GetText() << "."
            << " Please specify the SDF protocol supported in the model"
            << " configuration file. The first sdf tag in the config file"
            << " will be used \n";
  }

  _modelFileName = sdfXML->GetText();
  return bestVersionStr;
}

//////////////////////////////////////////////////
std::string getModelFilePath(const std::string &_modelDirPath)
{
  std::string configFilePath;

  /// \todo This hardcoded bit is very Gazebo centric. It should
  /// be abstracted away, possibly through a plugin to SDF.
  configFilePath = sdf::filesystem::append(_modelDirPath, "model.config");
  if (!sdf::filesystem::exists(configFilePath))
  {
    // We didn't find model.config, look for manifest.xml instead
    configFilePath = sdf::filesystem::append(_modelDirPath, "manifest.xml");
    if (!sdf::filesystem::exists(configFilePath))
    {
      // We didn't find manifest.xml either, output an error and get out.
      sdferr << "Could not find model.config or manifest.xml for the model\n";
      return std::string();
    }
    else
    {
      // We found manifest.xml, but since it is deprecated print a warning.
      sdfwarn << "The manifest.xml for a model is deprecated. "
              << "Please rename manifest.xml to "
              << "model.config" << ".\n";
    }
  }

  auto configFileDoc = makeSdfDoc();
  if (tinyxml2::XML_SUCCESS != configFileDoc.LoadFile(configFilePath.c_str()))
  {
    sdferr << "Error parsing XML in file ["
           << configFilePath << "]: "
           << configFileDoc.ErrorStr() << '\n';
    return std::string();
  }

  tinyxml2::XMLElement *modelXML = configFileDoc.FirstChildElement("model");

  if (!modelXML)
  {
    sdferr << "No <model> element in configFile[" << configFilePath << "]\n";
    return std::string();
  }

  std::string modelFileName;
  if (getBestSupportedModelVersion(modelXML, modelFileName).empty())
  {
    return std::string();
  }

  return sdf::filesystem::append(_modelDirPath, modelFileName);
}

//////////////////////////////////////////////////
bool readXml(tinyxml2::XMLElement *_xml, ElementPtr _sdf, Errors &_errors)
{
  // Check if the element pointer is deprecated.
  if (_sdf->GetRequired() == "-1")
  {
    _errors.push_back({ErrorCode::ELEMENT_DEPRECATED,
        "SDF Element[" + _sdf->GetName() + "] is deprecated"});
    return true;
  }

  if (!_xml)
  {
    if (_sdf->GetRequired() == "1" || _sdf->GetRequired() =="+")
    {
      _errors.push_back({ErrorCode::ELEMENT_MISSING,
          "SDF Element<" + _sdf->GetName() + "> is missing"});
      return false;
    }
    else
    {
      return true;
    }
  }

  if (_xml->GetText() != nullptr && _sdf->GetValue())
  {
    if (!_sdf->GetValue()->SetFromString(_xml->GetText()))
      return false;
  }

  // check for nested sdf
  std::string refSDFStr = _sdf->ReferenceSDF();
  if (!refSDFStr.empty())
  {
    ElementPtr refSDF;
    refSDF.reset(new Element);
    std::string refFilename = refSDFStr + ".sdf";
    initFile(refFilename, refSDF);
    _sdf->RemoveFromParent();
    _sdf->Copy(refSDF);
  }

  const tinyxml2::XMLAttribute *attribute = _xml->FirstAttribute();

  unsigned int i = 0;

  // Iterate over all the attributes defined in the give XML element
  while (attribute)
  {
    // Avoid printing a warning message for missing attributes if a namespaced
    // attribute is found
    if (std::strchr(attribute->Name(), ':') != NULL)
    {
      _sdf->AddAttribute(attribute->Name(), "string", "", 1, "");
      _sdf->GetAttribute(attribute->Name())->SetFromString(
          attribute->Value());
      attribute = attribute->Next();
      continue;
    }
    // Find the matching attribute in SDF
    for (i = 0; i < _sdf->GetAttributeCount(); ++i)
    {
      ParamPtr p = _sdf->GetAttribute(i);
      if (p->GetKey() == attribute->Name())
      {
        // Set the value of the SDF attribute
        if (!p->SetFromString(attribute->Value()))
        {
          _errors.push_back({ErrorCode::ATTRIBUTE_INVALID,
              "Unable to read attribute[" + p->GetKey() + "]"});
          return false;
        }
        break;
      }
    }

    if (i == _sdf->GetAttributeCount())
    {
      sdfwarn << "XML Attribute[" << attribute->Name()
              << "] in element[" << _xml->Value()
              << "] not defined in SDF, ignoring.\n";
    }

    attribute = attribute->Next();
  }

  // Check that all required attributes have been set
  for (i = 0; i < _sdf->GetAttributeCount(); ++i)
  {
    ParamPtr p = _sdf->GetAttribute(i);
    if (p->GetRequired() && !p->GetSet())
    {
      _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
          "Required attribute[" + p->GetKey() + "] in element[" + _xml->Value()
          + "] is not specified in SDF."});
      return false;
    }
  }

  if (_sdf->GetCopyChildren())
  {
    copyChildren(_sdf, _xml, false);
  }
  else
  {
    std::string filename;

    // Iterate over all the child elements
    tinyxml2::XMLElement *elemXml = nullptr;
    for (elemXml = _xml->FirstChildElement(); elemXml;
         elemXml = elemXml->NextSiblingElement())
    {
      if (std::string("include") == elemXml->Value())
      {
        std::string modelPath;

        if (elemXml->FirstChildElement("uri"))
        {
          std::string uri = elemXml->FirstChildElement("uri")->GetText();
          modelPath = sdf::findFile(uri, true, true);

          // Test the model path
          if (modelPath.empty())
          {
            _errors.push_back({ErrorCode::URI_LOOKUP,
                "Unable to find uri[" + uri + "]"});

            size_t modelFound = uri.find("model://");
            if (modelFound != 0u)
            {
              _errors.push_back({ErrorCode::URI_INVALID,
                  "Invalid uri[" + uri + "]. Should be model://" + uri});
            }
            continue;
          }
          else
          {
            if (!sdf::filesystem::is_directory(modelPath))
            {
              _errors.push_back({ErrorCode::DIRECTORY_NONEXISTANT,
                  "Directory doesn't exist[" + modelPath + "]"});
              continue;
            }
          }

          // Get the config.xml filename
          filename = getModelFilePath(modelPath);
        }
        else
        {
          _errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
              "<include> element missing 'uri' attribute"});
          continue;
        }

        // NOTE: sdf::init is an expensive call. For performance reason,
        // a new sdf pointer is created here by cloning a fresh sdf template
        // pointer instead of calling init every iteration.
        // SDFPtr includeSDF(new SDF);
        // init(includeSDF);
        static SDFPtr includeSDFTemplate;
        if (!includeSDFTemplate)
        {
          includeSDFTemplate.reset(new SDF);
          init(includeSDFTemplate);
        }
        SDFPtr includeSDF(new SDF);
        includeSDF->Root(includeSDFTemplate->Root()->Clone());

        if (!readFile(filename, includeSDF))
        {
          _errors.push_back({ErrorCode::FILE_READ,
              "Unable to read file[" + filename + "]"});
          return false;
        }

        sdf::ElementPtr topLevelElem;
        bool isModel{false};
        bool isActor{false};
        if (includeSDF->Root()->HasElement("model"))
        {
          topLevelElem = includeSDF->Root()->GetElement("model");
          isModel = true;
        }
        else if (includeSDF->Root()->HasElement("actor"))
        {
          topLevelElem = includeSDF->Root()->GetElement("actor");
          isActor = true;
        }
        else if (includeSDF->Root()->HasElement("light"))
        {
          topLevelElem = includeSDF->Root()->GetElement("light");
        }
        else
        {
          _errors.push_back({ErrorCode::ELEMENT_MISSING,
              "Failed to find top level <model> / <actor> / <light> for "
              "<include>\n"});
          continue;
        }

        if (elemXml->FirstChildElement("name"))
        {
          topLevelElem->GetAttribute("name")->SetFromString(
                elemXml->FirstChildElement("name")->GetText());
        }

        tinyxml2::XMLElement *poseElemXml = elemXml->FirstChildElement("pose");
        if (poseElemXml)
        {
          sdf::ElementPtr poseElem = topLevelElem->GetElement("pose");

          if (poseElemXml->GetText())
          {
            poseElem->GetValue()->SetFromString(poseElemXml->GetText());
          }
          else
          {
            poseElem->GetValue()->Reset();
          }

          const char *relativeTo = poseElemXml->Attribute("relative_to");
          if (relativeTo)
          {
            poseElem->GetAttribute("relative_to")->SetFromString(relativeTo);
          }
          else
          {
            poseElem->GetAttribute("relative_to")->Reset();
          }
        }

        if (isModel && elemXml->FirstChildElement("static"))
        {
          topLevelElem->GetElement("static")->GetValue()->SetFromString(
                elemXml->FirstChildElement("static")->GetText());
        }

        if (isModel || isActor)
        {
          for (auto *childElemXml = elemXml->FirstChildElement();
               childElemXml; childElemXml = childElemXml->NextSiblingElement())
          {
            if (std::string("plugin") == childElemXml->Value())
            {
              sdf::ElementPtr pluginElem;
              pluginElem = topLevelElem->AddElement("plugin");

              if (!readXml(childElemXml, pluginElem, _errors))
              {
                _errors.push_back({ErrorCode::ELEMENT_INVALID,
                                   "Error reading plugin element"});
                return false;
              }
            }
          }
        }

        if (_sdf->GetName() == "model")
        {
          addNestedModel(_sdf, includeSDF->Root(), _errors);
        }
        else
        {
          includeSDF->Root()->GetFirstElement()->SetParent(_sdf);
          _sdf->InsertElement(includeSDF->Root()->GetFirstElement());
          // TODO: This was used to store the included filename so that when
          // a world is saved, the included model's SDF is not stored in the
          // world file. This highlights the need to make model inclusion
          // a core feature of SDF, and not a hack that that parser handles
          // includeSDF->Root()->GetFirstElement()->SetInclude(
          // elemXml->Attribute("filename"));
        }

        continue;
      }

      // Find the matching element in SDF
      unsigned int descCounter = 0;
      for (descCounter = 0;
           descCounter != _sdf->GetElementDescriptionCount(); ++descCounter)
      {
        ElementPtr elemDesc = _sdf->GetElementDescription(descCounter);
        if (elemDesc->GetName() == elemXml->Value())
        {
          ElementPtr element = elemDesc->Clone();
          element->SetParent(_sdf);
          if (readXml(elemXml, element, _errors))
          {
            _sdf->InsertElement(element);
          }
          else
          {
            _errors.push_back({ErrorCode::ELEMENT_INVALID,
                std::string("Error reading element <") +
                elemXml->Value() + ">"});
            return false;
          }
          break;
        }
      }

      if (descCounter == _sdf->GetElementDescriptionCount())
      {
        sdfdbg << "XML Element[" << elemXml->Value()
               << "], child of element[" << _xml->Value()
               << "], not defined in SDF. Copying[" << elemXml->Value() << "] "
               << "as children of [" << _xml->Value() << "].\n";
        continue;
      }
    }

    // Copy unknown elements outside the loop so it only happens one time
    copyChildren(_sdf, _xml, true);

    // Check that all required elements have been set
    for (unsigned int descCounter = 0;
         descCounter != _sdf->GetElementDescriptionCount(); ++descCounter)
    {
      ElementPtr elemDesc = _sdf->GetElementDescription(descCounter);

      if (elemDesc->GetRequired() == "1" || elemDesc->GetRequired() == "+")
      {
        if (!_sdf->HasElement(elemDesc->GetName()))
        {
          if (_sdf->GetName() == "joint" &&
              _sdf->Get<std::string>("type") != "ball")
          {
            _errors.push_back({ErrorCode::ELEMENT_MISSING,
                "XML Missing required element[" + elemDesc->GetName() +
                "], child of element[" + _sdf->GetName() + "]"});
            return false;
          }
          else
          {
            // Add default element
            _sdf->AddElement(elemDesc->GetName());
          }
        }
      }
    }
  }

  return true;
}

/////////////////////////////////////////////////
static void replace_all(std::string &_str,
                        const std::string &_from,
                        const std::string &_to)
{
  if (_from.empty())
  {
    return;
  }
  size_t start_pos = 0;
  while ((start_pos = _str.find(_from, start_pos)) != std::string::npos)
  {
    _str.replace(start_pos, _from.length(), _to);
    // We need to advance our starting position beyond what we
    // just replaced to deal with the case where the '_to' string
    // happens to contain a piece of '_from'.
    start_pos += _to.length();
  }
}

/////////////////////////////////////////////////
void copyChildren(ElementPtr _sdf,
                  tinyxml2::XMLElement *_xml,
                  const bool _onlyUnknown)
{
  // Iterate over all the child elements
  tinyxml2::XMLElement *elemXml = nullptr;
  for (elemXml = _xml->FirstChildElement(); elemXml;
       elemXml = elemXml->NextSiblingElement())
  {
    std::string elem_name = elemXml->Name();

    if (_sdf->HasElementDescription(elem_name))
    {
      if (!_onlyUnknown)
      {
        sdf::ElementPtr element = _sdf->AddElement(elem_name);

        // FIXME: copy attributes
        for (const auto *attribute = elemXml->FirstAttribute();
             attribute; attribute = attribute->Next())
        {
          element->GetAttribute(attribute->Name())->SetFromString(
            attribute->Value());
        }

        // copy value
        std::string value = elemXml->GetText();
        if (!value.empty())
        {
          element->GetValue()->SetFromString(value);
        }
        copyChildren(element, elemXml, _onlyUnknown);
      }
    }
    else
    {
      ElementPtr element(new Element);
      element->SetParent(_sdf);
      element->SetName(elem_name);
      if (elemXml->GetText() != nullptr)
      {
        element->AddValue("string", elemXml->GetText(), "1");
      }

      for (const tinyxml2::XMLAttribute *attribute = elemXml->FirstAttribute();
           attribute; attribute = attribute->Next())
      {
        element->AddAttribute(attribute->Name(), "string", "", 1, "");
        element->GetAttribute(attribute->Name())->SetFromString(
          attribute->Value());
      }

      copyChildren(element, elemXml, _onlyUnknown);
      _sdf->InsertElement(element);
    }
  }
}

/////////////////////////////////////////////////
void addNestedModel(ElementPtr _sdf, ElementPtr _includeSDF)
{
  Errors errors;
  addNestedModel(_sdf, _includeSDF, errors);
  for (const auto &e : errors)
  {
    sdferr << e << '\n';
  }
}

/////////////////////////////////////////////////
void addNestedModel(ElementPtr _sdf, ElementPtr _includeSDF, Errors &_errors)
{
  ElementPtr modelPtr = _includeSDF->GetElement("model");
  ElementPtr elem = modelPtr->GetFirstElement();
  std::map<std::string, std::string> replace;

  ignition::math::Pose3d modelPose =
    modelPtr->Get<ignition::math::Pose3d>("pose");

  std::string modelName = modelPtr->Get<std::string>("name");

  // Inject a frame to represent the nested __model__ frame.
  ElementPtr nestedModelFrame = _sdf->AddElement("frame");
  const std::string nestedModelFrameName = modelName + "::__model__";
  nestedModelFrame->GetAttribute("name")->Set(nestedModelFrameName);

  replace["__model__"] = nestedModelFrameName;

  std::string canonicalLinkName = "";
  if (modelPtr->GetAttribute("canonical_link")->GetSet())
  {
    canonicalLinkName = modelPtr->GetAttribute("canonical_link")->GetAsString();
  }
  else if (modelPtr->HasElement("link"))
  {
    canonicalLinkName =
      modelPtr->GetElement("link")->GetAttribute("name")->GetAsString();
  }
  nestedModelFrame->GetAttribute("attached_to")
      ->Set(modelName + "::" + canonicalLinkName);

  ElementPtr nestedModelFramePose = nestedModelFrame->AddElement("pose");
  nestedModelFramePose->Set(modelPose);

  // Set the nestedModelFrame's //pose/@relative_to to the frame used in
  // //include/pose/@relative_to.
  std::string modelPoseRelativeTo = "";
  if (modelPtr->HasElement("pose"))
  {
    modelPoseRelativeTo =
        modelPtr->GetElement("pose")->Get<std::string>("relative_to");
  }

  // If empty, use "__model__", since leaving it empty would make it
  // relative_to the canonical link frame specified in //frame/@attached_to.
  if (modelPoseRelativeTo.empty())
  {
    modelPoseRelativeTo = "__model__";
  }

  nestedModelFramePose->GetAttribute("relative_to")->Set(modelPoseRelativeTo);

  while (elem)
  {
    if ((elem->GetName() == "link") ||
        (elem->GetName() == "joint") ||
        (elem->GetName() == "frame"))
    {
      std::string elemName = elem->Get<std::string>("name");
      std::string newName =  modelName + "::" + elemName;
      replace[elemName] = newName;
    }

    if ((elem->GetName() == "link"))
    {
      // Add a pose element even if the element doesn't originally have one
      auto elemPose = elem->GetElement("pose");

      // If //pose/@relative_to is empty, explicitly set it to the name
      // of the nested model frame.
      auto relativeTo = elemPose->GetAttribute("relative_to");
      if (relativeTo->GetAsString().empty())
      {
        relativeTo->Set(nestedModelFrameName);
      }

      // If //pose/@relative_to is set, let the replacement step handle it.
    }
    else if (elem->GetName() == "frame")
    {
      // If //frame/@attached_to is empty, explicitly set it to the name
      // of the nested model frame.
      auto attachedTo = elem->GetAttribute("attached_to");
      if (attachedTo->GetAsString().empty())
      {
        attachedTo->Set(nestedModelFrameName);
      }

      // If //frame/@attached_to is set, let the replacement step handle it.
    }
    elem = elem->GetNextElement();
  }

  std::string str = _includeSDF->ToString("");
  for (std::map<std::string, std::string>::iterator iter = replace.begin();
       iter != replace.end(); ++iter)
  {
    replace_all(str, std::string("\"") + iter->first + "\"",
                std::string("\"") + iter->second + "\"");
    replace_all(str, std::string("'") + iter->first + "'",
                std::string("'") + iter->second + "'");
    replace_all(str, std::string(">") + iter->first + "<",
                std::string(">") + iter->second + "<");
  }

  _includeSDF->ClearElements();
  readString(str, _includeSDF, _errors);

  elem = _includeSDF->GetElement("model")->GetFirstElement();
  ElementPtr nextElem;
  while (elem)
  {
    nextElem = elem->GetNextElement();

    if (elem->GetName() != "pose")
    {
      elem->SetParent(_sdf);
      _sdf->InsertElement(elem);
    }
    elem = nextElem;
  }
}

/////////////////////////////////////////////////
bool convertFile(const std::string &_filename, const std::string &_version,
                 SDFPtr _sdf)
{
  std::string filename = sdf::findFile(_filename);

  if (filename.empty())
  {
    sdferr << "Error finding file [" << _filename << "].\n";
    return false;
  }

  if (nullptr == _sdf || nullptr == _sdf->Root())
  {
    sdferr << "SDF pointer or its Root is null.\n";
    return false;
  }

  auto xmlDoc = makeSdfDoc();
  if (!xmlDoc.LoadFile(filename.c_str()))
  {
    // read initial sdf version
    std::string originalVersion;
    {
      tinyxml2::XMLElement *sdfNode = xmlDoc.FirstChildElement("sdf");
      if (sdfNode && sdfNode->Attribute("version"))
      {
        originalVersion = sdfNode->Attribute("version");
      }
    }

    _sdf->SetOriginalVersion(originalVersion);

    if (sdf::Converter::Convert(&xmlDoc, _version, true))
    {
      Errors errors;
      bool result = sdf::readDoc(&xmlDoc, _sdf, filename, false, errors);

      // Output errors
      for (auto const &e : errors)
        std::cerr << e << std::endl;

      return result;
    }
  }
  else
  {
    sdferr << "Error parsing file[" << filename << "]\n";
  }

  return false;
}

/////////////////////////////////////////////////
bool convertString(const std::string &_sdfString, const std::string &_version,
                   SDFPtr _sdf)
{
  if (_sdfString.empty())
  {
    sdferr << "SDF string is empty.\n";
    return false;
  }

  auto xmlDoc = makeSdfDoc();
  xmlDoc.Parse(_sdfString.c_str());

  if (!xmlDoc.Error())
  {
    // read initial sdf version
    std::string originalVersion;
    {
      tinyxml2::XMLElement *sdfNode = xmlDoc.FirstChildElement("sdf");
      if (sdfNode && sdfNode->Attribute("version"))
      {
        originalVersion = sdfNode->Attribute("version");
      }
    }

    _sdf->SetOriginalVersion(originalVersion);

    if (sdf::Converter::Convert(&xmlDoc, _version, true))
    {
      Errors errors;
      bool result = sdf::readDoc(&xmlDoc, _sdf, "data-string", false, errors);

      // Output errors
      for (auto const &e : errors)
        std::cerr << e << std::endl;

      return result;
    }
  }
  else
  {
    sdferr << "Error parsing XML from string[" << _sdfString << "]\n";
  }

  return false;
}

//////////////////////////////////////////////////
bool checkCanonicalLinkNames(const sdf::Root *_root)
{
  if (!_root)
  {
    std::cerr << "Error: invalid sdf::Root pointer, unable to "
              << "check canonical link names."
              << std::endl;
    return false;
  }

  bool result = true;

  auto checkModelCanonicalLinkName = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    std::string canonicalLink = _model->CanonicalLinkName();
    if (!canonicalLink.empty() && !_model->LinkNameExists(canonicalLink))
    {
      std::cerr << "Error: canonical_link with name[" << canonicalLink
                << "] not found in model with name[" << _model->Name()
                << "]."
                << std::endl;
      modelResult = false;
    }
    return modelResult;
  };

  for (uint64_t m = 0; m < _root->ModelCount(); ++m)
  {
    auto model = _root->ModelByIndex(m);
    result = checkModelCanonicalLinkName(model) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelCanonicalLinkName(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool checkFrameAttachedToNames(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelFrameAttachedToNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t f = 0; f < _model->FrameCount(); ++f)
    {
      auto frame = _model->FrameByIndex(f);

      const std::string &attachedTo = frame->AttachedTo();

      // the attached_to attribute is always permitted to be empty
      if (attachedTo.empty())
      {
        continue;
      }

      if (attachedTo == frame->Name())
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] is identical to frame name[" << frame->Name()
                  << "], causing a graph cycle "
                  << "in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }
      else if (!_model->LinkNameExists(attachedTo) &&
               !_model->ModelNameExists(attachedTo) &&
               !_model->JointNameExists(attachedTo) &&
               !_model->FrameNameExists(attachedTo))
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] specified by frame with name[" << frame->Name()
                  << "] does not match a nested model, link, joint, "
                  << "or frame name in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }
    }
    return modelResult;
  };

  auto checkWorldFrameAttachedToNames = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    for (uint64_t f = 0; f < _world->FrameCount(); ++f)
    {
      auto frame = _world->FrameByIndex(f);

      const std::string &attachedTo = frame->AttachedTo();

      // the attached_to attribute is always permitted to be empty
      if (attachedTo.empty())
      {
        continue;
      }

      if (attachedTo == frame->Name())
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] is identical to frame name[" << frame->Name()
                  << "], causing a graph cycle "
                  << "in world with name[" << _world->Name()
                  << "]."
                  << std::endl;
        worldResult = false;
      }
      else if (!_world->ModelNameExists(attachedTo) &&
               !_world->FrameNameExists(attachedTo))
      {
        std::cerr << "Error: attached_to name[" << attachedTo
                  << "] specified by frame with name[" << frame->Name()
                  << "] does not match a model or frame name "
                  << "in world with name[" << _world->Name()
                  << "]."
                  << std::endl;
        worldResult = false;
      }
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root->ModelCount(); ++m)
  {
    auto model = _root->ModelByIndex(m);
    result = checkModelFrameAttachedToNames(model) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    result = checkWorldFrameAttachedToNames(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelFrameAttachedToNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool recursiveSameTypeUniqueNames(sdf::ElementPtr _elem)
{
  if (!shouldValidateElement(_elem))
    return true;

  bool result = true;
  auto typeNames = _elem->GetElementTypeNames();
  for (const std::string &typeName : typeNames)
  {
    if (!_elem->HasUniqueChildNames(typeName))
    {
      std::cerr << "Error: Non-unique names detected in type "
                << typeName << " in\n"
                << _elem->ToString("")
                << std::endl;
      result = false;
    }
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSameTypeUniqueNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
bool recursiveSiblingUniqueNames(sdf::ElementPtr _elem)
{
  if (!shouldValidateElement(_elem))
    return true;

  bool result = _elem->HasUniqueChildNames();
  if (!result)
  {
    std::cerr << "Error: Non-unique names detected in "
              << _elem->ToString("")
              << std::endl;
    result = false;
  }

  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    result = recursiveSiblingUniqueNames(child) && result;
    child = child->GetNextElement();
  }

  return result;
}

//////////////////////////////////////////////////
bool checkFrameAttachedToGraph(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelFrameAttachedToGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    sdf::FrameAttachedToGraph graph;
    auto errors = sdf::buildFrameAttachedToGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    errors = sdf::validateFrameAttachedToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validateFrameAttachedToGraph: "
                  << error.Message()
                  << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  auto checkWorldFrameAttachedToGraph = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    sdf::FrameAttachedToGraph graph;
    auto errors = sdf::buildFrameAttachedToGraph(graph, _world);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      worldResult = false;
    }
    errors = sdf::validateFrameAttachedToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validateFrameAttachedToGraph: "
                  << error.Message()
                  << std::endl;
      }
      worldResult = false;
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root->ModelCount(); ++m)
  {
    auto model = _root->ModelByIndex(m);
    result = checkModelFrameAttachedToGraph(model) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    result = checkWorldFrameAttachedToGraph(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelFrameAttachedToGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool checkPoseRelativeToGraph(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelPoseRelativeToGraph = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    sdf::PoseRelativeToGraph graph;
    auto errors = sdf::buildPoseRelativeToGraph(graph, _model);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      modelResult = false;
    }
    errors = sdf::validatePoseRelativeToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validatePoseRelativeToGraph: "
                  << error.Message()
                  << std::endl;
      }
      modelResult = false;
    }
    return modelResult;
  };

  auto checkWorldPoseRelativeToGraph = [](
      const sdf::World *_world) -> bool
  {
    bool worldResult = true;
    sdf::PoseRelativeToGraph graph;
    auto errors = sdf::buildPoseRelativeToGraph(graph, _world);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error: " << error.Message() << std::endl;
      }
      worldResult = false;
    }
    errors = sdf::validatePoseRelativeToGraph(graph);
    if (!errors.empty())
    {
      for (auto &error : errors)
      {
        std::cerr << "Error in validatePoseRelativeToGraph: "
                  << error.Message()
                  << std::endl;
      }
      worldResult = false;
    }
    return worldResult;
  };

  for (uint64_t m = 0; m < _root->ModelCount(); ++m)
  {
    auto model = _root->ModelByIndex(m);
    result = checkModelPoseRelativeToGraph(model) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    result = checkWorldPoseRelativeToGraph(world) && result;
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelPoseRelativeToGraph(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool checkJointParentChildLinkNames(const sdf::Root *_root)
{
  bool result = true;

  auto checkModelJointParentChildNames = [](
      const sdf::Model *_model) -> bool
  {
    bool modelResult = true;
    for (uint64_t j = 0; j < _model->JointCount(); ++j)
    {
      auto joint = _model->JointByIndex(j);

      const std::string &parentName = joint->ParentLinkName();
      if (parentName != "world" && !_model->LinkNameExists(parentName))
      {
        std::cerr << "Error: parent link with name[" << parentName
                  << "] specified by joint with name[" << joint->Name()
                  << "] not found in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      const std::string &childName = joint->ChildLinkName();
      if (childName != "world" && !_model->LinkNameExists(childName))
      {
        std::cerr << "Error: child link with name[" << childName
                  << "] specified by joint with name[" << joint->Name()
                  << "] not found in model with name[" << _model->Name()
                  << "]."
                  << std::endl;
        modelResult = false;
      }

      if (childName == parentName)
      {
        std::cerr << "Error: joint with name[" << joint->Name()
                  << "] in model with name[" << _model->Name()
                  << "] must specify different link names for "
                  << "parent and child, while [" << childName
                  << "] was specified for both."
                  << std::endl;
        modelResult = false;
      }
    }
    return modelResult;
  };

  for (uint64_t m = 0; m < _root->ModelCount(); ++m)
  {
    auto model = _root->ModelByIndex(m);
    result = checkModelJointParentChildNames(model) && result;
  }

  for (uint64_t w = 0; w < _root->WorldCount(); ++w)
  {
    auto world = _root->WorldByIndex(w);
    for (uint64_t m = 0; m < world->ModelCount(); ++m)
    {
      auto model = world->ModelByIndex(m);
      result = checkModelJointParentChildNames(model) && result;
    }
  }

  return result;
}

//////////////////////////////////////////////////
bool shouldValidateElement(sdf::ElementPtr _elem)
{
  if (_elem->GetName() == "plugin")
  {
    // Ignore <plugin> elements
    return false;
  }

  // Check if the element name has a colon. This is treated as a namespaced
  // element and should be ignored
  if (_elem->GetName().find(":") != std::string::npos)
  {
    return false;
  }

  return true;
}
}
}
