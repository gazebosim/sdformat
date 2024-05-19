/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "sdf/Types.hh"
#include "sdf/Plugin.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::PluginPrivate
{
  /// \brief Name of the plugin
  public: std::string name = "";

  /// \brief Filename of the shared library
  public: std::string filename = "";

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief SDF elements inside the plugin.
  public: std::vector<sdf::ElementPtr> contents;
};

/////////////////////////////////////////////////
Plugin::Plugin()
  : dataPtr(std::make_unique<sdf::PluginPrivate>())
{
}

/////////////////////////////////////////////////
Plugin::Plugin(const std::string &_filename, const std::string &_name,
               const std::string &_xmlContent)
  : dataPtr(std::make_unique<sdf::PluginPrivate>())
{
  sdf::Errors errors;
  this->Init(errors, _filename, _name, _xmlContent);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
Plugin::Plugin(sdf::Errors &_errors, const std::string &_filename,
               const std::string &_name, const std::string &_xmlContent)
  : dataPtr(std::make_unique<sdf::PluginPrivate>())
{
  this->Init(_errors, _filename, _name, _xmlContent);
}

void Plugin::Init(sdf::Errors &_errors, const std::string &_filename,
               const std::string &_name, const std::string &_xmlContent)
{
  this->SetFilename(_filename);
  this->SetName(_name);
  std::string trimmed = sdf::trim(_xmlContent);
  if (!trimmed.empty())
    this->InsertContent(_errors, trimmed);
}

/////////////////////////////////////////////////
Plugin::~Plugin() = default;

/////////////////////////////////////////////////
Plugin::Plugin(const Plugin &_plugin)
  : dataPtr(std::make_unique<sdf::PluginPrivate>())
{
  // Copy
  *this = _plugin;
}

/////////////////////////////////////////////////
Plugin::Plugin(Plugin &&_plugin) noexcept
{
  this->dataPtr = std::move(_plugin.dataPtr);
}

/////////////////////////////////////////////////
Errors Plugin::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that sdf is a valid pointer
  if (!_sdf)
  {
    errors.push_back({ErrorCode::ELEMENT_MISSING,
        "Attempting to load a plugin, but the provided SDF "
        "element is null."});
    return errors;
  }

  // We need a plugin element
  if (_sdf->GetName() != "plugin")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a plugin, but the provided SDF "
        "element is not a <plugin>."});
    return errors;
  }

  // Read the models's name
  loadName(_sdf, this->dataPtr->name);

  // Read the filename
  std::pair<std::string, bool> filenamePair =
    _sdf->Get<std::string>(errors, "filename", this->dataPtr->filename);
  this->dataPtr->filename = filenamePair.first;
  if (!filenamePair.second)
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
        "A plugin filename is required, but the filename is not set."});
  }

  // Copy the contents of the plugin
  for (sdf::ElementPtr innerElem = _sdf->GetFirstElement();
       innerElem; innerElem = innerElem->GetNextElement(""))
  {
    this->dataPtr->contents.push_back(innerElem->Clone(errors));
  }

  return errors;
}

/////////////////////////////////////////////////
const std::string &Plugin::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Plugin::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
const std::string &Plugin::Filename() const
{
  return this->dataPtr->filename;
}

/////////////////////////////////////////////////
void Plugin::SetFilename(const std::string &_filename)
{
  this->dataPtr->filename = _filename;
}

/////////////////////////////////////////////////
sdf::ElementPtr Plugin::Element() const
{
  return this->dataPtr->sdf;
}

sdf::ElementPtr Plugin::ToElement() const
{
  sdf::Errors errors;
  auto result = this->ToElement(errors);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
sdf::ElementPtr Plugin::ToElement(sdf::Errors &_errors) const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile(std::string(this->SchemaFile()), elem);

  elem->GetAttribute("name")->Set(this->Name(), _errors);
  elem->GetAttribute("filename")->Set(this->Filename(), _errors);

  // Insert plugin content
  for (const sdf::ElementPtr &content : this->dataPtr->contents)
    elem->InsertElement(content, true);

  return elem;
}

/////////////////////////////////////////////////
void Plugin::ClearContents()
{
  this->dataPtr->contents.clear();
}

/////////////////////////////////////////////////
const std::vector<sdf::ElementPtr> &Plugin::Contents() const
{
  return this->dataPtr->contents;
}

/////////////////////////////////////////////////
void Plugin::InsertContent(const sdf::ElementPtr _elem)
{
  sdf::Errors errors;
  this->InsertContent(errors, _elem);
  sdf::throwOrPrintErrors(errors);
}

/////////////////////////////////////////////////
void Plugin::InsertContent(sdf::Errors &_errors, const sdf::ElementPtr _elem)
{
  this->dataPtr->contents.push_back(_elem->Clone(_errors));
}

/////////////////////////////////////////////////
bool Plugin::InsertContent(const std::string _content)
{
  sdf::Errors errors;
  bool result = this->InsertContent(errors, _content);
  sdf::throwOrPrintErrors(errors);
  return result;
}

/////////////////////////////////////////////////
bool Plugin::InsertContent(sdf::Errors &_errors, const std::string _content)
{
  // Read the XML content
  auto xmlDoc = tinyxml2::XMLDocument(true, tinyxml2::COLLAPSE_WHITESPACE);
  xmlDoc.Parse(_content.c_str());
  if (xmlDoc.Error())
  {
    std::stringstream ss;
    ss << "Error parsing XML from string: " << xmlDoc.ErrorStr();
    _errors.push_back({ErrorCode::PARSING_ERROR, ss.str()});
    return false;
  }

  // Insert each XML element
  for (tinyxml2::XMLElement *xml = xmlDoc.FirstChildElement(); xml;
       xml = xml->NextSiblingElement())
  {
    sdf::ElementPtr element(new sdf::Element);

    // Copy the name
    element->SetName(xml->Name());

    // Copy attributes
    for (const tinyxml2::XMLAttribute *attribute = xml->FirstAttribute();
        attribute; attribute = attribute->Next())
    {
      element->AddAttribute(attribute->Name(), "string", "", 1, _errors, "");
      element->GetAttribute(attribute->Name())->SetFromString(
          attribute->Value(), _errors);
    }

    // Copy the value
    if (xml->GetText() != nullptr)
      element->AddValue("string", xml->GetText(), true, _errors);

    // Copy all children
    copyChildren(element, xml, false);

    // Add the element to this plugin
    this->InsertContent(_errors, element);
  }

  return true;
}

/////////////////////////////////////////////////
Plugin &Plugin::operator=(const Plugin &_plugin)
{
  if (!this->dataPtr)
    this->dataPtr = std::make_unique<sdf::PluginPrivate>();

  this->dataPtr->name = _plugin.Name();
  this->dataPtr->filename = _plugin.Filename();
  if (_plugin.Element())
    this->dataPtr->sdf = _plugin.Element()->Clone();

  this->dataPtr->contents.clear();
  // Copy the contents of the plugin
  for (const sdf::ElementPtr &content : _plugin.Contents())
  {
    this->dataPtr->contents.push_back(content->Clone());
  }

  return *this;
}

/////////////////////////////////////////////////
Plugin &Plugin::operator=(Plugin &&_plugin) noexcept
{
  this->dataPtr = std::move(_plugin.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
bool Plugin::operator==(const Plugin &_plugin) const
{
  // Simplest thing to do is compare the string form of each plugin
  return _plugin.ToElement()->ToString("") == this->ToElement()->ToString("");
}

/////////////////////////////////////////////////
bool Plugin::operator!=(const Plugin &_plugin) const
{
  return !(*this == _plugin);
}

/////////////////////////////////////////////////
inline std::string_view Plugin::SchemaFile() 
{
    static char kSchemaFile[] = "plugin.sdf";
    return kSchemaFile;
}

