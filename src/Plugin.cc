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

#include "sdf/Plugin.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

class sdf::Plugin::Implementation
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
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Plugin::Plugin(const Plugin &_plugin)
  : dataPtr(ignition::utils::MakeImpl<Implementation>())
{
  // Copy
  *this = _plugin;
}

/////////////////////////////////////////////////
Plugin::Plugin(Plugin &&_plugin) noexcept = default;

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
  if (!loadName(_sdf, this->dataPtr->name))
  {
    errors.push_back({ErrorCode::ATTRIBUTE_MISSING,
                     "A plugin name is required, but the name is not set."});
  }

  // Read the filename
  std::pair<std::string, bool> filenamePair =
    _sdf->Get<std::string>("filename", this->dataPtr->filename);
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
    this->dataPtr->contents.push_back(innerElem->Clone());
  }

  return errors;
}

/////////////////////////////////////////////////
std::string Plugin::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Plugin::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
std::string Plugin::Filename() const
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

/////////////////////////////////////////////////
sdf::ElementPtr Plugin::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("plugin.sdf", elem);

  elem->GetAttribute("name")->Set(this->Name());
  elem->GetAttribute("filename")->Set(this->Filename());

  // Insert plugin content
  for (const sdf::ElementPtr content : this->dataPtr->contents)
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
  this->dataPtr->contents.push_back(_elem->Clone());
}

/////////////////////////////////////////////////
Plugin &Plugin::operator=(const Plugin &_plugin)
{
  this->dataPtr->name = _plugin.Name();
  this->dataPtr->filename = _plugin.Filename();
  if (_plugin.Element())
    this->dataPtr->sdf = _plugin.Element()->Clone();

  this->dataPtr->contents.clear();
  // Copy the contents of the plugin
  for (const sdf::ElementPtr content : _plugin.Contents())
  {
    this->dataPtr->contents.push_back(content->Clone());
  }

  return *this;
}

/////////////////////////////////////////////////
Plugin &Plugin::operator=(Plugin &&_plugin) noexcept = default;
