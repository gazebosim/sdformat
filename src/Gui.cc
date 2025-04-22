/*
 * Copyright 2018 Open Source Robotics Foundation
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
#include <vector>
#include "sdf/Gui.hh"
#include "sdf/parser.hh"
#include "Utils.hh"

using namespace sdf;

/// \brief Gui private data.
class sdf::Gui::Implementation
{
  /// \brief True if the GUI should be fullscreen.
  public: bool fullscreen = false;

  /// \brief The SDF element pointer used during load.
  public: sdf::ElementPtr sdf;

  /// \brief GUI plugins.
  public: std::vector<Plugin> plugins;
};

/////////////////////////////////////////////////
Gui::Gui()
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Errors Gui::Load(ElementPtr _sdf)
{
  Errors errors;

  this->dataPtr->sdf = _sdf;

  // Check that the provided SDF element is a <gui> element.
  // This is an error that cannot be recovered, so return an error.
  if (_sdf->GetName() != "gui")
  {
    errors.push_back({ErrorCode::ELEMENT_INCORRECT_TYPE,
        "Attempting to load a Gui, but the provided SDF element is not a "
        "<gui>."});
    return errors;
  }

  // Get the full screen property
  this->dataPtr->fullscreen = _sdf->Get<bool>("fullscreen",
      this->dataPtr->fullscreen).first;

  Errors pluginErrors = loadRepeated<Plugin>(_sdf, "plugin",
    this->dataPtr->plugins);
  errors.insert(errors.end(), pluginErrors.begin(), pluginErrors.end());

  // \todo(nkoenig) Parse all the elements in gui.sdf

  return errors;
}

/////////////////////////////////////////////////
bool Gui::Fullscreen() const
{
  return this->dataPtr->fullscreen;
}

/////////////////////////////////////////////////
void Gui::SetFullscreen(const bool _fullscreen)
{
  this->dataPtr->fullscreen = _fullscreen;
}

/////////////////////////////////////////////////
bool Gui::operator==(const Gui &_gui) const
{
  return this->dataPtr->fullscreen == _gui.dataPtr->fullscreen;
}

/////////////////////////////////////////////////
sdf::ElementPtr Gui::Element() const
{
  return this->dataPtr->sdf;
}

/////////////////////////////////////////////////
sdf::ElementPtr Gui::ToElement() const
{
  sdf::ElementPtr elem(new sdf::Element);
  sdf::initFile("gui.sdf", elem);

  elem->GetAttribute("fullscreen")->Set(this->dataPtr->fullscreen);

  // Add in the plugins
  for (const Plugin &plugin : this->dataPtr->plugins)
    elem->InsertElement(plugin.ToElement(), true);

  return elem;
}

/////////////////////////////////////////////////
uint64_t Gui::PluginCount() const
{
  return this->dataPtr->plugins.size();
}

/////////////////////////////////////////////////
const Plugin *Gui::PluginByIndex(const uint64_t _index) const
{
  if (_index < this->dataPtr->plugins.size())
    return &this->dataPtr->plugins[_index];
  return nullptr;
}

/////////////////////////////////////////////////
void Gui::ClearPlugins()
{
  this->dataPtr->plugins.clear();
}

/////////////////////////////////////////////////
void Gui::AddPlugin(const Plugin &_plugin)
{
  this->dataPtr->plugins.push_back(_plugin);
}

/////////////////////////////////////////////////
const sdf::Plugins &Gui::Plugins() const
{
  return this->dataPtr->plugins;
}

/////////////////////////////////////////////////
sdf::Plugins &Gui::Plugins()
{
  return this->dataPtr->plugins;
}


