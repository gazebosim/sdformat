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
#ifndef SDF_GUI_HH_
#define SDF_GUI_HH_

#include <ignition/utils/ImplPtr.hh>
#include "sdf/Element.hh"
#include "sdf/Plugin.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  class SDFORMAT_VISIBLE Gui
  {
    /// \brief Default constructor
    public: Gui();

    /// \brief Load the gui based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get whether the Gui should be fullscreen.
    /// \return True if the Gui should be fullscreen.
    public: bool Fullscreen() const;

    /// \brief Set whether the Gui should be full screen.
    /// \param[in] _fullscreen True indicates that the Gui should be
    /// fullscreen.
    public: void SetFullscreen(const bool _fullscreen);

    /// \brief Equality operator that returns true if this Gui
    /// instance equals the given Gui instance.
    /// \param[in] _gui Gui instance to compare.
    /// \return True if this instance equals the given Gui.
    public: bool operator==(const Gui &_gui) const;

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// gui.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated gui values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Get the number of plugins.
    /// \return Number of plugins contained in this Gui object.
    public: uint64_t PluginCount() const;

    /// \brief Get a plugin based on an index.
    /// \param[in] _index Index of the plugin. The index should be in the
    /// range [0..PluginCount()).
    /// \return Pointer to the plugin. Nullptr if the index does not exist.
    /// \sa uint64_t PluginCount() const
    public: const Plugin *PluginByIndex(const uint64_t _index) const;

    /// \brief Remove all plugins
    public: void ClearPlugins();

    /// \brief Add a plugin to this object.
    /// \param[in] _plugin Plugin to add.
    public: void AddPlugin(const Plugin &_plugin);

    /// \brief Get the plugins attached to this object.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: const sdf::Plugins &Plugins() const;

    /// \brief Get a mutable vector of plugins attached to this object.
    /// \return A vector of Plugin, which will be empty if there are no
    /// plugins.
    public: sdf::Plugins &Plugins();

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
  }
}
#endif
