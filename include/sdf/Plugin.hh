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
#ifndef SDF_PLUGIN_HH_
#define SDF_PLUGIN_HH_

#include <memory>
#include <string>
#include <vector>

#include <sdf/Element.hh>
#include <sdf/Error.hh>
#include <sdf/parser.hh>
#include <sdf/Types.hh>
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  class PluginPrivate;

  class SDFORMAT_VISIBLE Plugin
  {
    /// \brief Default constructor
    public: Plugin();

    /// \brief Default destructor
    public: ~Plugin();

    /// \brief Copy constructor.
    /// \param[in] _plugin Plugin to copy.
    public: Plugin(const Plugin &_plugin);

    /// \brief Move constructor.
    /// \param[in] _plugin Plugin to copy.
    public: Plugin(Plugin &&_plugin) noexcept;

    /// \brief Load the plugin based on a element pointer. This is *not* the
    /// usual entry point. Typical usage of the SDF DOM is through the Root
    /// object.
    /// \param[in] _sdf The SDF Element pointer
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no error.
    public: Errors Load(ElementPtr _sdf);

    /// \brief Get the name of the plugin.
    /// The name of the plugin should be unique within the scope of its
    /// parent.
    /// \return Name of the plugin.
    public: std::string Name() const;

    /// \brief Set the name of the plugin.
    /// The name of the plugin should be unique within the scope of its
    /// parent.
    /// \param[in] _name Name of the plugin.
    public: void SetName(const std::string &_name);

    /// \brief Get the filename of the shared library.
    /// \return Filename of the shared library associated with the plugin.
    public: std::string Filename() const;

    /// \brief Remove the contents of the plugin, this is everything that
    /// is a child element of the `<plugin>`.
    public: void ClearContents();

    /// \brief Get the plugin contents. This is all the SDF elements that
    /// are children of the `<plugin>`.
    /// \return The child elements of this plugin.
    public: const std::vector<sdf::ElementPtr> &Contents() const;

    /// \brief Insert an element into the plugin content. This does not
    /// modify the values in the sdf::ElementPtr returned by the `Element()`
    /// function.
    /// \param[in] _elem Element to insert.
    public: void InsertContent(const sdf::ElementPtr _elem);

    /// \brief Set the filename of the shared library.
    /// \param[in] _filename Filename of the shared library associated with
    /// this plugin.
    public: void SetFilename(const std::string &_filename);

    /// \brief Get a pointer to the SDF element that was used during
    /// load.
    /// \return SDF element pointer. The value will be nullptr if Load has
    /// not been called.
    public: sdf::ElementPtr Element() const;

    /// \brief Create and return an SDF element filled with data from this
    /// plugin.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \return SDF element pointer with updated plugin values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Copy assignment operator
    /// \param[in] _plugin Plugin to copy
    /// \return A reference to this plugin
    public: Plugin &operator=(const Plugin &_plugin);

    /// \brief Move assignment operator
    /// \param[in] _plugin Plugin to move
    /// \return A reference to this plugin
    public: Plugin &operator=(Plugin &&_plugin) noexcept;

    /// \brief Output stream operator for a Plugin.
    /// \param[in] _out The output stream
    /// \param[in] _plugin Plugin to output
    public: friend std::ostream &operator<<(std::ostream& _out,
                                            const sdf::Plugin &_plugin)
    {
      return _out << _plugin.ToElement()->ToString("");
    }

    /// \brief Input stream operator for a Plugin.
    /// \param[in] _out The output stream
    /// \param[in] _plugin Plugin to output
    public: friend std::istream &operator>>(std::istream &_in,
                                            sdf::Plugin &_plugin)
    {
      std::ostringstream stream;
      stream << "<sdf version='" << SDF_VERSION << "'>";
      stream << std::string(std::istreambuf_iterator<char>(_in), {});
      stream << "</sdf>";

      sdf::SDFPtr sdfParsed(new sdf::SDF());
      sdf::init(sdfParsed);
      bool result = sdf::readString(stream.str(), sdfParsed);
      if (!result)
        return _in;

      _plugin.ClearContents();
      _plugin.Load(sdfParsed->Root()->GetFirstElement());

      return _in;
    }

    /// \brief Private data pointer.
    std::unique_ptr<sdf::PluginPrivate> dataPtr;
  };

  /// \brief A vector of Plugin.
  using Plugins = std::vector<Plugin>;
}
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
