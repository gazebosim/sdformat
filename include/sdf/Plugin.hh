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

    /// \brief Get the schema file name accessor
    public: static inline std::string_view SchemaFile();

    /// \brief Default destructor
    public: ~Plugin();

    /// \brief Copy constructor.
    /// \param[in] _plugin Plugin to copy.
    public: Plugin(const Plugin &_plugin);

    /// \brief Move constructor.
    /// \param[in] _plugin Plugin to copy.
    public: Plugin(Plugin &&_plugin) noexcept;

    /// \brief A constructor that initializes the plugin's filename, name, and
    /// optionally the content.
    /// \param[in] _filename Filename of the shared library associated with
    /// this plugin.
    /// \param[in] _name The name of the plugin.
    /// \param[in] _xmlContent Optional XML content that will be stored in
    /// this plugin.
    public: Plugin(const std::string &_filename, const std::string &_name,
                   const std::string &_xmlContent = "");

    /// \brief A constructor that initializes the plugin's filename, name, and
    /// optionally the content.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _filename Filename of the shared library associated with
    /// this plugin.
    /// \param[in] _name The name of the plugin.
    /// \param[in] _xmlContent Optional XML content that will be stored in
    /// this plugin.
    public: Plugin(sdf::Errors &_errors, const std::string &_filename,
                   const std::string &_name,
                   const std::string &_xmlContent = "");

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
    public: const std::string &Name() const;

    /// \brief Set the name of the plugin.
    /// The name of the plugin should be unique within the scope of its
    /// parent.
    /// \param[in] _name Name of the plugin.
    public: void SetName(const std::string &_name);

    /// \brief Get the filename of the shared library.
    /// \return Filename of the shared library associated with the plugin.
    public: const std::string &Filename() const;

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

    /// \brief Insert an element into the plugin content. This does not
    /// modify the values in the sdf::ElementPtr returned by the `Element()`
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _elem Element to insert.
    public: void InsertContent(sdf::Errors &_errors,
                               const sdf::ElementPtr _elem);

    /// \brief Insert XML content into this plugin. This function does not
    /// modify the values in the sdf::ElementPtr returned by the `Element()`
    /// function. The provided content must be valid XML.
    /// \param[in] _content A string that contains valid XML. The XML is
    /// inserted into this plugin if it is valid.
    /// \return False if the provided content was invalid, in which case the
    /// content of this plugin is not modified. True otherwise
    public: bool InsertContent(const std::string _content);

    /// \brief Insert XML content into this plugin. This function does not
    /// modify the values in the sdf::ElementPtr returned by the `Element()`
    /// function. The provided content must be valid XML.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _content A string that contains valid XML. The XML is
    /// inserted into this plugin if it is valid.
    /// \return False if the provided content was invalid, in which case the
    /// content of this plugin is not modified. True otherwise
    public: bool InsertContent(sdf::Errors &_errors,
                               const std::string _content);

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

    /// \brief Create and return an SDF element filled with data from this
    /// plugin.
    /// Note that parameter passing functionality is not captured with this
    /// function.
    /// \param[out] _errors Vector of errors.
    /// \return SDF element pointer with updated plugin values.
    public: sdf::ElementPtr ToElement(sdf::Errors &_errors) const;

    /// \brief Copy assignment operator
    /// \param[in] _plugin Plugin to copy
    /// \return A reference to this plugin
    public: Plugin &operator=(const Plugin &_plugin);

    /// \brief Move assignment operator
    /// \param[in] _plugin Plugin to move
    /// \return A reference to this plugin
    public: Plugin &operator=(Plugin &&_plugin) noexcept;

    /// \brief Plugin equality operator.
    /// \param[in] _plugin Plugin to compare against.
    /// \return True if this plugin matches the provided plugin. The name,
    /// filename, and contents must all match to return true.
    public: bool operator==(const Plugin &_plugin) const;

    /// \brief Plugin inequality operator.
    /// \param[in] _plugin Plugin to compare against.
    /// \return True if this plugin does not match the provided plugin.
    /// The name, filename, or contents must be different to return false.
    public: bool operator!=(const Plugin &_plugin) const;

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

    /// \brief Initializer function to help Plugin constructors.
    /// \param[out] _errors Vector of errors.
    /// \param[in] _filename Filename of the shared library associated with
    /// this plugin.
    /// \param[in] _name The name of the plugin.
    /// \param[in] _xmlContent Optional XML content that will be stored in
    /// this plugin.
    private: void Init(sdf::Errors &_errors, const std::string &_filename,
                    const std::string &_name, const std::string &_xmlContent);

    /// \brief Private data pointer.
    private: std::unique_ptr<sdf::PluginPrivate> dataPtr;
  };

  /// \brief A vector of Plugin.
  using Plugins = std::vector<Plugin>;
}
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
