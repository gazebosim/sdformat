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

#include <string>
#include <vector>

#include <ignition/utils/ImplPtr.hh>
#include <sdf/Element.hh>
#include <sdf/Error.hh>
#include <sdf/Types.hh>
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  class SDFORMAT_VISIBLE Plugin
  {
    /// \brief Default constructor
    public: Plugin();

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
    /// \return SDF element pointer with updated plugin values.
    public: sdf::ElementPtr ToElement() const;

    /// \brief Private data pointer.
    IGN_UTILS_IMPL_PTR(dataPtr)
  };
}
}
#endif
