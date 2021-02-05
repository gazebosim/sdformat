/*
 * Copyright 2020 Open Source Robotics Foundation
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

#ifndef SDF_PARSER_CONFIG_HH_
#define SDF_PARSER_CONFIG_HH_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <ignition/utils/ImplPtr.hh>

#include "sdf/Error.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"


namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE
{
// Forward declare private data class.
class ParserConfigPrivate;

/// This class contains configuration options for the libsdformat parser.
///
/// The configuration options include:
///   * A callback function used to locate a file from a given URI
///   * A map from URI schemes to search directories
///
/// For backward compatibility, the functions sdf::setFindCallback() and
/// sdf::addURIPath() update a singleton ParserConfig object, which can be
/// retrieved by ParserConfig::GlobalConfig().
///
/// The functions sdf::readFile(), sdf::readString(), and \ref Root::Load
/// "sdf::Root::Load()" have overloads that take a ParserConfig object. If
/// the ParserConfig object is omitted, these functions will use the singleton
/// ParserConfig object.
///
/// Example:
/// To set an additional URI scheme search directory without affecting the
/// global config,
///
/// \code{.cpp}
///   // Copy the global config
///   sdf::ParserConfig config = ParserConfig::GlobalConfig();
///
///   // Add the new scheme to the config
///   config.AddURIPath("newScheme://", "path/to/directory");
///
///   // Use the new config when loading a new SDFormat file
///   sdf::Root root;
///   root.Load("path/to/file.sdf", config);
/// \endcode
class SDFORMAT_VISIBLE ParserConfig
{
  /// type alias for the map from URI scheme to search directories
  public: using SchemeToPathMap =
          std::map<std::string, std::vector<std::string> >;

  /// \brief Default constructor
  public: ParserConfig();

  /// Mutable access to a singleton ParserConfig that serves as the global
  /// ParserConfig object for all parsing operations that do not specify their
  /// own ParserConfig.
  /// \return A mutable reference to the singleton ParserConfig object
  public: static ParserConfig &GlobalConfig();

  /// \brief Get the find file callback function
  /// \return Immutable reference to the find file callback function
  public: const std::function<std::string(const std::string &)> &
          FindFileCallback() const;

  /// \brief Set the callback to use when libsdformat can't find a file.
  /// The callback should return a complete path to the requested file, or
  /// an empty string if the file was not found in the callback. Generally, the
  /// input argument is a URI or a file path (absolute or relative) obtained
  /// from a `//include/uri` element. For example, if the value
  /// `custom://model_name` is given in a `//include/uri`, sdf::findFile() may
  /// invoke the callback with the argument `custom://model_name` if it is
  /// unable to find a file using the steps listed in sdf::findfile().
  ///
  /// Note, however, the input is not limited to URIs and file paths, and it is
  /// left up to the callback to interpret the contents of the input string.
  /// \param[in] _cb The callback function.
  /// \sa sdf::findFile() for the order of search operations
  public: void SetFindCallback(
              std::function<std::string(const std::string &)> _cb);

  /// \brief Get the URI scheme to search directories map
  /// \return Immutable reference to the URI scheme to search directories map
  public: const SchemeToPathMap &URIPathMap() const;

  /// \brief Associate paths to a URI.
  /// Example paramters: "model://", "/usr/share/models:~/.gazebo/models"
  /// \param[in] _uri URI that will be mapped to _path
  /// \param[in] _path Colon separated set of paths.
  /// \sa sdf::findFile() for the order of search operations
  public: void AddURIPath(const std::string &_uri, const std::string &_path);

  /// \brief Private data pointer.
  IGN_UTILS_IMPL_PTR(dataPtr)
};
}
}

#endif
