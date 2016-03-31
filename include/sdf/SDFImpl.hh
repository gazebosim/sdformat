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
#ifndef _SDFIMPL_HH_
#define _SDFIMPL_HH_

#include <string>
#include <memory>
#include <functional>

#include "sdf/Param.hh"
#include "sdf/Element.hh"
#include "sdf/system_util.hh"

/// \todo Remove this diagnositic push/pop in version 5
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#include "sdf/Types.hh"
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  class SDFORMAT_VISIBLE SDF;

  /// \def SDFPtr
  /// \brief Shared pointer to SDF
  typedef std::shared_ptr<SDF> SDFPtr;

  /// \addtogroup sdf
  /// \{

  /// \brief Find the absolute path of a file.
  /// \param[in] _filename Name of the file to find.
  /// \param[in] _searchLocalPath True to search for the file in the current
  /// working directory.
  /// \param[in] _useCallback True to find a file based on a registered
  /// callback if the file is not found via the normal mechanism.
  SDFORMAT_VISIBLE
  std::string findFile(const std::string &_filename,
                       bool _searchLocalPath = true,
                       bool _useCallback = false);

  /// \brief Associate paths to a URI.
  /// Example paramters: "model://", "/usr/share/models:~/.gazebo/models"
  /// \param[in] _uri URI that will be mapped to _path
  /// \param[in] _path Colon separated set of paths.
  SDFORMAT_VISIBLE
  void addURIPath(const std::string &_uri, const std::string &_path);

  /// \brief Set the callback to use when SDF can't find a file.
  /// The callback should return a complete path to the requested file, or
  /// and empty string if the file was not found in the callback.
  /// \param[in] _cb The callback function.
  SDFORMAT_VISIBLE
  void setFindCallback(std::function<std::string (const std::string &)> _cb);

  /// \brief Base SDF class
  class SDFORMAT_VISIBLE SDF
  {
    public: SDF();
    /// \brief Destructor
    public: ~SDF();
    public: void PrintDescription();
    public: void PrintValues();
    public: void PrintWiki();
    public: void PrintDoc();
    public: void Write(const std::string &_filename);
    public: std::string ToString() const;

    /// \brief Set SDF values from a string
    public: void SetFromString(const std::string &_sdfData);

    /// \brief Get a pointer to the root element
    /// \return Pointer to the root element
    public: ElementPtr Root() const;

    /// \brief Set the root pointer
    /// \param[in] _root Root element
    public: void Root(const ElementPtr _root);

    /// \brief Get the version
    /// \return The version as a string
    public: static std::string Version();

    /// \brief Set the version string
    /// \param[in] _version SDF version string.
    public: static void Version(const std::string &_version);

// \todo Remove this warning push/pop after sdformat 4.0
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

    /// \brief Deprecated.
    /// \sa ElementPtr Root()
    /// \sa void Root(const ElementPtr _root)
    public: ElementPtr root SDF_DEPRECATED(4.0);

    /// \brief Deprecated.
    /// \sa std::string Version()
    /// \sa Version(const std::string &_version)
    public: static std::string version SDF_DEPRECATED(4.0);

#ifdef _MSC_VER
#pragma warning(pop)
#endif
  };
  /// \}
}
#endif
