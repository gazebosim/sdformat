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

#include <functional>
#include <memory>
#include <string>

#include "sdf/Param.hh"
#include "sdf/Element.hh"
#include "sdf/system_util.hh"
#include "sdf/Types.hh"

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
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
  /// \return File's full path.
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

  class SDFPrivate;

  /// \brief Base SDF class
  class SDFORMAT_VISIBLE SDF
  {
    public: SDF();
    /// \brief Destructor
    public: ~SDF();
    public: void PrintDescription();
    public: void PrintValues();
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

    /// \brief wraps the SDF element into a root element with the version info.
    /// \param[in] _sdf the sdf element. Will be cloned by this function.
    /// \return a wrapped clone of the SDF element
    public: static ElementPtr WrapInRoot(const ElementPtr &_sdf);

    /// \brief Get a string representation of an SDF specification file.
    /// This function uses a built-in version of a .sdf file located in
    /// the sdf directory. The parser.cc code uses this function, which avoids
    /// touching the filesystem.
    ///
    /// Most people should not use this function.
    ///
    /// \param[in] _filename Base name of the SDF specification file to
    /// load. For example "root.sdf" or "world.sdf".
    /// \param[in] _quiet True to suppress console messages.
    /// \return A string that contains the contents of the specified
    /// _filename. An empty string is returned if the _filename could not be
    /// found.
    public: static const std::string &EmbeddedSpec(
                const std::string &_filename, const bool _quiet);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<SDFPrivate> dataPtr;

    /// \brief The SDF version. Set to SDF_VERSION by default, or through
    /// the Version function at runtime.
    private: static std::string version;
  };
  /// \}
}

#ifdef _WIN32
#pragma warning(pop)
#endif

#endif
