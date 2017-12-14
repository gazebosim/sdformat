/*
 * Copyright 2017 Open Source Robotics Foundation
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
#ifndef SDF_ROOT_HH_
#define SDF_ROOT_HH_

#include <string>

#include "sdf/Error.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/Element.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Forward declare private data class.
  class RootPrivate;

  /// \brief Root class that acts as an entry point to the SDF document
  /// model.
  ///
  /// # Usage
  ///
  /// In this example, a root object is loaded from a file specified in
  /// the first command line argument to a program.
  ///
  /// \snippet examples/dom.cc rootUsage
  ///
  class SDFORMAT_VISIBLE Root
  {
    /// \brief Default constructor
    public: Root();

    /// \brief Destructor
    public: ~Root();

    /// \brief Parse the given SDF file, and generate objects based on types
    /// specified in the SDF file.
    /// \param[in] _filename Name of the SDF file to parse.
    /// \return Error object that includes an error code and message. An
    /// ErrorCode of NONE indicates no error.
    /// \sa Error
    public: Error Load(const std::string &_filename);

    /// \brief Get the SDF version specified in the parsed file or SDF
    /// pointer.
    /// \return SDF version string.
    /// \sa void SetVersion(const std::string &_version)
    public: std::string Version() const;

    /// \brief Set the SDF version string.
    /// \param[in] _version The new SDF version.
    /// \sa std::string Version()
    public: void SetVersion(const std::string &_version);

    /// \brief Output debug information to standard out.
    /// \param[in] _prefix String to prefix all output.
    public: void DebugPrint(const std::string &_prefix = "") const;

    /// \brief Private data pointer
    private: RootPrivate *dataPtr;
  };
}
#endif
