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
#ifndef SDF_DOM_ROOT_HH_
#define SDF_DOM_ROOT_HH_

#include <string>

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
    /// \return True if the file was parsed without any errors.
    public: bool Load(const std::string &_filename);

    /// \brief Parse the given SDF pointer, and generate objects based on types
    /// specified in the SDF pointer.
    /// An SDF pointer can be acquired by the sdf::readFile function. For
    /// example:
    ///
    /// ```
    /// sdf::SDFPtr sdfParsed = sdf::readFile(_filename);
    /// sdf::Root root;
    /// root.Load(sdfParsed);
    /// ```
    ///
    /// \param[in] _sdf Pointer to an SDF object
    /// \return True if the SDF pointer was parsed without any errors.
    /// \sa sdf::readFile(const std::string &_filename)
    public: bool Load(const sdf::SDFPtr _sdf);

    /// \brief Parse the given SDF Element pointer, and generate objects based
    /// on types specified in the SDF Element pointer.
    /// One way to get an SDF Element pointer is though the SDF::Root()
    /// function. For example:
    ///
    /// ```
    /// sdf::SDFPtr sdfParsed = sdf::readFile(_filename);
    /// sdf::Root root;
    /// root.Load(sdfParsed->Root());
    /// ```
    ///
    /// \return True if the SDF Element pointer was parsed without any errors.
    public: bool Load(ElementPtr _sdf);

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
    public: void Print(const std::string &_prefix = "") const;

    /// \brief Private data pointer
    private: RootPrivate *dataPtr;
  };
}
#endif
