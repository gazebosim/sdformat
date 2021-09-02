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
#ifndef SDFORMAT_URDF2SDF_HH_
#define SDFORMAT_URDF2SDF_HH_

#include <tinyxml.h>
#include <sdf/sdf_config.h>

#include <string>

#include "sdf/Console.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief URDF to SDF converter
  ///
  /// This is now deprecated for external usage and will be removed in the next
  /// major version of libsdformat. Instead, consider using `sdf::readFile` or
  /// `sdf::readString`, which automatically convert URDF to SDF.
  class SDFORMAT_VISIBLE SDF_DEPRECATED(9.2) URDF2SDF
  {
    /// \brief constructor
    public: URDF2SDF();

    /// \brief destructor
    public: ~URDF2SDF();

    /// \brief convert urdf xml document string to sdf xml document
    /// \param[in] _xmlDoc a tinyxml document containing the urdf model
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelDoc(TiXmlDocument* _xmlDoc);

    /// \brief convert urdf file to sdf xml document
    /// \param[in] _urdfStr a string containing filename of the urdf model
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelFile(const std::string &_filename);

    /// \brief convert urdf string to sdf xml document, with option to enforce
    /// limits.
    /// \param[in] _urdfStr a string containing model urdf
    /// \param[in] _enforceLimits option to enforce joint limits
    /// \return a tinyxml document containing sdf of the model
    public: TiXmlDocument InitModelString(const std::string &_urdfStr,
                                          bool _enforceLimits = true);

    /// \brief Return true if the filename is a URDF model.
    /// \param[in] _filename File to check.
    /// \return True if _filename is a URDF model.
    public: static bool IsURDF(const std::string &_filename);

    /// things that do not belong in urdf but should be mapped into sdf
    /// @todo: do this using sdf definitions, not hard coded stuff
    private: void ParseSDFExtension(TiXmlDocument &_urdfXml);

    /// list extensions for debugging
    // cppcheck-suppress unusedPrivateFunction
    // cppcheck-suppress unmatchedSuppression
    private: void ListSDFExtensions();

    /// list extensions for debugging
    // cppcheck-suppress unusedPrivateFunction
    // cppcheck-suppress unmatchedSuppression
    private: void ListSDFExtensions(const std::string &_reference);
  };
  }
}
#endif
