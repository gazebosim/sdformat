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

#include <tinyxml2.h>
#include <sdf/config.hh>

#include <string>

#include "sdf/Console.hh"
#include "sdf/ParserConfig.hh"
#include "sdf/Types.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief URDF to SDF converter
  class URDF2SDF
  {
    /// \brief constructor
    public: URDF2SDF();

    /// \brief destructor
    public: ~URDF2SDF();

    /// \brief convert urdf xml document string to sdf xml document
    /// \param[in] _xmlDoc document containing the urdf model.
    /// \param[in] _config Custom parser configuration
    /// \param[inout] _sdfXmlDoc document to populate with the sdf model.
    public: void InitModelDoc(const tinyxml2::XMLDocument* _xmlDoc,
                              const ParserConfig& _config,
                              tinyxml2::XMLDocument *_sdfXmlDoc);

    /// \brief convert urdf file to sdf xml document
    /// \param[in] _urdfStr a string containing filename of the urdf model.
    /// \param[in] _config Custom parser configuration
    /// \param[inout] _sdfXmlDoc document to populate with the sdf model.
    public: void InitModelFile(const std::string &_filename,
                               const ParserConfig &_config,
                               tinyxml2::XMLDocument *_sdfXmlDoc);

    /// \brief convert urdf string to sdf xml document, with option to enforce
    /// limits.
    /// \param[in] _urdfStr a string containing model urdf
    /// \param[in] _config Custom parser configuration
    /// \param[inout] _sdfXmlDoc document to populate with the sdf model.
    /// \param[in] _enforceLimits option to enforce joint limits
    public: void InitModelString(const std::string &_urdfStr,
                                 const ParserConfig& _parserConfig,
                                 tinyxml2::XMLDocument *_sdfXmlDoc,
                                 bool _enforceLimits = true);

    /// \brief Return true if the filename is a URDF model.
    /// \param[in] _filename File to check.
    /// \return True if _filename is a URDF model.
    public: static bool IsURDF(const std::string &_filename);

    /// list extensions for debugging
    public: void ListSDFExtensions();

    /// list extensions for debugging
    public: void ListSDFExtensions(const std::string &_reference);

    /// things that do not belong in urdf but should be mapped into sdf
    /// @todo: do this using sdf definitions, not hard coded stuff
    private: void ParseSDFExtension(tinyxml2::XMLDocument &_urdfXml);
  };
  }
}
#endif
