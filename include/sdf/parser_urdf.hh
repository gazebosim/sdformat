/*
 * Copyright 2012-2015 Open Source Robotics Foundation
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
#ifndef _SDFORMAT_URDF2SDF_HH_
#define _SDFORMAT_URDF2SDF_HH_

#include <tinyxml.h>
#include <string>

#include "sdf/Console.hh"
#include "sdf/system_util.hh"

namespace sdf
{
  /// \brief URDF to SDF converter
  class SDFORMAT_VISIBLE URDF2SDF
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

    /// things that do not belong in urdf but should be mapped into sdf
    /// @todo: do this using sdf definitions, not hard coded stuff
    private: void ParseSDFExtension(TiXmlDocument &_urdfXml);

    /// list extensions for debugging
    private: void ListSDFExtensions();

    /// list extensions for debugging
    private: void ListSDFExtensions(const std::string &_reference);
  };
}
#endif
