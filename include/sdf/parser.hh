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
#ifndef _SDF_PARSER_HH_
#define _SDF_PARSER_HH_

#include <string>

#include "sdf/SDFImpl.hh"
#include "sdf/system_util.hh"

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  /// \brief Init based on the installed sdf_format.xml file
  SDFORMAT_VISIBLE
  bool init(SDFPtr _sdf);

  // \brief Initialize the SDF interface using a file
  SDFORMAT_VISIBLE
  bool initFile(const std::string &_filename, SDFPtr _sdf);

  // \brief Initialize and SDFElement interface using a file
  SDFORMAT_VISIBLE
  bool initFile(const std::string &_filename, ElementPtr _sdf);

  // \brief Initialize the SDF interface using a string
  SDFORMAT_VISIBLE
  bool initString(const std::string &_xmlString, SDFPtr _sdf);

  /// \brief Populate the SDF values from a file
  SDFORMAT_VISIBLE
  bool readFile(const std::string &_filename, SDFPtr _sdf);

  /// \brief Populate the SDF values from a string
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, SDFPtr _sdf);

  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, ElementPtr _sdf);

  SDFORMAT_VISIBLE
  void addNestedModel(ElementPtr _sdf, ElementPtr _includeSDF);

  /// \brief Convert an SDF file to a specific SDF version.
  /// \param[in] _filename Name of the SDF file to convert.
  /// \param[in] _version Version to convert _filename to.
  /// \param[out] _sdf Pointer to the converted SDF document.
  /// \return True on success.
  SDFORMAT_VISIBLE
  bool convertFile(const std::string &_filename, const std::string &_version,
                   SDFPtr _sdf);

  /// \brief Convert an SDF string to a specific SDF version.
  /// \param[in] _sdfString The SDF string to convert.
  /// \param[in] _version Version to convert _filename to.
  /// \param[out] _sdf Pointer to the converted SDF document.
  /// \return True on success.
  SDFORMAT_VISIBLE
  bool convertString(const std::string &_sdfString,
                     const std::string &_version, SDFPtr _sdf);
}
#endif
