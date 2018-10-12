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
#ifndef SDF_PARSER_HH_
#define SDF_PARSER_HH_

#include <string>

#include "sdf/SDFImpl.hh"
#include "sdf/system_util.hh"

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
///
/// The parsing functions read XML elements contained in either a file or
/// string and translates the XML elements into SDF data structures. This
/// translation finds errors in the provided XML, fills in default values,
/// and performs any necessary version related conversions.
///
/// XML elements that are not part of the SDF specification are copied in
/// place. This preserves the given XML structure and data.
namespace sdf
{
  /// \brief Init based on the installed sdf_format.xml file
  SDFORMAT_VISIBLE
  bool init(SDFPtr _sdf);

  /// \brief Initialize the SDF interface using a file
  SDFORMAT_VISIBLE
  bool initFile(const std::string &_filename, SDFPtr _sdf);

  /// \brief Initialize an SDFElement interface using a file
  SDFORMAT_VISIBLE
  bool initFile(const std::string &_filename, ElementPtr _sdf);

  /// \brief Initialize the SDF interface using a string
  SDFORMAT_VISIBLE
  bool initString(const std::string &_xmlString, SDFPtr _sdf);

  /// \brief Populate the SDF values from a file
  /// \param[in] _filename Name of the SDF file
  /// \return Populated SDF pointer.
  SDFORMAT_VISIBLE
  sdf::SDFPtr readFile(const std::string &_filename);

  /// \brief Populate the SDF values from a file
  /// \param[in] _filename Name of the SDF file
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return Populated SDF pointer.
  SDFORMAT_VISIBLE
  sdf::SDFPtr readFile(const std::string &_filename, Errors &_errors);

  /// \brief Populate the SDF values from a file
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. All files are converted to the latest
  /// SDF version
  /// \param[in] _filename Name of the SDF file
  /// \param[in] _sdf Pointer to an SDF object.
  /// \param[out] _errors Parsing errors will be appended to this variable.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readFile(const std::string &_filename, SDFPtr _sdf, Errors &_errors);

  /// \brief Populate the SDF values from a file
  ///
  /// This populates the given sdf pointer from a file. If the file is a URDF
  /// file it is converted to SDF first. All files are converted to the latest
  /// SDF version
  /// \param[in] _filename Name of the SDF file
  /// \param[in] _sdf Pointer to an SDF object.
  /// \return True if successful.
  SDFORMAT_VISIBLE
  bool readFile(const std::string &_filename, SDFPtr _sdf);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All string are converted to the
  /// latest SDF version
  /// \param[out] _errors Parsing errors will be appended to this variable.
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, SDFPtr _sdf, Errors &_errors);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All string are converted to the
  /// latest SDF version
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, SDFPtr _sdf);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All strings are converted to the
  /// latest SDF version
  /// \param[out] _errors Parsing errors will be appended to this variable.
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, ElementPtr _sdf,
      Errors &_errors);

  /// \brief Populate the SDF values from a string
  ///
  /// This populates the sdf pointer from a string. If the string is a URDF
  /// string it is converted to SDF first. All strings are converted to the
  /// latest SDF version
  SDFORMAT_VISIBLE
  bool readString(const std::string &_xmlString, ElementPtr _sdf);

  /// \brief Get the file path to the model file
  /// \param[in] _modelDirPath directory system path of the model
  /// \return string with the full filesystem path to the best version (greater
  ///         SDF protocol supported by this sdformat version) of the .sdf
  ///         model files hosted by _modelDirPath.
  SDFORMAT_VISIBLE
  std::string getModelFilePath(const std::string &_modelDirPath);

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
