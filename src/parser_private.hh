/*
 * Copyright 2016 Open Source Robotics Foundation
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
#ifndef SDF_PARSER_PRIVATE_HH_
#define SDF_PARSER_PRIVATE_HH_

#include <tinyxml.h>

#include <string>

#include "sdf/SDFImpl.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //

  /// \brief Get the best SDF version from models supported by this sdformat
  /// \param[in] _modelXML XML element from config file pointing to the
  ///            model XML tag
  /// \param[out] _modelFileName file name of the best model file
  /// \return string with the best SDF version supported
  static std::string getBestSupportedModelVersion(TiXmlElement *_modelXML,
                                                  std::string &_modelFileName);

  /// \brief Initialize the SDF interface using a TinyXML document.
  ///
  /// This actually forwards to initXml after converting the inputs
  /// \param[in] _xmlDoc TinyXML document containing the SDFormat description
  /// file that corresponds with the input SDFPtr
  /// \param[in] _sdf SDF interface to be initialized
  static bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf);

  /// \brief Initialize the SDF Element using a TinyXML document
  ///
  /// This actually forwards to initXml after converting the inputs
  /// \param[in] _xmlDoc TinyXML document containing the SDFormat description
  /// file that corresponds with the input ElementPtr
  /// \param[in] _sdf SDF Element to be initialized
  static bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf);

  /// \brief Initialize the SDF Element by parsing the SDFormat description in
  /// the input TinyXML element. This is where SDFormat spec/description files
  /// are parsed
  /// \remark For internal use only. Do not use this function.
  /// \param[in] _xml TinyXML element containing the SDFormat description
  /// file that corresponds with the input ElementPtr
  /// \param[in] _sdf SDF ElementPtr to be initialized
  static bool initXml(TiXmlElement *_xml, ElementPtr _sdf);

  /// \brief Populate the SDF values from a TinyXML document
  static bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf,
                      const std::string &_source, bool _convert,
                      Errors &_errors);

  /// \brief Populate the SDF values from a TinyXML document
  static bool readDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf,
      const std::string &_source, bool _convert, Errors &_errors);

  /// \brief Populate an SDF Element from the XML input. The XML input here is
  /// an actual SDFormat file or string, not the description of the SDFormat
  /// spec.
  /// \remark For internal use only. Do not use this function.
  /// \param[in] _xml Pointer to the TinyXML element
  /// \param[in,out] _sdf SDF pointer to parse data into.
  /// \param[out] _errors Captures errors found during parsing.
  /// \return True on success, false on error.
  static bool readXml(TiXmlElement *_xml, ElementPtr _sdf, Errors &_errors);

  /// \brief Copy child XML elements into the _sdf element.
  /// \param[in] _sdf Parent Element.
  /// \param[in] _xml Pointer to element from which child elements should be
  /// copied.
  /// \param[in] _onlyUnknown True to copy only elements that are NOT part of
  /// the SDF spec. Set this to false to copy everything.
  static void copyChildren(ElementPtr _sdf, TiXmlElement *_xml,
                    const bool _onlyUnknown);
  }
}
#endif
