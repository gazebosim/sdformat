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
#include "sdf/system_util.hh"

/// \ingroup sdf_parser
/// \brief namespace for Simulation Description Format parser
namespace sdf
{
  /// \brief Get the best SDF version from models supported by this sdformat
  /// \param[in] _modelXML XML element from config file pointing to the
  ///            model XML tag
  /// \param[out] _modelFileName file name of the best model file
  /// \return string with the best SDF version supported
  std::string getBestSupportedModelVersion(TiXmlElement *_modelXML,
                                           std::string &_modelFileName);

  /// \brief Initialize the SDF interface using a TinyXML document
  bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf);

  /// \brief Initialize and SDF Element using a TinyXML document
  bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf);

  /// \brief For internal use only. Do not use this function.
  bool initXml(TiXmlElement *_xml, ElementPtr _sdf);

  /// \brief Populate the SDF values from a TinyXML document
  bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf, const std::string &_source,
      bool _convert, Errors &_errors);

  bool readDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf,
      const std::string &_source, bool _convert, Errors &_errors);

  /// \brief For internal use only. Do not use this function.
  /// \param[in] _xml Pointer to the XML document
  /// \param[in,out] _sdf SDF pointer to parse data into.
  /// \param[out] _errors Captures errors found during parsing.
  /// \return True on success, false on error.
  bool readXml(TiXmlElement *_xml, ElementPtr _sdf, Errors &_errors);

  void copyChildren(ElementPtr _sdf, TiXmlElement *_xml);
}
#endif
