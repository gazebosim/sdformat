/*
 * Copyright 2022 Open Source Robotics Foundation
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

#ifndef USD_PARSER_USD2SDF_HH_
#define USD_PARSER_USD2SDF_HH_

#include <string>

#include <ignition/math/Vector3.hh>
#include <tinyxml2.h>

#include "sdf/sdf_config.h"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
  namespace usd
  {
    /// \brief USD to SDF converter
    /// This class helps generate the SDF file
    class USD2SDF
    {
      /// \brief constructor
      public: USD2SDF() = default;

      /// \brief convert USD file to sdf xml document
      /// \param[in] _fileMame string containing USD filename.
      /// \param[out] _sdfXmlDoc Document to populate with the sdf model.
      /// \return UsdErrors, which is a list of UsdError objects. An empty list
      /// means no errors occurred when populating _sdfXmlOut with the contents
      /// of _fileName
      public: UsdErrors Read(
        const std::string &_fileName,
        tinyxml2::XMLDocument *_sdfXmlOut);

      /// \brief get value from <key value="..."/> pair and return it as string
      /// \param[in] _elem pointer to xml element
      /// return a string with the key
      private: std::string GetKeyValueAsString(
                   const tinyxml2::XMLElement *_elem) const;

      /// \brief append key value pair to the end of the xml element
      /// \param[in] _elem pointer to xml element
      /// \param[in] _key string containing key to add to xml element
      /// \param[in] _value string containing value for the key added
      private: void AddKeyValue(
        tinyxml2::XMLElement *_elem,
        const std::string &_key,
        const std::string &_value);

      /// \brief convert Vector3 to string
      /// \param[in] _vector a ignition::math::Vector3d
      /// \return string representation of Vector3
      private: std::string Vector32Str(
                   const ignition::math::Vector3d &_vector) const;
    };
  }
}
}

#endif
