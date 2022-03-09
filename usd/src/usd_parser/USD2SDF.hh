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

#ifndef SDF_USD_USD_PARSER_USD2SDF_HH
#define SDF_USD_USD_PARSER_USD2SDF_HH

#include <string>

#include <ignition/math/Vector3.hh>

#include <tinyxml2.h>

#include <sdf/sdf_config.h>

#include "sdf/Light.hh"

#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
    /// \brief USD to SDF converter
    /// This class helps to generate the SDF file
    namespace usd
    {
    class USD2SDF
    {
      /// \brief constructor
      public: USD2SDF();

      /// \brief destructor
      public: ~USD2SDF();

      /// \brief convert USD file to sdf xml document
      /// \param[in] _filename string containing filename of the urdf model.
      /// \param[inout] _sdfXmlDoc Document to populate with the sdf model.
      public: UsdErrors Read(
        const std::string &_filename,
        tinyxml2::XMLDocument* _sdfXmlOut);

      /// \brief Add light to the xml
      /// \param[in] _lights Map with the name of the light and sdf Light class
      /// with the light properties
      /// \param[inout] _attach Xml element to attach the light
      void AddLights(
        const std::map<std::string, std::shared_ptr<sdf::Light>> &_lights,
        tinyxml2::XMLElement *_attach);

      /// \brief get value from <key value="..."/> pair and return it as string
      /// \param[in] _elem pointer to xml element
      /// return a string with the key
      private: std::string GetKeyValueAsString(tinyxml2::XMLElement* _elem);

      /// \brief append key value pair to the end of the xml element
      /// \param[in] _elem pointer to xml element
      /// \param[in] _key string containing key to add to xml element
      /// \param[in] _value string containing value for the key added
      private: void AddKeyValue(
        tinyxml2::XMLElement *_elem,
        const std::string &_key,
        const std::string &_value);

      /// \brief Convert double array to string
      /// \param[in] _count Number of elements
      /// \param[in] _values Double array
      /// \return Vector representation as an array
      std::string Values2str(unsigned int _count, const double *_values);

      /// \brief convert Vector3 to string
      /// \param[in] _vector a ignition::math::Vector3d
      /// \return a string
      private: std::string Vector32Str(const ignition::math::Vector3d _vector);
    };
  }
  }
}

#endif
