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

#include "sdf/sdf_config.h"
#include "sdf/usd/UsdError.hh"

#include "sdf/Root.hh"

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
        sdf::Root &_root);
    };
  }
}
}

#endif
