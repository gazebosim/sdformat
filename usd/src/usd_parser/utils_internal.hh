/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef SDF_USD_USD_PARSER_UTILS_INTERNAL_HH_
#define SDF_USD_USD_PARSER_UTILS_INTERNAL_HH_

#include <string>

#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// Remove substring appearance in the input string
    /// \param[in] _str Original string where the substring will be removed
    /// \param[in] _substr Substring to use to find and remove in the original
    /// \return A new string without the substring
    std::string RemoveSubStr(
      const std::string &_str,
      const std::string &_substr);
}
}
}
#endif
