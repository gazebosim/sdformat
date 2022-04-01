/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SDF_USD_USD_PARSER_USD_LINKS_HH
#define SDF_USD_USD_PARSER_USD_LINKS_HH

#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/system_util.hh"

#include "sdf/usd/usd_parser/USDData.hh"

#include "sdf/Link.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    sdf::Link * ParseUSDLinks(
      const pxr::UsdPrim &_prim,
      const std::string &_nameLink,
      sdf::Link *_link,
      USDData &_usdData,
      int &_skipPrim);
  }
  }
}

#endif
