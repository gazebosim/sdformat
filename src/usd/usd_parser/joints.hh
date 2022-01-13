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

#ifndef USD_PARSER_JOINTS_HH
#define USD_PARSER_JOINTS_HH

#include <pxr/usd/usd/primRange.h>

#include "sdf/system_util.hh"
#include "sdf/Joint.hh"
#include "usd_model/link.hh"
#include "usd/USDData.hh"

namespace usd
{
  std::shared_ptr<sdf::Joint> SDFORMAT_VISIBLE ParseJoints(
    const pxr::UsdPrim &_prim, const std::string &_path,
    USDData &_usdData);

  std::pair<std::shared_ptr<sdf::Joint>, LinkSharedPtr>
    SDFORMAT_VISIBLE ParseVehicleJoints(
      const pxr::UsdPrim &_prim,
      const std::string &_path,
      USDData &_usdData);
}

#endif
