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


#ifndef USD_PARSER_USD_PARSER_HH
#define USD_PARSER_USD_PARSER_HH

#include <string>

#include "sdf/system_util.hh"
#include "usd_model/model.hh"
#include "usd_model/link.hh"
#include "usd_model/types.hh"
#include "usd_model/world.hh"

namespace usd {
  WorldInterfaceSharedPtr SDFORMAT_VISIBLE parseUSDFile(
      const std::string &filename);

  WorldInterfaceSharedPtr SDFORMAT_VISIBLE parseUSD(
      const std::string &xml_string);

  void SDFORMAT_VISIBLE exportUSD();

  bool SDFORMAT_VISIBLE isUSD(const std::string &filename);
}
#endif
