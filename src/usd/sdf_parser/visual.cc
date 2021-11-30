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

#ifndef SDF_PARSER_VISUAL_HH_
#define SDF_PARSER_VISUAL_HH_

#include "visual.hh"

#include <iostream>
#include <string>

#include <pxr/usd/usd/stage.h>

#include "sdf/Visual.hh"
#include "usd/sdf_parser/geometry.hh"

using namespace usd;

bool ParseSdfVisual(const sdf::Visual &_visual, pxr::UsdStageRefPtr &_stage,
    const std::string &_path)
{
  const auto geometry = *(_visual.Geom());
  const auto geometryPath = std::string(_path + "/geometry");
  if (!ParseSdfGeometry(geometry, _stage, geometryPath))
  {
    std::cerr << "Error parsing geometry attached to visual ["
              << _visual.Name() << "]\n";
    return false;
  }

  return true;
}

#endif
