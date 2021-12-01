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

#include "link.hh"

#include <string>

#include <pxr/usd/usd/stage.h>

#include "sdf/Link.hh"
#include "usd/sdf_parser/visual.hh"

using namespace usd;

bool ParseSdfLink(const sdf::Link &_link, pxr::UsdStageRefPtr &_stage,
    const std::string &_path)
{
  // TODO(adlarkin) finish parsing link. It will look something like this
  // (this does not cover all elements of a link that need to be parsed):
  //  * ParseSdfVisual
  //  * ParseSdfCollision
  //  * ParseSdfSensor

  // parse all of the link's visuals and convert them to USD
  for (uint64_t i = 0; i < _link.VisualCount(); ++i)
  {
    const auto visual = *(_link.VisualByIndex(i));
    const auto visualPath = std::string(_path + "/" + visual.Name());
    if (!ParseSdfVisual(visual, _stage, visualPath))
    {
      std::cerr << "Error parsing visual [" << visual.Name() << "]\n.";
      return false;
    }
  }
  return true;
}
