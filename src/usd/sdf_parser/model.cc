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

#ifndef SDF_PARSER_MODEL_HH_
#define SDF_PARSER_MODEL_HH_

#include "model.hh"

#include <iostream>
#include <string>

#include <pxr/usd/usd/stage.h>

#include "sdf/Model.hh"
#include "usd/sdf_parser/link.hh"

using namespace usd;

bool ParseSdfModel(const sdf::Model &_model, pxr::UsdStageRefPtr &_stage,
    const std::string &_path)
{
  // TODO(adlarkin) finish parsing model. It will look something like this
  // (this does not cover parsing all elements of a model):
  //  * ParseSdfLink
  //  * ParseSdfJoint

  // parse all of the model's links and convert them to USD
  for (uint64_t i = 0; i < _model.LinkCount(); ++i)
  {
    const auto link = *(_model.LinkByIndex(i));
    const auto linkPath = std::string(_path + "/" + link.Name());
    if (!ParseSdfLink(link, _stage, linkPath))
    {
      std::cerr << "Error parsing link [" << link.Name() << "]\n.";
      return false;
    }
  }

  return true;
}

#endif
