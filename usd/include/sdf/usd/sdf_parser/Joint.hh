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

#ifndef SDF_USD_SDF_PARSER_JOINT_HH_
#define SDF_USD_SDF_PARSER_JOINT_HH_

#include <unordered_map>
#include <string>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Joint.hh"
#include "sdf/Model.hh"
#include "sdf/sdf_config.h"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse a SDF joint into a USD stage.
    /// \param[in] _joint The SDF joint to parse.
    /// \param[in] _stage The stage that should contain the USD representation
    /// of _joint. This must be a valid, initialized stage.
    /// \param[in] _path The USD path of the parsed joint in _stage, which must
    /// be a valid USD path.
    /// \param[in] _parentModel The model that is the parent of _joint
    /// \param[in] _linkToUSDPath a map of a link's SDF name to the link's USD
    /// path. This is used to determine which USD prims should be assigned as
    /// the USD joint's relative links.
    /// \param[in] _worldPath The USD path of the world prim. This is needed if
    /// _joint's parent is the world.
    /// \return Errors, which is a vector of Error objects. Each Error includes
    /// an error code and message. An empty vector indicates no errors occurred
    /// when parsing _joint to its USD representation.
    sdf::Errors SDFORMAT_VISIBLE ParseSdfJoint(const sdf::Joint &_joint,
        pxr::UsdStageRefPtr &_stage, const std::string &_path,
        const sdf::Model &_parentModel,
        const std::unordered_map<std::string, pxr::SdfPath> &_linkToUSDPath,
        const pxr::SdfPath &_worldPath);
  }
  }
}

#endif
