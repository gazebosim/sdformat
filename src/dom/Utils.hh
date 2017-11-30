/*
 * Copyright 2017 Open Source Robotics Foundation
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

#ifndef SDFORMAT_DOM_UTILS_HH
#define SDFORMAT_DOM_UTILS_HH

#include <string>
#include <map>
#include <ignition/math/Pose3.hh>
#include "sdf/dom/Light.hh"
#include "sdf/dom/Model.hh"
#include "sdf/Element.hh"

namespace sdf
{
  bool loadName(sdf::ElementPtr _sdf, std::string &_name);

  bool loadPose(sdf::ElementPtr _sdf, ignition::math::Pose3d &_pose,
                std::string &_frame);
  bool loadLights(sdf::ElementPtr _sdf, std::map<std::string, Light> &_lights);

  bool loadModels(sdf::ElementPtr _sdf, std::map<std::string, Model> &_models);
}
#endif
