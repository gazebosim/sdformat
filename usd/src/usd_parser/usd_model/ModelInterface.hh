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

#ifndef SDF_USD_USD_PARSER_USD_MODEL_MODEL_INTERFACE_HH_
#define SDF_USD_USD_PARSER_USD_MODEL_MODEL_INTERFACE_HH_

#include <string>

#include <ignition/math/Pose3.hh>

#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    class ModelInterface
    {
      public:
        const std::string& Name() const
        {
          return name;
        }

        void Clear()
        {
          name.clear();
        }

        /// \brief Model pose
        ignition::math::Pose3d pose;

        /// \brief The name of the robot model
        std::string name;
    };
  }
  }
}

#endif
