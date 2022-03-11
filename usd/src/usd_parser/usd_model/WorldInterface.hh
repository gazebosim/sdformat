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

#ifndef USD_PARSER_USD_MODEL_WORLD_INTERFACE_HH_
#define USD_PARSER_USD_MODEL_WORLD_INTERFACE_HH_

#include <ostream>
#include <string>
#include <vector>

#include <ignition/math/Vector3.hh>

#include "sdf/sdf_config.h"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief This class stores data about the world
    class WorldInterface {
      public:
        /// \brief World name
        std::string worldName;

        /// \brief Magnitude of the gravity
        float magnitude {9.8f};

        /// \brief Gravity (X, Y, Z)
        ignition::math::Vector3d gravity {0.0, 0.0, -1.0};

        friend std::ostream& operator<<(
          std::ostream& os, const WorldInterface& _world)
        {
          os << "World name: " << _world.worldName
             << ", Gravity: " << _world.gravity * _world.magnitude;
          return os;
        }
    };
  }
  }
}

#endif
